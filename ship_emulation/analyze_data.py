"""
Quick visual inspection of vessel_motion_clean.csv before robot testing.

Usage:
    python -m ship_emulation.analyze_data
    python -m ship_emulation.analyze_data --csv path/to/other.csv

    # Preview augmented data (m→mm, 5× angular scale, 10 deg roll overlay at 0.1 Hz):
    python -m ship_emulation.analyze_data --linear-scale 1000 --angular-scale 5 \
        --overlay-roll-amp 10 --overlay-roll-freq 0.1
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.interpolate import UnivariateSpline

POSITION_COLS = ["linear_x", "linear_y", "linear_z"]
ROTATION_COLS = ["rotation_x", "rotation_y", "rotation_z"]
TIME_COL = "time"

DEFAULT_CSV = Path(__file__).parent / "vessel_motion_clean.csv"


def load(path: Path):
    import csv
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return {col: np.array([r[col] for r in rows]) for col in rows[0]}


def apply_augmentation(data, linear_scale, angular_scale, overlay):
    data = dict(data)
    for col in POSITION_COLS:
        data[col] = data[col] * linear_scale
    for col in ROTATION_COLS:
        data[col] = data[col] * angular_scale
    if overlay is not None:
        t = data[TIME_COL]
        amps_freqs = [
            ("rotation_x", overlay["roll_amp"], overlay["roll_freq"]),
            ("rotation_y", overlay["pitch_amp"], overlay["pitch_freq"]),
            ("rotation_z", overlay["yaw_amp"], overlay["yaw_freq"]),
        ]
        for col, amp, freq in amps_freqs:
            if amp != 0.0:
                data[col] = data[col] + amp * np.sin(2 * math.pi * freq * t)
    return data


def print_stats(data, pos_unit, rot_unit):
    print(f"\n{'Column':<14} {'units':>6} {'min':>10} {'max':>10} {'mean':>10} {'std':>10}")
    print("-" * 66)
    for col in POSITION_COLS:
        v = data[col]
        print(f"{col:<14} {pos_unit:>6} {v.min():>10.4f} {v.max():>10.4f} {v.mean():>10.4f} {v.std():>10.4f}")
    for col in ROTATION_COLS:
        v = data[col]
        print(f"{col:<14} {rot_unit:>6} {v.min():>10.4f} {v.max():>10.4f} {v.mean():>10.4f} {v.std():>10.4f}")
    t = data[TIME_COL]
    dt = np.diff(t)
    print(f"\nTime range : {t[0]:.1f} s → {t[-1]:.1f} s  ({len(t)} samples)")
    print(f"Step size  : min={dt.min():.4f} s  max={dt.max():.4f} s  median={np.median(dt):.4f} s")


def apply_smoothing(data, s_linear, s_angular):
    t = data[TIME_COL]
    smoothed = dict(data)
    for col in POSITION_COLS:
        smoothed[col] = UnivariateSpline(t, data[col], s=s_linear)(t)
    for col in ROTATION_COLS:
        smoothed[col] = UnivariateSpline(t, data[col], s=s_angular)(t)
    return smoothed


def print_rate_stats(data, pos_unit, rot_unit):
    _, linear_rate, angular_rate = compute_rates(data)
    print(f"\n{'Rate':<20} {'units':>8} {'min':>10} {'max':>10} {'mean':>10} {'std':>10}")
    print("-" * 74)
    for label, values, unit in [
        ("linear_rate",  linear_rate,  f"{pos_unit}/s"),
        ("angular_rate", angular_rate, f"{rot_unit}/s"),
    ]:
        print(f"{label:<20} {unit:>8} {values.min():>10.4f} {values.max():>10.4f} {values.mean():>10.4f} {values.std():>10.4f}")


def compute_rates(data):
    t = data[TIME_COL]
    dt = np.diff(t)
    pos = np.stack([data[c] for c in POSITION_COLS], axis=1)
    rot = np.stack([data[c] for c in ROTATION_COLS], axis=1)
    linear_rate = np.linalg.norm(np.diff(pos, axis=0), axis=1) / dt
    angular_rate = np.max(np.abs(np.diff(rot, axis=0)), axis=1) / dt
    return t[1:], linear_rate, angular_rate


def plot(data, pos_unit, rot_unit, title_suffix, smoothed_data=None, save_path=None):
    t = data[TIME_COL]
    t_rate, linear_rate, angular_rate = compute_rates(data)

    title = "Vessel Motion Inspection"
    if title_suffix:
        title += f"  [{title_suffix}]"

    fig = plt.figure(figsize=(16, 14))
    fig.suptitle(title, fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(4, 2, figure=fig, hspace=0.45, wspace=0.35)

    ax_pos = fig.add_subplot(gs[0, :])
    colors = {}
    for col in POSITION_COLS:
        line, = ax_pos.plot(t, data[col], label=col, linewidth=0.6, alpha=0.4 if smoothed_data else 1.0)
        colors[col] = line.get_color()
    if smoothed_data:
        for col in POSITION_COLS:
            ax_pos.plot(t, smoothed_data[col], color=colors[col], linewidth=1.2, label=f"{col} (smoothed)")
    ax_pos.set_title("Position")
    ax_pos.set_ylabel(pos_unit)
    ax_pos.legend(loc="upper right", fontsize=8)
    ax_pos.grid(True, alpha=0.3)

    ax_rot = fig.add_subplot(gs[1, :])
    for col in ROTATION_COLS:
        line, = ax_rot.plot(t, data[col], label=col, linewidth=0.6, alpha=0.4 if smoothed_data else 1.0)
        colors[col] = line.get_color()
    if smoothed_data:
        for col in ROTATION_COLS:
            ax_rot.plot(t, smoothed_data[col], color=colors[col], linewidth=1.2, label=f"{col} (smoothed)")
    ax_rot.set_title("Rotation")
    ax_rot.set_ylabel(rot_unit)
    ax_rot.set_xlabel("time (s)")
    ax_rot.legend(loc="upper right", fontsize=8)
    ax_rot.grid(True, alpha=0.3)

    ax_xy = fig.add_subplot(gs[2, 0])
    ax_xy.plot(data["linear_x"], data["linear_y"], linewidth=0.4, color="steelblue")
    ax_xy.scatter(data["linear_x"][0], data["linear_y"][0], color="green", s=30, zorder=5, label="start")
    ax_xy.scatter(data["linear_x"][-1], data["linear_y"][-1], color="red", s=30, zorder=5, label="end")
    ax_xy.set_title("XY trajectory (top-down)")
    ax_xy.set_xlabel(f"linear_x ({pos_unit})")
    ax_xy.set_ylabel(f"linear_y ({pos_unit})")
    ax_xy.legend(fontsize=8)
    ax_xy.set_aspect("equal")
    ax_xy.grid(True, alpha=0.3)

    ax_xz = fig.add_subplot(gs[2, 1])
    ax_xz.plot(data["linear_x"], data["linear_z"], linewidth=0.4, color="darkorange")
    ax_xz.scatter(data["linear_x"][0], data["linear_z"][0], color="green", s=30, zorder=5, label="start")
    ax_xz.scatter(data["linear_x"][-1], data["linear_z"][-1], color="red", s=30, zorder=5, label="end")
    ax_xz.set_title("XZ trajectory (side view)")
    ax_xz.set_xlabel(f"linear_x ({pos_unit})")
    ax_xz.set_ylabel(f"linear_z ({pos_unit})")
    ax_xz.legend(fontsize=8)
    ax_xz.set_aspect("equal")
    ax_xz.grid(True, alpha=0.3)

    ax_lr = fig.add_subplot(gs[3, 0])
    ax_lr.plot(t_rate, linear_rate, linewidth=0.6, color="steelblue", alpha=0.4 if smoothed_data else 1.0, label="raw")
    ax_lr.set_title("Linear rate of change (‖Δpos‖/Δt)")
    ax_lr.set_xlabel("time (s)")
    ax_lr.set_ylabel(f"{pos_unit}/s")
    ax_lr.grid(True, alpha=0.3)
    p95 = np.percentile(linear_rate, 95)
    ax_lr.axhline(p95, color="red", linestyle="--", linewidth=0.8, label=f"p95={p95:.1f}")
    if smoothed_data:
        _, s_linear_rate, _ = compute_rates(smoothed_data)
        ax_lr.plot(t_rate, s_linear_rate, linewidth=1.2, color="steelblue", label="smoothed")
        s_p95 = np.percentile(s_linear_rate, 95)
        ax_lr.axhline(s_p95, color="navy", linestyle="--", linewidth=0.8, label=f"smoothed p95={s_p95:.1f}")
    ax_lr.legend(fontsize=8)

    ax_ar = fig.add_subplot(gs[3, 1])
    ax_ar.plot(t_rate, angular_rate, linewidth=0.6, color="darkorange", alpha=0.4 if smoothed_data else 1.0, label="raw")
    ax_ar.set_title("Angular rate of change (max|Δrot|/Δt)")
    ax_ar.set_xlabel("time (s)")
    ax_ar.set_ylabel(f"{rot_unit}/s")
    ax_ar.grid(True, alpha=0.3)
    p95 = np.percentile(angular_rate, 95)
    ax_ar.axhline(p95, color="red", linestyle="--", linewidth=0.8, label=f"p95={p95:.1f}")
    if smoothed_data:
        _, _, s_angular_rate = compute_rates(smoothed_data)
        ax_ar.plot(t_rate, s_angular_rate, linewidth=1.2, color="darkorange", label="smoothed")
        s_p95 = np.percentile(s_angular_rate, 95)
        ax_ar.axhline(s_p95, color="saddlebrown", linestyle="--", linewidth=0.8, label=f"smoothed p95={s_p95:.1f}")
    ax_ar.legend(fontsize=8)

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"\nPlot saved to {save_path}")
    else:
        plt.show()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Inspect vessel motion CSV data.")
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV, metavar="FILE")
    parser.add_argument("--save", type=Path, default=None, metavar="OUT.png",
                        help="Save plot to file instead of showing interactively")
    parser.add_argument("--linear-scale", type=float, default=1.0, metavar="FACTOR",
                        help="Scale applied to x/y/z (e.g. 1000 to preview robot output in mm)")
    parser.add_argument("--angular-scale", type=float, default=1.0, metavar="FACTOR",
                        help="Scale applied to roll/pitch/yaw")
    parser.add_argument("--overlay-roll-amp", type=float, default=0.0, metavar="DEG")
    parser.add_argument("--overlay-roll-freq", type=float, default=0.1, metavar="HZ")
    parser.add_argument("--overlay-pitch-amp", type=float, default=0.0, metavar="DEG")
    parser.add_argument("--overlay-pitch-freq", type=float, default=0.1, metavar="HZ")
    parser.add_argument("--overlay-yaw-amp", type=float, default=0.0, metavar="DEG")
    parser.add_argument("--overlay-yaw-freq", type=float, default=0.1, metavar="HZ")
    parser.add_argument("--smooth-linear", type=float, default=0.0, metavar="S",
                        help="Spline smoothing factor for linear channels in mm² (0 = disabled)")
    parser.add_argument("--smooth-angular", type=float, default=0.0, metavar="S",
                        help="Spline smoothing factor for angular channels in deg² (0 = disabled)")
    args = parser.parse_args(argv)

    if not args.csv.exists():
        sys.exit(f"File not found: {args.csv}")

    has_overlay = any([args.overlay_roll_amp, args.overlay_pitch_amp, args.overlay_yaw_amp])
    overlay = {
        "roll_amp": args.overlay_roll_amp, "roll_freq": args.overlay_roll_freq,
        "pitch_amp": args.overlay_pitch_amp, "pitch_freq": args.overlay_pitch_freq,
        "yaw_amp": args.overlay_yaw_amp, "yaw_freq": args.overlay_yaw_freq,
    } if has_overlay else None

    pos_unit = {1.0: "m", 1000.0: "mm"}.get(args.linear_scale, f"m×{args.linear_scale}")
    rot_unit = "deg"

    title_parts = []
    if args.linear_scale != 1.0:
        title_parts.append(f"linear×{args.linear_scale}")
    if args.angular_scale != 1.0:
        title_parts.append(f"angular×{args.angular_scale}")
    if overlay:
        parts = []
        if args.overlay_roll_amp:
            parts.append(f"roll {args.overlay_roll_amp}°@{args.overlay_roll_freq}Hz")
        if args.overlay_pitch_amp:
            parts.append(f"pitch {args.overlay_pitch_amp}°@{args.overlay_pitch_freq}Hz")
        if args.overlay_yaw_amp:
            parts.append(f"yaw {args.overlay_yaw_amp}°@{args.overlay_yaw_freq}Hz")
        title_parts.append("overlay: " + ", ".join(parts))

    print(f"Loading {args.csv} ...")
    data = load(args.csv)
    data = apply_augmentation(data, args.linear_scale, args.angular_scale, overlay)
    print_stats(data, pos_unit, rot_unit)
    print("\nRaw velocity stats:")
    print_rate_stats(data, pos_unit, rot_unit)

    smoothed_data = None
    if args.smooth_linear > 0 or args.smooth_angular > 0:
        smoothed_data = apply_smoothing(data, args.smooth_linear, args.smooth_angular)
        title_parts.append(f"smooth lin={args.smooth_linear:.3g} ang={args.smooth_angular:.3g}")
        print(f"\nSmoothed (linear s={args.smooth_linear:.3g}, angular s={args.smooth_angular:.3g}) — sampled at original timestamps:")
        print_stats(smoothed_data, pos_unit, rot_unit)
        print("\nSmoothed velocity stats:")
        print_rate_stats(smoothed_data, pos_unit, rot_unit)

    plot(data, pos_unit, rot_unit, title_suffix=", ".join(title_parts),
         smoothed_data=smoothed_data, save_path=args.save)


if __name__ == "__main__":
    main([
        "--csv", "ship_emulation/vessel_motion_clean.csv",
        "--linear-scale", "1000",
        "--angular-scale", "1.0",
        "--overlay-roll-amp", "0.0",
        "--overlay-roll-freq", "0.1",
        "--overlay-pitch-amp", "0.0",
        "--overlay-pitch-freq", "0.1",
        "--overlay-yaw-amp", "0.0",
        "--overlay-yaw-freq", "0.1",
        "--smooth-linear", "1e9",
        # "--smooth-angular", "1e2",
        # "--save", "vessel_motion.png",
    ])
