"""Phase 2 — ship motion emulation from a random 3-minute window.

Loads CSV_FILE, draws a random contiguous window of WINDOW_DURATION seconds,
optionally applies a smooth fade-in (raised-cosine ramp), and runs the emulator.

Edit the CONFIGURATION block below, then run:
    python -m ship_emulation.phase2

Set RANDOM_SEED to an integer to reproduce the same window across runs.
Set FADE_IN_ENABLED = False to start playback at full amplitude immediately.
"""
import logging
import random
import sys
from pathlib import Path
from typing import Iterator, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ship_emulation.config import AugmentationConfig, EmulatorConfig
from ship_emulation.data_source import AugmentedSource, CsvFileSource, DataSource, ShipPose
from ship_emulation.emulator import EmulationError, ShipEmulator
from ship_emulation.motion_primitives import FadeInSource
from ship_emulation.robot_interface import UR16Interface
from ship_emulation.ros_bridge import RosBridge
from ship_emulation.safety import SafetyChecker

# ── CONFIGURATION ─────────────────────────────────────────────────────────────

ROBOT_IP = "172.17.0.2"

# Path to the vessel motion CSV (relative to working directory or absolute).
# Available files in ship_emulation/:
#   1_vessel_motion_clean.csv
#   2_vessel_motion_clean.csv  /  2a_vessel_motion_clean.csv  /  2b_vessel_motion_clean.csv
#   3_vessel_motion_clean.csv
CSV_FILE = "ship_emulation/1_vessel_motion_clean.csv"

WINDOW_DURATION = 180.0  # s — length of the sampled window (3 minutes)
RANDOM_SEED: Optional[int] = None  # set to an int for reproducibility

# Unit conversion applied to raw CSV columns
LINEAR_SCALE  = 1000.0  # multiply x/y/z  (default: m → mm)
ANGULAR_SCALE =    1.0  # multiply roll/pitch/yaw

# Fade-in: smooth amplitude ramp from the initial pose at the start of playback
FADE_IN_ENABLED  = True
FADE_IN_DURATION = 10.0  # s — duration of the raised-cosine onset ramp

# ── END CONFIGURATION ─────────────────────────────────────────────────────────

log = logging.getLogger(__name__)


class _WindowedSource(DataSource):
    """Yields pre-loaded poses whose timestamps fall within [t_start, t_end]."""

    def __init__(self, all_poses: List[ShipPose], t_start: float, t_end: float):
        self._poses  = all_poses
        self._t_start = t_start
        self._t_end   = t_end

    def poses(self) -> Iterator[ShipPose]:
        for pose in self._poses:
            if pose.timestamp < self._t_start:
                continue
            if pose.timestamp > self._t_end:
                break
            yield pose

    def close(self):
        pass


def _build_source(
    csv_path: str,
    window_duration: float,
    seed: Optional[int],
    linear_scale: float,
    angular_scale: float,
    fade_in_enabled: bool,
    fade_in_duration: float,
) -> DataSource:
    with CsvFileSource(csv_path) as raw:
        all_poses = list(raw.poses())

    if not all_poses:
        raise ValueError(f"No poses loaded from {csv_path!r}")

    t_data_start = all_poses[0].timestamp
    t_data_end   = all_poses[-1].timestamp
    total_span   = t_data_end - t_data_start

    if total_span < window_duration:
        raise ValueError(
            f"CSV spans only {total_span:.1f} s — cannot sample a {window_duration:.1f} s window"
        )

    rng     = random.Random(seed)
    t_start = t_data_start + rng.uniform(0.0, total_span - window_duration)
    t_end   = t_start + window_duration

    log.info(
        "CSV data span:    t=%.3f s – %.3f s  (%.1f s total)",
        t_data_start, t_data_end, total_span,
    )
    log.info(
        "Selected window:  t=%.3f s – %.3f s  (%.1f s, seed=%s)",
        t_start, t_end, window_duration, seed,
    )

    windowed  = _WindowedSource(all_poses, t_start, t_end)
    augmented = AugmentedSource(
        windowed,
        AugmentationConfig(linear_scale=linear_scale, angular_scale=angular_scale),
    )
    return FadeInSource(augmented, duration=fade_in_duration, enabled=fade_in_enabled)


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    cfg = EmulatorConfig()
    cfg.robot.ip = ROBOT_IP

    try:
        source = _build_source(
            CSV_FILE,
            WINDOW_DURATION,
            RANDOM_SEED,
            LINEAR_SCALE,
            ANGULAR_SCALE,
            FADE_IN_ENABLED,
            FADE_IN_DURATION,
        )
    except ValueError as e:
        print(f"Configuration error: {e}", file=sys.stderr)
        return 1

    print(
        "\nWARNING: This will move the UR16 robot arm through Phase 2 ship motion emulation.\n"
        f"CSV:      {CSV_FILE}\n"
        f"Window:   {WINDOW_DURATION:.0f} s    Fade-in: {FADE_IN_DURATION:.0f} s "
        f"({'enabled' if FADE_IN_ENABLED else 'DISABLED'})\n"
        "Ensure the workspace is clear and the physical E-stop is within reach.\n"
        "Press ENTER to connect and start, or Ctrl+C to abort.\n"
    )
    try:
        input()
    except KeyboardInterrupt:
        print("\nAborted.")
        return 0

    bridge = RosBridge("phase2")

    try:
        with source, UR16Interface(cfg.robot) as interface:
            safety = SafetyChecker(cfg.safety)
            bridge.start_feedback(lambda: interface.current_pose)
            emulator = ShipEmulator(source, interface, safety, cfg, on_move=bridge)
            emulator.run()

    except EmulationError as e:
        print(f"Emulation error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Unexpected error: {e}", file=sys.stderr)
        return 1
    finally:
        bridge.close()

    log.info("Phase 2 complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
