"""Phase 2 — ship motion emulation from a random 3-minute window.

Loads CSV_FILE, draws a random contiguous window of WINDOW_DURATION seconds,
optionally applies a smooth fade-in (raised-cosine ramp), and streams Cartesian
velocities to the robot via speedL for smooth, gap-free trajectory following.

Edit the CONFIGURATION block below, then run:
    python -m ship_emulation.phase2

Set RANDOM_SEED to an integer to reproduce the same window across runs.
Set FADE_IN_ENABLED = False to start playback at full amplitude immediately.
"""
import logging
import random
import signal
import sys
from pathlib import Path
from typing import Iterator, List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ship_emulation.config import AugmentationConfig, EmulatorConfig
from ship_emulation.data_source import AugmentedSource, CsvFileSource, DataSource, ShipPose
from ship_emulation.emulator import EmulationError
from ship_emulation.motion_primitives import FadeInSource
from ship_emulation.robot_interface import UR16Interface
from ship_emulation.ros_bridge import RosBridge
from ship_emulation.safety import SafetyChecker, SafetyViolation

# ── CONFIGURATION ─────────────────────────────────────────────────────────────

ROBOT_IP = "172.17.0.2"

# Path to the vessel motion CSV (relative to working directory or absolute).
# Available files in ship_emulation/:
#   1_vessel_motion_clean.csv
#   2_vessel_motion_clean.csv  /  2a_vessel_motion_clean.csv  /  2b_vessel_motion_clean.csv
#   3_vessel_motion_clean.csv
CSV_FILE = "ship_emulation/1_vessel_motion_com_1m.csv"

WINDOW_DURATION = 180.0  # s — length of the sampled window (3 minutes)
RANDOM_SEED: Optional[int] = None  # set to an int for reproducibility

# Unit conversion applied to raw CSV columns
LINEAR_SCALE  = 1000.0  # multiply x/y/z  (default: m → mm)
ANGULAR_SCALE =    1.0  # multiply roll/pitch/yaw

# Fade-in: smooth amplitude ramp from the initial pose at the start of playback
FADE_IN_ENABLED  = True
FADE_IN_DURATION = 10.0  # s — duration of the raised-cosine onset ramp

# Acceleration for velocity streaming (speedL).  Higher values let the robot
# track rapid velocity changes more faithfully; lower values give smoother but
# more lag-prone motion.  2000 mm/s² is a good default for ship-motion rates.
SERVO_ACCEL = 2000.0  # mm/s²

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
) -> tuple:
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
    return FadeInSource(augmented, duration=fade_in_duration, enabled=fade_in_enabled), t_start


def _servo_run(
    poses: List[ShipPose],
    interface: UR16Interface,
    safety: SafetyChecker,
    on_move,
    servo_accel: float,
) -> bool:
    """Stream Cartesian velocities derived from the pose sequence via speedL.

    For each consecutive pair of poses, the finite-difference velocity is sent
    as a speedL segment of exactly dt seconds.  The URScript executes each
    segment on a background thread and returns immediately; the next segment
    joins the thread first, giving gapless back-to-back execution with no
    moveL stop-start overhead.

    Returns True on clean completion, False if interrupted.
    """
    if not poses:
        return True

    # Pre-check the entire commanded trajectory against workspace limits.
    for pose in poses:
        try:
            safety.check(pose)
        except SafetyViolation as e:
            raise EmulationError(f"Safety violation in planned trajectory: {e}") from e

    # Move to the first pose slowly before streaming begins.
    log.info("Moving to first trajectory pose ...")
    interface.move_linear_at(poses[0].as_robot_pose(), 100.0, 5.0)
    interface.wait_until_stopped()

    stop_requested = False

    def _handle(sig, frame):
        nonlocal stop_requested
        stop_requested = True
        raise KeyboardInterrupt

    orig_sigint  = signal.getsignal(signal.SIGINT)
    orig_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)

    n_segments = 0
    try:
        for i in range(len(poses) - 1):
            if stop_requested:
                break
            p0, p1 = poses[i], poses[i + 1]
            dt = p1.timestamp - p0.timestamp
            r0, r1 = p0.as_robot_pose(), p1.as_robot_pose()
            velocity = tuple((r1[j] - r0[j]) / dt for j in range(6))
            interface.servo_linear_velocity(velocity, servo_accel, dt)
            if on_move is not None:
                on_move(p1)
            n_segments += 1

        interface.stop_linear(servo_accel)

    except KeyboardInterrupt:
        try:
            interface.stop_linear(servo_accel)
        except Exception:
            pass
        return False
    finally:
        signal.signal(signal.SIGINT, orig_sigint)
        signal.signal(signal.SIGTERM, orig_sigterm)

    log.info("Velocity streaming complete: %d segments", n_segments)
    return not stop_requested


def _enter(prompt: str) -> None:
    print(prompt)
    input()  # KeyboardInterrupt propagates to caller


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    cfg = EmulatorConfig()
    cfg.robot.ip = ROBOT_IP

    try:
        source, window_t_start = _build_source(
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
        "\n" + "═" * 62 + "\n"
        "  Phase 2 — Ship motion emulation\n"
        + "═" * 62 + "\n"
        f"  CSV    : {CSV_FILE}\n"
        f"  Window : {WINDOW_DURATION:.0f} s\n"
        f"  Fade-in: {FADE_IN_DURATION:.0f} s ({'enabled' if FADE_IN_ENABLED else 'DISABLED'})\n"
        + "─" * 62 + "\n"
        "  Ensure the workspace is clear.\n"
        "  Keep the physical E-stop within reach at all times.\n"
        + "─" * 62
    )

    bridge = RosBridge("phase2")

    try:
        _enter("\nPress ENTER to connect to the robot, or Ctrl+C to abort.")

        with source, UR16Interface(cfg.robot) as interface:
            safety = SafetyChecker(cfg.safety)
            bridge.start_feedback(lambda: interface.current_pose)
            bridge.set_context("phase2")

            _enter(
                "\nConnected to robot.\n"
                "Press ENTER to move to home position, or Ctrl+C to abort."
            )
            interface.move_home()
            interface.wait_until_stopped()
            print("\nAt home position.")

            log.info("Loading trajectory ...")
            poses = list(source.poses())
            actual_duration = poses[-1].timestamp - poses[0].timestamp if poses else 0.0
            log.info("Loaded %d poses  (%.1f s)", len(poses), actual_duration)

            _enter(
                f"\nTrajectory loaded: {len(poses)} poses  ({actual_duration:.1f} s)\n"
                f"Window start     : t={window_t_start:.3f} s in CSV  (seed={RANDOM_SEED})\n"
                f"Fade-in          : {FADE_IN_DURATION:.0f} s ({'enabled' if FADE_IN_ENABLED else 'DISABLED'})\n"
                "Press ENTER to start emulation, or Ctrl+C to abort."
            )

            _servo_run(poses, interface, safety, bridge, SERVO_ACCEL)

            print("\nEmulation complete. Returning to home position.")
            interface.move_home()
            interface.wait_until_stopped()
            print("At home position.")

    except KeyboardInterrupt:
        print("\nAborted.")
        return 0
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
