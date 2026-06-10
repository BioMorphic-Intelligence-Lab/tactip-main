import logging
import signal
import time

from ship_emulation.config import EmulatorConfig
from ship_emulation.data_source import DataSource
from ship_emulation.robot_interface import UR16Interface
from ship_emulation.safety import SafetyChecker, SafetyViolation

log = logging.getLogger(__name__)

_SLEEP_MIN = 0.005  # skip sleep calls shorter than this to avoid OS overhead


class EmulationError(RuntimeError):
    pass


class ShipEmulator:
    def __init__(
        self,
        source: DataSource,
        interface: UR16Interface,
        safety: SafetyChecker,
        config: EmulatorConfig,
        on_move=None,
        home_at_start: bool = True,
        home_at_end: bool = True,
    ):
        self._source = source
        self._interface = interface
        self._safety = safety
        self._config = config
        self._on_move = on_move  # optional callable(ShipPose) called after each move_to
        self._home_at_start = home_at_start
        self._home_at_end = home_at_end

    def run(self):
        self._safety.reset()

        stop_requested = False

        def _handle_signal(sig, frame):
            nonlocal stop_requested
            log.info("Signal %s received — stopping", sig)
            stop_requested = True
            raise KeyboardInterrupt

        original_sigint = signal.getsignal(signal.SIGINT)
        original_sigterm = signal.getsignal(signal.SIGTERM)
        signal.signal(signal.SIGINT, _handle_signal)
        signal.signal(signal.SIGTERM, _handle_signal)

        try:
            if self._home_at_start:
                self._interface.move_home()
                self._interface.wait_until_stopped()

            pose_iter = iter(self._source.poses())
            first_pose = next(pose_iter, None)
            if first_pose is None:
                return

            try:
                self._safety.check(first_pose)
            except SafetyViolation as e:
                log.error("Safety violation on first pose: %s", e)
                self._interface.emergency_stop()
                raise EmulationError(f"Safety violation: {e}") from e

            log.info("Moving to start pose before beginning playback")
            self._interface.move_to(first_pose.as_robot_pose())
            self._interface.wait_for_motion()

            wall_origin = time.monotonic()
            sim_origin = first_pose.timestamp
            pose_count = 1
            late_count = 0

            for pose in pose_iter:
                if stop_requested:
                    break

                try:
                    self._safety.check(pose)
                except SafetyViolation as e:
                    log.error("Safety violation: %s", e)
                    self._interface.emergency_stop()
                    raise EmulationError(f"Safety violation: {e}") from e

                target_wall = wall_origin + (pose.timestamp - sim_origin)
                now = time.monotonic()
                sleep_s = target_wall - now
                if sleep_s > _SLEEP_MIN:
                    time.sleep(sleep_s)

                if not self._interface.is_motion_done():
                    self._interface.wait_for_motion()
                    now = time.monotonic()
                    if now > target_wall:
                        sim_t = pose.timestamp - sim_origin
                        lateness_ms = (now - target_wall) * 1000.0
                        n_skip = 0
                        while target_wall < time.monotonic():
                            nxt = next(pose_iter, None)
                            if nxt is None:
                                pose = None
                                break
                            n_skip += 1
                            pose = nxt
                            target_wall = wall_origin + (pose.timestamp - sim_origin)
                        late_count += 1
                        log.warning(
                            "pose %d  sim_t=%.3fs  late=%.1fms  skipped %d stale pose(s)",
                            pose_count, sim_t, lateness_ms, n_skip,
                        )
                        if pose is None:
                            break
                        try:
                            self._safety.check(pose)
                        except SafetyViolation as e:
                            log.error("Safety violation on catch-up pose: %s", e)
                            self._interface.emergency_stop()
                            raise EmulationError(f"Safety violation: {e}") from e

                self._interface.move_to(pose.as_robot_pose())
                if self._on_move is not None:
                    self._on_move(pose)
                pose_count += 1

            self._interface.wait_for_motion()
            if self._home_at_end:
                self._interface.move_home()

        except KeyboardInterrupt:
            log.info("Interrupted — closing connection")
            try:
                self._interface.close()
            except Exception:
                pass
            return False

        finally:
            signal.signal(signal.SIGINT, original_sigint)
            signal.signal(signal.SIGTERM, original_sigterm)

        log.info(
            "Emulation complete: %d poses replayed, %d late moves",
            pose_count,
            late_count,
        )
        return True
