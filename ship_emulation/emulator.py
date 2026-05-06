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
    ):
        self._source = source
        self._interface = interface
        self._safety = safety
        self._config = config

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
                sleep_s = target_wall - time.monotonic()
                if sleep_s > _SLEEP_MIN:
                    time.sleep(sleep_s)

                if not self._interface.is_motion_done():
                    late_count += 1
                    log.warning(
                        "Robot still moving for pose %d — waiting. "
                        "Consider reducing max_linear_rate/max_angular_rate or "
                        "increasing robot speed.",
                        pose_count,
                    )
                    self._interface.wait_for_motion()

                self._interface.move_to(pose.as_robot_pose())
                pose_count += 1

            self._interface.wait_for_motion()
            self._interface.move_home()

        except KeyboardInterrupt:
            log.info("Interrupted — closing connection")
            try:
                self._interface.close()
            except Exception:
                pass
            return

        finally:
            signal.signal(signal.SIGINT, original_sigint)
            signal.signal(signal.SIGTERM, original_sigterm)

        log.info(
            "Emulation complete: %d poses replayed, %d late moves",
            pose_count,
            late_count,
        )
