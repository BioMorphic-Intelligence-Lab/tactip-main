import logging
import math
import time
from typing import Tuple

from cri.robot import SyncRobot, AsyncRobot
from cri.controller import RTDEController

from ship_emulation.config import RobotConfig

log = logging.getLogger(__name__)


class UR16Interface:
    def __init__(self, config: RobotConfig):
        self._config = config
        self._controller: RTDEController = None
        self._robot: AsyncRobot = None
        self._motion_pending = False

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.close()

    def connect(self):
        cfg = self._config
        self._controller = RTDEController(ip=cfg.ip)
        self._controller.linear_accel = cfg.linear_accel
        self._controller.angular_accel = cfg.angular_accel
        sync = SyncRobot(self._controller)
        self._robot = AsyncRobot(sync)
        self._robot.tcp = cfg.tcp
        self._robot.coord_frame = cfg.work_frame
        self._robot.linear_speed = cfg.linear_speed
        self._robot.angular_speed = cfg.angular_speed
        self._robot.blend_radius = cfg.blend_radius
        self._motion_pending = False
        log.info("Connected to UR16 at %s — %s", cfg.ip, self._robot.info)

    def close(self):
        if self._robot is not None:
            try:
                if self._motion_pending:
                    self._robot.async_result()
            except Exception:
                pass
            self._robot.close()
            self._robot = None
            self._controller = None
            self._motion_pending = False

    def move_home(self):
        if self._motion_pending:
            self._robot.async_result()
            self._motion_pending = False
        self._robot.move_joints(self._config.home_joint_angles)
        log.info("At home position")

    def move_to(self, pose: Tuple[float, ...]):
        if self._motion_pending:
            self._robot.async_result()
            self._motion_pending = False
        log.debug("move_to: x=%.1f y=%.1f z=%.1f roll=%.3f pitch=%.3f yaw=%.3f", *pose)
        self._robot.async_move_linear(pose)
        self._motion_pending = True

    def wait_for_motion(self):
        if self._motion_pending:
            self._robot.async_result()
            self._motion_pending = False

    def wait_until_stopped(self, threshold_mm_s: float = 2.0, timeout_s: float = 10.0) -> None:
        """Poll actual TCP speed until the robot is physically at rest."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            vel = self._controller.linear_velocity
            if math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2) < threshold_mm_s:
                return
            time.sleep(0.05)
        log.warning("wait_until_stopped: robot did not stop within %.1f s", timeout_s)

    def is_motion_done(self) -> bool:
        if not self._motion_pending:
            return True
        if self._robot.async_done():
            self._robot.async_result()
            self._motion_pending = False
            return True
        return False

    def emergency_stop(self):
        log.warning("Emergency stop triggered — software stop only; use physical E-stop as primary")
        try:
            self._controller.stop_linear_velocity(5000)
        except Exception as e:
            log.error("Emergency stop command failed: %s", e)
        finally:
            self.close()

    @property
    def current_pose(self):
        return self._robot.pose if self._robot else None

    @property
    def current_joint_angles(self):
        return self._robot.joint_angles if self._robot else None
