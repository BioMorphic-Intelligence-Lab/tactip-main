from dataclasses import dataclass, field
from typing import Tuple


@dataclass
class WorkspaceLimits:
    x_min: float = -500.0
    x_max: float = 500.0
    y_min: float = -500.0
    y_max: float = 500.0
    z_min: float = -500.0
    z_max: float = 500.0
    roll_min: float = -30.0
    roll_max: float = 30.0
    pitch_min: float = -30.0
    pitch_max: float = 30.0
    yaw_min: float = -30.0
    yaw_max: float = 30.0


@dataclass
class SafetyConfig:
    workspace: WorkspaceLimits = field(default_factory=WorkspaceLimits)
    max_linear_rate: float = 200.0   # mm/s
    max_angular_rate: float = 30.0   # deg/s
    min_dt: float = 0.005            # s


@dataclass
class RobotConfig:
    ip: str = "192.168.1.100"
    tcp: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    work_frame: Tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    home_joint_angles: Tuple[float, ...] = (0.0, -90.0, 0.0, -90.0, 0.0, 0.0)
    linear_speed: float = 100.0      # mm/s
    angular_speed: float = 20.0      # deg/s
    linear_accel: float = 500.0      # mm/s²
    angular_accel: float = 50.0      # deg/s²
    blend_radius: float = 0.0        # mm


@dataclass
class EmulatorConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
