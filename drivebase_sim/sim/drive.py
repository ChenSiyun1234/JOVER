"""Skid-steer kinematics in ROS2 diff_drive_controller form, not team code."""

from dataclasses import dataclass
import numpy as np
from .motor import KE

WHEELS = tuple(f"{side}{tag}" for side in "lr" for tag in "fmr")


@dataclass
class Drive:
    separation: float
    radius: float
    slip_multiplier: float = 1.25

    @classmethod
    def from_scene(cls, model, data):
        """Read wheel centres after forward kinematics; multiplier is assumed."""
        left = data.xpos[model.body("wheel_lf").id]
        right = data.xpos[model.body("wheel_rf").id]
        radius = float(model.geom_size[model.geom("tire_lf").id, 0])
        return cls(float(np.linalg.norm(left - right)), radius)

    def speeds(self, velocity: float, yaw_rate: float):
        half_turn = yaw_rate * self.separation * self.slip_multiplier / 2
        return np.repeat([(velocity - half_turn) / self.radius,
                          (velocity + half_turn) / self.radius], 3)


@dataclass
class YawRatePI:
    """Optional 100 Hz outer loop; caller supplies IMU-site gyro z.

    Gains map yaw error to an additive kinematic yaw command. Conditional
    integration prevents windup at wheel-speed clipping, not torque saturation.
    Noise is added at sampling with an independent reproducible generator.
    """

    kp: float = 2.0
    ki: float = 1.0
    period: float = 0.010
    noise_std: float = 0.01
    seed: int = 20261109
    integral: float = 0.0

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)
        self.measurement = 0.0

    def speeds(self, drive, velocity, yaw_rate, gyro_z, voltage):
        """Sample once, update PI and clip each wheel to V/Ke rad/s."""
        self.measurement = gyro_z + self.rng.normal(0.0, self.noise_std)
        error = yaw_rate - self.measurement
        requested = drive.speeds(velocity, yaw_rate + self.kp * error + self.integral)
        limit = max(0.0, voltage) / KE
        targets = np.clip(requested, -limit, limit)
        # Turn correction increases right targets and decreases left targets.
        blocked = error * np.mean((requested - targets)[3:] -
                                  (requested - targets)[:3]) > 0
        if not blocked:
            self.integral += self.ki * error * self.period
        return targets


@dataclass
class TorqueGovernor:
    """100 Hz yaw scaler using only decoded CAN torque, no ground sensors.

    Fast multiplicative reduction above 85%; slow linear recovery below 75%.
    A disabled/missing driver inhibits turning until a deliberate restart.
    """

    limit_torque: float = 9.8
    scale: float = 1.0
    period: float = .01

    def update(self, yaw, feedback):
        if len(feedback) != 6 or any(v["error"] != 1 for v in feedback.values()):
            self.scale = 0.0
            return 0.0
        ratio = max(abs(v["torque"]) for v in feedback.values()) / self.limit_torque
        if ratio > .85:
            self.scale *= max(0.0, 1 - 4*self.period*(ratio/.85-1))
        elif ratio < .75:
            self.scale = min(1.0, self.scale + .03*self.period)
        return yaw * self.scale
