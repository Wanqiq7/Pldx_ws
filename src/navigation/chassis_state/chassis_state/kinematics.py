"""Rigid-body recovery. All input twists are expressed in the IMU child frame."""
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Geometry:
    pivot: np.ndarray = field(default_factory=lambda: np.zeros(3))
    axis_rotation: Rotation = field(default_factory=Rotation.identity)
    imu_translation: np.ndarray = field(default_factory=lambda: np.zeros(3))
    imu_rotation: Rotation = field(default_factory=Rotation.identity)
    joint_zero: float = 0.0
    joint_direction: float = 1.0

    def mount_rotation(self, angle):
        return self.axis_rotation * Rotation.from_rotvec(
            [0., 0., self.joint_direction * angle + self.joint_zero])


@dataclass
class State:
    position: np.ndarray
    rotation: Rotation
    linear: np.ndarray
    angular: np.ndarray


def recover(geometry, position, rotation, linear, angular, angle, velocity):
    """Return chassis pose in world and twist in chassis, at the input timestamp."""
    mount = geometry.mount_rotation(angle)
    chassis = rotation * (mount * geometry.imu_rotation).inv()
    pivot_world = chassis.apply(geometry.pivot)
    arm_world = (chassis * mount).apply(geometry.imu_translation)
    imu_omega = rotation.apply(angular)
    axis_world = (chassis * geometry.axis_rotation).apply([0., 0., 1.])
    omega = imu_omega - axis_world * geometry.joint_direction * velocity
    speed = (rotation.apply(linear) - np.cross(omega, pivot_world)
             - np.cross(imu_omega, arm_world))
    return State(np.asarray(position) - pivot_world - arm_world, chassis,
                 chassis.inv().apply(speed), chassis.inv().apply(omega))
