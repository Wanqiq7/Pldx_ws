"""Small rigid-transform helpers used by the simulation odometry adapter."""

import math


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in quaternion)


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_from_rpy(roll, pitch, yaw):
    """Create an xyzw quaternion from intrinsic roll, pitch, and yaw."""
    half_roll = roll / 2.0
    half_pitch = pitch / 2.0
    half_yaw = yaw / 2.0
    sin_roll = math.sin(half_roll)
    cos_roll = math.cos(half_roll)
    sin_pitch = math.sin(half_pitch)
    cos_pitch = math.cos(half_pitch)
    sin_yaw = math.sin(half_yaw)
    cos_yaw = math.cos(half_yaw)
    return (
        sin_roll * cos_pitch * cos_yaw - cos_roll * sin_pitch * sin_yaw,
        cos_roll * sin_pitch * cos_yaw + sin_roll * cos_pitch * sin_yaw,
        cos_roll * cos_pitch * sin_yaw - sin_roll * sin_pitch * cos_yaw,
        cos_roll * cos_pitch * cos_yaw + sin_roll * sin_pitch * sin_yaw,
    )


def yaw_from_quaternion(quaternion):
    """Extract the planar yaw component from an xyzw quaternion."""
    x, y, z, w = normalize_quaternion(quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def planarize_transform(transform):
    """Keep a rigid transform's translation and project its rotation to yaw."""
    translation, rotation = transform
    return translation, quaternion_from_yaw(yaw_from_quaternion(rotation))


def project_to_navigation_plane(transform):
    """Project a map-frame base pose to x/y/yaw with zero navigation height."""
    translation, rotation = transform
    return (
        (translation[0], translation[1], 0.0),
        quaternion_from_yaw(yaw_from_quaternion(rotation)),
    )


def rotate_vector(quaternion, vector):
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def compose_transform(left, right):
    """Compose T_a_b and T_b_c, returning T_a_c."""
    left_translation, left_rotation = left
    right_translation, right_rotation = right
    rotated = rotate_vector(left_rotation, right_translation)
    translation = tuple(left_translation[index] + rotated[index] for index in range(3))
    rotation = normalize_quaternion(quaternion_multiply(left_rotation, right_rotation))
    return translation, rotation


def inverse_transform(transform):
    translation, rotation = transform
    qx, qy, qz, qw = normalize_quaternion(rotation)
    inverse_rotation = (-qx, -qy, -qz, qw)
    inverse_translation = rotate_vector(
        inverse_rotation, tuple(-value for value in translation)
    )
    return inverse_translation, inverse_rotation


def compute_map_to_camera(
    map_to_base_initial, base_to_tracking, camera_to_tracking_initial
):
    """Align Point-LIO's native frame to the requested initial base pose."""
    return compose_transform(
        compose_transform(map_to_base_initial, base_to_tracking),
        inverse_transform(camera_to_tracking_initial),
    )


def compute_map_to_base(map_to_camera, camera_to_tracking, base_to_tracking):
    """Recover the base pose without discarding tracking-frame tilt."""
    return compose_transform(
        compose_transform(map_to_camera, camera_to_tracking),
        inverse_transform(base_to_tracking),
    )


def cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def transform_twist(linear, angular, base_to_tracking):
    """Move a tracking-frame twist to the base origin and coordinates."""
    offset, rotation = base_to_tracking
    angular_base = rotate_vector(rotation, angular)
    linear_at_tracking = rotate_vector(rotation, linear)
    lever_velocity = cross(angular_base, offset)
    linear_base = tuple(
        linear_at_tracking[index] - lever_velocity[index] for index in range(3)
    )
    return linear_base, angular_base


def rotation_matrix(quaternion):
    x, y, z, w = normalize_quaternion(quaternion)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def transform_twist_covariance(covariance, base_to_tracking):
    """Apply the rigid-body twist adjoint to a row-major 6x6 covariance."""
    if len(covariance) != 36:
        raise ValueError("twist covariance must contain 36 values")

    offset, rotation = base_to_tracking
    rotation = rotation_matrix(rotation)
    skew = (
        (0.0, -offset[2], offset[1]),
        (offset[2], 0.0, -offset[0]),
        (-offset[1], offset[0], 0.0),
    )
    skew_rotation = tuple(
        tuple(
            sum(skew[row][k] * rotation[k][column] for k in range(3))
            for column in range(3)
        )
        for row in range(3)
    )

    adjoint = [[0.0] * 6 for _ in range(6)]
    for row in range(3):
        for column in range(3):
            adjoint[row][column] = rotation[row][column]
            adjoint[row][column + 3] = skew_rotation[row][column]
            adjoint[row + 3][column + 3] = rotation[row][column]

    source = [list(covariance[row * 6 : (row + 1) * 6]) for row in range(6)]
    intermediate = [
        [
            sum(adjoint[row][k] * source[k][column] for k in range(6))
            for column in range(6)
        ]
        for row in range(6)
    ]
    transformed = [
        [
            sum(intermediate[row][k] * adjoint[column][k] for k in range(6))
            for column in range(6)
        ]
        for row in range(6)
    ]
    return [value for row in transformed for value in row]


def project_twist_covariance_to_navigation_plane(covariance):
    """Remove covariance terms for suppressed z, roll, and pitch velocities."""
    if len(covariance) != 36:
        raise ValueError("twist covariance must contain 36 values")

    projected = list(covariance)
    for axis in (2, 3, 4):
        for index in range(6):
            projected[axis * 6 + index] = 0.0
            projected[index * 6 + axis] = 0.0
    return projected
