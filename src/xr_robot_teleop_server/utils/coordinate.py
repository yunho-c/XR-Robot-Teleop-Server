import numpy as np
from scipy.spatial.transform import Rotation as R

from xr_robot_teleop_server.schemas.body_pose import Bone


def convert_unity_to_right_handed_z_up(
    position: tuple[float, float, float],
    rotation: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """
    Converts position and rotation from Unity's left-handed, Y-up coordinate system
    to a right-handed, Z-up coordinate system (+X Forward, +Y Left, +Z Up).
    """
    # Position conversion: Unity (x,y,z) -> (z, -x, y)
    new_position = (position[2], -position[0], position[1])

    # Rotation quaternion conversion: Unity (qx,qy,qz,qw) -> (-qz, qx, -qy, qw)
    qx, qy, qz, qw = rotation
    new_rotation = (-qz, qx, -qy, qw)

    return new_position, new_rotation


def transform_to_frame(world_pos, origin, rotation):
    """Transform a position to another frame."""
    translated = world_pos - origin
    pos = rotation.T @ translated
    return pos


def rotate_bones(bones: list[Bone], rotation: R) -> list[Bone]:
    """
    Applies a rotation to a list of bones.

    Args:
        bones: A list of Bone objects.
        rotation: A scipy.spatial.transform.Rotation object.

    Returns:
        A new list of Bone objects with the rotation applied.
    """
    rotated_bones = []
    for bone in bones:
        # Apply rotation to position
        new_position = rotation.apply(bone.position)

        # Apply rotation to orientation
        bone_rotation = R.from_quat(bone.rotation)
        new_orientation = rotation * bone_rotation
        new_orientation_quat = new_orientation.as_quat()

        rotated_bones.append(
            Bone(
                id=bone.id,
                position=tuple(new_position),
                rotation=tuple(new_orientation_quat),
            )
        )
    return rotated_bones
