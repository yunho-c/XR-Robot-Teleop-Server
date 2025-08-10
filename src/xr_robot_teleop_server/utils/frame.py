import numpy as np
from scipy.spatial.transform import Rotation as R

from ..schemas.body_pose import Bone
from ..schemas.openxr_skeletons import FullBodyBoneId
from .coordinate import transform_to_frame


def get_body_frame(
    bone_positions: dict,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    """
    Calculates the body-centric coordinate frame (origin and rotation matrix).

    Args:
        bone_positions (dict): A dictionary mapping bone IDs to their positions.

    Returns:
        A tuple containing:
        - np.ndarray: The origin of the body frame (shoulder_center).
        - np.ndarray: The rotation matrix from world to body frame (R_world_body).
        Returns (None, None) if essential bones are missing.
    """
    # Get key body landmarks
    left_shoulder = bone_positions.get(FullBodyBoneId.FullBody_LeftArmUpper)
    right_shoulder = bone_positions.get(FullBodyBoneId.FullBody_RightArmUpper)
    hips = bone_positions.get(FullBodyBoneId.FullBody_Hips)

    if left_shoulder is None or right_shoulder is None or hips is None:
        return None, None

    # Calculate body center (origin of the body frame)
    shoulder_center = (left_shoulder + right_shoulder) / 2
    hip_center = hips

    # Create body-centric coordinate frame
    # Y-axis: right to left (shoulder line)
    y_axis = left_shoulder - right_shoulder
    y_axis = y_axis / (np.linalg.norm(y_axis) + 1e-8)

    # Z-axis: up direction (shoulder to hip, inverted)
    torso_vector = hip_center - shoulder_center
    z_axis = -torso_vector / (np.linalg.norm(torso_vector) + 1e-8)

    # X-axis: forward direction (cross product)
    x_axis = np.cross(y_axis, z_axis)
    x_axis = x_axis / (np.linalg.norm(x_axis) + 1e-8)

    # Create transformation matrix from world to body-centric frame
    rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
    # normalize the rotation matrix
    U_rot, _, V_rot_T = np.linalg.svd(rotation_matrix)
    R_world_body = U_rot @ V_rot_T

    return shoulder_center, R_world_body


def get_body_centric_coordinates(bones: list[Bone]) -> list[Bone]:
    """
    Convert bone positions to a body-centric coordinate system.
    """
    bone_positions = {b.id: np.array(b.position) for b in bones}
    bone_rotations = {b.id: np.array(b.rotation) for b in bones}

    shoulder_center, R_world_body = get_body_frame(bone_positions)

    if shoulder_center is None or R_world_body is None:
        return []

    body_centric_bones = []
    r_world_body = R.from_matrix(R_world_body)

    for bone in bones:
        # Transform position
        body_centric_pos = transform_to_frame(
            bone_positions[bone.id], shoulder_center, R_world_body
        )

        # Transform rotation
        r_world_bone = R.from_quat(bone_rotations[bone.id])
        r_body_bone = r_world_body.inv() * r_world_bone
        body_centric_rot = r_body_bone.as_quat()

        body_centric_bones.append(
            Bone(
                id=bone.id,
                position=tuple(body_centric_pos),
                rotation=tuple(body_centric_rot),
            )
        )

    return body_centric_bones


def get_hand_frame(side: str, bone_positions: dict) -> tuple[np.ndarray | None, np.ndarray | None]:
    """
    Calculates the hand-centric coordinate frame for a given side.

    Args:
        side (str): "left" or "right".
        bone_positions (dict): A dictionary mapping bone IDs to their positions.

    Returns:
        A tuple containing:
        - np.ndarray: The origin of the hand frame (wrist position).
        - np.ndarray: The rotation matrix from world to hand frame.
        Returns (None, None) if essential bones are missing.
    """
    side_pascal = side.capitalize()
    wrist_pos = bone_positions.get(getattr(FullBodyBoneId, f"FullBody_{side_pascal}HandWrist"))
    middle_metacarpal_pos = bone_positions.get(
        getattr(FullBodyBoneId, f"FullBody_{side_pascal}HandMiddleMetacarpal")
    )

    if wrist_pos is None or middle_metacarpal_pos is None:
        return None, None

    # Z-axis: from wrist to middle metacarpal (up)
    z_axis = middle_metacarpal_pos - wrist_pos
    z_axis /= np.linalg.norm(z_axis) + 1e-8

    # To get a consistent frame, we need another vector.
    # We can use other fingers to define a plane.
    index_meta_pos = bone_positions.get(
        getattr(FullBodyBoneId, f"FullBody_{side_pascal}HandIndexMetacarpal")
    )
    little_meta_pos = bone_positions.get(
        getattr(FullBodyBoneId, f"FullBody_{side_pascal}HandLittleMetacarpal")
    )

    if index_meta_pos is None or little_meta_pos is None:
        return None, None

    # Y-axis: from index to little finger, defines the hand's horizontal direction
    y_axis_temp = little_meta_pos - index_meta_pos
    y_axis_temp /= np.linalg.norm(y_axis_temp) + 1e-8

    # Create an orthonormal basis
    x_axis = np.cross(y_axis_temp, z_axis)
    x_axis /= np.linalg.norm(x_axis) + 1e-8

    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis) + 1e-8

    rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
    U_rot, _, V_rot_T = np.linalg.svd(rotation_matrix)
    R_world_hand = U_rot @ V_rot_T

    return wrist_pos, R_world_hand


def get_hand_centric_coordinates(bones: list[Bone], side: str) -> list[Bone]:
    """
    Convert hand bone positions and rotations to a hand-centric coordinate system.

    Args:
        bones (list[Bone]): A list of all bone objects.
        side (str): "left" or "right".

    Returns:
        list[Bone]: A list of bone objects for the specified hand, with transformed
                    position and rotation relative to the hand's frame.
    """
    bone_positions = {b.id: np.array(b.position) for b in bones}
    bone_rotations = {b.id: np.array(b.rotation) for b in bones}
    side_pascal = side.capitalize()

    hand_origin, r_world_hand = get_hand_frame(side, bone_positions)

    if hand_origin is None or r_world_hand is None:
        return []

    hand_centric_bones = []
    r_world_hand_rot = R.from_matrix(r_world_hand)

    for bone in bones:
        bone_name = FullBodyBoneId(bone.id).name
        if f"{side_pascal}Hand" in bone_name:
            # Transform position
            hand_centric_pos = transform_to_frame(
                bone_positions[bone.id], hand_origin, r_world_hand
            )

            # Transform rotation
            r_world_bone = R.from_quat(bone_rotations[bone.id])
            r_hand_bone = r_world_hand_rot.inv() * r_world_bone
            hand_centric_rot = r_hand_bone.as_quat()

            hand_centric_bones.append(
                Bone(
                    id=bone.id,
                    position=tuple(hand_centric_pos),
                    rotation=tuple(hand_centric_rot),
                )
            )

    return hand_centric_bones


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
