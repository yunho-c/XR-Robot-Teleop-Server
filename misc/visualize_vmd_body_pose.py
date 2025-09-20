"""
Visualizes body pose data from a VMD CSV file.

This script reads bone position and rotation data from a CSV file with VMD (Vocaloid Motion Data)
bone names and visualizes it using rerun's 3D visualization. The CSV format should be:
frame,bone_name,loc_x,loc_y,loc_z,rot_w,rot_x,rot_y,rot_z,scale_x,scale_y,scale_z

Usage:
    python visualize_vmd_body_pose.py --file path/to/animation.csv
    python visualize_vmd_body_pose.py --file path/to/animation.csv --frame 10
    python visualize_vmd_body_pose.py --file path/to/animation.csv --animate
"""

import argparse
import csv
import enum
import time

import numpy as np

from xr_robot_teleop_server import configure_logging
from xr_robot_teleop_server.schemas.openxr_skeletons import (
    FULL_BODY_SKELETON_CONNECTIONS,
    FullBodyBoneId,
    SkeletonType,
)


# Enum for special IK markers used in calculations
class IKMarker(enum.IntEnum):
    """Special markers for IK bones used in calculations."""

    TOE_IK_LEFT = 1000
    TOE_IK_RIGHT = 1001
    TOE_IK_LEFT_TIP = 1002
    TOE_IK_RIGHT_TIP = 1003

# VMD to FullBody bone name mapping
VMD_TO_FULLBODY_MAPPING = {
    # Core body - Japanese VMD names
    "全ての親": None,  # Root bone, skip
    "センター": FullBodyBoneId.FullBody_Hips,
    "センター_tip": FullBodyBoneId.FullBody_Start,
    "下半身": FullBodyBoneId.FullBody_SpineLower,
    "上半身": FullBodyBoneId.FullBody_SpineMiddle,
    "上半身2": FullBodyBoneId.FullBody_SpineUpper,
    "首": FullBodyBoneId.FullBody_Neck,
    "頭": FullBodyBoneId.FullBody_Head,
    # Left arm
    "肩.L": FullBodyBoneId.FullBody_LeftShoulder,
    "腕.L": FullBodyBoneId.FullBody_LeftArmUpper,
    "ひじ.L": FullBodyBoneId.FullBody_LeftArmLower,
    # Right arm
    "肩.R": FullBodyBoneId.FullBody_RightShoulder,
    "腕.R": FullBodyBoneId.FullBody_RightArmUpper,
    "ひじ.R": FullBodyBoneId.FullBody_RightArmLower,
    # Left hand
    "手首.L": FullBodyBoneId.FullBody_LeftHandWrist,
    "親指０.L": FullBodyBoneId.FullBody_LeftHandThumbMetacarpal,
    "親指１.L": FullBodyBoneId.FullBody_LeftHandThumbProximal,
    "親指２.L": FullBodyBoneId.FullBody_LeftHandThumbDistal,
    "人指０.L": FullBodyBoneId.FullBody_LeftHandIndexProximal,
    "人指１.L": FullBodyBoneId.FullBody_LeftHandIndexIntermediate,
    "人指２.L": FullBodyBoneId.FullBody_LeftHandIndexDistal,
    "中指０.L": FullBodyBoneId.FullBody_LeftHandMiddleProximal,
    "中指１.L": FullBodyBoneId.FullBody_LeftHandMiddleIntermediate,
    "中指２.L": FullBodyBoneId.FullBody_LeftHandMiddleDistal,
    "薬指０.L": FullBodyBoneId.FullBody_LeftHandRingProximal,
    "薬指１.L": FullBodyBoneId.FullBody_LeftHandRingIntermediate,
    "薬指２.L": FullBodyBoneId.FullBody_LeftHandRingDistal,
    "小指０.L": FullBodyBoneId.FullBody_LeftHandLittleProximal,
    "小指１.L": FullBodyBoneId.FullBody_LeftHandLittleIntermediate,
    "小指２.L": FullBodyBoneId.FullBody_LeftHandLittleDistal,
    # Right hand
    "手首.R": FullBodyBoneId.FullBody_RightHandWrist,
    "親指０.R": FullBodyBoneId.FullBody_RightHandThumbMetacarpal,
    "親指１.R": FullBodyBoneId.FullBody_RightHandThumbProximal,
    "親指２.R": FullBodyBoneId.FullBody_RightHandThumbDistal,
    "人指０.R": FullBodyBoneId.FullBody_RightHandIndexProximal,
    "人指１.R": FullBodyBoneId.FullBody_RightHandIndexIntermediate,
    "人指２.R": FullBodyBoneId.FullBody_RightHandIndexDistal,
    "中指０.R": FullBodyBoneId.FullBody_RightHandMiddleProximal,
    "中指１.R": FullBodyBoneId.FullBody_RightHandMiddleIntermediate,
    "中指２.R": FullBodyBoneId.FullBody_RightHandMiddleDistal,
    "薬指０.R": FullBodyBoneId.FullBody_RightHandRingProximal,
    "薬指１.R": FullBodyBoneId.FullBody_RightHandRingIntermediate,
    "薬指２.R": FullBodyBoneId.FullBody_RightHandRingDistal,
    "小指０.R": FullBodyBoneId.FullBody_RightHandLittleProximal,
    "小指１.R": FullBodyBoneId.FullBody_RightHandLittleIntermediate,
    "小指２.R": FullBodyBoneId.FullBody_RightHandLittleDistal,
    # Left leg
    "足.L": FullBodyBoneId.FullBody_LeftUpperLeg,
    "ひざ.L": FullBodyBoneId.FullBody_LeftLowerLeg,
    "足首.L": FullBodyBoneId.FullBody_LeftFootAnkle,
    # Right leg
    "足.R": FullBodyBoneId.FullBody_RightUpperLeg,
    "ひざ.R": FullBodyBoneId.FullBody_RightLowerLeg,
    "足首.R": FullBodyBoneId.FullBody_RightFootAnkle,
    # IK bones - map toe IK bones for subtalar calculation
    "足ＩＫ.L": None,
    "つま先ＩＫ.L": IKMarker.TOE_IK_LEFT,  # Special marker for calculation
    "足ＩＫ先.L": None,
    "足ＩＫ.R": None,
    "つま先ＩＫ.R": IKMarker.TOE_IK_RIGHT,  # Special marker for calculation
    "足ＩＫ先.R": None,
}

# Alternative mapping for TODO
# TODO: write explanation (it's for SEW-Mimic proj)
VMD_TO_FULLBODY_MAPPING_MEDIAPIPE_CONVENTION = {
    # Left hand - TODO
    "親指２.L": FullBodyBoneId.FullBody_LeftHandThumbTip,
    "人指２.L": FullBodyBoneId.FullBody_LeftHandIndexTip,
    "中指２.L": FullBodyBoneId.FullBody_LeftHandMiddleTip,
    "薬指２.L": FullBodyBoneId.FullBody_LeftHandRingTip,
    "小指２.L": FullBodyBoneId.FullBody_LeftHandLittleTip,
    # Right hand - TODO
    "親指２.R": FullBodyBoneId.FullBody_RightHandThumbTip,
    "人指２.R": FullBodyBoneId.FullBody_RightHandIndexTip,
    "中指２.R": FullBodyBoneId.FullBody_RightHandMiddleTip,
    "薬指２.R": FullBodyBoneId.FullBody_RightHandRingTip,
    "小指２.R": FullBodyBoneId.FullBody_RightHandLittleTip,
}

# Alternative mapping for tip convention (hand bones only)
# In tip convention, joints are named after the bone segment they terminate (tip of bone)
# rather than the bone segment they start (root of bone)
VMD_TO_FULLBODY_MAPPING_TIP_CONVENTION = {
    # Left hand - tip convention (shift mapping down by one level)
    "親指０.L": None,  # Skip metacarpal in VMD tip convention
    "親指１.L": FullBodyBoneId.FullBody_LeftHandThumbMetacarpal,
    "親指２.L": FullBodyBoneId.FullBody_LeftHandThumbProximal,
    "人指０.L": FullBodyBoneId.FullBody_LeftHandIndexMetacarpal,
    "人指１.L": FullBodyBoneId.FullBody_LeftHandIndexProximal,
    "人指２.L": FullBodyBoneId.FullBody_LeftHandIndexIntermediate,
    "中指０.L": FullBodyBoneId.FullBody_LeftHandMiddleMetacarpal,
    "中指１.L": FullBodyBoneId.FullBody_LeftHandMiddleProximal,
    "中指２.L": FullBodyBoneId.FullBody_LeftHandMiddleIntermediate,
    "薬指０.L": FullBodyBoneId.FullBody_LeftHandRingMetacarpal,
    "薬指１.L": FullBodyBoneId.FullBody_LeftHandRingProximal,
    "薬指２.L": FullBodyBoneId.FullBody_LeftHandRingIntermediate,
    "小指０.L": FullBodyBoneId.FullBody_LeftHandLittleMetacarpal,
    "小指１.L": FullBodyBoneId.FullBody_LeftHandLittleProximal,
    "小指２.L": FullBodyBoneId.FullBody_LeftHandLittleIntermediate,
    # Right hand - tip convention (shift mapping down by one level)
    "親指０.R": None,  # Skip metacarpal in VMD tip convention
    "親指１.R": FullBodyBoneId.FullBody_RightHandThumbMetacarpal,
    "親指２.R": FullBodyBoneId.FullBody_RightHandThumbProximal,
    "人指０.R": FullBodyBoneId.FullBody_RightHandIndexMetacarpal,
    "人指１.R": FullBodyBoneId.FullBody_RightHandIndexProximal,
    "人指２.R": FullBodyBoneId.FullBody_RightHandIndexIntermediate,
    "中指０.R": FullBodyBoneId.FullBody_RightHandMiddleMetacarpal,
    "中指１.R": FullBodyBoneId.FullBody_RightHandMiddleProximal,
    "中指２.R": FullBodyBoneId.FullBody_RightHandMiddleIntermediate,
    "薬指０.R": FullBodyBoneId.FullBody_RightHandRingMetacarpal,
    "薬指１.R": FullBodyBoneId.FullBody_RightHandRingProximal,
    "薬指２.R": FullBodyBoneId.FullBody_RightHandRingIntermediate,
    "小指０.R": FullBodyBoneId.FullBody_RightHandLittleMetacarpal,
    "小指１.R": FullBodyBoneId.FullBody_RightHandLittleProximal,
    "小指２.R": FullBodyBoneId.FullBody_RightHandLittleIntermediate,
}

# Alternative mapping for tip extension bones (bones with "_tip" suffix)
# Maps bones that have been extended with tip markers to their corresponding OpenXR bones
VMD_TO_FULLBODY_MAPPING_TIP_EXTENSION = {
    # Main body tip extensions
    "頭_tip": None,  # Head tip - no direct OpenXR equivalent
    "足首.L_tip": FullBodyBoneId.FullBody_LeftFootBall,  # Left ankle tip -> foot ball
    "足首.R_tip": FullBodyBoneId.FullBody_RightFootBall,  # Right ankle tip -> foot ball

    # Left hand tip extensions - map to OpenXR tip bones
    "小指２.L_tip": FullBodyBoneId.FullBody_LeftHandLittleTip,
    "薬指２.L_tip": FullBodyBoneId.FullBody_LeftHandRingTip,
    "中指２.L_tip": FullBodyBoneId.FullBody_LeftHandMiddleTip,
    "人指２.L_tip": FullBodyBoneId.FullBody_LeftHandIndexTip,
    "親指２.L_tip": FullBodyBoneId.FullBody_LeftHandThumbTip,

    # Right hand tip extensions - map to OpenXR tip bones
    "小指２.R_tip": FullBodyBoneId.FullBody_RightHandLittleTip,
    "薬指２.R_tip": FullBodyBoneId.FullBody_RightHandRingTip,
    "中指２.R_tip": FullBodyBoneId.FullBody_RightHandMiddleTip,
    "人指２.R_tip": FullBodyBoneId.FullBody_RightHandIndexTip,
    "親指２.R_tip": FullBodyBoneId.FullBody_RightHandThumbTip,

    # IK bone tip extensions - map toe IK tips for subtalar calculation
    "つま先ＩＫ.L_tip": IKMarker.TOE_IK_LEFT_TIP,  # Toe IK tip for calculation
    "足ＩＫ先.L_tip": None,    # Foot IK tip - no OpenXR equivalent
    "つま先ＩＫ.R_tip": IKMarker.TOE_IK_RIGHT_TIP,  # Toe IK tip for calculation
    "足ＩＫ先.R_tip": None,    # Foot IK tip - no OpenXR equivalent
}


def get_active_mapping(convention: str) -> dict[str, FullBodyBoneId | None]:
    """
    Get the active bone mapping based on convention choice.

    Args:
        convention: Convention to use - 'root', 'tip', or 'mediapipe'

    Returns:
        Dictionary mapping VMD bone names to FullBodyBoneId
    """
    if convention == "tip":
        # Start with standard mapping and update with tip convention overrides
        mapping = VMD_TO_FULLBODY_MAPPING.copy()
        mapping.update(VMD_TO_FULLBODY_MAPPING_TIP_CONVENTION)
        mapping.update(VMD_TO_FULLBODY_MAPPING_TIP_EXTENSION)
        return mapping
    elif convention == "mediapipe":
        # Start with standard mapping and update with mediapipe convention overrides
        mapping = VMD_TO_FULLBODY_MAPPING.copy()
        mapping.update(VMD_TO_FULLBODY_MAPPING_MEDIAPIPE_CONVENTION)
        mapping.update(VMD_TO_FULLBODY_MAPPING_TIP_EXTENSION)
        return mapping
    else:  # convention == "root"
        # Include tip extensions for all conventions
        mapping = VMD_TO_FULLBODY_MAPPING.copy()
        mapping.update(VMD_TO_FULLBODY_MAPPING_TIP_EXTENSION)
        return mapping


class BoneData:
    """Represents bone position and rotation data."""

    def __init__(self, bone_id: FullBodyBoneId, position: tuple[float, float, float]):
        self.id = bone_id
        self.position = position


def interpolate_spine_lower(frame_bones: list[BoneData]) -> BoneData | None:
    """
    Create FullBody_SpineLower by interpolating midpoint between FullBody_SpineMiddle and FullBody_Hips.

    Args:
        frame_bones: List of BoneData for a single frame

    Returns:
        BoneData for interpolated SpineLower joint, or None if required bones not found
    """
    spine_middle = None
    hips = None

    for bone in frame_bones:
        if bone.id == FullBodyBoneId.FullBody_SpineMiddle:
            spine_middle = bone
        elif bone.id == FullBodyBoneId.FullBody_Hips:
            hips = bone

    if spine_middle is None or hips is None:
        return None

    # Interpolate midpoint
    mid_x = (spine_middle.position[0] + hips.position[0]) / 2.0
    mid_y = (spine_middle.position[1] + hips.position[1]) / 2.0
    mid_z = (spine_middle.position[2] + hips.position[2]) / 2.0

    return BoneData(FullBodyBoneId.FullBody_SpineLower, (mid_x, mid_y, mid_z))


def calculate_fake_subtalar_bones(frame_bones: list[BoneData]) -> list[BoneData]:
    """
    Calculate fake subtalar bones using the vector from つま先ＩＫ to つま先ＩＫ.R_tip added to the ankle position.

    Args:
        frame_bones: List of BoneData for a single frame

    Returns:
        List of BoneData for calculated subtalar bones
    """
    subtalar_bones = []

    # Build position lookup including special markers
    bone_positions = {}
    for bone in frame_bones:
        bone_positions[bone.id] = bone.position

    # Calculate left subtalar bone
    left_ankle = bone_positions.get(FullBodyBoneId.FullBody_LeftFootAnkle)
    toe_ik_left = bone_positions.get(IKMarker.TOE_IK_LEFT)
    toe_ik_left_tip = bone_positions.get(IKMarker.TOE_IK_LEFT_TIP)

    if left_ankle and toe_ik_left and toe_ik_left_tip:
        # Vector from つま先ＩＫ.L to つま先ＩＫ.L_tip
        toe_vector = (
            toe_ik_left_tip[0] - toe_ik_left[0],
            toe_ik_left_tip[1] - toe_ik_left[1],
            toe_ik_left_tip[2] - toe_ik_left[2]
        )

        # Add this vector to the ankle position
        subtalar_pos = (
            left_ankle[0] + toe_vector[0],
            left_ankle[1] + toe_vector[1],
            left_ankle[2] + toe_vector[2]
        )

        subtalar_bones.append(BoneData(FullBodyBoneId.FullBody_LeftFootSubtalar, subtalar_pos))

    # Calculate right subtalar bone
    right_ankle = bone_positions.get(FullBodyBoneId.FullBody_RightFootAnkle)
    toe_ik_right = bone_positions.get(IKMarker.TOE_IK_RIGHT)
    toe_ik_right_tip = bone_positions.get(IKMarker.TOE_IK_RIGHT_TIP)

    if right_ankle and toe_ik_right and toe_ik_right_tip:
        # Vector from つま先ＩＫ.R to つま先ＩＫ.R_tip
        toe_vector = (
            toe_ik_right_tip[0] - toe_ik_right[0],
            toe_ik_right_tip[1] - toe_ik_right[1],
            toe_ik_right_tip[2] - toe_ik_right[2]
        )

        # Add this vector to the ankle position
        subtalar_pos = (
            right_ankle[0] + toe_vector[0],
            right_ankle[1] + toe_vector[1],
            right_ankle[2] + toe_vector[2]
        )

        subtalar_bones.append(BoneData(FullBodyBoneId.FullBody_RightFootSubtalar, subtalar_pos))

    return subtalar_bones


def parse_csv_data(
    csv_file: str, convention: str = "root", interpolate_spine: bool = False, calculate_subtalar: bool = True
) -> dict[int, list[BoneData]]:
    """
    Parse CSV data and return frame-indexed bone data.

    Args:
        csv_file: Path to the CSV file
        convention: Convention to use - 'root', 'tip', or 'mediapipe'
        interpolate_spine: Whether to interpolate FullBody_SpineLower from FullBody_SpineMiddle and FullBody_Hips
        calculate_subtalar: Whether to calculate fake subtalar bones from toe IK data

    Returns:
        Dictionary mapping frame numbers to lists of BoneData
    """
    frame_data = {}
    active_mapping = get_active_mapping(convention)

    try:
        with open(csv_file) as f:
            reader = csv.DictReader(f)
            for row in reader:
                frame = int(row["frame"])
                bone_name = row["bone_name"]

                # Skip unmapped bones or bones mapped to None
                if bone_name not in active_mapping or active_mapping[bone_name] is None:
                    continue

                bone_id = active_mapping[bone_name]

                # Parse position (VMD data appears to be in meters already)
                # VMD (exported via Blender) uses Z-up coordinate system
                loc_x = float(row["loc_x"])
                loc_y = float(row["loc_y"])
                loc_z = float(row["loc_z"])

                position = (loc_x, loc_y, loc_z)
                bone_data = BoneData(bone_id, position)

                if frame not in frame_data:
                    frame_data[frame] = []
                frame_data[frame].append(bone_data)

    except FileNotFoundError:
        print(f"Error: Could not find file {csv_file}")
        return {}
    except Exception as e:
        print(f"Error parsing CSV file: {e}")
        return {}

    # Add interpolated SpineLower joints if requested
    if interpolate_spine:
        for bones in frame_data.values():
            spine_lower = interpolate_spine_lower(bones)
            if spine_lower is not None:
                bones.append(spine_lower)

    # Add calculated subtalar bones if requested
    if calculate_subtalar:
        subtalar_count = 0
        for bones in frame_data.values():
            subtalar_bones = calculate_fake_subtalar_bones(bones)
            bones.extend(subtalar_bones)
            subtalar_count += len(subtalar_bones)
        if subtalar_count > 0:
            print(f"Calculated {subtalar_count} fake subtalar bones across all frames")

    return frame_data


def build_skeleton_tree():
    """Build a tree structure from skeleton connections for traversal."""
    children = {}  # parent_id -> [child_ids]
    for parent, child in FULL_BODY_SKELETON_CONNECTIONS:
        if parent not in children:
            children[parent] = []
        children[parent].append(child)
    return children


def calculate_foot_normal_vectors(frame_bones: list[BoneData]) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """
    Calculate foot normal vectors for left and right feet.

    The foot normal is calculated as the cross product of:
    - Vector from subtalar to ankle
    - Vector from subtalar to foot ball

    Args:
        frame_bones: List of BoneData for a single frame

    Returns:
        List of tuples containing (position, direction) for each foot normal vector
    """
    normals = []

    # Find required bones
    bone_positions = {bone.id: bone.position for bone in frame_bones}

    # Calculate left foot normal
    left_subtalar = bone_positions.get(FullBodyBoneId.FullBody_LeftFootSubtalar)
    left_ankle = bone_positions.get(FullBodyBoneId.FullBody_LeftFootAnkle)
    left_foot_ball = bone_positions.get(FullBodyBoneId.FullBody_LeftFootBall)

    if left_subtalar and left_ankle and left_foot_ball:
        # Vector from subtalar to ankle
        subtalar_to_ankle = (
            left_ankle[0] - left_subtalar[0],
            left_ankle[1] - left_subtalar[1],
            left_ankle[2] - left_subtalar[2]
        )

        # Vector from subtalar to foot ball
        subtalar_to_ball = (
            left_foot_ball[0] - left_subtalar[0],
            left_foot_ball[1] - left_subtalar[1],
            left_foot_ball[2] - left_subtalar[2]
        )

        # Cross product (subtalar_to_ankle × subtalar_to_ball)
        normal = (
            subtalar_to_ankle[1] * subtalar_to_ball[2] - subtalar_to_ankle[2] * subtalar_to_ball[1],
            subtalar_to_ankle[2] * subtalar_to_ball[0] - subtalar_to_ankle[0] * subtalar_to_ball[2],
            subtalar_to_ankle[0] * subtalar_to_ball[1] - subtalar_to_ankle[1] * subtalar_to_ball[0]
        )

        # Normalize the vector
        length = (normal[0]**2 + normal[1]**2 + normal[2]**2)**0.5
        if length > 0:
            normal = (normal[0]/length, normal[1]/length, normal[2]/length)
            normals.append((left_subtalar, normal))

    # Calculate right foot normal
    right_subtalar = bone_positions.get(FullBodyBoneId.FullBody_RightFootSubtalar)
    right_ankle = bone_positions.get(FullBodyBoneId.FullBody_RightFootAnkle)
    right_foot_ball = bone_positions.get(FullBodyBoneId.FullBody_RightFootBall)

    if right_subtalar and right_ankle and right_foot_ball:
        # Vector from subtalar to ankle
        subtalar_to_ankle = (
            right_ankle[0] - right_subtalar[0],
            right_ankle[1] - right_subtalar[1],
            right_ankle[2] - right_subtalar[2]
        )

        # Vector from subtalar to foot ball
        subtalar_to_ball = (
            right_foot_ball[0] - right_subtalar[0],
            right_foot_ball[1] - right_subtalar[1],
            right_foot_ball[2] - right_subtalar[2]
        )

        # Cross product (subtalar_to_ankle × subtalar_to_ball)
        normal = (
            subtalar_to_ankle[1] * subtalar_to_ball[2] - subtalar_to_ankle[2] * subtalar_to_ball[1],
            subtalar_to_ankle[2] * subtalar_to_ball[0] - subtalar_to_ankle[0] * subtalar_to_ball[2],
            subtalar_to_ankle[0] * subtalar_to_ball[1] - subtalar_to_ankle[1] * subtalar_to_ball[0]
        )

        # Normalize the vector
        length = (normal[0]**2 + normal[1]**2 + normal[2]**2)**0.5
        if length > 0:
            normal = (normal[0]/length, normal[1]/length, normal[2]/length)
            normals.append((right_subtalar, normal))

    return normals


def calculate_foot_plane_meshes(frame_bones: list[BoneData], strategy="tip", offset=0.01) -> list[tuple[list, list]]:
    """
    Calculate foot plane meshes for left and right feet.

    Creates rectangular planes defined by the two vectors:
    - Vector from subtalar to foot ball
    - Cross product normal vector (orthogonal to subtalar-to-ankle and subtalar-to-ball)

    Args:
        frame_bones: List of BoneData for a single frame
        strategy: Positioning strategy - "tip" or "center"
        offset: Small offset (in meters) to apply to all vertices for easier mouse interaction

    Returns:
        List of tuples containing (vertices, triangles) for each foot plane mesh
    """
    meshes = []

    # Find required bones
    bone_positions = {bone.id: bone.position for bone in frame_bones}

    # Calculate left foot plane
    left_subtalar = bone_positions.get(FullBodyBoneId.FullBody_LeftFootSubtalar)
    left_ankle = bone_positions.get(FullBodyBoneId.FullBody_LeftFootAnkle)
    left_foot_ball = bone_positions.get(FullBodyBoneId.FullBody_LeftFootBall)

    if left_subtalar and left_ankle and left_foot_ball:
        # Vector from subtalar to ankle
        subtalar_to_ankle = (
            left_ankle[0] - left_subtalar[0],
            left_ankle[1] - left_subtalar[1],
            left_ankle[2] - left_subtalar[2]
        )

        # Vector from subtalar to foot ball
        subtalar_to_ball = (
            left_foot_ball[0] - left_subtalar[0],
            left_foot_ball[1] - left_subtalar[1],
            left_foot_ball[2] - left_subtalar[2]
        )

        # Cross product (subtalar_to_ankle × subtalar_to_ball) for normal vector
        normal = (
            subtalar_to_ankle[1] * subtalar_to_ball[2] - subtalar_to_ankle[2] * subtalar_to_ball[1],
            subtalar_to_ankle[2] * subtalar_to_ball[0] - subtalar_to_ankle[0] * subtalar_to_ball[2],
            subtalar_to_ankle[0] * subtalar_to_ball[1] - subtalar_to_ankle[1] * subtalar_to_ball[0]
        )

        # Normalize the normal vector
        normal_length = (normal[0]**2 + normal[1]**2 + normal[2]**2)**0.5
        if normal_length > 0:
            # normal = (normal[0]/normal_length, normal[1]/normal_length, normal[2]/normal_length)
            desired_normal_length = 0.05
            division_factor = normal_length / desired_normal_length
            normal = (normal[0]/division_factor, normal[1]/division_factor, normal[2]/division_factor)

            # Scale vectors for better plane visualization
            # scale_factor = 1.0
            scale_factor = 0.8
            scaled_ball = tuple(v * scale_factor for v in subtalar_to_ball)
            scaled_normal = tuple(v * scale_factor for v in normal)

            # Normalized vectors for offset direction
            normal_unit = (normal[0]/normal_length, normal[1]/normal_length, normal[2]/normal_length)
            ball_length = (subtalar_to_ball[0]**2 + subtalar_to_ball[1]**2 + subtalar_to_ball[2]**2)**0.5
            if ball_length > 0:
                ball_unit = (subtalar_to_ball[0]/ball_length, subtalar_to_ball[1]/ball_length, subtalar_to_ball[2]/ball_length)
            else:
                ball_unit = (0, 0, 0)

            # Apply offset in both normal and ball directions
            offset_normal_x = normal_unit[0] * offset
            offset_normal_y = normal_unit[1] * offset
            offset_normal_z = normal_unit[2] * offset
            offset_ball_x = ball_unit[0] * offset
            offset_ball_y = ball_unit[1] * offset
            offset_ball_z = ball_unit[2] * offset

            # Create rectangular plane vertices centered at ankle
            if strategy == "center":
                vertices = [
                    # Bottom-left
                    (left_subtalar[0] - scaled_ball[0]/2 - scaled_normal[0]/2 + offset_normal_x + offset_ball_x,
                    left_subtalar[1] - scaled_ball[1]/2 - scaled_normal[1]/2 + offset_normal_y + offset_ball_y,
                    left_subtalar[2] - scaled_ball[2]/2 - scaled_normal[2]/2 + offset_normal_z + offset_ball_z),
                    # Bottom-right
                    (left_subtalar[0] + scaled_ball[0]/2 - scaled_normal[0]/2 + offset_normal_x + offset_ball_x,
                    left_subtalar[1] + scaled_ball[1]/2 - scaled_normal[1]/2 + offset_normal_y + offset_ball_y,
                    left_subtalar[2] + scaled_ball[2]/2 - scaled_normal[2]/2 + offset_normal_z + offset_ball_z),
                    # Top-right
                    (left_subtalar[0] + scaled_ball[0]/2 + scaled_normal[0]/2 + offset_normal_x + offset_ball_x,
                    left_subtalar[1] + scaled_ball[1]/2 + scaled_normal[1]/2 + offset_normal_y + offset_ball_y,
                    left_subtalar[2] + scaled_ball[2]/2 + scaled_normal[2]/2 + offset_normal_z + offset_ball_z),
                    # Top-left
                    (left_subtalar[0] - scaled_ball[0]/2 + scaled_normal[0]/2 + offset_normal_x + offset_ball_x,
                    left_subtalar[1] - scaled_ball[1]/2 + scaled_normal[1]/2 + offset_normal_y + offset_ball_y,
                    left_subtalar[2] - scaled_ball[2]/2 + scaled_normal[2]/2 + offset_normal_z + offset_ball_z),
                ]
            elif strategy == "tip":
                vertices = [
                    # Bottom-left
                    (left_subtalar[0] + offset_normal_x + offset_ball_x,
                    left_subtalar[1] + offset_normal_y + offset_ball_y,
                    left_subtalar[2] + offset_normal_z + offset_ball_z),
                    # Bottom-right
                    (left_subtalar[0] + scaled_ball[0] + offset_normal_x + offset_ball_x,
                    left_subtalar[1] + scaled_ball[1] + offset_normal_y + offset_ball_y,
                    left_subtalar[2] + scaled_ball[2] + offset_normal_z + offset_ball_z),
                    # Top-right
                    (left_subtalar[0] + scaled_ball[0] + scaled_normal[0] + offset_normal_x + offset_ball_x,
                    left_subtalar[1] + scaled_ball[1] + scaled_normal[1] + offset_normal_y + offset_ball_y,
                    left_subtalar[2] + scaled_ball[2] + scaled_normal[2] + offset_normal_z + offset_ball_z),
                    # Top-left
                    (left_subtalar[0] + scaled_normal[0] + offset_normal_x + offset_ball_x,
                    left_subtalar[1] + scaled_normal[1] + offset_normal_y + offset_ball_y,
                    left_subtalar[2] + scaled_normal[2] + offset_normal_z + offset_ball_z),
                ]

            # Two triangles forming a rectangle
            triangles = [
                [0, 1, 2],  # First triangle
                [0, 2, 3],  # Second triangle
            ]

            meshes.append((vertices, triangles))

    # Calculate right foot plane
    right_subtalar = bone_positions.get(FullBodyBoneId.FullBody_RightFootSubtalar)
    right_ankle = bone_positions.get(FullBodyBoneId.FullBody_RightFootAnkle)
    right_foot_ball = bone_positions.get(FullBodyBoneId.FullBody_RightFootBall)

    if right_subtalar and right_ankle and right_foot_ball:
        # Vector from subtalar to ankle
        subtalar_to_ankle = (
            right_ankle[0] - right_subtalar[0],
            right_ankle[1] - right_subtalar[1],
            right_ankle[2] - right_subtalar[2]
        )

        # Vector from subtalar to foot ball
        subtalar_to_ball = (
            right_foot_ball[0] - right_subtalar[0],
            right_foot_ball[1] - right_subtalar[1],
            right_foot_ball[2] - right_subtalar[2]
        )

        # Cross product (subtalar_to_ankle × subtalar_to_ball) for normal vector
        normal = (
            subtalar_to_ankle[1] * subtalar_to_ball[2] - subtalar_to_ankle[2] * subtalar_to_ball[1],
            subtalar_to_ankle[2] * subtalar_to_ball[0] - subtalar_to_ankle[0] * subtalar_to_ball[2],
            subtalar_to_ankle[0] * subtalar_to_ball[1] - subtalar_to_ankle[1] * subtalar_to_ball[0]
        )

        # Normalize the normal vector
        normal_length = (normal[0]**2 + normal[1]**2 + normal[2]**2)**0.5
        if normal_length > 0:
            # normal = (normal[0]/normal_length, normal[1]/normal_length, normal[2]/normal_length)
            desired_normal_length = 0.05
            division_factor = normal_length / desired_normal_length
            normal = (normal[0]/division_factor, normal[1]/division_factor, normal[2]/division_factor)

            # Scale vectors for better plane visualization
            # scale_factor = 1.0
            scale_factor = 0.8
            scaled_ball = tuple(v * scale_factor for v in subtalar_to_ball)
            scaled_normal = tuple(v * scale_factor for v in normal)

            # Normalized vectors for offset direction
            normal_unit = (normal[0]/normal_length, normal[1]/normal_length, normal[2]/normal_length)
            ball_length = (subtalar_to_ball[0]**2 + subtalar_to_ball[1]**2 + subtalar_to_ball[2]**2)**0.5
            if ball_length > 0:
                ball_unit = (subtalar_to_ball[0]/ball_length, subtalar_to_ball[1]/ball_length, subtalar_to_ball[2]/ball_length)
            else:
                ball_unit = (0, 0, 0)

            # Apply offset in both normal and ball directions
            offset_normal_x = normal_unit[0] * offset
            offset_normal_y = normal_unit[1] * offset
            offset_normal_z = normal_unit[2] * offset
            offset_ball_x = ball_unit[0] * offset
            offset_ball_y = ball_unit[1] * offset
            offset_ball_z = ball_unit[2] * offset

            # Create rectangular plane vertices centered at ankle
            if strategy == "center":
                vertices = [
                    # Bottom-left
                    (right_subtalar[0] - scaled_ball[0]/2 - scaled_normal[0]/2 + offset_normal_x + offset_ball_x,
                     right_subtalar[1] - scaled_ball[1]/2 - scaled_normal[1]/2 + offset_normal_y + offset_ball_y,
                     right_subtalar[2] - scaled_ball[2]/2 - scaled_normal[2]/2 + offset_normal_z + offset_ball_z),
                    # Bottom-right
                    (right_subtalar[0] + scaled_ball[0]/2 - scaled_normal[0]/2 + offset_normal_x + offset_ball_x,
                     right_subtalar[1] + scaled_ball[1]/2 - scaled_normal[1]/2 + offset_normal_y + offset_ball_y,
                     right_subtalar[2] + scaled_ball[2]/2 - scaled_normal[2]/2 + offset_normal_z + offset_ball_z),
                    # Top-right
                    (right_subtalar[0] + scaled_ball[0]/2 + scaled_normal[0]/2 + offset_normal_x + offset_ball_x,
                     right_subtalar[1] + scaled_ball[1]/2 + scaled_normal[1]/2 + offset_normal_y + offset_ball_y,
                     right_subtalar[2] + scaled_ball[2]/2 + scaled_normal[2]/2 + offset_normal_z + offset_ball_z),
                    # Top-left
                    (right_subtalar[0] - scaled_ball[0]/2 + scaled_normal[0]/2 + offset_normal_x + offset_ball_x,
                     right_subtalar[1] - scaled_ball[1]/2 + scaled_normal[1]/2 + offset_normal_y + offset_ball_y,
                     right_subtalar[2] - scaled_ball[2]/2 + scaled_normal[2]/2 + offset_normal_z + offset_ball_z),
                ]
            elif strategy == "tip":
                vertices = [
                    # Bottom-left
                    (right_subtalar[0] + offset_normal_x + offset_ball_x,
                     right_subtalar[1] + offset_normal_y + offset_ball_y,
                     right_subtalar[2] + offset_normal_z + offset_ball_z),
                    # Bottom-right
                    (right_subtalar[0] + scaled_ball[0] + offset_normal_x + offset_ball_x,
                     right_subtalar[1] + scaled_ball[1] + offset_normal_y + offset_ball_y,
                     right_subtalar[2] + scaled_ball[2] + offset_normal_z + offset_ball_z),
                    # Top-right
                    (right_subtalar[0] + scaled_ball[0] + scaled_normal[0] + offset_normal_x + offset_ball_x,
                     right_subtalar[1] + scaled_ball[1] + scaled_normal[1] + offset_normal_y + offset_ball_y,
                     right_subtalar[2] + scaled_ball[2] + scaled_normal[2] + offset_normal_z + offset_ball_z),
                    # Top-left
                    (right_subtalar[0] + scaled_normal[0] + offset_normal_x + offset_ball_x,
                     right_subtalar[1] + scaled_normal[1] + offset_normal_y + offset_ball_y,
                     right_subtalar[2] + scaled_normal[2] + offset_normal_z + offset_ball_z),
                ]

            # Two triangles forming a rectangle
            triangles = [
                [0, 1, 2],  # First triangle
                [0, 2, 3],  # Second triangle
            ]

            meshes.append((vertices, triangles))

    return meshes


def find_recovered_connections(available_bones: set[FullBodyBoneId]):
    """
    Find recovered connections that bypass missing bones.

    Returns:
        List of (parent, child, is_recovered) tuples where is_recovered indicates
        if this connection bypasses missing intermediate bones.
    """
    children_map = build_skeleton_tree()
    connections = []

    def find_available_descendants(
        bone_id: FullBodyBoneId, visited: set = None
    ) -> list[FullBodyBoneId]:
        """Recursively find all available descendant bones."""
        if visited is None:
            visited = set()

        if bone_id in visited:
            return []

        visited.add(bone_id)
        descendants = []

        if bone_id in children_map:
            for child in children_map[bone_id]:
                if child in available_bones:
                    descendants.append(child)
                else:
                    # Child is missing, look deeper
                    descendants.extend(find_available_descendants(child, visited))

        return descendants

    # Process each original connection
    for parent, child in FULL_BODY_SKELETON_CONNECTIONS:
        parent_available = parent in available_bones
        child_available = child in available_bones

        if parent_available and child_available:
            # Original connection is intact
            connections.append((parent, child, False))
        elif parent_available and not child_available:
            # Direct child is missing, find available descendants
            descendants = find_available_descendants(child)
            for descendant in descendants:
                connections.append((parent, descendant, True))

    return connections


def visualize_frame(rr, frame_data: list[BoneData], frame_number: int, show_recovered: bool = True, show_foot_normals: bool = False):
    """Visualize a single frame of bone data with separate original and recovered connections."""
    if not frame_data:
        return

    # Get available bones from the frame data (only enum IDs, skip string markers)
    available_bones = {bone.id: bone for bone in frame_data if isinstance(bone.id, FullBodyBoneId)}

    # Get original connections (both bones available)
    original_connections = []
    for parent, child in FULL_BODY_SKELETON_CONNECTIONS:
        if parent in available_bones and child in available_bones:
            original_connections.append((parent, child))

    # Get recovered connections (bypass missing bones)
    recovered_connections = []
    if show_recovered:
        recovered_connections = find_recovered_connections(available_bones)
        # Filter out the original connections to keep only the recovered ones
        original_set = set(original_connections)
        recovered_connections = [
            (p, c)
            for p, c, is_recovered in recovered_connections
            if is_recovered and (p, c) not in original_set
        ]

    print(
        f"Frame {frame_number}: {len(original_connections)} original + {len(recovered_connections)} recovered connections"
    )

    positions = []
    keypoint_ids = []

    for bone in frame_data:
        # Skip IK markers used for calculation - only visualize FullBodyBoneId
        if not isinstance(bone.id, FullBodyBoneId):
            continue
        positions.append(bone.position)
        keypoint_ids.append(bone.id.value)

    # Set timestamp for this frame
    rr.set_time_sequence("frame", frame_number)

    # Log bone points
    rr.log(
        "world/user/bones",
        rr.Points3D(
            positions=positions,
            keypoint_ids=keypoint_ids,
            class_ids=SkeletonType.FullBody.value,
            radii=0.01,
        ),
    )

    # Create position lookup for line drawing (only for enum IDs, skip string markers)
    bone_positions = {bone.id: bone.position for bone in frame_data if isinstance(bone.id, FullBodyBoneId)}

    # Log original connections (green lines)
    if original_connections:
        original_lines = []
        for parent, child in original_connections:
            if parent in bone_positions and child in bone_positions:
                original_lines.append([bone_positions[parent], bone_positions[child]])

        if original_lines:
            # rr.log(
            #     "world/user/skeleton_original",
            #     rr.LineStrips3D(
            #         strips=original_lines,
            #         colors=[[0, 200, 0, 255]],  # Green for original connections
            #         radii=[0.005],
            #     ),
            # )
            pass  # HACK
            # NOTE: Because the bone connection is already visualized, it should not need a
            #       separate, redundant line segment visualization.

    # Log recovered connections (light blue lines)
    if show_recovered and recovered_connections:
        recovered_lines = []
        for parent, child in recovered_connections:
            if parent in bone_positions and child in bone_positions:
                recovered_lines.append([bone_positions[parent], bone_positions[child]])

        if recovered_lines:
            rr.log(
                "world/user/skeleton_recovered",
                rr.LineStrips3D(
                    strips=recovered_lines,
                    colors=[
                        [135, 206, 250, 255]
                    ],  # Light blue for recovered connections, slightly transparent
                    radii=[0.001],
                ),
            )

    # Log foot normal vectors (red arrows)
    if show_foot_normals:
        foot_normals = calculate_foot_normal_vectors(frame_data)
        if foot_normals:
            arrow_lines = []
            arrow_length = 0.05  # 5 cm vectors

            for position, direction in foot_normals:
                end_point = (
                    position[0] + direction[0] * arrow_length,
                    position[1] + direction[1] * arrow_length,
                    position[2] + direction[2] * arrow_length
                )
                arrow_lines.append([position, end_point])

            if arrow_lines:
                rr.log(
                    "world/user/foot_normals",
                    rr.LineStrips3D(
                        strips=arrow_lines,
                        colors=[[255, 90, 20, 200]],  # Red for foot normal vectors
                        radii=[0.001],
                    ),
                )

        # Log foot plane meshes
        foot_planes = calculate_foot_plane_meshes(frame_data)
        for i, (vertices, triangles) in enumerate(foot_planes):
            rr.log(
                f"world/user/foot_plane_{i}",
                rr.Mesh3D(
                    vertex_positions=vertices,
                    triangle_indices=triangles,
                    vertex_colors=[[0, 250, 150, 100]]*4,
                ),
            )


def main():
    parser = argparse.ArgumentParser(description="Visualize VMD-exported body pose data")
    parser.add_argument(
        "--file", type=str, required=True, help="Path to the CSV file containing pose data"
    )
    parser.add_argument(
        "--frame", type=int, help="Specific frame to visualize (if not provided, shows frame 1)"
    )
    parser.add_argument("--animate", action="store_true", help="Animate through all frames")
    parser.add_argument(
        "--fps", type=float, default=30.0, help="Frames per second for animation (default: 30)"
    )
    parser.add_argument(
        "--show-recovered",
        action="store_true",
        default=True,
        help="Show recovered connections that bypass missing bones (default: True)",
    )
    parser.add_argument(
        "--no-recovered",
        dest="show_recovered",
        action="store_false",
        help="Hide recovered connections, show only original intact connections",
    )
    parser.add_argument(
        "--convention",
        type=str,
        default="root",
        choices=["root", "tip", "mediapipe"],
        help="Bone mapping convention: 'root' (default), 'tip' (joints named after bone tips), "
        "or 'mediapipe' (for MediaPipe compatibility)",
    )
    parser.add_argument(
        "--interpolate-spine",
        action="store_true",
        help="Create FullBody_SpineLower by interpolating midpoint between FullBody_SpineMiddle and FullBody_Hips",
    )
    parser.add_argument(
        "--show-foot-normals",
        action="store_true",
        help="Show foot normal vectors and transparent foot planes (orthogonal to lower leg and foot ball vectors)",
    )
    parser.add_argument(
        "--calculate-subtalar",
        action="store_true",
        default=True,
        help="Calculate fake subtalar bones from toe IK data (default: True)",
    )
    parser.add_argument(
        "--no-subtalar",
        dest="calculate_subtalar",
        action="store_false",
        help="Skip calculating fake subtalar bones",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        help="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
    )
    args = parser.parse_args()

    # Configure logging
    configure_logging(level=args.log_level)

    # Parse CSV data
    print(f"Loading pose data from {args.file}...")
    if args.convention != "root":
        print(f"Using {args.convention} convention for bone mapping")
    if args.interpolate_spine:
        print("Interpolating FullBody_SpineLower from FullBody_SpineMiddle and FullBody_Hips")
    if args.calculate_subtalar:
        print("Calculating fake subtalar bones from toe IK data")
    frame_data = parse_csv_data(args.file, args.convention, args.interpolate_spine, args.calculate_subtalar)

    if not frame_data:
        print("No valid pose data found!")
        return

    print(f"Loaded {len(frame_data)} frames")

    # Initialize rerun
    try:
        import rerun as rr
        from matplotlib import colormaps as cm
    except ImportError:
        print("Please install rerun SDK and matplotlib: pip install -e .[viz]")
        return

    rr.init("vmd-pose-visualizer", spawn=True)

    # Set coordinate system to right-handed, Z-up (converted from VMD Y-up data)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # Create skeleton class description
    colormap = cm.get_cmap("jet")
    keypoint_annotations = [
        rr.AnnotationInfo(
            id=member.value,
            label=member.name,
            color=(np.array(colormap(member.value / FullBodyBoneId.FullBody_End)) * 255).astype(
                np.uint8
            ),
        )
        for member in FullBodyBoneId
    ]

    rr.log(
        "/",
        rr.AnnotationContext(
            rr.ClassDescription(
                info=rr.AnnotationInfo(
                    id=SkeletonType.FullBody.value,
                    label="SkeletonType.FullBody",
                    color=np.array([251, 251, 251, 251], dtype=np.uint8),
                ),
                keypoint_annotations=keypoint_annotations,
                keypoint_connections=FULL_BODY_SKELETON_CONNECTIONS,
            )
        ),
        static=True,
    )

    # Visualize data
    if args.animate:
        print("Animating through frames... Press Ctrl+C to stop")
        try:
            sorted_frames = sorted(frame_data.keys())
            sleep_time = 1.0 / args.fps
            while True:
                for frame_num in sorted_frames:
                    visualize_frame(rr, frame_data[frame_num], frame_num, args.show_recovered, args.show_foot_normals)
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("\nAnimation stopped")
    else:
        # Show specific frame or frame 1
        target_frame = args.frame if args.frame is not None else 1
        if target_frame in frame_data:
            print(f"Visualizing frame {target_frame}")
            print(f"Connection recovery: {'enabled' if args.show_recovered else 'disabled'}")
            visualize_frame(rr, frame_data[target_frame], target_frame, args.show_recovered, args.show_foot_normals)
        else:
            print(f"Frame {target_frame} not found. Available frames: {sorted(frame_data.keys())}")


if __name__ == "__main__":
    main()
