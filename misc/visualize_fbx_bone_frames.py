"""
Visualizes bone frames (coordinate axes) from a static CSV file (exported from FBX via Blender).

This script reads bone position and rotation data from a CSV file with Mixamo bone names
and visualizes bone coordinate frames using rerun's 3D visualization. The CSV format should be:
frame,bone_name,loc_x,loc_y,loc_z,rot_w,rot_x,rot_y,rot_z,scale_x,scale_y,scale_z

Usage:
    python visualize_fbx_bone_frames.py --file path/to/animation.csv
    python visualize_fbx_bone_frames.py --file path/to/animation.csv --frame 10
    python visualize_fbx_bone_frames.py --file path/to/animation.csv --animate
"""

import argparse
import csv
import time
from typing import Literal

import numpy as np

from xr_robot_teleop_server import configure_logging
from xr_robot_teleop_server.schemas.openxr_skeletons import (
    FULL_BODY_SKELETON_CONNECTIONS,
    FullBodyBoneId,
    SkeletonType,
)

# Mixamo to FullBody bone name mapping
MIXAMO_TO_FULLBODY_MAPPING = {
    # Core body
    "mixamorig:Hips": FullBodyBoneId.FullBody_Hips,
    "mixamorig:Spine": FullBodyBoneId.FullBody_SpineLower,
    "mixamorig:Spine1": FullBodyBoneId.FullBody_SpineMiddle,
    "mixamorig:Spine2": FullBodyBoneId.FullBody_SpineUpper,
    "mixamorig:Neck": FullBodyBoneId.FullBody_Neck,
    "mixamorig:Head": FullBodyBoneId.FullBody_Head,
    # Left arm
    "mixamorig:LeftShoulder": FullBodyBoneId.FullBody_LeftShoulder,
    "mixamorig:LeftArm": FullBodyBoneId.FullBody_LeftArmUpper,
    "mixamorig:LeftForeArm": FullBodyBoneId.FullBody_LeftArmLower,
    # Right arm
    "mixamorig:RightShoulder": FullBodyBoneId.FullBody_RightShoulder,
    "mixamorig:RightArm": FullBodyBoneId.FullBody_RightArmUpper,
    "mixamorig:RightForeArm": FullBodyBoneId.FullBody_RightArmLower,
    # Left hand
    "mixamorig:LeftHand": FullBodyBoneId.FullBody_LeftHandWrist,
    "mixamorig:LeftHandThumb1": FullBodyBoneId.FullBody_LeftHandThumbMetacarpal,
    "mixamorig:LeftHandThumb2": FullBodyBoneId.FullBody_LeftHandThumbProximal,
    "mixamorig:LeftHandThumb3": FullBodyBoneId.FullBody_LeftHandThumbDistal,
    "mixamorig:LeftHandThumb4": FullBodyBoneId.FullBody_LeftHandThumbTip,
    "mixamorig:LeftHandIndex1": FullBodyBoneId.FullBody_LeftHandIndexProximal,
    "mixamorig:LeftHandIndex2": FullBodyBoneId.FullBody_LeftHandIndexIntermediate,
    "mixamorig:LeftHandIndex3": FullBodyBoneId.FullBody_LeftHandIndexDistal,
    "mixamorig:LeftHandIndex4": FullBodyBoneId.FullBody_LeftHandIndexTip,
    "mixamorig:LeftHandMiddle1": FullBodyBoneId.FullBody_LeftHandMiddleProximal,
    "mixamorig:LeftHandMiddle2": FullBodyBoneId.FullBody_LeftHandMiddleIntermediate,
    "mixamorig:LeftHandMiddle3": FullBodyBoneId.FullBody_LeftHandMiddleDistal,
    "mixamorig:LeftHandMiddle4": FullBodyBoneId.FullBody_LeftHandMiddleTip,
    "mixamorig:LeftHandRing1": FullBodyBoneId.FullBody_LeftHandRingProximal,
    "mixamorig:LeftHandRing2": FullBodyBoneId.FullBody_LeftHandRingIntermediate,
    "mixamorig:LeftHandRing3": FullBodyBoneId.FullBody_LeftHandRingDistal,
    "mixamorig:LeftHandRing4": FullBodyBoneId.FullBody_LeftHandRingTip,
    "mixamorig:LeftHandPinky1": FullBodyBoneId.FullBody_LeftHandLittleProximal,
    "mixamorig:LeftHandPinky2": FullBodyBoneId.FullBody_LeftHandLittleIntermediate,
    "mixamorig:LeftHandPinky3": FullBodyBoneId.FullBody_LeftHandLittleDistal,
    "mixamorig:LeftHandPinky4": FullBodyBoneId.FullBody_LeftHandLittleTip,
    # Right hand
    "mixamorig:RightHand": FullBodyBoneId.FullBody_RightHandWrist,
    "mixamorig:RightHandThumb1": FullBodyBoneId.FullBody_RightHandThumbMetacarpal,
    "mixamorig:RightHandThumb2": FullBodyBoneId.FullBody_RightHandThumbProximal,
    "mixamorig:RightHandThumb3": FullBodyBoneId.FullBody_RightHandThumbDistal,
    "mixamorig:RightHandThumb4": FullBodyBoneId.FullBody_RightHandThumbTip,
    "mixamorig:RightHandIndex1": FullBodyBoneId.FullBody_RightHandIndexProximal,
    "mixamorig:RightHandIndex2": FullBodyBoneId.FullBody_RightHandIndexIntermediate,
    "mixamorig:RightHandIndex3": FullBodyBoneId.FullBody_RightHandIndexDistal,
    "mixamorig:RightHandIndex4": FullBodyBoneId.FullBody_RightHandIndexTip,
    "mixamorig:RightHandMiddle1": FullBodyBoneId.FullBody_RightHandMiddleProximal,
    "mixamorig:RightHandMiddle2": FullBodyBoneId.FullBody_RightHandMiddleIntermediate,
    "mixamorig:RightHandMiddle3": FullBodyBoneId.FullBody_RightHandMiddleDistal,
    "mixamorig:RightHandMiddle4": FullBodyBoneId.FullBody_RightHandMiddleTip,
    "mixamorig:RightHandRing1": FullBodyBoneId.FullBody_RightHandRingProximal,
    "mixamorig:RightHandRing2": FullBodyBoneId.FullBody_RightHandRingIntermediate,
    "mixamorig:RightHandRing3": FullBodyBoneId.FullBody_RightHandRingDistal,
    "mixamorig:RightHandRing4": FullBodyBoneId.FullBody_RightHandRingTip,
    "mixamorig:RightHandPinky1": FullBodyBoneId.FullBody_RightHandLittleProximal,
    "mixamorig:RightHandPinky2": FullBodyBoneId.FullBody_RightHandLittleIntermediate,
    "mixamorig:RightHandPinky3": FullBodyBoneId.FullBody_RightHandLittleDistal,
    "mixamorig:RightHandPinky4": FullBodyBoneId.FullBody_RightHandLittleTip,
    # Left leg
    "mixamorig:LeftUpLeg": FullBodyBoneId.FullBody_LeftUpperLeg,
    "mixamorig:LeftLeg": FullBodyBoneId.FullBody_LeftLowerLeg,
    "mixamorig:LeftFoot": FullBodyBoneId.FullBody_LeftFootAnkle,
    "mixamorig:LeftToeBase": FullBodyBoneId.FullBody_LeftFootBall,
    # Right leg
    "mixamorig:RightUpLeg": FullBodyBoneId.FullBody_RightUpperLeg,
    "mixamorig:RightLeg": FullBodyBoneId.FullBody_RightLowerLeg,
    "mixamorig:RightFoot": FullBodyBoneId.FullBody_RightFootAnkle,
    "mixamorig:RightToeBase": FullBodyBoneId.FullBody_RightFootBall,
}

# Params
VIZ_POINT_RADIUS = 0.01
ARROW_LENGTH = 0.05

# Choose the reference frame for the bone rotations.
# "world" - rotation with respect to the world frame
# "parent" - rotation with respect to the parent bone
ROTATION_REFERENCE: Literal["world", "parent"] = "parent"

LABELS = ["x", "y", "z"]


# Quaternion utilities
def q_conjugate(q: np.ndarray) -> np.ndarray:
    """Calculate the conjugate of a quaternion."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def q_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([x, y, z, w])


def qv_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate a vector by a quaternion."""
    q_vec = np.array([v[0], v[1], v[2], 0.0])
    q_rotated = q_multiply(q_multiply(q, q_vec), q_conjugate(q))
    return q_rotated[:3]


class BoneData:
    """Represents bone position and rotation data."""

    def __init__(
        self,
        bone_id: FullBodyBoneId,
        position: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
    ):
        self.id = bone_id
        self.position = position
        self.rotation = rotation  # quaternion (x, y, z, w)


def parse_csv_data(csv_file: str) -> dict[int, list[BoneData]]:
    """
    Parse CSV data and return frame-indexed bone data.

    Args:
        csv_file: Path to the CSV file

    Returns:
        Dictionary mapping frame numbers to lists of BoneData
    """
    frame_data = {}

    try:
        with open(csv_file) as f:
            reader = csv.DictReader(f)
            for row in reader:
                frame = int(row["frame"])
                bone_name = row["bone_name"]

                # Skip unmapped bones
                if bone_name not in MIXAMO_TO_FULLBODY_MAPPING:
                    continue

                bone_id = MIXAMO_TO_FULLBODY_MAPPING[bone_name]

                # Parse position (convert to meters and adjust coordinate system)
                # Blender uses Z-up, right-handed. Convert to the expected coordinate system.
                loc_x = float(row["loc_x"]) / 100.0  # Convert cm to meters
                loc_y = float(row["loc_y"]) / 100.0
                loc_z = float(row["loc_z"]) / 100.0

                # Blender uses Z-up. Good!
                position = (loc_x, loc_y, loc_z)

                # Parse rotation (quaternion w, x, y, z -> x, y, z, w)
                rot_w = float(row["rot_w"])
                rot_x = float(row["rot_x"])
                rot_y = float(row["rot_y"])
                rot_z = float(row["rot_z"])
                rotation = (rot_x, rot_y, rot_z, rot_w)

                bone_data = BoneData(bone_id, position, rotation)

                if frame not in frame_data:
                    frame_data[frame] = []
                frame_data[frame].append(bone_data)

    except FileNotFoundError:
        print(f"Error: Could not find file {csv_file}")
        return {}
    except Exception as e:
        print(f"Error parsing CSV file: {e}")
        return {}

    return frame_data


def visualize_frame(rr, frame_data: list[BoneData], frame_number: int):
    """Visualize a single frame of bone data with coordinate frames."""
    if not frame_data:
        return

    positions = []
    keypoint_ids = []
    bone_map: dict[int, BoneData] = {}

    # Create bone map and positions for skeleton visualization
    for bone in frame_data:
        positions.append(bone.position)
        keypoint_ids.append(bone.id.value)
        bone_map[bone.id.value] = bone

    # Set timestamp for this frame
    rr.set_time_sequence("frame", frame_number)

    # Log skeleton points
    rr.log(
        "world/user/bones",
        rr.Points3D(
            positions=positions,
            keypoint_ids=keypoint_ids,
            class_ids=SkeletonType.FullBody.value,
            radii=VIZ_POINT_RADIUS,
        ),
    )

    # Create parent map for relative rotations
    parent_map = {child.value: parent.value for parent, child in FULL_BODY_SKELETON_CONNECTIONS}

    # Visualize local frames
    origins = []
    vectors = []
    colors = []
    labels = []

    for bone_id, bone in bone_map.items():
        rotation = np.array(bone.rotation)

        if ROTATION_REFERENCE == "parent":
            parent_id = parent_map.get(bone_id)
            if parent_id in bone_map:
                parent_bone = bone_map[parent_id]
                parent_rotation = np.array(parent_bone.rotation)
                # rotation = relative rotation
                rotation = q_multiply(q_conjugate(parent_rotation), rotation)

        # Basis vectors
        x_axis = qv_rotate(rotation, np.array([ARROW_LENGTH, 0, 0]))
        y_axis = qv_rotate(rotation, np.array([0, ARROW_LENGTH, 0]))
        z_axis = qv_rotate(rotation, np.array([0, 0, ARROW_LENGTH]))

        # Add arrows for visualization
        origins.extend([bone.position] * 3)
        vectors.extend([x_axis, y_axis, z_axis])
        colors.extend([[255, 0, 0], [0, 255, 0], [0, 0, 255]])

        labels.extend(LABELS)

    if origins:
        rr.log(
            "world/user/bone_frames",
            rr.Arrows3D(
                origins=origins,
                vectors=vectors,
                colors=colors,
                radii=0.001,
                labels=labels,
            ),
        )


def main():
    parser = argparse.ArgumentParser(description="Visualize FBX-exported bone frames")
    parser.add_argument(
        "--file", type=str, required=True, help="Path to the CSV file containing pose data"
    )
    parser.add_argument(
        "--frame", type=int, help="Specific frame to visualize (if not provided, shows frame 1)"
    )
    parser.add_argument("--animate", action="store_true", help="Animate through all frames")
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
    frame_data = parse_csv_data(args.file)

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

    rr.init("fbx-bone-frames-visualizer", spawn=True)

    # Set coordinate system to right-handed, Y-up (typical for processed data)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)

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
            while True:
                for frame_num in sorted_frames:
                    visualize_frame(rr, frame_data[frame_num], frame_num)
                    time.sleep(0.1)  # 10 FPS
        except KeyboardInterrupt:
            print("\nAnimation stopped")
    else:
        # Show specific frame or frame 1
        target_frame = args.frame if args.frame is not None else 1
        if target_frame in frame_data:
            print(f"Visualizing frame {target_frame}")
            visualize_frame(rr, frame_data[target_frame], target_frame)
        else:
            print(f"Frame {target_frame} not found. Available frames: {sorted(frame_data.keys())}")


if __name__ == "__main__":
    main()
