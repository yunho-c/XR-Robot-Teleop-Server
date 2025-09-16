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
import time

import numpy as np

from xr_robot_teleop_server import configure_logging
from xr_robot_teleop_server.schemas.openxr_skeletons import (
    FULL_BODY_SKELETON_CONNECTIONS,
    FullBodyBoneId,
    SkeletonType,
)

# VMD to FullBody bone name mapping
VMD_TO_FULLBODY_MAPPING = {
    # Core body - Japanese VMD names
    "全ての親": None,  # Root bone, skip
    "センター": FullBodyBoneId.FullBody_Hips,
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

    # IK bones - these don't directly map to OpenXR bones, so we'll skip them
    "足ＩＫ.L": None,
    "つま先ＩＫ.L": None,
    "足ＩＫ先.L": None,
    "足ＩＫ.R": None,
    "つま先ＩＫ.R": None,
    "足ＩＫ先.R": None,
}


class BoneData:
    """Represents bone position and rotation data."""

    def __init__(self, bone_id: FullBodyBoneId, position: tuple[float, float, float]):
        self.id = bone_id
        self.position = position


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

                # Skip unmapped bones or bones mapped to None
                if (
                    bone_name not in VMD_TO_FULLBODY_MAPPING
                    or VMD_TO_FULLBODY_MAPPING[bone_name] is None
                ):
                    continue

                bone_id = VMD_TO_FULLBODY_MAPPING[bone_name]

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

    return frame_data


def visualize_frame(rr, frame_data: list[BoneData], frame_number: int):
    """Visualize a single frame of bone data."""
    if not frame_data:
        return

    positions = []
    keypoint_ids = []

    for bone in frame_data:
        positions.append(bone.position)
        keypoint_ids.append(bone.id.value)

    # Set timestamp for this frame
    rr.set_time_sequence("frame", frame_number)

    rr.log(
        "world/user/bones",
        rr.Points3D(
            positions=positions,
            keypoint_ids=keypoint_ids,
            class_ids=SkeletonType.FullBody.value,
            radii=0.01,
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
                    visualize_frame(rr, frame_data[frame_num], frame_num)
                    time.sleep(sleep_time)
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
