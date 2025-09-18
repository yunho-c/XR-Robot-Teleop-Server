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
    
    # IK bone tip extensions - no suitable OpenXR equivalents, skip
    "つま先ＩＫ.L_tip": None,  # Toe IK tip - no OpenXR equivalent
    "足ＩＫ先.L_tip": None,    # Foot IK tip - no OpenXR equivalent  
    "つま先ＩＫ.R_tip": None,  # Toe IK tip - no OpenXR equivalent
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


def parse_csv_data(
    csv_file: str, convention: str = "root", interpolate_spine: bool = False
) -> dict[int, list[BoneData]]:
    """
    Parse CSV data and return frame-indexed bone data.

    Args:
        csv_file: Path to the CSV file
        convention: Convention to use - 'root', 'tip', or 'mediapipe'
        interpolate_spine: Whether to interpolate FullBody_SpineLower from FullBody_SpineMiddle and FullBody_Hips

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

    return frame_data


def build_skeleton_tree():
    """Build a tree structure from skeleton connections for traversal."""
    children = {}  # parent_id -> [child_ids]
    for parent, child in FULL_BODY_SKELETON_CONNECTIONS:
        if parent not in children:
            children[parent] = []
        children[parent].append(child)
    return children


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


def visualize_frame(rr, frame_data: list[BoneData], frame_number: int, show_recovered: bool = True):
    """Visualize a single frame of bone data with separate original and recovered connections."""
    if not frame_data:
        return

    # Get available bones from the frame data
    available_bones = {bone.id: bone for bone in frame_data}

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

    # Create position lookup for line drawing
    bone_positions = {bone.id: bone.position for bone in frame_data}

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
    frame_data = parse_csv_data(args.file, args.convention, args.interpolate_spine)

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
            print(f"Connection recovery: {'enabled' if args.show_recovered else 'disabled'}")
            visualize_frame(rr, frame_data[target_frame], target_frame, args.show_recovered)
        else:
            print(f"Frame {target_frame} not found. Available frames: {sorted(frame_data.keys())}")


if __name__ == "__main__":
    main()
