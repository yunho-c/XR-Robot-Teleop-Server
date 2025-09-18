"""
Visualizes body pose data from a static CSV file (exported from FBX via Blender).

This script reads bone position and rotation data from a CSV file with Mixamo bone names
and visualizes it using rerun's 3D visualization. The CSV format should be:
frame,bone_name,loc_x,loc_y,loc_z,rot_w,rot_x,rot_y,rot_z,scale_x,scale_y,scale_z

Usage:
    python visualize_fbx_body_pose.py --file path/to/animation.csv
    python visualize_fbx_body_pose.py --file path/to/animation.csv --frame 10
    python visualize_fbx_body_pose.py --file path/to/animation.csv --animate
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

# Alternative mapping for tip convention (hand bones only)
# In tip convention, joints are named after the bone segment they terminate (tip of bone)
# rather than the bone segment they start (root of bone)
MIXAMO_TO_FULLBODY_MAPPING_TIP_CONVENTION = {
    # Left hand - tip convention (shift mapping down by one level)
    "mixamorig:LeftHandThumb1": FullBodyBoneId.FullBody_LeftHandThumbMetacarpal,
    "mixamorig:LeftHandThumb2": FullBodyBoneId.FullBody_LeftHandThumbProximal,
    "mixamorig:LeftHandThumb3": FullBodyBoneId.FullBody_LeftHandThumbDistal,
    "mixamorig:LeftHandThumb4": None,  # No corresponding bone for tip in this scheme
    "mixamorig:LeftHandIndex1": FullBodyBoneId.FullBody_LeftHandIndexMetacarpal,
    "mixamorig:LeftHandIndex2": FullBodyBoneId.FullBody_LeftHandIndexProximal,
    "mixamorig:LeftHandIndex3": FullBodyBoneId.FullBody_LeftHandIndexIntermediate,
    "mixamorig:LeftHandIndex4": FullBodyBoneId.FullBody_LeftHandIndexDistal,
    "mixamorig:LeftHandMiddle1": FullBodyBoneId.FullBody_LeftHandMiddleMetacarpal,
    "mixamorig:LeftHandMiddle2": FullBodyBoneId.FullBody_LeftHandMiddleProximal,
    "mixamorig:LeftHandMiddle3": FullBodyBoneId.FullBody_LeftHandMiddleIntermediate,
    "mixamorig:LeftHandMiddle4": FullBodyBoneId.FullBody_LeftHandMiddleDistal,
    "mixamorig:LeftHandRing1": FullBodyBoneId.FullBody_LeftHandRingMetacarpal,
    "mixamorig:LeftHandRing2": FullBodyBoneId.FullBody_LeftHandRingProximal,
    "mixamorig:LeftHandRing3": FullBodyBoneId.FullBody_LeftHandRingIntermediate,
    "mixamorig:LeftHandRing4": FullBodyBoneId.FullBody_LeftHandRingDistal,
    "mixamorig:LeftHandPinky1": FullBodyBoneId.FullBody_LeftHandLittleMetacarpal,
    "mixamorig:LeftHandPinky2": FullBodyBoneId.FullBody_LeftHandLittleProximal,
    "mixamorig:LeftHandPinky3": FullBodyBoneId.FullBody_LeftHandLittleIntermediate,
    "mixamorig:LeftHandPinky4": FullBodyBoneId.FullBody_LeftHandLittleDistal,
    # Right hand - tip convention (shift mapping down by one level)
    "mixamorig:RightHandThumb1": FullBodyBoneId.FullBody_RightHandThumbMetacarpal,
    "mixamorig:RightHandThumb2": FullBodyBoneId.FullBody_RightHandThumbProximal,
    "mixamorig:RightHandThumb3": FullBodyBoneId.FullBody_RightHandThumbDistal,
    "mixamorig:RightHandThumb4": None,  # No corresponding bone for tip in this scheme
    "mixamorig:RightHandIndex1": FullBodyBoneId.FullBody_RightHandIndexMetacarpal,
    "mixamorig:RightHandIndex2": FullBodyBoneId.FullBody_RightHandIndexProximal,
    "mixamorig:RightHandIndex3": FullBodyBoneId.FullBody_RightHandIndexIntermediate,
    "mixamorig:RightHandIndex4": FullBodyBoneId.FullBody_RightHandIndexDistal,
    "mixamorig:RightHandMiddle1": FullBodyBoneId.FullBody_RightHandMiddleMetacarpal,
    "mixamorig:RightHandMiddle2": FullBodyBoneId.FullBody_RightHandMiddleProximal,
    "mixamorig:RightHandMiddle3": FullBodyBoneId.FullBody_RightHandMiddleIntermediate,
    "mixamorig:RightHandMiddle4": FullBodyBoneId.FullBody_RightHandMiddleDistal,
    "mixamorig:RightHandRing1": FullBodyBoneId.FullBody_RightHandRingMetacarpal,
    "mixamorig:RightHandRing2": FullBodyBoneId.FullBody_RightHandRingProximal,
    "mixamorig:RightHandRing3": FullBodyBoneId.FullBody_RightHandRingIntermediate,
    "mixamorig:RightHandRing4": FullBodyBoneId.FullBody_RightHandRingDistal,
    "mixamorig:RightHandPinky1": FullBodyBoneId.FullBody_RightHandLittleMetacarpal,
    "mixamorig:RightHandPinky2": FullBodyBoneId.FullBody_RightHandLittleProximal,
    "mixamorig:RightHandPinky3": FullBodyBoneId.FullBody_RightHandLittleIntermediate,
    "mixamorig:RightHandPinky4": FullBodyBoneId.FullBody_RightHandLittleDistal,
}


class BoneData:
    """Represents bone position and rotation data."""

    def __init__(self, bone_id: FullBodyBoneId, position: tuple[float, float, float]):
        self.id = bone_id
        self.position = position


def get_active_mapping(use_tip_convention: bool = False) -> dict:
    """Get the appropriate bone mapping based on convention."""
    if use_tip_convention:
        # Merge main mapping with tip convention overrides (hand bones only)
        mapping = MIXAMO_TO_FULLBODY_MAPPING.copy()
        mapping.update(MIXAMO_TO_FULLBODY_MAPPING_TIP_CONVENTION)
        return mapping
    else:
        return MIXAMO_TO_FULLBODY_MAPPING


def parse_csv_data(csv_file: str, use_tip_convention: bool = False) -> dict[int, list[BoneData]]:
    """
    Parse CSV data and return frame-indexed bone data.

    Args:
        csv_file: Path to the CSV file
        use_tip_convention: Whether to use tip convention for hand bones

    Returns:
        Dictionary mapping frame numbers to lists of BoneData
    """
    frame_data = {}
    active_mapping = get_active_mapping(use_tip_convention)

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

                # Parse position (convert to meters and adjust coordinate system)
                # Blender uses Z-up, right-handed. Convert to the expected coordinate system.
                loc_x = float(row["loc_x"]) / 100.0  # Convert cm to meters
                loc_y = float(row["loc_y"]) / 100.0
                loc_z = float(row["loc_z"]) / 100.0

                # Blender uses Z-up. Good!
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

    def find_available_descendants(bone_id: FullBodyBoneId, visited: set = None) -> list[FullBodyBoneId]:
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
    available_bones = {bone.id for bone in frame_data}

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
        recovered_connections = [(p, c) for p, c, is_recovered in recovered_connections
                               if is_recovered and (p, c) not in original_set]

    print(f"Frame {frame_number}: {len(original_connections)} original + {len(recovered_connections)} recovered connections")

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
                    colors=[[135, 206, 250, 255]],  # Light blue for recovered connections, slightly transparent
                    radii=[0.001],
                ),
            )


def main():
    parser = argparse.ArgumentParser(description="Visualize FBX-exported body pose data")
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
    parser.add_argument("--show-recovered", action="store_true", default=True,
                       help="Show recovered connections that bypass missing bones (default: True)")
    parser.add_argument("--no-recovered", dest="show_recovered", action="store_false",
                       help="Hide recovered connections, show only original intact connections")
    parser.add_argument("--tip-convention", action="store_true", default=False,
                       help="Use tip convention for hand bones (joints named after bone tip rather than root)")
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
    convention_str = "tip" if args.tip_convention else "root"
    print(f"Using {convention_str} convention for hand bones")
    frame_data = parse_csv_data(args.file, args.tip_convention)

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

    rr.init("fbx-pose-visualizer", spawn=True)

    # Set coordinate system to right-handed, Z-up (converted from FBX Y-up data)
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
