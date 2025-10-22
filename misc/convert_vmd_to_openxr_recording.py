"""
Converts VMD-exported CSV animation data to OpenXR body pose recording format.

This script reads bone position and rotation data from a CSV file (exported from VMD via Blender)
and converts it to the same CSV format produced by examples/record_body_pose.py.

Input format (from VMD/Blender export):
frame,bone_name,loc_x,loc_y,loc_z,rot_w,rot_x,rot_y,rot_z,scale_x,scale_y,scale_z

Output format (matching record_body_pose.py):
time_elapsed,bone_id,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w

Usage:
    python convert_vmd_to_openxr_recording.py --input animation.csv --output converted.csv
    python convert_vmd_to_openxr_recording.py --input animation.csv --output converted.csv --fps 30
"""

import argparse
import csv
import os
from datetime import datetime

from visualize_vmd_body_pose import (
    BoneData,
    calculate_fake_subtalar_bones,
    get_active_mapping,
    interpolate_spine_lower,
)

from xr_robot_teleop_server import configure_logging


def convert_vmd_to_openxr_csv(
    input_file: str,
    output_file: str,
    fps: float = 30.0,
    decimal_precision: int = 3,
    bone_id_as_int: bool = True,
    convert_coords: bool = True,
    convention: str = "root",
    interpolate_spine: bool = False,
    calculate_subtalar: bool = True,
):
    """
    Convert VMD CSV data to OpenXR body pose CSV format.

    Args:
        input_file: Path to input VMD CSV file
        output_file: Path to output CSV file
        fps: Frames per second for time conversion
        decimal_precision: Number of decimal places for values
        bone_id_as_int: Whether to output bone IDs as integers (True) or strings (False)
        convert_coords: Whether to convert Y-up to Z-up coordinates
        convention: Bone mapping convention ('root', 'tip', or 'mediapipe')
        interpolate_spine: Whether to interpolate FullBody_SpineLower from FullBody_SpineMiddle and FullBody_Hips
        calculate_subtalar: Whether to calculate fake subtalar bones from toe IK data
    """
    print(f"Loading VMD data from {input_file}...")

    # Read the VMD CSV file
    frame_data = {}
    bone_data_objects = {}  # Store BoneData objects for interpolation
    active_mapping = get_active_mapping(convention)

    try:
        with open(input_file) as f:
            reader = csv.DictReader(f)
            for row in reader:
                frame = int(row["frame"])
                bone_name = row["bone_name"]

                # Skip unmapped bones or bones mapped to None
                if bone_name not in active_mapping or active_mapping[bone_name] is None:
                    continue

                bone_id = active_mapping[bone_name]

                # Parse position (VMD data appears to be in meters already)
                loc_x = float(row["loc_x"])
                loc_y = float(row["loc_y"])
                loc_z = float(row["loc_z"])

                # Convert coordinate system if requested
                if convert_coords:
                    # Convert Y-up to Z-up: X stays X, Y becomes -Z, Z becomes Y
                    pos_x, pos_y, pos_z = loc_x, -loc_z, loc_y
                else:
                    pos_x, pos_y, pos_z = loc_x, loc_y, loc_z

                # Parse rotation (VMD outputs WXYZ, we need XYZW for OpenXR format)
                rot_w = float(row["rot_w"])
                rot_x = float(row["rot_x"])
                rot_y = float(row["rot_y"])
                rot_z = float(row["rot_z"])

                # Convert quaternion coordinate system if requested
                if convert_coords:
                    # For Y-up to Z-up conversion, quaternion needs adjustment
                    # Quaternion rotation: (x, y, z, w) -> (x, -z, y, w)
                    rot_x_new, rot_y_new, rot_z_new = rot_x, -rot_z, rot_y
                else:
                    rot_x_new, rot_y_new, rot_z_new = rot_x, rot_y, rot_z

                if frame not in frame_data:
                    frame_data[frame] = []

                bone_dict = {
                    "bone_id": bone_id.value if bone_id_as_int else bone_id.name,
                    "pos_x": pos_x,
                    "pos_y": pos_y,
                    "pos_z": pos_z,
                    "rot_x": rot_x_new,
                    "rot_y": rot_y_new,
                    "rot_z": rot_z_new,
                    "rot_w": rot_w,
                }
                frame_data[frame].append(bone_dict)

                # Store BoneData object for interpolation/calculation if needed
                if interpolate_spine or calculate_subtalar:
                    if frame not in bone_data_objects:
                        bone_data_objects[frame] = []
                    bone_data_objects[frame].append(BoneData(bone_id, (pos_x, pos_y, pos_z)))

    except FileNotFoundError:
        print(f"Error: Could not find input file {input_file}")
        return False
    except Exception as e:
        print(f"Error reading input file: {e}")
        return False

    if not frame_data:
        print("No valid bone data found in input file")
        return False

    # Count mapped bones
    mapped_bones = sum(1 for bone_id in active_mapping.values() if bone_id is not None)
    print(f"Loaded {len(frame_data)} frames with {mapped_bones} mapped bones")

    # Add interpolated SpineLower joints if requested
    if interpolate_spine:
        interpolated_count = 0
        for frame_num in frame_data.keys():
            if frame_num in bone_data_objects:
                spine_lower = interpolate_spine_lower(bone_data_objects[frame_num])
                if spine_lower is not None:
                    # Convert BoneData back to dictionary format
                    spine_lower_dict = {
                        "bone_id": spine_lower.id.value if bone_id_as_int else spine_lower.id.name,
                        "pos_x": spine_lower.position[0],
                        "pos_y": spine_lower.position[1],
                        "pos_z": spine_lower.position[2],
                        "rot_x": 0.0,  # Default rotation
                        "rot_y": 0.0,
                        "rot_z": 0.0,
                        "rot_w": 1.0,
                    }
                    frame_data[frame_num].append(spine_lower_dict)
                    interpolated_count += 1
        print(f"Interpolated FullBody_SpineLower for {interpolated_count} frames")

    # Add calculated subtalar bones if requested
    if calculate_subtalar:
        subtalar_count = 0
        for frame_num in frame_data.keys():
            if frame_num in bone_data_objects:
                subtalar_bones = calculate_fake_subtalar_bones(bone_data_objects[frame_num])
                for subtalar_bone in subtalar_bones:
                    # Convert BoneData back to dictionary format
                    subtalar_dict = {
                        "bone_id": subtalar_bone.id.value
                        if bone_id_as_int
                        else subtalar_bone.id.name,
                        "pos_x": subtalar_bone.position[0],
                        "pos_y": subtalar_bone.position[1],
                        "pos_z": subtalar_bone.position[2],
                        "rot_x": 0.0,  # Default rotation
                        "rot_y": 0.0,
                        "rot_z": 0.0,
                        "rot_w": 1.0,
                    }
                    frame_data[frame_num].append(subtalar_dict)
                    subtalar_count += 1
        if subtalar_count > 0:
            print(f"Calculated {subtalar_count} fake subtalar bones across all frames")

    # Write output CSV
    print(f"Writing OpenXR CSV data to {output_file}...")

    try:
        with open(output_file, "w", newline="") as f:
            fieldnames = [
                "time_elapsed",
                "bone_id",
                "pos_x",
                "pos_y",
                "pos_z",
                "rot_x",
                "rot_y",
                "rot_z",
                "rot_w",
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            # Convert frames to time and write rows
            sorted_frames = sorted(frame_data.keys())
            for frame_num in sorted_frames:
                time_elapsed = frame_num / fps  # Convert frame to time

                for bone_data in frame_data[frame_num]:
                    row = {
                        "time_elapsed": round(time_elapsed, decimal_precision),
                        "bone_id": bone_data["bone_id"],
                        "pos_x": round(bone_data["pos_x"], decimal_precision),
                        "pos_y": round(bone_data["pos_y"], decimal_precision),
                        "pos_z": round(bone_data["pos_z"], decimal_precision),
                        "rot_x": round(bone_data["rot_x"], decimal_precision),
                        "rot_y": round(bone_data["rot_y"], decimal_precision),
                        "rot_z": round(bone_data["rot_z"], decimal_precision),
                        "rot_w": round(bone_data["rot_w"], decimal_precision),
                    }
                    writer.writerow(row)

        print(f"Successfully converted {len(sorted_frames)} frames to {output_file}")
        return True

    except Exception as e:
        print(f"Error writing output file: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Convert VMD CSV to OpenXR body pose CSV format")
    parser.add_argument("--input", "-i", type=str, required=True, help="Input VMD CSV file path")
    parser.add_argument(
        "--output", "-o", type=str, help="Output CSV file path (default: auto-generated)"
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Frames per second for time conversion (default: 30.0)",
    )
    parser.add_argument(
        "--decimal-precision",
        type=int,
        default=3,
        help="Number of decimal places for values (default: 3)",
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        help="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
    )
    parser.add_argument(
        "--bone-id-as-string",
        action="store_true",
        help="Output bone IDs as strings instead of integers (default: integers)",
    )
    parser.add_argument(
        "--no-coord-conversion",
        action="store_true",
        help="Skip coordinate system conversion from Y-up to Z-up (default: convert)",
    )
    parser.add_argument(
        "--convention",
        type=str,
        default="root",
        choices=["root", "tip", "mediapipe"],
        help="Bone mapping convention: 'root' (default), 'tip' (joints named after bone tips), or 'mediapipe' (for MediaPipe compatibility)",
    )
    parser.add_argument(
        "--interpolate-spine",
        action="store_true",
        help="Create FullBody_SpineLower by interpolating midpoint between FullBody_SpineMiddle and FullBody_Hips",
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

    args = parser.parse_args()

    # Configure logging
    configure_logging(level=args.log_level)

    # Generate output filename if not provided
    if args.output is None:
        input_base = os.path.splitext(os.path.basename(args.input))[0]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"{input_base}_openxr_{timestamp}.csv"

    # Ensure input file exists
    if not os.path.exists(args.input):
        print(f"Error: Input file {args.input} does not exist")
        return 1

    # Convert the file
    if args.convention != "root":
        print(f"Using {args.convention} convention for bone mapping")
    if args.interpolate_spine:
        print("Interpolating FullBody_SpineLower from FullBody_SpineMiddle and FullBody_Hips")
    if args.calculate_subtalar:
        print("Calculating fake subtalar bones from toe IK data")

    if not args.no_coord_conversion:
        print(
            "WARNING: If input file was exported using `blender_export_script.py`, the data "
            "is already in Z-up. Enabling coordinate conversion will create an incorrect file."
        )

    success = convert_vmd_to_openxr_csv(
        input_file=args.input,
        output_file=args.output,
        fps=args.fps,
        decimal_precision=args.decimal_precision,
        bone_id_as_int=not args.bone_id_as_string,
        convert_coords=not args.no_coord_conversion,
        convention=args.convention,
        interpolate_spine=args.interpolate_spine,
        calculate_subtalar=args.calculate_subtalar,
    )

    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
