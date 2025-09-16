"""
Converts FBX-exported CSV animation data to OpenXR body pose recording format.

This script reads bone position and rotation data from a CSV file (exported from FBX via Blender)
and converts it to the same CSV format produced by examples/record_body_pose.py.

Input format (from Blender FBX export):
frame,bone_name,loc_x,loc_y,loc_z,rot_w,rot_x,rot_y,rot_z,scale_x,scale_y,scale_z

Output format (matching record_body_pose.py):
time_elapsed,bone_id,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w

Usage:
    python misc/convert_fbx_to_openxr.py --input animation.csv --output converted.csv
    python misc/convert_fbx_to_openxr.py --input animation.csv --output converted.csv --fps 30
"""

import argparse
import csv
import os
from datetime import datetime

from visualize_fbx_body_pose import get_active_mapping

from xr_robot_teleop_server import configure_logging


def convert_fbx_to_openxr_csv(
    input_file: str, output_file: str, fps: float = 24.0, decimal_precision: int = 3, bone_id_as_int: bool = True, use_tip_convention: bool = False
):
    """
    Convert FBX CSV data to OpenXR body pose CSV format.

    Args:
        input_file: Path to input FBX CSV file
        output_file: Path to output CSV file
        fps: Frames per second for time conversion
        decimal_precision: Number of decimal places for values
        bone_id_as_int: Whether to output bone IDs as integers (True) or strings (False)
        use_tip_convention: If True, use tip convention for hand bone mapping
    """
    print(f"Loading FBX data from {input_file}...")

    # Read the FBX CSV file directly (not using parse_csv_data since it only returns positions)
    frame_data = {}
    active_mapping = get_active_mapping(use_tip_convention)

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

                # Parse position (convert cm to meters)
                pos_x = float(row["loc_x"]) / 100.0
                pos_y = float(row["loc_y"]) / 100.0
                pos_z = float(row["loc_z"]) / 100.0

                # Parse rotation (Blender outputs WXYZ, we need XYZW)
                rot_w = float(row["rot_w"])
                rot_x = float(row["rot_x"])
                rot_y = float(row["rot_y"])
                rot_z = float(row["rot_z"])

                if frame not in frame_data:
                    frame_data[frame] = []

                frame_data[frame].append(
                    {
                        "bone_id": bone_id.value if bone_id_as_int else bone_id.name,
                        "pos_x": pos_x,
                        "pos_y": pos_y,
                        "pos_z": pos_z,
                        "rot_x": rot_x,
                        "rot_y": rot_y,
                        "rot_z": rot_z,
                        "rot_w": rot_w,
                    }
                )

    except FileNotFoundError:
        print(f"Error: Could not find input file {input_file}")
        return False
    except Exception as e:
        print(f"Error reading input file: {e}")
        return False

    if not frame_data:
        print("No valid bone data found in input file")
        return False

    # Count mapped bones (excluding None values)
    mapped_bones = sum(1 for bone_id in active_mapping.values() if bone_id is not None)
    print(f"Loaded {len(frame_data)} frames with {mapped_bones} mapped bones")

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
    parser = argparse.ArgumentParser(description="Convert FBX CSV to OpenXR body pose CSV format")
    parser.add_argument("--input", "-i", type=str, required=True, help="Input FBX CSV file path")
    parser.add_argument(
        "--output", "-o", type=str, help="Output CSV file path (default: auto-generated)"
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Frames per second for time conversion (default: 24.0)",
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
        "--tip-convention",
        action="store_true",
        help="Use tip convention for hand bone mapping (joints named after bone tips rather than roots)",
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
    if args.tip_convention:
        print("Using tip convention for hand bone mapping")
    
    success = convert_fbx_to_openxr_csv(
        input_file=args.input,
        output_file=args.output,
        fps=args.fps,
        decimal_precision=args.decimal_precision,
        bone_id_as_int=not args.bone_id_as_string,
        use_tip_convention=args.tip_convention,
    )

    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
