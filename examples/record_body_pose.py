"""
Usage:
# HDF5 format (requires: pip install h5py)
python examples/record_body_pose.py --format hdf5

# Other formats
python examples/record_body_pose.py --format csv
python examples/record_body_pose.py --format jsonl

HDF5 Structure:
body_pose_YYYYMMDD_HHMMSS.h5
├── metadata/           # Reserved for future metadata
├── pose_data/
│   ├── frame_0/
│   │   ├── bone_ids    # Array of bone IDs
│   │   ├── positions   # [N, 3] array (compressed)
│   │   └── rotations   # [N, 4] array (compressed)
│   └── frame_1/
│       └── ...

The script gracefully handles missing h5py dependency with a clear error message.
"""

import argparse
import csv
import json
import os
import time
from datetime import datetime
from functools import partial

try:
    import h5py

    HDF5_AVAILABLE = True
except ImportError:
    HDF5_AVAILABLE = False

from xr_robot_teleop_server import configure_logging
from xr_robot_teleop_server.schemas.body_pose import (
    deserialize_pose_data,
)
from xr_robot_teleop_server.streaming import WebRTCServer

# Params
CONVERT_UNITY_COORDS = True


class RecorderState:
    def __init__(self, output_file: str, output_format: str = "jsonl"):
        self.output_file = output_file
        self.output_format = output_format.lower()
        self.recording_started = False
        self.pose_count = 0
        self.csv_writer = None
        self.csv_file = None
        self.hdf5_file = None
        self.hdf5_datasets = {}

    def __repr__(self):
        return f"<RecorderState output_file={self.output_file}>"


def on_body_pose_message(message: bytes, state: RecorderState):
    try:
        if isinstance(message, bytes):
            pose_data = deserialize_pose_data(message, z_up=CONVERT_UNITY_COORDS)
            if not state.recording_started:
                print(f"Started recording body pose data to {state.output_file}")
                state.recording_started = True

                # Initialize writers based on format
                if state.output_format == "csv":
                    state.csv_file = open(state.output_file, "w", newline="")
                    fieldnames = [
                        "timestamp",
                        "datetime",
                        "bone_id",
                        "pos_x",
                        "pos_y",
                        "pos_z",
                        "rot_x",
                        "rot_y",
                        "rot_z",
                        "rot_w",
                    ]
                    state.csv_writer = csv.DictWriter(state.csv_file, fieldnames=fieldnames)
                    state.csv_writer.writeheader()
                elif state.output_format == "hdf5":
                    state.hdf5_file = h5py.File(state.output_file, "w")
                    # Create groups for organized data
                    state.hdf5_file.create_group("metadata")
                    state.hdf5_file.create_group("pose_data")
                    # Store metadata
                    state.hdf5_file.attrs["format_version"] = "1.0"
                    state.hdf5_file.attrs["created_at"] = datetime.now().isoformat()
                    state.hdf5_file.attrs["convert_unity_coords"] = CONVERT_UNITY_COORDS

            timestamp = time.time()
            datetime_str = datetime.fromtimestamp(timestamp).isoformat()

            if state.output_format == "csv":
                # Write each bone as a separate row in CSV
                for bone in pose_data:
                    row = {
                        "timestamp": timestamp,
                        "datetime": datetime_str,
                        "bone_id": bone.id,
                        "pos_x": float(bone.position[0]),
                        "pos_y": float(bone.position[1]),
                        "pos_z": float(bone.position[2]),
                        "rot_x": float(bone.rotation[0]),
                        "rot_y": float(bone.rotation[1]),
                        "rot_z": float(bone.rotation[2]),
                        "rot_w": float(bone.rotation[3]),
                    }
                    state.csv_writer.writerow(row)
                state.csv_file.flush()
            elif state.output_format == "hdf5":
                # Store data efficiently in HDF5 format
                frame_group = state.hdf5_file["pose_data"].create_group(f"frame_{state.pose_count}")
                frame_group.attrs["timestamp"] = timestamp
                frame_group.attrs["datetime"] = datetime_str

                # Create datasets for positions and rotations
                bone_ids = [bone.id for bone in pose_data]
                positions = [
                    [bone.position[0], bone.position[1], bone.position[2]] for bone in pose_data
                ]
                rotations = [
                    [bone.rotation[0], bone.rotation[1], bone.rotation[2], bone.rotation[3]]
                    for bone in pose_data
                ]

                frame_group.create_dataset("bone_ids", data=bone_ids)
                frame_group.create_dataset("positions", data=positions, compression="gzip")
                frame_group.create_dataset("rotations", data=rotations, compression="gzip")
                # Flush every 10 frames to ensure data is written
                if state.pose_count % 10 == 0:
                    state.hdf5_file.flush()
            else:
                # JSONL format (original behavior)
                pose_entry = {
                    "timestamp": timestamp,
                    "datetime": datetime_str,
                    "bones": [],
                }

                for bone in pose_data:
                    bone_entry = {
                        "id": bone.id,
                        "position": {
                            "x": float(bone.position[0]),
                            "y": float(bone.position[1]),
                            "z": float(bone.position[2]),
                        },
                        "rotation": {
                            "x": float(bone.rotation[0]),
                            "y": float(bone.rotation[1]),
                            "z": float(bone.rotation[2]),
                            "w": float(bone.rotation[3]),
                        },
                    }
                    pose_entry["bones"].append(bone_entry)

                # Append to file
                with open(state.output_file, "a") as f:
                    f.write(json.dumps(pose_entry) + "\n")

            state.pose_count += 1
            if state.pose_count % 100 == 0:  # Print every 100 poses
                print(f"Recorded {state.pose_count} pose frames")

    except Exception as e:
        print(f"Could not process body pose data: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="XR Body Pose Recorder")
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        help="Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)",
    )
    parser.add_argument(
        "--output-file",
        type=str,
        default=None,
        help="Output file path for recorded data (default: body_pose_YYYYMMDD_HHMMSS.jsonl)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="./recordings",
        help="Output directory for recorded data (default: ./recordings)",
    )
    parser.add_argument(
        "--format",
        type=str,
        choices=["jsonl", "csv", "hdf5"],
        default="jsonl",
        help="Output format: jsonl, csv, or hdf5 (default: jsonl)",
    )
    args = parser.parse_args()

    # Configure logging
    configure_logging(level=args.log_level)

    # Create output directory if it doesn't exist
    os.makedirs(args.output_dir, exist_ok=True)

    # Check HDF5 availability if requested
    if args.format == "hdf5" and not HDF5_AVAILABLE:
        print("Error: HDF5 format requires h5py. Install with: pip install h5py")
        exit(1)

    # Generate output filename if not provided
    if args.output_file is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if args.format == "csv":
            extension = "csv"
        elif args.format == "hdf5":
            extension = "h5"
        else:
            extension = "jsonl"
        output_file = os.path.join(args.output_dir, f"body_pose_{timestamp}.{extension}")
    else:
        output_file = args.output_file

    print(f"Body pose data will be recorded to: {output_file}")

    state_factory = partial(RecorderState, output_file=output_file, output_format=args.format)

    data_handlers = {
        "body_pose": on_body_pose_message,
    }

    server = WebRTCServer(
        datachannel_handlers=data_handlers,
        state_factory=state_factory,
    )

    try:
        server.run()
    except KeyboardInterrupt:
        # Clean up files if they were opened
        state = state_factory()
        if hasattr(state, "csv_file") and state.csv_file:
            state.csv_file.close()
        if hasattr(state, "hdf5_file") and state.hdf5_file:
            state.hdf5_file.close()
        print(
            f"\nRecording stopped. Total poses recorded: "
            f"{state.pose_count if hasattr(state, 'pose_count') else 'unknown'}"
        )
        print(f"Data saved to: {output_file}")
