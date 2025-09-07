"""
Usage:
# HDF5 format (requires: pip install h5py)
python examples/record_body_pose.py --format hdf5

# Other formats
python examples/record_body_pose.py --format csv
python examples/record_body_pose.py --format jsonl

HDF5 Structure (follows time-varying data best practices):
body_pose_YYYYMMDD_HHMMSS.h5
├── time              # [timesteps] - time index with proper units
├── positions         # [timesteps, bones, 3] - XYZ positions (compressed)
├── rotations         # [timesteps, bones, 4] - XYZW quaternions (compressed)
├── bone_ids          # [bones] - bone identifiers
├── bone_index        # [bones] - dimension scale for bone axis
└── attributes:       # metadata including format_version, created_at, etc.

The script gracefully handles missing h5py dependency with a clear error message.
"""

import argparse
import csv
import json
import os
import time
from datetime import datetime

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

# Global state reference for cleanup
_current_state = None


class RecorderState:
    def __init__(
        self,
        output_file: str,
        output_format: str = "jsonl",
        decimal_precision: int = 3,
        relative_time: bool = True,
    ):
        self.output_file = output_file
        self.output_format = output_format.lower()
        self.decimal_precision = decimal_precision
        self.relative_time = relative_time
        self.recording_started = False
        self.pose_count = 0
        self.start_time = None
        self.csv_writer = None
        self.csv_file = None
        self.hdf5_file = None
        self.hdf5_datasets = {}
        self.time_data = []
        self.positions_data = []
        self.rotations_data = []
        self.bone_ids_data = []
        self.initial_bone_ids = None

    def __repr__(self):
        return f"<RecorderState output_file={self.output_file}>"


def on_body_pose_message(message: bytes, state: RecorderState):
    try:
        if isinstance(message, bytes):
            pose_data = deserialize_pose_data(message, z_up=CONVERT_UNITY_COORDS)
            if not state.recording_started:
                print(f"Started recording body pose data to {state.output_file}")
                state.recording_started = True
                state.start_time = time.time()

                # Initialize writers based on format
                if state.output_format == "csv":
                    state.csv_file = open(state.output_file, "w", newline="")
                    if state.relative_time:
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
                    else:
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
                    # Store metadata at root level
                    state.hdf5_file.attrs["format_version"] = "2.0"
                    state.hdf5_file.attrs["created_at"] = datetime.now().isoformat()
                    state.hdf5_file.attrs["convert_unity_coords"] = CONVERT_UNITY_COORDS
                    state.hdf5_file.attrs["description"] = (
                        "Body pose recording with time-varying data structure"
                    )
                    # Store initial bone IDs for consistency checking
                    state.initial_bone_ids = [bone.id for bone in pose_data]

            timestamp = time.time()
            datetime_str = datetime.fromtimestamp(timestamp).isoformat()

            if state.output_format == "csv":
                # Write each bone as a separate row in CSV
                for bone in pose_data:
                    if state.relative_time:
                        time_elapsed = timestamp - state.start_time
                        row = {
                            "time_elapsed": round(time_elapsed, state.decimal_precision),
                            "bone_id": bone.id,
                            "pos_x": round(float(bone.position[0]), state.decimal_precision),
                            "pos_y": round(float(bone.position[1]), state.decimal_precision),
                            "pos_z": round(float(bone.position[2]), state.decimal_precision),
                            "rot_x": round(float(bone.rotation[0]), state.decimal_precision),
                            "rot_y": round(float(bone.rotation[1]), state.decimal_precision),
                            "rot_z": round(float(bone.rotation[2]), state.decimal_precision),
                            "rot_w": round(float(bone.rotation[3]), state.decimal_precision),
                        }
                    else:
                        row = {
                            "timestamp": timestamp,
                            "datetime": datetime_str,
                            "bone_id": bone.id,
                            "pos_x": round(float(bone.position[0]), state.decimal_precision),
                            "pos_y": round(float(bone.position[1]), state.decimal_precision),
                            "pos_z": round(float(bone.position[2]), state.decimal_precision),
                            "rot_x": round(float(bone.rotation[0]), state.decimal_precision),
                            "rot_y": round(float(bone.rotation[1]), state.decimal_precision),
                            "rot_z": round(float(bone.rotation[2]), state.decimal_precision),
                            "rot_w": round(float(bone.rotation[3]), state.decimal_precision),
                        }
                    state.csv_writer.writerow(row)
                state.csv_file.flush()
            elif state.output_format == "hdf5":
                # Collect data in memory for efficient batch writing
                state.time_data.append(timestamp)
                # Verify bone IDs are consistent
                current_bone_ids = [bone.id for bone in pose_data]
                if current_bone_ids != state.initial_bone_ids:
                    print(f"Warning: Bone IDs changed at frame {state.pose_count}")
                # Collect positions and rotations
                positions = [
                    [bone.position[0], bone.position[1], bone.position[2]] for bone in pose_data
                ]
                rotations = [
                    [bone.rotation[0], bone.rotation[1], bone.rotation[2], bone.rotation[3]]
                    for bone in pose_data
                ]

                state.positions_data.append(positions)
                state.rotations_data.append(rotations)
                # Write to HDF5 every 100 frames for efficiency
                if state.pose_count % 100 == 0:
                    state.hdf5_file.flush()
            else:
                # JSONL format (original behavior)
                if state.relative_time:
                    time_elapsed = timestamp - state.start_time
                    pose_entry = {
                        "time_elapsed": round(time_elapsed, state.decimal_precision),
                        "bones": [],
                    }
                else:
                    pose_entry = {
                        "timestamp": timestamp,
                        "datetime": datetime_str,
                        "bones": [],
                    }

                for bone in pose_data:
                    bone_entry = {
                        "id": bone.id,
                        "pos": {
                            "x": round(float(bone.position[0]), state.decimal_precision),
                            "y": round(float(bone.position[1]), state.decimal_precision),
                            "z": round(float(bone.position[2]), state.decimal_precision),
                        },
                        "rot": {
                            "x": round(float(bone.rotation[0]), state.decimal_precision),
                            "y": round(float(bone.rotation[1]), state.decimal_precision),
                            "z": round(float(bone.rotation[2]), state.decimal_precision),
                            "w": round(float(bone.rotation[3]), state.decimal_precision),
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


def finalize_hdf5_recording(state: RecorderState):
    """Write accumulated data to HDF5 file using best practices for time-varying data."""
    if state.output_format != "hdf5" or not state.hdf5_file or not state.time_data:
        return
    import numpy as np

    # Convert lists to numpy arrays for efficient storage
    time_array = np.array(state.time_data, dtype=np.float64)
    positions_array = np.array(state.positions_data, dtype=np.float32)
    rotations_array = np.array(state.rotations_data, dtype=np.float32)

    # Create the time dataset (1D array)
    time_dataset = state.hdf5_file.create_dataset(
        "time", data=time_array, compression="gzip", shuffle=True, fletcher32=True
    )
    time_dataset.attrs["units"] = (
        f"seconds since {datetime.fromtimestamp(time_array[0]).isoformat()}"
    )
    time_dataset.attrs["description"] = "Unix timestamps for each pose frame"
    # Create positions dataset (3D array: [timesteps, bones, xyz])
    positions_dataset = state.hdf5_file.create_dataset(
        "positions", data=positions_array, compression="gzip", shuffle=True, fletcher32=True
    )
    positions_dataset.attrs["units"] = "meters" if CONVERT_UNITY_COORDS else "unity_units"
    positions_dataset.attrs["description"] = "XYZ positions for each bone at each timestep"
    positions_dataset.attrs["dimensions"] = "time, bone_index, xyz"
    # Create rotations dataset (3D array: [timesteps, bones, wxyz])
    rotations_dataset = state.hdf5_file.create_dataset(
        "rotations", data=rotations_array, compression="gzip", shuffle=True, fletcher32=True
    )
    rotations_dataset.attrs["units"] = "quaternion"
    rotations_dataset.attrs["description"] = (
        "XYZW quaternion rotations for each bone at each timestep"
    )
    rotations_dataset.attrs["dimensions"] = "time, bone_index, xyzw"
    # Create bone_ids dataset (1D array of bone identifiers)
    bone_ids_dataset = state.hdf5_file.create_dataset(
        "bone_ids", data=np.array(state.initial_bone_ids, dtype=h5py.string_dtype())
    )
    bone_ids_dataset.attrs["description"] = (
        "Bone identifiers corresponding to the bone_index dimension"
    )
    # Set up dimension scales for proper HDF5 structure
    try:
        # Make time dataset a dimension scale
        time_dataset.make_scale("time")
        # Create bone index dimension scale
        bone_index = np.arange(len(state.initial_bone_ids), dtype=np.int32)
        bone_index_dataset = state.hdf5_file.create_dataset("bone_index", data=bone_index)
        bone_index_dataset.make_scale("bone_index")
        # Attach dimension scales to data arrays
        positions_dataset.dims[0].attach_scale(time_dataset)
        positions_dataset.dims[1].attach_scale(bone_index_dataset)
        positions_dataset.dims[0].label = "time"
        positions_dataset.dims[1].label = "bone_index"
        positions_dataset.dims[2].label = "xyz"

        rotations_dataset.dims[0].attach_scale(time_dataset)
        rotations_dataset.dims[1].attach_scale(bone_index_dataset)
        rotations_dataset.dims[0].label = "time"
        rotations_dataset.dims[1].label = "bone_index"
        rotations_dataset.dims[2].label = "xyzw"
    except Exception as e:
        print(f"Warning: Could not set up dimension scales: {e}")
    # Store final statistics
    state.hdf5_file.attrs["total_frames"] = len(state.time_data)
    state.hdf5_file.attrs["num_bones"] = len(state.initial_bone_ids)
    state.hdf5_file.attrs["duration_seconds"] = (
        float(time_array[-1] - time_array[0]) if len(time_array) > 1 else 0.0
    )
    state.hdf5_file.flush()
    print(
        f"HDF5 data finalized: {len(state.time_data)} frames, {len(state.initial_bone_ids)} bones"
    )


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
    parser.add_argument(
        "--decimal-precision",
        type=int,
        default=3,
        help=(
            "Number of decimal places for position/rotation values "
            "in JSONL and CSV formats (default: 3)"
        ),
    )
    parser.add_argument(
        "--absolute-time",
        action="store_true",
        help=(
            "Use absolute timestamp/datetime instead of relative "
            "time_elapsed for JSONL and CSV formats"
        ),
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

    def state_factory():
        global _current_state
        _current_state = RecorderState(
            output_file=output_file,
            output_format=args.format,
            decimal_precision=args.decimal_precision,
            relative_time=not args.absolute_time,
        )
        return _current_state

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
        pass
    finally:
        print("\nStopping recording...")
        # Use the global state reference
        state = _current_state
        # Finalize HDF5 recording with proper data structure
        if state.output_format == "hdf5":
            finalize_hdf5_recording(state)
        # Clean up files
        if hasattr(state, "csv_file") and state.csv_file:
            state.csv_file.close()
        if hasattr(state, "hdf5_file") and state.hdf5_file:
            state.hdf5_file.close()
        print(
            f"Recording stopped. Total poses recorded: "
            f"{state.pose_count if hasattr(state, 'pose_count') else 'unknown'}"
        )
        print(f"Data saved to: {output_file}")
