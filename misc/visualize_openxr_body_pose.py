"""
Visualize body pose data from a standard OpenXR CSV file.

Assumptions:
- CSV was recorded via examples/record_body_pose.py (OpenXR format).
- bone_id is an integer matching FullBodyBoneId values.
- Positions are already in right-handed Z-up coordinates.

CSV columns (one row per bone per frame):
    time_elapsed,bone_id,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w
or (absolute time variant):
    timestamp,datetime,bone_id,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w
"""

import argparse
import csv
import time
from collections import defaultdict
from dataclasses import dataclass

import numpy as np
import rerun as rr
from matplotlib import colormaps as cm

from xr_robot_teleop_server.schemas.openxr_skeletons import (
    FULL_BODY_SKELETON_CONNECTIONS,
    FullBodyBoneId,
    SkeletonType,
)

# Visualization params
VIZ_POINT_RADIUS = 0.01


@dataclass
class BoneData:
    """Represents bone position and rotation data."""

    id: FullBodyBoneId
    position: tuple[float, float, float]


def parse_openxr_csv(csv_file: str) -> tuple[list[float], dict[int, list[BoneData]]]:
    """Parse OpenXR CSV and group rows by time into frames."""
    groups: dict[float, list[BoneData]] = defaultdict(list)

    with open(csv_file, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Time key
            if "time_elapsed" in row and row["time_elapsed"]:
                t = float(row["time_elapsed"])
            elif "timestamp" in row and row["timestamp"]:
                t = float(row["timestamp"])  # absolute seconds
            else:
                t = 0.0

            # Bone id and position
            try:
                bone_id = FullBodyBoneId(int(row["bone_id"]))
                px = float(row["pos_x"])  # meters, Z-up
                py = float(row["pos_y"])
                pz = float(row["pos_z"])
            except Exception:
                continue

            groups[t].append(BoneData(bone_id, (px, py, pz)))

    times_sorted = sorted(groups.keys())
    frame_map: dict[int, list[BoneData]] = {i: groups[t] for i, t in enumerate(times_sorted)}
    return times_sorted, frame_map


def log_annotation_context() -> None:
    """Configure rerun with FullBody skeleton annotations and connections."""
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


def visualize_frame(bones: list[BoneData], time_key: float | int, seq_name: str) -> None:
    """Visualize a single frame of bone data."""
    if not bones:
        return
    # rr.set_time_sequence(seq_name, time_key)
    # rr.set_time(seq_name, duration=time_key)  # not sure
    # rr.set_time(seq_name, timestamp=time_key)  # not sure
    rr.set_time(seq_name, sequence=time_key)  # not sure
    positions = [b.position for b in bones]
    keypoint_ids = [b.id.value for b in bones]
    rr.log(
        "world/user/bones",
        rr.Points3D(
            positions=positions,
            keypoint_ids=keypoint_ids,
            class_ids=SkeletonType.FullBody.value,
            radii=VIZ_POINT_RADIUS,
        ),
    )


def main():
    parser = argparse.ArgumentParser(description="Visualize OpenXR CSV body pose")
    parser.add_argument("--file", type=str, required=True, help="Path to OpenXR CSV file")
    args = parser.parse_args()

    # Load CSV
    times, frames = parse_openxr_csv(args.file)

    # Init rerun
    rr.init("openxr-body-pose-visualizer", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    log_annotation_context()

    # Playback
    for i in range(len(times)):
        visualize_frame(frames.get(i, []), i, "time")

if __name__ == "__main__":
    main()
