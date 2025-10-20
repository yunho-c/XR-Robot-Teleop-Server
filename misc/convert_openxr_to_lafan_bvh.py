"""Convert OpenXR body pose recordings into LAFAN-1 style BVH files.

The conversion expects the CSV format produced by
``examples/record_body_pose.py --format csv`` which stores one bone per row
with world-space position and orientation data in a right-handed, Z-up
coordinate system.  The output BVH uses the 22-joint hierarchy that ships with
the LAFAN-1 dataset so that recordings can be evaluated with animation tools
and research code that rely on that skeleton definition.

Example
-------
```bash
python misc/convert_openxr_to_lafan_bvh.py \
    --input recordings/sample_body_pose_data.csv \
    --output recordings/sample_body_pose_data.bvh
```
"""

from __future__ import annotations

import argparse
import csv
from collections import OrderedDict
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as R

from visualize_kinematic_tree import LAFAN_TO_FULLBODY_MAPPING
from xr_robot_teleop_server.schemas.openxr_skeletons import FullBodyBoneId


# ---------------------------------------------------------------------------
# Skeleton definition

LAFAN_JOINT_HIERARCHY: Sequence[tuple[str, Optional[str]]] = (
    ("Hips", None),
    ("Spine", "Hips"),
    ("Spine1", "Spine"),
    ("Spine2", "Spine1"),
    ("Neck", "Spine2"),
    ("Head", "Neck"),
    ("LeftShoulder", "Spine2"),
    ("LeftArm", "LeftShoulder"),
    ("LeftForeArm", "LeftArm"),
    ("LeftHand", "LeftForeArm"),
    ("RightShoulder", "Spine2"),
    ("RightArm", "RightShoulder"),
    ("RightForeArm", "RightArm"),
    ("RightHand", "RightForeArm"),
    ("LeftUpLeg", "Hips"),
    ("LeftLeg", "LeftUpLeg"),
    ("LeftFoot", "LeftLeg"),
    ("LeftToe", "LeftFoot"),
    ("RightUpLeg", "Hips"),
    ("RightLeg", "RightUpLeg"),
    ("RightFoot", "RightLeg"),
    ("RightToe", "RightFoot"),
)


FULLBODY_TO_LAFAN: Dict[FullBodyBoneId, str] = {
    full_body_id: joint
    for joint, full_body_id in LAFAN_TO_FULLBODY_MAPPING.items()
}


# ---------------------------------------------------------------------------
# Coordinate conversions


def position_zup_to_yup(position: Sequence[float]) -> np.ndarray:
    """Convert a right-handed Z-up vector into a Y-up vector."""

    x, y, z = position
    return np.array([x, z, -y], dtype=float)


def quaternion_zup_to_yup(quaternion: Sequence[float]) -> np.ndarray:
    """Convert a right-handed Z-up quaternion into a Y-up quaternion."""

    x, y, z, w = quaternion
    return np.array([x, z, -y, w], dtype=float)


# ---------------------------------------------------------------------------
# Loading OpenXR CSV data


def parse_float(value: str) -> float:
    try:
        return float(value)
    except ValueError as exc:  # pragma: no cover - defensive
        raise ValueError(f"Unable to parse float from value '{value}'") from exc


FrameData = Dict[FullBodyBoneId, Dict[str, np.ndarray]]


def load_openxr_csv(input_file: Path) -> tuple[List[float], List[FrameData]]:
    """Load OpenXR CSV recording into per-frame dictionaries."""

    with input_file.open() as fh:
        reader = csv.DictReader(fh)
        if reader.fieldnames is None:
            raise ValueError("CSV file is missing a header row")

        time_field = None
        for candidate in ("time_elapsed", "timestamp"):
            if candidate in reader.fieldnames:
                time_field = candidate
                break

        if time_field is None:
            raise ValueError(
                "Input CSV must contain either a 'time_elapsed' or 'timestamp' column"
            )

        frames: "OrderedDict[float, FrameData]" = OrderedDict()

        for row in reader:
            time_value = parse_float(row[time_field])

            bone_id_raw = row["bone_id"].strip()
            full_body_id = None

            try:
                bone_id = int(bone_id_raw)
            except ValueError:
                bone_id = None

            if bone_id is not None:
                full_body_id = FullBodyBoneId._value2member_map_.get(bone_id)

            if full_body_id is None:
                # Support string-based bone IDs as a fallback (e.g., 'FullBody_Hips')
                full_body_id = FullBodyBoneId.__members__.get(bone_id_raw)

            if full_body_id is None:
                # Skip joints that are not part of the OpenXR enum
                continue

            if full_body_id not in FULLBODY_TO_LAFAN:
                # Skip bones that are not part of the LAFAN skeleton
                continue

            position = np.array(
                [
                    parse_float(row["pos_x"]),
                    parse_float(row["pos_y"]),
                    parse_float(row["pos_z"]),
                ],
                dtype=float,
            )
            quaternion = np.array(
                [
                    parse_float(row["rot_x"]),
                    parse_float(row["rot_y"]),
                    parse_float(row["rot_z"]),
                    parse_float(row["rot_w"]),
                ],
                dtype=float,
            )

            if time_value not in frames:
                frames[time_value] = {}

            frames[time_value][full_body_id] = {
                "position": position,
                "rotation": quaternion,
            }

    frame_times = list(frames.keys())
    frame_data = list(frames.values())

    return frame_times, frame_data


# ---------------------------------------------------------------------------
# BVH generation helpers


def build_children_map(hierarchy: Sequence[tuple[str, Optional[str]]]) -> Dict[str, List[str]]:
    children: Dict[str, List[str]] = {joint: [] for joint, _ in hierarchy}
    for joint, parent in hierarchy:
        if parent:
            children[parent].append(joint)
    return children


def compute_offsets(first_frame: FrameData, center_root: bool = True) -> Dict[str, np.ndarray]:
    """Compute joint offsets from the first frame positions."""

    offsets: Dict[str, np.ndarray] = {}
    yup_positions: Dict[str, np.ndarray] = {}

    for joint, parent in LAFAN_JOINT_HIERARCHY:
        full_body_id = LAFAN_TO_FULLBODY_MAPPING[joint]
        try:
            bone_data = first_frame[full_body_id]
        except KeyError as exc:
            raise KeyError(f"Missing bone data for joint '{joint}' in first frame") from exc

        pos_yup = position_zup_to_yup(bone_data["position"])
        yup_positions[joint] = pos_yup

        if parent is None:
            offsets[joint] = pos_yup if not center_root else np.zeros(3, dtype=float)
        else:
            offsets[joint] = pos_yup - yup_positions[parent]

    return offsets


def estimate_frame_time(frame_times: Sequence[float], fps_override: Optional[float]) -> float:
    if fps_override is not None:
        return 1.0 / fps_override

    if len(frame_times) < 2:
        return 1.0 / 60.0

    deltas = np.diff(sorted(frame_times))
    mean_delta = float(np.mean(deltas))
    if mean_delta <= 0:
        return 1.0 / 60.0
    return mean_delta


def joint_traversal_order(children: Mapping[str, Sequence[str]]) -> List[str]:
    order: List[str] = []

    def recurse(joint: str) -> None:
        order.append(joint)
        for child in children[joint]:
            recurse(child)

    recurse("Hips")
    return order


def rotation_channels(rotation_order: str) -> List[str]:
    return [f"{axis.upper()}rotation" for axis in rotation_order]


def format_offset(vector: Sequence[float]) -> str:
    return "{:.6f} {:.6f} {:.6f}".format(*vector)


def write_bvh_hierarchy(
    fh,
    offsets: Mapping[str, np.ndarray],
    children: Mapping[str, Sequence[str]],
    rotation_order: str,
) -> None:
    indent = "  "

    def write_joint(name: str, depth: int, is_root: bool = False) -> None:
        prefix = indent * depth
        joint_type = "ROOT" if is_root else "JOINT"
        fh.write(f"{prefix}{joint_type} {name}\n")
        fh.write(f"{prefix}{{\n")
        fh.write(f"{prefix}{indent}OFFSET {format_offset(offsets[name])}\n")

        if is_root:
            channels = ["Xposition", "Yposition", "Zposition"] + rotation_channels(rotation_order)
        else:
            channels = rotation_channels(rotation_order)
        fh.write(f"{prefix}{indent}CHANNELS {len(channels)} {' '.join(channels)}\n")

        if children[name]:
            for child in children[name]:
                write_joint(child, depth + 1)
        else:
            fh.write(f"{prefix}{indent}End Site\n")
            fh.write(f"{prefix}{indent}{{\n")
            fh.write(f"{prefix}{indent}{indent}OFFSET 0.000000 0.000000 0.000000\n")
            fh.write(f"{prefix}{indent}}}\n")

        fh.write(f"{prefix}}}\n")

    fh.write("HIERARCHY\n")
    write_joint("Hips", depth=0, is_root=True)


def compute_local_rotations(frame: FrameData) -> Dict[str, R]:
    global_rots: Dict[str, R] = {}
    local_rots: Dict[str, R] = {}

    for joint, parent in LAFAN_JOINT_HIERARCHY:
        full_body_id = LAFAN_TO_FULLBODY_MAPPING[joint]
        if full_body_id not in frame:
            raise KeyError(f"Missing bone data for joint '{joint}' in frame")

        quat_yup = quaternion_zup_to_yup(frame[full_body_id]["rotation"])
        quat_yup /= np.linalg.norm(quat_yup) + 1e-8

        global_rot = R.from_quat(quat_yup)
        global_rots[joint] = global_rot

        if parent is None:
            local_rots[joint] = global_rot
        else:
            parent_rot = global_rots[parent]
            local_rots[joint] = parent_rot.inv() * global_rot

    return local_rots


def compute_root_translation(frame: FrameData, root_reference: np.ndarray | None) -> np.ndarray:
    root_data = frame[LAFAN_TO_FULLBODY_MAPPING["Hips"]]
    pos_yup = position_zup_to_yup(root_data["position"])
    if root_reference is None:
        return pos_yup
    return pos_yup - root_reference


def build_motion_data(
    frames: Sequence[FrameData],
    traversal: Sequence[str],
    rotation_order: str,
    center_root: bool,
) -> tuple[np.ndarray, np.ndarray]:
    root_reference = None
    if center_root and frames:
        root_reference = position_zup_to_yup(
            frames[0][LAFAN_TO_FULLBODY_MAPPING["Hips"]]["position"]
        )

    motion_rows: List[List[float]] = []

    for frame in frames:
        local_rotations = compute_local_rotations(frame)
        row_values: List[float] = []

        root_translation = compute_root_translation(frame, root_reference)
        row_values.extend(root_translation.tolist())

        root_local_rot = local_rotations["Hips"].as_euler(rotation_order, degrees=True)
        row_values.extend(root_local_rot.tolist())

        for joint in traversal[1:]:
            euler = local_rotations[joint].as_euler(rotation_order, degrees=True)
            row_values.extend(euler.tolist())

        motion_rows.append(row_values)

    return np.array(motion_rows, dtype=float)


def write_bvh(
    output_file: Path,
    frame_times: Sequence[float],
    frames: Sequence[FrameData],
    rotation_order: str = "ZXY",
    fps: Optional[float] = None,
    center_root: bool = True,
) -> None:
    if not frames:
        raise ValueError("No frame data found in input CSV")

    offsets = compute_offsets(frames[0], center_root=center_root)
    children = build_children_map(LAFAN_JOINT_HIERARCHY)
    traversal = joint_traversal_order(children)

    motion_data = build_motion_data(frames, traversal, rotation_order, center_root)
    frame_time = estimate_frame_time(frame_times, fps)

    with output_file.open("w", encoding="utf-8") as fh:
        write_bvh_hierarchy(fh, offsets, children, rotation_order)

        fh.write("MOTION\n")
        fh.write(f"Frames: {len(frames)}\n")
        fh.write(f"Frame Time: {frame_time:.6f}\n")

        for row in motion_data:
            fh.write(" ".join(f"{value:.6f}" for value in row) + "\n")


# ---------------------------------------------------------------------------
# Command-line interface


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert OpenXR CSV recordings into LAFAN-1 style BVH files",
    )
    parser.add_argument("--input", type=Path, required=True, help="Path to OpenXR CSV file")
    parser.add_argument("--output", type=Path, required=True, help="Path to output BVH file")
    parser.add_argument(
        "--fps",
        type=float,
        default=None,
        help="Override the output frame rate (frames per second)",
    )
    parser.add_argument(
        "--rotation-order",
        type=str,
        default="ZXY",
        help=(
            "Euler rotation order to use for BVH channels (e.g., ZXY, XYZ). "
            "Must be a permutation of X, Y, Z."
        ),
    )
    parser.add_argument(
        "--keep-root-global",
        action="store_true",
        help="Do not re-center the root to the origin; keep absolute hip positions",
    )
    return parser.parse_args(argv)


def validate_rotation_order(order: str) -> str:
    order = order.upper()
    if sorted(order) != ["X", "Y", "Z"]:
        raise ValueError("Rotation order must be a permutation of 'XYZ'")
    return order


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parse_args(argv)
    rotation_order = validate_rotation_order(args.rotation_order)

    frame_times, frames = load_openxr_csv(args.input)
    write_bvh(
        output_file=args.output,
        frame_times=frame_times,
        frames=frames,
        rotation_order=rotation_order,
        fps=args.fps,
        center_root=not args.keep_root_global,
    )

    print(f"Wrote BVH with {len(frames)} frames to {args.output}")


if __name__ == "__main__":
    main()
