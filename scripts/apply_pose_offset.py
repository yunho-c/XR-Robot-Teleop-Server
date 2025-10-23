#!/usr/bin/env python3
"""
Apply global translation and rotation offsets to OpenXR-style pose CSV files.

Each row in the input CSV is expected to contain the fields:
`time_elapsed,bone_id,pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,rot_w`.
The script rotates positions about the origin using the supplied quaternion
offset and then adds the translation offset. Orientations are left-multiplied
by the same quaternion. The resulting rows are written to stdout or an output
file.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from typing import Iterable, Sequence, Tuple


Quaternion = Tuple[float, float, float, float]
Vector3 = Tuple[float, float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Apply a global translation and rotation offset to pose data."
    )
    parser.add_argument("input", help="Path to the input pose CSV file.")
    parser.add_argument(
        "-o",
        "--output",
        default="-",
        help="Output CSV path (default: stdout).",
    )
    parser.add_argument(
        "--translate",
        nargs=3,
        type=float,
        default=(0.0, 0.0, 0.0),
        metavar=("TX", "TY", "TZ"),
        help="Translation offset applied to every position (default: 0 0 0).",
    )
    parser.add_argument(
        "--rotate",
        nargs=4,
        type=float,
        default=(0.0, 0.0, 0.0, 1.0),
        metavar=("RX", "RY", "RZ", "RW"),
        help="Quaternion rotation offset (x y z w). Default is identity.",
    )
    parser.add_argument(
        "--precision",
        type=int,
        default=6,
        help="Decimal precision used when writing floating point values.",
    )
    return parser.parse_args()


def normalize_quaternion(quat: Sequence[float]) -> Quaternion:
    qx, qy, qz, qw = quat
    mag = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if mag == 0:
        raise ValueError("Rotation quaternion magnitude must be non-zero.")
    return qx / mag, qy / mag, qz / mag, qw / mag


def quaternion_multiply(lhs: Quaternion, rhs: Quaternion) -> Quaternion:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate_vector(quat: Quaternion, vec: Vector3) -> Vector3:
    qx, qy, qz, qw = quat
    vx, vy, vz = vec
    dot_uv = qx * vx + qy * vy + qz * vz
    dot_uu = qx * qx + qy * qy + qz * qz
    cross_x = qy * vz - qz * vy
    cross_y = qz * vx - qx * vz
    cross_z = qx * vy - qy * vx

    term1_x = 2.0 * dot_uv * qx
    term1_y = 2.0 * dot_uv * qy
    term1_z = 2.0 * dot_uv * qz

    term2_factor = qw * qw - dot_uu
    term2_x = term2_factor * vx
    term2_y = term2_factor * vy
    term2_z = term2_factor * vz

    term3_x = 2.0 * qw * cross_x
    term3_y = 2.0 * qw * cross_y
    term3_z = 2.0 * qw * cross_z

    return (
        term1_x + term2_x + term3_x,
        term1_y + term2_y + term3_y,
        term1_z + term2_z + term3_z,
    )


def format_float(value: float, precision: int) -> str:
    if precision < 0:
        return str(value)
    formatted = f"{value:.{precision}f}"
    if precision > 0:
        formatted = formatted.rstrip("0").rstrip(".")
    if formatted in {"-0", "-0.0", ""}:
        return "0"
    return formatted


def process_rows(
    rows: Iterable[dict],
    translation: Vector3,
    rotation_offset: Quaternion,
    precision: int,
) -> Iterable[dict]:
    tx, ty, tz = translation
    for row in rows:
        try:
            pos = (
                float(row["pos_x"]),
                float(row["pos_y"]),
                float(row["pos_z"]),
            )
            rot = (
                float(row["rot_x"]),
                float(row["rot_y"]),
                float(row["rot_z"]),
                float(row["rot_w"]),
            )
        except KeyError as exc:
            raise KeyError(f"Missing expected field in row: {exc}") from exc
        except ValueError as exc:
            raise ValueError(f"Could not parse pose values in row: {row}") from exc

        rotated_pos = rotate_vector(rotation_offset, pos)
        translated_pos = (
            rotated_pos[0] + tx,
            rotated_pos[1] + ty,
            rotated_pos[2] + tz,
        )

        rotated_rot = quaternion_multiply(rotation_offset, rot)
        rotated_rot = normalize_quaternion(rotated_rot)

        row["pos_x"] = format_float(translated_pos[0], precision)
        row["pos_y"] = format_float(translated_pos[1], precision)
        row["pos_z"] = format_float(translated_pos[2], precision)
        row["rot_x"] = format_float(rotated_rot[0], precision)
        row["rot_y"] = format_float(rotated_rot[1], precision)
        row["rot_z"] = format_float(rotated_rot[2], precision)
        row["rot_w"] = format_float(rotated_rot[3], precision)
        yield row


def main() -> None:
    args = parse_args()
    translation = tuple(args.translate)  # type: ignore[arg-type]
    rotation_offset = normalize_quaternion(args.rotate)

    with open(args.input, newline="", encoding="utf-8") as src:
        reader = csv.DictReader(src)
        if not reader.fieldnames:
            raise ValueError("Input CSV must include a header row.")

        if args.output == "-":
            dest = sys.stdout
            close_dest = False
        else:
            dest = open(args.output, "w", newline="", encoding="utf-8")
            close_dest = True

        try:
            writer = csv.DictWriter(dest, fieldnames=reader.fieldnames)
            writer.writeheader()
            for row in process_rows(reader, translation, rotation_offset, args.precision):
                writer.writerow(row)
        finally:
            if close_dest:
                dest.close()


if __name__ == "__main__":
    main()

