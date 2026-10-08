#!/usr/bin/env python3
"""Validate the public TUM-format contract for the 2026 SLAM challenge."""

import argparse
import math
from pathlib import Path
from typing import Dict, Union


def validate(path: Union[str, Path]) -> Dict[str, float]:
    path = Path(path)
    if not path.is_file():
        raise ValueError(f"missing trajectory: {path}")

    count = 0
    first_stamp = None
    previous_stamp = None
    minimum_quaternion_norm = math.inf
    maximum_quaternion_norm = 0.0

    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            columns = line.split()
            if len(columns) != 8:
                raise ValueError(
                    f"{path}:{line_number}: expected 8 columns, got {len(columns)}"
                )
            try:
                values = [float(value) for value in columns]
            except ValueError as error:
                raise ValueError(f"{path}:{line_number}: non-numeric value") from error
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"{path}:{line_number}: NaN or Inf is forbidden")

            stamp = values[0]
            if previous_stamp is not None and stamp <= previous_stamp:
                raise ValueError(
                    f"{path}:{line_number}: timestamp {stamp:.15f} is not greater "
                    f"than {previous_stamp:.15f}"
                )
            quaternion_norm = math.sqrt(sum(value * value for value in values[4:8]))
            if quaternion_norm <= 1e-12:
                raise ValueError(f"{path}:{line_number}: zero quaternion is forbidden")

            first_stamp = stamp if first_stamp is None else first_stamp
            previous_stamp = stamp
            minimum_quaternion_norm = min(minimum_quaternion_norm, quaternion_norm)
            maximum_quaternion_norm = max(maximum_quaternion_norm, quaternion_norm)
            count += 1

    if count == 0:
        raise ValueError(f"empty trajectory: {path}")

    return {
        "poses": float(count),
        "first_stamp": float(first_stamp),
        "last_stamp": float(previous_stamp),
        "duration": float(previous_stamp - first_stamp),
        "minimum_quaternion_norm": minimum_quaternion_norm,
        "maximum_quaternion_norm": maximum_quaternion_norm,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory", nargs="+", help="TUM trajectory file(s)")
    args = parser.parse_args()

    failed = False
    for trajectory in args.trajectory:
        try:
            summary = validate(trajectory)
            print(
                f"OK {trajectory}: poses={int(summary['poses'])} "
                f"duration={summary['duration']:.3f}s "
                f"qnorm=[{summary['minimum_quaternion_norm']:.6f},"
                f"{summary['maximum_quaternion_norm']:.6f}]"
            )
        except ValueError as error:
            failed = True
            print(f"ERROR {error}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
