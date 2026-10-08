#!/usr/bin/env python3
"""Create the exact ZIP layout required by LiDAR-SLAM-Challenge-2026."""

import argparse
import tempfile
import zipfile
from pathlib import Path

from validate_challenge_tum import validate


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-0001", required=True, type=Path)
    parser.add_argument("--scene-0002", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--team", default="TODO-team-name")
    parser.add_argument("--readme", type=Path)
    args = parser.parse_args()

    for trajectory in (args.scene_0001, args.scene_0002):
        summary = validate(trajectory)
        print(f"validated {trajectory}: {int(summary['poses'])} poses")

    if args.readme is not None:
        readme = args.readme.read_text(encoding="utf-8")
    else:
        readme = (
            f"# {args.team}\n\n"
            "Algorithm: SLICT2 adapted for Airy LiDAR and its built-in IMU.\n\n"
            "Trajectory convention: TUM, Airy IMU origin, original LiDAR header timestamps.\n"
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as temporary_directory:
        temporary = Path(temporary_directory)
        readme_path = temporary / "README.md"
        readme_path.write_text(readme, encoding="utf-8")
        with zipfile.ZipFile(args.output, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            archive.write(readme_path, "README.md")
            archive.write(args.scene_0001, "trajectories/scene_0001.txt")
            archive.write(args.scene_0002, "trajectories/scene_0002.txt")

    print(f"created {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
