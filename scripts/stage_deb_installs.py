#!/usr/bin/env python3
"""Stage selected ROS packages from a merged colcon install tree."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


def copy_path(src_root: Path, dst_root: Path, relative: Path) -> bool:
    src = src_root / relative
    if not src.exists() and not src.is_symlink():
        return False

    dst = dst_root / relative
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists() or dst.is_symlink():
        if dst.is_dir() and not dst.is_symlink():
            shutil.rmtree(dst)
        else:
            dst.unlink()

    if src.is_dir() and not src.is_symlink():
        shutil.copytree(src, dst, symlinks=True)
    else:
        shutil.copy2(src, dst, follow_symlinks=False)
    return True


def copy_glob(src_root: Path, dst_root: Path, pattern: str) -> int:
    copied = 0
    for src in src_root.glob(pattern):
        if copy_path(src_root, dst_root, src.relative_to(src_root)):
            copied += 1
    return copied


def stage_package(src_root: Path, dst_root: Path, package: str) -> None:
    copy_path(src_root, dst_root, Path("share") / package)
    copy_path(src_root, dst_root, Path("include") / package)
    copy_path(src_root, dst_root, Path("lib") / package)
    copy_glob(src_root, dst_root, f"lib/lib{package}*")
    copy_glob(src_root, dst_root, f"lib/python*/site-packages/{package}")
    copy_glob(src_root, dst_root, f"lib/python*/site-packages/{package}-*.egg-info")

    resource_root = src_root / "share" / "ament_index" / "resource_index"
    if resource_root.exists():
        for resource_dir in resource_root.iterdir():
            copy_path(
                src_root,
                dst_root,
                Path("share") / "ament_index" / "resource_index" / resource_dir.name / package,
            )

    copy_path(src_root, dst_root, Path("share") / "colcon-core" / "packages" / package)

    package_xml = dst_root / "share" / package / "package.xml"
    if not package_xml.exists():
        raise RuntimeError(f"{package}: missing staged share/{package}/package.xml")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("packages", nargs="+")
    args = parser.parse_args()

    if not args.source.exists():
        raise RuntimeError(f"source install tree does not exist: {args.source}")

    if args.destination.exists():
        shutil.rmtree(args.destination)
    args.destination.mkdir(parents=True)

    for package in args.packages:
        stage_package(args.source, args.destination, package)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
