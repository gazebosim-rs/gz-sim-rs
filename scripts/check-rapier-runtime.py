#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Inspect the selected CLI Cargo closure and the actual no-native executable."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
NATIVE_PACKAGES = {"gz-physics-dart", "gz-physics-mujoco"}
REQUIRED_PACKAGES = {"gz-sim-cli", "gz-sim-server", "gz-sim-scene", "gz-physics-rapier", "gz-physics-tpe"}
NATIVE_LIBRARIES = ("libdart", "libmujoco", "libode", "libgz-", "libsdformat")


def validate_closure(tree, libraries):
    packages = {line.split()[0] for line in tree.splitlines() if line.strip()}
    if not REQUIRED_PACKAGES <= packages or packages & NATIVE_PACKAGES:
        raise ValueError("CLI closure is incomplete or includes native physics engines")
    if "not found" in libraries or any(name in libraries for name in NATIVE_LIBRARIES):
        raise ValueError("Executable has missing libraries or a native engine/SDK dependency")
    return sorted(packages)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--features", help="Additional CLI features, retaining --no-default-features")
    args = parser.parse_args()
    command = ["cargo", "tree", "--locked", "--offline", "-p", "gz-sim-cli", "--no-default-features",
               "--edges", "normal,build", "--prefix", "none", "--format", "{p}"]
    if args.features:
        command += ["--features", args.features]
    tree = subprocess.check_output(command, cwd=ROOT, text=True, timeout=60)
    binary = args.binary.resolve(strict=True)
    libraries = subprocess.check_output(["ldd", str(binary)], cwd=ROOT, text=True, timeout=20)
    packages = validate_closure(tree, libraries)
    print(json.dumps({"binary": str(binary), "cargo_command": command,
                      "packages": packages, "libraries": libraries,
                      "scope": "Selected no-native CLI closure only; full default verification remains required"}, indent=2))


if __name__ == "__main__":
    main()
