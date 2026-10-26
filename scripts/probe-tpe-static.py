#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Capture pinned native static-model motion and an unchanged Model original."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_original(root):
    """Require the selected original to run and complete without skipped/error cases."""
    suites = root.findall("testsuite")
    cases = root.findall(".//testcase")
    if root.tag != "testsuites" or len(suites) != 1 or len(cases) != 1:
        raise ValueError("native original selection differs")
    for node in [root, suites[0]]:
        if node.get("tests") != "1" or any(node.get(key, "0") != "0" for key in ["failures", "disabled", "errors", "skipped"]):
            raise ValueError("native original did not pass")
    case = cases[0]
    if suites[0].get("name") != "Model" or case.get("classname") != "Model" or case.get("name") != "BasicAPI":
        raise ValueError("native original identity differs")
    if case.get("status") != "run" or case.get("result") != "completed" or list(case):
        raise ValueError("native original did not complete")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ["physics-source", "physics-build", "dependency-prefix", "work-dir", "output"]:
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    source, build, prefix, work = [getattr(args, name).resolve() for name in ["physics_source", "physics_build", "dependency_prefix", "work_dir"]]
    revision = next(r["commit"] for r in json.loads((ROOT / "planning/baseline.json").read_text())["repositories"] if r["name"] == "gz-physics")
    git_env = dict(os.environ, GIT_OPTIONAL_LOCKS="0")
    for key in subprocess.check_output(["git", "rev-parse", "--local-env-vars"], env=git_env, text=True).splitlines():
        git_env.pop(key, None)
    actual = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], env=git_env, text=True).strip()
    if actual != revision:
        raise ValueError("native physics revision differs")
    names = ["tpe/lib/src/Entity.cc", "tpe/lib/src/Entity.hh", "tpe/lib/src/Model.cc", "tpe/lib/src/Model.hh", "tpe/lib/src/Model_TEST.cc", "tpe/lib/src/World.cc", "tpe/lib/src/World.hh"]
    hashes = {}
    for name in names:
        expected = subprocess.check_output(["git", "-C", str(source), "show", revision + ":" + name], env=git_env)
        if expected != (source / name).read_bytes():
            raise ValueError("native source differs: " + name)
        hashes[name] = digest(source / name)
    cache = (build / "CMakeCache.txt").read_text()
    home = next(s.split("=", 1)[1] for s in cache.splitlines() if s.startswith("CMAKE_HOME_DIRECTORY:"))
    if Path(home).resolve() != source:
        raise ValueError("native build belongs to another source")
    work.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, LD_LIBRARY_PATH=os.pathsep.join(map(str, [build / "lib", prefix / "lib"])), PKG_CONFIG_PATH=str(prefix / "lib/pkgconfig"))
    flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "--libs", "gz-math", "gz-utils"], env=env, text=True))
    probe = ROOT / "crates/gz-sim-scene/tests/native_tpe_static.cc"
    binary = work / "native-static"
    command = ["c++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-I" + str(source / "tpe/lib/src"), "-I" + str(build / "include"), str(probe), "-L" + str(build / "lib"), "-lgz-physics-tpelib", *flags, "-o", str(binary)]
    subprocess.run(command, env=env, check=True)
    result = subprocess.run([str(binary)], env=env, capture_output=True, text=True)
    (work / "controls.stdout").write_text(result.stdout)
    (work / "controls.stderr").write_text(result.stderr)
    result.check_returncode()
    if result.stderr:
        raise ValueError("unexpected native stderr")
    rows = json.loads(result.stdout)
    if len(rows) != 6 or [r["static"] for r in rows] != [False] * 3 + [True] * 3:
        raise ValueError("native static control rows differ")
    if [r["iteration"] for r in rows] != [1, 2, 3] * 2:
        raise ValueError("native static iteration sequence differs")
    for dynamic, fixed in zip(rows[:3], rows[3:]):
        for field in ["position", "orientation", "velocity", "angular_velocity"]:
            if dynamic[field] != fixed[field]:
                raise ValueError("native static/dynamic field differs: " + field)
        if abs(dynamic["position"][0] - dynamic["iteration"] * 0.01) > 1e-12 or dynamic["position"][1:] != [0, 5]:
            raise ValueError("pinned native translation differs")
        if dynamic["velocity"] != [1, 0, 0] or dynamic["angular_velocity"] != [0, 0, 1] or dynamic["orientation"] == [0, 0, 0, 1]:
            raise ValueError("pinned native stored-motion behavior differs")
    original = build / "bin/UNIT_tpelib_Model_TEST"
    xml = work / "original.xml"
    result = subprocess.run([str(original), "--gtest_filter=Model.BasicAPI", "--gtest_output=xml:" + str(xml)], env=env, capture_output=True, text=True)
    (work / "original.stdout").write_text(result.stdout)
    (work / "original.stderr").write_text(result.stderr)
    result.check_returncode()
    parsed = ET.parse(xml).getroot()
    check_original(parsed)
    linkage = subprocess.check_output(["ldd", str(binary)], env=env, text=True)
    libraries = {str(Path(p).resolve()): digest(Path(p)) for p in linkage.split() if p.startswith("/") and Path(p).is_file()}
    native = [p for p in libraries if Path(p).name.startswith("libgz-physics-tpelib.so.")]
    if len(native) != 1 or libraries[native[0]] != digest(build / "lib" / Path(native[0]).name):
        raise ValueError("native TPE library differs")
    report = {"scope": "Direct pinned native World/Model stored-velocity static controls, not a complete simulator/static command API test. Original Model.BasicAPI unchanged. Installed dependency/build provenance remains qualified; corrected Rust static policy intentionally differs.", "upstream": revision, "sources_sha256": hashes, "probe_sha256": digest(probe), "runner_sha256": digest(Path(__file__)), "command": command, "compiler": subprocess.check_output(["c++", "--version"], text=True), "cmake_cache_sha256": digest(build / "CMakeCache.txt"), "binary_sha256": digest(binary), "original_binary_sha256": digest(original), "original_xml_sha256": digest(xml), "original_tests": 1, "libraries_sha256": libraries, "observations": rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print("Six native static-motion observations and unchanged Model.BasicAPI passed")


if __name__ == "__main__":
    main()
