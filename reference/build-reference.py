#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Build a pinned native reference SDK and retain its provenance; no test-pass credit."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_lock(lock, baseline):
    pins = {row["name"]: row for row in baseline["repositories"]}
    names = [row["name"] for row in lock["repositories"]]
    expected = set(pins) - {"gz-gui"}
    if len(names) != len(set(names)) or set(names) != expected:
        raise ValueError("reference repository selection differs from baseline")
    if not re.fullmatch(r"docker.io/library/ubuntu@sha256:[0-9a-f]{64}", lock["base_image"]):
        raise ValueError("reference base must use a digest")
    if not re.fullmatch(r"\d{8}T\d{6}Z", lock["ubuntu_snapshot"]):
        raise ValueError("reference snapshot must be fixed")
    if not lock["apt_dependencies"] or any(not re.fullmatch(r"[a-z0-9][a-z0-9+.-]*", p) for p in lock["apt_dependencies"]):
        raise ValueError("invalid reference package selection")
    for row in lock["repositories"]:
        pin = pins[row["name"]]
        if row["url"] != pin["upstream_url"] or row["sha"] != pin["commit"] or not re.fullmatch(r"[0-9a-f]{40}", row["sha"]):
            raise ValueError("reference source differs from baseline: " + row["name"])
        flags = lock["cmake_common"] | row["cmake"]
        if flags.get("BUILD_TESTING") != "ON" or flags.get("CMAKE_BUILD_TYPE") != "Release":
            raise ValueError("reference requires Release with tests retained")
        if flags.get("CMAKE_C_COMPILER") != "/usr/bin/gcc" or flags.get("CMAKE_CXX_COMPILER") != "/usr/bin/g++":
            raise ValueError("reference compiler selection differs")
    return lock


def git_environment():
    env = os.environ.copy()
    keys = subprocess.check_output(["git", "rev-parse", "--local-env-vars"], text=True).splitlines()
    for key in keys:
        env.pop(key, None)
    env.update(GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL="/dev/null", GIT_TERMINAL_PROMPT="0", GIT_OPTIONAL_LOCKS="0", PYTHONDONTWRITEBYTECODE="1")
    return env


def validate_recipe(lock, bootstrap, recipe):
    snapshots = set(re.findall(r"snapshot\.ubuntu\.com/ubuntu/(\d{8}T\d{6}Z)/", recipe))
    package = bootstrap["package"]
    bases = [line.split()[1] for line in recipe.splitlines() if line.startswith("FROM ")]
    if bases != [lock["base_image"], "reference-deps"] or snapshots != {lock["ubuntu_snapshot"]}:
        raise ValueError("recipe base/snapshot differs from lock")
    if "RUN sh /reference/reference/apt-sources.sh /etc/apt " + lock["ubuntu_snapshot"] + "\n" not in recipe:
        raise ValueError("recipe APT source snapshot differs")
    if bootstrap["snapshot"] != lock["ubuntu_snapshot"] or not lock["base_image"].endswith("@" + bootstrap["base_digest"]):
        raise ValueError("bootstrap metadata differs from lock")
    if "--checksum=sha256:" + package["SHA256"] not in recipe or package["Filename"] not in recipe:
        raise ValueError("recipe bootstrap package differs from signed metadata")


def provenance_inputs(root):
    names = ["reference/Dockerfile", "reference/Dockerfile.dockerignore", "reference/apt-sources.sh", "reference/build-reference.py", "reference/headless-tpe.lock.json", "reference/ubuntu-bootstrap.json", "planning/baseline.json", "planning/reference/non-gtest-registration-contracts.json", "scripts/inventory-native-tests.py", "LICENSE", "NOTICE"]
    return {name: digest(root / name) for name in names}


def verify_inputs(root, initial):
    if provenance_inputs(root) != initial:
        raise ValueError("reference input changed during build")


def retained_listing(command, path, env):
    result = subprocess.run(command, env=env, capture_output=True)
    path.write_bytes(result.stdout)
    path.with_suffix(".stderr").write_bytes(result.stderr)
    result.check_returncode()
    return json.loads(result.stdout)


def build_environment(prefix):
    env = git_environment()
    for key in ["CC", "CXX", "CFLAGS", "CXXFLAGS", "CPPFLAGS", "LDFLAGS", "CMAKE_TOOLCHAIN_FILE", "CPATH", "CPLUS_INCLUDE_PATH", "C_INCLUDE_PATH", "LIBRARY_PATH", "COMPILER_PATH", "GCC_EXEC_PREFIX", "PKG_CONFIG_LIBDIR", "PKG_CONFIG_SYSROOT_DIR"]:
        env.pop(key, None)
    env.update(CMAKE_PREFIX_PATH=str(prefix), PKG_CONFIG_PATH=str(prefix / "lib/pkgconfig"), LD_LIBRARY_PATH=str(prefix / "lib"))
    env["PATH"] = str(prefix / "bin") + os.pathsep + env["PATH"]
    return env


def output(command, env):
    return subprocess.check_output(command, env=env, text=True).strip()


def verify_source(source, row, env):
    if Path(output(["git", "-C", str(source), "rev-parse", "--show-toplevel"], env)).resolve() != source.resolve():
        raise ValueError("reference source root differs")
    if output(["git", "-C", str(source), "rev-parse", "HEAD"], env) != row["sha"]:
        raise ValueError("reference source revision differs")
    if output(["git", "-C", str(source), "status", "--porcelain", "--untracked-files=all", "--ignored=matching"], env):
        raise ValueError("reference source is dirty")
    if any(line.startswith("160000 ") for line in output(["git", "-C", str(source), "ls-tree", "-r", "HEAD"], env).splitlines()):
        raise ValueError("reference submodules require an explicit source lock")


def files_under(directory):
    result = {}
    for path in sorted(directory.rglob("*")):
        name = str(path.relative_to(directory))
        if path.is_symlink():
            if not path.resolve(strict=True).is_relative_to(directory.resolve()):
                raise ValueError("reference install symlink escapes prefix: " + name)
            result[name] = {"symlink": os.readlink(path)}
        elif path.is_file():
            result[name] = {"sha256": digest(path)}
    return result


def run(command, log, env):
    with log.open("a") as stream:
        stream.write(json.dumps(list(map(str, command))) + "\n")
        stream.flush()
        result = subprocess.run(list(map(str, command)), env=env, stdout=stream, stderr=subprocess.STDOUT)
    if result.returncode:
        print("\n".join(log.read_text(errors="replace").splitlines()[-80:]), flush=True)
        raise subprocess.CalledProcessError(result.returncode, command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path("/reference"))
    parser.add_argument("--jobs", type=int, default=2)
    args = parser.parse_args()
    if not 1 <= args.jobs <= 4:
        parser.error("reference jobs must be between one and four")
    root = args.root.resolve()
    initial_inputs = provenance_inputs(root)
    if digest(Path(__file__)) != initial_inputs["reference/build-reference.py"]:
        raise ValueError("running builder differs from copied reference input")
    lock_path = root / "reference/headless-tpe.lock.json"
    baseline_path = root / "planning/baseline.json"
    lock = validate_lock(json.loads(lock_path.read_text()), json.loads(baseline_path.read_text()))
    if lock["baseline_sha256"] != digest(baseline_path):
        raise ValueError("reference baseline bytes differ")
    validate_recipe(lock, json.loads((root / "reference/ubuntu-bootstrap.json").read_text()), (root / "reference/Dockerfile").read_text())
    prefix = root / "install"
    if prefix.exists() or (root / "builds").exists():
        raise ValueError("reference requires fresh build and install directories")
    env = build_environment(prefix)
    logs = root / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    records = []
    for row in lock["repositories"]:
        name = row["name"]
        source, build = root / "sources" / name, root / "builds" / name
        log = logs / (name + ".log")
        print("Building pinned reference " + name, flush=True)
        if not source.exists():
            source.mkdir(parents=True)
            run(["git", "init", "--quiet", source], log, env)
            run(["git", "-C", source, "-c", "credential.helper=", "fetch", "--depth", "1", row["url"], row["sha"]], log, env)
            run(["git", "-C", source, "checkout", "--detach", "FETCH_HEAD"], log, env)
        verify_source(source, row, env)
        flags = lock["cmake_common"] | row["cmake"] | {"CMAKE_INSTALL_PREFIX": str(prefix)}
        run(["cmake", "-S", source, "-B", build, "-G", "Unix Makefiles", *[f"-D{k}={v}" for k, v in flags.items()]], log, env)
        cache_path = build / "CMakeCache.txt"
        cache = dict(line.split("=", 1) for line in cache_path.read_text().splitlines() if "=" in line and not line.startswith(("#", "//")))
        for key, value in flags.items():
            actual = [v for k, v in cache.items() if k.split(":", 1)[0] == key]
            if actual != [value]:
                raise ValueError("reference CMake flag differs: " + name + ":" + key)
        run(["cmake", "--build", build, "--parallel", str(args.jobs)], log, env)
        run(["cmake", "--install", build], log, env)
        verify_source(source, row, env)
        listing_path = logs / (name + "-ctest.json")
        parsed = retained_listing(["ctest", "--test-dir", str(build), "--show-only=json-v1"], listing_path, env)
        compilers = {}
        for key in ["CMAKE_C_COMPILER", "CMAKE_CXX_COMPILER"]:
            path = next(v for k, v in cache.items() if k.split(":", 1)[0] == key)
            compilers[key] = {"path": path, "sha256": digest(Path(path).resolve()), "version": output([path, "--version"], env)}
        records.append({"name": name, "sha": row["sha"], "flags": flags, "compilers": compilers, "cmake_cache_sha256": digest(cache_path), "build_log_sha256": digest(log), "ctest_json_sha256": digest(listing_path), "ctest_stderr_sha256": digest(listing_path.with_suffix(".stderr")), "registered_tests": len(parsed["tests"])})
    installed = files_under(prefix)
    for required in ["libgz-sim.so", "libgz-sim-physics-system.so", "libgz-physics-tpelib.so", "libgz-physics-tpe-plugin.so", "libgz-sensors-imu.so", "libgz-transport.so"]:
        if not any(Path(name).name.startswith(required) for name in installed):
            raise ValueError("required native reference library missing: " + required)
    verify_inputs(root, initial_inputs)
    manifest = {"scope": "Clean headless TPE build only; registered tests are not execution-pass credit. Other backend, renderer and language profiles remain unqualified.", "inputs_sha256": initial_inputs, "build_jobs": args.jobs, "lock_sha256": initial_inputs["reference/headless-tpe.lock.json"], "baseline_sha256": initial_inputs["planning/baseline.json"], "builder_sha256": initial_inputs["reference/build-reference.py"], "base_image": lock["base_image"], "ubuntu_snapshot": lock["ubuntu_snapshot"], "tools_sha256": {name: digest(Path(shutil.which(name, path=env["PATH"])).resolve()) for name in ["c++", "cmake", "git", "python3"]}, "compiler": output(["c++", "--version"], env), "cmake": output(["cmake", "--version"], env), "packages": output(["dpkg-query", "-W", "-f=${binary:Package}\t${Version}\t${Architecture}\n"], env), "repositories": records, "installed_files": installed}
    (root / "environment.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
