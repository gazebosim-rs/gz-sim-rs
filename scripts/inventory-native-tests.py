#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""List available pinned CTest/gtest configurations without claiming test passes."""
import argparse
from datetime import datetime, timezone
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def configuration_key(command, properties):
    # Working directory and ordered environment operations can change registration.
    return digest(json.dumps([command, properties], sort_keys=True).encode())


def parse_cases(output):
    suite, cases, seen, suite_has_cases = None, [], set(), False
    for line in output.splitlines():
        text = line.split("#", 1)[0].rstrip()
        if not text or text.startswith("Running main() from "):
            continue
        if re.fullmatch(r"[A-Za-z0-9_./-]+\.", text):
            if suite is not None and not suite_has_cases:
                raise ValueError("empty_suite")
            suite, suite_has_cases = text[:-1], False
        elif suite and re.fullmatch(r"  [A-Za-z0-9_./-]+", text):
            name = text.strip()
            identity = suite + "." + name
            if identity in seen:
                raise ValueError("duplicate_case")
            seen.add(identity)
            suite_has_cases = True
            disabled = any(p.startswith("DISABLED_") for p in (suite + "/" + name).split("/"))
            cases.append({"id": identity, "disabled": disabled})
        else:
            raise ValueError("unrecognized_list_output")
    if suite is not None and not suite_has_cases:
        raise ValueError("empty_suite")
    if not cases:
        raise ValueError("empty_list")
    return cases


def environment_list(properties, name):
    value = properties.get(name, [])
    if not isinstance(value, list) or any(not isinstance(s, str) for s in value):
        raise ValueError("unsupported_environment_property")
    return value


def enumerate_job(job, raw, base_env):
    row, command, properties, cwd = job
    env = base_env.copy()
    binary = Path(command[0])
    try:
        for assignment in environment_list(properties, "ENVIRONMENT"):
            key, value = assignment.split("=", 1)
            if not key:
                raise ValueError("invalid_environment_assignment")
            env[key] = value
        modifications = environment_list(properties, "ENVIRONMENT_MODIFICATION")
        # CMake applies its own ordered modification/reset semantics after ENVIRONMENT.
        if modifications:
            prefix = ["cmake", "-E", "env"]
            for modification in modifications:
                prefix.extend(["--modify", modification])
            command = [*prefix, "--", *command]
        result = subprocess.run(command, cwd=cwd, env=env, capture_output=True, timeout=20)
        stem = raw / (row["repository"] + "-" + row["configuration"])
        stem.with_suffix(".stdout").write_bytes(result.stdout)
        stem.with_suffix(".stderr").write_bytes(result.stderr)
        row.update(stdout_sha256=digest(result.stdout), stderr_sha256=digest(result.stderr), exit_code=result.returncode)
        if result.returncode:
            return row | {"state": "enumeration_failed"}
        if "binary_sha256" in row and digest(binary.read_bytes()) != row["binary_sha256"]:
            return row | {"state": "enumeration_failed", "reason": "binary_changed"}
        cases = parse_cases(result.stdout.decode("utf-8"))
        return row | {"state": "listed", "cases": cases}
    except subprocess.TimeoutExpired as error:
        stem = raw / (row["repository"] + "-" + row["configuration"])
        stdout, stderr = error.stdout or b"", error.stderr or b""
        stem.with_suffix(".stdout").write_bytes(stdout)
        stem.with_suffix(".stderr").write_bytes(stderr)
        return row | {"state": "enumeration_failed", "reason": "TimeoutExpired", "stdout_sha256": digest(stdout), "stderr_sha256": digest(stderr)}
    except (OSError, ValueError) as error:
        # Raw command/output paths remain local, including private SDK prefixes.
        return row | {"state": "enumeration_failed", "reason": type(error).__name__}


def collect(repository, build, baseline, raw):
    cache_path = build / "CMakeCache.txt"
    cache = cache_path.read_text()
    source = Path(next(s.split("=", 1)[1] for s in cache.splitlines() if s.startswith("CMAKE_HOME_DIRECTORY:"))).resolve()
    git_env = dict(os.environ, GIT_OPTIONAL_LOCKS="0")
    for key in subprocess.check_output(["git", "rev-parse", "--local-env-vars"], env=git_env, text=True).splitlines():
        git_env.pop(key, None)
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], env=git_env, text=True).strip()
    if revision != baseline:
        raise ValueError("source_revision_differs")
    subprocess.run(["git", "-C", str(source), "diff", "--quiet", baseline, "--"], env=git_env, check=True)
    result = subprocess.run(["ctest", "--test-dir", str(build), "--show-only=json-v1"], capture_output=True)
    (raw / (repository + "-ctest.json")).write_bytes(result.stdout)
    (raw / (repository + "-ctest.stderr")).write_bytes(result.stderr)
    result.check_returncode()
    data = json.loads(result.stdout)
    rows, jobs, seen, binaries = [], [], set(), {}
    for test in data["tests"]:
        row = {"repository": repository, "ctest": test["name"]}
        command = test.get("command")
        if not command:
            rows.append(row | {"state": "missing_command"})
            continue
        binary = Path(command[0])
        if not binary.is_file():
            rows.append(row | {"state": "missing_executable"})
            continue
        if binary not in binaries:
            content = binary.read_bytes()
            binaries[binary] = (digest(content), content.startswith(b"\x7fELF") and b"gtest_list_tests" in content)
        binary_hash, recognized = binaries[binary]
        row.update(binary=binary.name, binary_sha256=binary_hash)
        if not recognized:
            rows.append(row | {"state": "wrapper_or_unrecognized_binary"})
            continue
        properties = {p["name"]: p["value"] for p in test.get("properties", [])}
        arguments = [a for a in command[1:] if not a.startswith("--gtest_output=")]
        command = [str(binary), *arguments, "--gtest_list_tests", "--gtest_color=no"]
        key = configuration_key(command, properties)
        row.update(configuration=key, ctest_disabled=bool(properties.get("DISABLED", False)))
        if key in seen:
            rows.append(row | {"state": "duplicate_configuration"})
            continue
        seen.add(key)
        jobs.append((row, command, properties, properties.get("WORKING_DIRECTORY", str(build))))
    reference = {
        "repository": repository,
        "source_revision": revision,
        "cmake_cache_sha256": digest(cache_path.read_bytes()),
        "ctest_sha256": digest(result.stdout),
        "registered_entries": len(data["tests"]),
    }
    return reference, rows, jobs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="append", required=True, metavar="REPOSITORY=PATH")
    parser.add_argument("--library-path", action="append", default=[], type=Path)
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--jobs", type=int, choices=range(1, 17), default=4)
    args = parser.parse_args()
    baseline = {r["name"]: r["commit"] for r in json.loads((ROOT / "planning/baseline.json").read_text())["repositories"]}
    args.raw_dir.mkdir(parents=True, exist_ok=True)
    raw = Path(tempfile.mkdtemp(prefix="native-lists-", dir=args.raw_dir))
    repositories, rows, jobs, names = [], [], [], set()
    for entry in args.build:
        name, path = entry.split("=", 1)
        if name not in baseline or name in names:
            parser.error("build repositories must be unique pinned baseline names")
        names.add(name)
        try:
            repository, omitted, pending = collect(name, Path(path).resolve(), baseline[name], raw)
            repositories.append(repository)
            rows.extend(omitted)
            jobs.extend(pending)
        except (OSError, ValueError, KeyError, TypeError, StopIteration, subprocess.CalledProcessError) as error:
            repositories.append({"repository": name, "state": "unqualified_build", "reason": type(error).__name__})
    env = os.environ.copy()
    if args.library_path:
        env["LD_LIBRARY_PATH"] = os.pathsep.join(map(str, args.library_path))
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        rows.extend(pool.map(lambda job: enumerate_job(job, raw, env), jobs))
    failed = sum(r.get("state") == "unqualified_build" for r in repositories) + sum(r["state"] == "enumeration_failed" for r in rows)
    report = {
        "scope": "Available configured native CTest/gtest lists only. Listed cases are not executed passes. Wrappers, unrecognized shared-gtest binaries and unavailable builds remain gaps. Checkout/cache/binary hashes are not clean-build attestation; full runtime inventory and immutable dependency image remain open.",
        "acceptance": "inventory_only",
        "enumerated_at": datetime.now(timezone.utc).isoformat(),
        "collector_sha256": digest(Path(__file__).read_bytes()),
        "baseline_sha256": digest((ROOT / "planning/baseline.json").read_bytes()),
        "tool_versions": {name: subprocess.check_output([name, "--version"], text=True).splitlines()[0] for name in ["cmake", "ctest"]},
        "repositories": repositories,
        "entries": sorted(rows, key=lambda r: (r["repository"], r["ctest"])),
        "unqualified_entries": sum(r["state"] not in {"listed", "duplicate_configuration"} for r in rows),
        "listed_configurations": sum(r["state"] == "listed" for r in rows),
        "listed_case_expansions": sum(len(r.get("cases", [])) for r in rows),
        "failed_or_unqualified_builds": failed,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    (raw / "inventory.json").write_bytes(args.output.read_bytes())
    summary = ["listed_configurations", "listed_case_expansions", "unqualified_entries", "failed_or_unqualified_builds"]
    print(json.dumps({k: report[k] for k in summary}))
    return bool(failed)


if __name__ == "__main__":
    raise SystemExit(main())
