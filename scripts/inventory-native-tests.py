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



# Pinned gz-cmake tools/check_test_ran.py; recognition does not execute the guard.
RESULT_GUARD_SHA256 = "31b778bd4aebe0cc8907802cc5cd39dd648335e9de2c1eef1219d6651cb607ed"

# These pinned CTest registrations require compiler diagnostics, not a gtest list.
COMPILE_FAILURES = {
    "gz-physics": {
        "directory": "test/static_assert",
        "helper_sha256": "8fcd6c7d9b348cd5a27e12c2402cb485a7fcef601c2b053c387ac5480343bb2c",
        "cmake_sha256": "5d7e972ccffd097511d8fc10ed531ff7675144621011ad58606d089106742895",
        "targets": {"FeatureList_InvalidFeature": "NOT A Feature",
                    "FeatureList_conflict": "FEATURE CONFLICT DETECTED",
                    "FeatureList_nested_conflict": "FEATURE CONFLICT DETECTED"},
    },
    "gz-plugin": {
        "directory": "test/static_assertions",
        "helper_sha256": "e39476402e9d020340328af57f3a18fc40e20bb8a2c3021162e32fe15d759a50",
        "cmake_sha256": "0c03081edaf0fb17aa81341b6fd2423d4091d93b4e27628c49a8dd8d2235b577",
        "targets": {"plugin_bad_const_construct": "The requested PluginPtr cast would discard const qualifiers",
                    "plugin_bad_const_assignment": "The requested PluginPtr cast would discard const qualifiers",
                    "plugin_no_base_class": "BUT THE INTERFACE IS NOT A BASE CLASS OF THE",
                    "plugin_wrong_base_class": "BUT THE INTERFACE IS NOT A BASE CLASS OF THE"},
    },
}


def compile_failure_registration(repository, test, tests, build, source):
    contract = COMPILE_FAILURES.get(repository)
    if contract is None or sum(row["name"] == test["name"] for row in tests) != 1:
        return None
    target = test["name"].removeprefix("STATIC_ASSERT_")
    if test["name"] != "STATIC_ASSERT_" + target or target not in contract["targets"]:
        return None
    directory = source / contract["directory"]
    helper, cmake, unit = (directory / name for name in ["testrunner.bash", "CMakeLists.txt", target + ".cc"])
    command = [str(helper), "/usr/bin/cmake", "--build", ".", "--target", target, "--config", "Release"]
    properties = [{"name": "PASS_REGULAR_EXPRESSION", "value": [contract["targets"][target]]},
                  {"name": "WORKING_DIRECTORY", "value": str(build)}]
    if test.get("command") != command or test.get("properties") != properties:
        return None
    paths = [helper, cmake, unit]
    if any(path.is_symlink() or not path.is_file()
           or not path.resolve().is_relative_to(source.resolve()) for path in paths):
        return None
    hashes = {path.name: digest(path.read_bytes()) for path in paths}
    if hashes[helper.name] != contract["helper_sha256"] or hashes[cmake.name] != contract["cmake_sha256"]:
        return None
    return {"state": "registered_compile_failure", "framework": "ctest_compiler_diagnostic",
            "scope": "registration_only_not_executed", "target": target,
            "expected_diagnostic": contract["targets"][target], "source_sha256": hashes,
            "configuration": configuration_key(command, properties)}


def result_guard(test, tests, known_gtests, build):
    command = test.get("command", [])
    if len(command) != 3 or command[0] != "/usr/bin/python3":
        return None
    helper = Path(command[1])
    if not helper.is_file() or digest(helper.read_bytes()) != RESULT_GUARD_SHA256:
        return None
    name = test["name"]
    if not name.startswith("check_") or name[6:] not in known_gtests:
        return None
    if sum(t["name"] == name for t in tests) != 1:
        return None
    parents = [t for t in tests if t["name"] == name[6:]]
    if len(parents) != 1:
        return None
    parent = parents[0]
    properties = test.get("properties", [])
    if any(p["name"] != "WORKING_DIRECTORY" for p in properties) or len(properties) > 1:
        return None
    cwd = properties[0]["value"] if properties else str(build)
    parent_cwds = [p["value"] for p in parent.get("properties", []) if p["name"] == "WORKING_DIRECTORY"]
    if parent_cwds != [cwd] or not isinstance(cwd, str):
        return None
    outputs = [a.removeprefix("--gtest_output=xml:") for a in parent["command"][1:] if a.startswith("--gtest_output=")]
    if outputs != [command[2]] or "--gtest_output=xml:" + command[2] not in parent["command"]:
        return None
    target = Path(command[2])
    if not target.is_absolute() or not target.resolve().is_relative_to((build / "test_results").resolve()):
        return None
    if target.resolve() != (build / "test_results" / (parent["name"] + ".xml")).resolve():
        return None
    return {"state": "registered_result_guard", "guarded_ctest": parent["name"],
            "helper_sha256": RESULT_GUARD_SHA256, "scope": "registration_only_not_executed",
            "configuration": configuration_key(command, properties)}


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
    known_gtests = {job[0]["ctest"] for job in jobs}
    registrations = {t["name"]: t for t in data["tests"]}
    for row in rows:
        if row["state"] == "wrapper_or_unrecognized_binary":
            guard = result_guard(registrations[row["ctest"]], data["tests"], known_gtests, build)
            if guard is None:
                guard = compile_failure_registration(repository, registrations[row["ctest"]], data["tests"], build, source)
            if guard is not None:
                row.update(guard)
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
        "scope": "Available configured native CTest/gtest lists only. Listed cases are not executed passes. Exact pinned result-file and compiler-diagnostic guards are registered separately, never executed or counted as gtest cases. Other wrappers, unrecognized shared-gtest binaries and unavailable builds remain gaps. Checkout/cache/binary hashes are not clean-build attestation; full runtime inventory and immutable dependency image remain open.",
        "acceptance": "inventory_only",
        "enumerated_at": datetime.now(timezone.utc).isoformat(),
        "collector_sha256": digest(Path(__file__).read_bytes()),
        "baseline_sha256": digest((ROOT / "planning/baseline.json").read_bytes()),
        "tool_versions": {name: subprocess.check_output([name, "--version"], text=True).splitlines()[0] for name in ["cmake", "ctest"]},
        "repositories": repositories,
        "entries": sorted(rows, key=lambda r: (r["repository"], r["ctest"])),
        "unqualified_entries": sum(r["state"] not in {"listed", "duplicate_configuration", "registered_result_guard", "registered_compile_failure"} for r in rows),
        "registered_result_guards": sum(r["state"] == "registered_result_guard" for r in rows),
        "registered_compile_failures": sum(r["state"] == "registered_compile_failure" for r in rows),
        "listed_configurations": sum(r["state"] == "listed" for r in rows),
        "listed_case_expansions": sum(len(r.get("cases", [])) for r in rows),
        "failed_or_unqualified_builds": failed,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    (raw / "inventory.json").write_bytes(args.output.read_bytes())
    summary = ["registered_result_guards", "registered_compile_failures", "listed_configurations", "listed_case_expansions", "unqualified_entries", "failed_or_unqualified_builds"]
    print(json.dumps({k: report[k] for k in summary}))
    return bool(failed)


if __name__ == "__main__":
    raise SystemExit(main())
