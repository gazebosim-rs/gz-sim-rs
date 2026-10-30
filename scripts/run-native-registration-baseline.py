#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Run the ten audited non-gtest registrations; listing is never execution."""
import argparse
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("native_inventory", Path(__file__).with_name("inventory-native-tests.py"))
inventory = importlib.util.module_from_spec(spec)
spec.loader.exec_module(inventory)


def validate_xml(data, contract):
    root = ET.fromstring(data)
    suites, cases = list(root.iter("testsuite")), list(root.iter("testcase"))
    if root.tag not in {"testsuites", "testsuite"} or len(suites) != 1 or len(cases) != 1:
        raise ValueError("unexpected XML test count")
    if suites[0].attrib.get("tests") != "1":
        raise ValueError("missing XML tests total")
    for summary in [root, suites[0]]:
        if "tests" in summary.attrib and summary.attrib["tests"] != "1":
            raise ValueError("unexpected XML tests total")
        if any(summary.attrib.get(key, "0") != "0" for key in ["failures", "errors", "disabled", "skipped"]):
            raise ValueError("XML failure or skip")
    if any(list(root.iter(tag)) for tag in ["failure", "error", "skipped"]):
        raise ValueError("XML failure or skip element")
    case = cases[0]
    expected = contract["case_class_suffix"]
    actual = case.attrib.get("classname", "")
    matches = actual == expected or (contract["framework"] == "pytest_xml" and actual.endswith("." + expected))
    if not matches or case.attrib.get("name") != contract["case_name"]:
        raise ValueError("unexpected XML case identity")
    if case.attrib.get("status", "run") != "run" or case.attrib.get("result", "completed") != "completed":
        raise ValueError("XML case did not run")
    return {"tests": 1, "failures": 0, "errors": 0, "skipped": 0,
            "classname": expected, "name": contract["case_name"]}


def execute_registration(repository, test, tests, build, source, raw, base_env):
    record = {"repository": repository, "ctest": test["name"],
              "scope": "native_reference_runtime_only_no_rust_credit"}
    stdout, stderr = b"", b""
    try:
        registration = inventory.audited_registration(repository, test, tests, build, source)
        if not registration or registration.get("state") != "registered_native_wrapper":
            raise ValueError("unqualified registration")
        contract = inventory.registration_contracts()[1][(repository, test["name"])]
        record.update(framework=registration["framework"], configuration=registration["configuration"],
                      input_sha256=registration["input_sha256"])
        report = None
        if "result_xml" in contract:
            report = Path(contract["result_xml"].format(build=build))
            if report.exists() or report.is_symlink():
                raise ValueError("stale XML endpoint")
        properties = {p["name"]: p["value"] for p in test.get("properties", [])}
        env = dict(base_env)
        for assignment in inventory.environment_list(properties, "ENVIRONMENT"):
            key, value = assignment.split("=", 1)
            if not key:
                raise ValueError("invalid environment assignment")
            env[key] = value
        command = list(test["command"])
        modifications = inventory.environment_list(properties, "ENVIRONMENT_MODIFICATION")
        if modifications:
            prefix = ["/usr/bin/cmake", "-E", "env"]
            for modification in modifications:
                prefix += ["--modify", modification]
            command = [*prefix, "--", *command]
        result = subprocess.run(command, cwd=properties["WORKING_DIRECTORY"], env=env,
                                capture_output=True, timeout=90)
        stdout, stderr = result.stdout, result.stderr
        record["exit_code"] = result.returncode
        if result.returncode != 0:
            raise ValueError("native command failed")
        if inventory.audited_registration(repository, test, tests, build, source) != registration:
            raise ValueError("registration inputs changed during execution")
        if report is not None:
            if report.is_symlink() or not report.is_file():
                raise ValueError("missing or linked result XML")
            xml = report.read_bytes()
            record["result"] = validate_xml(xml, contract)
            record["xml_sha256"] = inventory.digest(xml)
            (raw / (registration["configuration"] + ".xml")).write_bytes(xml)
        else:
            match = re.fullmatch(rb"No unversioned symbols found \(num versioned symbols:(\d+)\)\n", stdout)
            if not match or int(match[1]) == 0 or stderr:
                raise ValueError("invalid native ABI guard result")
            record["versioned_symbols"] = int(match[1])
        record["state"] = "executed_pass"
    except subprocess.TimeoutExpired as error:
        stdout, stderr = error.stdout or b"", error.stderr or b""
        record.update(state="execution_failed", reason="TimeoutExpired")
    except OSError as error:
        record.update(state="execution_failed", reason=type(error).__name__)
    except (ValueError, KeyError, ET.ParseError) as error:
        record.update(state="execution_failed", reason=str(error))
    stem = inventory.digest((repository + ":" + test["name"]).encode())
    (raw / (stem + ".stdout")).write_bytes(stdout)
    (raw / (stem + ".stderr")).write_bytes(stderr)
    record.update(stdout_sha256=inventory.digest(stdout), stderr_sha256=inventory.digest(stderr))
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="append", required=True, metavar="REPOSITORY=PATH")
    parser.add_argument("--inventory", required=True, type=Path)
    parser.add_argument("--environment", required=True, type=Path)
    parser.add_argument("--image-id", required=True)
    parser.add_argument("--raw-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    contract_path = ROOT / "planning/reference/non-gtest-registration-contracts.json"
    contracts = json.loads(contract_path.read_text())
    contract_sha = inventory.digest(contract_path.read_bytes())
    if args.image_id != contracts["image_id"] or inventory.digest(args.environment.read_bytes()) != contracts["environment_sha256"]:
        parser.error("runtime environment differs from selected immutable SDK contract")
    public = json.loads(args.inventory.read_text())
    if public.get("registration_contracts_sha256") != inventory.digest(contract_path.read_bytes()):
        parser.error("inventory does not bind these registration contracts")
    public_rows = {(r["repository"], r["ctest"]): r for r in public["entries"]}
    if len(public_rows) != len(public["entries"]):
        parser.error("duplicate inventory identities")
    pins = {r["name"]: r["commit"] for r in json.loads((ROOT / "planning/baseline.json").read_text())["repositories"]}
    args.raw_dir.mkdir(parents=True, exist_ok=True)
    raw = Path(tempfile.mkdtemp(prefix="native-runtime-", dir=args.raw_dir))
    results, seen, repositories, blocked = [], set(), [], []
    for value in args.build:
        name, path = value.split("=", 1)
        if name in seen or name not in pins:
            parser.error("build repositories must be unique pinned names")
        seen.add(name)
        build = Path(path).resolve()
        metadata, registered, jobs = inventory.collect(name, build, pins[name], raw)
        repositories.append(metadata)
        for job in jobs:
            if "_known_profile_gate" in job[0]:
                row = inventory.enumerate_job(job, raw, os.environ)
                if row.get("state") != "profile_blocked" or public_rows.get((name, row["ctest"])) != row:
                    parser.error("runtime profile gate differs from bound inventory")
                blocked.append(row)
        tests = json.loads((raw / (name + "-ctest.json")).read_text())["tests"]
        source = Path(next(line.split("=", 1)[1] for line in (build / "CMakeCache.txt").read_text().splitlines()
                           if line.startswith("CMAKE_HOME_DIRECTORY:")))
        for row in registered:
            if row["state"] != "registered_native_wrapper":
                continue
            prior = public_rows.get((name, row["ctest"]))
            if prior != row:
                parser.error("runtime registration differs from bound inventory")
            test = next(test for test in tests if test["name"] == row["ctest"])
            results.append(execute_registration(name, test, tests, build, source, raw, os.environ))
    expected = {key for key, row in inventory.registration_contracts()[1].items()
                if row["framework"] != "profile_blocked_zenoh"}
    actual = {(row["repository"], row["ctest"]) for row in results}
    if actual != expected:
        parser.error("runtime selection must contain exactly the ten audited registrations")
    if len(blocked) != 1 or inventory.digest(contract_path.read_bytes()) != contract_sha:
        parser.error("missing profile proof or changed registration contracts")
    report = {"scope": "ten audited native registrations only; no gtest listing-to-pass conversion, Rust port or retirement",
              "acceptance": "bounded_native_runtime_only", "observed_at": datetime.now(timezone.utc).isoformat(),
              "image_id": args.image_id, "environment_sha256": inventory.digest(args.environment.read_bytes()),
              "inventory_sha256": inventory.digest(args.inventory.read_bytes()),
              "registration_contracts_sha256": inventory.registration_contracts()[0],
              "runner_sha256": inventory.digest(Path(__file__).read_bytes()),
              "collector_sha256": inventory.digest(Path(inventory.__file__).read_bytes()),
              "repositories": repositories, "results": results,
              "executed_pass": sum(row["state"] == "executed_pass" for row in results),
              "profile_blocked": blocked,
              "remaining": "Other profiles, all listed gtest execution, result/compile guards and complete source-ledger reconciliation remain open."}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    (raw / "runtime.json").write_bytes(args.output.read_bytes())
    print(json.dumps({"executed_pass": report["executed_pass"], "profile_blocked": len(report["profile_blocked"])}))
    return report["executed_pass"] != 10


if __name__ == "__main__":
    raise SystemExit(main())
