#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Bind generated-source accounting to this consumer's actual Cargo build inputs."""

import hashlib
import json
from pathlib import Path


PACKAGE = "gz-sim-proto"
FACADE = Path("crates/gz-sim-proto/src/lib.rs")
BUILD = Path("crates/gz-sim-proto/build.rs")
AUDITOR = Path("crates/gz-sim-proto/tests/source_audit.rs")
PROOF = "syn2-proto-generated-include-v1"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generated_inputs(root, target, output):
    """Validate the successful standard generator's identities, not a path glob."""
    if (output.is_symlink() or not output.is_dir()
            or not output.resolve().is_relative_to(target.resolve())):
        raise ValueError("proto build output is outside the owned target")
    evidence_path = output / "schema-validation.json"
    if evidence_path.is_symlink():
        raise ValueError("unsafe proto build evidence")
    evidence = json.loads(evidence_path.read_text())
    package = root / "crates/gz-sim-proto"
    schemas = package / "schemas"
    if schemas.is_symlink() or not schemas.is_dir():
        raise ValueError("proto schema root is not a real directory")
    policy = json.loads((package / "schema-source.json").read_text())
    bindings = output / "gz.msgs.rs"
    descriptor = output / "schema-descriptor.pb"
    expected = {
        "schema_source_sha256": digest(package / "schema-source.json"),
        "manifest_sha256": digest(package / "schemas/manifest.json"),
        "schema_revision": policy["schema_only_candidate"],
        "schema_count": 216,
        "generator": "prost-build 0.14.4",
        "compiler_version": policy["compiler_version"],
        "external_any_sha256": policy["external_any_sha256"],
        "descriptor_sha256": policy["descriptor_sha256"],
        "generated": {"path": str(bindings), "sha256": digest(bindings)},
    }
    if any(evidence.get(key) != value for key, value in expected.items()):
        raise ValueError("proto generated-source identity differs")
    if (bindings.is_symlink() or descriptor.is_symlink()
            or digest(descriptor) != expected["descriptor_sha256"]
            or digest(Path(evidence["compiler"])) != evidence["compiler_sha256"]
            or digest(Path(evidence["external_any_path"])) != expected["external_any_sha256"]):
        raise ValueError("proto compiler, descriptor or standard import changed")
    manifest = json.loads((package / "schemas/manifest.json").read_text())
    if (policy["manifest_sha256"] != expected["manifest_sha256"]
            or len(manifest["schemas"]) != 216
            or {path.relative_to(package).as_posix() for path in (package / "schemas").rglob("*.proto")}
                != {item["path"] for item in manifest["schemas"]}
            or any(digest(package / item["path"]) != item["sha256"]
                   for item in manifest["schemas"])):
        raise ValueError("proto schema inputs changed")
    return evidence


def zero_facade(root, report, lcov, crates, audit):
    """Classify only an absent, AST-proven facade over a measured owned build."""
    facade = (root / FACADE).resolve()
    build = (root / BUILD).resolve()
    if PACKAGE not in crates or facade not in crates[PACKAGE] or build not in crates[PACKAGE]:
        raise ValueError("proto production sources are missing")
    files = [item for data in report["data"] for item in data["files"]]
    measured = [item for item in files if Path(item["filename"]).resolve() == build]
    if len(measured) != 1 or any(measured[0]["summary"][metric]["count"] == 0
                                  for metric in ("lines", "regions")):
        raise ValueError("proto owned build script is inactive or uninstrumented")
    present = [item for item in files if Path(item["filename"]).resolve() == facade]
    if present:
        return frozenset()  # Real instrumentation always takes precedence over proof.
    if facade in {Path(line[3:]).resolve() for line in lcov.splitlines() if line.startswith("SF:")}:
        raise ValueError("proto facade has LCOV instrumentation without a report")
    expected = {"proof": PROOF, "source": str(facade), "source_sha256": digest(facade)}
    if audit != expected:
        raise ValueError("missing or stale proto facade syntax proof")
    return frozenset({facade})


def generated_report(report, evidence):
    """The only generated source admitted by this leaf is the verified binding file."""
    binding = Path(evidence["generated"]["path"]).resolve()
    files = [item for data in report["data"] for item in data["files"]
             if Path(item["filename"]).resolve().is_relative_to(binding.parent)]
    if (len(files) != 1 or Path(files[0]["filename"]).resolve() != binding
            or any(files[0]["summary"][metric]["count"] == 0 for metric in ("lines", "regions"))):
        raise ValueError("verified proto generated source is absent or unexpected")
