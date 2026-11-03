#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Verify the current tree or an isolated copy of the Git index."""

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

# Keep the source-bound proto proof independent of renderer test support.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import proto_inputs
import zenoh_peer

ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    return subprocess.run(command, cwd=ROOT, check=True, **kwargs)


def owned_coverage(report, lcov, crates):
    """Select whole owned records without merging aliases or changing counters."""
    owned = {path for paths in crates.values() for path in paths}
    view = {"data": [{"files": [file for file in data["files"]
                               if Path(file["filename"]).resolve() in owned]}
                     for data in report["data"]]}
    records, block = [], []
    for line in lcov.split("\n"):
        block.append(line)
        if line == "end_of_record":
            sources = [value[3:] for value in block if value.startswith("SF:")]
            if len(sources) != 1:
                raise ValueError("LCOV record must have exactly one source")
            if Path(sources[0]).resolve() in owned:
                records.append("\n".join(block) + "\n")
            block = []
    if any(block):
        raise ValueError("unterminated LCOV record")
    return view, "".join(records)


def check_coverage(report, crates, zero_executable=frozenset()):
    """Reject missing instrumentation and enforce each crate independently."""
    facade = (ROOT / "crates/gz-renderer/src/lib.rs").resolve()
    if not set(zero_executable) <= {facade, (ROOT / proto_inputs.FACADE).resolve()}:
        raise ValueError("unrecognized zero-executable source")
    if facade in zero_executable:
        check_gpu_source(report)
    files = {}
    for data in report["data"]:
        for file in data["files"]:
            path = Path(file["filename"]).resolve()
            if path in files:
                raise ValueError(f"duplicate coverage for {path}")
            files[path] = file
    total = {metric: [0, 0] for metric in ("lines", "regions")}
    summaries = {}
    for crate, sources in crates.items():
        if not sources:
            raise ValueError(f"{crate}: no production sources")
        missing = set(sources) - files.keys() - set(zero_executable)
        if missing:
            raise ValueError(f"{crate}: uninstrumented sources: {sorted(map(str, missing))}")
        counts = {metric: [0, 0] for metric in total}
        for source in sources:
            if source in zero_executable:
                if source in files and any(files[source]["summary"][metric]["count"] for metric in counts):
                    raise ValueError(f"zero-executable source has executable regions: {source}")
                continue
            for metric in counts:
                value = files[source]["summary"][metric]
                counts[metric][0] += value["covered"]
                counts[metric][1] += value["count"]
        for metric, (covered, count) in counts.items():
            if count == 0 or covered * 100 <= count * 95:
                raise ValueError(f"{crate}: {metric} {covered}/{count} must exceed 95%")
            total[metric][0] += covered
            total[metric][1] += count
        summaries[crate] = counts
    if not summaries:
        raise ValueError("no production crates")
    summaries["workspace"] = total
    return summaries


TEST_SUPPORT_PACKAGE = "gz-render-test-support"


def validate_test_support(metadata, required=True):
    members = set(metadata["workspace_members"])
    packages = [package for package in metadata["packages"] if package["id"] in members]
    support = [package for package in packages if package["name"] == TEST_SUPPORT_PACKAGE]
    if not support and not required:
        return
    if len(support) != 1:
        raise ValueError("missing or duplicate render test-support package")
    package = support[0]
    expected = (ROOT / "tests/support/render-fixtures/Cargo.toml").resolve()
    if (Path(package["manifest_path"]).resolve() != expected or package.get("publish") != []
            or package.get("metadata", {}).get("gz-sim", {}).get("role") != "test-support"):
        raise ValueError("invalid render test-support ownership")
    targets = package["targets"]
    root = expected.parent
    if (not any("bin" in target["kind"] for target in targets)
            or not any("test" in target["kind"] for target in targets)
            or any(not set(target["kind"]) <= {"bin", "test"}
                   or not Path(target["src_path"]).resolve().is_relative_to(root)
                   for target in targets)):
        raise ValueError("invalid render test-support targets")
    for other in packages:
        if other["id"] != package["id"] and any(
                dependency["name"] == TEST_SUPPORT_PACKAGE for dependency in other["dependencies"]):
            raise ValueError("production package depends on test-support")


def production_sources():
    metadata = json.loads(run(["cargo", "metadata", "--locked", "--no-deps", "--format-version=1"], capture_output=True, text=True).stdout)
    validate_test_support(metadata, required=False)
    members = set(metadata["workspace_members"])
    production_kinds = {"lib", "rlib", "staticlib", "cdylib", "proc-macro", "bin", "custom-build"}
    sources = {}
    for package in metadata["packages"]:
        if package["id"] not in members or package["name"] == TEST_SUPPORT_PACKAGE:
            continue
        root = Path(package["manifest_path"]).parent.resolve()
        paths = set((root / "src").rglob("*.rs"))
        if (root / "build.rs").exists():
            paths.add(root / "build.rs")
        for target in package["targets"]:
            if production_kinds.intersection(target["kind"]):
                source = Path(target["src_path"]).resolve()
                if not source.is_relative_to(root):
                    raise ValueError(f"production target outside package root: {source}")
                paths.add(source)
        sources[package["name"]] = sorted(paths)
    return sources


def changed_lines(diff):
    result, path = {}, None
    old_left = new_left = 0
    have_file = expect_new = False
    for line in diff.split("\n"):
        if line == "\\ No newline at end of file":
            continue
        if old_left or new_left:
            prefix = line[:1]
            if prefix not in ("+", "-", " "):
                raise ValueError("incomplete changed-line hunk")
            old_left -= prefix != "+"
            new_left -= prefix != "-"
            if old_left < 0 or new_left < 0:
                raise ValueError("changed-line hunk exceeds its counts")
            continue
        if line.startswith("diff --git "):
            path, have_file, expect_new = None, False, False
        elif line.startswith("--- "):
            path, have_file, expect_new = None, False, True
        elif line.startswith("+++ "):
            if not expect_new:
                raise ValueError("changed-line file header without old path")
            name = line[4:].removesuffix("\t")
            if name != "/dev/null" and not name.startswith("b/"):
                raise ValueError("unsupported quoted diff path")
            path = None if name == "/dev/null" else name[2:]
            have_file, expect_new = True, False
        elif line.startswith("@@"):
            match = re.fullmatch(r"@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@(?: .*)?", line)
            if not match or not have_file:
                raise ValueError("invalid changed-line hunk")
            old_left, new_left = int(match[2] or 1), int(match[4] or 1)
            start = int(match[3])
            if path is None and new_left:
                raise ValueError("changed lines without a file")
            if path is not None and new_left:
                result.setdefault(path, set()).update(range(start, start + new_left))
        elif line.startswith(("+", "-", " ")):
            raise ValueError("changed-line content outside a hunk")
    if old_left or new_left or expect_new:
        raise ValueError("incomplete changed-line diff")
    return result


def source_line_numbers(path):
    data = path.read_bytes()
    count = data.count(b"\n") + bool(data and not data.endswith(b"\n"))
    return set(range(1, count + 1))


def changed_sources(sources, index=False):
    files = {str(path.relative_to(ROOT)): path for paths in sources.values() for path in paths}
    base = os.environ.get("GZ_COVERAGE_BASE") if not index else None
    head = subprocess.run(["git", "rev-parse", "--verify", "HEAD"], cwd=ROOT, capture_output=True, text=True)
    command = ["git", "-c", "core.quotepath=false", "diff", "--no-ext-diff", "--no-renames", "--unified=0"]
    if index:
        command += ["--cached", *(["HEAD"] if head.returncode == 0 else [])]
    elif base:
        if not re.fullmatch(r"[0-9a-f]{40}", base):
            raise ValueError("invalid coverage base SHA")
        if base == "0" * 40:
            return {name: source_line_numbers(path) for name, path in files.items()}
        run(["git", "cat-file", "-e", f"{base}^{{commit}}"], capture_output=True)
        command += [base, "HEAD"]
    else:
        if head.returncode != 0:
            return {name: source_line_numbers(path) for name, path in files.items()}
        command += [head.stdout.strip()]
    diff = run([*command, "--", "*.rs"], capture_output=True).stdout.decode("utf-8")
    changed = changed_lines(diff)
    if not index and not base:
        untracked = run(["git", "ls-files", "--others", "--exclude-standard", "-z"], capture_output=True).stdout.decode("utf-8").split("\0")
        for name in set(untracked) & files.keys():
            changed[name] = source_line_numbers(files[name])
        if not changed:
            # A clean checkout validates its latest commit; CI supplies the full push/PR base.
            parent = subprocess.run(["git", "rev-parse", "--verify", "HEAD^"], cwd=ROOT, capture_output=True, text=True)
            if parent.returncode == 0:
                changed = changed_lines(run([*command[:-1], "HEAD^", "HEAD", "--", "*.rs"], capture_output=True).stdout.decode("utf-8"))
            else:
                changed = {name: source_line_numbers(path) for name, path in files.items()}
    return changed


def check_changed_coverage(lcov, changed, crates, executable_counts=None, zero_executable=frozenset()):
    lines, source = {}, None
    for line in lcov.split("\n"):
        if line.startswith("SF:"):
            source = Path(line[3:]).resolve()
            if source in lines:
                raise ValueError(f"duplicate LCOV source: {source}")
            lines[source] = {}
        elif line.startswith("DA:"):
            if source is None:
                raise ValueError("LCOV count without source")
            number, count, *_ = line[3:].split(",")
            number, count = int(number), int(count)
            if number <= 0 or count < 0 or number in lines[source]:
                raise ValueError(f"invalid or duplicate LCOV line: {source}:{number}")
            lines[source][number] = count
    totals = {}
    for crate, sources in crates.items():
        covered, count = 0, 0
        for path in sources:
            if path in zero_executable:
                if lines.get(path):
                    raise ValueError(f"zero-executable source has LCOV lines: {path}")
                continue
            if path not in lines:
                raise ValueError(f"missing LCOV source: {path}")
            if not lines[path] and (executable_counts is None or executable_counts[path] > 0):
                raise ValueError(f"missing executable LCOV lines: {path}")
            changed_in_file = set(changed.get(str(path.relative_to(ROOT)), []))
            for number in changed_in_file & lines[path].keys():
                count += 1
                covered += lines[path][number] > 0
        if count and covered * 100 <= count * 95:
            raise ValueError(f"{crate}: changed executable lines {covered}/{count} must exceed 95%")
        totals[crate] = [covered, count]
    return totals


# Renderer-only ownership stays separate from shared CPU production crates.
RENDER_PACKAGES = frozenset({
    "gz-camera-output",
    "gz-render-frame", "gz-renderer", "gz-render-protocol",
    "gz-render-client", "gz-render-worker", "gz-render-scene",
})
RENDER_SHARED_PACKAGES = frozenset({"gz-sim-core", "gz-sim-scene", "gz-sim-cli", proto_inputs.PACKAGE})
RGB_FEATURE = "gz-sim-cli/rgb-camera"
RENDER_FEATURES = ["gz-renderer/gpu", RGB_FEATURE]


def load_profile_contract():
    path = ROOT / "planning/coverage-profiles.json"
    if path.is_symlink() or not path.is_file() or not path.resolve().is_relative_to(ROOT.resolve()):
        raise ValueError("missing or unsafe coverage profile contract")
    contract = json.loads(path.read_text())
    expected = {
        "version": 2,
        "native_reference": "gazebo-zenoh-client-v1",
        "profiles": {
            "headless": {"exclude_packages": sorted(RENDER_PACKAGES | {TEST_SUPPORT_PACKAGE}),
                         "runner": "cpu", "features": [RGB_FEATURE]},
            "renderer": {"packages": sorted(RENDER_PACKAGES | RENDER_SHARED_PACKAGES),
                         "features": RENDER_FEATURES, "runner": "vulkan"},
        },
        "excluded_sources": [],
        "test_support_packages": [TEST_SUPPORT_PACKAGE],
    }
    if contract != expected:
        raise ValueError("invalid coverage profile contract")
    return contract


def profile_plan(sources):
    """Measure every source and exercise the shared CLI camera path in both runs."""
    missing = (RENDER_PACKAGES | RENDER_SHARED_PACKAGES) - sources.keys()
    if missing:
        raise ValueError(f"required renderer profile packages missing: {sorted(missing)}")
    unexpected = {name for name in sources if name.startswith("gz-render") and name not in RENDER_PACKAGES}
    if unexpected:
        raise ValueError(f"undeclared renderer packages: {sorted(unexpected)}")
    headless = {name: files for name, files in sources.items() if name not in RENDER_PACKAGES}
    render = {name: sources[name] for name in sorted(RENDER_PACKAGES | RENDER_SHARED_PACKAGES)}
    if not headless or any(not files for files in sources.values()):
        raise ValueError("empty production crate or headless profile")
    owners = {}
    for name, files in sources.items():
        for path in files:
            path = Path(path).resolve()
            if path in owners:
                raise ValueError(f"production source has multiple owners: {path}")
            owners[path] = name
    headless_command = ["cargo", "test", "--locked", "--workspace", "--all-targets", "--features", RGB_FEATURE]
    for name in sorted(RENDER_PACKAGES | {TEST_SUPPORT_PACKAGE}):
        headless_command += ["--exclude", name]
    render_command = ["cargo", "test", "--locked", "--all-targets"]
    for name in render:
        render_command += ["--package", name]
    render_command += ["--package", TEST_SUPPORT_PACKAGE, "--features", ",".join(RENDER_FEATURES)]
    return {"headless": (headless, headless_command), "renderer": (render, render_command)}


def check_profile_reports(plan, reports, zero_sources=None):
    """Each mandatory profile is independently complete and strictly covered."""
    if set(plan) != {"headless", "renderer"} or set(reports) != set(plan):
        raise ValueError("missing or unexpected required coverage profiles")
    zero_sources = zero_sources or {}
    if (set(zero_sources) - {"headless", "renderer"}
            or set(zero_sources.get("headless", ())) - {(ROOT / proto_inputs.FACADE).resolve()}):
        raise ValueError("zero-executable classification in an unexpected profile")
    return {name: check_coverage(reports[name], crates, zero_sources.get(name, frozenset()))
            for name, (crates, _) in plan.items()}


def check_gpu_source(report):
    gpu_source = (ROOT / "crates/gz-renderer/src/gpu.rs").resolve()
    gpu_files = [file for data in report["data"] for file in data["files"]
                 if Path(file["filename"]).resolve() == gpu_source]
    if (len(gpu_files) != 1 or any(gpu_files[0]["summary"][metric]["count"] == 0
                                   for metric in ("lines", "regions"))):
        raise ValueError("renderer GPU implementation is inactive or uninstrumented")


def check_headless_dependencies():
    metadata = json.loads(run(["cargo", "metadata", "--locked", "--no-deps", "--format-version=1"],
                              capture_output=True, text=True).stdout)
    defaults = set(metadata["workspace_default_members"])
    default_names = {package["name"] for package in metadata["packages"] if package["id"] in defaults}
    forbidden_defaults = default_names & (RENDER_PACKAGES | {TEST_SUPPORT_PACKAGE})
    if forbidden_defaults:
        raise ValueError(f"renderer packages in default workspace build: {sorted(forbidden_defaults)}")
    forbidden = {"bevy_render", "bevy_window", "bevy_winit", "bevy_core_pipeline", "bevy_pbr",
                 "bevy_shader", "ash", "naga", "vulkano", "winit", "raw-window-handle"}
    common = ["cargo", "tree", "--locked", "--edges", "normal,build", "--prefix", "none", "--format", "{p}"]
    selected = ["--workspace"]
    for name in sorted(RENDER_PACKAGES | {TEST_SUPPORT_PACKAGE}):
        selected += ["--exclude", name]
    for label, selection in (("CLI", ["--package", "gz-sim-cli"]), ("workspace", selected)):
        result = run([*common, *selection], capture_output=True, text=True)
        packages = {line.split()[0] for line in result.stdout.splitlines() if line.strip()}
        found = {name for name in packages if name in forbidden or name.startswith(("gz-render", "wgpu"))}
        if found:
            raise ValueError(f"headless {label} has rendering dependencies: {sorted(found)}")


def require_toolchain(version, environment):
    if "release: 1.97.1" not in version.splitlines():
        raise ValueError("coverage requires Rust 1.97.1")
    if any(environment.get(name) for name in ("RUSTC", "CARGO_BUILD_RUSTC", "RUSTC_WRAPPER")):
        raise ValueError("unverified compiler override")


def native_peer_inputs():
    return [ROOT / path for path in zenoh_peer.CLIENTS.values()]


def check_native_peer_evidence(directory, expected, binding=None):
    """Require the actual C++ source, executable and selected-reference binding."""
    if binding is None:
        reference, _ = selected_native_reference()
        binding = {"reference_sha256": hashlib.sha256((reference / "environment.json").read_bytes()).hexdigest(),
                   "helper_sha256": hashlib.sha256((ROOT / "scripts/zenoh_peer.py").read_bytes()).hexdigest(),
                   "fixtures": {name: hashlib.sha256((ROOT / path).read_bytes()).hexdigest()
                                for name, path in zenoh_peer.CLIENTS.items()}}
    found, records = set(), []
    for path in sorted(directory.glob("*/provenance.json")):
        if path.is_symlink() or not path.resolve().is_relative_to(directory.resolve()):
            raise ValueError("unsafe native peer provenance")
        data = json.loads(path.read_text())
        client = data.get("client")
        if (data.get("profile") != zenoh_peer.PROFILE or client not in zenoh_peer.CLIENTS
                or data.get("reference_sha256") != binding["reference_sha256"]
                or data.get("source") != zenoh_peer.CLIENTS[client]
                or data.get("helper_sha256") != binding["helper_sha256"]):
            raise ValueError("native peer reference or fixture binding differs")
        source = path.parent / "source.cc"
        binary = path.parent / (client + "-client")
        for artifact, expected_hash in [(source, data.get("source_sha256")),
                                        (binary, data.get("binary_sha256"))]:
            if (not artifact.is_file() or artifact.is_symlink()
                    or not artifact.resolve().is_relative_to(directory.resolve())
                    or hashlib.sha256(artifact.read_bytes()).hexdigest() != expected_hash):
                raise ValueError("native peer artifact is missing or changed")
        if hashlib.sha256(source.read_bytes()).hexdigest() != binding["fixtures"][client]:
            raise ValueError("native peer captured source differs")
        found.add(client)
        records.append({"client": client, "provenance": str(path.relative_to(directory)),
                        "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    if not expected <= found:
        raise ValueError(f"missing native conformance clients: {sorted(expected - found)}")
    return records


def renderer_executables(stdout, target_root):
    """Bind cross-package subprocesses to Cargo's actual feature-enabled binaries."""
    expected = {
        "gz-sim-rs": ("gz-sim-cli", "crates/gz-sim-cli/src/main.rs"),
        "gz-render-worker": ("gz-render-worker", "crates/gz-render-worker/src/main.rs"),
    }
    bound = {}
    target_root = Path(target_root).resolve()
    for line in stdout.splitlines():
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if not isinstance(item, dict) or item.get("reason") != "compiler-artifact":
            continue
        target = item.get("target", {})
        name = target.get("name")
        if name not in expected or item.get("profile", {}).get("test") is not False:
            continue
        package, source = expected[name]
        manifest = (ROOT / "crates" / package / "Cargo.toml").resolve()
        path = Path(item.get("executable") or "")
        if (name in bound or target.get("kind") != ["bin"]
                or target.get("crate_types") != ["bin"]
                or target.get("src_path") != str((ROOT / source).resolve())
                or item.get("manifest_path") != str(manifest)
                or not item.get("package_id", "").startswith("path+" + manifest.parent.as_uri() + "#")
                or not path.is_absolute() or path.is_symlink() or not path.is_file()
                or str(path) != str(path.resolve()) or not path.resolve().is_relative_to(target_root)
                or str(path) not in item.get("filenames", [])
                or not os.access(path, os.X_OK)
                or (name == "gz-sim-rs" and "rgb-camera" not in item.get("features", []))):
            raise ValueError(f"invalid renderer executable binding: {name}")
        with path.open("rb") as binary:
            if binary.read(4) != b"\x7fELF":
                raise ValueError(f"renderer executable is not ELF: {name}")
            binary.seek(0)
            digest = hashlib.file_digest(binary, "sha256").hexdigest()
        bound[name] = {"sha256": digest, "cargo_artifact": item}
    if set(bound) != set(expected):
        raise ValueError("missing renderer CLI/worker executable binding")
    return bound


def same_renderer_executables(prepared, actual):
    """Cargo may mark a cached artifact fresh; its identity and bytes cannot drift."""
    identity = ("package_id", "manifest_path", "target", "profile", "features", "executable", "filenames")
    if (set(prepared) != set(actual) or any(
            prepared[name]["sha256"] != actual[name]["sha256"]
            or any(prepared[name]["cargo_artifact"].get(key) != actual[name]["cargo_artifact"].get(key)
                   for key in identity) for name in prepared)):
        raise ValueError("renderer executable changed between preparation and execution")


def profile_coverage(name, crates, command, directory, changed, wrapper, llvm, version, target=None):
    directory.mkdir(parents=True)
    raw_dir = directory / "raw"
    raw_dir.mkdir()
    source_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for paths in crates.values() for path in paths}
    auditor_source = ROOT / "tests/support/render-fixtures/src/bin/source_audit.rs"
    auditor_source_hash = (hashlib.sha256(auditor_source.read_bytes()).hexdigest()
                           if name == "renderer" and auditor_source.is_file() else None)
    proto_auditor_hash = (hashlib.sha256((ROOT / proto_inputs.AUDITOR).read_bytes()).hexdigest()
                          if proto_inputs.PACKAGE in crates else None)
    env = os.environ.copy()
    env["CARGO_TARGET_DIR"] = str(target or ROOT / "target/coverage")
    env["RUSTFLAGS"] = ""
    env.pop("CARGO_ENCODED_RUSTFLAGS", None)
    env["RUSTC_WORKSPACE_WRAPPER"] = str(wrapper)
    env["LLVM_PROFILE_FILE"] = str(raw_dir / "%p-%m.profraw")
    env["GZ_ZENOH_PEER_EVIDENCE_DIR"] = str(directory / "native-peers")
    for key in ("GZ_RENDER_CLI_PATH", "GZ_RENDER_WORKER_PATH", "GZ_RENDER_EXECUTABLES_JSON"):
        env.pop(key, None)
    prepared_executables = None
    if name == "renderer":
        # Worker integration tests pass this exact path as the child's LLVM_PROFILE_FILE.
        env["GZ_RENDER_WORKER_PROFILE"] = str(raw_dir / "worker-%p-%m.profraw")
        if "gz-sim-cli" in crates:
            # Build-only preparation supplies actual cross-package paths before
            # tests run. It executes no test, CLI, renderer or network probe.
            preparation = subprocess.run([*command, "--no-run", "--message-format=json"],
                                         cwd=ROOT, env=env, capture_output=True, text=True)
            (directory / "renderer-build-output.log").write_text(preparation.stdout + preparation.stderr)
            if preparation.returncode:
                raise subprocess.CalledProcessError(preparation.returncode, preparation.args,
                                                    preparation.stdout, preparation.stderr)
            prepared_executables = renderer_executables(preparation.stdout, env["CARGO_TARGET_DIR"])
            env["GZ_RENDER_CLI_PATH"] = prepared_executables["gz-sim-rs"]["cargo_artifact"]["executable"]
            env["GZ_RENDER_WORKER_PATH"] = prepared_executables["gz-render-worker"]["cargo_artifact"]["executable"]
            env["GZ_RENDER_EXECUTABLES_JSON"] = json.dumps(prepared_executables)
    test_command = [*command, "--message-format=json"]
    if name == "renderer":
        # Preserve adapter identity in evidence, including for successful tests.
        test_command += ["--", "--nocapture"]
    result = subprocess.run(test_command, cwd=ROOT, env=env, capture_output=True, text=True)
    (directory / "test-output.log").write_text(result.stdout + result.stderr)
    if result.returncode:
        raise subprocess.CalledProcessError(result.returncode, result.args, result.stdout, result.stderr)
    if prepared_executables is not None:
        same_renderer_executables(prepared_executables,
                                  renderer_executables(result.stdout, env["CARGO_TARGET_DIR"]))
    contract_path = ROOT / "planning/coverage-profiles.json"
    if contract_path.is_file() and json.loads(contract_path.read_text()).get("version") == 2:
        expected = {"camera"} if name == "renderer" else {"clock", "rate", "force"}
        peers = check_native_peer_evidence(directory / "native-peers", expected)
        (directory / "native-peers.json").write_text(json.dumps(peers, indent=2) + "\n")
    objects = []
    artifacts = []
    worker_objects = []
    audit_objects = []
    proto_auditors, proto_ids, build_events = [], set(), []
    for line in result.stdout.splitlines():
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
        if item.get("reason") == "build-script-executed":
            build_events.append(item)
        if item.get("reason") == "compiler-artifact":
            target = item.get("target", {})
            source = Path(target.get("src_path", "")).resolve()
            if source == (ROOT / proto_inputs.BUILD).resolve() and "custom-build" in target.get("kind", []):
                proto_ids.add(item["package_id"])
            if (source == (ROOT / proto_inputs.AUDITOR).resolve()
                    and "test" in target.get("kind", []) and item.get("executable")):
                proto_auditors.append(Path(item["executable"]))
        if item.get("reason") == "compiler-artifact" and item.get("executable"):
            objects.append(item["executable"])
            artifacts.append(item)
            target = item.get("target", {})
            source = Path(target.get("src_path", "")).resolve()
            if (target.get("name") == "gz-render-worker" and "bin" in target.get("kind", [])
                    and source == (ROOT / "crates/gz-render-worker/src/main.rs").resolve()
                    and not item.get("profile", {}).get("test", False)):
                worker_objects.append(item["executable"])
            if (target.get("name") == "source_audit" and "bin" in target.get("kind", [])
                    and source == (ROOT / "tests/support/render-fixtures/src/bin/source_audit.rs").resolve()
                    and not item.get("profile", {}).get("test", False)):
                audit_objects.append(item["executable"])
    if not objects:
        raise ValueError(f"{name}: no instrumented test binaries")
    if name == "renderer":
        target_root = Path(env["CARGO_TARGET_DIR"]).resolve()
        if (len(worker_objects) != 1 or not Path(worker_objects[0]).is_file()
                or not Path(worker_objects[0]).resolve().is_relative_to(target_root)):
            raise ValueError("renderer worker binary absent from feature test graph")
    if run(["rustc", "-vV"], capture_output=True, text=True).stdout != version:
        raise ValueError(f"{name}: compiler changed during profile")
    current_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                      for paths in crates.values() for path in paths}
    if current_hashes != source_hashes:
        raise ValueError(f"{name}: production sources changed during profile")
    if auditor_source_hash is not None and hashlib.sha256(auditor_source.read_bytes()).hexdigest() != auditor_source_hash:
        raise ValueError("renderer source auditor changed during profile")
    facade = (ROOT / "crates/gz-renderer/src/lib.rs").resolve()
    facade_audit = None
    if name == "renderer" and len(audit_objects) == 1:
        auditor = Path(audit_objects[0])
        if auditor.is_file() and auditor.resolve().is_relative_to(Path(env["CARGO_TARGET_DIR"]).resolve()):
            audit = subprocess.run([str(auditor), str(facade)], cwd=ROOT, env=env,
                                   capture_output=True, text=True)
            if audit.returncode == 0 and audit.stdout == "renderer-facade-v1\n":
                facade_audit = auditor
    proto_audit, proto_proof = None, None
    if proto_inputs.PACKAGE in crates:
        if hashlib.sha256((ROOT / proto_inputs.AUDITOR).read_bytes()).hexdigest() != proto_auditor_hash:
            raise ValueError("proto source auditor changed during profile")
        generated = [Path(item["out_dir"]) for item in build_events if item.get("package_id") in proto_ids]
        if len(proto_ids) != 1 or len(generated) != 1 or len(proto_auditors) != 1:
            raise ValueError("missing or duplicate proto build/auditor artifacts")
        target_root = Path(env["CARGO_TARGET_DIR"]).resolve()
        proto_evidence = proto_inputs.generated_inputs(ROOT, target_root, generated[0])
        (directory / "proto-generated.json").write_text(json.dumps(proto_evidence, indent=2) + "\n")
        (directory / "proto-generated.rs").write_bytes((generated[0] / "gz.msgs.rs").read_bytes())
        (directory / "proto-descriptor.pb").write_bytes((generated[0] / "schema-descriptor.pb").read_bytes())
        proto_audit = proto_auditors[0]
        if (not proto_audit.is_file() or proto_audit.is_symlink()
                or not proto_audit.resolve().is_relative_to(target_root)):
            raise ValueError("proto auditor is outside the owned target")
        proof_path = directory / "proto-facade-audit.json"
        audit_env = dict(env, GZ_PROTO_FACADE=str((ROOT / proto_inputs.FACADE).resolve()),
                         GZ_PROTO_FACADE_PROOF=str(proof_path))
        audit = subprocess.run([str(proto_audit), "--exact", "source_audit_child", "--ignored", "--nocapture"],
                               cwd=ROOT, env=audit_env, capture_output=True, text=True)
        (directory / "proto-facade-audit-output.log").write_text(audit.stdout + audit.stderr)
        if audit.returncode or not proof_path.is_file() or proof_path.is_symlink():
            raise ValueError("proto facade syntax proof failed")
        proto_proof = json.loads(proof_path.read_text())
    raw = sorted(raw_dir.glob("*.profraw"))
    if not raw:
        raise ValueError(f"{name}: no raw coverage profiles")
    if name == "renderer" and not list(raw_dir.glob("worker-*.profraw")):
        raise ValueError("renderer worker emitted no raw profile")
    merged = directory / "merged.profdata"
    run([str(llvm / "llvm-profdata"), "merge", "-sparse", *map(str, raw), "-o", str(merged)])
    export = [str(llvm / "llvm-cov"), "export", "--fatal-warnings", "-instr-profile=" + str(merged)]
    for binary in dict.fromkeys(objects):
        export += ["-object", binary]
    with (directory / "proto-coverage.json").open("w") as output:
        run(export, stdout=output)
    unfiltered = json.loads((directory / "proto-coverage.json").read_text())
    if proto_proof is not None:
        proto_inputs.generated_report(unfiltered, proto_evidence)
    lcov = run([*export, "-format=lcov"], capture_output=True).stdout.decode("utf-8")
    (directory / "coverage-unfiltered.lcov").write_text(lcov)
    report, lcov = owned_coverage(unfiltered, lcov, crates)
    (directory / "coverage.json").write_text(json.dumps(report) + "\n")
    (directory / "coverage.lcov").write_text(lcov)
    zero_executable = frozenset()
    if name == "renderer":
        check_gpu_source(report)
        present = {Path(file["filename"]).resolve() for data in report["data"] for file in data["files"]}
        if facade not in present:
            if facade_audit is None:
                (directory / "facade-audit.json").write_text(json.dumps({
                    "source": str(facade.relative_to(ROOT)), "classification": "unproven"
                }, indent=2) + "\n")
                raise ValueError("missing renderer facade without a syntax proof")
            zero_executable = frozenset({facade})
        audit_evidence = {"source": str(facade.relative_to(ROOT)),
                          "source_sha256": source_hashes[str(facade.relative_to(ROOT))],
                          "classification": "zero-executable" if zero_executable else "instrumented"}
        if zero_executable:
            with facade_audit.open("rb") as auditor:
                audit_evidence["auditor_sha256"] = hashlib.file_digest(auditor, "sha256").hexdigest()
            audit_evidence["proof"] = "syn2-renderer-facade-v1"
            audit_evidence["auditor_source_sha256"] = auditor_source_hash
        (directory / "facade-audit.json").write_text(json.dumps(audit_evidence, indent=2) + "\n")
    if proto_inputs.PACKAGE in crates:
        zero_executable |= proto_inputs.zero_facade(ROOT, report, lcov, crates, proto_proof)
    summary = check_coverage(report, crates, zero_executable)
    counts = {Path(file["filename"]).resolve(): file["summary"]["lines"]["count"]
              for data in report["data"] for file in data["files"]}
    changed_totals = check_changed_coverage(lcov, changed, crates, counts, zero_executable)
    object_evidence = []
    for artifact in artifacts:
        path = Path(artifact["executable"])
        with path.open("rb") as executable:
            digest = hashlib.file_digest(executable, "sha256").hexdigest()
        object_evidence.append({"path": str(path), "sha256": digest,
                                "package_id": artifact.get("package_id"),
                                "target": artifact.get("target"), "profile": artifact.get("profile"),
                                "features": artifact.get("features", [])})
    evidence = {"profile": name, "rustc": version, "coverage": summary,
                "changed_lines": changed_totals, "sources": source_hashes,
                "zero_executable_sources": {str(path.relative_to(ROOT)): {
                    "proof": (proto_inputs.PROOF if path == (ROOT / proto_inputs.FACADE).resolve() else "syn2-renderer-facade-v1"),
                    "source_sha256": source_hashes[str(path.relative_to(ROOT))],
                    "auditor": str(proto_audit if path == (ROOT / proto_inputs.FACADE).resolve() else facade_audit)} for path in zero_executable},
                "objects": object_evidence}
    if prepared_executables is not None:
        evidence["prepared_executables"] = prepared_executables
    (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    (directory / "evidence.json").write_text(json.dumps(evidence, indent=2) + "\n")
    return report, zero_executable


def selected_native_reference():
    reference = Path(os.environ.get("GZ_ZENOH_REFERENCE_ROOT",
                                   ROOT / "artifacts/zenoh-native-reference")).absolute()
    return reference, zenoh_peer.bootstrap.validate_reference(reference)


def coverage(changed_file=None):
    native_root, native_metadata = selected_native_reference()
    native_bytes = (native_root / "environment.json").read_bytes()
    out = ROOT / "artifacts/coverage"
    out.mkdir(parents=True, exist_ok=True)
    load_profile_contract()
    metadata = json.loads(run(["cargo", "metadata", "--locked", "--no-deps", "--format-version=1"],
                              capture_output=True, text=True).stdout)
    validate_test_support(metadata)
    sources = production_sources()
    plan = profile_plan(sources)
    check_headless_dependencies()
    changed = json.loads(Path(changed_file).read_text()) if changed_file else changed_sources(sources)
    wrapper = ROOT / "scripts/rustc-coverage.py"
    digest = hashlib.sha256(wrapper.read_bytes()).hexdigest()
    executable = out / f"rustc-coverage-{digest}"
    executable.write_bytes(wrapper.read_bytes())
    executable.chmod(0o755)
    sysroot = Path(run(["rustc", "--print", "sysroot"], capture_output=True, text=True).stdout.strip())
    version = run(["rustc", "-vV"], capture_output=True, text=True).stdout
    require_toolchain(version, os.environ)
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
              for paths in sources.values() for path in paths}
    metadata = json.loads(run(["cargo", "metadata", "--locked", "--no-deps", "--format-version=1"],
                              capture_output=True, text=True).stdout)
    members = set(metadata["workspace_members"])
    manifests = [Path(package["manifest_path"]) for package in metadata["packages"]
                 if package["id"] in members]
    inputs = list(dict.fromkeys([ROOT / "Cargo.toml", ROOT / "Cargo.lock", ROOT / "rust-toolchain.toml",
                                 ROOT / ".cargo/config.toml",
                                 ROOT / "scripts/zenoh_peer.py", ROOT / "scripts/native-peer.py",
                                 ROOT / "tests/support/zenoh_peer.rs",
                                 *[ROOT / name for name in zenoh_peer.bootstrap.INPUTS],
                                 *native_peer_inputs(),
                                 wrapper, ROOT / "scripts/verify.py",
                                 ROOT / "planning/coverage-profiles.json", ROOT / "scripts/proto_inputs.py",
                                 ROOT / proto_inputs.AUDITOR,
                                 ROOT / "crates/gz-sim-proto/schema-source.json",
                                 ROOT / "crates/gz-sim-proto/schemas/manifest.json",
                                 ROOT / "third_party/prost-derive-0.14.4/Cargo.toml",
                                 ROOT / "third_party/prost-derive-0.14.4/LICENSE",
                                 *sorted((ROOT / "third_party/prost-derive-0.14.4/src").rglob("*.rs")),
                                 *sorted((ROOT / "crates/gz-sim-proto/schemas").rglob("*.proto")),
                                 ROOT / "tests/support/render-fixtures/src/bin/source_audit.rs", *manifests]))
    input_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in inputs}
    host = next(line.split(": ", 1)[1] for line in version.splitlines() if line.startswith("host: "))
    llvm = sysroot / "lib/rustlib" / host / "bin"
    run_dir = Path(tempfile.mkdtemp(prefix="profiles-", dir=out))
    index = {"run": run_dir.name, "profiles": sorted(plan), "rustc": version,
             "sources": hashes, "inputs": input_hashes,
             "zenoh_reference": {"root": str(native_root),
                                  "environment_sha256": hashlib.sha256(native_bytes).hexdigest()}}
    (out / "profile-index.json").write_text(json.dumps(index, indent=2) + "\n")
    (run_dir / "zenoh-native-reference.json").write_bytes(native_bytes)
    reports, zero_sources = {}, {}
    for name, (crates, command) in plan.items():
        reports[name], proof = profile_coverage(name, crates, command, run_dir / name,
                                                changed, executable, llvm, version)
        if proof:
            zero_sources[name] = proof
    summaries = check_profile_reports(plan, reports, zero_sources)
    if (selected_native_reference() != (native_root, native_metadata)
            or (native_root / "environment.json").read_bytes() != native_bytes):
        raise ValueError("selected Zenoh reference changed between profiles")
    final_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for paths in sources.values() for path in paths}
    final_inputs = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in inputs}
    if hashes != final_hashes or input_hashes != final_inputs:
        raise ValueError("workspace sources or verification inputs changed between profiles")
    # Keep legacy headless evidence paths intact for existing staged consumers.
    for filename in ("test-output.log", "evidence.json", "summary.json"):
        (out / filename).write_bytes((run_dir / "headless" / filename).read_bytes())
    print(json.dumps(summaries, indent=2))


def validate_snapshot(tree, candidate):
    """Require raw staged bytes before executing the exported verifier."""
    algorithm = run(["git", "rev-parse", "--show-object-format"], capture_output=True, text=True).stdout.strip()
    expected = set()
    for entry in run(["git", "ls-tree", "-rzt", tree], capture_output=True).stdout.split(b"\0"):
        if not entry:
            continue
        metadata, name = entry.split(b"\t", 1)
        mode, kind, oid = metadata.split()
        relative = os.fsdecode(name)
        expected.add(relative)
        path = candidate / relative
        if kind == b"tree":
            if not path.is_dir() or path.is_symlink():
                raise ValueError("exported directory differs from staged tree")
            continue
        if kind != b"blob":
            raise ValueError("unsupported non-blob staged input")
        if mode == b"120000" and path.is_symlink():
            data = os.fsencode(os.readlink(path))
        elif mode in (b"100644", b"100755") and path.is_file() and not path.is_symlink():
            if bool(path.stat().st_mode & 0o100) != (mode == b"100755"):
                raise ValueError("exported executable mode differs from staged tree")
            data = path.read_bytes()
        else:
            raise ValueError("exported file type differs from staged tree")
        actual = hashlib.new(algorithm, f"blob {len(data)}\0".encode() + data).hexdigest()
        if actual != oid.decode():
            raise ValueError(f"exported blob differs from staged tree: {relative}")
    actual_paths = {str(path.relative_to(candidate)) for path in candidate.rglob("*")}
    if actual_paths != expected:
        raise ValueError("exported paths differ from staged tree")


def export_snapshot(tree, candidate):
    candidate.mkdir()
    with tempfile.TemporaryDirectory(prefix="gz-sim-export-index-") as directory:
        env = dict(os.environ, GIT_INDEX_FILE=str(Path(directory) / "index"))
        run(["git", "read-tree", tree], env=env)
        run(["git", "checkout-index", "--all", f"--prefix={candidate}/"], env=env)
    validate_snapshot(tree, candidate)


def retain_profile_evidence(candidate, output, hashes, missing):
    """Copy separate raw evidence, rejecting symlinks and incomplete profiles."""
    coverage = candidate / "artifacts/coverage"
    index = coverage / "profile-index.json"
    if index.is_symlink():
        raise ValueError("unsafe profile index")
    if not index.exists():
        return  # Older staged callers can still retain their legacy evidence.
    if not index.resolve().is_relative_to(candidate.resolve()):
        raise ValueError("unsafe profile index")
    data = json.loads(index.read_text())
    run_name = data.get("run", "")
    if not re.fullmatch(r"profiles-[a-z0-9_]+", run_name) or data.get("profiles") != ["headless", "renderer"]:
        raise ValueError("invalid staged profile index")
    (output / "profile-index.json").write_bytes(index.read_bytes())
    hashes["profile-index.json"] = hashlib.sha256(index.read_bytes()).hexdigest()
    contract_path = candidate / "planning/coverage-profiles.json"
    if contract_path.is_file():
        if contract_path.is_symlink() or not contract_path.resolve().is_relative_to(candidate.resolve()):
            raise ValueError("unsafe staged coverage contract")
        contract = json.loads(contract_path.read_text())
        if contract.get("version") == 2 and "zenoh_reference" not in data:
            missing.append("zenoh_reference")
    if "zenoh_reference" in data:
        name = "zenoh-native-reference.json"
        source = coverage / run_name / name
        if (not source.is_file() or source.is_symlink()
                or not source.resolve().is_relative_to(candidate.resolve())):
            missing.append(name)
        else:
            blob = source.read_bytes()
            digest = hashlib.sha256(blob).hexdigest()
            if digest != data["zenoh_reference"].get("environment_sha256"):
                raise ValueError("staged selected Zenoh reference metadata differs")
            (output / name).write_bytes(blob)
            hashes[name] = digest
    for profile in data["profiles"]:
        base = coverage / run_name / profile
        required = ["test-output.log", "coverage.json", "coverage.lcov", "summary.json",
                    "evidence.json", "merged.profdata", "proto-generated.json",
                    "proto-facade-audit.json", "proto-facade-audit-output.log", "proto-generated.rs", "proto-descriptor.pb", "proto-coverage.json", "coverage-unfiltered.lcov"]
        if profile == "renderer":
            required.append("facade-audit.json")
            if contract_path.is_file() and RGB_FEATURE in contract["profiles"]["renderer"].get("features", []):
                required.append("renderer-build-output.log")
        if "zenoh_reference" in data:
            required.append("native-peers.json")
            peers = base / "native-peers"
            if not peers.is_dir() or peers.is_symlink():
                missing.append(f"{run_name}/{profile}/native-peers")
            else:
                peer_index = base / "native-peers.json"
                if peer_index.is_file() and not peer_index.is_symlink():
                    expected = {"camera"} if profile == "renderer" else {"clock", "rate", "force"}
                    try:
                        actual = check_native_peer_evidence(peers, expected, {
                            "reference_sha256": data["zenoh_reference"]["environment_sha256"],
                            "helper_sha256": data["inputs"]["scripts/zenoh_peer.py"],
                            "fixtures": {client: data["inputs"][path]
                                         for client, path in zenoh_peer.CLIENTS.items()},
                        })
                        if json.loads(peer_index.read_text()) != actual:
                            raise ValueError("native peer index differs")
                    except (ValueError, KeyError):
                        missing.append(f"{run_name}/{profile}/native-peers evidence")
                for artifact in sorted(peers.rglob("*")):
                    if artifact.is_symlink() or not artifact.resolve().is_relative_to(candidate.resolve()):
                        raise ValueError("unsafe native peer artifact")
                    if artifact.is_file():
                        required.append(str(artifact.relative_to(base)))
        raw = base / "raw"
        if (not raw.is_dir() or raw.is_symlink()
                or not raw.resolve().is_relative_to(candidate.resolve())):
            missing.append(f"{run_name}/{profile}/raw")
            raw_files = []
        else:
            raw_files = sorted(raw.glob("*.profraw"))
            if not raw_files:
                missing.append(f"{run_name}/{profile}/raw/*.profraw")
        entries = [base / name for name in required] + raw_files
        for source in entries:
            name = str(source.relative_to(coverage))
            if (not source.is_file() or source.is_symlink()
                    or not source.resolve().is_relative_to(candidate.resolve())):
                missing.append(name)
                continue
            target = output / name
            target.parent.mkdir(parents=True, exist_ok=True)
            blob = source.read_bytes()
            target.write_bytes(blob)
            hashes[name] = hashlib.sha256(blob).hexdigest()


def retain_rapier_evidence(candidate, output, hashes, missing):
    """Retain the additional native-off proof, without its disposable target."""
    marker = candidate / "scripts/qualify-rapier-runtime.py"
    if not marker.exists():
        return  # Legacy retention fixtures have no additional configuration.
    if marker.is_symlink() or not marker.resolve().is_relative_to(candidate.resolve()):
        raise ValueError("unsafe native-off qualifier source")
    spec = importlib.util.spec_from_file_location(
        "retained_rapier_qualifier", marker)
    qualifier = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(qualifier)
    directory = candidate / "artifacts/coverage/rapier-runtime"
    report = None
    try:
        report = qualifier.validate_evidence(candidate, directory)
    except (ValueError, FileNotFoundError) as error:
        missing.append("rapier-runtime: " + str(error))
    if not directory.exists():
        return
    if directory.is_symlink() or not directory.resolve().is_relative_to(candidate.resolve()):
        raise ValueError("unsafe native-off evidence directory")
    expected = ({**report["artifacts_sha256"],
                 "result.json": hashlib.sha256((directory / "result.json").read_bytes()).hexdigest()}
                if report is not None else None)
    copied = set()
    for source in sorted(directory.rglob("*")):
        if source.is_symlink() or not source.resolve().is_relative_to(directory.resolve()):
            raise ValueError("unsafe native-off artifact during retention")
        if source.is_file():
            relative = str(source.relative_to(directory))
            blob = source.read_bytes()
            if expected is not None and hashlib.sha256(blob).hexdigest() != expected.get(relative):
                raise ValueError("native-off artifact changed during retention")
            name = "rapier-runtime/" + relative
            target = output / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(blob)
            target.chmod(source.stat().st_mode & 0o777)
            hashes[name] = hashlib.sha256(blob).hexdigest()
            copied.add(relative)
    if expected is not None and copied != set(expected):
        raise ValueError("native-off artifact disappeared during retention")


def retain_staged_evidence(tree, candidate, state):
    directory = ROOT / "artifacts/staged"
    directory.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=tree + "-", dir=directory))
    hashes, missing = {}, []
    for name in ("test-output.log", "evidence.json", "summary.json"):
        source = candidate / "artifacts/coverage" / name
        if (source.is_file() and not source.is_symlink()
                and source.resolve().is_relative_to(candidate.resolve())):
            data = source.read_bytes()
            (output / name).write_bytes(data)
            hashes[name] = hashlib.sha256(data).hexdigest()
        else:
            missing.append(name)
    profile_missing_at = len(missing)
    profile_contract = candidate / "planning/coverage-profiles.json"
    if profile_contract.is_file() and not (candidate / "artifacts/coverage/profile-index.json").is_file():
        missing.append("profile-index.json")
    retain_profile_evidence(candidate, output, hashes, missing)
    retain_rapier_evidence(candidate, output, hashes, missing)
    profile_missing = missing[profile_missing_at:]
    (output / "stage-result.json").write_text(json.dumps({
        "tree": tree, "checks_state": state,
        "evidence_state": "partial" if missing else "complete",
        "artifacts_sha256": hashes,
        "missing_artifacts": missing,
    }, indent=2) + "\n")
    print("Staged evidence:", output, flush=True)
    if state == "passed" and profile_missing:
        raise ValueError(f"missing retained profile evidence: {profile_missing}")


def staged():
    tree = run(["git", "write-tree"], capture_output=True, text=True).stdout.strip()
    with tempfile.TemporaryDirectory(prefix="gz-sim-index-") as directory:
        candidate = Path(directory) / "repo"
        state = "export_failed"
        failure = None
        try:
            export_snapshot(tree, candidate)
            changed = changed_sources({}, index=True)
            manifest = Path(directory) / "changed-lines.json"
            manifest.write_text(json.dumps({path: sorted(lines) for path, lines in changed.items()}))
            # Child Git commands must not inherit the caller's worktree or index.
            env = os.environ.copy()
            keys = run(["git", "rev-parse", "--local-env-vars"], capture_output=True, text=True).stdout.splitlines()
            for key in keys:
                env.pop(key, None)
            state = "checks_failed"
            subprocess.run([sys.executable, str(candidate / "scripts/verify.py"), "--changed-lines", str(manifest)], cwd=candidate, env=env, check=True)
            state = "index_changed"
            if run(["git", "write-tree"], capture_output=True, text=True).stdout.strip() != tree:
                raise ValueError("index changed during verification")
            state = "passed"
        except BaseException as error:
            failure = error
            raise
        finally:
            try:
                retain_staged_evidence(tree, candidate, state)
            except Exception as error:
                if failure is None:
                    raise
                print(f"Could not retain staged evidence: {error}", file=sys.stderr)


def main():
    os.environ["RUSTUP_TOOLCHAIN"] = "1.97.1"
    os.environ.setdefault("GZ_REFERENCE_PREFIX", str(ROOT / "artifacts/native-reference/install"))
    os.environ.setdefault("GZ_ZENOH_REFERENCE_ROOT", str(ROOT / "artifacts/zenoh-native-reference"))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--staged", action="store_true")
    parser.add_argument("--coverage-only", action="store_true")
    parser.add_argument("--changed-lines", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.staged:
        staged()
        return
    if not args.coverage_only:
        run(["cargo", "fmt", "--all", "--", "--check"])
        run(["cargo", "clippy", "--locked", "--workspace", "--all-targets", "--", "-D", "warnings"])
        render_packages = sorted(RENDER_PACKAGES | RENDER_SHARED_PACKAGES)
        selected = sum((["--package", name] for name in [*render_packages, TEST_SUPPORT_PACKAGE]), [])
        run(["cargo", "clippy", "--locked", "--all-targets", *selected,
             "--features", ",".join(RENDER_FEATURES), "--", "-D", "warnings"])
        doc_packages = sorted((RENDER_PACKAGES - {"gz-render-worker"}) | {"gz-sim-core", "gz-sim-scene"})
        doc_selected = sum((["--package", name] for name in doc_packages), [])
        run(["cargo", "test", "--locked", "--doc", *doc_selected, "--features", "gz-renderer/gpu"])
        run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_*.py"])
        run(["cargo", "test", "--locked", "--workspace", "--doc"])
    run([sys.executable, "scripts/qualify-rapier-runtime.py"])
    coverage(args.changed_lines)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, subprocess.CalledProcessError) as error:
        print(error, file=sys.stderr)
        if isinstance(error, subprocess.CalledProcessError):
            print(error.stdout or "", file=sys.stderr)
            print(error.stderr or "", file=sys.stderr)
        raise SystemExit(1)
