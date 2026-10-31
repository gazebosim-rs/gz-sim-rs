#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Verify the current tree or an isolated copy of the Git index."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run(command, **kwargs):
    return subprocess.run(command, cwd=ROOT, check=True, **kwargs)


def check_coverage(report, crates, zero_executable=frozenset()):
    """Reject missing instrumentation and enforce each crate independently."""
    facade = (ROOT / "crates/gz-renderer/src/lib.rs").resolve()
    if not set(zero_executable) <= {facade}:
        raise ValueError("unrecognized zero-executable source")
    if zero_executable:
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


# These names are the proposed production package contract, not yet adopted.
RENDER_PACKAGES = frozenset({
    "gz-camera-output",
    "gz-render-frame", "gz-renderer", "gz-render-protocol",
    "gz-render-client", "gz-render-worker", "gz-render-scene",
})


def load_profile_contract():
    path = ROOT / "planning/coverage-profiles.json"
    if path.is_symlink() or not path.is_file() or not path.resolve().is_relative_to(ROOT.resolve()):
        raise ValueError("missing or unsafe coverage profile contract")
    contract = json.loads(path.read_text())
    expected = {
        "version": 1,
        "profiles": {
            "headless": {"exclude_packages": sorted(RENDER_PACKAGES | {TEST_SUPPORT_PACKAGE}), "runner": "cpu"},
            "renderer": {"packages": sorted(RENDER_PACKAGES) + ["gz-sim-core", "gz-sim-scene"],
                         "features": ["gz-renderer/gpu"], "runner": "vulkan"},
        },
        "excluded_sources": [],
        "test_support_packages": [TEST_SUPPORT_PACKAGE],
    }
    if contract != expected:
        raise ValueError("invalid coverage profile contract")
    return contract


def profile_plan(sources):
    """Partition every production crate and retain core and scene in the renderer run."""
    missing = (RENDER_PACKAGES | {"gz-sim-core", "gz-sim-scene"}) - sources.keys()
    if missing:
        raise ValueError(f"required renderer profile packages missing: {sorted(missing)}")
    unexpected = {name for name in sources if name.startswith("gz-render") and name not in RENDER_PACKAGES}
    if unexpected:
        raise ValueError(f"undeclared renderer packages: {sorted(unexpected)}")
    headless = {name: files for name, files in sources.items() if name not in RENDER_PACKAGES}
    render = {name: sources[name] for name in sorted(RENDER_PACKAGES | {"gz-sim-core", "gz-sim-scene"})}
    if not headless or any(not files for files in sources.values()):
        raise ValueError("empty production crate or headless profile")
    owners = {}
    for name, files in sources.items():
        for path in files:
            path = Path(path).resolve()
            if path in owners:
                raise ValueError(f"production source has multiple owners: {path}")
            owners[path] = name
    headless_command = ["cargo", "test", "--locked", "--workspace", "--all-targets"]
    for name in sorted(RENDER_PACKAGES | {TEST_SUPPORT_PACKAGE}):
        headless_command += ["--exclude", name]
    render_command = ["cargo", "test", "--locked", "--all-targets"]
    for name in render:
        render_command += ["--package", name]
    render_command += ["--package", TEST_SUPPORT_PACKAGE, "--features", "gz-renderer/gpu"]
    return {"headless": (headless, headless_command), "renderer": (render, render_command)}


def check_profile_reports(plan, reports, zero_sources=None):
    """Each mandatory profile is independently complete and strictly covered."""
    if set(plan) != {"headless", "renderer"} or set(reports) != set(plan):
        raise ValueError("missing or unexpected required coverage profiles")
    zero_sources = zero_sources or {}
    if set(zero_sources) - {"renderer"}:
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


def profile_coverage(name, crates, command, directory, changed, wrapper, llvm, version):
    directory.mkdir(parents=True)
    raw_dir = directory / "raw"
    raw_dir.mkdir()
    source_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for paths in crates.values() for path in paths}
    auditor_source = ROOT / "tests/support/render-fixtures/src/bin/source_audit.rs"
    auditor_source_hash = (hashlib.sha256(auditor_source.read_bytes()).hexdigest()
                           if name == "renderer" and auditor_source.is_file() else None)
    env = os.environ.copy()
    env["CARGO_TARGET_DIR"] = str(ROOT / "target/coverage")
    env["RUSTFLAGS"] = ""
    env.pop("CARGO_ENCODED_RUSTFLAGS", None)
    env["RUSTC_WORKSPACE_WRAPPER"] = str(wrapper)
    env["LLVM_PROFILE_FILE"] = str(raw_dir / "%p-%m.profraw")
    if name == "renderer":
        # Worker integration tests pass this exact path as the child's LLVM_PROFILE_FILE.
        env["GZ_RENDER_WORKER_PROFILE"] = str(raw_dir / "worker-%p-%m.profraw")
    test_command = [*command, "--message-format=json"]
    if name == "renderer":
        # Preserve adapter identity in evidence, including for successful tests.
        test_command += ["--", "--nocapture"]
    result = subprocess.run(test_command, cwd=ROOT, env=env, capture_output=True, text=True)
    (directory / "test-output.log").write_text(result.stdout + result.stderr)
    if result.returncode:
        raise subprocess.CalledProcessError(result.returncode, result.args, result.stdout, result.stderr)
    objects = []
    artifacts = []
    worker_objects = []
    audit_objects = []
    for line in result.stdout.splitlines():
        try:
            item = json.loads(line)
        except json.JSONDecodeError:
            continue
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
    with (directory / "coverage.json").open("w") as output:
        run(export, stdout=output)
    lcov = run([*export, "-format=lcov"], capture_output=True).stdout.decode("utf-8")
    (directory / "coverage.lcov").write_text(lcov)
    report = json.loads((directory / "coverage.json").read_text())
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
                    "proof": "syn2-renderer-facade-v1", "source_sha256": source_hashes[str(path.relative_to(ROOT))],
                    "auditor": str(facade_audit)} for path in zero_executable},
                "objects": object_evidence}
    (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    (directory / "evidence.json").write_text(json.dumps(evidence, indent=2) + "\n")
    return report, zero_executable


def coverage(changed_file=None):
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
                                 wrapper, ROOT / "scripts/verify.py",
                                 ROOT / "planning/coverage-profiles.json",
                                 ROOT / "tests/support/render-fixtures/src/bin/source_audit.rs", *manifests]))
    input_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in inputs}
    host = next(line.split(": ", 1)[1] for line in version.splitlines() if line.startswith("host: "))
    llvm = sysroot / "lib/rustlib" / host / "bin"
    run_dir = Path(tempfile.mkdtemp(prefix="profiles-", dir=out))
    index = {"run": run_dir.name, "profiles": sorted(plan), "rustc": version,
             "sources": hashes, "inputs": input_hashes}
    (out / "profile-index.json").write_text(json.dumps(index, indent=2) + "\n")
    reports, zero_sources = {}, {}
    for name, (crates, command) in plan.items():
        reports[name], proof = profile_coverage(name, crates, command, run_dir / name,
                                                changed, executable, llvm, version)
        if proof:
            zero_sources[name] = proof
    summaries = check_profile_reports(plan, reports, zero_sources)
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
    for profile in data["profiles"]:
        base = coverage / run_name / profile
        required = ["test-output.log", "coverage.json", "coverage.lcov", "summary.json",
                    "evidence.json", "merged.profdata"]
        if profile == "renderer":
            required.append("facade-audit.json")
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
        render_packages = sorted(RENDER_PACKAGES | {"gz-sim-core", "gz-sim-scene"})
        selected = sum((["--package", name] for name in [*render_packages, TEST_SUPPORT_PACKAGE]), [])
        run(["cargo", "clippy", "--locked", "--all-targets", *selected,
             "--features", "gz-renderer/gpu", "--", "-D", "warnings"])
        doc_packages = sorted((RENDER_PACKAGES - {"gz-render-worker"}) | {"gz-sim-core", "gz-sim-scene"})
        doc_selected = sum((["--package", name] for name in doc_packages), [])
        run(["cargo", "test", "--locked", "--doc", *doc_selected, "--features", "gz-renderer/gpu"])
        run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_*.py"])
        run(["cargo", "test", "--locked", "--workspace", "--doc"])
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
