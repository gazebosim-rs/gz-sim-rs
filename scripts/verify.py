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


def check_coverage(report, crates):
    """Reject missing instrumentation and enforce each crate independently."""
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
        missing = set(sources) - files.keys()
        if missing:
            raise ValueError(f"{crate}: uninstrumented sources: {sorted(map(str, missing))}")
        counts = {metric: [0, 0] for metric in total}
        for source in sources:
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


def production_sources():
    metadata = json.loads(run(["cargo", "metadata", "--locked", "--no-deps", "--format-version=1"], capture_output=True, text=True).stdout)
    return {
        package["name"]: sorted(
            list((Path(package["manifest_path"]).parent / "src").rglob("*.rs"))
            + ([Path(package["manifest_path"]).parent / "build.rs"]
               if (Path(package["manifest_path"]).parent / "build.rs").exists() else [])
        ) for package in metadata["packages"] if package["id"] in metadata["workspace_members"]
    }


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


def check_changed_coverage(lcov, changed, crates, executable_counts=None):
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


def coverage(changed_file=None):
    out = ROOT / "artifacts/coverage"
    out.mkdir(parents=True, exist_ok=True)
    # A unique profile directory prevents previous source versions inflating coverage.
    with tempfile.TemporaryDirectory(prefix="raw-", dir=out) as directory:
        env = os.environ.copy()
        env["CARGO_TARGET_DIR"] = str(ROOT / "target/coverage")
        env["RUSTFLAGS"] = ""
        env.pop("CARGO_ENCODED_RUSTFLAGS", None)
        wrapper = ROOT / "scripts/rustc-coverage.py"
        digest = hashlib.sha256(wrapper.read_bytes()).hexdigest()
        executable = out / f"rustc-coverage-{digest}"
        executable.write_bytes(wrapper.read_bytes())
        executable.chmod(0o755)
        env["RUSTC_WORKSPACE_WRAPPER"] = str(executable)
        env["LLVM_PROFILE_FILE"] = directory + "/%p-%m.profraw"
        result = run(["cargo", "test", "--locked", "--workspace", "--all-targets", "--message-format=json"], env=env, capture_output=True, text=True)
        (out / "test-output.log").write_text(result.stdout + result.stderr)
        objects = []
        for line in result.stdout.split("\n"):
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            if item.get("reason") == "compiler-artifact" and item.get("executable"):
                objects.append(item["executable"])
        if not objects:
            raise ValueError("no instrumented test binaries")
        sysroot = Path(run(["rustc", "--print", "sysroot"], capture_output=True, text=True).stdout.strip())
        version = run(["rustc", "-vV"], capture_output=True, text=True).stdout
        host = next(line.split(": ", 1)[1] for line in version.splitlines() if line.startswith("host: "))
        llvm = sysroot / "lib/rustlib" / host / "bin"
        raw = sorted(Path(directory).glob("*.profraw"))
        if not raw:
            raise ValueError("no raw coverage profiles")
        merged = out / "merged.profdata"
        run([str(llvm / "llvm-profdata"), "merge", "-sparse", *map(str, raw), "-o", str(merged)])
        command = [str(llvm / "llvm-cov"), "export", "--fatal-warnings", "-instr-profile=" + str(merged)]
        for binary in dict.fromkeys(objects):
            command += ["-object", binary]
        with (out / "coverage.json").open("w") as output:
            run(command, stdout=output)
        lcov = run([*command, "-format=lcov"], capture_output=True).stdout.decode("utf-8")
        (out / "coverage.lcov").write_text(lcov)
    report = json.loads((out / "coverage.json").read_text())
    sources = production_sources()
    summaries = check_coverage(report, sources)
    changed = json.loads(Path(changed_file).read_text()) if changed_file else changed_sources(sources)
    executable_counts = {Path(file["filename"]).resolve(): file["summary"]["lines"]["count"]
                         for data in report["data"] for file in data["files"]}
    changed_totals = check_changed_coverage(lcov, changed, sources, executable_counts)
    (out / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
    evidence = {"rustc": version, "coverage": summaries, "changed_lines": changed_totals, "sources": {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for paths in sources.values() for path in paths
    }}
    (out / "evidence.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(summaries, indent=2))
    print("Changed executable lines:", json.dumps(changed_totals))


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
    (output / "stage-result.json").write_text(json.dumps({
        "tree": tree, "checks_state": state,
        "evidence_state": "partial" if missing else "complete",
        "artifacts_sha256": hashes,
        "missing_artifacts": missing,
    }, indent=2) + "\n")
    print("Staged evidence:", output, flush=True)


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
