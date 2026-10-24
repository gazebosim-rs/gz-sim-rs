#!/usr/bin/env python3
"""Verify the current tree or an isolated copy of the Git index."""

import argparse
import hashlib
import json
import os
from pathlib import Path
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


def coverage():
    out = ROOT / "artifacts/coverage"
    out.mkdir(parents=True, exist_ok=True)
    # A unique profile directory prevents previous source versions inflating coverage.
    with tempfile.TemporaryDirectory(prefix="raw-", dir=out) as directory:
        env = os.environ.copy()
        env["CARGO_TARGET_DIR"] = str(ROOT / "target/coverage")
        env["RUSTFLAGS"] = "-C instrument-coverage"
        env["LLVM_PROFILE_FILE"] = directory + "/%p-%m.profraw"
        result = run(["cargo", "test", "--locked", "--workspace", "--all-targets", "--message-format=json"], env=env, capture_output=True, text=True)
        (out / "test-output.log").write_text(result.stdout + result.stderr)
        objects = []
        for line in result.stdout.splitlines():
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
        command = [str(llvm / "llvm-cov"), "export", "-instr-profile=" + str(merged)]
        for binary in dict.fromkeys(objects):
            command += ["-object", binary]
        with (out / "coverage.json").open("w") as output:
            run(command, stdout=output)
    report = json.loads((out / "coverage.json").read_text())
    sources = production_sources()
    summaries = check_coverage(report, sources)
    (out / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
    evidence = {"rustc": version, "coverage": summaries, "sources": {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for paths in sources.values() for path in paths
    }}
    (out / "evidence.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(summaries, indent=2))


def staged():
    tree = run(["git", "write-tree"], capture_output=True, text=True).stdout.strip()
    with tempfile.TemporaryDirectory(prefix="gz-sim-index-") as directory:
        archive = run(["git", "archive", "--format=tar", tree], capture_output=True).stdout
        subprocess.run(["tar", "-xf", "-", "-C", directory], input=archive, check=True)
        subprocess.run([sys.executable, str(Path(directory) / "scripts/verify.py")], cwd=directory, check=True)
        if run(["git", "write-tree"], capture_output=True, text=True).stdout.strip() != tree:
            raise ValueError("index changed during verification")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--staged", action="store_true")
    parser.add_argument("--coverage-only", action="store_true")
    args = parser.parse_args()
    if args.staged:
        staged()
        return
    if not args.coverage_only:
        run(["cargo", "fmt", "--all", "--", "--check"])
        run(["cargo", "clippy", "--locked", "--workspace", "--all-targets", "--", "-D", "warnings"])
        run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-p", "test_*.py"])
        run(["cargo", "test", "--locked", "--workspace", "--doc"])
    coverage()


if __name__ == "__main__":
    try:
        main()
    except (ValueError, subprocess.CalledProcessError) as error:
        print(error, file=sys.stderr)
        if isinstance(error, subprocess.CalledProcessError):
            print(error.stdout or "", file=sys.stderr)
            print(error.stderr or "", file=sys.stderr)
        raise SystemExit(1)
