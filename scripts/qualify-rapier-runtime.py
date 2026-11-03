#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Additional native-off execution proof; never replaces the full coverage profiles."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parent))
import verify

ROOT = Path(__file__).resolve().parents[1]
SCENE = "crates/gz-sim-scene/src/lib.rs"
RETURN = "return Err(LoadError::BackendUnavailable(engine));"
TARGETS = {
    "gz-sim-scene": ("no_native_backends", [
        "absent_native_backends_return_the_selected_engine_without_fallback",
        "rapier_and_tpe_still_load_and_step_without_native_engines",
        "disabled_native_build_scripts_ignore_absent_or_ambient_native_metadata",
        "enabled_native_build_scripts_keep_verified_rpath_and_missing_metadata_failure",
    ]),
    "gz-sim-server": ("no_native_build", [
        "disabled_native_build_scripts_ignore_absent_or_ambient_native_metadata",
        "enabled_native_build_scripts_keep_verified_rpath_and_missing_metadata_failure",
    ]),
    "gz-sim-cli": ("no_native_runtime", [
        "non_native_executable_runs_rapier_and_tpe",
        "native_short_names_and_plugin_aliases_fail_instead_of_falling_back",
        "disabled_native_build_scripts_ignore_absent_or_ambient_native_metadata",
        "enabled_native_build_scripts_keep_verified_rpath_and_missing_metadata_failure",
    ]),
}
EXECUTABLE_SOURCES = {
    "gz-sim-rs": "crates/gz-sim-cli/src/main.rs",
    "no_native_backends": "crates/gz-sim-scene/tests/no_native_backends.rs",
    "no_native_runtime": "crates/gz-sim-cli/tests/no_native_runtime.rs",
    "no_native_build": "crates/gz-sim-server/tests/no_native_build.rs",
}
RGB_TESTS = [
    "camera_worker_flag_requires_server_mode",
    "no_configured_camera_never_starts_the_operator_worker",
    "admitted_rgb_camera_without_image_interest_keeps_its_worker_lazy",
    "camera_worker_and_calibration_errors_fail_before_a_successful_run",
    "camera_and_server_use_distinct_listeners_and_release_the_fixed_server_address",
    "camera_owner_shuts_down_when_viewer_setup_fails_and_its_listener_can_be_reused",
    "configured_camera_without_zenoh_config_uses_the_default_network_lazily",
    "actual_image_interest_bounds_silent_worker_backpressure_and_propagates_spawn_failure",
]
RGB_SOURCES = {"gz-sim-rs": EXECUTABLE_SOURCES["gz-sim-rs"],
               "rgb_camera": "crates/gz-sim-cli/tests/rgb_camera.rs"}
GPU_PACKAGES = {"gz-renderer", "gz-render-worker", "bevy_render", "bevy_window", "bevy_winit",
                "bevy_core_pipeline", "bevy_pbr", "bevy_shader", "ash", "naga", "vulkano",
                "winit", "raw-window-handle"}
GPU_LIBRARIES = ("libvulkan", "libEGL", "libGL.so", "libGLX", "libOpenGL", "libwgpu")


def rgb_command():
    return ["cargo", "test", "--locked", "--offline", "-p", "gz-sim-cli",
            "--no-default-features", "--features", "rgb-camera", "--test", "rgb_camera",
            "--message-format=json", "--", "--nocapture", "--test-threads=1"]


def require_rgb_tests(stdout):
    totals = re.findall(r"test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;", stdout)
    if (len(totals) != 1 or tuple(map(int, totals[0])) != (len(RGB_TESTS), 0, 0)
            or any(f"test {name} ... ok" not in stdout for name in RGB_TESTS)):
        raise ValueError("missing/failed/skipped native-free RGB cases")


def rgb_artifacts(events, source_root, target):
    objects = {}
    for event in events:
        if event.get("reason") != "compiler-artifact":
            continue
        name = event["target"]["name"]
        package = name.replace("_", "-")
        if (package in {"gz-physics-dart", "gz-physics-mujoco"}
                or name in GPU_PACKAGES or package in GPU_PACKAGES or package.startswith("wgpu")
                or "native-backends" in event["features"]):
            raise ValueError("native/GPU dependency in native-free RGB artifacts")
        if not event.get("executable"):
            continue
        original = event["executable"]
        path = Path(original)
        if (not path.is_absolute() or ".." in path.parts or str(path) != original
                or not path.is_relative_to(target) or name not in RGB_SOURCES
                or name in objects or event["features"] != ["rgb-camera"]
                or event["target"]["kind"] != ["bin" if name == "gz-sim-rs" else "test"]
                or event["profile"]["test"] != (name != "gz-sim-rs")
                or event["target"]["src_path"] != str(source_root / RGB_SOURCES[name])):
            raise ValueError("RGB executable is not its actual feature/target/source")
        objects[name] = event
    if set(objects) != set(RGB_SOURCES):
        raise ValueError("missing actual native-free RGB CLI or test executable")
    return objects


def validate_rgb_closure(root, closure, binary):
    spec = importlib.util.spec_from_file_location("rgb_runtime_closure", root / "scripts/check-rapier-runtime.py")
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    packages = set(closure.get("packages", []))
    libraries = closure.get("libraries", "")
    command = ["cargo", "tree", "--locked", "--offline", "-p", "gz-sim-cli", "--no-default-features",
               "--edges", "normal,build", "--prefix", "none", "--format", "{p}", "--features", "rgb-camera"]
    if (closure.get("binary") != binary or closure.get("cargo_command") != command
            or checker.validate_closure("\n".join(sorted(packages)), libraries) != sorted(packages)
            or not {"gz-camera-output", "gz-render-client", "gz-render-protocol", "gz-render-scene", "gz-render-frame"} <= packages
            or packages & GPU_PACKAGES or any(name.startswith("wgpu") for name in packages)
            or not libraries.strip() or any(name in libraries for name in GPU_LIBRARIES)):
        raise ValueError("native-free RGB closure is missing, mislabeled or includes GPU dependencies")


def validate_rgb_evidence(root, directory, report, observed_root, hashes):
    if (not isinstance(report, dict) or report.get("state") != "passed"
            or report.get("features") != ["rgb-camera"] or report.get("private_target_removed") is not True):
        raise ValueError("missing or incomplete native-free RGB evidence")
    target_text = report.get("target_root", "")
    target = Path(target_text)
    if (not target.is_absolute() or ".." in target.parts or str(target) != target_text
            or target.parent != observed_root / "target" or not target.name.startswith("rapier-rgb-")):
        raise ValueError("unsafe original RGB target attribution")
    rgb = directory / "rgb-camera"
    if (rgb / "target").exists():
        raise ValueError("RGB compilation cache was not retired")
    required = {"inputs.json", "command.json", "stdout", "stderr", "cargo-events.json", "objects.json",
                "closure.stdout", "closure.stderr", "coverage.json", "coverage.lcov", "merged.profdata"}
    if (not {"rgb-camera/" + name for name in required} <= hashes.keys()
            or not any(name.startswith("rgb-camera/raw/") and name.endswith(".profraw") for name in hashes)):
        raise ValueError("missing native-free RGB raw/report evidence")
    if json.loads((rgb / "inputs.json").read_text()) != inputs(root):
        raise ValueError("native-free RGB source/input binding differs")
    if json.loads((rgb / "command.json").read_text()) != rgb_command():
        raise ValueError("native-free RGB command differs")
    require_rgb_tests((rgb / "stdout").read_text())
    coverage = json.loads((rgb / "coverage.json").read_text())
    names = {entry["filename"] for data in coverage.get("data", []) for entry in data.get("files", [])}
    lcov = (rgb / "coverage.lcov").read_text()
    for source in ["crates/gz-sim-cli/src/main.rs", "crates/gz-sim-cli/src/live.rs"]:
        name = str(observed_root / source)
        if name not in names or f"SF:{name}\n" not in lcov:
            raise ValueError("native-free RGB reports lack their actual CLI source")
    events = json.loads((rgb / "cargo-events.json").read_text())
    objects = rgb_artifacts(events, observed_root, target)
    bindings = report.get("objects_bound_after_execution")
    if not bindings or json.loads((rgb / "objects.json").read_text()) != bindings:
        raise ValueError("missing retained native-free RGB object bindings")
    seen, saved_names = set(), set()
    for item in bindings:
        event = item.get("cargo_artifact", {})
        name = event.get("target", {}).get("name")
        saved = item.get("retained_path", "")
        if (name in seen or saved in saved_names or objects.get(name) != event or item.get("original_path") != event.get("executable")
                or not saved.startswith("rgb-camera/objects/") or hashes.get(saved) != item.get("sha256")):
            raise ValueError("unbound native-free RGB object copy")
        seen.add(name)
        saved_names.add(saved)
    if seen != set(RGB_SOURCES):
        raise ValueError("missing retained native-free RGB executable")
    validate_rgb_closure(root, json.loads((rgb / "closure.stdout").read_text()), objects["gz-sim-rs"]["executable"])


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def inputs(root=ROOT):
    paths = [root / name for name in ["Cargo.toml", "Cargo.lock", "rust-toolchain.toml",
             ".cargo/config.toml", "scripts/rustc-coverage.py", "scripts/verify.py",
             "scripts/qualify-rapier-runtime.py", "scripts/check-rapier-runtime.py",
             "crates/gz-sim-cli/tests/empty.sdf", "crates/gz-camera-output/tests/fixtures/m5-rgb.sdf"]]
    paths += sorted((root / "crates").rglob("*.rs"))
    paths += sorted((root / "crates").glob("*/Cargo.toml"))
    paths += sorted((root / "crates/gz-sim-proto/schemas").rglob("*.proto"))
    paths += [root / "crates/gz-sim-proto/schema-source.json",
              root / "crates/gz-sim-proto/schemas/manifest.json"]
    patched = root / "third_party/prost-derive-0.14.4"
    paths += [patched / "Cargo.toml", patched / "LICENSE", *sorted((patched / "src").rglob("*.rs"))]
    if any(path.is_symlink() or not path.resolve().is_relative_to(root.resolve()) for path in paths):
        raise ValueError("unsafe native-off source/input")
    return {str(path.relative_to(root)): digest(path) for path in paths}


def validate_evidence(root, directory, report=None):
    """Validate only this auxiliary proof, without invoking a workload or compiler."""
    if (directory.is_symlink() or not directory.is_dir()
            or not directory.resolve().is_relative_to(root.resolve())):
        raise ValueError("missing or unsafe native-off evidence directory")
    entries = list(directory.rglob("*"))
    if any(path.is_symlink() or not path.resolve().is_relative_to(directory.resolve()) for path in entries):
        raise ValueError("unsafe native-off evidence path")
    report = report if report is not None else json.loads((directory / "result.json").read_text())
    if report.get("state") != "passed" or report.get("private_target_removed") is not True:
        raise ValueError("incomplete native-off evidence")
    if (directory / "target").exists():
        raise ValueError("private compilation cache was not retired")
    if report.get("sources_and_inputs") != inputs(root):
        raise ValueError("native-off source/input binding differs")
    observed_root = Path(report.get("source_root", ""))
    target_root = Path(report.get("target_root", ""))
    if (not observed_root.is_absolute() or ".." in observed_root.parts
            or not target_root.is_absolute() or ".." in target_root.parts
            or target_root.parent != observed_root / "target"
            or not target_root.name.startswith("rapier-runtime-")):
        raise ValueError("missing original native-off source/target attribution")
    hashes = report.get("artifacts_sha256")
    if not isinstance(hashes, dict) or not hashes:
        raise ValueError("missing native-off artifact bindings")
    required = {"inputs.json", "commands.json", "cargo-events.json", "objects.json", "rustc-coverage.py",
                "closure.stdout", "closure.stderr", "coverage.json", "coverage.lcov", "merged.profdata"}
    required |= {test + "." + stream for test, _ in TARGETS.values() for stream in ["stdout", "stderr"]}
    if not required <= hashes.keys() or not any(name.startswith("raw/") and name.endswith(".profraw") for name in hashes):
        raise ValueError("missing required native-off raw/report evidence")
    actual = {str(path.relative_to(directory)) for path in entries if path.is_file()} - {"result.json"}
    if actual != set(hashes):
        raise ValueError("missing or undeclared native-off artifact")
    for name, expected in hashes.items():
        path = Path(name)
        if path.is_absolute() or ".." in path.parts or str(path) != name or name == "result.json":
            raise ValueError("unsafe native-off manifest path")
        if digest(directory / path) != expected:
            raise ValueError("tampered native-off artifact: " + name)
    if json.loads((directory / "inputs.json").read_text()) != report["sources_and_inputs"]:
        raise ValueError("native-off input record differs")
    objects = report.get("objects_bound_after_execution")
    if not objects or json.loads((directory / "objects.json").read_text()) != objects:
        raise ValueError("missing native-off retained objects")
    events = json.loads((directory / "cargo-events.json").read_text())
    object_names, retained_names = set(), set()
    for item in objects:
        name = item.get("retained_path", "")
        if not name.startswith("objects/") or hashes.get(name) != item.get("sha256"):
            raise ValueError("unbound native-off object copy")
        artifact = item.get("cargo_artifact", {})
        target_name = artifact.get("target", {}).get("name")
        original_text = item.get("original_path", "")
        original = Path(original_text)
        if (not original.is_absolute() or ".." in original.parts
                or str(original) != original_text):
            raise ValueError("unsafe original Cargo executable path")
        if (artifact not in events or artifact.get("reason") != "compiler-artifact"
                or target_name not in EXECUTABLE_SOURCES or target_name in object_names
                or name in retained_names or not original.is_relative_to(target_root)
                or artifact.get("executable") != str(original)
                or Path(artifact["target"]["src_path"]) != observed_root / EXECUTABLE_SOURCES[target_name]
                or artifact["profile"]["test"] != (target_name != "gz-sim-rs")):
            raise ValueError("retained object is not its actual Cargo target/source")
        object_names.add(target_name)
        retained_names.add(name)
    if object_names != set(EXECUTABLE_SOURCES):
        raise ValueError("missing retained CLI or targeted test object")
    for event in events:
        if event.get("reason") == "compiler-artifact":
            if (event["target"]["name"].replace("_", "-") in {"gz-physics-dart", "gz-physics-mujoco"}
                    or "native-backends" in event["features"]):
                raise ValueError("native artifact in reduced proof")
    source = (root / SCENE).read_text()
    proof = disabled_region(json.loads((directory / "coverage.json").read_text()),
                            (directory / "coverage.lcov").read_text(), observed_root, source)
    if proof != report.get("disabled_backend"):
        raise ValueError("disabled-backend line/region proof differs")
    validate_rgb_evidence(root, directory, report.get("rgb_camera"), observed_root, hashes)
    return report


def disabled_region(report, lcov, root, source):
    lines = source.splitlines()
    returns = [i + 1 for i, line in enumerate(lines) if line.strip() == RETURN]
    if len(returns) != 1:
        raise ValueError("expected one exact disabled-backend return")
    line = returns[0]
    if (lines[line - 2].strip() != "PhysicsEngine::Dart | PhysicsEngine::Mujoco => {"
            or lines[line - 3].strip() != '#[cfg(not(feature = "native-backends"))]'
            or lines[line].strip() != "}"):
        raise ValueError("disabled backend arm/source shape changed")
    expected = (root / SCENE).resolve()
    hits, active = [], False
    for value in lcov.splitlines():
        if value.startswith("SF:"):
            active = Path(value[3:]).resolve() == expected
        elif active and value.startswith("DA:"):
            number, count, *_ = value[3:].split(",")
            number, count = int(number), int(count)
            if number == line:
                hits.append(count)
    if len(hits) != 1 or hits[0] <= 0:
        raise ValueError("actual no-native return line did not execute")
    regions = []
    for data in report["data"]:
        for function in data.get("functions", []):
            for region in function.get("regions", []):
                if (len(region) >= 8 and region[7] == 0
                        and Path(function["filenames"][region[5]]).resolve() == expected
                        and line - 1 <= region[0] <= line <= region[2] <= line + 1):
                    regions.append(region)
    if not regions or not any(region[4] > 0 for region in regions):
        raise ValueError("disabled-backend code region is absent or cold")
    return {"source": SCENE, "line": line, "line_hits": hits[0], "regions": regions}


def checked_artifacts(events, target):
    artifacts = [event for event in events if event.get("reason") == "compiler-artifact"]
    for item in artifacts:
        name = item["target"]["name"].replace("_", "-")
        if name in {"gz-physics-dart", "gz-physics-mujoco"} or "native-backends" in item["features"]:
            raise ValueError("native backend or enabled native feature in actual Cargo artifacts")
    executables = {}
    for item in artifacts:
        if (not item.get("executable")
                or not {"bin", "test"}.intersection(item["target"]["kind"])):
            continue
        path = Path(item["executable"]).resolve()
        if not path.is_relative_to(target.resolve()) or not path.is_file():
            raise ValueError("Cargo executable outside the independent target")
        source = Path(item["target"]["src_path"]).resolve()
        if not source.is_relative_to(ROOT / "crates"):
            raise ValueError("Cargo executable source outside the workspace")
        executables[str(path)] = item
    names = {item["target"]["name"] for item in executables.values()}
    if not set(EXECUTABLE_SOURCES) <= names:
        raise ValueError("missing actual CLI or no-native test executable")
    for item in executables.values():
        name = item["target"]["name"]
        if name in EXECUTABLE_SOURCES and Path(item["target"]["src_path"]).resolve() != ROOT / EXECUTABLE_SOURCES[name]:
            raise ValueError("target name does not bind its actual owned source")
    return executables


def qualify_rgb(out, environment, version, source_hashes, llvm):
    rgb = out / "rgb-camera"
    rgb.mkdir()
    raw = rgb / "raw"
    raw.mkdir()
    cache_root = ROOT / "target"
    target = Path(tempfile.mkdtemp(prefix="rapier-rgb-", dir=cache_root))
    env = dict(environment, CARGO_TARGET_DIR=str(target), LLVM_PROFILE_FILE=str(raw / "%p-%m.profraw"))
    wrapper = Path(env["RUSTC_WORKSPACE_WRAPPER"])
    if digest(wrapper) != source_hashes["scripts/rustc-coverage.py"]:
        raise ValueError("native-free RGB instrumentation wrapper differs")
    command = rgb_command()
    (rgb / "command.json").write_text(json.dumps(command, indent=2) + "\n")
    (rgb / "inputs.json").write_text(json.dumps(source_hashes, indent=2) + "\n")
    result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True)
    (rgb / "stdout").write_text(result.stdout)
    (rgb / "stderr").write_text(result.stderr)
    if result.returncode:
        raise subprocess.CalledProcessError(result.returncode, command)
    require_rgb_tests(result.stdout)
    events = [json.loads(line) for line in result.stdout.splitlines() if line.startswith("{")]
    (rgb / "cargo-events.json").write_text(json.dumps(events, indent=2) + "\n")
    objects = rgb_artifacts(events, ROOT, target)
    binary = objects["gz-sim-rs"]["executable"]
    closure = subprocess.run([sys.executable, str(ROOT / "scripts/check-rapier-runtime.py"),
                              "--binary", binary, "--features", "rgb-camera"],
                             cwd=ROOT, env=env, capture_output=True, text=True)
    (rgb / "closure.stdout").write_text(closure.stdout)
    (rgb / "closure.stderr").write_text(closure.stderr)
    closure.check_returncode()
    validate_rgb_closure(ROOT, json.loads(closure.stdout), binary)
    profiles = sorted(raw.glob("*.profraw"))
    if not profiles:
        raise ValueError("no actual native-free RGB raw profiles")
    merged = rgb / "merged.profdata"
    subprocess.run([str(llvm / "llvm-profdata"), "merge", "-sparse", *map(str, profiles),
                    "-o", str(merged)], cwd=ROOT, check=True)
    export = [str(llvm / "llvm-cov"), "export", "--fatal-warnings", "-instr-profile=" + str(merged)]
    for event in objects.values():
        export += ["-object", event["executable"]]
    with (rgb / "coverage.json").open("w") as stream:
        subprocess.run(export, cwd=ROOT, stdout=stream, check=True)
    with (rgb / "coverage.lcov").open("w") as stream:
        subprocess.run([*export, "-format=lcov"], cwd=ROOT, stdout=stream, check=True)
    if (inputs() != source_hashes or digest(wrapper) != source_hashes["scripts/rustc-coverage.py"]
            or subprocess.check_output(["rustc", "-vV"], cwd=ROOT, text=True) != version):
        raise ValueError("native-free RGB source/input/compiler changed")
    copies = rgb / "objects"
    copies.mkdir()
    bindings = []
    for name, event in sorted(objects.items()):
        original = Path(event["executable"])
        saved = copies / name
        expected = digest(original)
        shutil.copy2(original, saved)
        if digest(saved) != expected or digest(original) != expected:
            raise ValueError("native-free RGB executable changed while preserving it")
        bindings.append({"original_path": str(original), "retained_path": str(saved.relative_to(out)),
                         "sha256": expected, "cargo_artifact": event})
    (rgb / "objects.json").write_text(json.dumps(bindings, indent=2) + "\n")
    if (target.is_symlink() or not target.is_dir() or target.parent != cache_root
            or not target.name.startswith("rapier-rgb-")):
        raise ValueError("refuse cleanup outside the private native-free RGB target")
    shutil.rmtree(target)
    return {"state": "passed", "features": ["rgb-camera"], "target_root": str(target),
            "private_target_removed": True, "objects_bound_after_execution": bindings}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/coverage/rapier-runtime")
    args = parser.parse_args()
    out = args.output.resolve()
    if not out.is_relative_to(ROOT / "artifacts"):
        raise ValueError("native-off output must stay under this source tree's artifacts")
    out.mkdir(parents=True, exist_ok=False)
    version = subprocess.check_output(["rustc", "-vV"], cwd=ROOT, text=True)
    verify.require_toolchain(version, os.environ)
    source_hashes = inputs()
    (out / "inputs.json").write_text(json.dumps(source_hashes, indent=2) + "\n")
    cache_root = ROOT / "target"
    cache_root.mkdir(exist_ok=True)
    target = Path(tempfile.mkdtemp(prefix="rapier-runtime-", dir=cache_root))
    raw = out / "raw"
    raw.mkdir()
    wrapper = out / "rustc-coverage.py"
    wrapper.write_bytes((ROOT / "scripts/rustc-coverage.py").read_bytes())
    wrapper.chmod(0o755)
    env = dict(os.environ, CARGO_TARGET_DIR=str(target), CARGO_INCREMENTAL="0",
               CARGO_BUILD_JOBS=os.environ.get("CARGO_BUILD_JOBS", "2"),
               CARGO_PROFILE_DEV_DEBUG="0", CARGO_PROFILE_TEST_DEBUG="0", RUSTFLAGS="",
               RUSTC_WORKSPACE_WRAPPER=str(wrapper), LLVM_PROFILE_FILE=str(raw / "%p-%m.profraw"))
    env.pop("CARGO_ENCODED_RUSTFLAGS", None)
    events, commands = [], []
    for package, (test, names) in TARGETS.items():
        command = ["cargo", "test", "--locked", "--offline", "-p", package,
                   "--no-default-features", "--test", test, "--message-format=json",
                   "--", "--nocapture", "--test-threads=1"]
        commands.append(command)
        (out / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
        result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True)
        (out / (test + ".stdout")).write_text(result.stdout)
        (out / (test + ".stderr")).write_text(result.stderr)
        if result.returncode:
            raise subprocess.CalledProcessError(result.returncode, command)
        totals = re.findall(r"test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;", result.stdout)
        if len(totals) != 1 or tuple(map(int, totals[0])) != (len(names), 0, 0):
            raise ValueError("missing/failed/skipped targeted no-native tests")
        if any(f"test {name} ... ok" not in result.stdout for name in names):
            raise ValueError("targeted no-native case missing")
        events += [json.loads(line) for line in result.stdout.splitlines() if line.startswith("{")]
    (out / "cargo-events.json").write_text(json.dumps(events, indent=2) + "\n")
    objects = checked_artifacts(events, target)
    binary = [path for path, item in objects.items()
              if item["target"]["name"] == "gz-sim-rs" and not item["profile"]["test"]]
    if len(binary) != 1:
        raise ValueError("expected actual non-test CLI executable")
    closure = subprocess.run([sys.executable, str(ROOT / "scripts/check-rapier-runtime.py"),
                              "--binary", binary[0]], cwd=ROOT, env=env, capture_output=True, text=True)
    (out / "closure.stdout").write_text(closure.stdout)
    (out / "closure.stderr").write_text(closure.stderr)
    closure.check_returncode()
    sysroot = Path(subprocess.check_output(["rustc", "--print", "sysroot"], cwd=ROOT, text=True).strip())
    host = next(line.split(": ", 1)[1] for line in version.splitlines() if line.startswith("host: "))
    llvm = sysroot / "lib/rustlib" / host / "bin"
    profiles = sorted(raw.glob("*.profraw"))
    if not profiles:
        raise ValueError("no actual no-native raw profiles")
    merged = out / "merged.profdata"
    subprocess.run([str(llvm / "llvm-profdata"), "merge", "-sparse", *map(str, profiles),
                    "-o", str(merged)], cwd=ROOT, check=True)
    export = [str(llvm / "llvm-cov"), "export", "--fatal-warnings", "-instr-profile=" + str(merged)]
    for path in objects:
        export += ["-object", path]
    with (out / "coverage.json").open("w") as stream:
        subprocess.run(export, cwd=ROOT, stdout=stream, check=True)
    with (out / "coverage.lcov").open("w") as stream:
        subprocess.run([*export, "-format=lcov"], cwd=ROOT, stdout=stream, check=True)
    proof = disabled_region(json.loads((out / "coverage.json").read_text()),
                            (out / "coverage.lcov").read_text(), ROOT, (ROOT / SCENE).read_text())
    if inputs() != source_hashes or digest(wrapper) != source_hashes["scripts/rustc-coverage.py"]:
        raise ValueError("no-native sources/inputs changed during qualification")
    if subprocess.check_output(["rustc", "-vV"], cwd=ROOT, text=True) != version:
        raise ValueError("compiler changed during no-native qualification")
    copies = out / "objects"
    copies.mkdir()
    bindings = []
    for number, (path, item) in enumerate(sorted(objects.items())):
        saved = copies / f"{number:03d}-{Path(path).name}"
        expected = digest(path)
        shutil.copy2(path, saved)
        if digest(saved) != expected or digest(path) != expected:
            raise ValueError("native-off executable changed while preserving it")
        bindings.append({"original_path": path, "retained_path": str(saved.relative_to(out)),
                         "sha256": expected, "cargo_artifact": item})
    (out / "objects.json").write_text(json.dumps(bindings, indent=2) + "\n")
    # Only this invocation's fresh compilation cache is disposable. Copies,
    # sources, raw counters and reports live outside it and remain available.
    if (target.is_symlink() or not target.is_dir() or target.parent != cache_root
            or not target.name.startswith("rapier-runtime-")):
        raise ValueError("refuse cleanup outside the private native-off target")
    shutil.rmtree(target)
    # The original feature-off objects/raw/export are already preserved. A
    # separate target and raw stream prevent feature unification or overwrites.
    rgb = qualify_rgb(out, env, version, source_hashes, llvm)
    report = {"state": "passed", "scope": "Additional native-off behavior/disabled-arm evidence only",
              "source_root": str(ROOT),
              "target_root": str(target),
              "rustc": version, "sources_and_inputs": source_hashes, "commands": commands,
              "disabled_backend": proof, "objects_bound_after_execution": bindings,
              "raw_profiles": {str(p.relative_to(out)): digest(p) for p in profiles},
              "coverage_json_sha256": digest(out / "coverage.json"),
              "coverage_lcov_sha256": digest(out / "coverage.lcov"),
              "closure_sha256": digest(out / "closure.stdout"),
              "private_target_removed": True,
              "rgb_camera": rgb,
              "limits": "Unfiltered report retained. No whole-native-off >95% claim or report union; unchanged full headless/renderer gates remain mandatory."}
    report["artifacts_sha256"] = {str(path.relative_to(out)): digest(path)
                                 for path in sorted(out.rglob("*")) if path.is_file()}
    validate_evidence(ROOT, out, report)
    (out / "result.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
