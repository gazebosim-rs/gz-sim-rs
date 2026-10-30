#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Build a fresh DART-enabled native layer; runtime qualification is separate."""
import argparse
from email.parser import BytesParser
import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

spec = importlib.util.spec_from_file_location("reference_build", Path(__file__).with_name("build-reference.py"))
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)
ROOT = Path(__file__).resolve().parents[1]
VERSION = "6.13.2+ds1-1~osrf2~noble"
FIELDS = ("Package", "Version", "Architecture", "Source", "Filename", "Size", "SHA256")
PACKAGES = {"libdart-core+collisions+odelcpsolver6.13", "libdart-utils6.13", "libdart-utils-urdf6.13",
            "libdart6.13-dev", "libdart6.13-external-odelcpsolver-dev", "libdart6.13-external-convhull-3d-dev",
            "libdart6.13-collision-bullet-dev", "libdart6.13-collision-ode-dev", "libdart6.13-utils-dev",
            "libdart6.13-utils-urdf-dev"}


def validate_lock(lock):
    if (lock["schema_version"] != 1 or lock["builds"] != ["gz-physics", "gz-sim"]
            or lock["prefix"] != "/reference/dart-install" or lock["build_root"] != "/reference/dart-builds"
            or lock["DART_DIR"] != "/usr/share/dart/cmake"
            or lock["osrf_url"] != "https://packages.osrfoundation.org/gazebo/ubuntu-stable/"
            or not re.fullmatch(r"sha256:[0-9a-f]{64}", lock["base_image_id"])):
        raise ValueError("DART profile domain differs")
    if len(lock["packages"]) != len(PACKAGES) or {row["Package"] for row in lock["packages"]} != PACKAGES:
        raise ValueError("DART package selection differs")


def verify_recipe(recipe, lock):
    expected = ["ADD --checksum=sha256:" + row["SHA256"] + " " + lock["osrf_url"] + row["Filename"]
                + " /tmp/dart-debs/" + Path(row["Filename"]).name for row in lock["packages"]]
    if ([line for line in recipe.split("\n") if line.startswith("FROM ")] != ["FROM " + lock["base_image_tag"]]
            or [line for line in recipe.split("\n") if line.startswith("ADD ")] != expected):
        raise ValueError("DART recipe base/packages differ")


def records(raw):
    # Signed Debian control paragraphs use RFC822 field syntax. Only the
    # selected single-valued identity/hash fields participate in this contract.
    result = []
    for paragraph in raw.split(b"\n\n"):
        if not paragraph.strip():
            continue
        message = BytesParser().parsebytes(paragraph)
        if message.defects or any(len(message.get_all(key, [])) != 1 for key in message.keys()):
            raise ValueError("ambiguous signed package paragraph")
        result.append(message)
    return result


def verify_metadata(root, lock):
    validate_lock(lock)
    expected_names = {"InRelease", "archive-keyring.gpg", "Packages.gz", "Sources.gz"}
    paths = {Path(name).name: root / name for name in lock["metadata_sha256"]}
    if set(paths) != expected_names or any(reference.digest(root / name) != value
            for name, value in lock["metadata_sha256"].items()):
        raise ValueError("DART metadata bytes differ")
    with tempfile.TemporaryDirectory() as directory:
        payload = Path(directory) / "Release"
        result = subprocess.run(["gpgv", "--status-fd", "1", "--keyring", str(paths["archive-keyring.gpg"]),
                                 "--output", str(payload), str(paths["InRelease"])], capture_output=True, check=True)
        signatures = [line.split()[2] for line in result.stdout.decode().split("\n")
                      if line.startswith("[GNUPG:] VALIDSIG ")]
        if signatures != [lock["signing_fingerprint"]]:
            raise ValueError("DART archive signer differs")
        release = payload.read_text()
    payloads = {}
    for filename, index in [("Packages.gz", "main/binary-amd64/Packages"), ("Sources.gz", "main/source/Sources")]:
        raw = gzip.decompress(paths[filename].read_bytes())
        digest = hashlib.sha256(raw).hexdigest()
        if len(re.findall(r"^ " + digest + r"\s+" + str(len(raw)) + r"\s+" + re.escape(index) + r"$", release, re.M)) != 1:
            raise ValueError("signed DART index binding differs")
        payloads[filename] = records(raw)
    packages = lock["packages"]
    if len(packages) != 10 or len({row["Package"] for row in packages}) != 10:
        raise ValueError("DART package selection differs")
    for row in packages:
        if (row["Version"] != VERSION or row["Architecture"] != "amd64" or row["Source"] != "dart"
                or not re.fullmatch(r"pool/main/d/dart/[A-Za-z0-9+_.~%-]+\.deb", row["Filename"])
                or not re.fullmatch(r"[0-9a-f]{64}", row["SHA256"])):
            raise ValueError("DART package identity differs")
        found = [value for value in payloads["Packages.gz"]
                 if value.get("Package") == row["Package"] and value.get("Version") == VERSION
                 and value.get("Architecture") == "amd64"]
        if len(found) != 1 or {key: found[0][key] for key in FIELDS} != row:
            raise ValueError("DART package differs from signed index")
    source = lock["source_package"]
    found = [value for value in payloads["Sources.gz"]
             if value.get("Package") == "dart" and value.get("Version") == VERSION]
    if len(found) != 1 or any(found[0].get(key, "").strip() != value.strip() for key, value in source.items()):
        raise ValueError("DART source-package identity differs")
    return {"signing_fingerprint": signatures[0], "packages": len(packages), "source": source}


def verify_debs(directory, lock):
    result = {}
    for row in lock["packages"]:
        path = directory / Path(row["Filename"]).name
        if (path.is_symlink() or not path.is_file() or path.stat().st_size != int(row["Size"])
                or reference.digest(path) != row["SHA256"]):
            raise ValueError("DART package payload differs")
        fields = subprocess.check_output(["dpkg-deb", "--field", str(path), "Package", "Version", "Architecture", "Source"], text=True)
        actual = records(fields.encode())[0]
        if any(actual.get(key) != row[key] for key in ["Package", "Version", "Architecture", "Source"]):
            raise ValueError("DART payload control identity differs")
        result[path.name] = reference.digest(path)
    return result


def build_environment(prefix, base):
    # Sanitize before any subprocess: inherited loader hooks must not reach git.
    env = {key: value for key, value in os.environ.items() if not key.startswith(("GIT_", "ROS_", "AMENT_", "COLCON_"))}
    for key in list(env):
        if key in {
                "LD_PRELOAD", "LD_AUDIT", "CMAKE_FIND_ROOT_PATH", "CMAKE_IGNORE_PREFIX_PATH",
                "PYTHONPATH", "PYTHONHOME", "GZ_SIM_SYSTEM_PLUGIN_PATH", "GZ_PHYSICS_ENGINE_PATH",
                "CC", "CXX", "CFLAGS", "CXXFLAGS", "CPPFLAGS", "LDFLAGS", "CMAKE_TOOLCHAIN_FILE",
                "CPATH", "CPLUS_INCLUDE_PATH", "C_INCLUDE_PATH", "LIBRARY_PATH", "COMPILER_PATH",
                "GCC_EXEC_PREFIX", "PKG_CONFIG_SYSROOT_DIR"}:
            env.pop(key)
    env.update(GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL="/dev/null", GIT_TERMINAL_PROMPT="0",
               GIT_OPTIONAL_LOCKS="0", PYTHONDONTWRITEBYTECODE="1", DEBIAN_FRONTEND="noninteractive")
    env.update(PATH=os.pathsep.join(map(str, [prefix / "bin", base / "bin", Path("/usr/bin"), Path("/bin")])),
               CMAKE_PREFIX_PATH=os.pathsep.join(map(str, [prefix, base])),
               PKG_CONFIG_PATH=os.pathsep.join(map(str, [prefix / "lib/pkgconfig", base / "lib/pkgconfig"])),
               PKG_CONFIG_LIBDIR="/usr/lib/x86_64-linux-gnu/pkgconfig:/usr/share/pkgconfig",
               LD_LIBRARY_PATH=os.pathsep.join(map(str, [prefix / "lib", base / "lib", Path("/usr/lib/x86_64-linux-gnu")])) )
    return env


def verify_dart_files(root, lock):
    for name, value in lock["dart_files_sha256"].items():
        path = root / name
        if path.is_symlink() or not path.is_file() or reference.digest(path) != value:
            raise ValueError("installed DART header/library differs from package")


def check_cache(cache, source, prefix, physics):
    values = {}
    for line in cache.split("\n"):
        if "=" not in line or line.startswith(("#", "//")):
            continue
        key, value = line.split("=", 1)
        if key in values:
            raise ValueError("duplicate raw DART cache key: " + key)
        values[key] = value
    def value(key):
        found = [v for k, v in values.items() if k.split(":", 1)[0] == key]
        if len(found) != 1:
            raise ValueError("missing or ambiguous DART build setting: " + key)
        return found[0]
    expected = {"CMAKE_HOME_DIRECTORY": str(source), "CMAKE_INSTALL_PREFIX": str(prefix),
                "CMAKE_BUILD_TYPE": "Release", "BUILD_TESTING": "ON",
                "CMAKE_C_COMPILER": "/usr/bin/gcc", "CMAKE_CXX_COMPILER": "/usr/bin/g++"}
    if physics:
        expected.update(SKIP_dartsim="OFF", SKIP_bullet="ON", **{"SKIP_bullet-featherstone": "ON"},
                        SKIP_mujoco="ON", DART_DIR="/usr/share/dart/cmake", DART_HAS_CONTACT_SURFACE_HEADER="1")
    else:
        expected.update(ENABLE_GUI="OFF", SKIP_PYBIND11="ON")
    if any(value(key) != item for key, item in expected.items()) or "/opt/ros" in cache:
        raise ValueError("DART build settings/search paths differ")
    if physics and any(not path.startswith("/usr/include") for path in value("DART_INCLUDE_DIRS").split(";")):
        raise ValueError("DART headers are not from the selected system packages")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--debs", type=Path, required=True)
    parser.add_argument("--jobs", type=int, choices=range(1, 5), default=2)
    args = parser.parse_args()
    if ROOT != Path("/reference") or Path("/opt/ros").exists():
        raise ValueError("build requires the isolated qualified SDK without ROS")
    lock_path = ROOT / "reference/headless-dart.lock.json"
    lock = json.loads(lock_path.read_text())
    validate_lock(lock)
    verify_recipe((ROOT / "reference/Dockerfile.dart").read_text(), lock)
    base_manifest = ROOT / "environment.json"
    if reference.digest(base_manifest) != lock["base_environment_sha256"]:
        raise ValueError("DART base SDK manifest differs")
    baseline = ROOT / "planning/baseline.json"
    if reference.digest(baseline) != lock["baseline_sha256"]:
        raise ValueError("DART source baseline differs")
    metadata = verify_metadata(ROOT, lock)
    payloads = verify_debs(args.debs, lock)
    protected = {str(path): reference.digest(path) for path in [lock_path, baseline, base_manifest, Path(__file__),
        ROOT / "reference/build-reference.py", ROOT / "reference/Dockerfile.dart", ROOT / "reference/Dockerfile.dart.dockerignore"]}
    protected.update({str(ROOT / name): value for name, value in lock["metadata_sha256"].items()})
    base = ROOT / "install"
    original = json.loads(base_manifest.read_bytes())
    if reference.files_under(base) != original["installed_files"]:
        raise ValueError("DART base SDK installed bytes differ")
    prefix, builds = Path(lock["prefix"]), Path(lock["build_root"])
    if prefix.exists() or builds.exists():
        raise ValueError("DART build/install directories must be fresh")
    env = build_environment(prefix, base)
    subprocess.run(["sh", str(ROOT / "reference/apt-sources.sh"), "/etc/apt", lock["ubuntu_snapshot"]], env=env, check=True)
    subprocess.run(["apt-get", "-o", "APT::Update::Error-Mode=any", "update"], env=env, check=True)
    subprocess.run(["apt-get", "install", "-y", "--no-install-recommends",
                    *[str(args.debs / name) for name in payloads]], env=env, check=True)
    verify_dart_files(Path("/"), lock)
    if subprocess.check_output(["pkg-config", "--modversion", "dart"], env=env, text=True).strip() != "6.13.2":
        raise ValueError("installed DART version differs")
    pins = {row["name"]: row for row in json.loads(baseline.read_bytes())["repositories"]}
    protected.update({str(path): reference.digest(path) for path in [Path("/usr/bin/gcc").resolve(), Path("/usr/bin/g++").resolve()]})
    records_out = []
    for name in lock["builds"]:
        source, build = ROOT / "sources" / name, builds / name
        pin = pins[name]
        reference.verify_source(source, {"sha": pin["commit"]}, env)
        flags = {"CMAKE_BUILD_TYPE": "Release", "BUILD_TESTING": "ON", "CMAKE_INSTALL_LIBDIR": "lib",
                 "CMAKE_C_COMPILER": "/usr/bin/gcc", "CMAKE_CXX_COMPILER": "/usr/bin/g++",
                 "CMAKE_FIND_USE_PACKAGE_REGISTRY": "OFF", "CMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY": "OFF"}
        if name == "gz-physics":
            flags.update(SKIP_dartsim="OFF", SKIP_bullet="ON", **{"SKIP_bullet-featherstone": "ON"},
                         SKIP_mujoco="ON", DART_DIR="/usr/share/dart/cmake")
        else:
            flags.update(ENABLE_GUI="OFF", SKIP_PYBIND11="ON")
        build.mkdir(parents=True)
        log = build / "build.log"
        reference.run(["cmake", "-S", source, "-B", build, "-DCMAKE_INSTALL_PREFIX=" + str(prefix),
                       *["-D" + key + "=" + value for key, value in flags.items()]], log, env)
        cache = build / "CMakeCache.txt"
        check_cache(cache.read_text(), source, prefix, name == "gz-physics")
        reference.run(["cmake", "--build", build, "--parallel", str(args.jobs)], log, env)
        reference.run(["cmake", "--install", build], log, env)
        reference.verify_source(source, {"sha": pin["commit"]}, env)
        listing = reference.retained_listing(["ctest", "--test-dir", str(build), "--show-only=json-v1"], build / "ctest.json", env)
        records_out.append({"name": name, "source_revision": pin["commit"], "cache_sha256": reference.digest(cache),
                            "build_log_sha256": reference.digest(log), "registrations": len(listing["tests"])})
    if any(reference.digest(Path(name)) != value for name, value in protected.items()) or verify_debs(args.debs, lock) != payloads:
        raise ValueError("DART profile inputs changed during build")
    verify_dart_files(Path("/"), lock)
    if reference.files_under(base) != original["installed_files"]:
        raise ValueError("base SDK changed during DART build")
    installed = reference.files_under(prefix)
    for required in ["libgz-physics.so", "libgz-physics-dartsim-plugin.so", "libgz-sim.so", "libgz-sim-physics-system.so"]:
        if not any(Path(name).name.startswith(required) for name in installed):
            raise ValueError("missing required new DART profile module")
    manifest = {"scope": "Fresh native DART-enabled physics/simulator layer over the qualified TPE SDK; no runtime passes, source-built DART, full REF01 or P0 completion.",
                "state": "built_pending_independent_runtime_qualification", "base_environment_sha256": reference.digest(base_manifest),
                "inputs_sha256": protected, "osrf_metadata": metadata, "package_payload_sha256": payloads,
                "packages": reference.output(["dpkg-query", "-W", "-f=${binary:Package}\\t${Version}\\t${Architecture}\\n"], env),
                "repositories": records_out, "installed_files": installed}
    (ROOT / "dart-environment.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()
