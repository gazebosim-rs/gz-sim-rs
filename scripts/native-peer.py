#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Compile the required C++ client against the pinned native transport reference."""
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
from native_reference import LEGACY, SDK, builder, compiler_identity, validate_reference

root = Path(__file__).resolve().parents[1]
prefix = Path(os.environ.get('GZ_REFERENCE_PREFIX', root / 'artifacts/native-reference/install')).resolve()
metadata_path = prefix.parent / 'environment.json'
metadata_bytes = metadata_path.read_bytes()
metadata = json.loads(metadata_bytes)
lock = json.loads((root / 'planning/transport-reference-lock.json').read_text())
profile = os.environ.get('GZ_REFERENCE_PROFILE', LEGACY)
sdk = json.loads((root / 'planning/reference/headless-sdk-environment.json').read_text()) if profile == SDK else None
reference = validate_reference(prefix, metadata, lock, sdk, profile)
env = builder.build_environment(prefix) if profile == SDK else dict(os.environ, PKG_CONFIG_PATH=str(prefix / 'lib/pkgconfig'), LD_LIBRARY_PATH=str(prefix / 'lib'))
compiler, compiler_record = compiler_identity(env, reference.get('expected_compiler_sha256'))
actual = subprocess.check_output(['pkg-config', '--variable=prefix', 'gz-transport'], env=env, text=True).strip()
assert Path(actual).resolve() == prefix, 'pkg-config selected a different native installation'
flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gz-transport'], env=env, text=True))
source = root / 'crates/gz-sim-server/tests/clock_client.cc'
out = root / 'artifacts/native-peer'
out.mkdir(parents=True, exist_ok=True)
source_bytes = source.read_bytes()
captured_source = out / 'clock-client-source.cc'
captured_source.write_bytes(source_bytes)
binary = out / 'clock-client'
subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', str(captured_source), *flags, '-Wl,-rpath,' + str(prefix / 'lib'), '-o', str(binary)], env=env, check=True)
if (source.read_bytes() != source_bytes or captured_source.read_bytes() != source_bytes
        or metadata_path.read_bytes() != metadata_bytes):
    raise ValueError('native client input changed during compilation')
if validate_reference(prefix, metadata, lock, sdk, profile) != reference:
    raise ValueError('native reference changed during compilation')
compiler_identity(env, compiler_record['sha256'])
linkage = subprocess.check_output(['ldd', str(binary)], env=env, text=True)
assert 'not found' not in linkage, linkage
for line in linkage.splitlines():
    if line.strip().startswith('libgz-'):
        assert Path(line.split('=>', 1)[1].split()[0]).resolve().is_relative_to(prefix), line
(out / 'provenance.json').write_text(json.dumps({**reference, 'compiler': compiler_record, 'metadata_sha256': hashlib.sha256(metadata_bytes).hexdigest(), 'peer_source_sha256': hashlib.sha256(source_bytes).hexdigest(), 'peer_binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(), 'libraries_sha256': reference['libraries_sha256'], 'linkage': linkage}, indent=2) + '\n')
print(binary)
