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

root = Path(__file__).resolve().parents[1]
prefix = Path(os.environ.get('GZ_REFERENCE_PREFIX', root / 'artifacts/native-reference/install')).resolve()
metadata = json.loads((prefix.parent / 'environment.json').read_text())
lock = json.loads((root / 'planning/transport-reference-lock.json').read_text())
assert metadata['source_lock'] == lock, 'native reference differs from pinned source/configuration'
for name, expected in metadata['libraries_sha256'].items():
    assert hashlib.sha256((prefix / name).read_bytes()).hexdigest() == expected, name
env = dict(os.environ, PKG_CONFIG_PATH=str(prefix / 'lib/pkgconfig'), LD_LIBRARY_PATH=str(prefix / 'lib'))
actual = subprocess.check_output(['pkg-config', '--variable=prefix', 'gz-transport'], env=env, text=True).strip()
assert Path(actual).resolve() == prefix, 'pkg-config selected a different native installation'
flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'gz-transport'], env=env, text=True))
source = root / 'crates/gz-sim-server/tests/clock_client.cc'
out = root / 'artifacts/native-peer'
out.mkdir(parents=True, exist_ok=True)
binary = out / 'clock-client'
subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', str(source), *flags, '-Wl,-rpath,' + str(prefix / 'lib'), '-o', str(binary)], env=env, check=True)
linkage = subprocess.check_output(['ldd', str(binary)], env=env, text=True)
assert 'not found' not in linkage, linkage
for line in linkage.splitlines():
    if line.strip().startswith('libgz-'):
        assert Path(line.split('=>', 1)[1].split()[0]).resolve().is_relative_to(prefix), line
(out / 'provenance.json').write_text(json.dumps({'source_lock': lock, 'peer_source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(), 'peer_binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(), 'libraries_sha256': metadata['libraries_sha256'], 'linkage': linkage}, indent=2) + '\n')
print(binary)
