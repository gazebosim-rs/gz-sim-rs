# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Compile the retained conformance fixtures against the selected Zenoh reference."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
PROFILE = 'gazebo-zenoh-client-v1'
CLIENTS = {
    'clock': 'crates/gz-sim-server/tests/clock_client.cc',
    'rate': 'crates/gz-sim-server/tests/rate_client.cc',
    'force': 'crates/gz-sim-server/tests/force_client.cc',
    'camera': 'crates/gz-render-worker/tests/fixtures/camera_subscriber.cc',
}
spec = importlib.util.spec_from_file_location('zenoh_bootstrap', ROOT / 'scripts/bootstrap-zenoh-native.py')
bootstrap = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bootstrap)


def compile_client(client, reference):
    if client not in CLIENTS:
        raise ValueError('unknown conformance client')
    metadata = bootstrap.validate_reference(reference)
    prefix = reference / 'install'
    vendor = reference / 'vendor' / metadata['lock']['vendor']['prefix']
    env = bootstrap.build_environment(prefix, vendor)
    source = ROOT / CLIENTS[client]
    if not source.resolve(strict=True).is_relative_to(ROOT):
        raise ValueError('client source escapes workspace')
    source_bytes = source.read_bytes()
    metadata_bytes = (reference / 'environment.json').read_bytes()
    helper_bytes = Path(__file__).read_bytes()
    pkg = metadata['tools']['pkg-config']['path']
    selected = subprocess.check_output([pkg, '--variable=prefix', 'gz-transport'], env=env, text=True).strip()
    if Path(selected).resolve() != prefix.resolve():
        raise ValueError('pkg-config selected a different native reference')
    flags = shlex.split(subprocess.check_output([pkg, '--cflags', '--libs', 'gz-transport'], env=env, text=True))
    parent = Path(os.environ.get('GZ_ZENOH_PEER_EVIDENCE_DIR', ROOT / 'artifacts/native-zenoh-peer')).absolute()
    parent.mkdir(parents=True, exist_ok=True)
    if parent.is_symlink():
        raise ValueError('native peer output must not be a symlink')
    output = Path(tempfile.mkdtemp(prefix=client + '-', dir=parent))
    captured = output / 'source.cc'
    captured.write_bytes(source_bytes)
    binary = output / (client + '-client')
    command = [metadata['tools']['c++']['path'], '-std=c++17', '-Wall', '-Wextra', '-Werror',
               str(captured), *flags, '-Wl,-rpath,' + str(prefix / 'lib'),
               '-Wl,-rpath,' + str(vendor / 'lib'), '-o', str(binary)]
    compiled = subprocess.run(command, env=env, capture_output=True, text=True)
    (output / 'compiler.stdout').write_text(compiled.stdout)
    (output / 'compiler.stderr').write_text(compiled.stderr)
    compiled.check_returncode()
    linkage = subprocess.check_output(['/usr/bin/ldd', str(binary)], env=env, text=True)
    libraries = bootstrap.validate_linkage(linkage, prefix, vendor)
    if (source.read_bytes() != source_bytes or captured.read_bytes() != source_bytes
            or (reference / 'environment.json').read_bytes() != metadata_bytes
            or Path(__file__).read_bytes() != helper_bytes
            or bootstrap.validate_reference(reference) != metadata):
        raise ValueError('native client input changed during compilation')
    provenance = output / 'provenance.json'
    provenance.write_text(json.dumps({
        'profile': PROFILE, 'reference_root': str(reference), 'client': client,
        'reference_sha256': hashlib.sha256(metadata_bytes).hexdigest(),
        'helper_sha256': hashlib.sha256(helper_bytes).hexdigest(),
        'source': CLIENTS[client], 'source_sha256': hashlib.sha256(source_bytes).hexdigest(),
        'binary_sha256': bootstrap.digest(binary), 'command': command,
        'libraries': libraries, 'linkage': linkage,
    }, indent=2) + '\n')
    return {'profile': PROFILE, 'binary': str(binary), 'prefix': str(prefix),
            'vendor_prefix': str(vendor), 'provenance': str(provenance)}
