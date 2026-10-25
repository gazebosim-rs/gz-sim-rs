#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Record installed native Fuel cache selection and SDF directory handoff."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dependency-prefix', type=Path, required=True)
    parser.add_argument('--work-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    cache = work / 'cache-root'
    source = ROOT / 'crates/gz-assets/tests/native_fuel_cache.cc'
    binary = work / 'native-cache'
    env = os.environ.copy()
    env['PKG_CONFIG_PATH'] = str(args.dependency_prefix.resolve() / 'lib/pkgconfig')
    env['LD_LIBRARY_PATH'] = str(args.dependency_prefix.resolve() / 'lib')
    env.pop('GZ_FUEL_CACHE_PATH', None)
    flags = shlex.split(subprocess.check_output(
        ['pkg-config', '--cflags', '--libs', 'gz-fuel_tools', 'sdformat', 'gz-common'], env=env, text=True))
    command = ['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', str(source), *flags, '-o', str(binary)]
    subprocess.run(command, env=env, check=True)
    linkage = subprocess.check_output(['ldd', str(binary)], env=env, text=True)
    for version in [1, 2]:
        directory = cache / 'fuel.gazebosim.org/alice/models/robot' / str(version)
        directory.mkdir(parents=True, exist_ok=True)
        (directory / 'model.config').write_text("<model><sdf version='1.12'>model.sdf</sdf></model>")
        (directory / 'model.sdf').write_text(str(version))
    rows = []
    for suffix, parsed, selected in [('/1', True, 1), ('/2', True, 2), ('/tip', True, 2),
                                      ('', True, 2), ('/3', True, None), ('/invalid', False, None)]:
        uri = 'https://fuel.gazebosim.org/1.0/ALICE/models/ROBOT' + suffix
        result = subprocess.run([str(binary), str(cache), uri], env=env, capture_output=True, text=True, check=True)
        native = json.loads(result.stdout)
        expected_path = cache / 'fuel.gazebosim.org/alice/models/robot' / str(selected)
        expected = {'parsed': parsed, 'cached': selected is not None,
                    'owner': 'alice' if parsed else '', 'name': 'robot' if parsed else '',
                    'version': int(suffix[1:]) if suffix in ['/1', '/2', '/3'] else 0,
                    'path': str(expected_path) if selected else '',
                    'file': str(expected_path / 'model.sdf') if selected else '', 'errors': 0}
        if native != expected:
            raise ValueError(f'{uri}: {native} != {expected}')
        rows.append({'uri': uri, 'native': native, 'stderr': result.stderr})
    report = {'scope': 'Installed-native SDK cache-only controls, not a rebuilt full simulator/Fuel baseline; no download path invoked.',
              'command': command, 'source_sha256': digest(source), 'binary_sha256': digest(binary),
              'libraries': {token: digest(Path(token)) for token in linkage.split()
                            if token.startswith('/') and Path(token).is_file()}, 'rows': rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(f'{len(rows)} native cache controls passed')


if __name__ == '__main__':
    main()
