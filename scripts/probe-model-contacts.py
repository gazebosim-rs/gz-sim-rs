#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Qualify native model contact defaults against the baseline DART plugin."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def output(command, **kwargs):
    return subprocess.check_output(command, text=True, **kwargs).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['physics-source', 'physics-build', 'dependency-prefix', 'work-dir', 'output']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    source, build, prefix = (path.resolve() for path in
                             [args.physics_source, args.physics_build, args.dependency_prefix])
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    baseline = json.loads((ROOT / 'planning/baseline.json').read_text())
    revision = next(item['commit'] for item in baseline['repositories'] if item['name'] == 'gz-physics')
    if output(['git', '-C', str(source), 'rev-parse', 'HEAD']) != revision:
        raise ValueError('native source is not at the baseline revision')
    sources = {}
    names = output(['git', '-C', str(source), 'ls-files', 'dartsim/src']).splitlines()
    if not names:
        raise ValueError('native source inventory is empty')
    for name in names:
        original = subprocess.check_output(['git', '-C', str(source), 'show', revision + ':' + name])
        if original != (source / name).read_bytes():
            raise ValueError('native source changed: ' + name)
        sources[name] = digest(source / name)
    cache = build / 'CMakeCache.txt'
    settings = dict(line.split('=', 1) for line in cache.read_text().splitlines()
                    if '=' in line and not line.startswith(('#', '//')))
    if Path(settings['CMAKE_HOME_DIRECTORY:INTERNAL']).resolve() != source:
        raise ValueError('native build belongs to another source')
    if Path(settings['CMAKE_INSTALL_PREFIX:PATH']).resolve() != prefix:
        raise ValueError('native build has a different dependency prefix')
    build_command = ['cmake', '--build', str(build), '--target', 'gz-physics-dartsim-plugin', '--parallel', '2']
    subprocess.run(build_command, check=True, stdout=subprocess.DEVNULL)
    env = os.environ.copy()
    env['PKG_CONFIG_PATH'] = str(prefix / 'lib/pkgconfig')
    packages = ['gz-physics', 'gz-physics-sdf', 'gz-plugin-loader', 'sdformat', 'gz-math-eigen3']
    flags = shlex.split(output(['pkg-config', '--cflags', '--libs', *packages], env=env))
    compiler = shlex.split(env.get('CXX', 'c++'))
    helper = ROOT / 'crates/gz-sim-scene/tests/native_model_contacts.cc'
    binary, maps = work / 'native-model-contacts', work / 'loaded.maps'
    command = [*compiler, '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', str(helper), *flags,
               '-Wl,-rpath,' + str(prefix / 'lib'), '-o', str(binary)]
    subprocess.run(command, env=env, check=True)
    plugin = (build / 'lib/libgz-physics-dartsim-plugin.so').resolve(strict=True)
    env['LD_LIBRARY_PATH'] = str(build / 'lib') + ':' + str(prefix / 'lib')
    run = subprocess.run([str(binary), str(plugin), str(maps)], env=env, text=True,
                         capture_output=True, timeout=30, check=True)
    (work / 'stdout.log').write_text(run.stdout)
    (work / 'stderr.log').write_text(run.stderr)
    rows = [line.split() for line in run.stdout.splitlines()]
    if run.stderr or len(rows) != 6 or any(len(row) < 4 for row in rows):
        raise ValueError('native probe returned unexpected output')
    observations = []
    for mode, row in enumerate(rows):
        index, a, b, contacts = int(row[0]), float(row[1]), float(row[2]), int(row[3])
        if index != mode or contacts < 0 or not all(map(math.isfinite, [a, b])):
            raise ValueError('native probe returned invalid observations')
        pairs = [pair.split(':') for pair in row[4:]]
        expected = ['l0', 'l2'] if mode >= 4 else ['l0', 'l1']
        if len(pairs) != contacts or any(pair != expected for pair in pairs):
            raise ValueError('native contacts have unexpected link owners')
        if mode in [0, 2] and (a != 0.0 or b != 0.75):
            raise ValueError('native default same-model contact exclusion changed')
        if mode in [4, 5] and (a != 0.0 or b != 0.0):
            raise ValueError('native articulated nonadjacent contact exclusion changed')
        if mode in [1, 3] and not (a < -1e-7 and b > 0.7500001):
            raise ValueError('native positive contact controls did not respond')
        if (mode in [0, 2, 4] and contacts != 0) or (mode in [1, 3, 5] and contacts == 0):
            raise ValueError('native contact counts contradict the exclusion controls')
        observations.append({'mode': mode, 'first_x': a, 'second_link': 'l2' if mode >= 4 else 'l1',
                             'second_x': b, 'contacts': contacts, 'contact_pairs': pairs})
    libraries = {}
    for line in maps.read_text().splitlines():
        parts = line.split(maxsplit=5)
        if len(parts) != 6 or not parts[5].startswith('/'):
            continue
        path = Path(parts[5]).resolve()
        if any(token in path.name for token in ['libgz-physics', 'libdart', 'libsdformat', 'libode', 'libgz-math']):
            libraries[str(path)] = digest(path)
    if str(plugin) not in libraries or not any(Path(path).name.startswith('libdart.so') for path in libraries):
        raise ValueError('expected native plugin and DART runtime were not loaded')
    result = {'scope': 'Native default model self-contact exclusion, distinct-model contact, link-only flag, model flag and three-link articulated nonadjacent counts with an enabled-contact control. Contact presence only; no numerical or performance parity claim.',
              'revision': revision, 'sources': sources, 'cmake_cache_sha256': digest(cache),
              'build_command': build_command, 'compile_command': command,
              'compiler': output([*compiler, '--version']).splitlines()[0],
              'package_versions': {name: output(['pkg-config', '--modversion', name], env=env) for name in packages},
              'helper_sha256': digest(helper), 'binary_sha256': digest(binary),
              'libraries': dict(sorted(libraries.items())), 'observations': observations}
    args.output.write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    main()
