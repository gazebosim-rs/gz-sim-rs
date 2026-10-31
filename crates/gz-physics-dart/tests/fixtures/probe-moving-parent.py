#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Replay DART generalized torque through the pinned C++ Gazebo physics plugin."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[4]
FIXTURES = ROOT / 'crates/gz-physics-dart/tests/fixtures'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def header_identity(plugin_root, runtime_root):
    """Record build/runtime header identity without implying a fresh SDK build."""
    trees = []
    for root in [plugin_root, runtime_root]:
        assert (root / 'dynamics/Skeleton.hpp').is_file(), str(root)
        files = {str(path.relative_to(root)): digest(path) for path in sorted(root.rglob('*'))
                 if path.is_file() and path.suffix in {'.h', '.hh', '.hpp'}}
        trees.append(files)
    differences = {name: {'plugin': trees[0].get(name), 'runtime': trees[1].get(name)}
                   for name in sorted(trees[0].keys() | trees[1].keys())
                   if trees[0].get(name) != trees[1].get(name)}
    assert set(differences) <= {'config.hpp', 'external/lodepng/lodepng.h'}, differences
    if 'external/lodepng/lodepng.h' in differences:
        assert differences['external/lodepng/lodepng.h'] == {
            'plugin': 'a3da75f219111a891b13bf23e9d15c226a3a404b5e51c904c7be35d28d320a0e',
            'runtime': None}, differences
    macros, normalized = [], []
    pattern = re.compile(r'^#define (HAVE_NLOPT|HAVE_IPOPT|DART_ROOT_PATH|DART_DATA_PATH|DART_DATA_LOCAL_PATH|DART_DATA_GLOBAL_PATH) (.+)$', re.MULTILINE)
    for root in [plugin_root, runtime_root]:
        text = (root / 'config.hpp').read_text()
        macros.append(dict(pattern.findall(text)))
        normalized.append(pattern.sub('', text))
    assert normalized[0] == normalized[1], 'unexpected DART config difference'
    assert macros[0].keys() == macros[1].keys() and len(macros[0]) == 6
    for name in ['HAVE_NLOPT', 'HAVE_IPOPT']:
        assert macros[0][name] == macros[1][name] or (macros[0][name], macros[1][name]) == ('0', '1')
    for name in macros[0].keys() - {'HAVE_NLOPT', 'HAVE_IPOPT'}:
        assert all(re.fullmatch(r'"/[^"\n]*"', values[name]) for values in macros), name
    common = trees[0].keys() & trees[1].keys() - {'config.hpp'}
    assert all(trees[0][name] == trees[1][name] for name in common)
    return {'plugin_root': str(plugin_root), 'runtime_root': str(runtime_root),
            'header_counts': list(map(len, trees)), 'differences': differences,
            'config_macros': {'plugin': macros[0], 'runtime': macros[1]},
            'identical_common_headers': len(common),
            'tree_sha256': [hashlib.sha256(json.dumps(tree, sort_keys=True).encode()).hexdigest()
                            for tree in trees]}


def installed_header_identity(source_root, installed_root):
    files = {str(path.relative_to(source_root)): digest(path)
             for path in sorted(source_root.rglob('*.hh'))}
    assert files, str(source_root)
    for name, expected in files.items():
        assert digest(installed_root / name) == expected, name
    return {'source_root': str(source_root), 'installed_root': str(installed_root),
            'identical_headers': len(files),
            'tree_sha256': hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['physics-source', 'build', 'prefix', 'work-dir', 'report']:
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--write-fixture', action='store_true')
    args = parser.parse_args()
    source, build, prefix, work = [p.resolve() for p in
        [args.physics_source, args.build, args.prefix, args.work_dir]]
    lock = next(row['commit'] for row in json.loads((ROOT / 'planning/baseline.json').read_text())['repositories']
                if row['name'] == 'gz-physics')
    assert subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip() == lock
    subprocess.run(['git', '-C', str(source), 'diff', '--quiet', lock, '--'], check=True)
    sources = {}
    for name in ['dartsim/src/JointFeatures.cc', 'dartsim/src/SimulationFeatures.cc', 'dartsim/src/SDFFeatures.cc']:
        assert (source / name).read_bytes() == subprocess.check_output(['git', '-C', str(source), 'show', lock + ':' + name])
        sources[name] = digest(source / name)
    cache = (build / 'CMakeCache.txt').read_text()
    home = next(line.split('=', 1)[1] for line in cache.splitlines() if line.startswith('CMAKE_HOME_DIRECTORY:'))
    assert Path(home).resolve() == source
    install_prefix = next(line.split('=', 1)[1] for line in cache.splitlines()
                          if line.startswith('CMAKE_INSTALL_PREFIX:'))
    assert Path(install_prefix).resolve() == prefix
    build_command = ['cmake', '--build', str(build), '--target', 'gz-physics-dartsim-plugin', '--parallel', '2']
    subprocess.run(build_command, check=True, stdout=subprocess.DEVNULL)
    work.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PKG_CONFIG_PATH=str(prefix / 'lib/pkgconfig'))
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs',
        'gz-physics', 'gz-physics-sdf', 'gz-plugin-loader', 'sdformat', 'gz-math-eigen3'], env=env, text=True))
    installed_headers = Path(subprocess.check_output(
        ['pkg-config', '--variable=includedir', 'gz-physics'], env=env, text=True).strip()).resolve()
    sdk_headers = {name: installed_header_identity(source / relative, installed_headers)
                   for name, relative in [('core', 'include'), ('sdf', 'sdf/include')]}
    cpp, binary, maps = FIXTURES / 'moving_parent.cc', work / 'moving-parent', work / 'loaded.maps'
    command = ['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', str(cpp), '-o', str(binary),
               *flags, '-Wl,-rpath,' + str(prefix / 'lib')]
    subprocess.run(command, check=True)
    system = dict(os.environ, PKG_CONFIG_PATH='')
    system.pop('PKG_CONFIG_LIBDIR', None)
    assert subprocess.check_output(['pkg-config', '--modversion', 'dart'], env=system, text=True).strip() == '6.13.2'
    dart = Path(subprocess.check_output(['pkg-config', '--variable=libdir', 'dart'], env=system, text=True).strip()).resolve()
    ode = Path(subprocess.check_output(['pkg-config', '--variable=libdir', 'ode'], env=system, text=True).strip()).resolve()
    include_roots = next(line.split('=', 1)[1] for line in cache.splitlines()
                         if line.startswith('DART_INCLUDE_DIRS:')).split(';')
    runtime_headers = Path(subprocess.check_output(
        ['pkg-config', '--variable=includedir', 'dart'], env=system, text=True).strip()).resolve()
    headers = [header_identity((Path(path) / 'dart').resolve(), runtime_headers)
               for path in include_roots if (Path(path) / 'dart/dynamics/Skeleton.hpp').is_file()]
    assert headers, 'native build has no recorded DART header root'
    native_flags = build / 'dartsim/CMakeFiles/gz-physics-dartsim-plugin.dir/flags.make'
    native_link = native_flags.with_name('link.txt')
    target_flags, target_link = native_flags.read_text(), native_link.read_text()
    includes = shlex.split(next(line.split('=', 1)[1] for line in target_flags.splitlines()
                               if line.startswith('CXX_INCLUDES =')))
    for path in include_roots:
        assert path in includes or '-I' + path in includes, path
    linked_dart = [str(Path(token).resolve(strict=True)) for token in shlex.split(target_link)
                   if token.startswith('/') and Path(token).name.startswith('libdart')]
    assert linked_dart, 'native plugin link command has no explicit DART library'
    env['LD_LIBRARY_PATH'] = str(dart) + ':' + str(ode) + ':' + str(build / 'lib') + ':' + str(prefix / 'lib')
    plugin = (build / 'lib/libgz-physics-dartsim-plugin.so').resolve(strict=True)
    result = subprocess.run([str(binary), str(plugin), str(maps)], env=env,
                            capture_output=True, text=True, check=True, timeout=30)
    assert result.stderr == '', result.stderr
    observations = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(observations) == 34
    paths = {Path(line.split(maxsplit=5)[5]) for line in maps.read_text().splitlines()
             if len(line.split(maxsplit=5)) == 6 and line.split(maxsplit=5)[5].startswith('/')}
    libraries = {}
    for name, directory in [('libdart.so.', dart), ('libdart-collision-ode.so.', dart),
                            ('libdart-external-odelcpsolver.so.', dart), ('libode.so.', ode),
                            ('libgz-physics-dartsim-plugin.so.', build / 'lib')]:
        matches = [path for path in paths if path.name.startswith(name)]
        assert len(matches) == 1, (name, matches)
        actual = matches[0]
        assert actual.parent.resolve() == directory.resolve(), str(actual)
        libraries[actual.name] = digest(actual)
    fixture = {'source_sha256': digest(cpp), 'physics_commit': lock,
               'dart_version': '6.13.2', 'observations': observations}
    path = FIXTURES / 'moving_parent.json'
    if args.write_fixture:
        path.write_text(json.dumps(fixture, indent=2) + '\n')
    else:
        assert json.loads(path.read_text()) == fixture
    report = dict(scope='Fresh client and rebuilt pinned plugin target; the existing SDK uses ROS DART headers/link-time libraries and runs against verified OSRF DART libraries. Header identity and the bounded configuration differences are recorded. Not a homogeneous or fresh SDK configuration or performance comparison.',
                  fixture=fixture, sources=sources, loaded_libraries=libraries,
                  dart_headers=headers, sdk_headers=sdk_headers,
                  plugin_build_flags=target_flags, plugin_link_command=target_link,
                  plugin_linked_dart=linked_dart,
                  cmake_cache_sha256=digest(build / 'CMakeCache.txt'), build_command=build_command, command=command,
                  compiler=subprocess.check_output(['c++', '--version'], text=True),
                  binary_sha256=digest(binary), stderr=result.stderr)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n')
    print('34 native moving-parent observations passed')


if __name__ == '__main__':
    main()
