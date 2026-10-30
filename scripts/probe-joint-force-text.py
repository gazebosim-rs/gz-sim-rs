#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Observe native plugin scalar text through the pinned clone/Get path."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / 'crates/gz-sim-scene/tests/fixtures'
CASES = [
    ('plain', 'hinge', True),
    ('ascii_edges', ' \t hinge \n ', True),
    ('ordinary_spaces', 'hi   nge', True),
    ('ordinary_tabs', 'hi\t\nnge', True),
    ('cdata', '<![CDATA[hinge]]>', True),
    ('cdata_spaces', '<![CDATA[ hi   nge ]]>', True),
    ('leading_comment', '<!--leading-->hinge', True),
    ('trailing_comment', 'hinge<!--trailing-->', True),
    ('entity', 'hi&#110;ge', True),
    ('nbsp', '\u00a0hinge\u00a0', True),
    ('split_cdata', '<![CDATA[hi]]><![CDATA[nge]]>', False),
    ('split_comment', 'hi<!--split-->nge', False),
    ('mixed', 'hi<![CDATA[nge]]>', False),
    ('processing_instruction', '<?instruction value?>hinge', False),
]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['sdformat-source', 'sim-source', 'prefix', 'work-dir', 'report']:
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--write-fixture', action='store_true')
    args = parser.parse_args()
    lock = {row['name']: row['commit'] for row in
            json.loads((ROOT / 'planning/baseline.json').read_text())['repositories']}
    sources = {}
    for name, source, files in [
        ('sdformat', args.sdformat_source.resolve(), ['src/parser.cc', 'src/Utils.cc', 'src/Plugin.cc']),
        ('gz-sim', args.sim_source.resolve(), ['src/systems/apply_joint_force/ApplyJointForce.cc', 'src/SystemManager.cc']),
    ]:
        assert subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip() == lock[name]
        for file in files:
            assert (source / file).read_bytes() == subprocess.check_output(['git', '-C', str(source), 'show', lock[name] + ':' + file])
            sources[name + '/' + file] = digest(source / file)
    prefix, work = args.prefix.resolve(), args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PKG_CONFIG_PATH=str(prefix / 'lib/pkgconfig'))
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'sdformat'], env=env, text=True))
    cpp = FIXTURES / 'joint_force_text.cc'
    binary, maps = work / 'joint-force-text', work / 'loaded.maps'
    command = ['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', str(cpp), *flags,
               '-Wl,--disable-new-dtags', '-Wl,-rpath,' + str(prefix / 'lib'), '-o', str(binary)]
    subprocess.run(command, check=True)
    observations, libraries = [], {}
    for name, content, supported in CASES:
        xml = '<joint_name>' + content + '</joint_name>'
        result = subprocess.run([str(binary), str(maps)], input=xml, text=True,
                                capture_output=True, check=True, timeout=10)
        observations.append(dict(name=name, xml=xml, supported=supported,
                                 stdout=result.stdout, stderr=result.stderr))
        loaded = {Path(line.split(maxsplit=5)[5]) for line in maps.read_text().splitlines()
                  if len(line.split(maxsplit=5)) == 6 and line.split(maxsplit=5)[5].startswith('/')}
        current = {}
        for stem in ['libsdformat.so.', 'libtinyxml2.so.']:
            paths = [path for path in loaded if path.name.startswith(stem)]
            assert len(paths) == 1, (stem, paths)
            if stem == 'libsdformat.so.':
                assert paths[0].parent.resolve() == (prefix / 'lib').resolve()
            current[paths[0].name] = digest(paths[0])
        if libraries:
            assert current == libraries
        libraries = current
    fixture = dict(source_sha256=digest(cpp), script_sha256=digest(Path(__file__)),
                   sdformat_commit=lock['sdformat'], sim_commit=lock['gz-sim'], observations=observations)
    path = FIXTURES / 'joint_force_text.json'
    if args.write_fixture:
        path.write_text(json.dumps(fixture, indent=2) + '\n')
    else:
        assert json.loads(path.read_text()) == fixture
    report = dict(scope='Fresh C++ installed-SDK clone/Get observations with selected pinned source checks; not a fresh SDK build or full system test.',
                  fixture=fixture, sources=sources, loaded_libraries=libraries, command=command,
                  compiler=subprocess.check_output(['c++', '--version'], text=True), binary_sha256=digest(binary))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + '\n')
    print(len(observations), 'native plugin text observations passed')


if __name__ == '__main__':
    main()
