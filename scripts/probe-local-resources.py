#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Reproduce installed SDFormat lookup/include controls without network callbacks."""
import argparse
import hashlib
import html
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
    env = os.environ.copy()
    env['PKG_CONFIG_PATH'] = str(args.dependency_prefix.resolve() / 'lib/pkgconfig')
    env['LD_LIBRARY_PATH'] = str(args.dependency_prefix.resolve() / 'lib')
    for name in ['SDF_PATH', 'GZ_SIM_RESOURCE_PATH', 'GZ_FILE_PATH']:
        env.pop(name, None)
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'sdformat'], env=env, text=True))
    programs, builds = {}, []
    for name in ['lookup', 'include', 'model_file']:
        source = ROOT / f'crates/gz-assets/tests/native_{name}.cc'
        binary = work / name
        command = ['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', str(source), *flags, '-o', str(binary)]
        subprocess.run(command, env=env, check=True)
        linkage = subprocess.check_output(['ldd', str(binary)], env=env, text=True)
        builds.append({'command': command, 'source_sha256': digest(source), 'binary_sha256': digest(binary),
                       'libraries': {token: digest(Path(token)) for token in linkage.split()
                                     if token.startswith('/') and Path(token).is_file()}})
        programs[name] = binary
    for directory in ['cwd', 'a', 'b', 'parent']:
        (work / directory).mkdir(exist_ok=True)
        (work / directory / 'choice.sdf').write_text(directory)
    (work / 'cwd/a b.sdf').write_text('space')
    (work / 'cwd/a%20b.sdf').write_text('percent')
    (work / 'cwd/block').write_text('file')
    (work / 'a/block').mkdir(exist_ok=True)
    (work / 'a/block/selected.sdf').write_text('selected')
    for directory in ['a', 'b']:
        (work / directory / 'env-only.sdf').write_text(directory)
    rows = []
    cases = [
        ('cwd-first', 'choice.sdf', [], 'cwd', 'cwd/choice.sdf'),
        ('map-first', 'model://choice.sdf', [str(work / 'a'), str(work / 'b')], 'cwd', 'a/choice.sdf'),
        ('model-no-map', 'model://choice.sdf', [], 'cwd', 'cwd/choice.sdf'),
        ('file-relative', 'file://choice.sdf', [], 'cwd', 'cwd/choice.sdf'),
        ('file-absolute', 'file://' + str(work / 'parent/choice.sdf'), [], 'cwd', 'parent/choice.sdf'),
        ('percent-literal', 'file://a%20b.sdf', [], 'cwd', 'cwd/a%20b.sdf'),
        ('space-literal', 'file://a b.sdf', [], 'cwd', 'cwd/a b.sdf'),
        ('sdf-path', 'env-only.sdf', [], 'parent', 'a/env-only.sdf'),
        ('not-directory-cwd-falls-through', 'block/selected.sdf', [], 'cwd', 'a/block/selected.sdf')]
    for label, uri, maps, cwd, expected in cases:
        env['SDF_PATH'] = os.pathsep.join([str(work / 'a'), str(work / 'b')])
        result = subprocess.run([str(programs['lookup']), uri, *maps], cwd=work / cwd, env=env, capture_output=True, text=True, check=True)
        native = json.loads(result.stdout)
        if native != {'path': str(work / expected), 'errors': 0}:
            raise ValueError(f'{label}: unexpected native result {native}')
        rows.append({'label': label, 'uri': uri, 'native': native, 'stderr': result.stderr})
    absolute = work / 'parent/choice.sdf'
    shadow = work / 'cwd' / absolute.relative_to(absolute.anchor)
    shadow.parent.mkdir(parents=True, exist_ok=True)
    shadow.write_text('shadow')
    extra = [('cwd-absolute-shadow', 'file://' + str(absolute), [], shadow),
             ('map-leading-root', 'model:///choice.sdf', [str(work / 'a')], work / 'a/choice.sdf')]
    for label, uri, maps, expected in extra:
        result = subprocess.run([str(programs['lookup']), uri, *maps], cwd=work / 'cwd', env=env, capture_output=True, text=True, check=True)
        native = json.loads(result.stdout)
        if native['errors'] != 0 or Path(native['path']).resolve() != expected.resolve():
            raise ValueError(f'{label}: unexpected native result {native}')
        rows.append({'label': label, 'uri': uri, 'native': native, 'stderr': result.stderr})
    model = work / 'configured-model'
    model.mkdir(exist_ok=True)
    (model / 'model.config').write_text("<model><name>model</name><sdf version='1.12'>" + html.escape(str(absolute)) + "</sdf></model>")
    result = subprocess.run([str(programs['model_file']), str(model)], env=env, capture_output=True, text=True, check=True)
    native = json.loads(result.stdout)
    if native != {'path': str(model) + '/' + str(absolute), 'errors': 0}:
        raise ValueError(f'absolute-config-filename: unexpected native result {native}')
    rows.append({'label': 'absolute-config-filename', 'native': native, 'stderr': result.stderr})
    (work / 'cwd/stat-loop.sdf').unlink(missing_ok=True)
    (work / 'cwd/stat-loop.sdf').symlink_to('stat-loop.sdf')
    (work / 'a/stat-loop.sdf').write_text('fallback')
    env['SDF_PATH'] = str(work / 'a')
    result = subprocess.run([str(programs['lookup']), 'stat-loop.sdf'], cwd=work / 'cwd', env=env, capture_output=True, text=True, check=True)
    native = json.loads(result.stdout)
    if native != {'path': str(work / 'a/stat-loop.sdf'), 'errors': 0}:
        raise ValueError(f'stat-loop: unexpected result {native}')
    rows.append({'label': 'stat-loop-falls-through', 'native': native, 'stderr': result.stderr})
    model = work / 'loop-config'
    model.mkdir(exist_ok=True)
    (model / 'model.config').unlink(missing_ok=True)
    (model / 'model.config').symlink_to('model.config')
    (model / 'manifest.xml').write_text("<model><name>model</name><sdf version='1.12'>alternative.sdf</sdf></model>")
    result = subprocess.run([str(programs['model_file']), str(model)], env=env, capture_output=True, text=True, check=True)
    native = json.loads(result.stdout)
    if native != {'path': str(model / 'alternative.sdf'), 'errors': 1}:
        raise ValueError(f'loop-config: unexpected result {native}')
    rows.append({'label': 'stat-loop-manifest-fallback', 'native': native, 'stderr': result.stderr})
    model = work / 'bad-manifest'
    model.mkdir(exist_ok=True)
    (model / 'manifest.xml').write_text('broken')
    result = subprocess.run([str(programs['model_file']), str(model), '--codes'], env=env, capture_output=True, text=True, check=True)
    native = json.loads(result.stdout)
    if native != {'path': '', 'errors': 2, 'codes': [37, 40]}:
        raise ValueError(f'bad-manifest: unexpected result {native}')
    rows.append({'label': 'bad-manifest-preserves-legacy-warning', 'native': native, 'stderr': result.stderr})
    for directory in ['cwd', 'parent', 'a']:
        (work / directory / 'picked.sdf').write_text(f"<sdf version='1.12'><model name='{directory}'><static>true</static><link name='body'/></model></sdf>")
    world = work / 'parent/world.sdf'
    world.write_text("<sdf version='1.12'><world name='default'><include><uri>picked.sdf</uri></include></world></sdf>")
    for label, expected in [('cwd-first', 'cwd'), ('env-next', 'a'), ('parent-only', '')]:
        env['SDF_PATH'] = str(work / 'a') if label != 'parent-only' else ''
        if label == 'env-next':
            (work / 'cwd/picked.sdf').unlink()
        result = subprocess.run([str(programs['include']), str(world)], cwd=work / 'cwd', env=env, capture_output=True, text=True, check=True)
        native = json.loads(result.stdout)
        if native != {'name': expected, 'errors': [14] if label == 'parent-only' else []}:
            raise ValueError(f'{label}: unexpected include result {native}')
        rows.append({'label': 'include-' + label, 'native': native, 'stderr': result.stderr})
    report = {'qualification': 'Installed SDFormat SDK, no network callback; not a complete native gz-sim/Fuel or reproduced baseline. Linked hashes bind the actual SDK.',
              'builds': builds, 'rows': rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(f'{len(rows)} native lookup/include controls passed')


if __name__ == '__main__':
    main()
