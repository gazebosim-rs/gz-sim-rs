# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Validate the distinct native client reference contracts before compilation."""
import hashlib
import importlib.util
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('reference_builder', ROOT / 'reference/build-reference.py')
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)

LEGACY = 'transport-client-v1'
SDK = 'clean-headless-sdk-v1'


def contained_file(root, name):
    relative = Path(name)
    if relative.is_absolute() or '..' in relative.parts:
        raise ValueError('reference path escapes its root')
    path = root / relative
    if not path.is_file() or not path.resolve(strict=True).is_relative_to(root.resolve()):
        raise ValueError('reference file is missing or escapes its root: ' + name)
    return path


def validate_reference(prefix, metadata, transport_lock, sdk_environment, profile):
    if profile == LEGACY:
        if 'installed_files' in metadata or metadata.get('source_lock') != transport_lock:
            raise ValueError('native transport reference differs from pinned contract')
        libraries = metadata.get('libraries_sha256', {})
        if not libraries:
            raise ValueError('native reference has no library hashes')
        for name, expected in libraries.items():
            if builder.digest(contained_file(prefix, name)) != expected:
                raise ValueError('native reference library changed: ' + name)
        return {'profile': LEGACY, 'source_lock': transport_lock,
                'libraries_sha256': libraries}
    if profile != SDK:
        raise ValueError('unknown native reference profile')
    if metadata != sdk_environment or 'source_lock' in metadata:
        raise ValueError('native SDK differs from the independently qualified manifest')
    root = prefix.parent
    for name, expected in metadata['inputs_sha256'].items():
        path = contained_file(root, name)
        if path.is_symlink() or builder.digest(path) != expected:
            raise ValueError('native SDK input changed: ' + name)
    for path in prefix.rglob('*'):
        if not (path.is_symlink() or path.is_file() or path.is_dir()):
            raise ValueError('unsupported native SDK installed entry')
    if builder.files_under(prefix) != metadata['installed_files']:
        raise ValueError('native SDK installed inventory changed')
    env = builder.git_environment()
    for row in metadata['repositories']:
        builder.verify_source(root / 'sources' / row['name'], row, env)
    libraries = {name: entry['sha256'] for name, entry in metadata['installed_files'].items()
                 if 'sha256' in entry and '.so' in Path(name).name}
    if not libraries:
        raise ValueError('native SDK has no shared libraries')
    return {'profile': SDK, 'libraries_sha256': libraries,
            'inputs_sha256': metadata['inputs_sha256'],
            'expected_compiler_sha256': metadata['tools_sha256']['c++']}


def compiler_identity(env, expected=None):
    executable = shutil.which('c++', path=env.get('PATH', ''))
    if executable is None:
        raise ValueError('native client compiler is unavailable')
    path = Path(executable).resolve(strict=True)
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if expected is not None and digest != expected:
        raise ValueError('native client compiler differs from the expected identity')
    version = subprocess.check_output([str(path), '--version'], env=env, text=True)
    return str(path), {'sha256': digest, 'version': version}
