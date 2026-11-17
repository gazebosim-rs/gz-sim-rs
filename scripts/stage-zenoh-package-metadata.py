#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Stage nine hash-pinned signed inputs; bootstrap remains the signature verifier.

Only the maintained archive manifest supplies the download identity. This does
not install packages, build an SDK, or replace bootstrap's gpgv/package checks.
"""
import argparse
import ctypes
import errno
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import tempfile
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'planning/zenoh-native-package-archive.json'
PINS = ROOT / 'planning/zenoh-native-packages.json'
DEFAULT_ROOT = ROOT / 'artifacts/zenoh-native-package-metadata'
SNAPSHOT = '20261024T000000Z'
URL = ('https://github.com/gazebosim-rs/gz-sim-rs/releases/download/'
       'native-build-inputs-v1/ubuntu-20261024T000000Z-zenoh-package-metadata.tar')
ARCHIVE_CAP = 32 * 1024 * 1024
RELEASE_CAP = 1024 * 1024
BLOCK = 512
SECONDS = 180


def require(condition, message):
    if not condition:
        raise ValueError(message)


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'duplicate JSON field')
        result[key] = value
    return result


def read_json(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    try:
        before = os.fstat(fd)
        require(stat.S_ISREG(before.st_mode) and before.st_size <= 128 * 1024,
                'input must be a bounded ordinary JSON file')
        data = b''
        while len(data) <= 128 * 1024:
            part = os.read(fd, min(65536, 128 * 1024 + 1 - len(data)))
            if not part:
                break
            data += part
        signature = lambda row: (row.st_dev, row.st_ino, row.st_mode, row.st_size,
                                 row.st_mtime_ns, row.st_ctime_ns)
        require(len(data) == before.st_size and signature(before) == signature(os.fstat(fd)),
                'JSON input changed or exceeds cap')
        return json.loads(data, object_pairs_hook=unique_object)
    finally:
        os.close(fd)


def digest_string(value):
    return isinstance(value, str) and re.fullmatch('[0-9a-f]{64}', value) is not None


def archive_identity(manifest):
    require(type(manifest.get('schema_version')) is int and manifest['schema_version'] == 1,
            'archive schema differs')
    require(manifest.get('profile') == 'ubuntu-zenoh-native-package-metadata-archive-v1'
            and manifest.get('snapshot') == SNAPSHOT and manifest.get('url') == URL,
            'archive identity differs')
    require(digest_string(manifest.get('sha256')), 'archive SHA256 missing')
    size = manifest.get('size')
    require(type(size) is int and 2 * BLOCK <= size <= ARCHIVE_CAP and size % BLOCK == 0,
            'archive size differs or exceeds cap')
    return manifest['sha256'], size


def expected_members(pins):
    require(pins.get('schema_version') == 1 and pins.get('snapshot') == SNAPSHOT
            and pins.get('profile') == 'ubuntu-zenoh-native-packages-v1',
            'canonical package metadata identity differs')
    releases = pins['releases']
    require([row['suite'] for row in releases] == ['noble', 'noble-updates', 'noble-security'],
            'canonical suite selection differs')
    result = {}
    for release in releases:
        suite = release['suite']
        require(digest_string(release['sha256']), 'release SHA256 differs')
        result[suite + '.InRelease'] = (release['sha256'], None, RELEASE_CAP)
        require([row['component'] for row in release['indexes']] == ['main', 'universe'],
                'canonical index selection differs')
        for index in release['indexes']:
            component = index['component']
            size = index['size']
            require(index['path'] == component + '/binary-amd64/Packages.xz'
                    and digest_string(index['sha256']) and type(size) is int
                    and 0 < size <= ARCHIVE_CAP, 'canonical index pin differs')
            result[suite + '-' + component + '.Packages.xz'] = (index['sha256'], size, size)
    require(len(result) == 9, 'exactly nine canonical members required')
    return result


def download(archive, identity, began):
    digest, expected_size = identity
    count = 0
    observed = hashlib.sha256()
    with urllib.request.urlopen(URL, timeout=30) as response, archive.open('xb') as output:
        require(response.geturl().startswith('https://'), 'non-HTTPS archive redirect')
        while True:
            require(time.monotonic() - began < SECONDS, 'archive download deadline')
            chunk = response.read(min(65536, expected_size + 1 - count))
            if not chunk:
                break
            count += len(chunk)
            require(count <= expected_size and count <= ARCHIVE_CAP, 'archive byte cap exceeded')
            output.write(chunk)
            observed.update(chunk)
        output.flush()
        os.fsync(output.fileno())
    require(time.monotonic() - began < SECONDS, 'archive download deadline')
    require(count == expected_size and observed.hexdigest() == digest,
            'archive size or SHA256 differs')


def octal(field):
    require(field and not field[0] & 128, 'non-octal tar number')
    value = field.rstrip(b'\0 ').lstrip(b' ')
    require(bool(value) and all(c in b'01234567' for c in value), 'invalid tar number')
    return int(value, 8)


def validate_archive(archive, destination, expected, began):
    """Parse raw USTAR only; never let tarfile interpret extension records."""
    seen = set()
    with archive.open('rb') as stream:
        while True:
            require(time.monotonic() - began < SECONDS, 'archive validation deadline')
            header = stream.read(BLOCK)
            require(len(header) == BLOCK, 'truncated tar header')
            if header == bytes(BLOCK):
                require(stream.read(BLOCK) == bytes(BLOCK), 'missing tar end blocks')
                while True:
                    tail = stream.read(65536)
                    if not tail:
                        break
                    require(not any(tail), 'trailing nonzero archive bytes')
                break
            require(header[257:265] == b'ustar\00000', 'only raw USTAR accepted')
            require(octal(header[148:156]) == sum(header[:148]) + 8 * 32 + sum(header[156:]),
                    'tar header checksum differs')
            require(header[156:157] in (b'0', b'\0') and not any(header[157:257])
                    and not any(header[345:500]), 'non-ordinary or extended tar member')
            raw_name = header[:100]
            name_bytes, separator, padding = raw_name.partition(b'\0')
            require(bool(separator) and not any(padding), 'invalid member name padding')
            name = name_bytes.decode('ascii')
            require(name in expected and name not in seen and '/' not in name
                    and '\\' not in name and name not in ('.', '..'),
                    'extra, duplicate or unsafe member name')
            size = octal(header[124:136])
            mode = octal(header[100:108])
            wanted_sha, wanted_size, cap = expected[name]
            require(mode & ~0o777 == 0 and 0 < size <= cap
                    and (wanted_size is None or size == wanted_size), 'member size or mode differs')
            digest = hashlib.sha256()
            remaining = size
            with (destination / name).open('xb') as output:
                while remaining:
                    require(time.monotonic() - began < SECONDS, 'member validation deadline')
                    chunk = stream.read(min(65536, remaining))
                    require(bool(chunk), 'truncated tar member')
                    remaining -= len(chunk)
                    digest.update(chunk)
                    output.write(chunk)
                output.flush()
                os.fsync(output.fileno())
            require(digest.hexdigest() == wanted_sha, 'member SHA256 differs')
            require(stream.read((-size) % BLOCK) == bytes((-size) % BLOCK),
                    'invalid member padding')
            seen.add(name)
    require(seen == set(expected), 'missing canonical member')
    require(time.monotonic() - began < SECONDS, 'archive validation deadline')


def publish_fresh(source, destination):
    # Linux CI: RENAME_NOREPLACE is atomic even against an empty-directory race.
    libc = ctypes.CDLL(None, use_errno=True)
    rename = libc.renameat2
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    rename.restype = ctypes.c_int
    if rename(-100, os.fsencode(source), -100, os.fsencode(destination), 1) != 0:
        code = ctypes.get_errno()
        if code == errno.EEXIST:
            raise FileExistsError(destination)
        raise OSError(code, os.strerror(code), str(destination))


def stage(metadata_root, manifest_path=MANIFEST, pins_path=PINS):
    began = time.monotonic()
    identity = archive_identity(read_json(manifest_path))
    expected = expected_members(read_json(pins_path))
    destination = Path(os.path.abspath(metadata_root))
    require(not any(p.is_symlink() for p in (destination, *destination.parents)),
            'metadata output must not follow symlinks')
    require(not destination.exists(), 'metadata output already exists')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.zenoh-package-metadata-', dir=destination.parent) as temporary:
        temporary = Path(temporary)
        archive = temporary / 'archive.tar'
        bodies = temporary / 'metadata'
        bodies.mkdir()
        download(archive, identity, began)
        validate_archive(archive, bodies, expected, began)
        require(not any(p.is_symlink() for p in (destination, *destination.parents)),
                'metadata output changed to symlink')
        require(time.monotonic() - began < SECONDS, 'staging deadline')
        directory_fd = os.open(bodies, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        publish_fresh(bodies, destination)
        parent_fd = os.open(destination.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            os.fsync(parent_fd)
        finally:
            os.close(parent_fd)
    return {'state': 'SIGNED_METADATA_BYTES_STAGED_VERIFICATION_REQUIRED',
            'metadata_root': str(destination), 'files': len(expected),
            'archive_sha256': identity[0], 'archive_bytes': identity[1]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT,
                        help='fresh output directory for the unchanged bootstrap verifier')
    args = parser.parse_args()
    print(json.dumps(stage(args.root), sort_keys=True))


if __name__ == '__main__':
    main()
