#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Closed archive trust boundaries, independent of network or signature execution."""
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import tempfile
import time
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('stage_zenoh_package_metadata',
                                            ROOT / 'scripts/stage-zenoh-package-metadata.py')
STAGER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(STAGER)


class SignedMetadataStaging(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.pins = json.loads((ROOT / 'planning/zenoh-native-packages.json').read_text())
        self.bodies = {}
        for release in self.pins['releases']:
            name = release['suite'] + '.InRelease'
            self.bodies[name] = ('signed test body ' + name).encode()
            release['sha256'] = hashlib.sha256(self.bodies[name]).hexdigest()
            for index in release['indexes']:
                name = release['suite'] + '-' + index['component'] + '.Packages.xz'
                self.bodies[name] = ('opaque test index ' + name).encode()
                index['size'] = len(self.bodies[name])
                index['sha256'] = hashlib.sha256(self.bodies[name]).hexdigest()
        self.expected = STAGER.expected_members(self.pins)

    def archive(self, entries=None, archive_format=tarfile.USTAR_FORMAT):
        output = io.BytesIO()
        with tarfile.open(fileobj=output, mode='w', format=archive_format) as archive:
            for name, body, kind in (entries if entries is not None else
                                     [(name, body, tarfile.REGTYPE)
                                      for name, body in self.bodies.items()]):
                info = tarfile.TarInfo(name)
                info.mode = 0o644
                info.size = len(body)
                info.type = kind
                if kind in (tarfile.SYMTYPE, tarfile.LNKTYPE):
                    info.linkname = 'elsewhere'
                archive.addfile(info, io.BytesIO(body))
        return output.getvalue()

    def validate(self, data):
        archive = self.root / 'input.tar'
        archive.write_bytes(data)
        destination = self.root / 'validated'
        destination.mkdir(exist_ok=True)
        # Each malformed archive is validated in a fresh private directory.
        with tempfile.TemporaryDirectory(dir=destination) as private:
            STAGER.validate_archive(archive, Path(private), self.expected, time.monotonic())
            return {p.name: p.read_bytes() for p in Path(private).iterdir()}

    def stage_inputs(self, data):
        manifest = {'schema_version': 1,
                    'profile': 'ubuntu-zenoh-native-package-metadata-archive-v1',
                    'snapshot': STAGER.SNAPSHOT, 'url': STAGER.URL,
                    'sha256': hashlib.sha256(data).hexdigest(), 'size': len(data)}
        manifest_path = self.root / 'manifest.json'
        pins_path = self.root / 'pins.json'
        manifest_path.write_text(json.dumps(manifest))
        pins_path.write_text(json.dumps(self.pins))
        return manifest_path, pins_path

    def test_canonical_nine_members_preserve_opaque_signed_bytes(self):
        self.assertEqual(len(self.expected), 9)
        self.assertEqual(self.validate(self.archive()), self.bodies)
        self.assertEqual({n for n in self.expected if n.endswith('.InRelease')},
                         {'noble.InRelease', 'noble-updates.InRelease', 'noble-security.InRelease'})
        self.assertEqual(sum(n.endswith('.Packages.xz') for n in self.expected), 6)

    def test_missing_extra_duplicate_and_nonflat_names_are_rejected(self):
        entries = [(name, body, tarfile.REGTYPE) for name, body in self.bodies.items()]
        for bad in (entries[:-1], entries + [('extra', b'x', tarfile.REGTYPE)],
                    entries + [entries[0]],
                    [('../' + entries[0][0], entries[0][1], tarfile.REGTYPE)] + entries[1:],
                    [('/' + entries[0][0], entries[0][1], tarfile.REGTYPE)] + entries[1:],
                    [('sub/' + entries[0][0], entries[0][1], tarfile.REGTYPE)] + entries[1:]):
            with self.subTest(names=[entry[0] for entry in bad]), self.assertRaises(ValueError):
                self.validate(self.archive(bad))

    def test_links_directories_pax_and_gnu_extensions_are_not_regular_inputs(self):
        entries = [(name, body, tarfile.REGTYPE) for name, body in self.bodies.items()]
        for kind in (tarfile.SYMTYPE, tarfile.LNKTYPE, tarfile.DIRTYPE, tarfile.FIFOTYPE):
            bad = [(entries[0][0], b'', kind)] + entries[1:]
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                self.validate(self.archive(bad))
        for archive_format in (tarfile.GNU_FORMAT,):
            with self.assertRaises(ValueError):
                self.validate(self.archive(archive_format=archive_format))
        long_pax = [('p' * 110, b'x', tarfile.REGTYPE)] + entries
        with self.assertRaises(ValueError):
            self.validate(self.archive(long_pax, tarfile.PAX_FORMAT))

    def test_header_checksum_truncation_padding_and_trailing_payload_are_rejected(self):
        data = self.archive()
        damaged = bytearray(data)
        damaged[10] ^= 1
        first_size = len(next(iter(self.bodies.values())))
        padding = bytearray(data)
        padding[512 + first_size] = 1
        for bad in (bytes(damaged), data[:600], bytes(padding), data + b'not-zero'):
            with self.subTest(size=len(bad)), self.assertRaises(ValueError):
                self.validate(bad)

    def test_body_hash_exact_index_size_and_release_cap_are_enforced(self):
        entries = [(name, body, tarfile.REGTYPE) for name, body in self.bodies.items()]
        for target in (0, 1):
            bad = copy.deepcopy(entries)
            name, body, kind = bad[target]
            bad[target] = (name, b'X' + body[1:], kind)
            with self.assertRaises(ValueError):
                self.validate(self.archive(bad))
        bad = copy.deepcopy(entries)
        name, body, kind = bad[1]
        bad[1] = (name, body + b'X', kind)
        with self.assertRaises(ValueError):
            self.validate(self.archive(bad))
        bad = copy.deepcopy(entries)
        bad[0] = (bad[0][0], b'X' * (STAGER.RELEASE_CAP + 1), tarfile.REGTYPE)
        with self.assertRaises(ValueError):
            self.validate(self.archive(bad))

    def test_manifest_has_one_fixed_locator_snapshot_and_bounded_identity(self):
        data = self.archive()
        manifest_path, _ = self.stage_inputs(data)
        manifest = json.loads(manifest_path.read_text())
        for key, value in (('url', 'https://example.invalid/archive'), ('snapshot', 'later'),
                           ('profile', 'other'), ('sha256', '0' * 63),
                           ('size', True), ('size', STAGER.ARCHIVE_CAP + 512),
                           ('size', len(data) - 1), ('schema_version', True)):
            bad = dict(manifest, **{key: value})
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                STAGER.archive_identity(bad)
        self.assertEqual(STAGER.archive_identity(manifest),
                         (hashlib.sha256(data).hexdigest(), len(data)))

    def test_download_stream_checks_size_hash_cap_and_actual_fixed_url(self):
        data = self.archive()
        identity = (hashlib.sha256(data).hexdigest(), len(data))
        for payload, wanted in ((data + b'x', identity), (data[:-1], identity),
                                (data, ('0' * 64, len(data)))):
            response = io.BytesIO(payload)
            response.geturl = lambda: STAGER.URL
            destination = self.root / 'download.tar'
            destination.unlink(missing_ok=True)
            with mock.patch.object(STAGER.urllib.request, 'urlopen', return_value=response) as opened:
                with self.assertRaises(ValueError):
                    STAGER.download(destination, wanted, time.monotonic())
                opened.assert_called_once_with(STAGER.URL, timeout=30)
        response = io.BytesIO(data)
        response.geturl = lambda: STAGER.URL
        destination = self.root / 'valid.tar'
        with mock.patch.object(STAGER.urllib.request, 'urlopen', return_value=response):
            STAGER.download(destination, identity, time.monotonic())
        self.assertEqual(destination.read_bytes(), data)

    def test_publish_is_atomic_and_bad_late_member_never_creates_output(self):
        data = self.archive()
        manifest, pins = self.stage_inputs(data)
        destination = self.root / 'metadata'
        def supply(archive, _identity, _began):
            archive.write_bytes(data)
        with mock.patch.object(STAGER, 'download', side_effect=supply):
            result = STAGER.stage(destination, manifest, pins)
        self.assertEqual(result['files'], 9)
        self.assertEqual({p.name: p.read_bytes() for p in destination.iterdir()}, self.bodies)
        self.assertEqual(result['state'], 'SIGNED_METADATA_BYTES_STAGED_VERIFICATION_REQUIRED')
        with mock.patch.object(STAGER, 'download') as download, self.assertRaises(ValueError):
            STAGER.stage(destination, manifest, pins)
        download.assert_not_called()
        destination2 = self.root / 'invalid-metadata'
        entries = [(name, body, tarfile.REGTYPE) for name, body in self.bodies.items()]
        name, body, kind = entries[-1]
        entries[-1] = (name, b'X' + body[1:], kind)
        bad = self.archive(entries)
        manifest2, pins2 = self.stage_inputs(bad)
        with mock.patch.object(STAGER, 'download', side_effect=lambda p, *_: p.write_bytes(bad)):
            with self.assertRaises(ValueError):
                STAGER.stage(destination2, manifest2, pins2)
        self.assertFalse(destination2.exists())
        self.assertFalse(list(self.root.glob('.zenoh-package-metadata-*')))

    def test_existing_symlink_and_racing_empty_directory_are_not_overwritten(self):
        data = self.archive()
        manifest, pins = self.stage_inputs(data)
        real = self.root / 'real'
        real.mkdir()
        link = self.root / 'link'
        link.symlink_to(real, target_is_directory=True)
        with self.assertRaises(ValueError):
            STAGER.stage(link / 'metadata', manifest, pins)
        source = self.root / 'source'
        destination = self.root / 'occupied'
        source.mkdir()
        destination.mkdir()
        with self.assertRaises(FileExistsError):
            STAGER.publish_fresh(source, destination)
        self.assertTrue(source.is_dir())
        self.assertTrue(destination.is_dir())

    def test_canonical_member_selection_and_duplicate_json_cannot_expand_scope(self):
        for mutation in ('suite', 'component', 'hash', 'size'):
            pins = copy.deepcopy(self.pins)
            if mutation == 'suite':
                pins['releases'].append(pins['releases'][0])
            elif mutation == 'component':
                pins['releases'][0]['indexes'][0]['component'] = '../main'
            elif mutation == 'hash':
                pins['releases'][0]['sha256'] = 'not-a-hash'
            else:
                pins['releases'][0]['indexes'][0]['size'] = True
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                STAGER.expected_members(pins)
        path = self.root / 'duplicate.json'
        path.write_text('{"size":1,"size":2}')
        with self.assertRaises(ValueError):
            STAGER.read_json(path)

    def test_expired_download_and_validation_never_publish(self):
        data = self.archive()
        response = io.BytesIO(data)
        response.geturl = lambda: STAGER.URL
        with mock.patch.object(STAGER.urllib.request, 'urlopen', return_value=response):
            with self.assertRaises(ValueError):
                STAGER.download(self.root / 'late.tar',
                                (hashlib.sha256(data).hexdigest(), len(data)),
                                time.monotonic() - STAGER.SECONDS - 1)
        archive = self.root / 'late-input.tar'
        archive.write_bytes(data)
        output = self.root / 'late-output'
        output.mkdir()
        with self.assertRaises(ValueError):
            STAGER.validate_archive(archive, output, self.expected,
                                    time.monotonic() - STAGER.SECONDS - 1)
        self.assertEqual(list(output.iterdir()), [])


if __name__ == '__main__':
    unittest.main()
