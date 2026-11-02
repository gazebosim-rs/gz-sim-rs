# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Historical oracle projection must not compile the current adapter header."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'three_body_static_reference', ROOT / 'scripts/reference-three-body-static-contacts.py')
REPLAY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPLAY)


class ThreeBodyStaticReference(unittest.TestCase):
    def fixture(self):
        return json.loads(REPLAY.FIXTURE.read_text())

    def inputs(self, root):
        for name in (REPLAY.PROBE, REPLAY.IDENTITY, REPLAY.HISTORICAL_HEADER):
            target = root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes((ROOT / name).read_bytes())
        candidate = root / REPLAY.HEADER
        candidate.parent.mkdir(parents=True, exist_ok=True)
        candidate.write_text('#error Candidate header must not enter the historical oracle\n')

    def test_historical_fixture_is_the_original_bound_header(self):
        fixture = self.fixture()
        self.assertEqual(set(fixture['native_sources']),
                         {REPLAY.PROBE, REPLAY.IDENTITY, REPLAY.HEADER})
        self.assertEqual(fixture['native_sources'][REPLAY.HEADER],
                         '7e5de3ff53d2e6b4a16aa55d176564942cca05ae5611c875951d1c3041a7ec5d')
        for logical, physical in ((REPLAY.PROBE, REPLAY.PROBE),
                                  (REPLAY.IDENTITY, REPLAY.IDENTITY),
                                  (REPLAY.HEADER, REPLAY.HISTORICAL_HEADER)):
            self.assertEqual(REPLAY.digest(ROOT / physical), fixture['native_sources'][logical])
        header = (ROOT / REPLAY.HISTORICAL_HEADER).read_text()
        self.assertIn('Copyright (C) 2026 Open Source Robotics Foundation', header)
        self.assertIn('SPDX-License-Identifier: Apache-2.0', header)

    def test_projection_preserves_actual_relative_includes_and_records_inputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'input'
            output = Path(temporary) / 'output'
            output.mkdir()
            self.inputs(root)
            source, records = REPLAY.project_sources(root, output, self.fixture())
            self.assertEqual(source, output / 'historical-sources' / REPLAY.PROBE)
            by_name = {row['logical_path']: row for row in records}
            self.assertEqual(len(by_name), 3)
            for logical, row in by_name.items():
                projected = Path(row['projected_path'])
                self.assertEqual(projected, output / 'historical-sources' / logical)
                self.assertEqual(projected.read_bytes(), Path(row['input_path']).read_bytes())
                self.assertEqual(REPLAY.digest(projected), row['sha256'])
            header = (source.parent / '../../src/joint_state.hh').resolve()
            self.assertEqual(header.read_bytes(), (root / REPLAY.HISTORICAL_HEADER).read_bytes())
            self.assertNotEqual(header.read_bytes(), (root / REPLAY.HEADER).read_bytes())
            self.assertEqual((source.parent / 'body_revolute_identity.hh').read_bytes(),
                             (root / REPLAY.IDENTITY).read_bytes())
            self.assertEqual(by_name[REPLAY.HEADER]['input_path'],
                             str(root / REPLAY.HISTORICAL_HEADER))
            with self.assertRaises(FileExistsError):
                REPLAY.project_sources(root, output, self.fixture())

    def test_changed_or_missing_logical_input_refuses_before_projection(self):
        for name in (REPLAY.PROBE, REPLAY.IDENTITY, REPLAY.HISTORICAL_HEADER):
            for change in ('modify', 'remove'):
                with self.subTest(name=name, change=change), tempfile.TemporaryDirectory() as temporary:
                    root = Path(temporary) / 'input'
                    output = Path(temporary) / 'output'
                    output.mkdir()
                    self.inputs(root)
                    path = root / name
                    if change == 'modify':
                        path.write_bytes(path.read_bytes() + b'\n// changed\n')
                    else:
                        path.unlink()
                    with self.assertRaises((AssertionError, FileNotFoundError)):
                        REPLAY.project_sources(root, output, self.fixture())
                    self.assertFalse((output / 'historical-sources').exists())

    def test_fixture_cannot_redirect_or_drop_owned_sources(self):
        for change in ('drop', 'extra', 'redirect', 'probe_hash'):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary) / 'input'
                output = Path(temporary) / 'output'
                output.mkdir()
                self.inputs(root)
                fixture = self.fixture()
                if change == 'drop':
                    fixture['native_sources'].pop(REPLAY.IDENTITY)
                elif change == 'extra':
                    fixture['native_sources']['extra.hh'] = '0' * 64
                elif change == 'redirect':
                    value = fixture['native_sources'].pop(REPLAY.HEADER)
                    fixture['native_sources']['../escape.hh'] = value
                else:
                    fixture['probe_source_sha256'] = '0' * 64
                with self.assertRaises(AssertionError):
                    REPLAY.project_sources(root, output, fixture)
                self.assertFalse((output / 'historical-sources').exists())

    def test_dependency_guard_requires_all_projected_inputs_and_exact_system_closure(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'input'
            output = Path(temporary) / 'output'
            output.mkdir()
            self.inputs(root)
            _, records = REPLAY.project_sources(root, output, self.fixture())
            system = {'/qualified/system/header.hh': '1' * 64}
            owned = {str(Path(row['projected_path']).resolve()): row['sha256'] for row in records}
            valid = owned | system
            REPLAY.check_compiler_inputs(valid, records, system)
            for name in owned:
                for change in ('drop', 'modify'):
                    with self.subTest(name=name, change=change):
                        invalid = copy.copy(valid)
                        if change == 'drop':
                            del invalid[name]
                        else:
                            invalid[name] = '0' * 64
                        with self.assertRaises(AssertionError):
                            REPLAY.check_compiler_inputs(invalid, records, system)
            for invalid in (owned,
                            valid | {'/qualified/system/header.hh': '0' * 64},
                            valid | {'/unexpected/system/header.hh': '1' * 64},
                            valid | {str((root / REPLAY.HEADER).resolve()):
                                     REPLAY.digest(root / REPLAY.HEADER)}):
                with self.subTest(dependencies=invalid), self.assertRaises(AssertionError):
                    REPLAY.check_compiler_inputs(invalid, records, system)
