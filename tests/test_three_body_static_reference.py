# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Historical oracle projection must not compile the current adapter header."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

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


class DependencySnapshotSelection(unittest.TestCase):
    """Owned JSON/byte fixtures only; no SDK queries, compiler or native probe."""
    def fixture(self):
        return json.loads(REPLAY.FIXTURE.read_text())

    def synthetic(self, root):
        fixture = self.fixture()
        fixture_raw = (ROOT / REPLAY.FIXTURE).read_bytes()
        fixture_path = root / 'fixture.json'
        fixture_path.write_bytes(fixture_raw)
        fixture_card = {'path': str(fixture_path), 'sha256': REPLAY.FIXTURE_IDENTITY['sha256'], 'bytes': len(fixture_raw)}
        nodes = {}
        changed = []
        rows = []
        for path, (old, current) in REPLAY.APPROVED_DEPENDENCY_CHANGES.items():
            nodes[path] = {'sha256': current, 'bytes': 7}
            rows.append({'path': path, 'actual': {'sha256': current, 'bytes': 7}})
            changed.append({'path': path, 'previous_sha256': old, 'current_sha256': current,
                            'bytes': 7, 'dynamic_text_byte_identical': True,
                            'resolved_needed_unchanged': True,
                            'package_md5_comparison': {'matches_package_md5': True}})
        resolver = '/usr/lib/x86_64-linux-gnu/libresolv.so.2'
        nodes[resolver] = {'sha256': 'c' * 64, 'bytes': 7}
        rows.append({'path': resolver, 'actual': {'sha256': 'c' * 64, 'bytes': 7}})
        changed.append({'path': resolver})
        for n in range(345):
            name = '/synthetic/saved-node-' + str(n)
            nodes[name] = {'sha256': 'a' * 64, 'bytes': 7}
            rows.append({'path': name, 'actual': {'sha256': 'a' * 64, 'bytes': 7}})
        for n in range(2):
            rows.append({'path': '/synthetic/old-two-' + str(n), 'actual': {'sha256': 'b' * 64, 'bytes': 7}})
        inventory = {'state': 'SAVED351_CURRENT_IDENTITY_DIAGNOSTIC_ONLY_NOT_ADMISSION',
                     'internal_exit_code': 0, 'errors': [], 'observed_count': 351,
                     'matching_count': 347, 'delta_count': 4, 'rows': rows,
                     'deltas': [row['path'] for row in changed]}
        def store(name, value):
            path = root / name
            raw = (json.dumps(value) + '\n').encode()
            path.write_bytes(raw)
            return {'path': str(path), 'sha256': REPLAY.hashlib.sha256(raw).hexdigest(), 'bytes': len(raw)}
        inventory_card = store('inventory.json', inventory)
        receipt = {'report': inventory_card, 'actual_root_exit_code': 0,
                   'tool_exit_basis': {'actual_exit_code': 0, 'completion_chunk': 'synthetic-owned-zero'}}
        receipt_card = store('receipt.json', receipt)
        snapshot = {'nodes': nodes, 'environment_update_M58': {
            'retained_complete_inventory': inventory_card,
            'retained_inventory_actual_Root_receipt': receipt_card,
            'same_349_node_paths_and_needed_edges': True,
            'changed_current349_node_hashes': 4, 'unchanged_current349_node_hashes': 345,
            'changed_files': changed}}
        pins = {'snapshot': store('snapshot.json', snapshot),
                'inventory': inventory_card, 'actual_exit': receipt_card}
        return fixture, fixture_card, pins, snapshot, inventory, receipt

    def test_unset_is_exact_historical_and_present_empty_or_invalid_never_falls_back(self):
        fixture = self.fixture()
        libraries, record = REPLAY.dependency_environment(fixture, {}, {})
        self.assertEqual(libraries, fixture['native_libraries'])
        self.assertEqual(record['projection'], [])
        self.assertEqual(record['selected_metadata'], {})
        self.assertEqual(record['state'], 'HISTORICAL_STRICT_DEFAULT')
        for value in ['', '/not-the-reviewed-snapshot', 'relative.json']:
            with self.subTest(value=value), self.assertRaises(AssertionError):
                REPLAY.dependency_environment(fixture, {}, {REPLAY.DEPENDENCY_VARIABLE: value})

    def test_selected_exact_three_projection_preserves_full_eighteen_and_separate_custody(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture, card, pins, _, _, _ = self.synthetic(Path(temporary))
            original = copy.deepcopy(fixture)
            with patch.object(REPLAY, 'DEPENDENCY_PINS', pins):
                libraries, record = REPLAY.dependency_environment(fixture, card, {REPLAY.DEPENDENCY_VARIABLE: pins['snapshot']['path']})
                REPLAY.check_dependency_environment(record)
            self.assertEqual(fixture, original)
            self.assertEqual(len(libraries), 18)
            self.assertEqual(set(libraries), set(fixture['native_libraries']))
            self.assertEqual({name for name in libraries if libraries[name] != fixture['native_libraries'][name]}, set(REPLAY.APPROVED_DEPENDENCY_CHANGES))
            restored = dict(libraries)
            for name, (old, current) in REPLAY.APPROVED_DEPENDENCY_CHANGES.items():
                self.assertEqual(libraries[name], current)
                restored[name] = old
            self.assertEqual(restored, fixture['native_libraries'])
            self.assertEqual(record['selected_metadata'], pins)
            self.assertFalse(record['native_replay_pass_claimed_by_selection'])
            self.assertEqual(record['historical_libraries'], fixture['native_libraries'])
            self.assertEqual(record['effective_libraries'], libraries)

    def test_changed_metadata_and_postselection_mutation_have_no_fallback(self):
        for role in ('snapshot', 'inventory', 'actual_exit'):
            with self.subTest(role=role), tempfile.TemporaryDirectory() as temporary:
                fixture, card, pins, _, _, _ = self.synthetic(Path(temporary))
                with patch.object(REPLAY, 'DEPENDENCY_PINS', pins):
                    _, record = REPLAY.dependency_environment(fixture, card, {REPLAY.DEPENDENCY_VARIABLE: pins['snapshot']['path']})
                    Path(pins[role]['path']).write_text('{}')
                    with self.assertRaisesRegex(AssertionError, 'metadata identity mismatch'):
                        REPLAY.check_dependency_environment(record)
                    with self.assertRaisesRegex(AssertionError, 'metadata identity mismatch'):
                        REPLAY.dependency_environment(fixture, card, {REPLAY.DEPENDENCY_VARIABLE: pins['snapshot']['path']})

    def test_wrong_fixture_card_and_late_fixture_mutation_refuse(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture, card, pins, _, _, _ = self.synthetic(Path(temporary))
            with patch.object(REPLAY, 'DEPENDENCY_PINS', pins):
                for wrong in [dict(card, sha256='0' * 64), dict(card, bytes=card['bytes'] + 1)]:
                    with self.assertRaisesRegex(AssertionError, 'selected historical fixture mismatch'):
                        REPLAY.dependency_environment(fixture, wrong, {REPLAY.DEPENDENCY_VARIABLE: pins['snapshot']['path']})
                _, record = REPLAY.dependency_environment(fixture, card, {REPLAY.DEPENDENCY_VARIABLE: pins['snapshot']['path']})
                Path(card['path']).write_bytes(b'changed fixture')
                with self.assertRaisesRegex(AssertionError, 'metadata identity mismatch'):
                    REPLAY.check_dependency_environment(record)

    def test_extra_delta_or_changed_projected_baseline_refuses(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture, _, pins, snapshot, inventory, receipt = self.synthetic(Path(temporary))
            with patch.object(REPLAY, 'DEPENDENCY_PINS', pins):
                changed = copy.deepcopy(snapshot)
                changed['environment_update_M58']['changed_files'].append({'path': '/unexpected/fourth-native-delta'})
                with self.assertRaises(AssertionError):
                    REPLAY.project_dependency_snapshot(fixture, changed, inventory, receipt)
                for path in REPLAY.APPROVED_DEPENDENCY_CHANGES:
                    changed_fixture = copy.deepcopy(fixture)
                    changed_fixture['native_libraries'][path] = '0' * 64
                    with self.subTest(path=path), self.assertRaises(AssertionError):
                        REPLAY.project_dependency_snapshot(changed_fixture, snapshot, inventory, receipt)

    def test_root_or_internal_nonzero_boolean_or_uncompleted_is_not_eligible(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture, _, pins, snapshot, inventory, receipt = self.synthetic(Path(temporary))
            with patch.object(REPLAY, 'DEPENDENCY_PINS', pins):
                for value in [1, None, False]:
                    changed = copy.deepcopy(receipt)
                    changed['actual_root_exit_code'] = value
                    with self.subTest(root=value), self.assertRaises(AssertionError):
                        REPLAY.project_dependency_snapshot(fixture, snapshot, inventory, changed)
                    changed_inventory = copy.deepcopy(inventory)
                    changed_inventory['internal_exit_code'] = value
                    with self.subTest(internal=value), self.assertRaises(AssertionError):
                        REPLAY.project_dependency_snapshot(fixture, snapshot, changed_inventory, receipt)
                changed = copy.deepcopy(receipt)
                changed['tool_exit_basis']['completion_chunk'] = ''
                with self.assertRaises(AssertionError):
                    REPLAY.project_dependency_snapshot(fixture, snapshot, inventory, changed)

    def test_metadata_reader_refuses_symlink_oversize_duplicate_keys_and_hash_change(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            target = root / 'ordinary'
            target.write_bytes(b'0123456789')
            link = root / 'link'
            link.symlink_to(target)
            with self.assertRaises(OSError):
                REPLAY.verified_bytes(link)
            with self.assertRaisesRegex(AssertionError, 'metadata type/size'):
                REPLAY.verified_bytes(target, cap=9)
            with self.assertRaisesRegex(AssertionError, 'metadata identity mismatch') as failure:
                REPLAY.verified_bytes(target, {'path': str(target), 'sha256': '0' * 64, 'bytes': 10})
            self.assertIn('expected', str(failure.exception))
            self.assertIn('observed', str(failure.exception))
            with self.assertRaisesRegex(AssertionError, 'duplicate metadata key'):
                REPLAY.metadata_json(b'{"key":1,"key":2}')

    def test_live_library_guard_reports_expected_and_observed_on_owned_synthetic_bytes(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first, second, missing, good = [root / name for name in ('first', 'second', 'missing', 'last-good')]
            first.write_bytes(b'first ordinary synthetic bytes, never executed')
            second.write_bytes(b'second ordinary synthetic bytes, never executed')
            good.write_bytes(b'last ordinary synthetic bytes, never executed')
            with self.assertRaisesRegex(AssertionError, 'native dependency hash mismatch') as failure:
                REPLAY.check_libraries({str(first): '0' * 64, str(second): '1' * 64,
                                        str(missing): '2' * 64, str(good): REPLAY.digest(good)})
            report = failure.exception.args[0][1]
            self.assertEqual(report['observed_count'], 4)
            self.assertEqual([row['path'] for row in report['observations']], [str(first), str(second), str(missing), str(good)])
            self.assertEqual([row['path'] for row in report['mismatches']], [str(first), str(second), str(missing)])
            self.assertIsNotNone(report['mismatches'][2]['read_error'])
            self.assertIsNone(report['mismatches'][2]['observed_sha256'])
            self.assertEqual(report['observations'][-1]['observed_sha256'], REPLAY.digest(good))
            for field in ('expected_sha256', 'observed_sha256', 'observed_bytes', 'read_error'):
                self.assertIn(field, str(failure.exception))
