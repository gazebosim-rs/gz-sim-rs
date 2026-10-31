# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Reject stale source, headers or baseline identities in physics oracles."""
import hashlib
import importlib.util
import json
import math
import tempfile
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class JointEffortReference(unittest.TestCase):
    def test_native_fixture_binds_peer_source_and_physics_baseline(self):
        directory = ROOT / 'crates/gz-physics-dart/tests/fixtures'
        fixture = json.loads((directory / 'joint_effort.json').read_text())
        self.assertEqual(fixture['source_sha256'], hashlib.sha256((directory / 'joint_effort.cc').read_bytes()).hexdigest())
        baseline = json.loads((ROOT / 'planning/baseline.json').read_text())
        physics = next(row['commit'] for row in baseline['repositories'] if row['name'] == 'gz-physics')
        self.assertEqual(fixture['physics_commit'], physics)
        self.assertEqual(fixture['dart_version'], '6.13.2')
        self.assertEqual([row['effort'] for row in fixture['observations']], [1.5, 0, -3, 0, 2, 0])

    def test_moving_parent_fixture_binds_native_source_and_both_graphs(self):
        directory = ROOT / 'crates/gz-physics-dart/tests/fixtures'
        fixture = json.loads((directory / 'moving_parent.json').read_text())
        self.assertEqual(fixture['source_sha256'], hashlib.sha256((directory / 'moving_parent.cc').read_bytes()).hexdigest())
        baseline = json.loads((ROOT / 'planning/baseline.json').read_text())
        physics = next(row['commit'] for row in baseline['repositories'] if row['name'] == 'gz-physics')
        self.assertEqual(fixture['physics_commit'], physics)
        self.assertEqual(fixture['dart_version'], '6.13.2')
        ticks = list(range(7)) + list(range(100, 1001, 100))
        self.assertEqual([(row['tilted'], row['tick']) for row in fixture['observations']],
                         [(tilted, tick) for tilted in (False, True) for tick in ticks])
        pulse = [0, 1.5, 0, -3, 0, 2, 0] + [0] * 10
        for index, row in enumerate(fixture['observations']):
            self.assertIs(type(row['tilted']), bool)
            self.assertIs(type(row['tick']), int)
            self.assertEqual(row['effort'], pulse[index % len(ticks)])
            self.assertEqual(len(row['bodies']), 2)
            for body in row['bodies']:
                vectors = ['position', 'orientation', 'velocity', 'angular_velocity']
                if row['tick'] == 0:
                    vectors += ['center', 'anchor', 'axis']
                self.assertEqual(set(body), set(vectors) | {'q', 'dq'})
                for name in vectors:
                    self.assertEqual(len(body[name]), 4 if name == 'orientation' else 3)
                values = [body['q'], body['dq']] + [value for name in body if isinstance(body[name], list)
                                                  for value in body[name]]
                self.assertTrue(all(type(value) in (float, int) and math.isfinite(value) for value in values))

    def test_moving_parent_header_guard_accepts_only_documented_config_differences(self):
        script = ROOT / 'crates/gz-physics-dart/tests/fixtures/probe-moving-parent.py'
        spec = importlib.util.spec_from_file_location('moving_parent_headers', script)
        probe = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(probe)
        with tempfile.TemporaryDirectory() as temporary:
            roots = [Path(temporary) / name for name in ('plugin', 'runtime')]
            for index, root in enumerate(roots):
                (root / 'dynamics').mkdir(parents=True)
                (root / 'dynamics/Skeleton.hpp').write_text('same dynamics interface')
                (root / 'config.hpp').write_text(
                    f'#define HAVE_NLOPT {index}\n#define HAVE_IPOPT {index}\n'
                    + ''.join(f'#define {name} "/{index}/data"\n' for name in
                              ('DART_ROOT_PATH', 'DART_DATA_PATH', 'DART_DATA_LOCAL_PATH', 'DART_DATA_GLOBAL_PATH')))
            self.assertEqual(probe.header_identity(*roots)['identical_common_headers'], 1)
            header = roots[1] / 'dynamics/Skeleton.hpp'
            header.write_text('different dynamics interface')
            with self.assertRaises(AssertionError):
                probe.header_identity(*roots)
            header.write_text('same dynamics interface')
            config = roots[1] / 'config.hpp'
            original = config.read_text()
            config.write_text(original + '#define UNREVIEWED_ABI_CHANGE 1\n')
            with self.assertRaises(AssertionError):
                probe.header_identity(*roots)
            config.write_text(original.replace('HAVE_NLOPT 1', 'HAVE_NLOPT 2'))
            with self.assertRaises(AssertionError):
                probe.header_identity(*roots)

    def test_moving_parent_installed_header_guard_rejects_stale_headers(self):
        script = ROOT / 'crates/gz-physics-dart/tests/fixtures/probe-moving-parent.py'
        spec = importlib.util.spec_from_file_location('moving_parent_installed_headers', script)
        probe = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(probe)
        with tempfile.TemporaryDirectory() as temporary:
            source, installed = [Path(temporary) / name for name in ('source', 'installed')]
            for root in (source, installed):
                (root / 'gz/physics').mkdir(parents=True)
                (root / 'gz/physics/World.hh').write_text('pinned SDK interface')
            self.assertEqual(probe.installed_header_identity(source, installed)['identical_headers'], 1)
            (installed / 'gz/physics/World.hh').write_text('stale SDK interface')
            with self.assertRaises(AssertionError):
                probe.installed_header_identity(source, installed)

    def test_mujoco_fixture_binds_peer_source_and_sdk_revision(self):
        directory = ROOT / 'crates/gz-physics-mujoco/tests/fixtures'
        fixture = json.loads((directory / 'joint_effort.json').read_text())
        self.assertEqual(fixture['source_sha256'], hashlib.sha256((directory / 'joint_effort.cc').read_bytes()).hexdigest())
        spec = importlib.util.spec_from_file_location('physics_bootstrap', ROOT / 'scripts/bootstrap-physics.py')
        bootstrap = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(bootstrap)
        self.assertEqual(fixture['mujoco_commit'], bootstrap.MUJOCO_COMMIT)
        self.assertEqual(fixture['version'], bootstrap.MUJOCO_VERSION)
        self.assertEqual([row['effort'] for row in fixture['observations']], [1.5, 0, -3, 0, 2, 0])

    def test_rapier_rate_fixture_is_the_bound_native_dart_oracle(self):
        relative = 'tests/fixtures/joint_effort.json'
        native = ROOT / 'crates/gz-physics-dart' / relative
        rapier = ROOT / 'crates/gz-physics-rapier' / relative
        self.assertEqual(rapier.read_bytes(), native.read_bytes())
