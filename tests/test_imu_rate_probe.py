# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Capture failures must retain their observations; no native execution credit."""
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('imu_rate_probe', Path(__file__).resolve().parents[1] / 'scripts/probe-imu-rate.py')
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)


class TemporaryCase(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.out = Path(self.directory.name)


class CaptureTests(TemporaryCase):
    def capture(self):
        return PROBE.capture(self.out, 'case', ['probe'], {}, 3, cwd=self.out, stdin='xml')

    def state(self):
        return json.loads((self.out / 'case.exit.json').read_text())

    def test_launch_failure_retains_error(self):
        with patch.object(PROBE.subprocess, 'run', side_effect=FileNotFoundError('missing compiler')):
            with self.assertRaises(RuntimeError):
                self.capture()
        self.assertEqual(self.state()['state'], 'launch_failed')
        self.assertEqual(self.state()['command'], ['probe'])
        self.assertEqual((self.out / 'case.stdout').read_text(), '')
        self.assertIn('missing compiler', (self.out / 'case.stderr').read_text())

    def test_timeout_retains_partial_bytes(self):
        error = subprocess.TimeoutExpired(['probe'], 3, output=b'partial out', stderr=b'partial error')
        with patch.object(PROBE.subprocess, 'run', side_effect=error):
            with self.assertRaises(RuntimeError):
                self.capture()
        self.assertEqual(self.state()['state'], 'timeout')
        self.assertEqual((self.out / 'case.stdout').read_text(), 'partial out')
        self.assertEqual((self.out / 'case.stderr').read_text(), 'partial error')

    def test_timeout_retains_partial_text_and_absent_stderr(self):
        error = subprocess.TimeoutExpired(['probe'], 3, output='partial out')
        with patch.object(PROBE.subprocess, 'run', side_effect=error):
            with self.assertRaises(RuntimeError):
                self.capture()
        self.assertEqual((self.out / 'case.stdout').read_text(), 'partial out')
        self.assertEqual((self.out / 'case.stderr').read_text(), '')

    def test_nonzero_exit_retains_both_streams(self):
        result = subprocess.CompletedProcess(['probe'], 7, 'out', 'err')
        with patch.object(PROBE.subprocess, 'run', return_value=result):
            with self.assertRaises(RuntimeError):
                self.capture()
        self.assertEqual(self.state()['returncode'], 7)
        self.assertEqual((self.out / 'case.stdout').read_text(), 'out')
        self.assertEqual((self.out / 'case.stderr').read_text(), 'err')

    def test_success_keeps_diagnostics_and_execution_options(self):
        result = subprocess.CompletedProcess(['probe'], 0, 'out', 'diagnostic')
        with patch.object(PROBE.subprocess, 'run', return_value=result) as run:
            self.assertEqual(self.capture(), 'out')
        run.assert_called_once_with(['probe'], cwd=self.out, env={}, input='xml', capture_output=True, text=True, timeout=3)
        self.assertEqual(self.state()['returncode'], 0)
        self.assertEqual((self.out / 'case.stderr').read_text(), 'diagnostic')

class WorkflowTests(TemporaryCase):
    def args(self):
        from types import SimpleNamespace
        return SimpleNamespace(kind='reset', engine='tpe', sim_build=self.out / 'build',
                               dependency_prefix=self.out / 'prefix', upstream_root=self.out / 'native',
                               headless_reference=PROBE.ROOT / 'planning/reference/headless-imu.json',
                               schedule_reference=PROBE.ROOT / 'planning/reference/sensor-schedule.json')

    def test_relocated_sensor_reference_key_preserves_role_identity(self):
        digest = 'known-sensor-hash'
        for path in ['/original/private/build/lib/libgz-sensors.so.11', '/relocated/prefix/lib/libgz-sensors.so.11.0']:
            self.assertEqual(PROBE.library_hash({path: digest}, 'libgz-sensors.so.11'), digest)
        for entries in [{}, {'/a/libgz-sensors.so.11': digest, '/b/libgz-sensors.so.11': digest}]:
            with self.assertRaises(ValueError):
                PROBE.library_hash(entries, 'libgz-sensors.so.11')

    def test_preflight_git_error_keeps_stage_partial_inputs_and_diagnostics(self):
        failure = subprocess.CalledProcessError(128, ['git'], output='partial git', stderr='not a repository')
        with patch.object(PROBE.subprocess, 'check_output', side_effect=failure):
            with self.assertRaises(subprocess.CalledProcessError):
                PROBE.execute(self.args(), self.out)
        state = json.loads((self.out / 'workflow.json').read_text())
        self.assertEqual(state['state'], 'failed')
        self.assertEqual(state['stage'], 'source_inputs')
        self.assertEqual(state['error_stderr'], 'not a repository')
        self.assertGreaterEqual(len(state['inputs_sha256']), 6)
        self.assertEqual(state['completed_modes'], [])

    def test_post_native_validation_error_keeps_raw_observation(self):
        def invalid(args, out, state):
            state.update(stage='validate:all', active_mode='all')
            state['inputs_sha256']['fixture'] = 'retained'
            (out / 'all.stdout').write_bytes(b'{partial native observation')
            raise ValueError('invalid native observation')
        with patch.object(PROBE, 'run', side_effect=invalid):
            with self.assertRaisesRegex(ValueError, 'invalid native'):
                PROBE.execute(self.args(), self.out)
        state = json.loads((self.out / 'workflow.json').read_text())
        self.assertEqual(state['stage'], 'validate:all')
        self.assertEqual(state['inputs_sha256'], {'fixture': 'retained'})
        self.assertEqual(state['raw_sha256']['all.stdout'], PROBE.sha(self.out / 'all.stdout'))
        self.assertEqual((self.out / 'all.stdout').read_bytes(), b'{partial native observation')


if __name__ == '__main__':
    unittest.main()
