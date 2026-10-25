# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import hashlib
import io
from contextlib import nullcontext, redirect_stderr
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

VERIFIER = Path(__file__).resolve().parents[1] / 'scripts/verify.py'


class StagedEvidence(unittest.TestCase):
    def check_run(self, mode, state, retention_failure=False):
        with tempfile.TemporaryDirectory(prefix='gz-sim-stage-evidence-') as temporary:
            root = Path(temporary)
            env = os.environ.copy()
            for key in subprocess.check_output(['git', 'rev-parse', '--local-env-vars'], text=True).splitlines():
                env.pop(key, None)
            env.update(GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM='1',
                       GZ_STAGE_REVIEW_MODE=mode, GZ_STAGE_REVIEW_PARENT=str(root))
            def git(*args):
                return subprocess.check_output(['git', '-C', str(root), *args], env=env, text=True).strip()
            git('init', '-q')
            (root / 'scripts').mkdir()
            (root / 'scripts/verify.py').write_text('''import os, pathlib, subprocess, sys
out = pathlib.Path('artifacts/coverage')
out.mkdir(parents=True)
(out / 'test-output.log').write_text('exact child result\\n')
(out / 'evidence.json').write_text('{"qualified": false}\\n')
if os.environ['GZ_STAGE_REVIEW_MODE'] != 'missing':
    (out / 'summary.json').write_text('{}\\n')
mode = os.environ['GZ_STAGE_REVIEW_MODE']
if mode == 'index':
    parent = pathlib.Path(os.environ['GZ_STAGE_REVIEW_PARENT'])
    (parent / 'late').write_text('index changed during child checks')
    subprocess.run(['git', '-C', str(parent), 'add', 'late'], check=True)
sys.exit(7 if mode == 'failure' else 0)
''')
            git('add', 'scripts/verify.py')
            tree = git('write-tree')
            spec = importlib.util.spec_from_file_location('review_verifier', VERIFIER)
            verify = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(verify)
            verify.ROOT = root
            errors = io.StringIO()
            retainer = patch.object(verify, 'retain_staged_evidence', side_effect=OSError('retention IO failure')) if retention_failure else nullcontext()
            with patch.dict(os.environ, env, clear=True), retainer, redirect_stderr(errors):
                if mode == 'failure':
                    with self.assertRaises(subprocess.CalledProcessError) as error:
                        verify.staged()
                    self.assertEqual(error.exception.returncode, 7)
                elif mode == 'index':
                    with self.assertRaisesRegex(ValueError, 'index changed during verification'):
                        verify.staged()
                elif retention_failure:
                    with self.assertRaisesRegex(OSError, 'retention IO failure'):
                        verify.staged()
                else:
                    verify.staged()
            if retention_failure:
                if mode != 'success':
                    self.assertIn('Could not retain staged evidence: retention IO failure', errors.getvalue())
                return
            retained = list((root / 'artifacts/staged').iterdir())
            self.assertEqual(len(retained), 1)
            output = retained[0]
            report = json.loads((output / 'stage-result.json').read_text())
            self.assertEqual(report['tree'], tree)
            self.assertEqual(report['checks_state'], state)
            self.assertEqual(report['evidence_state'], 'partial' if mode == 'missing' else 'complete')
            self.assertEqual(report['missing_artifacts'], ['summary.json'] if mode == 'missing' else [])
            self.assertEqual(set(report['artifacts_sha256']), {'test-output.log', 'evidence.json'} | (set() if mode == 'missing' else {'summary.json'}))
            for name, digest in report['artifacts_sha256'].items():
                self.assertEqual(hashlib.sha256((output / name).read_bytes()).hexdigest(), digest)
            self.assertEqual((output / 'test-output.log').read_text(), 'exact child result\n')
            self.assertFalse((root / 'artifacts/coverage').exists())
            self.assertEqual(git('write-tree') == tree, mode != 'index')

    def test_success_retains_exact_outputs_after_export_cleanup(self):
        self.check_run('success', 'passed')

    def test_child_failure_is_retained_without_becoming_a_pass(self):
        self.check_run('failure', 'checks_failed')

    def test_changed_index_still_fails_and_is_recorded(self):
        self.check_run('index', 'index_changed')


    def test_missing_output_is_partial_even_when_checks_succeed(self):
        self.check_run('missing', 'passed')

    def test_retention_failure_does_not_hide_failed_child(self):
        self.check_run('failure', 'checks_failed', retention_failure=True)

    def test_retention_failure_does_not_hide_changed_index(self):
        self.check_run('index', 'index_changed', retention_failure=True)

    def test_retention_failure_alone_still_fails(self):
        self.check_run('success', 'passed', retention_failure=True)

    def test_artifact_directory_symlink_cannot_copy_files_outside_export(self):
        with tempfile.TemporaryDirectory(prefix='gz-sim-stage-link-') as temporary:
            root = Path(temporary)
            candidate = root / 'candidate'
            (candidate / 'artifacts').mkdir(parents=True)
            outside = root / 'outside'
            outside.mkdir()
            (outside / 'test-output.log').write_text('outside data')
            (candidate / 'artifacts/coverage').symlink_to(outside, target_is_directory=True)
            spec = importlib.util.spec_from_file_location('review_verifier_link', VERIFIER)
            verify = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(verify)
            verify.ROOT = root
            verify.retain_staged_evidence('a' * 40, candidate, 'checks_failed')
            output = next((root / 'artifacts/staged').iterdir())
            report = json.loads((output / 'stage-result.json').read_text())
            self.assertEqual(report['artifacts_sha256'], {})
            self.assertEqual(report['missing_artifacts'], ['test-output.log', 'evidence.json', 'summary.json'])
            self.assertEqual(list(output.iterdir()), [output / 'stage-result.json'])

if __name__ == '__main__':
    unittest.main()
