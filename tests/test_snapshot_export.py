# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import importlib.util
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

VERIFIER = Path(__file__).resolve().parents[1] / 'scripts/verify.py'


class SnapshotExport(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='gz-sim-export-test-')
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.root = self.directory / 'source'
        self.root.mkdir()
        self.env = os.environ.copy()
        for key in subprocess.check_output(['git', 'rev-parse', '--local-env-vars'], text=True).splitlines():
            self.env.pop(key, None)
        self.env.update(GIT_CONFIG_GLOBAL=os.devnull, GIT_CONFIG_NOSYSTEM='1')
        self.git('init', '-q')
        spec = importlib.util.spec_from_file_location('snapshot_verify', VERIFIER)
        self.verify = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.verify)
        self.verify.ROOT = self.root
        (self.root / 'data').mkdir()
        (self.root / 'data/payload').write_text('staged content\n')
        (self.root / 'executable').write_text('#!/bin/sh\nexit 0\n')
        (self.root / 'executable').chmod(0o755)
        (self.root / 'link').symlink_to('data/payload')
        self.output = self.directory / 'export'

    def git(self, *args):
        return subprocess.check_output(['git', '-C', str(self.root), *args], env=self.env, text=True).strip()

    def stage(self):
        self.git('add', '.')
        return self.git('write-tree')

    def export(self, tree):
        with patch.dict(os.environ, self.env, clear=True):
            self.verify.export_snapshot(tree, self.output)

    def validate(self, tree, message):
        with patch.dict(os.environ, self.env, clear=True), self.assertRaisesRegex(ValueError, message):
            self.verify.validate_snapshot(tree, self.output)

    def test_raw_roundtrip_and_fail_closed_mutations(self):
        (self.root / '.gitattributes').write_text('data/payload export-ignore\n')
        tree = self.stage()
        self.export(tree)
        payload = self.output / 'data/payload'
        self.assertEqual(payload.read_text(), 'staged content\n')
        self.assertTrue((self.output / 'executable').stat().st_mode & 0o100)
        self.assertEqual(os.readlink(self.output / 'link'), 'data/payload')
        payload.write_text('different content\n')
        self.validate(tree, 'blob differs')
        payload.write_text('staged content\n')
        executable = self.output / 'executable'
        executable.chmod(0o644)
        self.validate(tree, 'executable mode differs')
        executable.chmod(0o755)
        link = self.output / 'link'
        link.unlink()
        link.write_text('data/payload')
        self.validate(tree, 'file type differs')
        link.unlink()
        link.symlink_to('wrong-target')
        self.validate(tree, 'blob differs')
        link.unlink()
        link.symlink_to('data/payload')
        (self.output / 'extra-empty-directory').mkdir()
        self.validate(tree, 'paths differ')
        (self.output / 'extra-empty-directory').rmdir()
        (self.output / 'unexpected-file').write_text('extra')
        self.validate(tree, 'paths differ')

    def test_checkout_filter_cannot_change_verified_source(self):
        command = shlex.quote(sys.executable) + ' -c ' + shlex.quote(
            'import sys; sys.stdout.write(sys.stdin.read().upper())')
        self.git('config', 'filter.rewrite.smudge', command)
        clean = shlex.quote(sys.executable) + ' -c ' + shlex.quote(
            'import sys; sys.stdout.write(sys.stdin.read())')
        self.git('config', 'filter.rewrite.clean', clean)
        self.git('config', 'filter.rewrite.required', 'true')
        (self.root / '.gitattributes').write_text('data/payload filter=rewrite\n')
        tree = self.stage()
        with self.assertRaisesRegex(ValueError, 'blob differs'):
            self.export(tree)
        self.assertEqual((self.output / 'data/payload').read_text(), 'STAGED CONTENT\n')
        self.assertEqual(self.git('write-tree'), tree)
        self.assertEqual((self.root / 'data/payload').read_text(), 'staged content\n')


if __name__ == '__main__':
    unittest.main()
