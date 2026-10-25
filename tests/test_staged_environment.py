# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
import unittest

VERIFIER = Path(__file__).resolve().parents[1] / 'scripts/verify.py'


class StagedEnvironment(unittest.TestCase):
    def test_worktree_hook_preserves_alternate_index_and_isolates_child_git(self):
        with tempfile.TemporaryDirectory(prefix='gz-sim-hook-') as directory:
            root = Path(directory)
            env = os.environ.copy()
            keys = subprocess.check_output(['git', 'rev-parse', '--local-env-vars'], text=True).splitlines()
            for key in keys:
                env.pop(key, None)
            global_config = root / 'global-config'
            global_config.write_text('[user]\nname = Fixture User\nemail = fixture@example.invalid\n')
            env.update(GIT_CONFIG_GLOBAL=str(global_config), GIT_CONFIG_NOSYSTEM='1')

            def git(cwd, *args, extra=None):
                result = subprocess.run(['git', '-C', str(cwd), *args], env=env | (extra or {}),
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                return result.stdout.strip()

            repository = root / 'main'
            repository.mkdir()
            git(repository, 'init', '-q')
            git(repository, 'config', 'commit.gpgsign', 'false')
            (repository / 'scripts').mkdir()
            probe = repository / 'scripts/verify.py'
            probe.write_text('''import json, os, pathlib, subprocess
report = pathlib.Path(os.environ['GZ_ENV_REPORT'])
assert pathlib.Path('staged.txt').read_text() == 'intended staged bytes\\n'
assert pathlib.Path('changed-lines.json').read_text() == 'owned source manifest'
keys = subprocess.check_output(['git', 'rev-parse', '--local-env-vars'], text=True).splitlines()
assert not any(key in os.environ for key in keys), sorted(key for key in keys if key in os.environ)
assert os.environ['GZ_REFERENCE_PREFIX'] == 'preserved-reference'
assert os.environ['GZ_COVERAGE_BASE'] == 'a' * 40
assert os.environ['GIT_CONFIG_GLOBAL'] == os.environ['GZ_EXPECT_GLOBAL']
fixtures = report.parent / 'child-fixtures'
fixtures.mkdir()
for name, bare in [('normal', False), ('bare', True)]:
    command = ['git', 'init', '-q'] + (['--bare'] if bare else []) + [str(fixtures / name)]
    subprocess.run(command, check=True)
    actual = subprocess.check_output(['git', '-C', str(fixtures / name), 'rev-parse', '--is-bare-repository'], text=True).strip()
    assert actual == str(bare).lower()
report.write_text(json.dumps({'isolated': True}))
''')
            git(repository, 'add', 'scripts/verify.py')
            git(repository, 'commit', '-qm', 'initial fixture')
            parent = git(repository, 'rev-parse', 'HEAD')
            worktree = root / 'worktree with spaces'
            git(repository, 'worktree', 'add', '-qb', 'candidate', str(worktree))
            ordinary_index = Path(git(worktree, 'rev-parse', '--path-format=absolute', '--git-path', 'index'))
            ordinary_before = ordinary_index.read_bytes()
            alternate = root / 'alternate-index'
            alternate_env = {'GIT_INDEX_FILE': str(alternate)}
            git(worktree, 'read-tree', 'HEAD', extra=alternate_env)
            (worktree / 'staged.txt').write_text('intended staged bytes\n')
            (worktree / '.gitattributes').write_text('staged.txt export-ignore\n')
            (worktree / 'changed-lines.json').write_text('owned source manifest')
            git(worktree, 'add', 'staged.txt', '.gitattributes', 'changed-lines.json', extra=alternate_env)
            intended = git(worktree, 'write-tree', extra=alternate_env)
            (worktree / 'staged.txt').write_text('unstaged bytes\n')
            hooks = root / 'hooks'
            hooks.mkdir()
            hook = hooks / 'pre-commit'
            code = f'''import importlib.util, os, pathlib
assert pathlib.Path(os.environ['GIT_DIR']).is_absolute()
spec = importlib.util.spec_from_file_location('actual_verify', {str(VERIFIER)!r})
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.ROOT = pathlib.Path.cwd()
module.staged()
'''
            hook.write_text('#!/bin/sh\nexec ' + shlex.quote(sys.executable) + ' -c ' + shlex.quote(code) + '\n')
            hook.chmod(0o755)
            git(worktree, 'config', 'core.hooksPath', str(hooks))
            report = root / 'report.json'
            git(worktree, 'commit', '-qm', 'intended alternate index', extra=alternate_env | {
                'GZ_ENV_REPORT': str(report), 'GZ_EXPECT_GLOBAL': str(global_config),
                'GZ_REFERENCE_PREFIX': 'preserved-reference', 'GZ_COVERAGE_BASE': 'a' * 40})
            self.assertEqual(json.loads(report.read_text()), {'isolated': True})
            self.assertEqual(git(worktree, 'rev-parse', 'HEAD^'), parent)
            self.assertEqual(git(worktree, 'rev-parse', 'HEAD^{tree}'), intended)
            self.assertEqual(git(repository, 'rev-parse', 'HEAD'), parent)
            self.assertEqual(git(repository, 'config', 'core.bare'), 'false')
            self.assertEqual(ordinary_index.read_bytes(), ordinary_before)
            self.assertEqual((worktree / 'staged.txt').read_text(), 'unstaged bytes\n')


if __name__ == '__main__':
    unittest.main()
