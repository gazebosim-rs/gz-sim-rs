#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Replay existing CI worker tests for diagnosis, never release acceptance."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import stat
import subprocess
import time
import tomllib

ROOT = Path(__file__).resolve().parents[1]
REF = 'refs/heads/diagnostic/worker-exit-state'
CASE = 'disconnected_output_reports_failure_and_worker_exits'
ATTEMPT_SECONDS = 60
TOTAL_SECONDS = 600
TERM_SECONDS = 2
KILL_SECONDS = 2
PLAN = [None] * 5 + [20] * 5


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def regular(root, path):
    path = Path(path)
    if not path.is_absolute():
        path = root / path
    if '..' in path.parts or path.resolve() != path or not path.is_relative_to(root):
        raise ValueError(f'noncanonical or escaping path: {path}')
    if not stat.S_ISREG(path.stat().st_mode):
        raise ValueError(f'not a regular file: {path}')
    return path


def read_json(root, path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f'duplicate JSON key: {key}')
            result[key] = value
        return result
    return json.loads(regular(root, path).read_text(), object_pairs_hook=unique)


def context(root, env):
    if (env.get('GITHUB_ACTIONS') != 'true' or env.get('GITHUB_REF') != REF
            or env.get('GITHUB_EVENT_NAME') != 'push'
            or env.get('GITHUB_REPOSITORY') != 'gazebosim-rs/gz-sim-rs'
            or not env.get('GITHUB_WORKSPACE')
            or Path(env['GITHUB_WORKSPACE']).resolve() != root):
        raise ValueError('requires the isolated diagnostic push workflow')
    head = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True).strip()
    if env.get('GITHUB_SHA') != head:
        raise ValueError('workflow SHA differs from checkout')
    subprocess.run(['git', '-C', str(root), 'diff', '--quiet', 'HEAD', '--'], check=True)
    if env.get('GZ_RENDER_REQUIRE_LLVMPIPE') != '1' or not env.get('VK_ICD_FILENAMES'):
        raise ValueError('software Vulkan selection is missing')
    return head


def bind(root, env):
    head = context(root, env)
    coverage = root / 'artifacts/coverage'
    index_path = coverage / 'profile-index.json'
    index = read_json(root, index_path)
    if index['profiles'] != ['headless', 'renderer'] or not re.fullmatch(r'profiles-[a-z0-9_]+', index['run']):
        raise ValueError('missing normal combined profile')
    for section in ['sources', 'inputs']:
        if not index[section]:
            raise ValueError('empty normal input binding')
        for relative, expected in index[section].items():
            if Path(relative).is_absolute() or str(Path(relative)) != relative:
                raise ValueError('input must be repository relative')
            path = regular(root, relative)
            committed = subprocess.check_output(['git', '-C', str(root), 'show', f'{head}:{relative}'])
            if digest(path) != expected or hashlib.sha256(committed).hexdigest() != expected:
                raise ValueError(f'stale normal source/input: {relative}')
    directory = coverage / index['run'] / 'renderer'
    evidence_path = directory / 'evidence.json'
    evidence = read_json(root, evidence_path)
    log_path = regular(root, directory / 'test-output.log')
    log = log_path.read_text()
    if evidence['profile'] != 'renderer' or evidence['rustc'] != index['rustc'] or not suite_passed(log):
        raise ValueError('normal renderer evidence is incomplete')
    main = 'crates/gz-render-worker/src/main.rs'
    if main not in evidence['sources'] or any(index['sources'].get(path) != sha
                                              for path, sha in evidence['sources'].items()):
        raise ValueError('renderer evidence has stale source bindings')
    version = tomllib.loads((root / 'Cargo.toml').read_text())['workspace']['package']['version']
    package = f'path+file://{root}/crates/gz-render-worker#{version}'
    selected = {}
    for name, kind, is_test, source in [
            ('worker', 'bin', False, 'src/main.rs'),
            ('tests', 'test', True, 'tests/direct_worker.rs')]:
        target_name = 'gz-render-worker' if name == 'worker' else 'direct_worker'
        matches = [obj for obj in evidence['objects']
                   if obj['target']['name'] == target_name and obj['target']['kind'] == [kind]
                   and obj['profile']['test'] is is_test]
        if len(matches) != 1:
            raise ValueError(f'ambiguous or missing {name} executable')
        obj = matches[0]
        source_path = root / 'crates/gz-render-worker' / source
        if obj['package_id'] != package or Path(obj['target']['src_path']) != source_path:
            raise ValueError('executable source/package differs')
        path = regular(root, obj['path'])
        expected_dir = root / 'target/coverage/debug'
        if ((name == 'worker' and path != expected_dir / 'gz-render-worker')
                or (name == 'tests' and (path.parent != expected_dir / 'deps'
                                        or not re.fullmatch(r'direct_worker-[0-9a-f]+', path.name)))):
            raise ValueError('executable is outside the normal Cargo target')
        with path.open('rb') as stream:
            elf = stream.read(4) == b'\x7fELF'
        if not elf or not os.access(path, os.X_OK) or digest(path) != obj['sha256']:
            raise ValueError('normal executable is missing or mutated')
        cargo = []
        for line in log.splitlines():
            if not line.startswith('{"reason":'):
                continue
            record = json.loads(line)
            if record.get('reason') == 'compiler-artifact' and record.get('executable') == str(path):
                cargo.append(record)
        if (len(cargo) != 1 or cargo[0]['target'] != obj['target']
                or cargo[0]['package_id'] != package or cargo[0]['profile'] != obj['profile']):
            raise ValueError('executable lacks unique normal Cargo evidence')
        # Rust test sources are not profile-index inputs. Refuse cached objects
        # rather than infer their source identity: this workflow has no Cargo
        # target cache, and ordinary verification freshly compiles both targets.
        # The diagnostic never invokes Cargo to repair a refused binding.
        if (cargo[0].get('fresh') is not False
                or not index_path.stat().st_mtime_ns <= path.stat().st_mtime_ns <= evidence_path.stat().st_mtime_ns
                or source_path.stat().st_mtime_ns > path.stat().st_mtime_ns):
            raise ValueError('executable is not freshly compiled in the normal profile')
        regular(root, source_path)
        source_bytes = subprocess.check_output(['git', '-C', str(root), 'show', f'{head}:{source_path.relative_to(root)}'])
        if source_path.read_bytes() != source_bytes:
            raise ValueError('test or worker source changed')
        selected[name] = {'path': str(path), 'sha256': obj['sha256'],
                          'source_sha256': hashlib.sha256(source_bytes).hexdigest(),
                          'normal_cargo_fresh': False}
    return {'commit': head, 'ref': REF, 'executables': selected,
            'normal_records': {str(p.relative_to(root)): digest(p)
                               for p in [index_path, evidence_path, log_path]}}


def suite_passed(stdout):
    return (f'test {CASE} ... ok' in stdout
            and re.search(r'^test result: ok\. 20 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out;',
                          stdout, re.MULTILINE) is not None)


def unchanged(root, binding):
    for obj in binding['executables'].values():
        if digest(regular(root, obj['path'])) != obj['sha256']:
            raise ValueError('executable changed after normal verification')
    for path, expected in binding['normal_records'].items():
        if digest(regular(root, path)) != expected:
            raise ValueError('normal verification record changed')


def classification(returncode, timed_out, stdout, stderr):
    if timed_out:
        return 'outer_timeout'
    if returncode == 0 and suite_passed(stdout) and 'LLVM Profile Error' not in stderr:
        return 'passed'
    original = re.search(r"thread '" + CASE + r"'[^\n]* panicked at[^\n]*\nworker shutdown timed out(?:\n|$)", stderr)
    if returncode == 101 and original:
        return 'original_target_timeout'
    return 'other_failure'


def execute(root, command, directory, env, deadline):
    # The 60-second outer bound includes up to four seconds for TERM/KILL/reap.
    started = time.monotonic()
    hard_end = min(started + ATTEMPT_SECONDS, deadline)
    allowance = hard_end - started - TERM_SECONDS - KILL_SECONDS
    record = {'command': command, 'outer_bound_seconds': ATTEMPT_SECONDS,
              'timed_out': False, 'leader_reaped': False, 'signals': []}
    if allowance <= 0:
        return record | {'classification': 'total_bound_exhausted', 'passed': False}
    with (directory / 'stdout.log').open('wb') as stdout, (directory / 'stderr.log').open('wb') as stderr:
        child = subprocess.Popen(command, cwd=root, env=env, stdout=stdout, stderr=stderr,
                                 start_new_session=True)
        record['pid'] = child.pid
        try:
            child.wait(timeout=max(0, hard_end - time.monotonic() - TERM_SECONDS - KILL_SECONDS))
        except subprocess.TimeoutExpired:
            record['timed_out'] = True
            for sig, seconds in [(signal.SIGTERM, TERM_SECONDS), (signal.SIGKILL, KILL_SECONDS)]:
                try:
                    os.killpg(child.pid, sig)
                    record['signals'].append(sig.name)
                except ProcessLookupError:
                    pass
                try:
                    child.wait(timeout=min(seconds, max(0, hard_end - time.monotonic())))
                    if sig == signal.SIGKILL:
                        break
                except subprocess.TimeoutExpired:
                    pass
            if child.poll() is None:
                return record | {'classification': 'unreaped_child', 'passed': False}
        record['returncode'] = child.returncode
        record['leader_reaped'] = True
    stdout = (directory / 'stdout.log').read_text(errors='replace')
    stderr = (directory / 'stderr.log').read_text(errors='replace')
    category = classification(child.returncode, record['timed_out'], stdout, stderr)
    return record | {'classification': category, 'passed': category == 'passed',
                     'elapsed_seconds_for_bound': time.monotonic() - started}


def batch(root, env):
    context(root, env)  # Refuse non-diagnostic execution before creating output.
    output = root / 'artifacts/worker-exit-stress'
    if output.resolve() != output or output.exists() or output.is_symlink():
        raise ValueError('diagnostic output already exists')
    output.mkdir(parents=True)
    try:
        binding = bind(root, env)
        (output / 'binding.json').write_text(json.dumps(binding, indent=2) + '\n')
        deadline = time.monotonic() + TOTAL_SECONDS
        attempts = []
        refused = None
        for number, threads in enumerate(PLAN, 1):
            directory = output / f'attempt-{number:02}'
            directory.mkdir()
            raw = directory / 'raw'
            raw.mkdir()
            for name in ['stdout.log', 'stderr.log']:
                (directory / name).touch()
            local = env.copy()
            for key in ['LLVM_PROFILE_FILE', 'GZ_RENDER_WORKER_PROFILE', 'RUST_TEST_THREADS',
                        'GZ_RENDER_WORKER_DIAGNOSTICS']:
                local.pop(key, None)
            local['LLVM_PROFILE_FILE'] = str(raw / '%p-%m.profraw')
            local['GZ_RENDER_WORKER_PROFILE'] = str(raw / 'worker-%p-%m.profraw')
            command = [binding['executables']['tests']['path'], '--nocapture']
            if threads is not None:
                command.append(f'--test-threads={threads}')
            request = {'number': number, 'threads': threads, 'command': command,
                       'profile_paths': {key: local[key] for key in ['LLVM_PROFILE_FILE', 'GZ_RENDER_WORKER_PROFILE']}}
            (directory / 'command.json').write_text(json.dumps(request, indent=2) + '\n')
            record = {}
            try:
                if refused:
                    raise ValueError(refused)
                unchanged(root, binding)
                record = execute(root, command, directory, local, deadline)
                if record['classification'] == 'unreaped_child':
                    refused = 'prior owned child was not reaped'
                unchanged(root, binding)
            except (ValueError, OSError) as error:
                refused = str(error)
                record = record | {'execution_classification': record.get('classification'),
                                   'classification': 'refused', 'passed': False, 'error': refused}
            record.update(number=number, threads=threads)
            record['files_sha256'] = {str(p.relative_to(directory)): digest(regular(root, p))
                                      for p in sorted(directory.rglob('*')) if p.is_file()}
            if record['passed'] and not any(raw.glob('*.profraw')):
                record.update(passed=False, classification='missing_diagnostic_profiles')
            (directory / 'result.json').write_text(json.dumps(record, indent=2) + '\n')
            attempts.append(record)
        result = {'commit': binding['commit'], 'scope': 'diagnosis only; no original failure fix or phase acceptance',
                  'attempt_seconds_including_cleanup': ATTEMPT_SECONDS, 'batch_seconds': TOTAL_SECONDS,
                  'executed_attempts': sum('pid' in x for x in attempts),
                  'attempts': attempts, 'passed': len(attempts) == 10 and all(x['passed'] for x in attempts)}
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        return result['passed']
    except Exception as error:
        (output / 'failure.json').write_text(json.dumps({'error': str(error)}, indent=2) + '\n')
        raise


if __name__ == '__main__':
    argparse.ArgumentParser(description=__doc__).parse_args()
    try:
        raise SystemExit(0 if batch(ROOT, dict(os.environ)) else 1)
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        raise SystemExit(str(error)) from error
