# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

output = Path('/out')
environment = Path('/reference/environment.json')
result = subprocess.run(['ctest', '--test-dir', '/reference/builds/gz-math', '-j2',
                         '--output-on-failure', '--output-junit', '/out/ctest-junit.xml'],
                        capture_output=True, timeout=600)
(output / 'stdout').write_bytes(result.stdout)
(output / 'stderr').write_bytes(result.stderr)
results = Path('/reference/builds/gz-math/test_results')
if results.exists():
    shutil.copytree(results, output / 'gtest-results')
(output / 'run.json').write_text(json.dumps({
    'scope': 'Actual unchanged native gz-math CTest commands; no Rust-port credit or broad REF-02 acceptance.',
    'image_id': 'sha256:11b7378c7cca2844a4b7b2c9f52525e983b19d6651d33c83375d220672462d1d',
    'environment_sha256': hashlib.sha256(environment.read_bytes()).hexdigest(),
    'command': result.args,
    'returncode': result.returncode,
    'state': 'native_run_pending_review',
}, indent=2) + '\n')
for path in output.rglob('*'):
    path.chmod(0o755 if path.is_dir() else 0o644)
raise SystemExit(result.returncode)
