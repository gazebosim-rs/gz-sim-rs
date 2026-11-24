#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Archived input staging must retain the real verification and build chain."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SignedMetadataWorkflow(unittest.TestCase):
    def test_staging_precedes_unchanged_verification_build_and_validation(self):
        workflow = (ROOT / '.github/workflows/ci.yml').read_text()
        step = workflow.split('      - name: Authenticate the archived Ubuntu snapshot', 1)[1]
        step = step.split('      - uses: actions/cache@', 1)[0]
        required = [
            'python3 scripts/stage-zenoh-package-metadata.py',
            'python3 scripts/bootstrap-zenoh-native.py --verify-packages-only',
            'file:%s/repo/\\ttype:index',
            'https://archive.ubuntu.com/ubuntu/\\tpriority:1 type:deb',
            'https://snapshot.ubuntu.com/ubuntu/%s/\\tpriority:2 type:deb',
            'Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg',
            'sudo apt-get "${options[@]}" --error-on=any update',
            'sudo apt-get "${options[@]}" -y --allow-downgrades install "${packages[@]}"',
            'artifacts/check-signed-apt\n      - name:',
            'Build and validate the selected Zenoh client reference',
            "subprocess.run(['curl', '--fail', '--location', archive_url, '--output', str(archive)], check=True)",
            "if hashlib.sha256(archive.read_bytes()).hexdigest() != vendor['sha256']:",
            "subprocess.run(['python3', 'scripts/bootstrap-zenoh-native.py', '--jobs', '2', '--vendor-archive', str(archive.resolve())], check=True)",
            "subprocess.run(['python3', 'scripts/bootstrap-zenoh-native.py', '--validate-only'], check=True)",
        ]
        for command in required:
            self.assertEqual(step.count(command), 1, command)
        positions = [step.index(command) for command in required]
        self.assertEqual(positions, sorted(positions))
        self.assertNotIn('continue-on-error:', step)
        self.assertNotIn('|| true', step)
        self.assertIn('python3 scripts/verify.py || status=$?', workflow)
        self.assertIn('exit "$status"', workflow)
        self.assertIn('if-no-files-found: error', workflow)


if __name__ == '__main__':
    unittest.main()
