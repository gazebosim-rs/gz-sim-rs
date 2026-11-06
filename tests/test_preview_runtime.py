# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import hashlib
import http.client
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/preview_runtime.py'
spec = importlib.util.spec_from_file_location('preview_runtime', SCRIPT)
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


class InstalledPreview(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='gz-preview-test-')
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.prefix = self.directory / 'installed prefix'
        self.prefix.mkdir()
        names = ['bin/gz-sim-rs', 'bin/gz-drive-command', 'share/robot.sdf',
                 'share/zenoh.json5', 'web/index.html', 'web/startup.mjs']
        names += ['web/target/wasm32-unknown-unknown/release/' + flavor + '/' + name
                  for flavor in ('webgpu', 'webgl2') for name in ('gzweb.js', 'gzweb_bg.wasm')]
        names += ['bin/gz-preview', 'LICENSE', 'NOTICE', 'M12_INSTALL.md', 'M12_CAPABILITIES.md',
                  'notices.json', *['notices/' + name for name in ('licenses/Apache-2.0.txt',
                  'OSRF-NOTICE.txt', 'Geist-OFL.txt', 'Rust-COPYRIGHT.html', 'Rust-LICENSE-APACHE.txt',
                  'Rust-LICENSE-MIT.txt', 'Viewer-mesh-loader-LICENSE.txt', 'Viewer-proto-LICENSE.txt')]]
        self.files = {}
        for name in names:
            path = self.prefix / name
            path.parent.mkdir(parents=True, exist_ok=True)
            content = ('fixture ' + name).encode()
            path.write_bytes(content)
            path.chmod(0o755 if name.startswith('bin/') else 0o644)
            self.files[name] = {'bytes': len(content), 'sha256': hashlib.sha256(content).hexdigest(),
                                'mode': 0o755 if name.startswith('bin/') else 0o644}
        notice_rows = [{'path': name.removeprefix('notices/'), 'sha256': entry['sha256'],
                        'bytes': entry['bytes']} for name, entry in self.files.items() if name.startswith('notices/')]
        content = json.dumps({'version': 1, 'files': notice_rows}).encode()
        (self.prefix / 'notices.json').write_bytes(content)
        self.files['notices.json'].update(bytes=len(content), sha256=hashlib.sha256(content).hexdigest())
        self.save_manifest()

    def save_manifest(self):
        (self.prefix / 'manifest.json').write_text(json.dumps({'schema_version': 1, 'files': self.files}))

    def test_relocated_prefix_has_no_development_directory_dependency(self):
        moved = self.directory / 'another prefix'
        shutil.move(self.prefix, moved)
        self.assertEqual(preview.verify_prefix(moved)['files'], self.files)
        with patch.dict(os.environ, {'ZENOH_CONFIG': '/wrong', 'GZ_PARTITION': 'other'}):
            env = preview.environment(moved, 'installed-demo')
        self.assertEqual(env['ZENOH_CONFIG'], str(moved / 'share/zenoh.json5'))
        self.assertEqual(env['GZ_PARTITION'], 'installed-demo')

    def test_missing_changed_linked_and_executable_mode_drift_are_refused(self):
        selected = self.prefix / 'bin/gz-sim-rs'
        original = selected.read_bytes()
        selected.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'differs'):
            preview.verify_prefix(self.prefix)
        selected.write_bytes(original)
        selected.chmod(0o644)
        with self.assertRaisesRegex(ValueError, 'differs'):
            preview.verify_prefix(self.prefix)
        selected.unlink()
        external = self.directory / 'outside'
        external.write_bytes(original)
        selected.symlink_to(external)
        with self.assertRaisesRegex(ValueError, 'linked'):
            preview.verify_prefix(self.prefix)
        selected.unlink()
        with self.assertRaisesRegex(ValueError, 'inventory'):
            preview.verify_prefix(self.prefix)

    def test_manifest_paths_and_omitted_backend_are_refused(self):
        self.files['../outside'] = dict(next(iter(self.files.values())))
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'manifest path'):
            preview.verify_prefix(self.prefix)
        del self.files['../outside']
        del self.files['web/target/wasm32-unknown-unknown/release/webgl2/gzweb_bg.wasm']
        self.save_manifest()
        with self.assertRaisesRegex(ValueError, 'omits'):
            preview.verify_prefix(self.prefix)

    def test_reduced_manifests_unlisted_files_and_escaping_web_links_are_refused(self):
        for required in ('bin/gz-preview', 'LICENSE', 'NOTICE', 'notices.json', 'M12_INSTALL.md'):
            entry = self.files.pop(required)
            self.save_manifest()
            with self.assertRaisesRegex(ValueError, 'omits'):
                preview.verify_prefix(self.prefix)
            self.files[required] = entry
        self.save_manifest()
        unexpected = self.prefix / 'web/unlisted'
        unexpected.write_text('unexpected')
        with self.assertRaisesRegex(ValueError, 'inventory'):
            preview.verify_prefix(self.prefix)
        unexpected.unlink()
        external = self.directory / 'private'
        external.write_text('must not be served')
        unexpected.symlink_to(external)
        with self.assertRaisesRegex(ValueError, 'linked'):
            preview.verify_prefix(self.prefix)

    def test_child_exit_signal_race_still_waits_for_reaping(self):
        child = Mock(pid=123)
        child.poll.return_value = None
        with patch.object(preview.os, 'killpg', side_effect=ProcessLookupError):
            preview.stop_child(child)
        child.wait.assert_called_once_with(timeout=10)

    def test_occupied_port_owner_is_untouched_and_no_child_is_started(self):
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            listener.listen()
            port = listener.getsockname()[1]
            with patch.object(preview.subprocess, 'Popen') as spawn:
                with self.assertRaises(OSError):
                    preview.serve(self.prefix, 8081, port, 'demo')
                spawn.assert_not_called()
            with socket.create_connection(('127.0.0.1', port), timeout=1):
                pass

    def test_server_serves_only_web_and_reaps_on_simulator_exit(self):
        with socket.socket() as candidate:
            candidate.bind(('127.0.0.1', 0))
            port = candidate.getsockname()[1]
        child = Mock(pid=123, returncode=0)
        child.poll.return_value = None
        observations = []

        def browser():
            try:
                for _ in range(100):
                    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=1)
                    try:
                        connection.request('GET', '/')
                        response = connection.getresponse()
                        observations.append((response.status, response.read()))
                        connection.close()
                        break
                    except OSError:
                        connection.close()
                        time.sleep(0.01)
                connection = http.client.HTTPConnection('127.0.0.1', port, timeout=1)
                connection.request('GET', '/../manifest.json')
                response = connection.getresponse()
                observations.append((response.status, response.read()))
                connection.close()
            finally:
                child.poll.return_value = 0

        client = threading.Thread(target=browser)
        with patch.object(preview, 'check_port'), patch.object(preview.subprocess, 'Popen', return_value=child) as spawn:
            with patch.object(preview.os, 'killpg') as kill:
                client.start()
                self.assertEqual(preview.serve(self.prefix, port, 9014, 'demo'), 0)
                client.join(timeout=5)
                self.assertFalse(client.is_alive())
                kill.assert_not_called()
        self.assertEqual(observations[0], (200, (self.prefix / 'web/index.html').read_bytes()))
        self.assertEqual(observations[1][0], 404)
        args, kwargs = spawn.call_args
        self.assertEqual(args[0][-3:], ['--server', '--web-viewer', '9014'])
        self.assertTrue(kwargs['start_new_session'])
        self.assertEqual(kwargs['env']['GZ_PARTITION'], 'demo')
        with socket.socket() as probe:
            self.assertNotEqual(probe.connect_ex(('127.0.0.1', port)), 0)

    def test_finite_drive_uses_installed_helper_and_same_local_partition(self):
        with patch.object(preview.os, 'execve', return_value=7) as send:
            self.assertEqual(preview.main(['--partition', 'demo', 'drive', '0.4', '1', '3'], self.prefix), 7)
        args, kwargs = send.call_args
        self.assertEqual(args[0], str(self.prefix / 'bin/gz-drive-command'))
        self.assertEqual(args[1], [args[0], 'tcp/127.0.0.1:7449', '0.4', '1', '3'])
        self.assertEqual(args[2]['GZ_PARTITION'], 'demo')
        self.assertEqual(args[2]['ZENOH_CONFIG'], str(self.prefix / 'share/zenoh.json5'))
        self.assertEqual(kwargs, {})

    def test_shutdown_reaps_only_its_owned_process_group(self):
        child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'],
                                 start_new_session=True, stderr=subprocess.DEVNULL)
        def reap():
            if child.poll() is None:
                child.kill()
            child.wait(timeout=5)

        self.addCleanup(reap)
        preview.stop_child(child)
        self.assertIsNotNone(child.poll())
        with patch.object(preview.os, 'killpg') as kill:
            preview.stop_child(child)
            kill.assert_not_called()

    def test_unresponsive_child_is_escalated_and_invalid_ports_launch_nothing(self):
        child = Mock(pid=123)
        child.poll.return_value = None
        child.wait.side_effect = [subprocess.TimeoutExpired('owned-child', 10), 0]
        with patch.object(preview.os, 'killpg') as kill:
            preview.stop_child(child)
        self.assertEqual([call.args for call in kill.call_args_list],
                         [(123, signal.SIGINT), (123, signal.SIGKILL)])
        with patch.object(preview, 'serve') as serve:
            self.assertEqual(preview.main(['run', '--ws-port', '70000'], self.prefix), 1)
            serve.assert_not_called()


if __name__ == '__main__':
    unittest.main()
