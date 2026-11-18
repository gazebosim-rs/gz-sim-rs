# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import hashlib
from functools import wraps
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
from urllib.parse import parse_qs, urljoin, urlsplit

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/preview_runtime.py'
spec = importlib.util.spec_from_file_location('preview_runtime', SCRIPT)
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


def isolated_camera_environment(method):
    """Keep hosted graphics settings outside synthetic camera admission cases."""
    @wraps(method)
    def isolated(self):
        ordinary = {name: value for name, value in os.environ.items()
                    if not name.startswith(('LD_', 'VK_', 'WGPU_', 'MESA_', 'GALLIUM_'))
                    and name != 'DRI_PRIME'}
        with patch.dict(os.environ, ordinary, clear=True):
            return method(self)
    return isolated


class InstalledPreview(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='gz-preview-test-')
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.prefix = self.directory / 'installed prefix'
        self.prefix.mkdir()
        names = ['bin/gz-sim-rs', 'bin/gz-drive-command', 'bin/gz-joint-command', 'share/robot.sdf',
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

    def install_guides(self, names):
        for name in ('INSTALL.md', 'CAPABILITIES.md', 'M12_INSTALL.md', 'M12_CAPABILITIES.md'):
            self.files.pop(name, None)
            (self.prefix / name).unlink(missing_ok=True)
        for name in names:
            content = ('installed guide ' + name).encode()
            path = self.prefix / name
            path.write_bytes(content)
            path.chmod(0o644)
            self.files[name] = {'bytes': len(content), 'sha256': hashlib.sha256(content).hexdigest(),
                                'mode': 0o644}
        self.save_manifest()

    def test_new_complete_guide_pair_verifies_after_relocation_with_full_file_checks(self):
        self.install_guides(('INSTALL.md', 'CAPABILITIES.md'))
        moved = self.directory / 'refreshed relocated prefix'
        shutil.move(self.prefix, moved)
        self.assertEqual(preview.verify_prefix(moved)['files'], self.files)
        guide = moved / 'CAPABILITIES.md'
        original = guide.read_bytes()
        guide.write_bytes(b'changed guide')
        with self.assertRaisesRegex(ValueError, 'differs'):
            preview.verify_prefix(moved)
        guide.write_bytes(original)
        guide.chmod(0o600)
        with self.assertRaisesRegex(ValueError, 'differs'):
            preview.verify_prefix(moved)
        guide.chmod(0o644)
        guide.unlink()
        external = self.directory / 'external guide'
        external.write_bytes(original)
        guide.symlink_to(external)
        with self.assertRaisesRegex(ValueError, 'linked'):
            preview.verify_prefix(moved)
        guide.unlink()
        with self.assertRaisesRegex(ValueError, 'inventory'):
            preview.verify_prefix(moved)

    def test_partial_and_mixed_only_guide_pairs_cannot_satisfy_required_guides(self):
        for names in ((), ('INSTALL.md',), ('CAPABILITIES.md',),
                      ('M12_INSTALL.md',), ('M12_CAPABILITIES.md',),
                      ('INSTALL.md', 'M12_CAPABILITIES.md'),
                      ('M12_INSTALL.md', 'CAPABILITIES.md'),
                      ('INSTALL.md', 'M12_INSTALL.md'),
                      ('CAPABILITIES.md', 'M12_CAPABILITIES.md')):
            with self.subTest(guides=names):
                self.install_guides(names)
                with self.assertRaisesRegex(ValueError, 'omits'):
                    preview.verify_prefix(self.prefix)

    def test_complete_pair_remains_valid_when_other_guide_names_are_also_listed(self):
        for names in (('INSTALL.md', 'CAPABILITIES.md', 'M12_INSTALL.md'),
                      ('M12_INSTALL.md', 'M12_CAPABILITIES.md', 'CAPABILITIES.md'),
                      ('INSTALL.md', 'CAPABILITIES.md', 'M12_INSTALL.md', 'M12_CAPABILITIES.md')):
            with self.subTest(guides=names):
                self.install_guides(names)
                self.assertEqual(preview.verify_prefix(self.prefix)['files'], self.files)

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
            for lifecycle in (False, True):
                with self.subTest(lifecycle=lifecycle), patch.object(preview.subprocess, 'Popen') as spawn:
                    with self.assertRaises(OSError):
                        preview.serve(self.prefix, 8081, port, 'demo', lifecycle=lifecycle)
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

    def test_http_parser_dispatches_only_admitted_ports_without_launching_a_server(self):
        for options, expected in (([], 8081), (['--http-port', '8080'], 8080),
                                  (['--http-port', '8081'], 8081),
                                  (['--http-port', '8082'], 8082)):
            with self.subTest(options=options):
                with patch.object(preview, 'serve', return_value=0) as serve:
                    self.assertEqual(preview.main(['--partition', 'm19', 'run', *options], self.prefix), 0)
                    serve.assert_called_once_with(self.prefix, expected, 9014, 'm19')
        with patch.object(preview, 'serve') as serve:
            with self.assertRaises(SystemExit) as stopped:
                preview.main(['run', '--http-port', '8083'], self.prefix)
            self.assertEqual(stopped.exception.code, 2)
            serve.assert_not_called()

    def test_lifecycle_pairs_native_argv_with_the_printed_outer_viewer_query(self):
        for lifecycle in (False, True):
            with self.subTest(lifecycle=lifecycle):
                child = Mock(pid=123, returncode=0)
                child.poll.return_value = 0
                server = Mock()
                thread = Mock()
                with patch.object(preview, 'check_port') as check, \
                        patch.object(preview.http.server, 'ThreadingHTTPServer', return_value=server) as http, \
                        patch.object(preview.threading, 'Thread', return_value=thread), \
                        patch.object(preview.subprocess, 'Popen', return_value=child) as spawn, \
                        patch('builtins.print') as output:
                    self.assertEqual(preview.serve(self.prefix, 8082, 9014, 'opt-in',
                                                  lifecycle=lifecycle), 0)
                expected = [str(self.prefix / 'bin/gz-sim-rs'), str(self.prefix / 'share/robot.sdf'),
                            '--server', '--web-viewer', '9014']
                if lifecycle:
                    expected.append('--web-lifecycle')
                self.assertEqual(spawn.call_args.args[0], expected)
                self.assertTrue(spawn.call_args.kwargs['start_new_session'])
                self.assertEqual(spawn.call_args.kwargs['env']['GZ_PARTITION'], 'opt-in')
                self.assertEqual([call.args for call in check.call_args_list], [(9014,), (7449,)])
                self.assertEqual(http.call_args.args[0], ('127.0.0.1', 8082))
                url = output.call_args_list[0].args[0].removeprefix('Open ')
                outer = urlsplit(url)
                self.assertEqual((outer.scheme, outer.hostname, outer.port), ('http', '127.0.0.1', 8082))
                query = parse_qs(outer.query, strict_parsing=True)
                self.assertEqual(query, {'backend': ['primitive'], 'ui': ['full'],
                                         'url': ['ws://127.0.0.1:9014/'],
                                         **({'lifecycle': ['1']} if lifecycle else {})})
                self.assertEqual(urlsplit(query['url'][0]).query, '')
                thread.start.assert_called_once_with()
                server.shutdown.assert_called_once_with()
                server.server_close.assert_called_once_with()

    def test_lifecycle_parser_is_explicit_and_rejects_other_modes_and_invalid_ports(self):
        with patch.object(preview, 'serve', return_value=0) as serve:
            self.assertEqual(preview.main(['--partition', 'm20', 'run', '--lifecycle',
                                           '--http-port', '8082', '--ws-port', '9016'], self.prefix), 0)
            serve.assert_called_once_with(self.prefix, 8082, 9016, 'm20', lifecycle=True)
        invalid = (['--lifecycle', 'run'], ['verify', '--lifecycle'],
                   ['drive', '0.4', '0', '3', '--lifecycle'],
                   ['run', '--web-lifecycle'], ['run', '--lifecycle=0'])
        for argv in invalid:
            with self.subTest(argv=argv), patch.object(preview, 'serve') as serve, \
                    patch.object(preview.os, 'execve') as drive:
                with self.assertRaises(SystemExit) as stopped:
                    preview.main(argv, self.prefix)
                self.assertEqual(stopped.exception.code, 2)
                serve.assert_not_called()
                drive.assert_not_called()
        with patch.object(preview, 'serve') as serve:
            self.assertEqual(preview.main(['run', '--lifecycle', '--ws-port', '70000'], self.prefix), 1)
            serve.assert_not_called()
        for port in ('7449', '8082'):
            with self.subTest(port=port), patch.object(preview.subprocess, 'Popen') as spawn:
                self.assertEqual(preview.main(['run', '--lifecycle', '--http-port', '8082',
                                               '--ws-port', port], self.prefix), 1)
                spawn.assert_not_called()

    def test_lifecycle_launch_failure_closes_http_without_printing_a_success_url(self):
        server = Mock()
        with patch.object(preview, 'check_port'), \
                patch.object(preview.http.server, 'ThreadingHTTPServer', return_value=server), \
                patch.object(preview.subprocess, 'Popen', side_effect=OSError('cannot launch')), \
                patch.object(preview.threading, 'Thread') as thread, patch('builtins.print') as output:
            with self.assertRaisesRegex(OSError, 'cannot launch'):
                preview.serve(self.prefix, 8082, 9014, 'm20', lifecycle=True)
        server.server_close.assert_called_once_with()
        server.shutdown.assert_not_called()
        thread.assert_not_called()
        output.assert_not_called()

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


    def test_joint_helper_is_required_even_if_manifest_is_rewritten(self):
        name = 'bin/gz-joint-command'
        entry = self.files.pop(name)
        target = self.prefix / name
        original = target.read_bytes()
        target.unlink()
        self.save_manifest()
        with patch.object(preview, 'serve') as serve:
            self.assertEqual(preview.main(['run'], self.prefix), 1)
            serve.assert_not_called()
        target.write_bytes(original)
        target.chmod(0o755)
        self.files[name] = entry
        self.save_manifest()
        self.assertEqual(preview.verify_prefix(self.prefix)['files'], self.files)


    def install_example(self, name='m59_multiple_diff_drives.sdf'):
        relative = 'share/examples/' + name
        target = self.prefix / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        content = b"<sdf version='1.12'><world name='default'/></sdf>\n"
        target.write_bytes(content)
        target.chmod(0o644)
        self.files[relative] = {'bytes': len(content), 'sha256': hashlib.sha256(content).hexdigest(),
                                'mode': 0o644}
        self.save_manifest()
        return target

    def test_manifest_bound_example_selects_the_relocated_world_and_preserves_lifecycle(self):
        self.install_example()
        moved = self.directory / 'relocated examples prefix'
        shutil.move(self.prefix, moved)
        basename = 'm59_multiple_diff_drives.sdf'
        with patch.object(preview, 'serve', return_value=0) as serve:
            self.assertEqual(preview.main(['--partition', 'two-robots', 'run', '--example', basename,
                                           '--lifecycle', '--http-port', '8082'], moved), 0)
            serve.assert_called_once_with(moved, 8082, 9014, 'two-robots',
                                          lifecycle=True, example=basename)
        child = Mock(pid=123, returncode=0)
        child.poll.return_value = 0
        http = Mock()
        thread = Mock()
        with patch.object(preview, 'check_port') as ports, \
                patch.object(preview.http.server, 'ThreadingHTTPServer', return_value=http), \
                patch.object(preview.threading, 'Thread', return_value=thread), \
                patch.object(preview.subprocess, 'Popen', return_value=child) as spawn, \
                patch.object(preview.os, 'killpg') as kill, patch('builtins.print'):
            self.assertEqual(preview.serve(moved, 8082, 9014, 'two-robots',
                                           lifecycle=True, example=basename), 0)
        self.assertEqual(spawn.call_args.args[0], [str(moved / 'bin/gz-sim-rs'),
                          str(moved / 'share/examples' / basename),
                          '--server', '--web-viewer', '9014', '--web-lifecycle'])
        self.assertEqual(spawn.call_args.kwargs['cwd'], moved)
        self.assertTrue(spawn.call_args.kwargs['start_new_session'])
        self.assertEqual(spawn.call_args.kwargs['env']['GZ_PARTITION'], 'two-robots')
        self.assertEqual(spawn.call_args.kwargs['env']['ZENOH_CONFIG'], str(moved / 'share/zenoh.json5'))
        self.assertEqual([call.args for call in ports.call_args_list], [(9014,), (7449,)])
        kill.assert_not_called()
        http.shutdown.assert_called_once_with()
        http.server_close.assert_called_once_with()

    def test_example_paths_and_unlisted_or_nested_targets_launch_nothing(self):
        self.install_example()
        for name in ('', '.', '..', '../robot.sdf', '/tmp/outside.sdf',
                     './m59_multiple_diff_drives.sdf', 'nested/m59_multiple_diff_drives.sdf',
                     'nested\\world.sdf', 'C:world.sdf', 'world.sdf\n',
                     'robot.txt', 'missing.sdf'):
            with self.subTest(name=name), patch.object(preview, 'serve') as serve, \
                    patch.object(preview.subprocess, 'Popen') as spawn:
                self.assertEqual(preview.main(['run', '--example', name], self.prefix), 1)
                serve.assert_not_called()
                spawn.assert_not_called()

    def test_selected_example_content_mode_link_and_inventory_changes_fail_before_listeners(self):
        target = self.install_example()
        original = target.read_bytes()
        for fault in ('content', 'mode', 'link', 'missing', 'unlisted'):
            with self.subTest(fault=fault):
                if fault == 'content':
                    target.write_bytes(b'changed')
                elif fault == 'mode':
                    target.chmod(0o600)
                elif fault == 'link':
                    target.unlink()
                    outside = self.directory / 'external world.sdf'
                    outside.write_bytes(original)
                    target.symlink_to(outside)
                elif fault == 'missing':
                    target.unlink()
                else:
                    self.files.pop('share/examples/m59_multiple_diff_drives.sdf')
                    self.save_manifest()
                with patch.object(preview, 'check_port') as port, \
                        patch.object(preview.subprocess, 'Popen') as spawn:
                    with self.assertRaises(ValueError):
                        preview.serve(self.prefix, 8081, 9014, 'demo',
                                      example='m59_multiple_diff_drives.sdf')
                    port.assert_not_called()
                    spawn.assert_not_called()
                if target.is_symlink():
                    target.unlink()
                self.install_example()

    def test_explicit_command_topic_is_forwarded_as_one_argument_without_changing_default_argv(self):
        for topic in ('/m59/left/cmd_vel', '/m59/right/cmd_vel'):
            with self.subTest(topic=topic), patch.object(preview.os, 'execve', return_value=0) as send:
                self.assertEqual(preview.main(['--partition', 'two-robots', 'drive', '--topic', topic,
                                               '0.4', '-0.5', '3'], self.prefix), 0)
                helper = str(self.prefix / 'bin/gz-drive-command')
                self.assertEqual(send.call_args.args[0], helper)
                self.assertEqual(send.call_args.args[1], [helper, 'tcp/127.0.0.1:7449',
                                                         '0.4', '-0.5', '3', '--topic', topic])
                self.assertEqual(send.call_args.args[2]['GZ_PARTITION'], 'two-robots')
                self.assertEqual(send.call_args.args[2]['ZENOH_CONFIG'], str(self.prefix / 'share/zenoh.json5'))
        # A packet topic is validated by the Rust helper; the wrapper uses no shell.
        with patch.object(preview.os, 'execve', return_value=0) as send:
            self.assertEqual(preview.main(['drive', '0', '0', '0', '--topic', '/x;echo unsafe'], self.prefix), 0)
            self.assertEqual(send.call_args.args[1][-1], '/x;echo unsafe')

    def test_new_selectors_reject_missing_values_and_other_command_modes_before_dispatch(self):
        for argv in (['run', '--example'], ['drive', '0', '0', '1', '--topic'],
                     ['verify', '--example', 'world.sdf'], ['verify', '--topic', '/a'],
                     ['drive', '0', '0', '1', '--example', 'world.sdf'], ['run', '--topic', '/a']):
            with self.subTest(argv=argv), patch.object(preview, 'serve') as serve, \
                    patch.object(preview.os, 'execve') as drive:
                with self.assertRaises(SystemExit) as stopped:
                    preview.main(argv, self.prefix)
                self.assertEqual(stopped.exception.code, 2)
                serve.assert_not_called()
                drive.assert_not_called()


    def install_m70_camera_fixture(self):
        self.install_guides(('INSTALL.md', 'CAPABILITIES.md'))
        for name, content, mode in (('PREVIEW.md', b'camera preview guide', 0o644),
                                    ('bin/gz-render-worker', b'packaged worker fixture', 0o755)):
            path = self.prefix / name
            path.write_bytes(content)
            path.chmod(mode)
            self.files[name] = {'bytes': len(content), 'sha256': hashlib.sha256(content).hexdigest(),
                                'mode': mode}
        self.camera_manifest = {
            'schema_version': 1, 'target': 'ubuntu-24.04-x86_64', 'guide_generation': 'M70',
            'profile': 'release; no-default-features; simulator rgb-camera; helpers native-free; optional packaged GPU worker',
            'binary_roles': {
                'simulator': {'path': 'bin/gz-sim-rs', 'package': 'gz-sim-cli',
                              'target': 'gz-sim-rs', 'features': ['rgb-camera']},
                'drive_command': {'path': 'bin/gz-drive-command', 'package': 'gz-sim-server',
                                  'target': 'drive_command', 'features': []},
                'joint_command': {'path': 'bin/gz-joint-command', 'package': 'gz-sim-server',
                                  'target': 'joint_command', 'features': []},
                'camera_worker': {'path': 'bin/gz-render-worker', 'package': 'gz-render-worker',
                                  'target': 'gz-render-worker', 'features': []}},
            'files': self.files}
        self.save_m70_camera_manifest()

    def save_m70_camera_manifest(self):
        (self.prefix / 'manifest.json').write_text(json.dumps(self.camera_manifest))

    def camera_os_fixture(self):
        icd = self.directory / 'icd'
        libraries = self.directory / 'system libraries'
        runtime = self.directory / 'private camera runtime'
        icd.mkdir(exist_ok=True)
        libraries.mkdir(exist_ok=True)
        runtime.mkdir(mode=0o700, exist_ok=True)
        provider = libraries / 'libvulkan_lvp.so'
        provider.write_bytes(b'OS provider admission fixture; not an executed driver')
        selected = icd / 'lvp_icd.x86_64.json'
        selected.write_text(json.dumps({'file_format_version': '1.0.0',
                                       'ICD': {'library_path': str(provider), 'api_version': '1.3.0'}}))
        return icd, libraries, runtime, selected

    @isolated_camera_environment
    def test_m70_manifest_requires_exact_roles_executable_worker_and_new_guides(self):
        self.install_m70_camera_fixture()
        self.assertEqual(preview.verify_prefix(self.prefix)['guide_generation'], 'M70')
        original = json.dumps(self.camera_manifest)
        for fault in ('missing', 'extra', 'swapped', 'features', 'profile', 'target', 'guide', 'mode'):
            with self.subTest(fault=fault):
                self.camera_manifest = json.loads(original)
                if fault == 'missing':
                    del self.camera_manifest['binary_roles']['camera_worker']
                elif fault == 'extra':
                    self.camera_manifest['binary_roles']['another_worker'] = {}
                elif fault == 'swapped':
                    self.camera_manifest['binary_roles']['camera_worker']['path'] = 'bin/gz-sim-rs'
                elif fault == 'features':
                    self.camera_manifest['binary_roles']['simulator']['features'] = []
                elif fault == 'profile':
                    self.camera_manifest['profile'] = 'release; no-default-features; no rgb-camera'
                elif fault == 'target':
                    self.camera_manifest['target'] = 'other-system'
                elif fault == 'guide':
                    del self.camera_manifest['files']['PREVIEW.md']
                else:
                    # A rewritten inventory cannot admit a nonexecutable worker.
                    self.camera_manifest['files']['bin/gz-render-worker']['mode'] = 0o644
                    (self.prefix / 'bin/gz-render-worker').chmod(0o644)
                self.save_m70_camera_manifest()
                with self.assertRaises(ValueError):
                    preview.verify_prefix(self.prefix)
                (self.prefix / 'bin/gz-render-worker').chmod(0o755)
        self.camera_manifest = json.loads(original)
        self.save_m70_camera_manifest()
        moved = self.directory / 'relocated camera prefix'
        shutil.move(self.prefix, moved)
        self.assertEqual(preview.verify_prefix(moved)['binary_roles'], self.camera_manifest['binary_roles'])

    @isolated_camera_environment
    def test_camera_output_flag_is_explicit_m70_only_and_conflicts_before_dispatch(self):
        with patch.object(preview, 'serve') as serve:
            self.assertEqual(preview.main(['run', '--camera-output'], self.prefix), 1)
            serve.assert_not_called()
        self.install_m70_camera_fixture()
        with patch.object(preview, 'serve', return_value=0) as serve:
            self.assertEqual(preview.main(['run'], self.prefix), 0)
            self.assertNotIn('camera_output', serve.call_args.kwargs)
        with patch.object(preview, 'serve', return_value=0) as serve:
            self.assertEqual(preview.main(['run', '--camera-output'], self.prefix), 0)
            self.assertEqual(serve.call_args.kwargs, {'camera_output': True})
        with patch.object(preview, 'serve') as serve:
            self.assertEqual(preview.main(['run', '--camera-output', '--lifecycle'], self.prefix), 1)
            serve.assert_not_called()
        for argv in (['verify', '--camera-output'], ['drive', '0', '0', '1', '--camera-output'],
                     ['run', '--camera-output=/arbitrary/worker']):
            with self.subTest(argv=argv), patch.object(preview, 'serve') as serve:
                with self.assertRaises(SystemExit) as stopped:
                    preview.main(argv, self.prefix)
                self.assertEqual(stopped.exception.code, 2)
                serve.assert_not_called()

    @isolated_camera_environment
    def test_camera_serve_derives_packaged_worker_without_changing_default_dispatch(self):
        self.install_m70_camera_fixture()
        with patch.object(preview, '_serve', return_value=7) as child, \
                patch.object(preview, 'camera_environment') as environment:
            self.assertEqual(preview.serve(self.prefix, 8082, 9014, 'plain'), 7)
            self.assertEqual(child.call_args.kwargs, {'lifecycle': False, 'example': None})
            environment.assert_not_called()
        bounded = {'GZ_PARTITION': 'camera-test', 'VK_ICD_FILENAMES': '/system/icd.json'}
        with patch.object(preview, '_serve', return_value=0) as child, \
                patch.object(preview, 'camera_environment', return_value=bounded) as environment:
            self.assertEqual(preview.serve(self.prefix, 8082, 9014, 'camera-test', camera_output=True), 0)
            self.assertEqual(child.call_args.kwargs, {'example': None,
                'worker': self.prefix / 'bin/gz-render-worker', 'child_environment': bounded,
                'retirement': {'confirmed': True}})
            runtime = environment.call_args.args[2]
            self.assertFalse(runtime.exists())
        with patch.object(preview, '_serve') as child, \
                patch.object(preview, 'camera_environment') as environment:
            with self.assertRaises(ValueError):
                preview.serve(self.prefix, 8082, 9014, 'camera', lifecycle=True, camera_output=True)
            child.assert_not_called()
            environment.assert_not_called()

    @isolated_camera_environment
    def test_camera_payload_changes_refuse_before_driver_admission_listeners_or_spawn(self):
        self.install_m70_camera_fixture()
        worker = self.prefix / 'bin/gz-render-worker'
        original = worker.read_bytes()
        for fault in ('bytes', 'mode', 'link', 'missing'):
            with self.subTest(fault=fault):
                if fault == 'bytes':
                    worker.write_bytes(b'changed')
                elif fault == 'mode':
                    worker.chmod(0o644)
                elif fault == 'link':
                    worker.unlink()
                    external = self.directory / 'external worker'
                    external.write_bytes(original)
                    worker.symlink_to(external)
                else:
                    worker.unlink()
                with patch.object(preview, 'camera_environment') as environment, \
                        patch.object(preview, '_serve') as child:
                    with self.assertRaises(ValueError):
                        preview.serve(self.prefix, 8082, 9014, 'camera', camera_output=True)
                    environment.assert_not_called()
                    child.assert_not_called()
                if worker.is_symlink():
                    worker.unlink()
                worker.write_bytes(original)
                worker.chmod(0o755)

    @isolated_camera_environment
    def test_camera_environment_pins_system_icd_and_private_runtime_without_inherited_overrides(self):
        icd, libraries, runtime, selected = self.camera_os_fixture()
        with patch.object(preview, 'CAMERA_ICD_DIRECTORY', icd), \
                patch.object(preview, 'CAMERA_LIBRARY_DIRECTORIES', (libraries,)), \
                patch.dict(os.environ, {'LD_PRELOAD': '', 'VK_DRIVER_FILES': '',
                    'HOME': '/wrong/home', 'XDG_RUNTIME_DIR': '/wrong/runtime',
                    'GZ_PARTITION': 'wrong'}):
            env = preview.camera_environment(self.prefix, 'camera-partition', runtime)
        self.assertEqual(env['VK_ICD_FILENAMES'], str(selected))
        self.assertEqual(env['XDG_RUNTIME_DIR'], str(runtime))
        self.assertEqual(env['HOME'], str(runtime))
        self.assertEqual(env['MESA_SHADER_CACHE_DISABLE'], 'true')
        self.assertEqual(env['GZ_PARTITION'], 'camera-partition')
        self.assertEqual(env['ZENOH_CONFIG'], str(self.prefix / 'share/zenoh.json5'))
        self.assertEqual(env['PATH'], '/usr/bin:/bin')
        self.assertTrue({'LD_PRELOAD', 'LD_LIBRARY_PATH', 'VK_DRIVER_FILES', 'WGPU_ADAPTER_NAME'}.isdisjoint(env))
        self.assertEqual(runtime.stat().st_mode & 0o777, 0o700)

    @isolated_camera_environment
    def test_camera_environment_refuses_ambiguous_linked_invalid_or_missing_os_provider(self):
        icd, libraries, runtime, selected = self.camera_os_fixture()
        original = selected.read_bytes()
        for fault in ('missing', 'multiple', 'link', 'large', 'invalid', 'missing-bare',
                      'foreign', 'provider', 'runtime'):
            with self.subTest(fault=fault):
                if fault == 'missing':
                    selected.unlink()
                elif fault == 'multiple':
                    (icd / 'lvp_icd.other.json').write_bytes(original)
                elif fault == 'link':
                    selected.unlink()
                    external = self.directory / 'external ICD'
                    external.write_bytes(original)
                    selected.symlink_to(external)
                elif fault == 'large':
                    selected.write_bytes(b'x' * 65537)
                elif fault == 'invalid':
                    selected.write_text(json.dumps({'ICD': {'library_path': '../other.so'}}))
                elif fault == 'missing-bare':
                    selected.write_text(json.dumps({'ICD': {'library_path': 'libvulkan_lvp.so'}}))
                    (libraries / 'libvulkan_lvp.so').unlink()
                elif fault == 'foreign':
                    other = self.directory / 'libvulkan_lvp.so'
                    other.write_bytes(b'foreign provider')
                    selected.write_text(json.dumps({'ICD': {'library_path': str(other)}}))
                elif fault == 'provider':
                    (libraries / 'libvulkan_lvp.so').unlink()
                else:
                    runtime.chmod(0o755)
                with patch.object(preview, 'CAMERA_ICD_DIRECTORY', icd), \
                        patch.object(preview, 'CAMERA_LIBRARY_DIRECTORIES', (libraries,)):
                    with self.assertRaises((OSError, ValueError)):
                        preview.camera_environment(self.prefix, 'camera', runtime)
                if selected.is_symlink():
                    selected.unlink()
                selected.write_bytes(original)
                (icd / 'lvp_icd.other.json').unlink(missing_ok=True)
                (libraries / 'libvulkan_lvp.so').write_bytes(b'OS provider admission fixture')
                runtime.chmod(0o700)
        with patch.object(preview, 'CAMERA_ICD_DIRECTORY', icd), \
                patch.object(preview, 'CAMERA_LIBRARY_DIRECTORIES', (libraries,)), \
                patch.object(preview.os, 'geteuid', return_value=-1):
            with self.assertRaisesRegex(ValueError, 'private owned'):
                preview.camera_environment(self.prefix, 'camera', runtime)

    @isolated_camera_environment
    def test_camera_environment_accepts_packaged_lavapipe_basename_without_rewriting_inputs(self):
        icd, libraries, runtime, selected = self.camera_os_fixture()
        selected = selected.rename(icd / 'lvp_icd.json')
        encoded = (b'{\n    "ICD": {\n        "api_version": "1.4.318",\n'
                   b'        "library_path": "libvulkan_lvp.so"\n    },\n'
                   b'    "file_format_version": "1.0.1"\n}')
        selected.write_bytes(encoded)
        provider = libraries / 'libvulkan_lvp.so'
        original_provider = provider.read_bytes()
        with patch.object(preview, 'CAMERA_ICD_DIRECTORY', icd), \
                patch.object(preview, 'CAMERA_LIBRARY_DIRECTORIES', (libraries,)):
            env = preview.camera_environment(self.prefix, 'camera', runtime)
        self.assertEqual(env['VK_ICD_FILENAMES'], str(selected))
        self.assertEqual(selected.read_bytes(), encoded)
        self.assertEqual(provider.read_bytes(), original_provider)
        self.assertTrue({'LD_LIBRARY_PATH', 'VK_DRIVER_FILES', 'VK_ADD_DRIVER_FILES'}.isdisjoint(env))
        self.assertEqual(env['HOME'], str(runtime))
        self.assertEqual(env['XDG_RUNTIME_DIR'], str(runtime))

    @isolated_camera_environment
    def test_camera_environment_basename_deduplicates_usrmerge_directory_aliases(self):
        icd, libraries, runtime, selected = self.camera_os_fixture()
        selected.write_text(json.dumps({'ICD': {'library_path': 'libvulkan_lvp.so'}}))
        alias = self.directory / 'system library alias'
        alias.symlink_to(libraries, target_is_directory=True)
        for directories in ((libraries, alias), (alias, libraries)):
            with self.subTest(directories=directories), \
                    patch.object(preview, 'CAMERA_ICD_DIRECTORY', icd), \
                    patch.object(preview, 'CAMERA_LIBRARY_DIRECTORIES', directories):
                env = preview.camera_environment(self.prefix, 'camera', runtime)
                self.assertEqual(env['VK_ICD_FILENAMES'], str(selected))

    @isolated_camera_environment
    def test_camera_environment_basename_refuses_missing_ambiguous_or_unsafe_providers(self):
        for fault in ('missing', 'ambiguous', 'directory', 'dangling', 'outside-link',
                      'link-loop', 'other-name', 'relative-path', 'traversal'):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                icd = root / 'icd'
                libraries = root / 'system libraries'
                runtime = root / 'private runtime'
                icd.mkdir()
                libraries.mkdir()
                runtime.mkdir(mode=0o700)
                provider = libraries / 'libvulkan_lvp.so'
                provider.write_bytes(b'Admission fixture, never an executed driver')
                directories = (libraries,)
                library = 'libvulkan_lvp.so'
                if fault == 'missing':
                    provider.unlink()
                elif fault == 'ambiguous':
                    second = root / 'second system directory'
                    second.mkdir()
                    (second / library).write_bytes(b'A different ordinary provider')
                    directories = (libraries, second)
                elif fault == 'directory':
                    provider.unlink()
                    provider.mkdir()
                elif fault in ('dangling', 'outside-link', 'link-loop'):
                    provider.unlink()
                    target = root / 'absent provider'
                    if fault == 'outside-link':
                        target = root / 'foreign directory' / 'libvulkan_lvp.so'
                        target.parent.mkdir()
                        target.write_bytes(b'Foreign provider')
                        second = root / 'valid system directory'
                        second.mkdir()
                        (second / library).write_bytes(b'An allowed provider')
                        directories = (libraries, second)
                    elif fault == 'link-loop':
                        target = provider
                    provider.symlink_to(target)
                elif fault == 'other-name':
                    library = 'libvulkan_other.so'
                    (libraries / library).write_bytes(b'Unselected driver')
                elif fault == 'relative-path':
                    library = 'system libraries/libvulkan_lvp.so'
                else:
                    library = '../system libraries/libvulkan_lvp.so'
                selected = icd / 'lvp_icd.json'
                selected.write_text(json.dumps({'ICD': {'library_path': library}}))
                with patch.object(preview, 'CAMERA_ICD_DIRECTORY', icd), \
                        patch.object(preview, 'CAMERA_LIBRARY_DIRECTORIES', directories):
                    with self.assertRaises((OSError, ValueError)):
                        preview.camera_environment(self.prefix, 'camera', runtime)

    @isolated_camera_environment
    def test_camera_environment_failure_retires_runtime_before_any_service_effect(self):
        self.install_m70_camera_fixture()
        seen = []
        def refuse(_prefix, _partition, runtime):
            seen.append(runtime)
            self.assertEqual(runtime.stat().st_mode & 0o777, 0o700)
            raise ValueError('provider refused')
        with patch.object(preview, 'camera_environment', side_effect=refuse), \
                patch.object(preview, '_serve') as child, patch.object(preview, 'check_port') as port:
            with self.assertRaisesRegex(ValueError, 'provider refused'):
                preview.serve(self.prefix, 8082, 9014, 'camera', camera_output=True)
            child.assert_not_called()
            port.assert_not_called()
        self.assertEqual(len(seen), 1)
        self.assertFalse(seen[0].exists())

    @isolated_camera_environment
    def test_camera_run_passes_fixed_worker_and_controlled_environment_to_owned_child(self):
        self.install_m70_camera_fixture()
        bounded = {'GZ_PARTITION': 'installed-camera', 'VK_ICD_FILENAMES': '/system/icd.json'}
        child = Mock(pid=98765, returncode=0)
        child.poll.return_value = 0
        with patch.object(preview, 'camera_environment', return_value=bounded), \
                patch.object(preview, 'check_port'), \
                patch.object(preview.http.server, 'ThreadingHTTPServer') as http, \
                patch.object(preview.threading, 'Thread'), \
                patch.object(preview.subprocess, 'Popen', return_value=child) as spawn:
            self.assertEqual(preview.serve(self.prefix, 8082, 9014, 'installed-camera', camera_output=True), 0)
            self.assertEqual(spawn.call_args.args[0], [str(self.prefix / 'bin/gz-sim-rs'),
                str(self.prefix / 'share/robot.sdf'), '--server', '--web-viewer', '9014',
                '--camera-worker', str(self.prefix / 'bin/gz-render-worker')])
            self.assertEqual(spawn.call_args.kwargs['env'], bounded)
            self.assertTrue(spawn.call_args.kwargs['start_new_session'])
            self.assertEqual(spawn.call_args.kwargs['cwd'], self.prefix)
            self.assertNotIn('--web-lifecycle', spawn.call_args.args[0])
            http.return_value.server_close.assert_called_once()

    @isolated_camera_environment
    def test_camera_overrides_refuse_before_runtime_driver_listener_or_spawn_effects(self):
        self.install_m70_camera_fixture()
        names = ('LD_PRELOAD', 'LD_AUDIT', 'LD_LIBRARY_PATH', 'VK_DRIVER_FILES',
                 'VK_ADD_DRIVER_FILES', 'VK_ICD_FILENAMES', 'VK_LAYER_PATH',
                 'VK_ADD_LAYER_PATH', 'VK_INSTANCE_LAYERS', 'WGPU_ADAPTER_NAME',
                 'WGPU_BACKEND', 'MESA_LOADER_DRIVER_OVERRIDE', 'DRI_PRIME',
                 'VK_LOADER_DRIVERS_SELECT', 'GALLIUM_DRIVER')
        for name in names:
            with self.subTest(name=name), patch.dict(os.environ, {name: 'uncontrolled'}), \
                    patch.object(preview.tempfile, 'mkdtemp') as runtime, \
                    patch.object(preview, 'camera_environment') as environment, \
                    patch.object(preview, '_serve') as serve:
                with self.assertRaisesRegex(ValueError, name):
                    preview.serve(self.prefix, 8082, 9014, 'camera', camera_output=True)
                runtime.assert_not_called()
                environment.assert_not_called()
                serve.assert_not_called()
        for name in names:
            with self.subTest(direct=name), patch.dict(os.environ, {name: 'uncontrolled'}):
                with self.assertRaisesRegex(ValueError, name):
                    preview.camera_environment(self.prefix, 'camera', self.directory / 'absent runtime')

    @isolated_camera_environment
    def test_camera_failed_retirement_retains_runtime_and_attempts_all_remaining_cleanup(self):
        self.install_m70_camera_fixture()
        seen = []
        def environment(_prefix, _partition, runtime):
            seen.append(runtime)
            self.addCleanup(shutil.rmtree, runtime)
            return {'HOME': str(runtime)}
        child = Mock(pid=98765)
        child.poll.return_value = None
        stopping = Mock()
        stopping.wait.return_value = True
        stopping.is_set.return_value = True
        failure = subprocess.TimeoutExpired(['owned simulator'], 5)
        with patch.object(preview, 'camera_environment', side_effect=environment), \
                patch.object(preview, 'check_port'), \
                patch.object(preview.http.server, 'ThreadingHTTPServer') as http, \
                patch.object(preview.threading, 'Thread') as thread, \
                patch.object(preview.threading, 'Event', return_value=stopping), \
                patch.object(preview.signal, 'signal', return_value='original') as signals, \
                patch.object(preview.subprocess, 'Popen', return_value=child), \
                patch.object(preview, 'stop_child', side_effect=failure), \
                patch('builtins.print') as output:
            http.return_value.shutdown.side_effect = OSError('HTTP shutdown failed')
            with self.assertRaises(subprocess.TimeoutExpired) as raised:
                preview.serve(self.prefix, 8082, 9014, 'camera', camera_output=True)
        self.assertIs(raised.exception, failure)
        self.assertEqual(len(seen), 1)
        self.assertTrue(seen[0].is_dir())
        http.return_value.shutdown.assert_called_once_with()
        thread.return_value.join.assert_called_once_with(timeout=5)
        http.return_value.server_close.assert_called_once_with()
        self.assertEqual(signals.call_count, 6)
        self.assertEqual([call.args for call in signals.call_args_list[-3:]],
                         [(signum, 'original') for signum in
                          (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)])
        self.assertTrue(any('HTTP shutdown failed' in note for note in failure.__notes__))
        self.assertTrue(any('runtime retained: ' + str(seen[0]) in note for note in failure.__notes__))
        self.assertTrue(any('runtime retained: ' + str(seen[0]) in str(call.args)
                            for call in output.call_args_list))

    @isolated_camera_environment
    def test_camera_constructor_uncertainty_retains_runtime_and_primary_failure(self):
        self.install_m70_camera_fixture()
        seen = []
        def environment(_prefix, _partition, runtime):
            seen.append(runtime)
            self.addCleanup(shutil.rmtree, runtime)
            return {'HOME': str(runtime)}
        failure = OSError('spawn outcome unknown')
        with patch.object(preview, 'camera_environment', side_effect=environment), \
                patch.object(preview, 'check_port'), \
                patch.object(preview.http.server, 'ThreadingHTTPServer') as http, \
                patch.object(preview.threading, 'Thread') as thread, \
                patch.object(preview.signal, 'signal', return_value='original') as signals, \
                patch.object(preview.subprocess, 'Popen', side_effect=failure), \
                patch.object(preview, 'stop_child') as stop, patch('builtins.print'):
            http.return_value.server_close.side_effect = OSError('HTTP close failed')
            with self.assertRaises(OSError) as raised:
                preview.serve(self.prefix, 8082, 9014, 'camera', camera_output=True)
        self.assertIs(raised.exception, failure)
        self.assertEqual(len(seen), 1)
        self.assertTrue(seen[0].is_dir())
        self.assertTrue(any('runtime retained: ' + str(seen[0]) in note for note in failure.__notes__))
        self.assertTrue(any('HTTP close failed' in note for note in failure.__notes__))
        thread.assert_not_called()
        stop.assert_not_called()
        http.return_value.shutdown.assert_not_called()
        http.return_value.server_close.assert_called_once_with()
        self.assertEqual(signals.call_count, 6)

    @isolated_camera_environment
    def test_camera_pre_spawn_listener_failure_removes_owned_runtime(self):
        self.install_m70_camera_fixture()
        seen = []
        def environment(_prefix, _partition, runtime):
            seen.append(runtime)
            return {'HOME': str(runtime)}
        with patch.object(preview, 'camera_environment', side_effect=environment), \
                patch.object(preview, 'check_port', side_effect=OSError('occupied')), \
                patch.object(preview.subprocess, 'Popen') as spawn:
            with self.assertRaisesRegex(OSError, 'occupied'):
                preview.serve(self.prefix, 8082, 9014, 'camera', camera_output=True)
        spawn.assert_not_called()
        self.assertEqual(len(seen), 1)
        self.assertFalse(seen[0].exists())

    @isolated_camera_environment
    def test_camera_runtime_identity_replacement_is_retained_and_refused(self):
        self.install_m70_camera_fixture()
        seen = []
        def environment(_prefix, _partition, runtime):
            seen.append(runtime)
            return {'HOME': str(runtime)}
        def replace(_prefix, _http, _ws, _partition, **_options):
            runtime = seen[0]
            displaced = runtime.with_name(runtime.name + '-original')
            runtime.rename(displaced)
            self.addCleanup(shutil.rmtree, displaced)
            runtime.mkdir(mode=0o700)
            self.addCleanup(shutil.rmtree, runtime)
            return 0
        with patch.object(preview, 'camera_environment', side_effect=environment), \
                patch.object(preview, '_serve', side_effect=replace):
            with self.assertRaisesRegex(ValueError, 'identity changed; retained:'):
                preview.serve(self.prefix, 8082, 9014, 'camera', camera_output=True)
        self.assertTrue(seen[0].is_dir())

    @isolated_camera_environment
    def test_camera_confirmed_wait_precedes_runtime_removal(self):
        self.install_m70_camera_fixture()
        seen = []
        def environment(_prefix, _partition, runtime):
            seen.append(runtime)
            return {'HOME': str(runtime)}
        child = Mock(pid=98765, returncode=0)
        child.poll.return_value = 0
        def stop(selected):
            self.assertIs(selected, child)
            self.assertTrue(seen[0].is_dir())
            selected.wait(timeout=10)
        with patch.object(preview, 'camera_environment', side_effect=environment), \
                patch.object(preview, 'check_port'), \
                patch.object(preview.http.server, 'ThreadingHTTPServer'), \
                patch.object(preview.threading, 'Thread'), \
                patch.object(preview.subprocess, 'Popen', return_value=child), \
                patch.object(preview, 'stop_child', side_effect=stop):
            self.assertEqual(preview.serve(self.prefix, 8082, 9014, 'camera', camera_output=True), 0)
        child.wait.assert_called_once_with(timeout=10)
        self.assertFalse(seen[0].exists())



    def static_request(self, url, method='GET', headers=None):
        parsed = urlsplit(url)
        connection = http.client.HTTPConnection(parsed.hostname, parsed.port, timeout=2)
        target = parsed.path + ('?' + parsed.query if parsed.query else '')
        try:
            connection.request(method, target, headers=headers or {})
            response = connection.getresponse()
            return response.status, dict(response.getheaders()), response.read()
        finally:
            connection.close()

    def launch_static_viewer(self, visit, port=None):
        if port is None:
            with socket.socket() as candidate:
                candidate.bind(('127.0.0.1', 0))
                port = candidate.getsockname()[1]
        child = Mock(pid=123, returncode=0)
        child.poll.return_value = None
        announced = threading.Event()
        urls = []
        failures = []

        def printed(message, **_options):
            if message.startswith('Open '):
                urls.append(message.removeprefix('Open '))
                announced.set()

        def browser():
            try:
                self.assertTrue(announced.wait(5), 'launcher did not publish its owned URL')
                self.assertEqual(len(urls), 1)
                parsed = urlsplit(urls[0])
                self.assertEqual((parsed.scheme, parsed.hostname, parsed.port),
                                 ('http', '127.0.0.1', port))
                self.assertRegex(parsed.path, r'^/__gz_preview__/[0-9a-f]{32}/$')
                visit(urls[0])
            except BaseException as error:
                failures.append(error)
            finally:
                child.poll.return_value = 0

        client = threading.Thread(target=browser)
        with patch.object(preview, 'check_port'), \
                patch.object(preview.subprocess, 'Popen', return_value=child), \
                patch.object(preview.os, 'killpg') as kill, \
                patch('builtins.print', side_effect=printed):
            client.start()
            try:
                result = preview.serve(self.prefix, port, 9014, 'fresh-static')
            finally:
                client.join(timeout=10)
        self.assertFalse(client.is_alive())
        kill.assert_not_called()
        if failures:
            raise failures[0]
        self.assertEqual(result, 0)
        with socket.socket() as probe:
            self.assertNotEqual(probe.connect_ex(('127.0.0.1', port)), 0)
        return urls[0], port

    def test_each_launch_has_fresh_directory_routes_for_all_six_unchanged_viewer_files(self):
        names = [name.removeprefix('web/') for name in self.files if name.startswith('web/')]
        expected = {name: (self.prefix / 'web' / name).read_bytes() for name in names}
        self.assertEqual(len(expected), 6)
        prior = []

        def visit(url):
            for name, content in expected.items():
                selected = urljoin(url, name)
                self.assertTrue(urlsplit(selected).path.startswith(urlsplit(url).path))
                status, headers, body = self.static_request(selected)
                self.assertEqual((status, body), (200, content))
                self.assertEqual(headers['Cache-Control'], 'no-store')
                self.assertEqual(int(headers['Content-Length']), len(content))
            for flavor in ('webgpu', 'webgl2'):
                module = urljoin(url, 'target/wasm32-unknown-unknown/release/' + flavor + '/gzweb.js')
                wasm = urljoin(module, 'gzweb_bg.wasm')
                status, _headers, body = self.static_request(wasm)
                expected_wasm = expected['target/wasm32-unknown-unknown/release/' + flavor + '/gzweb_bg.wasm']
                self.assertEqual((status, body), (200, expected_wasm))
            for old in prior:
                self.assertNotEqual(urlsplit(url).path, urlsplit(old).path)
                for name in expected:
                    status, headers, _body = self.static_request(urljoin(old, name))
                    self.assertEqual(status, 404)
                    self.assertEqual(headers['Cache-Control'], 'no-store')

        first, port = self.launch_static_viewer(visit)
        prior.append(first)
        second, same_port = self.launch_static_viewer(visit, port)
        self.assertEqual(same_port, port)
        self.assertNotEqual(first, second)
        self.assertEqual({name: (self.prefix / 'web' / name).read_bytes() for name in names}, expected)

    def test_future_cache_validators_still_return_full_200_get_and_head_on_fresh_and_root_routes(self):
        names = [name.removeprefix('web/') for name in self.files if name.startswith('web/')]

        def visit(url):
            paths = [(url, 'index.html'), (urljoin(url, '/'), 'index.html')]
            paths += [(urljoin(url, name), name) for name in names]
            paths += [(urljoin(url, '/' + name), name) for name in names]
            for selected, name in paths:
                expected = (self.prefix / 'web' / name).read_bytes()
                for validator in ({'If-Modified-Since': 'Fri, 01 Jan 2100 00:00:00 GMT'},
                                  {'If-None-Match': '"old-preview"'}):
                    for method in ('GET', 'HEAD'):
                        with self.subTest(path=urlsplit(selected).path, headers=validator, method=method):
                            status, headers, body = self.static_request(selected, method, validator)
                            self.assertEqual(status, 200)
                            self.assertEqual(headers['Cache-Control'], 'no-store')
                            self.assertEqual(int(headers['Content-Length']), len(expected))
                            self.assertEqual(body, expected if method == 'GET' else b'')

        self.launch_static_viewer(visit)

    def test_unknown_and_escaping_asset_routes_never_fall_back_to_root_files(self):
        def visit(url):
            prefix = urlsplit(url).path
            paths = ['/__gz_preview__', '/__gz_preview__/',
                     '/__gz_preview__/old-launch/startup.mjs',
                     '/__gz_preview__/old-launch/../../startup.mjs',
                     prefix.rstrip('/'), prefix + '../startup.mjs',
                     prefix + './startup.mjs', prefix + '%2e%2e/startup.mjs',
                     prefix + '..%2fstartup.mjs', prefix + '%2fstartup.mjs',
                     prefix + 'target//wasm32-unknown-unknown/release/webgpu/gzweb.js',
                     prefix + 'target\\..\\startup.mjs', prefix + '%00startup.mjs',
                     prefix.replace('/__gz_preview__/', '/%5f%5fgz_preview__/') + 'startup.mjs',
                     '/%2f__gz_preview__/old-launch/../../startup.mjs', '/../manifest.json']
            for path in paths:
                for method in ('GET', 'HEAD'):
                    with self.subTest(path=path, method=method):
                        target = 'http://127.0.0.1:' + str(urlsplit(url).port) + path
                        status, headers, body = self.static_request(target, method)
                        self.assertEqual(status, 404)
                        self.assertEqual(headers['Cache-Control'], 'no-store')
                        self.assertNotIn((self.prefix / 'web/startup.mjs').read_bytes(), body)
                        if method == 'HEAD':
                            self.assertEqual(body, b'')
            status, headers, body = self.static_request(urljoin(url, '/startup.mjs'))
            self.assertEqual((status, body), (200, (self.prefix / 'web/startup.mjs').read_bytes()))
            self.assertEqual(headers['Cache-Control'], 'no-store')

        self.launch_static_viewer(visit)

if __name__ == '__main__':
    unittest.main()
