#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Run the installed, loopback-only integrated preview without a source checkout."""
import argparse
import functools
import hashlib
import http.server
import json
import os
from pathlib import Path, PurePosixPath
import signal
import secrets
import shutil
import socket
import stat
import subprocess
import sys
import threading
import tempfile
from urllib.parse import unquote, urlsplit


M70_PROFILE = ('release; no-default-features; simulator rgb-camera; helpers native-free; '
               'optional packaged GPU worker')
M70_BINARY_ROLES = {
    'simulator': {'path': 'bin/gz-sim-rs', 'package': 'gz-sim-cli',
                  'target': 'gz-sim-rs', 'features': ['rgb-camera']},
    'drive_command': {'path': 'bin/gz-drive-command', 'package': 'gz-sim-server',
                      'target': 'drive_command', 'features': []},
    'joint_command': {'path': 'bin/gz-joint-command', 'package': 'gz-sim-server',
                      'target': 'joint_command', 'features': []},
    'camera_worker': {'path': 'bin/gz-render-worker', 'package': 'gz-render-worker',
                      'target': 'gz-render-worker', 'features': []},
}
CAMERA_ICD_DIRECTORY = Path('/usr/share/vulkan/icd.d')
CAMERA_LIBRARY_DIRECTORIES = (Path('/usr/lib/x86_64-linux-gnu'),
                              Path('/lib/x86_64-linux-gnu'))


def relative_file(name):
    if (not isinstance(name, str) or not name or "\\" in name or ":" in name
            or any(ord(character) < 32 for character in name)
            or any(part in {"", ".", ".."} for part in name.split("/"))):
        raise ValueError('invalid installed manifest path')
    path = PurePosixPath(name)
    if path.is_absolute():
        raise ValueError('invalid installed manifest path')
    return Path(*path.parts)


def verify_prefix(prefix):
    """Require the complete ordinary-file installation, including its notices."""
    if prefix.is_symlink() or not prefix.is_dir():
        raise ValueError('an ordinary installed prefix is required')
    manifest_path = prefix / 'manifest.json'
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise ValueError('missing or linked installed manifest')
    with manifest_path.open('rb') as stream:
        encoded = stream.read(16 * 1024 * 1024 + 1)
    if len(encoded) > 16 * 1024 * 1024:
        raise ValueError('installed manifest exceeds its bound')
    manifest = json.loads(encoded)
    if (type(manifest.get('schema_version')) is not int or manifest['schema_version'] != 1
            or not isinstance(manifest.get('files'), dict)
            or not 1 <= len(manifest['files']) <= 16384):
        raise ValueError('unsupported or incomplete preview manifest')
    files = manifest['files']
    camera_generation = manifest.get('guide_generation') == 'M70'
    if camera_generation and (manifest.get('target') != 'ubuntu-24.04-x86_64'
                              or manifest.get('profile') != M70_PROFILE
                              or manifest.get('binary_roles') != M70_BINARY_ROLES):
        raise ValueError('M70 requires the exact four installed binary roles')
    if 'manifest.json' in files:
        raise ValueError('manifest cannot list itself')
    required = {'bin/gz-preview', 'bin/gz-sim-rs', 'bin/gz-drive-command', 'bin/gz-joint-command',
                'share/robot.sdf', 'share/zenoh.json5', 'web/index.html', 'web/startup.mjs',
                'LICENSE', 'NOTICE', 'notices.json',
                *{'web/target/wasm32-unknown-unknown/release/' + flavor + '/' + name
                  for flavor in ('webgpu', 'webgl2') for name in ('gzweb.js', 'gzweb_bg.wasm')},
                *{'notices/' + name for name in ('licenses/Apache-2.0.txt', 'OSRF-NOTICE.txt',
                  'Geist-OFL.txt', 'Rust-COPYRIGHT.html', 'Rust-LICENSE-APACHE.txt',
                  'Rust-LICENSE-MIT.txt', 'Viewer-mesh-loader-LICENSE.txt', 'Viewer-proto-LICENSE.txt')}}
    if camera_generation:
        required.update({'bin/gz-render-worker', 'INSTALL.md', 'CAPABILITIES.md', 'PREVIEW.md'})
        for role in M70_BINARY_ROLES.values():
            entry = files.get(role['path'])
            if (not isinstance(entry, dict) or entry.get('mode') != 0o755
                    or type(entry.get('bytes')) is not int or not 0 < entry['bytes'] <= 256 * 1024**2):
                raise ValueError('M70 requires four bounded executable binary files')
    guide_pairs = ({'INSTALL.md', 'CAPABILITIES.md'},
                   {'M12_INSTALL.md', 'M12_CAPABILITIES.md'})
    if not required <= files.keys() or not any(pair <= files.keys() for pair in guide_pairs):
        raise ValueError('preview manifest omits required runtime, guide pair or notice files')
    directories = {'.'}
    for name in files:
        relative = relative_file(name)
        directories.update(str(parent) for parent in relative.parents)
    actual_files = set()
    for directory, children, names in os.walk(prefix, followlinks=False):
        parent = Path(directory)
        for name in [*children, *names]:
            path = parent / name
            info = path.lstat()
            if stat.S_ISLNK(info.st_mode):
                raise ValueError('linked installed file or directory: ' + str(path.relative_to(prefix)))
            if stat.S_ISDIR(info.st_mode):
                if path.relative_to(prefix).as_posix() not in directories:
                    raise ValueError('unlisted installed directory')
            elif stat.S_ISREG(info.st_mode):
                actual_files.add(path.relative_to(prefix).as_posix())
            else:
                raise ValueError('unsupported installed file type')
    if actual_files != set(files) | {'manifest.json'}:
        raise ValueError('installed file inventory differs from manifest')
    for name, entry in files.items():
        path = prefix / relative_file(name)
        with path.open('rb') as stream:
            digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        if (path.stat().st_size != entry['bytes'] or digest != entry['sha256']
                or stat.S_IMODE(path.stat().st_mode) != entry['mode']):
            raise ValueError('installed file differs from manifest: ' + name)
    notices = json.loads((prefix / 'notices.json').read_text())
    if notices.get('version') != 1 or not isinstance(notices.get('files'), list):
        raise ValueError('invalid installed notice inventory')
    notice_names = set()
    for entry in notices['files']:
        name = 'notices/' + relative_file(entry['path']).as_posix()
        if name in notice_names or name not in files:
            raise ValueError('invalid installed notice inventory')
        notice_names.add(name)
        if any(entry[key] != files[name][key] for key in ('bytes', 'sha256')):
            raise ValueError('installed notice differs from its inventory')
    if notice_names != {name for name in files if name.startswith('notices/')}:
        raise ValueError('incomplete installed notice inventory')
    return manifest


def example_path(prefix, manifest, basename):
    """Select only a direct copied SDF whose complete installation was verified."""
    relative = relative_file(basename)
    if relative.parts != (basename,) or not basename.endswith('.sdf'):
        raise ValueError('example must be a direct SDF basename')
    name = 'share/examples/' + basename
    if name not in manifest['files']:
        raise ValueError('example is not listed in the installed manifest')
    return prefix / name


def environment(prefix, partition):
    result = dict(os.environ)
    result['GZ_PARTITION'] = partition
    result['ZENOH_CONFIG'] = str(prefix / 'share/zenoh.json5')
    return result


def refuse_camera_overrides():
    overrides = sorted(name for name, value in os.environ.items() if value
                       and (name.startswith(('LD_', 'VK_', 'WGPU_', 'MESA_', 'GALLIUM_'))
                            or name == 'DRI_PRIME'))
    if overrides:
        raise ValueError('remove inherited camera/loader overrides: ' + ', '.join(overrides))


def camera_environment(prefix, partition, runtime):
    """Select one ordinary OS lavapipe provider; no inherited driver overrides.

    This is startup admission, not proof of the provider actually loaded or of
    resource fit. Those require the separate installed runtime observations.
    """
    refuse_camera_overrides()
    runtime = Path(runtime)
    info = runtime.lstat()
    if (not stat.S_ISDIR(info.st_mode) or runtime.is_symlink()
            or stat.S_IMODE(info.st_mode) != 0o700 or info.st_uid != os.geteuid()
            or any(parent.is_symlink() for parent in runtime.parents)):
        raise ValueError('camera runtime must be a private owned ordinary directory')
    directory = CAMERA_ICD_DIRECTORY
    if (not directory.is_dir()
            or any(path.is_symlink() for path in (directory, *directory.parents))):
        raise ValueError('ordinary system lavapipe ICD directory is required')
    candidates = list(directory.glob('lvp_icd*.json'))
    if len(candidates) != 1:
        raise ValueError('exactly one system lavapipe ICD is required')
    icd = candidates[0]
    before = icd.lstat()
    if not stat.S_ISREG(before.st_mode) or not 0 < before.st_size <= 65536:
        raise ValueError('ordinary bounded lavapipe ICD is required')
    fd = os.open(icd, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC)
    with os.fdopen(fd, 'rb') as stream:
        stamp = lambda value: (value.st_dev, value.st_ino, value.st_mode,
                                value.st_size, value.st_mtime_ns, value.st_ctime_ns)
        opened = os.fstat(stream.fileno())
        encoded = stream.read(65537)
        if (stamp(before) != stamp(opened) or len(encoded) != before.st_size
                or stamp(before) != stamp(os.fstat(stream.fileno()))
                or stamp(before) != stamp(icd.lstat())):
            raise ValueError('lavapipe ICD changed during admission')
    document = json.loads(encoded)
    record = document.get('ICD') if isinstance(document, dict) else None
    library = record.get('library_path') if isinstance(record, dict) else None
    if not isinstance(library, str) or not library or '\x00' in library:
        raise ValueError('lavapipe ICD lacks its actual library path')
    supplied = Path(library)
    allowed = {path.resolve() for path in CAMERA_LIBRARY_DIRECTORIES}
    if not supplied.is_absolute():
        raise ValueError('lavapipe ICD must select an absolute ordinary OS provider')
    provider = supplied.resolve()
    if (not supplied.is_file() or provider.name != 'libvulkan_lvp.so'
            or provider.parent not in allowed):
        raise ValueError('lavapipe OS provider is absent or outside its system directory')
    # Vulkan-only renderer Source is unchanged. Pin its one installed ICD and
    # keep its HOME/cache/runtime writes inside our owned, short-lived directory.
    result = {'PATH': '/usr/bin:/bin', 'LANG': 'C.UTF-8', 'LC_ALL': 'C',
              'HOME': str(runtime), 'XDG_RUNTIME_DIR': str(runtime),
              'VK_ICD_FILENAMES': str(icd), 'MESA_SHADER_CACHE_DISABLE': 'true',
              'GZ_PARTITION': partition, 'ZENOH_CONFIG': str(prefix / 'share/zenoh.json5')}
    return result


def serve(prefix, http_port, ws_port, partition, *, lifecycle=False, example=None,
          camera_output=False):
    if not camera_output:
        return _serve(prefix, http_port, ws_port, partition,
                      lifecycle=lifecycle, example=example)
    if lifecycle:
        raise ValueError('camera output cannot be combined with lifecycle editing')
    refuse_camera_overrides()
    manifest = verify_prefix(prefix)
    if manifest.get('guide_generation') != 'M70':
        raise ValueError('camera output requires the M70 packaged worker')
    worker = prefix / manifest['binary_roles']['camera_worker']['path']
    runtime = Path(tempfile.mkdtemp(prefix='gz-preview-camera-', dir='/tmp'))
    identity = None
    retirement = {'confirmed': True}
    try:
        identity = runtime.lstat()
        env = camera_environment(prefix, partition, runtime)
        return _serve(prefix, http_port, ws_port, partition, example=example,
                      worker=worker, child_environment=env, retirement=retirement)
    finally:
        primary = sys.exception()
        cleanup_error = None
        try:
            if identity is None:
                raise ValueError('camera runtime ownership is unconfirmed; retained: ' + str(runtime))
            current = runtime.lstat()
            if (not stat.S_ISDIR(current.st_mode)
                    or (current.st_dev, current.st_ino, current.st_uid,
                        stat.S_IMODE(current.st_mode))
                    != (identity.st_dev, identity.st_ino, identity.st_uid, 0o700)):
                raise ValueError('camera runtime identity changed; retained: ' + str(runtime))
            if not retirement['confirmed']:
                raise RuntimeError('camera child retirement is unconfirmed; runtime retained: '
                                   + str(runtime))
            shutil.rmtree(runtime)
        except BaseException as error:
            cleanup_error = error
        if cleanup_error is not None:
            if primary is not None:
                primary.add_note(str(cleanup_error))
                print(str(cleanup_error), file=sys.stderr)
            else:
                raise cleanup_error


def check_port(port):
    """Fail on an occupied local listener without touching its owner."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.bind(('127.0.0.1', port))


def stop_child(child):
    if child.poll() is not None:
        return
    # This process group was created exclusively for our simulator child.
    try:
        os.killpg(child.pid, signal.SIGINT)
    except ProcessLookupError:
        pass  # The child may have exited between poll and signal; still reap it.
    try:
        child.wait(timeout=10)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait(timeout=5)


class ViewerHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    """Keep each launch's relative module URLs inside its fresh asset path."""

    def __init__(self, *args, asset_prefix, **kwargs):
        self.asset_prefix = asset_prefix
        self.asset_relative_path = None
        super().__init__(*args, **kwargs)

    def send_head(self):
        self.asset_relative_path = None
        requested = urlsplit(self.path)
        if requested.scheme or requested.netloc:
            self.send_error(404, 'Unknown viewer asset path')
            return None
        path = requested.path
        try:
            decoded = unquote(path, errors='strict')
        except UnicodeError:
            self.send_error(404, 'Unknown viewer asset path')
            return None
        if ('\\' in decoded or any(ord(character) < 32 for character in decoded)
                or any(part in {'.', '..'} for part in decoded.split('/'))):
            self.send_error(404, 'Invalid viewer asset path')
            return None
        namespace_path = '/' + decoded.lstrip('/')
        if namespace_path == '/__gz_preview__' or namespace_path.startswith('/__gz_preview__/'):
            if (path != decoded or decoded != namespace_path
                    or not path.startswith(self.asset_prefix)):
                self.send_error(404, 'Unknown viewer asset path')
                return None
            relative = path[len(self.asset_prefix):]
            if relative and '' in relative.split('/'):
                self.send_error(404, 'Invalid viewer asset path')
                return None
            self.asset_relative_path = '/' + relative
        # A previous launch's validators must never turn this launch's body
        # into a 304. The directory token also isolates browser module caches.
        for name in ('If-Modified-Since', 'If-None-Match'):
            if name in self.headers:
                del self.headers[name]
        return super().send_head()

    def translate_path(self, path):
        return super().translate_path(self.asset_relative_path
                                      if self.asset_relative_path is not None else path)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()


def _serve(prefix, http_port, ws_port, partition, *, lifecycle=False, example=None,
           worker=None, child_environment=None, retirement=None):
    world = prefix / 'share/robot.sdf'
    if example is not None:
        world = example_path(prefix, verify_prefix(prefix), example)
    if http_port == ws_port or ws_port == 7449:
        raise ValueError('HTTP, WebSocket and Zenoh ports must be distinct')
    check_port(ws_port)
    check_port(7449)
    asset_prefix = '/__gz_preview__/' + secrets.token_hex(16) + '/'
    handler = functools.partial(ViewerHTTPRequestHandler, directory=str(prefix / 'web'),
                                asset_prefix=asset_prefix)
    server = http.server.ThreadingHTTPServer(('127.0.0.1', http_port), handler)
    child = None
    thread = None
    thread_started = False
    stopping = threading.Event()
    previous = {}

    def interrupt(_signum, _frame):
        stopping.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[signum] = signal.signal(signum, interrupt)
        command = [str(prefix / 'bin/gz-sim-rs'), str(world),
                   '--server', '--web-viewer', str(ws_port)]
        if lifecycle:
            command.append('--web-lifecycle')
        if worker is not None:
            command.extend(['--camera-worker', str(worker)])
        # Even a constructor exception can leave spawn outcome uncertain.
        if retirement is not None:
            retirement['confirmed'] = False
        child = subprocess.Popen(command, cwd=prefix,
                                 env=(environment(prefix, partition) if child_environment is None
                                      else child_environment),
                                 start_new_session=True)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        thread_started = True
        url = (f'http://127.0.0.1:{http_port}{asset_prefix}'
               f'?backend=primitive&ui=full&url=ws://127.0.0.1:{ws_port}/')
        if lifecycle:
            url += '&lifecycle=1'
        print('Open ' + url, flush=True)
        print('The robot starts paused. Use Play before sending a finite drive command.', flush=True)
        while child.poll() is None and not stopping.wait(0.1):
            pass
        if stopping.is_set():
            return 0
        return child.returncode if child.returncode >= 0 else 128 - child.returncode
    finally:
        primary = sys.exception()
        errors = []
        if child is not None:
            try:
                stop_child(child)
                if retirement is not None:
                    if child.poll() is None:
                        raise RuntimeError('owned camera child has not retired')
                    retirement['confirmed'] = True
            except BaseException as error:
                errors.append(error)
        if thread_started:
            for cleanup in (server.shutdown, lambda: thread.join(timeout=5)):
                try:
                    cleanup()
                except BaseException as error:
                    errors.append(error)
        try:
            server.server_close()
        except BaseException as error:
            errors.append(error)
        for signum, original in previous.items():
            try:
                signal.signal(signum, original)
            except BaseException as error:
                errors.append(error)
        if errors:
            for error in errors:
                print('Preview cleanup failed: ' + repr(error), file=sys.stderr)
            if primary is None:
                primary = errors.pop(0)
                for error in errors:
                    primary.add_note('additional cleanup failure: ' + repr(error))
                raise primary
            for error in errors:
                primary.add_note('cleanup failure: ' + repr(error))


def main(argv=None, prefix=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--partition', default='gz-preview')
    commands = parser.add_subparsers(dest='command', required=True)
    run = commands.add_parser('run', help='start the paused robot and local browser server')
    run.add_argument('--http-port', type=int, choices=(8080, 8081, 8082), default=8081)
    run.add_argument('--ws-port', type=int, default=9014)
    run.add_argument('--camera-output', action='store_true',
                     help='use the manifest-verified M70 camera worker and system lavapipe')
    run.add_argument('--lifecycle', action='store_true',
                     help='enable existing local lifecycle controls and publication credit')
    run.add_argument('--example', metavar='BASENAME',
                     help='select a manifest-listed copied share/examples/*.sdf basename')
    drive = commands.add_parser('drive', help='send a finite canonical drive command, then brake')
    drive.add_argument('--topic', metavar='KEY',
                       help='explicit concrete command topic; default is the M8 robot topic')
    drive.add_argument('linear_m_per_s')
    drive.add_argument('yaw_rad_per_s')
    drive.add_argument('duration_seconds')
    commands.add_parser('verify', help='verify installed files without launching the simulator')
    args = parser.parse_args(argv)
    root = Path(prefix) if prefix is not None else Path(__file__).resolve().parent.parent
    try:
        manifest = verify_prefix(root)
        if not args.partition or any(character.isspace() for character in args.partition):
            raise ValueError('a nonempty partition without whitespace is required')
        if args.command == 'verify':
            print('Installed preview files verified.')
            return 0
        if args.command == 'drive':
            # Replace the wrapper so a signal cannot orphan a still-driving helper.
            helper = str(root / 'bin/gz-drive-command')
            command = [helper, 'tcp/127.0.0.1:7449', args.linear_m_per_s,
                       args.yaw_rad_per_s, args.duration_seconds]
            if args.topic is not None:
                command.extend(['--topic', args.topic])
            return os.execve(helper, command, environment(root, args.partition))
        if not 1024 <= args.ws_port <= 65535:
            raise ValueError('WebSocket port must be in 1024..=65535')
        if args.camera_output and args.lifecycle:
            raise ValueError('camera output cannot be combined with lifecycle editing')
        options = {}
        if args.camera_output:
            if manifest.get('guide_generation') != 'M70':
                raise ValueError('camera output requires the M70 packaged worker')
            options['camera_output'] = True
        if args.lifecycle:
            options['lifecycle'] = True
        if args.example is not None:
            example_path(root, manifest, args.example)
            options['example'] = args.example
        return serve(root, args.http_port, args.ws_port, args.partition, **options)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print('Preview failed: ' + str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
