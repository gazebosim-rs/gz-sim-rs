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
import socket
import stat
import subprocess
import sys
import threading


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
    if 'manifest.json' in files:
        raise ValueError('manifest cannot list itself')
    required = {'bin/gz-preview', 'bin/gz-sim-rs', 'bin/gz-drive-command',
                'share/robot.sdf', 'share/zenoh.json5', 'web/index.html', 'web/startup.mjs',
                'LICENSE', 'NOTICE', 'notices.json',
                *{'web/target/wasm32-unknown-unknown/release/' + flavor + '/' + name
                  for flavor in ('webgpu', 'webgl2') for name in ('gzweb.js', 'gzweb_bg.wasm')},
                *{'notices/' + name for name in ('licenses/Apache-2.0.txt', 'OSRF-NOTICE.txt',
                  'Geist-OFL.txt', 'Rust-COPYRIGHT.html', 'Rust-LICENSE-APACHE.txt',
                  'Rust-LICENSE-MIT.txt', 'Viewer-mesh-loader-LICENSE.txt', 'Viewer-proto-LICENSE.txt')}}
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


def environment(prefix, partition):
    result = dict(os.environ)
    result['GZ_PARTITION'] = partition
    result['ZENOH_CONFIG'] = str(prefix / 'share/zenoh.json5')
    return result


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


def serve(prefix, http_port, ws_port, partition, *, lifecycle=False):
    if http_port == ws_port or ws_port == 7449:
        raise ValueError('HTTP, WebSocket and Zenoh ports must be distinct')
    check_port(ws_port)
    check_port(7449)
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(prefix / 'web'))
    server = http.server.ThreadingHTTPServer(('127.0.0.1', http_port), handler)
    child = None
    thread = None
    stopping = threading.Event()
    previous = {}

    def interrupt(_signum, _frame):
        stopping.set()

    try:
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            previous[signum] = signal.signal(signum, interrupt)
        command = [str(prefix / 'bin/gz-sim-rs'), str(prefix / 'share/robot.sdf'),
                   '--server', '--web-viewer', str(ws_port)]
        if lifecycle:
            command.append('--web-lifecycle')
        child = subprocess.Popen(command, cwd=prefix, env=environment(prefix, partition),
                                 start_new_session=True)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f'http://127.0.0.1:{http_port}/?backend=primitive&ui=full&url=ws://127.0.0.1:{ws_port}/'
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
        if child is not None:
            stop_child(child)
        if thread is not None:
            server.shutdown()
            thread.join(timeout=5)
        server.server_close()
        for signum, original in previous.items():
            signal.signal(signum, original)


def main(argv=None, prefix=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--partition', default='gz-preview')
    commands = parser.add_subparsers(dest='command', required=True)
    run = commands.add_parser('run', help='start the paused robot and local browser server')
    run.add_argument('--http-port', type=int, choices=(8080, 8081, 8082), default=8081)
    run.add_argument('--ws-port', type=int, default=9014)
    run.add_argument('--lifecycle', action='store_true',
                     help='enable existing local lifecycle controls and publication credit')
    drive = commands.add_parser('drive', help='send a finite canonical drive command, then brake')
    drive.add_argument('linear_m_per_s')
    drive.add_argument('yaw_rad_per_s')
    drive.add_argument('duration_seconds')
    commands.add_parser('verify', help='verify installed files without launching the simulator')
    args = parser.parse_args(argv)
    root = Path(prefix) if prefix is not None else Path(__file__).resolve().parent.parent
    try:
        verify_prefix(root)
        if not args.partition or any(character.isspace() for character in args.partition):
            raise ValueError('a nonempty partition without whitespace is required')
        if args.command == 'verify':
            print('Installed preview files verified.')
            return 0
        if args.command == 'drive':
            # Replace the wrapper so a signal cannot orphan a still-driving helper.
            helper = str(root / 'bin/gz-drive-command')
            return os.execve(helper, [helper, 'tcp/127.0.0.1:7449', args.linear_m_per_s,
                                     args.yaw_rad_per_s, args.duration_seconds],
                             environment(root, args.partition))
        if not 1024 <= args.ws_port <= 65535:
            raise ValueError('WebSocket port must be in 1024..=65535')
        if args.lifecycle:
            return serve(root, args.http_port, args.ws_port, args.partition, lifecycle=True)
        return serve(root, args.http_port, args.ws_port, args.partition)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print('Preview failed: ' + str(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
