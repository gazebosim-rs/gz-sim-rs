#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Capture a native CameraInfo wire on isolated held discovery ports."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import uuid

parser = argparse.ArgumentParser()
parser.add_argument("executable", type=Path)
parser.add_argument("wire", type=Path)
parser.add_argument("loader_log", type=Path)
parser.add_argument("--projection-offsets", action="store_true",
                    help="capture distinct P with signed metadata offsets; default recipe is unchanged")
parser.add_argument("--receipt", type=Path,
                    help="exclusive actual owned-child wait and capture receipt")
args = parser.parse_args()
held = []
ports = []
child = None
terminal = None
spawning = False
finalizing = False
complete = False
identity_before = None
wire = b""
log = b""
error = None
cleanup_errors = []
reaped = False
capture_complete = False
group_absent = False
pid_absent = False


def interrupted(number, _frame):
    global terminal
    terminal = number
    # Popen must return its handle before a signal can unwind. Cleanup cannot
    # be interrupted before the owned native child has been waited.
    if not spawning and not finalizing:
        raise InterruptedError(f"capture signal {number}")
    if complete:
        raise SystemExit(1)


def identity(pid):
    try:
        with open(f"/proc/{pid}/stat", "rb") as stream:
            raw = stream.read(8193)
        if len(raw) > 8192:
            raise ValueError("native stat cap")
        fields = raw.rsplit(b") ", 1)[1].split()
        with open(f"/proc/{pid}/status", "rb") as stream:
            status = stream.read(16385)
        if len(status) > 16384:
            raise ValueError("native status cap")
        uid = [int(v) for v in next(s for s in status.splitlines()
                                  if s.startswith(b"Uid:")).split()[1:]]
        return dict(pid=pid, start=int(fields[19]), state=fields[0].decode("ascii"),
                    parent=int(fields[1]), pgid=int(fields[2]), session=int(fields[3]), uid=uid)
    except FileNotFoundError:
        return None


def present_group(pid):
    try:
        os.killpg(pid, 0)
        return True
    except ProcessLookupError:
        return False


def retain(path, raw):
    # Existing default invocation still writes exactly the captured bytes. The
    # new executor supplies exclusive packet paths and requests an actual receipt.
    mode = "xb" if args.receipt is not None else "wb"
    with path.open(mode) as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
    return dict(path=str(path), bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest())


for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(number, interrupted)
try:
    for _ in range(2):
        port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        port.setsockopt(socket.IPPROTO_IP, 49, 0)  # Linux IP_MULTICAST_ALL
        port.bind(("0.0.0.0", 0))
        held.append(port)
    ports = [port.getsockname()[1] for port in held]
    if ports[0] == ports[1]:
        raise ValueError("distinct discovery ports required")
    env = os.environ.copy()
    for name in ("LD_PRELOAD", "LD_AUDIT", "LD_LIBRARY_PATH", "DISPLAY",
                 "WAYLAND_DISPLAY", "GZ_RELAY", "GZ_TRANSPORT_USERNAME",
                 "GZ_TRANSPORT_PASSWORD"):
        env.pop(name, None)
    env.update(GZ_PARTITION="camera_info_" + uuid.uuid4().hex, GZ_IP="127.0.0.1",
               GZ_DISCOVERY_MSG_PORT=str(ports[0]), GZ_DISCOVERY_SRV_PORT=str(ports[1]),
               GZ_DISCOVERY_MULTICAST_IP="239.255.0.7",
               GZ_TRANSPORT_TOPIC_STATISTICS="0", GZ_TRANSPORT_IMPLEMENTATION="zeromq",
               LD_DEBUG="libs")
    command = [str(args.executable.resolve()), "/camera_info_recipe"]
    if args.projection_offsets:
        command.append("--projection-offsets")
    spawning = True
    try:
        child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 env=env, start_new_session=True)
    finally:
        spawning = False
    if terminal is not None:
        raise InterruptedError(f"capture signal {terminal}")
    identity_before = identity(child.pid)
    if identity_before is not None:
        if (identity_before["parent"], identity_before["pgid"], identity_before["session"]) != (os.getpid(), child.pid, child.pid):
            raise ValueError("native owned session differs")
    wire, log = child.communicate(timeout=20)
    capture_complete = True
    reaped = True
    if child.returncode:
        raise RuntimeError(f"native CameraInfo probe exited {child.returncode}")
except BaseException as failure:
    if isinstance(failure, subprocess.TimeoutExpired):
        wire = failure.output or wire
        log = failure.stderr or log
    error = (type(failure).__name__ + ": " + str(failure))[:1024]
finally:
    finalizing = True
    if child is not None:
        try:
            if child.poll() is None:
                current = identity(child.pid)
                if current is None or (current["parent"], current["pgid"], current["session"]) != (os.getpid(), child.pid, child.pid):
                    raise RuntimeError("owned native group identity UNKNOWN; no signal sent")
                if identity_before is not None and current["start"] != identity_before["start"]:
                    raise RuntimeError("owned native start differs; no signal sent")
                os.killpg(child.pid, signal.SIGKILL)
            # Also completes output after timeout/signal. Never use an unbounded
            # second communicate or clear a handle on an uncertain wait.
            wire, log = child.communicate(timeout=3)
            capture_complete = True
            reaped = True
        except BaseException as failure:
            if isinstance(failure, subprocess.TimeoutExpired):
                wire = failure.output or wire
                log = failure.stderr or log
            cleanup_errors.append((type(failure).__name__ + ": " + str(failure))[:1024])
        if child.returncode is not None:
            reaped = True
        try:
            group_absent = not present_group(child.pid)
            pid_absent = identity(child.pid) is None
        except BaseException as failure:
            cleanup_errors.append((type(failure).__name__ + ": " + str(failure))[:1024])
    for port in held:
        try:
            port.close()
        except BaseException as failure:
            cleanup_errors.append((type(failure).__name__ + ": " + str(failure))[:1024])
    outputs = {}
    for label, path, raw in (("wire", args.wire, wire), ("loader", args.loader_log, log)):
        try:
            outputs[label] = retain(path, raw)
        except BaseException as failure:
            cleanup_errors.append((type(failure).__name__ + ": " + str(failure))[:1024])
    if len(wire) > 16384 or len(log) > 2 * 1024**2:
        cleanup_errors.append("captured wire/loader cap exceeded; not qualified")
    passed = (error is None and not cleanup_errors and terminal is None and child is not None and capture_complete
              and child.returncode == 0 and reaped and group_absent and pid_absent)
    if args.receipt is not None:
        value = dict(state="NATIVE_CAMERA_INFO_CAPTURE_CLEAR" if passed else "NATIVE_CAMERA_INFO_CAPTURE_FAILED",
                     mode="projection-offsets" if args.projection_offsets else "legacy-zero",
                     native_pid=child.pid if child is not None else None,
                     native_pgid=child.pid if child is not None else None,
                     identity_before=identity_before, actual_exit=child.returncode if child is not None else None,
                     reaped=reaped, group_absent=group_absent, pid_absent=pid_absent,
                     terminal_signal=terminal, error=error, cleanup_errors=cleanup_errors,
                     raw_retention_partial=not capture_complete, outputs=outputs,
                     discovery_ports=ports if len(ports) == 2 else None,
                     partition=env["GZ_PARTITION"] if child is not None else None,
                     native_timeout_seconds=20, cleanup_wait_seconds=3)
        with args.receipt.open("x") as stream:
            json.dump(value, stream, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
    complete = True
    finalizing = False
if not passed or terminal is not None:
    raise SystemExit(1)
