#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Capture a native CameraInfo wire on isolated held discovery ports."""
import argparse
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
args = parser.parse_args()
held = []
try:
    for _ in range(2):
        port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        port.setsockopt(socket.IPPROTO_IP, 49, 0)  # Linux IP_MULTICAST_ALL
        port.bind(("0.0.0.0", 0))
        held.append(port)
    ports = [port.getsockname()[1] for port in held]
    assert ports[0] != ports[1]
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
    child = subprocess.Popen([str(args.executable.resolve()), "/camera_info_recipe"],
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             env=env, start_new_session=True)
    try:
        wire, log = child.communicate(timeout=20)
    except BaseException:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        finally:
            child.communicate()
        raise
    args.wire.write_bytes(wire)
    args.loader_log.write_bytes(log)
    if child.returncode:
        raise SystemExit(f"native CameraInfo probe exited {child.returncode}")
finally:
    for port in held:
        port.close()
