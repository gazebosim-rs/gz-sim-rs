#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
# Fault fixture: valid framing and data, deliberately wrong captured frame kind.
import json
import struct
import sys

def send(value):
    raw = json.dumps(value, separators=(",", ":")).encode()
    sys.stdout.buffer.write(struct.pack("<I", len(raw)) + raw)
    sys.stdout.buffer.flush()

send({"Ready": {"version": 3}})
stamp = None
depth = False
while True:
    header = sys.stdin.buffer.read(4)
    if not header:
        break
    if len(header) != 4:
        raise SystemExit(2)
    size, = struct.unpack("<I", header)
    raw = sys.stdin.buffer.read(size)
    if len(raw) != size:
        raise SystemExit(2)
    request = json.loads(raw)
    if request == "Shutdown":
        break
    kind, fields = next(iter(request.items()))
    if kind in ("Snapshot", "CalibratedSnapshot", "CalibratedDepthSnapshot"):
        stamp = fields["stamp"]
        depth = kind == "CalibratedDepthSnapshot"
        send({"SnapshotAccepted": {"request_id": fields["request_id"]}})
    elif kind == "Capture":
        fields = {"request_id": fields["request_id"], "stamp": stamp}
        if depth:
            fields["rgba"] = [0] * (64 * 64 * 4)
            send({"Frame": fields})
        else:
            fields["metres_le"] = list(struct.pack("<f", 1.25) * (64 * 64))
            send({"DepthFrame": fields})
