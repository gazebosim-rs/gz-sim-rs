#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Replay the original ground-contact fixture using the shared pinned SDK provenance checks."""
import importlib.util
import hashlib
from pathlib import Path

if __name__ == '__main__':
    fixture = Path(__file__).with_name('apply_joint_force.sdf')
    assert hashlib.sha256(fixture.read_bytes()).hexdigest() == (
        'a25de92ae735a8b2b349a3e180928c1941153be7a787a1dcd54076c7d5e4d323'
    ), 'original apply_joint_force.sdf changed'
    spec = importlib.util.spec_from_file_location(
        'probe', Path(__file__).with_name('probe-moving-parent.py'))
    probe = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(probe)
    probe.main('ground_contact')
