#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Replay the exact earlier static-contact refusal inputs using the shared pinned SDK provenance checks."""
import importlib.util
from pathlib import Path

if __name__ == '__main__':
    spec = importlib.util.spec_from_file_location(
        'probe', Path(__file__).with_name('probe-moving-parent.py'))
    probe = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(probe)
    probe.main('static_contacts')
