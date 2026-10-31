#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Forward the retained clock fixture to the explicit selected Zenoh builder."""
import os
from pathlib import Path
import sys

root = Path(sys.argv[1]).resolve(strict=True)
os.execv(sys.executable, [sys.executable, str(root / 'scripts/native-peer.py'),
                         '--client', 'clock', '--profile', 'gazebo-zenoh-client-v1', '--json'])
