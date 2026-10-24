#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Cargo invokes this only for owned workspace crates."""
import os
import sys

os.execvp(sys.argv[1], [*sys.argv[1:], "-C", "instrument-coverage"])
