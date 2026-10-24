#!/usr/bin/env python3
"""Cargo invokes this only for owned workspace crates."""
import os
import sys

os.execvp(sys.argv[1], [*sys.argv[1:], "-C", "instrument-coverage"])
