# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('pipeline', Path(__file__).resolve().parents[1] / 'scripts/benchmark-headless-pipeline.py')
pipeline = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pipeline)


class PipelineEvidence(unittest.TestCase):
    def row(self):
        return dict(steps=2, nanos=10, logical_bytes=42, time_sum=5_000_000,
                    force_sum=19.62, final_x=0.003, fixture_hex=b'fixture'.hex())

    def decode(self, row):
        return pipeline.decode('PIPELINE ' + json.dumps(row), 2, hashlib.sha256(b'fixture').hexdigest())

    def test_complete_work_is_required(self):
        self.assertEqual(self.decode(self.row()), self.row())
        for key, value in [('steps', 1), ('nanos', 0), ('nanos', float('nan')),
                           ('logical_bytes', 0), ('time_sum', 0), ('force_sum', 0),
                           ('force_sum', float('nan')), ('final_x', float('nan')),
                           ('final_x', 0.004), ('fixture_hex', b'other'.hex())]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.decode(dict(self.row(), **{key: value}))

    def test_missing_and_duplicate_results_fail(self):
        expected = hashlib.sha256(b'fixture').hexdigest()
        for output in ['', 'PIPELINE {}\nPIPELINE {}']:
            with self.subTest(output=output), self.assertRaises(ValueError):
                pipeline.decode(output, 2, expected)
