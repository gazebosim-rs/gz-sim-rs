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

    def test_full_server_requires_native_warmup(self):
        row = self.row()
        row['warmup_force_x'] = 0.0
        fixture_hash = hashlib.sha256(b'fixture').hexdigest()
        row['fixture_hex'] = b'fixture'.hex()
        pipeline.decode('PIPELINE ' + json.dumps(row), 2, fixture_hash, warmup_force_x=0.0)
        for wrong in [1000.0, float('nan'), float('inf'), -float('inf'), 1e-9]:
            row['warmup_force_x'] = wrong
            with self.subTest(wrong=wrong), self.assertRaises(ValueError):
                pipeline.decode('PIPELINE ' + json.dumps(row), 2, fixture_hash, warmup_force_x=0.0)

    def test_complete_stream_evidence_and_opaque_id_accounting(self):
        row = self.row()
        row.update(warmup_force_x=0.0, wall_nanos=40_000_100, pacing_wait_nanos=40_000_000, request_period_us=20000, clock_count=2, pose_count=2,
                   clock_time_sum=5_000_000, pose_time_sum=5_000_000,
                   clock_bytes=60, pose_bytes=100, pose_bytes_without_id_values=92,
                   pose_ids=dict(ground=4, ball=7, base=5, body=8))
        fixture_hash = hashlib.sha256(bytes.fromhex(row['fixture_hex'])).hexdigest()
        def decode(value):
            return pipeline.decode('PIPELINE ' + json.dumps(value), 2, fixture_hash,
                                   warmup_force_x=0.0, streams=True, request_period_us=20000)
        decode(row)
        for key, wrong in [('request_period_us', 1000), ('wall_nanos', 10), ('pacing_wait_nanos', 50_000_000), ('clock_count', 1), ('pose_count', 3),
                           ('clock_time_sum', 4_000_000), ('pose_time_sum', 0),
                           ('clock_bytes', 0), ('pose_bytes_without_id_values', 91),
                           ('pose_ids', dict(ground=4, ball=7, base=5, body=7)),
                           ('pose_ids', dict(ground=4, ball=7, base=5, body=True)),
                           ('pose_ids', dict(ground=4, ball=7, base=5, body=2**32)),
                           ('pose_ids', dict(ground=4, ball=7, base=5))]:
            with self.subTest(key=key, wrong=wrong), self.assertRaises(ValueError):
                decode(dict(row, **{key: wrong}))
        larger_ids = dict(row, pose_ids=dict(ground=4, ball=7, base=5, body=128),
                          pose_bytes_without_id_values=90)
        decode(larger_ids)
