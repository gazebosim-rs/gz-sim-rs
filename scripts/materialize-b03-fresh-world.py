#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Extract fresh B03 states while retaining the original cached reference.

No simulator or compiler runs. The unchanged diagnostic validator checks all
101 previous cached frames before extracting actual fresh point observations.
"""
import argparse
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', type=Path)
    parser.add_argument('previous_prefix', type=Path)
    parser.add_argument('provenance', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    old = module('cached_b03', ROOT / 'scripts/materialize-b03-world.py')
    inspector_path = ROOT / 'scripts/inspect-b03-fresh-world.py'
    inspector = module('fresh_b03', inspector_path)
    source = ROOT / 'reference/b03_world_fresh.cc'
    historical = ROOT / 'crates/gz-sim-scene/tests/fixtures/b03_world_native.json'
    suffixes = ('.state', '.input.sdf', '.maps', '.metrics.json')
    captures = {suffix: args.prefix.with_suffix(suffix) for suffix in suffixes}
    previous = args.previous_prefix.with_suffix('.state')
    fidelity_path = args.prefix.parent / 'native-fidelity.json'
    paths = [args.provenance, historical, source, inspector_path, previous,
             fidelity_path, *captures.values()]
    old.require(all(p.is_file() and not p.is_symlink() for p in paths), 'missing regular capture input')
    before = {p: p.read_bytes() for p in paths}
    provenance = old.parse(before[args.provenance])
    cached = old.parse(before[historical])
    old.require(provenance.get('failure') is None and provenance.get('traceback') is None,
                'failed capture')
    old.require(provenance['state'] == 'Authoritative native query diagnostic collected; all101 cached observer fidelity passed against completed capture; savedRust101 comparison only; no acceptance claimed',
                'capture did not complete authoritative fidelity checks')
    old.require(provenance['generator_sha256'] == old.digest(before[source]), 'generator changed')
    old.require(provenance['validator_sha256'] == old.digest(before[inspector_path]), 'validator changed')
    old.require(provenance['native_baseline_sha256'] == old.digest(before[historical]), 'cached reference changed')
    old.require(provenance['fidelity_sha256'] == old.digest(before[fidelity_path]), 'fidelity report changed')
    old.require(provenance['reused_completed_inputs']['/probe-source/previous-native.state']
                == old.digest(before[previous]), 'previous 101-frame capture changed')
    for suffix, path in captures.items():
        old.require(provenance['observations_sha256'][suffix] == old.digest(before[path]),
                    'capture changed: ' + suffix)
    old.require(old.digest(before[captures['.input.sdf']]) == old.XML_SHA256, 'B03 XML changed')
    observed = inspector.inspect(SimpleNamespace(native_prefix=args.prefix,
        native_baseline=historical, previous_native_prefix=args.previous_prefix, native_only=True))
    frames = [frame for frame in observed['fresh_frames'] if frame['tick'] in (1, 10, 100)]
    old.require([frame['tick'] for frame in frames] == [1, 10, 100], 'snapshot scope changed')
    result = {'schema_version': 1, 'license': 'Apache-2.0',
        'copyright': 'Copyright (C) 2026 Open Source Robotics Foundation',
        'scope': 'Actual fresh engine point observations; no Rust parity or performance acceptance',
        'xml_sha256': old.XML_SHA256, 'body_fields': old.FIELDS,
        'generator_sha256': old.digest(before[source]),
        'validator_sha256': old.digest(before[inspector_path]),
        'provenance_sha256': old.digest(before[args.provenance]),
        'historical_cached_fixture_sha256': old.digest(before[historical]),
        'observations_sha256': provenance['observations_sha256'],
        'all101_cached_maximum_delta': observed['all101_cached_maximum_delta'],
        'native_selected_maximum_delta': observed['native_maximum_delta'],
        'frames': frames}
    old.require(all(p.read_bytes() == data for p, data in before.items()), 'inputs changed during extraction')
    with args.output.open('x') as output:
        output.write(json.dumps(result, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
