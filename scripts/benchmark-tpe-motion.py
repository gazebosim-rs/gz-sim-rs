#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Compare the real Rust and native TPE pose-update programs at identical work."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate(result):
    angle = math.hypot(0.1, 0.2, 0.3)
    factor = math.sin(angle / 2) / angle
    expected = [1.0, 2.5, 3.25, 0.1 * factor, 0.2 * factor, 0.3 * factor, math.cos(angle / 2)]
    observed = result['position'] + result['orientation']
    if len(observed) != len(expected) or any(not abs(a - b) < 1e-12 for a, b in zip(observed, expected)):
        raise ValueError('final pose does not match 1000 one-millisecond updates')
    checksum = 0.0
    for index in range(1000):
        for value in [index * 0.001 + 1.0, 2.5, 3.25, expected[-1]]:
            checksum += value
    if not abs(result['checksum'] - checksum) < 1e-8 or result['nanos'] <= 0:
        raise ValueError('invalid checksum or duration')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['native', 'rust', 'reference-build', 'output']:
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--dependency-lib-dir', action='append', type=Path, default=[])
    args = parser.parse_args()
    cpu = min(os.sched_getaffinity(0))
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = os.pathsep.join([
        str(args.reference_build.resolve() / 'lib'),
        *(str(path.resolve()) for path in args.dependency_lib_dir),
        env.get('LD_LIBRARY_PATH', '')])
    programs = {'native': args.native.resolve(), 'rust': args.rust.resolve()}
    samples = []
    for index in range(8):
        pair = {}
        order = ['native', 'rust'] if index % 2 == 0 else ['rust', 'native']
        for name in order:
            run = subprocess.run(['taskset', '-c', str(cpu), str(programs[name])], env=env,
                                 capture_output=True, text=True, check=True)
            pair[name] = json.loads(run.stdout)
            validate(pair[name])
        if not abs(pair['native']['checksum'] - pair['rust']['checksum']) < 1e-8:
            raise ValueError('native/Rust checksums disagree')
        samples.append(pair)
    linkage = subprocess.run(['ldd', str(programs['native'])], env=env,
                             capture_output=True, text=True, check=True).stdout
    expected_library = args.reference_build.resolve() / 'lib/libgz-physics-tpelib.so'
    if str(expected_library) not in linkage:
        raise ValueError('native program did not load the requested reference library')
    libraries = {}
    for token in linkage.split():
        path = Path(token)
        if token.startswith('/') and path.is_file():
            libraries[path.name] = digest(path)
    medians = {name: statistics.median(sample[name]['nanos'] / 1_000_000 for sample in samples[1:])
               for name in programs}
    model = next((line.split(':', 1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines()
                  if line.startswith('model name')), 'unknown')
    proof = {
        'operation': '1000 centered model poses x 1000 updates at 1 ms; setup outside timing',
        'platform': platform.machine(), 'cpu_model': model, 'cpu_affinity': cpu,
        'warmup_samples': 1, 'measured_samples': 7, 'samples': samples,
        'median_ns_per_body_step': medians, 'native_over_rust': medians['native'] / medians['rust'],
        'source_sha256': {str(path.relative_to(ROOT)): digest(path) for path in [
            ROOT / 'crates/gz-physics-tpe/src/lib.rs',
            ROOT / 'crates/gz-physics-tpe/examples/motion_benchmark.rs',
            ROOT / 'crates/gz-physics-tpe/tests/native_motion_benchmark.cc', Path(__file__).resolve()]},
        'binary_sha256': {name: digest(path) for name, path in programs.items()},
        'linked_libraries_sha256': libraries,
        'qualification': 'Exploratory CPU-pinned alternating samples on a shared host. Native Release shared-library and Rust release static packaging/compilers differ. Rust validates construction, preserves error atomicity and bounds unit drift. No ECS, contacts, sensors, transport or whole-simulator speedup/P2 acceptance.'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(proof, indent=2) + '\n')
    print(json.dumps({'median_ns_per_body_step': medians, 'native_over_rust': proof['native_over_rust']}))


if __name__ == '__main__':
    main()
