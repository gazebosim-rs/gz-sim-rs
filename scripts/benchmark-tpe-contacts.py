#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Compare matching kinematic TPE motion/contact work, excluding construction."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate(value):
    if (value['models'], value['steps'], value['contacts']) != (1000, 500, 250):
        raise ValueError('work or contact count differs')
    if value['nanos'] <= 0:
        raise ValueError('invalid elapsed time')
    positions = sum((i // 2) * 3 + (i % 2) * 0.75 +
                    (0.005 if (i // 2) % 2 == 0 else 0) for i in range(1000))
    contacts = sum(i * 3 + 0.375 + 0.005 for i in range(0, 500, 2))
    for key, expected in [('position_checksum', positions), ('contact_checksum', contacts)]:
        if not math.isfinite(value[key]) or abs(value[key] - expected) >= 1e-6:
            raise ValueError(f'{key} differs from analytic state')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['native', 'rust', 'reference-build', 'reference-source', 'output']:
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
    flags = shlex.split(subprocess.check_output(
        ['pkg-config', '--cflags', '--libs', 'gz-math', 'gz-utils', 'gz-common-graphics'],
        env=env, text=True))
    command = ['c++', '-std=c++17', '-O3', '-Wall', '-Wextra', '-Werror',
               '-I' + str(args.reference_source.resolve() / 'tpe/lib/src'),
               '-I' + str(args.reference_build.resolve() / 'include'),
               str(ROOT / 'crates/gz-physics-tpe/tests/native_contact_benchmark.cc'),
               '-L' + str(args.reference_build.resolve() / 'lib'), '-lgz-physics-tpelib',
               *flags, '-o', str(programs['native'])]
    programs['native'].parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(command, env=env, check=True)
    samples = []
    for trial in range(8):
        pair = {}
        order = ['native', 'rust'] if trial % 2 == 0 else ['rust', 'native']
        for name in order:
            result = subprocess.run(['taskset', '-c', str(cpu), str(programs[name])],
                                    env=env, capture_output=True, text=True, check=True)
            pair[name] = json.loads(result.stdout)
            validate(pair[name])
        for key in ['position_checksum', 'contact_checksum']:
            if abs(pair['native'][key] - pair['rust'][key]) >= 1e-6:
                raise ValueError('native/Rust state differs')
        samples.append(pair)
    linkage = subprocess.check_output(['ldd', str(programs['native'])], env=env, text=True)
    if str(args.reference_build.resolve() / 'lib/libgz-physics-tpelib.so') not in linkage:
        raise ValueError('native program did not load requested reference library')
    libraries = {token: digest(Path(token)) for token in linkage.split()
                 if token.startswith('/') and Path(token).is_file()}
    medians = {name: statistics.median(s[name]['nanos'] for s in samples[1:])
               for name in programs}
    sources = [*sorted((ROOT / 'crates/gz-physics-tpe/src').glob('*.rs')),
               ROOT / 'crates/gz-physics-tpe/examples/contact_benchmark.rs',
               ROOT / 'crates/gz-physics-tpe/tests/native_contact_benchmark.cc',
               ROOT / 'Cargo.lock', ROOT / 'Cargo.toml', ROOT / 'crates/gz-physics-tpe/Cargo.toml',
               *sorted((ROOT / 'crates/gz-sim-core/src').glob('*.rs')),
               ROOT / 'crates/gz-physics/src/lib.rs', Path(__file__).resolve()]
    report = {
        'scope': '1000 model boxes, half static, 250 dynamic overlapping pairs, 500 one-ms steps after two zero-dt priming steps; setup/priming excluded; both programs validate every final pose/pair outside timing',
        'qualification': 'Author measurements on a shared host; core TPE motion/contact work only, without sensors/scene publication/transport. Native profiling is disabled; native dynamic-library versus Rust static packaging and compiler/ISA settings differ. No whole-simulator/P2/release performance acceptance.',
        'cpu': cpu, 'platform': platform.platform(),
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'reference_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=args.reference_source, text=True).strip(),
        'rust_build': ['cargo', 'build', '--locked', '--release', '-p', 'gz-physics-tpe', '--example', 'contact_benchmark'],
        'rust_flags': {name: env.get(name) for name in ['RUSTFLAGS', 'CARGO_ENCODED_RUSTFLAGS']},
        'cpu_model': next(line.split(':', 1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name')),
        'native_compile': command,
        'rustc': subprocess.check_output(['rustc', '-vV'], text=True),
        'cxx': subprocess.check_output(['c++', '--version'], text=True),
        'cmake_cache_sha256': digest(args.reference_build / 'CMakeCache.txt'),
        'native_flags': {str(path.relative_to(args.reference_build)): path.read_text()
                         for path in sorted(args.reference_build.rglob('flags.make'))
                         if 'gz-physics-tpelib.dir' in str(path)},
        'sources': {str(path.relative_to(ROOT)): digest(path) for path in sources},
        'binaries': {name: digest(path) for name, path in programs.items()},
        'linked_libraries': libraries, 'samples_including_first_warmup': samples,
        'median_nanos': medians, 'native_over_rust_ratio': medians['native'] / medians['rust']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'median_nanos': medians, 'ratio': report['native_over_rust_ratio']}))


if __name__ == '__main__':
    main()
