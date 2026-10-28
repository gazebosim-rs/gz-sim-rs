#!/usr/bin/env python3
# Copyright (C) 2026 Open Source Robotics Foundation
# SPDX-License-Identifier: Apache-2.0
"""Build MuJoCo at the commit upstream gz-physics vendors, with its CMake options.

Parity fixtures compare to 1e-14, so the library must match upstream's build
rather than a prebuilt release. Prints the install prefix for MUJOCO_PREFIX.
"""
import argparse
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent.parent
# gz-physics third_party/mujoco_vendor/CMakeLists.txt: GIT_TAG for 3.11.0.
MUJOCO_COMMIT = 'b85fdca54f0e0038b804af146a0b4e94199e00d0'
MUJOCO_VERSION = '3.11.0'


def run(command, **kwargs):
    print(' '.join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=ROOT / 'artifacts/physics-reference')
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    root = args.root.resolve()
    prefix = root / 'mujoco'
    library = prefix / 'lib' / f'libmujoco.so.{MUJOCO_VERSION}'
    if library.exists():
        print(prefix)
        return
    source = root / 'mujoco-source'
    if not source.exists():
        run(['git', 'init', '-q', source])
        run(['git', '-C', source, 'fetch', '-q', '--depth', '1',
             'https://github.com/google-deepmind/mujoco', MUJOCO_COMMIT])
        run(['git', '-C', source, 'checkout', '-q', 'FETCH_HEAD'])
    head = subprocess.run(['git', '-C', source, 'rev-parse', 'HEAD'],
                          check=True, capture_output=True, text=True).stdout.strip()
    if head != MUJOCO_COMMIT:
        raise SystemExit(f'MuJoCo source is at {head}, expected {MUJOCO_COMMIT}')
    build = root / 'mujoco-build'
    # Same options as the upstream vendor build, including its GCC warning relaxation.
    run(['cmake', '-S', source, '-B', build, '-Wno-dev',
         f'-DCMAKE_INSTALL_PREFIX={prefix}', '-DCMAKE_INSTALL_LIBDIR=lib',
         '-DCMAKE_BUILD_TYPE=Release', '-DBUILD_SHARED_LIBS=ON',
         '-DMUJOCO_BUILD_EXAMPLES=OFF', '-DMUJOCO_BUILD_TESTS=OFF',
         '-DMUJOCO_BUILD_SIMULATE=OFF', '-DMUJOCO_TEST_PYTHON_UTIL=OFF',
         '-DCMAKE_C_FLAGS=-Wno-error=discarded-qualifiers',
         '-DCMAKE_CXX_FLAGS=-Wno-error=discarded-qualifiers'])
    run(['cmake', '--build', build, '--parallel', args.jobs])
    run(['cmake', '--install', build])
    if not library.exists():
        raise SystemExit(f'MuJoCo install is missing {library}')
    print(prefix)


if __name__ == '__main__':
    main()
