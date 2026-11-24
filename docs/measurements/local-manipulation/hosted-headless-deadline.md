<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Hosted headless coverage deadline

The headless test command at `71cd07a7d860a7f4f344f29eb14becd101e03d1a` exceeded its 3,600-second wall deadline. Its partial transcript contains 3,453 passing tests and zero failures across 762 completed libtest invocations. Cargo reported 13 minutes 27 seconds of compilation. The completed harnesses reported 2,770.98 seconds of testing. The final three-test garment-grasp harness had just started when the command expired. The renderer profile and per-crate coverage thresholds had yet to run. [Hosted verification job](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/45374505696/job/138376987349)

The longest completed harness was the CLI's `local_robot_foundations` example. Its 123 passing cases and one ignored case took 1,412.14 seconds on GitHub. On a local four-core comparison, four independent folded-sheet cases took 55.22 seconds as threads of one instrumented process and 36.64 seconds as separate processes using the same executable. The lower CPU time is consistent with less contention on LLVM counters shared by test threads. These timings include all solver work.

## Measured full-target comparison

All runs used Rust 1.97.1 on an AMD Ryzen 9 9950X3D, test optimization level 1, debug assertions and overflow checks. Each run was limited to four physical cores. The first three runs used the same workspace-instrumented ELF (SHA-256 `a2c6ee4755f5f62e24aa3a5eba4dba924915feaa6f129c7333d77573146350c9`) from the original checkout, built with the CLI's default features. Each executed 123 cases and preserved the original single ignored case. The machine also ran unrelated work.

| Run | CPU affinity | Wall seconds | User CPU seconds |
| --- | --- | ---: | ---: |
| Ordinary libtest | 0–3 | 504.68 | 1,600.98 |
| Isolated cases | 10–13 | 326.39 | 1,048.57 |
| Isolated cases, repeated on the original cores | 0–3 | 307.60 | 1,098.33 |

The same-core repeat reduced wall time by 39.05%. All three runs produced byte-identical sparse merged profiles (SHA-256 `8b9022f680f9f09a3d9cfd68eb5b7d4e9deee3e1a1b8b7adbde1d3c56cc6b39a`). Their exports agree on all 617 mapped files, every segment's hit/miss status, and the hit/miss status of all mapped regions in 16,478 functions. The same object contributed 22,754/143,765 lines and 37,623/232,252 regions in both runs. Those denominators include generated and example code and measure only this test executable. For all 616 mapped files in the source checkout, the counts were 22,754/141,446 and 37,623/229,181. The profile directory grew from 1,073,792 raw bytes in one file to 134,224,000 bytes in 125 files (one listing plus 124 cases). The normal profile merger consumes all of them.

A separate real Cargo invocation selected the two hosted features, `gz-sim-cli/rgb-camera` and `gz-sim-cli/landing-audit`, with the normal defaults. It ran all 123 active cases and one ignored case in 313.11 seconds on cores 4–7, including warm Cargo startup. That feature-specific ELF had SHA-256 `eb60e581dab07ee544c7eb1a2b009d78cb1b6a17678599ed2e0c6d2687c830cb`.

## Execution and verification

The Linux headless gate uses a Rust Cargo target runner for this one example. It checks the original binary's libtest listing, starts its measured longest case early, and dispatches each discovered case once. Up to four independent child processes inherit the profile environment and the existing process-group deadline. A child failure fails the suite after the remaining cases run. Other Cargo targets and explicit test filters pass through to their original executables. The renderer keeps its existing invocation. The gate binds the runner source alongside the other verification inputs.

The runner regressions use real libtest binaries, an actual Cargo invocation with spaces in its path, an intentionally failing test, a dormant ignored test, malformed or missing children, and an unrelated target. The scoped instrumented runner executed 106/107 lines (99.07%) and 170/176 regions (96.59%). The Ubuntu 24.04 reference container passed all 489 verifier regressions. The headless and renderer release result still requires a fresh complete run for the integrated commit.

The original streamed stdout and stderr are in artifact `14917770601`, archive SHA-256 `6746cd832a669328cb5402f930e8fde36a41e199867d032d5bdc493eb90d48f1`. The checkout's native evidence directory is `artifacts/local-implementation/hosted-71cd07a`. It contains per-run timers, Cargo JSON, libtest transcripts, original raw profiles, matched-object exports and the instrumented helper report.

To exercise the runner directly through Cargo on Linux, use a workspace-root shell with the pinned toolchain. The formal release gate builds the runner and sets its instrumentation environment itself.

```bash
mkdir -p target/robot-coverage
rustc --edition=2024 -O scripts/coverage-test-runner.rs -o target/robot-coverage/runner
export CARGO_TARGET_DIR="$PWD/target/robot-coverage"
cargo test --locked -p gz-sim-cli --example local_robot_foundations \
  --features gz-sim-cli/rgb-camera,gz-sim-cli/landing-audit \
  --config "target.x86_64-unknown-linux-gnu.runner=['$PWD/target/robot-coverage/runner']"
python3 -m unittest discover -s tests -p test_coverage_test_runner.py
```
