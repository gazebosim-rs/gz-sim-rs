<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Local CPU MVP release qualification

The repository requires more than 95% of executable lines and LLVM regions in each production crate, plus more than 95% of executable changed lines in each crate with changes. Every owned Rust source needs an instrumentation record or one of the verifier's two existing zero-executable proofs. The CPU and renderer profiles are independent. [Release verifier](../../../scripts/verify.py) · [Profile definitions](../../../planning/coverage-profiles.json)

## Local CPU checks

The native-free Rapier/TPE runtime prerequisite passed on the starting commit `1311332a4f29df047df3b82f8410a8417b6b31f8`. Two added tests run the normal SDF loader. They check that seven ambiguous or incomplete continuous-collision declarations report a load error and that reordering two swept links preserves both trajectories through impact. Using the same instrumented binary, the original ten tests exercised 73/79 lines and 121/131 regions in the swept-collision importer. Running all twelve exercised 78/79 lines (98.73%) and 127/131 regions (96.95%). All 12 compound-collision tests passed with `cargo test --locked -p gz-sim-scene --test local_compound_ccd`. [Regression tests](../../../crates/gz-sim-scene/tests/local_compound_ccd.rs)

## Reference measurement

The preceding archived headless LLVM report for `522a03fa7fd4` recorded the following shortfalls. The additional counts use the original denominators and round up to the first integer strictly above 95%.

| Crate | Lines | Regions | Additional lines | Additional regions |
|---|---:|---:|---:|---:|
| CLI | 7,965 / 8,470 (94.04%) | 13,673 / 14,723 (92.87%) | 82 | 314 |
| Scene | 29,119 / 30,950 (94.08%) | 44,534 / 48,129 (92.53%) | 284 | 1,189 |
| Distributed physics | 31,760 / 35,689 (88.99%) | 49,506 / 58,366 (84.82%) | 2,145 | 5,942 |

The other ten CPU crates exceeded both percentages among measured files. That CPU profile also lacked records for five worker source files, eleven GPU scene source files and two GPU server source files. The corresponding scope inventory and LLVM counts are in the integration checkout's `artifacts/local-implementation/integration-audit/gate-522a03fa7fd4-NnmfbE7G/{headless-all-crates.json,headless-scope.log}`. Its build cannot provide counters for this candidate.

## Reproduction

Run `python3 scripts/verify.py` on the pinned Rust toolchain in the CI Ubuntu environment with the locked Zenoh reference SDK. Set `GZ_COVERAGE_BASE` to the port comparison commit. The full command runs formatting, both Clippy configurations, documentation and verification tests, native-free runtime checks, the headless profile and the renderer profile. The hosted renderer profile selects Mesa llvmpipe. Separate device-specific physics qualification requires selecting a real supported Vulkan adapter. [CI environment](../../../.github/workflows/ci.yml)

## Candidate run

Both local jobs started November 24, 2026 on frozen commit `836bbcd8f6aae6c80568c11c2579fc835c319f75`, tree `018379598635ae51b36b5d7df040755c71fa67d2`. The comparison base was `dddf1f4284557c420e384d263a4365b2153dfde2`. The Ubuntu image digest was `sha256:c1fb881cdd3c52cd77b1b390acdc723ea3f9cbc24268466427e554a498d691c2`. Formatting and both strict Clippy configurations passed. The full release command and independent renderer profile write to `artifacts/local-cpu-mvp-coverage/018379598635ae51b36b5d7df040755c71fa67d2`.
