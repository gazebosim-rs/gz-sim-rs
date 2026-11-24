<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Release checks for the local robot integration

The frozen review candidate `522a03f` passed 3,960 ordinary headless Rust tests across 814 harnesses, with 151 ignored. The four mandatory worker checks also passed. They include 49 selected native cases, 1,021 gripper frames matched to an independent local world, and a curved-scene trial with 16 controller kills and 16 disk reopens. Formatting, both strict Clippy configurations, docs, startup checks and 482 verification-tool tests passed. The pinned runtime and both Git object IDs are recorded with the counts. [Exact counts and test scope](local-robot-release.json) · [Previous frozen run](replay-release-gate.md)

Ten of thirteen headless crates cleared both 95% release thresholds over their instrumented sources. Rapier reached 95.89% of lines and 95.28% of regions, multifidelity 96.14% and 96.59%, and XPBD 96.32% and 96.37%. The three remaining headless crates are listed below. The canonical release command stopped at the CLI threshold.

| Crate | Line coverage | Region coverage | Additional lines to exceed 95% | Additional regions to exceed 95% |
|---|---:|---:|---:|---:|
| Simulator CLI | 7,965/8,470 (94.04%) | 13,673/14,723 (92.87%) | 82 | 314 |
| Scene | 29,119/30,950 (94.08%) | 44,534/48,129 (92.53%) | 284 | 1,189 |
| Distributed physics | 31,760/35,689 (88.99%) | 49,506/58,366 (84.82%) | 2,145 | 5,942 |

The headless report covered 6,840 of 6,948 measured changed lines. Each affected crate exceeded 95% on its measured changed lines. GPU-feature scene/server files, opt-in worker tracing and a declaration-only file have separate source-ownership constraints. The eligibility list and complete source omissions are in the receipt. [Source and coverage detail](local-robot-release.json)

The physical RTX 5090 separately passed all 77 GPU cases across 14 exact renderer-build test targets. The table above uses only headless measurements. Later jaw, sheet and wrist phases carry their own code-level qualifications. The review branch still requires a complete passing release run on its final tree.
