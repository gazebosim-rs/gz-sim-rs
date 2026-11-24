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

The independent software-renderer continuation passed 2,875 ordinary tests across 772 harnesses, with 225 ignored. Nine of the twelve renderer-profile crates exceeded both whole-crate thresholds. Software-only GPU coverage was 2,519/7,064 lines and 4,675/12,195 regions. The canonical renderer command stopped at that threshold.

The same build passed all 77 physical GPU cases across 14 test targets on an RTX 5090. Adding those 34 raw profiles to the exact software-run objects covered 6,829/7,064 GPU lines (96.67%) and 11,680/12,195 regions (95.78%). The GPU covered 2,010/2,017 changed lines, scene 4,491/4,563 and CLI 621/625 on this combined renderer source list. The combined scene and CLI still fall below the whole-crate thresholds. This supplemental hardware qualification is recorded alongside the strict result. [Per-crate counts, conditional sources and changed-line scope](local-robot-release.json)

The 807-binary re-export reproduced the baseline counts for each of 220 source files before the hardware additions. LLVM reported mismatched function data for 9,704 functions in the headless export, 10,124 in the software renderer and 12,182 in the combined hardware export. The retained exact objects and source-level comparisons make the reported scope checkable. Later robot phases have their own code-level qualifications. A passing release run on the final integrated tree remains open.
