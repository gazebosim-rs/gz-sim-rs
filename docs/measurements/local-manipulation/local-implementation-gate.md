<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Local combined physics release qualification

The detached Ubuntu run used candidate `8c57ff69ca134cba7d201f5e2d5f4eb28e599647`, tree `de23a5a2cb6998405a5b5228f4242c9446302886`, and diff base `438b0afb6128951c2b7148a2d969231304fd356d`. Formatting, strict Clippy, the 481 verification-script tests, documentation and the startup budget checks passed. The CPU profile passed 3,727 ordinary tests, with 129 ignored. Its mandatory worker harness independently passed 39 selected native cases plus the production curved-scene checkpoint replay. That replay compared 257 states across 16 actual process kills and 16 cold reopens, including eight partially written checkpoints that it discarded.

The CPU release command stopped at the CLI's per-crate line threshold. We then ran the standard renderer profile command on the same detached tree and toolchain, reusing the Cargo cache. It passed 2,087 ordinary tests, with 96 ignored, and failed its first threshold on GPU physics under llvmpipe. The table shows both profiles' final LLVM counts for each crate below the strict 95% line or region requirement.

| Profile | Crate | Lines | Regions | Further regions needed |
|---|---|---:|---:|---:|
| Headless | gz-sim-cli | 7,058 / 7,852 (89.89%) | 11,941 / 13,427 (88.93%) | 815 |
| Headless | gz-sim-scene | 26,842 / 28,981 (92.62%) | 40,732 / 44,764 (90.99%) | 1,794 |
| Headless | gz-physics-distributed | 30,722 / 35,689 (86.08%) | 47,799 / 58,366 (81.90%) | 7,649 |
| Renderer | gz-sim-cli | 6,260 / 7,857 (79.67%) | 10,468 / 13,430 (77.94%) | 2,291 |
| Renderer | gz-sim-scene | 23,953 / 31,027 (77.20%) | 36,657 / 48,023 (76.33%) | 8,965 |
| Renderer | gz-physics-gpu | 1,985 / 5,320 (37.31%) | 3,491 / 8,867 (39.37%) | 4,933 |

Rapier reached 95.83% of lines and 95.20% of regions. The multifidelity and XPBD backends each exceeded 96% on both metrics. Seven camera and renderer crates, as well as shared core and protocol, exceeded both required percentages in the renderer profile.

These are the counts from the original profile commands. Later local tests uncovered a real live-server wrist admission error and exercised warm contact and motion paths missing from the original CLI and scene measurements. Their separate reports record the physical comparisons and the additional counters. The renderer's software adapter does not supply discrete GPU physics evidence. Each mandatory profile must still clear the thresholds independently on the release candidate. [Live serial robot](local-robot-operator.md) · [Warm CLI checkpoints](cli-warm-physics.md) · [Worker qualification](headless-worker-coverage.md)

The local receipts, both LLVM reports, native peer transcripts and raw profiles are in `artifacts/local-implementation/integration-audit/gate-8c57ff69ca13-T99feidj`. The `profiles/{headless,renderer}/object-paths.json` files map the compiler's original executable paths to their retained ELF files. The `{headless,renderer}-all-crates.json` files give counts for every crate and identify the largest uncovered files.
