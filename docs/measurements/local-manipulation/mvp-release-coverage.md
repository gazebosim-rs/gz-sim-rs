<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Local CPU MVP release qualification

**The local CPU backend crates passed their coverage targets. The repository-wide release gate failed.** On `fc80ca5`, 4,208 ordinary Rust cases passed in the CPU transcript (967 libtest summaries, 158 ignored). All four mandatory native suites also passed, including 49 selected worker cases across ten harnesses. The command exited 1 at CLI line coverage. The independently executed llvmpipe renderer passed 3,099 ordinary cases (802 summaries, 243 ignored) and all four native suites, then exited 1 at GPU line coverage.

## Provenance and checks

* The November 24, 2026 CPU run used commit `fc80ca51570b91bca0ad192187688524b4844f0c`, tree `bf932be7c3ad52a5a64946f905140b45f0c563eb`, against port commit `dddf1f4284557c420e384d263a4365b2153dfde2`. The full command was `python3 scripts/verify.py` in the CI Ubuntu environment with the locked Zenoh SDK and pinned Rust compiler. Formatting, both strict Clippy configurations, both documentation configurations, all 489 verifier tests, and the native-free Rapier/TPE and RGB qualifications passed. The original CPU command stopped before renderer dispatch. [CI environment](../../../.github/workflows/ci.yml)
* The integration attempt at `836bbcd` passed the prechecks and native-free runtime before a local control binary correction superseded it. At `2e7d155`, that binary was first rejected by the RGB evidence validator. The final build bound the simulator, RGB test executable, local replay and control binaries to their Cargo records, including independent verification of both ordinary companion binaries in the native-free graph.
* The independent renderer profile and the separate physical GPU tests used `2e7d1552587f703fb797419da1e0ab59b5111cc1`, tree `0ff1968e723c48f96ff4669e6f59b10cee2e30d2`. Comparing it with `fc80ca5` found identical Rust sources, Rust tests, manifests, dependency lock and renderer verifier. The differences were documentation, a native-free validation script and two Python tests for that script. The software run used Mesa llvmpipe. All 88 physical cases across 14 targets passed on an NVIDIA GeForce RTX 5090, driver 595.99.02.
* The common Ubuntu image digest was `sha256:c1fb881cdd3c52cd77b1b390acdc723ea3f9cbc24268466427e554a498d691c2`. Logs, per-crate JSON, comparisons and raw profiles are under `artifacts/local-cpu-mvp-coverage/<tree>`. The canonical CPU raw coverage is in the retained `source-final/artifacts/coverage/profiles-cynk717w/headless` checkout.

## Official crate counts

Each figure is covered/total for the measured files. Each production crate must exceed 95% independently for lines, regions and any changed executable lines. Every owned source also needs an instrumentation record or an existing zero-executable proof. The original LLVM exports warned of **13,428 functions with mismatched data on CPU and 9,201 on the renderer**. The reports below preserve their original counts. [Verifier](../../../scripts/verify.py) · [Profile definitions](../../../planning/coverage-profiles.json)

| Crate | CPU lines | CPU regions | Renderer lines | Renderer regions |
|---|---:|---:|---:|---:|
| gz-assets | 1,112/1,145 | 1,990/2,064 | — | — |
| gz-camera-output | — | — | 1,674/1,698 | 2,315/2,398 |
| gz-physics | 16,300/16,881 | 27,792/28,700 | — | — |
| gz-physics-distributed | 31,771/35,689 | 49,517/58,366 | — | — |
| gz-physics-gpu | — | — | 2,750/8,091 | 5,090/14,102 |
| gz-physics-multifidelity | 2,006/2,086 | 2,064/2,137 | — | — |
| gz-physics-rapier | 30,323/31,585 | 50,089/52,509 | — | — |
| gz-physics-tpe | 220/221 | 310/312 | — | — |
| gz-physics-xpbd | 11,702/12,141 | 20,404/21,163 | — | — |
| gz-render-client | — | — | 423/431 | 655/670 |
| gz-render-frame | — | — | 212/213 | 295/296 |
| gz-render-protocol | — | — | 182/185 | 274/280 |
| gz-render-scene | — | — | 346/347 | 425/426 |
| gz-render-worker | — | — | 570/576 | 686/699 |
| gz-renderer | — | — | 973/993 | 1,207/1,251 |
| gz-sensors | 546/554 | 735/745 | — | — |
| gz-sim-cli | 8,484/8,963 | 14,555/15,584 | 8,483/8,963 | 14,555/15,585 |
| gz-sim-core | 403/417 | 587/610 | 415/417 | 599/610 |
| gz-sim-proto | 189/190 | 329/340 | 189/190 | 329/340 |
| gz-sim-scene | 31,129/32,953 | 47,528/51,151 | 32,995/36,493 | 50,538/56,917 |
| gz-sim-server | 12,437/12,830 | 18,430/19,327 | — | — |

CPU CLI needs 31 more lines and 250 more regions, scene needs 177 lines and 1,066 regions, and distributed needs 2,134 lines and 5,931 regions, at the current denominators. Even if every source file touched since the three-step phase began at `1311332` reached 100%, at least **153 CLI, 1,061 scene and 5,931 distributed regions in unchanged files** would still need coverage. The prior canonical CPU run at `522a03f` already had corresponding region deficits of 314, 1,189 and 5,942. [Prior integration run](local-robot-release.md)

Against the full port comparison base, CPU coverage on measured changed executable lines was 3,660/3,948 for CLI, 18,766/20,139 for scene and 30,912/34,430 for distributed. The command stopped at whole-crate coverage before the formal changed-line check. Fields named `measured_changed` in the per-crate JSON are a separate audit of the same frozen LCOV data.

The CPU profile had no LLVM records for 11 GPU-only scene modules, two GPU-only server modules, four optional worker tracing modules and one rotor file containing type declarations and derives. The original renderer profile accounted for all of its owned files. Adding local robot scenarios alone cannot produce CPU records for features excluded by the headless build. Releasing under the current requirements needs an explicit source-ownership solution for GPU-specific adapters, an instrumented run of optional CPU tracing and evidence for the declaration-only source. The gates, their thresholds and their source lists were preserved.

## Additional local checks and next workloads

* Two new SDF tests check seven ambiguous or incomplete continuous-collision declarations and reproduce both the fast hit and clear trajectory after reversing swept-link declarations. With the same binary, the original ten compound-collision tests exercised 73/79 lines and 121/131 regions of the importer. All twelve passed and exercised 78/79 lines (98.73%) and 127/131 regions (96.95%). [Tests](../../../crates/gz-sim-scene/tests/local_compound_ccd.rs)
* For CLI coverage, prioritize the stock viewer's handling of an interrupted lifecycle reply, a stale epoch and a reconnect, checking the actual robot clock and state and the result of its next command. In this profile the CLI's WebSocket and lifecycle modules account for over 170 uncovered regions. For local scene coverage, the next physical fixture should exercise a nominated supported foot through an edge strike, release and reset, comparing per-contact receipts against a continuously dynamic run. Qualify the exact release feature graph for both.
* Existing worker scenarios ran as supplemental release checks on the frozen CPU build. A current/legacy native worker test reproduced the committed and subsequent articulated states at all ten vector-effort interruption points. A two-group curved-material fixture compared 107 complete states to a single local simulation across 12 migrations, six controller kills and one loaded reset. A separate run let one actual solver physically return its twelfth reply and exit normally, so LLVM could flush its counters. All 31 public state digests and load vectors matched through the recovery. The full physics comparison and binary hashes are retained in `supplemental-vector-headless` and `supplemental-partition-headless`.
* Re-exporting all **851** original CPU executables reproduced both metrics for all **313** compared CLI, scene and distributed files. Adding the preceding worker evidence (32 raw profiles and the ordinary candidate-built example) kept every denominator. Distributed gained **624 lines and 1,050 regions**, reaching 90.77% and 86.64%. Scene gained **112 lines and 175 regions**, reaching 94.80% and 93.26%. CLI counts stayed the same. The combined CPU export warned of 13,508 function-data mismatches. This documents an existing test workload that can strengthen qualification without adding distributed engine features.
* Re-exporting all **838** original renderer objects reproduced both metrics for all **215** compared GPU, scene and CLI files. Adding the **34 RTX profiles** kept every denominator. GPU reached 7,807/8,091 lines (96.49%) and 13,501/14,102 regions (95.74%). Scene reached 34,534/36,493 and 52,928/56,917 (94.63% and 92.99%). CLI reached 8,483/8,963 and 14,556/15,585 (94.64% and 93.40%). The combined renderer export warned of 11,371 function-data mismatches. These are supplemental hardware figures; the independent canonical software result remains the one in the table.
