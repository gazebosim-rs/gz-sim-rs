<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Release qualification after the local robot replay

The frozen candidate was `bdef251828799cf934b31adfbf48307379f636e7`, tree `918f017f3cc0b564045ecef12b012b95fbbef450`, compared with `438b0afb6128951c2b7148a2d969231304fd356d`. It includes the local serial gripper input journal, GPU tool contact, gripper performance work and public-clock coupling. The port checkout stayed separate. The qualification used the pinned Ubuntu image `c1fb881cdd3c`, rustc 1.97.1 and LLVM 22.1.6.

Both Clippy feature configurations, formatting, documentation, startup checks, 482 verification tests and the native-free RGB and CPU backend checks passed. Cargo builds the local replay binary alongside its regular simulator when it prepares a CLI integration test. Two corrections to the qualification scripts validate that companion against its Cargo target, source and features and retain the binaries the tests actually ran. The affected runs on the original candidate and the first correction are retained alongside the completed run.

## Headless

All 3,861 ordinary tests passed across 802 harnesses. Cargo listed 150 ignored tests. The subsequent physical qualification passed its four suites, including 49 explicitly selected cases across ten harnesses. The gripper matched an independently advanced local world for 1,021 frames. The checkpoint trial reproduced 257 states across 16 actual controller kills and 16 disk reopens. The strict command exited on CLI line coverage.

| Crate below target | Covered lines | Covered regions | Additional lines needed | Additional regions needed |
|---|---:|---:|---:|---:|
| CLI | 7,832 / 8,338 (93.93%) | 13,434 / 14,482 (92.76%) | 90 | 324 |
| Scene | 28,497 / 30,389 (93.77%) | 43,551 / 47,267 (92.14%) | 373 | 1,353 |
| Distributed physics | 31,779 / 35,689 (89.04%) | 49,525 / 58,366 (84.85%) | 2,126 | 5,923 |

The other ten crates exceeded 95% on both numeric measures among instrumented sources. Rapier covered 29,102 of 30,355 lines (95.87%) and 47,952 of 50,345 regions (95.25%). A separate check against the frozen diff counted 5,199 of 5,322 instrumented changed lines. Each crate with measured changes exceeded 95% on those lines. The strict source-ownership requirement also applies to sources absent from the LLVM output.

The headless profile leaves the GPU feature disabled. As a result its coverage report has no records for eight GPU-specific scene files and two GPU-specific server files. It also has no records for four opt-in trace files or for the rotor file containing only two type declarations and their derives. The existing syntax proof independently qualifies the protocol include facade. The documentation and counts here preserve the original source list and both release thresholds.

## Optional CPU tracing

An independent build of the same tree enabled `barrier-trace,lane-trace,sphere-pair-trace`. Three existing Rust tests measured four real workers at the barrier, matched per-world lane receipts, and reopened the first loaded sphere contact after losing an acknowledgement. The last case produced 199 timing records across four roles. Its original and independently resumed states matched. The counters below belong to that specific build.

| Feature source | Covered lines | Covered regions |
|---|---:|---:|
| Barrier timestamps | 115 / 179 | 166 / 279 |
| Linux scheduler samples | 18 / 18 | 45 / 58 |
| Sphere pair timings | 61 / 69 | 82 / 93 |
| Per-world lane tracing | 130 / 133 | 222 / 239 |

These fixtures can be rerun directly from the frozen checkout. The third command also writes the native timing records when `GZ_MULTI_SPHERE_TRACE_DIR` names an existing writable directory.

```sh
features=barrier-trace,lane-trace,sphere-pair-trace
RUSTC_WRAPPER= cargo test --locked -p gz-physics-distributed --features "$features" \
  --lib physics::partition::trace::tests::observed_threads_and_receipts_match_published_worlds -- --exact
RUSTC_WRAPPER= cargo test --locked -p gz-physics-distributed --features "$features" \
  --test barrier_trace scoped_clocks_distinguish_reply_ready_from_collection -- --exact
RUSTC_WRAPPER= cargo test --locked -p gz-physics-distributed --features "$features" \
  --test multi_sphere_ownership a_contact_on_the_first_tick_and_queued_cancellation_recover_exactly -- --exact
```

## Renderer and GPU

The renderer profile used Vulkan through llvmpipe. All 2,805 ordinary tests passed across 759 harnesses. Cargo listed 212 ignored tests. All four physical qualification suites passed in 137.50 seconds, including 20 explicitly selected cases across eight harnesses. Each source in the renderer ownership list had a coverage record or its existing syntax proof. The strict command exited on GPU physics line coverage.

| Crate below target | Covered lines | Covered regions | Additional lines needed | Additional regions needed |
|---|---:|---:|---:|---:|
| GPU physics | 2,226 / 5,858 (38.00%) | 4,064 / 10,002 (40.63%) | 3,340 | 5,438 |
| CLI | 7,837 / 8,343 (93.94%) | 13,437 / 14,485 (92.76%) | 89 | 324 |
| Scene | 29,623 / 32,628 (90.79%) | 45,246 / 50,834 (89.01%) | 1,374 | 3,047 |

The nine remaining renderer crates all exceeded both thresholds. Software-only changed-line coverage was 532/868 for GPU physics, 477/481 for CLI and 2,315/2,824 for scene.

The same Cargo executables then ran the repository's physical Vulkan qualification on the NVIDIA GeForce RTX 5090 with driver 595.99.02. All 65 hardware cases passed across 14 targets, including the planar arm, authored tool contacts, queued native impulses, sensor RPC and warm checkpoint recovery. The final-contact worker completed three seven-tick blocks against the independently advanced device in the parent. It closed on input EOF and emitted a raw LLVM profile matching the worker ELF build ID `71f8c2c5095ff123b2a5cf6a9c21175d312c2df9`.

Exporting the original merged profile with the 794 retained executable objects reproduced the original covered and total counts for every file checked. Adding the 34 hardware-run profiles kept all file denominators fixed. The comparison includes 207 renderer-owned, instrumented files plus the two GPU control files in the server. The added observations also cover every GPU-specific file absent from the headless run.

| Crate or file group | Lines with physical GPU | Regions with physical GPU | Covered changed lines |
|---|---:|---:|---:|
| GPU physics | 5,641 / 5,858 (96.30%) | 9,559 / 10,002 (95.57%) | 866 / 868 |
| CLI | 7,837 / 8,343 (93.94%) | 13,438 / 14,485 (92.77%) | 477 / 481 |
| Scene | 30,645 / 32,628 (93.92%) | 46,855 / 50,834 (92.17%) | 2,742 / 2,824 |
| Two server GPU control files | 162 / 172 (94.19%) | 222 / 232 (95.69%) | Outside renderer ownership |

## Remaining work

* Exercise real local CLI refusal, WebSocket recovery status and restored model lifecycle paths. Exercise scene primary contact and posture transitions against physical references. The hardware-augmented report still needs 323 more CLI regions and 1,438 more scene regions at its measured denominators. The original CPU-only CLI and scene gaps are in the first table.
* Exercise the GPU server's missed lines in a server build admitting physical GPU control. Keep the GPU-free build and the real adapter run independently identifiable. The current contract assigns every server source to the headless profile and assigns the server to no renderer profile. Capturing a device-side result does not satisfy that contract.
* Retain the explicitly enabled tracing build and its input/output evidence. A future ownership decision for the rotor declaration file needs a source-bound syntax proof or executable behavior. It contributes no LLVM counters in this build. The legacy worker and recovery package still needs 5,923 more regions at the observed CPU denominator.
* Repeat the strict runs on the candidate intended for release after the outstanding source and coverage gaps are addressed. The ordinary LLVM exports reported 9,654 mismatched functions in headless and 8,984 in the renderer. The exact-object hardware re-export reproduced the software counts; the supplemental merged report emitted its own mismatch warning. The original logs and every measured executable remain available for narrower source-bound checks.

The raw profiles, complete test output, per-source counts, executable identities and exact drivers are under `artifacts/local-implementation/integration-audit/gate-bdef25182879-dqltcOKl/`. The `profiles` subdirectory preserves the original runs; `trace` and `hardware` contain the separate feature and RTX checks. The original renderer Docker exit status is in `renderer-daemon.status` and the original headless status is in `after.txt`. [Machine-readable measurements](replay-release-gate.json) · [Previous combined release measurements](local-implementation-gate.md) · [Local robot journal and gz-web replay](local-cpu-input-replay.md)
