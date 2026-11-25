<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Can the local robot workloads close the CLI coverage gap?

The three measured local operator workflows added **nine executable lines and nine LLVM regions** to a baseline of 204 ordinary tests on candidate `836bbcd`. The earlier official CPU run on `522a03f` needed another **82 lines and 314 regions** to exceed 95% in the CLI crate. The nine additions belong to two files whose contents match across the two commits. Every newly hit region-entry coordinate in those two files was already exercised by the official run. These workloads establish robot and viewer behavior, but they do not close the crate-wide coverage gap. [Counts and source-coordinate comparison](mvp-cli-local-supplement.json)

| Same object set on 836bbcd | Covered lines / 7,354 | Covered regions / 12,636 |
| --- | ---: | ---: |
| 204 ordinary tests | 6,948 (94.48%) | 11,885 (94.06%) |
| With physical runs and WebGPU playback | 6,957 (94.60%) | 11,894 (94.13%) |

The ordinary tests comprised 159 library cases, two main-binary cases, 18 general CLI cases, 24 journal cases and one live transport test on the independent motorized robot. The supplemental commands recorded a 10-second screw-and-wrist gripper transfer, recorded a 610-row independent-robot journal using one Bevy thread, verified all rows with eight threads, and played the gripper recording in gz-web-rs. The browser checked all 251 frames, 3,765 body poses and shape records, and rewind. [Gripper](../../../crates/gz-sim-cli/examples/parked_spindle.rs) · [Independent robot journal](../../../examples/replay/upstream_joint_force.json) · [Browser check](../../../tools/physics-audit/browser/src/main.rs)

The local-only build uses `--release --no-default-features`. The official headless profile runs the whole workspace with the RGB camera and landing audit features and default features, including distributed physics. The official 14,723-region denominator and the local-only 12,636-region denominator therefore measure different source sets. The nine-region increase is a paired comparison on this candidate. No coverage from earlier builds was imported. [Official runner](../../../scripts/verify.py) · [Profile manifests](../../../planning/coverage-profiles.json)

## Where additional coverage would come from

The earlier official ordinary suite already ran live stepping and the replay/record CLI cases. Its largest remaining gaps included 132 regions in the distributed tree binary, 108 in WebSocket handling, 72 in lifecycle handling, 68 in convoy dispatch and 52 in the hinge binary. Local operator tests that exercise genuine lifecycle refusals, reconnects and recording I/O failures can reach some of those paths. The current long-running carry, contact-recovery and set-down matrices call into the CLI through the same visual snapshot and archive writer APIs. A finer solver step count by itself does not exercise the transport, lifecycle or file failure paths. [Socket](../../../crates/gz-sim-cli/src/web/socket.rs) · [Lifecycle](../../../crates/gz-sim-cli/src/web/lifecycle.rs) · [Recording writer](../../../crates/gz-sim-cli/src/web/recording/writer.rs)

## Reproduction

Compile the CLI library, main, `gz-sim-local-replay`, `parked_spindle` and the test harnesses with `-C instrument-coverage` on the pinned toolchain. Use a separate profile directory for each group. First run the library and main harnesses, `--test cli`, `--test local_cpu_replay` and the filtered `--test native upstream_motor`. Then run the following commands from the repository root with the instrumented release executables on the path. Choose fresh output paths.

```bash
parked_spindle target/cli-check-gripper --capped --threads 1
gz-sim-local-replay examples/worlds/upstream_joint_force.sdf examples/replay/upstream_joint_force.json \
  --trace target/cli-check-motor.trace --record target/cli-check-motor
gz-sim-local-replay examples/worlds/upstream_joint_force.sdf examples/replay/upstream_joint_force.json \
  --verify target/cli-check-motor.trace --threads 8
gz-sim-rs --server --replay --web-viewer 11629 target/cli-check-gripper/switched
```

Open the gz-web-rs replay and allow all 251 frames to render before rewinding and shutting down the server cleanly. Use `llvm-profdata merge -sparse` for the baseline alone, then again with the supplemental raw profiles added. For each merged profile, export `llvm-cov` counts for `crates/gz-sim-cli/src` using the same main, local replay, gripper, unit/integration test and native harness objects. Adding fields or changing the set of compiled objects changes the denominator.
