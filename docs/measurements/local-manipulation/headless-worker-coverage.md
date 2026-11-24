# Real worker coverage in the headless qualification

The headless job now runs the existing Rust robot, CLI and recovery fixtures that require a stock physics worker. Cargo still runs its ordinary workspace tests first. The qualification then launches the selected parent tests against the worker Cargo built in that same profile. Each family runs serially and records its executable identity, selected cases, elapsed time, stdout and stderr. LLVM merges the profiles from the worker, controller and simulator with the ordinary test profiles.

The previous hosted headless run at `38c47d8` covered 71.73% of regions in `gz-physics-distributed`, 77.84% in `gz-sim-cli` and 84.41% in `gz-sim-scene`. One named test illustrates the gap. The ordinary grounded robot test exercises the local solver and its SDF parameters. It covered 161 of 194 SDF specification regions and zero of 511 regions in the separate worker-backed grounded robot implementation. The existing humanoid posture and chest-load fixture drove its Bevy scene, committed the loaded step through the stock worker and exercised 318 of those 511 regions in 16.62 seconds on this machine. Both measurements used matching executable objects and generated profiles.

For a second comparison, the same frozen test and worker objects ran their nine ordinary Cargo suites first. Adding the 18 worker-backed CLI, example and integration cases to those profiles reached 8,790 more regions in distributed physics, 1,701 in CLI and 2,771 in scene. Every binary ID in the merged profiles matched a retained executable. For example, the CLI tree operator rose from 62 to 554 of 686 executable regions; the scene’s worker-backed grounded robot rose from 0 to 451 of 511. The 21 distributed-library recovery cases were excluded from this comparison.

## What runs

The selection contains 39 top-level tests across nine Cargo targets. In the initial instrumented run, 38 cases passed individually and took a combined 655.93 seconds. The remaining case exposed a real compatibility problem in the historical two-sphere checkpoint: its cached total inertias on two links differ by a few last-place bits from present construction. Its authored inertias, solver parameters and construction digest matched. The compatibility fix reconstructs the original offset-collider mass calculation for that format. The original native bytes reinstall unchanged and a one-ULP alteration to the stored inertia still fails validation. All four bodies, hinge motion and every contact reaction matched the archived uninterrupted solver for ticks 1,292–1,295, including the 0.129687805 N·s saved impulse and 0.003210633 N·s next reaction. The worker crash and merge case passed. The affected production source recorded 20/20 executed lines and 25/26 regions. The full combined headless run is tracked separately.

The selected Linux fixtures cover joint loads and warm contacts across a controller loss, recovery of moving rigid bodies, separate and combined gripper contacts, owner changes, rejected reinstall, and continuation after save, reopen and reset. Operator tests use the actual CLI children for effort, motor, stopped-tree, fleet and convoy commands. Three humanoid tests cover grounded posture, adoption of an already loaded local robot, and return to local ownership. The selection and exact test names live in [the headless worker case list](../../../crates/gz-physics-distributed/tests/support/headless-worker-cases.json).

The fixtures require permission to launch and signal child processes, private pipes, loopback networking and writable temporary storage. The fixtures create their live checkpoints and input commands in test-owned temporary directories. The two-sphere regression also reopens a 16,553-byte format-v1 checkpoint from the original four-worker run at tick 1,292. The archived bytes are checked in as `pair-warm-v1-tick-1292.gzmultisphere` (SHA-256 `7bdbbe2133701d371b99008a012924a4dd5605b3dde04edaae34f4095ba25017`). Historical protocol compatibility fixtures require a separately built legacy executable. Those fixtures remain available to their dedicated qualification. Child-only crash entry points run under their owning parent test, which supplies the checkpoint, expected failure boundary and process handles.

## Curved-scene checkpoint

The production `curved_scene_crash` example compares a native reference with a worker-driven Bevy scene. With seed 5 over two 120-tick runs it killed 16 controllers, reopened all 16 saved checkpoints and reproduced eight deliberately discarded next states. All 257 state comparisons passed. The scene carries a moving platform, a free cargo body, a pending wrench and a sampled IMU.

Those genuine kills leave the actor no opportunity to flush LLVM's counters. The same binary therefore also saves a moving scene at tick four, takes one more step, and exits on stdin EOF. A new actor opens the saved checkpoint, takes the same step with its pending wrench, then resets the world. The comparison checks the native state, scene geometry, catalog and IMU on both sides. In a matched-object run, the crash workload alone recorded zero of 645 checkpoint regions. The graceful actors exercised 498 of 645 and also reached 154 of 418 worker-management regions. A fresh locked Cargo build and the Rust qualification reproduced both counts. Its production example compiled in 2.15 seconds after dependencies were ready, and the full build-and-run test completed in 2.75 seconds. Exporting that example against its three matching LLVM process profiles produced the same counts without a mismatch warning. The qualification records the Cargo-built production executable SHA-256, crash counts, and both states in `worker-trajectory.json`. Both sets of child profiles enter the headless report.

The native recording shows the platform and cargo in the first epoch, including eight reopenings. The gz-web-rs WebGPU viewer checked all 29 frames and 58 rendered body poses and shapes as it played forward and rewound. [Watch the checkpoint replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9894%2F)

## Reproduce

Run the full profile with the repository's normal `python3 scripts/verify.py` command in its provisioned Linux environment. `headless-workers.json` and `worker-*.stdout` / `worker-*.stderr` accompany the normal coverage report.

For a focused run of the grounded robot case, from the repository root:

```sh
RUSTC_WRAPPER= cargo build --locked -p gz-physics-distributed --bin physics_worker
GZ_MULTI_TEST_WORKER="$PWD/target/debug/physics_worker" RUSTC_WRAPPER= \
  cargo test --locked -p gz-sim-cli --example humanoid_stand \
  tests::bevy_posture_and_chest_load_use_one_durable_robot_interval -- --ignored --exact
```

The full-workspace report determines release coverage for each crate. Individual fixture measurements describe only the paths that fixture executes.

## Loaded grippers against the ordinary solver

The headless gate runs the ordinary Cargo example `grasp_fleet_profiles` with the authored two-gripper SDF. One Bevy world advances both grippers locally. A second world advances the same motors and independent parcels through the stock worker processes. The example compares the complete native and public rigid state, joints, motor targets and contact reactions at every millisecond. At 750 ms it migrates the first gripper, replaces its worker, returns it to its original owner, replaces that worker, saves the checkpoint and reopens the scene. Both grippers carry pad load across the cut. At 1,020 ms the first has released while the second still holds.

| Time (ms) | First hand, left / right (mN·s) | Second hand, left / right (mN·s) |
| ---: | ---: | ---: |
| 0 | 0 / 0 | 0 / 0 |
| 749 | 1.311192 / 1.311534 | 1.311695 / 1.312039 |
| 750 | 1.311180 / 1.311521 | 1.311690 / 1.312033 |
| 751 | 1.311167 / 1.311508 | 1.311684 / 1.312027 |
| 1,020 | 0 / 0 | 1.308613 / 1.308917 |

All 1,021 frames matched the ordinary run. The fresh Ubuntu test took 33.5 seconds for this fixture and the curved checkpoint fixture together, including their production example builds. The physical run newly exercised 185 of 248 lines and 295 of 413 regions in the scene's gripper-fleet backend, previously zero in the combined headless report. The gate saves the example and worker hashes, checkpoint hash, measured pad impulses and frame count in `worker-grippers.json`. Its production Cargo object and real process counters enter the headless LLVM report.
