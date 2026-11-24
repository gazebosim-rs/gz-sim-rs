# Real worker coverage in the headless qualification

The headless job now runs the existing Rust robot, CLI and recovery fixtures that require a stock physics worker. Cargo still runs its ordinary workspace tests first. The qualification then launches the selected parent tests against the worker Cargo built in that same profile. Each family runs serially and records its executable identity, selected cases, elapsed time, stdout and stderr. LLVM merges the profiles from the worker, controller and simulator with the ordinary test profiles.

The previous hosted headless run at `38c47d8` covered 71.73% of regions in `gz-physics-distributed`, 77.84% in `gz-sim-cli` and 84.41% in `gz-sim-scene`. One named test illustrates the gap. The ordinary grounded robot test exercises the local solver and its SDF parameters. It covered 161 of 194 SDF specification regions and zero of 511 regions in the separate worker-backed grounded robot implementation. The existing humanoid posture and chest-load fixture drove its Bevy scene, committed the loaded step through the stock worker and exercised 318 of those 511 regions in 16.62 seconds on this machine. Both measurements used matching executable objects and generated profiles.

For a second comparison, the same frozen test and worker objects ran their nine ordinary Cargo suites first. Adding the 18 worker-backed CLI, example and integration cases to those profiles reached 8,790 more regions in distributed physics, 1,701 in CLI and 2,771 in scene. Every binary ID in the merged profiles matched a retained executable. For example, the CLI tree operator rose from 62 to 554 of 686 executable regions; the scene’s worker-backed grounded robot rose from 0 to 451 of 511. The 21 distributed-library recovery cases were excluded from this comparison.

## What runs

The selection contains 39 top-level tests across nine Cargo targets. In the initial instrumented run, 38 cases passed individually and took a combined 655.93 seconds. The remaining case exposed a real compatibility problem in the historical two-sphere checkpoint: its cached total inertias on two links differ by a few last-place bits from present construction. Its authored inertias, solver parameters and construction digest matched. This case remains required in the headless selection.

The selected Linux fixtures cover joint loads and warm contacts across a controller loss, recovery of moving rigid bodies, separate and combined gripper contacts, owner changes, rejected reinstall, and continuation after save, reopen and reset. Operator tests use the actual CLI children for effort, motor, stopped-tree, fleet and convoy commands. Three humanoid tests cover grounded posture, adoption of an already loaded local robot, and return to local ownership. The selection and exact test names live in [the headless worker case list](../../../crates/gz-physics-distributed/tests/support/headless-worker-cases.json).

The fixtures require permission to launch and signal child processes, private pipes, loopback networking and writable temporary storage. The fixtures create their live checkpoints and input commands in test-owned temporary directories. The two-sphere regression also reopens a 16,553-byte format-v1 checkpoint from the original four-worker run at tick 1,292. The archived bytes are checked in as `pair-warm-v1-tick-1292.gzmultisphere` (SHA-256 `7bdbbe2133701d371b99008a012924a4dd5605b3dde04edaae34f4095ba25017`). Historical protocol compatibility fixtures require a separately built legacy executable. Those fixtures remain available to their dedicated qualification. Child-only crash entry points run under their owning parent test, which supplies the checkpoint, expected failure boundary and process handles.

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
