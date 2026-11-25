<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# A position-controlled joint on a free-standing base

The unmodified Gazebo test world places a 0.6 kg rotor on a 120 kg base. Gazebo's position controller reads the relative joint angle and applies torque to the hinge every millisecond. The Rust replay and a separate Gazebo Sim 11.0.0~pre3 run using DART both moved the rotor to +1, −0.5 and +0.25 radians. Each command began after the preceding physics step finished. The base rested on its original ground plane throughout. [Original Gazebo world](https://github.com/gazebosim/gz-sim/blob/9e3b78f5de297def8fe47fd189500fb5cd6538ef/test/worlds/joint_position_controller.sdf) · [Runnable input](../../../examples/replay/upstream_joint_position.json)

| Target | Time held | Angle reached in Rust | Angle reached in native Gazebo |
|---:|---:|---:|---:|
| +1 rad | 50–450 ms | +0.999998880 | +0.999998860 |
| −0.5 rad | 450–850 ms | −0.499999432 | −0.499999402 |
| +0.25 rad | 850–1250 ms | +0.249999760 | +0.249999744 |

The comparison used all 1,250 advancing encoder samples. Before running it, we set limits of 0.10 rad on the maximum angle difference and 0.01 rad on each engine’s endpoint error. Both passed. The largest instantaneous angle difference was 0.05157 rad (2.95°), one step into the first reversal. The largest speed difference was 4.884 rad/s two steps later. Both engines reached −137.50 rad/s on the first reversal step. On the first positive step, their final speeds agreed at 91.67 rad/s while the Rust rotor turned 0.05729 rad and DART turned 0.09167 rad. That transient still matters for fast contacts. The stored native CSV was byte-identical on two independent launches. [Matched trajectories, comparisons, checksums and native startup log](upstream-joint-position.tar.xz)

The admitted force controller operates on a local scalar revolute model with one joint. It uses the signed position error, the previous sampled error, the weighted integral, separate integral and output clamps, the command offset and the axis effort cap. This world has an unlimited axis speed. The installed replay reads the stock empty SceneBroadcaster declaration and records the existing Rust scene snapshots. Reset clears the integral and derivative history, and paused updates preserve both. The regression also reduces the controller’s torque limit and the joint’s effort limit independently to 0.012 N·m. In each case the first-step joint speed is 0.100 rad/s. [Controller regressions](../../../crates/gz-sim-scene/tests/upstream_joint_position.rs)

## Run the world and compare the trajectory

With the preview binaries on the path, run the stock journal and compare it to the captured native trajectory. The capture and comparison tools are Rust. The capture calls the native Gazebo TestFixture bindings and uses the original controller’s Gazebo Transport double message. The comparator reads a replay journal and the native CSV.

```bash
out=$(mktemp -d "$PWD/position-check.XXXXXXXX")
tar -xJf docs/measurements/local-manipulation/upstream-joint-position.tar.xz -C "$out"
(cd "$out" && sha256sum --quiet -c SHA256SUMS)
gz-sim-local-replay examples/worlds/upstream_joint_position.sdf \
  examples/replay/upstream_joint_position.json --trace "$out/candidate.jsonl"
RUSTC_WRAPPER= cargo run --release --locked \
  --manifest-path tools/physics-audit/pendulum-native/Cargo.toml --bin position_compare -- \
  "$out/native.csv" "$out/candidate.jsonl" "$out/new-comparison.json"
```

To capture an independent native run, use the same native Gazebo and CPython bindings as the double-pendulum reference. Run from the repository root with a new output destination. [Native build and runtime environment](../../../tools/physics-audit/pendulum-native/README.md)

```bash
GZ_PARTITION="stock_position_$$" RUSTC_WRAPPER= cargo run --locked --release \
  --manifest-path tools/physics-audit/pendulum-native/Cargo.toml --bin position -- \
  examples/worlds/upstream_joint_position.sdf "$out/native-new.csv"
```
