<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# A two-axis sprung wrist carrying a 1 kg tool

A tilted universal wrist now accepts independent spring stiffness, damping and preload on either SDF axis. In the replay an offset tool swings under gravity before its springs settle. Axis 1 rotates with the parent. Axis 2 rotates with the child even after the wrist folds. [Play the native wrist in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9806%2F)

Each authored law adds one force-based angular velocity row to the native solver. The solver uses the continuous measured angle and the backward-Euler spring law at each rigid step. Reversing the body order for axis 2 makes its torque follow the tool at any wrist angle. Springs, commanded joint torques and stops act independently. Unconfigured axes add zero rows. Reset rebuilds the configured rows, and removing a joint or an endpoint retires them.

## Measured results

| Test at 1 ms | Result |
|---|---|
| Springs isolated in turn, −7.5 rad reference, 0.03 kg·m² inertia, k = 0.7 N·m/rad, d = 0.13 N·m·s/rad | Maximum analytical trajectory disagreement **0.01025 rad**, sampled every 10 ms on each axis. The inactive angle stayed below 0.0000001 rad. Both SDF joint spellings passed. |
| Tilted wrist, gravity and 0.31 m tool offset | Angles **0.689736, −0.480839 rad** at 5 s. Maximum residual about either axis **0.000220 N·m**. |
| Same wrist with additional sustained +2.1 and −1.4 N·m | Angles **1.482781, −1.407016 rad** after another 5 s. Maximum residual **0.000495 N·m**. Reset reproduced the final state exactly. |
| Free parent, pure axis-2 damper, torque 0.3 N·m and damping 0.15 N·m·s/rad | Relative rate within 0.0005 rad/s of **2 rad/s**. The two equal-inertia rotors spun in opposite directions, conserving angular momentum. Removing the joint released both bodies. |

Torque balance uses the published tool pose, the measured moment arm crossed with gravity, the spring torque and the applied load. The tilted loaded test removes the fixed stand so that the two springs and gravity account for every external reaction. Keeping the stand at the high load adds a physical stop when the swinging tool hits it. The native viewer retains the stand and shows the gravity-only trajectory.

All 9 universal scene tests, 8 native spring tests and 11 screw regressions passed. LLVM covered all 147 changed executable production lines and all 255 changed regions against the preceding screw phase. gz-web-rs verified the entire 176-frame archive, including playback, rewind and 704 native visual poses and shapes.

## Reproduce

Run from the simulation workspace with the gz-web-rs frontend on port 8082. Choose a new directory for each recording.

```sh
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- record examples/worlds/local_robot_universal_springs.sdf \
  3500 20 target/my-wrist-springs
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 9806 target/my-wrist-springs/replay
```

[Authored wrist](../../../examples/worlds/local_robot_universal_springs.sdf) · [Scene regressions](../../../crates/gz-sim-scene/tests/local_robot_universal.rs)
