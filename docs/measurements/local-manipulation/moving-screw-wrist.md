<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# A lead-screw lift carrying a controlled wrist

A world-mounted screw turns and lifts a platform while a two-axis wrist on its moving edge steers a 0.4 kg tool. Both actuators run through ordinary SDF position controllers. During a loaded reversal, the measured wrist pivot gap stayed below 0.10 µm and the screw travel error below 0.07 µm. [Play the serial robot in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9809%2F)

| Test with a 1 ms step | Measured result |
|---|---|
| Move screw and wrist to 4.2 and 0.65 rad | Measured 4.20000008 and 0.65000000 rad, lifting the platform 0.267 m. Peak rates were 1.100063 and 0.750055 rad/s. |
| Reverse both targets to 1.3 and −0.55 rad | Measured 1.30000025 and −0.55000001 rad. Loaded reset reproduced each reported metric exactly. |
| Two free equal-inertia rotors with a 1 rad/s relative limit, then reverse the drive | Rotors settled at opposite ±0.5 rad/s on a screw and on each wrist axis. Maximum angular momentum error was 1.60 × 10⁻¹⁰ kg·m²/s. Maximum screw lead error was 0.208 µm. A common impulse spun both rotors together at 0.6 rad/s. |
| Narrow and wide loaded wrist stops at 0.24 and 2.3 rad, then −0.21 rad | Maximum stop error was 1.64 µrad. The adjacent zero-speed axis drifted less than 0.026 µrad. |
| Cost of an inactive moving-wrist governor | Five interleaved release runs of 5,000 steps each had median p95 71.74 µs with a 20 rad/s governor and 28.64 µs with the speed limit omitted. The wrist was passive in both cost runs. The separate two-controller replay measured 54.71 µs p95. |

A positive speed limit on a moving axis uses a pair of massless links with prescribed spin relative to the bearing. Native multibody Jacobians transmit the governor reaction to the moving bearing and conserve common-mode momentum. The wrist's first governor follows the parent, and its second follows the child. Free root velocity is published from the solved generalized velocity. Native reset replays the original order of auxiliary constraints so constraint coloring remains repeatable.

A joint attached to one of these links uses Rapier's multibody constraint solver. Its universal constraint projects a point along the child witness onto the parent witness axis. This gives the cosine of the witness angle, which enforces perpendicularity and bounds both narrow and wide stops using its supported linear rows. A transferred coupled angular cone initially allowed 0.02 rad/s of excess relative speed in the free-wrist trial.

Forty-three local release tests passed across the five focused targets, alongside strict Clippy. Changed production coverage was 249/253 lines and 428/435 regions. The browser verified 251 native frames and 1,506 visual poses and shapes through playback and rewind.

## Reproduce

The ordinary recording uses both controllers' authored initial targets. The test subsequently reverses them and repeats after reset. Choose an unused recording directory and serve gz-web-rs on port 8082.

```sh
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- record examples/worlds/local_robot_screw_wrist.sdf \
  5000 20 target/my-serial-robot
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 9809 target/my-serial-robot/replay
```

[Authored robot](../../../examples/worlds/local_robot_screw_wrist.sdf) · [Free-body and physical-stop checks](../../../crates/gz-sim-scene/tests/local_robot_moving_speed.rs) · [Two-controller reversal](../../../crates/gz-sim-scene/tests/local_robot_controllers.rs)
