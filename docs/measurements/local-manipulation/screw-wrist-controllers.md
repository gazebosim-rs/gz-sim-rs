<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Native SDF controllers on a lead screw and universal wrist

The standard joint velocity and absolute-position controller plugins can drive a local screw or a selected universal axis. The wrist replay shows a 1 kg offset tool tracking its child-axis setpoint while the sprung parent axis swings under gravity. [Play the controlled wrist in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9808%2F)

Both plugins use the selected SDF axis's finite velocity and effort limits. Position mode requires the standard `use_velocity_commands` option and a positive `cmd_max`. The controller divides the continuous position error by the public timestep and bounds the resulting velocity command. An independent native motor supplies at most the authored torque. Velocity mode addresses axis 0. Position mode accepts `joint_index` 0 or 1 on a wrist. A joint entity currently owns one SDF plugin, and the backend supports two independently replaceable native wrist motors. The selected coordinate can have passive springs and physical stops.

| Loaded experiment at 1 ms | Result |
|---|---|
| Raise a 1 kg nut to 7.1 rad with a 0.4 m/revolution screw, 1.7 rad/s command bound and 2 N·m effort | Reached **7.1 rad** through a full revolution, equivalent to **0.452 m** of lift. Peak speed **1.7 rad/s**. Reversing the target reached **4.5 rad**. Reset and replay agreed exactly. |
| Screw velocity mode, first 0.7 rad/s target against gravity | First tick **0.0571682 rad/s**, compared with **0.0571856 rad/s** from the torque limit and reflected inertia. Later ±100 rad/s commands saturated at **±2 rad/s**. |
| Tilted, loaded wrist, axis 1 position command −0.82 then +0.41 rad | Final positions **−0.8199995, +0.4099981 rad**. The other axis settled at **0.78233, 0.67681 rad** on its spring. Peak absolute residual speed at the two sampled endpoints was **0.000453 rad/s**. |
| First wrist axis velocity demand 0.7 rad/s while the second swings | **0.700092 rad/s** after 300 ms. Four extra inner iterations reduced the measured tracking error from 0.01018 to **0.000092 rad/s** in this loaded fixture. |
| Native wrist on a freely rotating parent | Equal-inertia bodies spun in opposite directions. Clearing the first motor preserved the independently commanded second axis. |

The browser verified 176 native frames and 704 visual poses and shapes through playback and rewind. Sixty release tests covered both new fixtures, actuator limits, free-parent reaction, reset, saturated targets, malformed SDF, lifecycle and the existing controller suite. Strict release Clippy passed. Changed production coverage reached 277/278 executable lines and 430/440 regions.

## Reproduce

Use an unused output directory. For the screw substitute `examples/worlds/local_robot_screw_controller.sdf` and record 5,000 steps. Serve gz-web-rs on port 8082 for the viewer link.

```sh
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- record examples/worlds/local_robot_wrist_controller.sdf \
  3500 20 target/my-controlled-wrist
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 9808 target/my-controlled-wrist/replay
```

[Wrist SDF](../../../examples/worlds/local_robot_wrist_controller.sdf) · [Screw SDF](../../../examples/worlds/local_robot_screw_controller.sdf) · [Robot controller regressions](../../../crates/gz-sim-scene/tests/local_robot_controllers.rs)
