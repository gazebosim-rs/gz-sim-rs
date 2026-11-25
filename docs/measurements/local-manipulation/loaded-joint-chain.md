<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Six joint types carrying one tool

A single serial chain carries a 0.5 kg tool under gravity. Its other five links weigh 0.2 kg each. The world-mounted shoulder is a hinge, the bracket is welded to the shoulder, and a slide, ball socket, universal wrist and screw connect the remaining links. The shoulder, slide, both wrist axes and spindle receive simultaneous efforts for one second and then reverse for one second. The ball socket rotates freely. [World](../../../examples/worlds/loaded_joint_chain.sdf) · [Rust measurement](../../../crates/gz-sim-scene/examples/loaded_joint_chain.rs) · [Full results](loaded-joint-chain.json)

The check reconstructs the permitted relative motion of each joint from the measured poses of its two links and the reported joint coordinates after each of 8,000 public steps of 250 µs. The tolerances were 1 mm of anchor or travel error and 0.01 rad of constrained orientation error. The default solver missed the angular tolerance at the welded bracket, at 0.01082 rad. Four extra solver passes on this chain reduced that peak to 0.00469 rad. This setting applies only to the supplied chain.

| Joint | Measured motion with four extra passes | Peak anchor or travel error |
|---|---|---:|
| Revolute shoulder | Reached both ±0.55 rad stops under ±12 N·m | <0.001 µm |
| Fixed bracket | Maximum relative rotation 0.269° | 0.414 mm |
| Prismatic slide | Reached the 60 mm travel limit under 9 N | 0.151 µm |
| Ball socket | Swung through 3.14 rad while carrying the rest of the chain | 0.128 mm |
| Universal wrist | Both loaded axes moved, reaching 0.453 and 0.351 rad in magnitude under 1.4 and 1.1 N·m | 0.061 mm |
| Screw | 40 mm per turn, advanced to +6.945 rad and reversed to −7.000 rad under 0.14 N·m | 0.187 mm |

For the screw the error includes axial travel against the specified lead and lateral displacement. The universal rotation check composes the first parent-frame axis and second rotating axis. Separate moving-parent tests reversed drives with a 1 rad/s speed cap. The largest measured speed was 1.000000000002 rad/s for the screw and 1.000000000000 rad/s for the universal. The measured speed peaks of the uncapped six-joint run are in the JSON. [Moving-parent speed and reaction checks](../../../crates/gz-sim-scene/tests/local_robot_moving_speed.rs)

Run `./bin/loaded_joint_chain` from the extracted preview, or build from the simulator checkout. The command prints both measurements and exits unsuccessfully if the four-pass run misses the geometry or motion checks.

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-scene --example loaded_joint_chain
```
