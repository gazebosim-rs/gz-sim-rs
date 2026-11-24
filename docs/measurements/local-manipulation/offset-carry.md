<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# How much grip does an accelerating parcel need?

Two moving fingers lifted a 200 g parcel with its center of mass 4.5 mm to the right of its geometric center. At 4.2 N per finger the parcel rose 39.21 mm with the gripper. Reducing the squeeze to 2.3 N during the lift let the parcel slide 10.090 mm relative to the fingers over the next 80 ms. The same 2.3 N squeeze held the parcel when the lift stayed parked. These runs use gravity, friction and an actual force-driven vertical joint. [Compare the measured runs in gz-web-rs](offset-carry-view.md) · [World](../../../examples/worlds/local_offset_gripper_carry.sdf) · [Numerical results](offset-carry.json)

## Where slip begins

The lift pushes up with 10.4956 N and carries 0.560 kg of gripper plus 0.200 kg of parcel. With the grasp intact this gives an upward acceleration of 4 m/s². The parcel needs 2.762 N of upward friction, its mass times gravity plus its mass times the acceleration. For two opposed pads each squeezing with force N, the available traction is at most 2μN. With μ = 0.5, the lower bound is **2.762 N per finger**. At rest, the corresponding bound is 1.962 N per finger.

The offset also requires moment balance. A feasible static distribution for the 4.2 N grip uses equal upward shear forces and normal pressure centers differing in height by 2.96 mm, within the parcel’s 24 mm face. Several distributions can balance the same load. During the simulated lift the left pad supplied 0.965 N upward and the right supplied 1.797 N. Moving the center of mass to the other side swapped these readings to 1.797 N and 0.965 N. [Native force and moment regressions](../../../crates/gz-sim-scene/tests/support/offset_carry.rs)

The gripper closed for 150 ms and accelerated for 60 ms before the squeeze changed. At 210.4 ms the measured friction capacity first fell below the 2.762 N needed to carry the parcel. At 218.2 ms the parcel’s center of mass had moved 0.100 mm downward relative to the gripper. Both adaptive and continuously fine physics crossed this displacement threshold on the same 200 µs public tick. By 290 ms the respective drops were 10.089907 mm and 10.089632 mm, a difference of 0.275 µm.

| Squeeze per finger | Predicted state | Measured drop over 80 ms | Downward speed over final 10 ms |
| ---: | --- | ---: | ---: |
| 2.30 N | Below capacity needed | 10.090 mm | 236.22 mm/s |
| 2.76 N | Just below | 0.210 mm | 3.387 mm/s |
| 2.77 N | Just above | 0.037 mm | 0.036 mm/s |
| 4.20 N | Above | <0.001 mm | <0.001 mm/s |

Here a sustained slide exceeds 1 mm/s in the final 10 ms, and a held parcel moves below 0.1 mm/s in that window. The brief settling displacement is visible even at 2.77 N. The 1 µs reference gave 3.377 mm/s and 0.036 mm/s on opposite sides of the boundary. Changing the parcel to 150 g or 250 g moved the predicted limits to 2.217 N and 3.239 N. Reducing friction to 0.4 at 200 g raised it to 3.453 N. Runs on either side of all three limits reproduced the corresponding sustained-slip and hold outcomes. [CLI regression](../../../crates/gz-sim-cli/tests/local_cpu_replay.rs)

## A shove during the lift

In a third run the fingers kept their 4.2 N squeeze. A single 7 N·m torque pulse lasting 200 µs rocked the parcel 6.43° while the gripper traveled upward. The parcel reseated 2.669 mm lower relative to the fingers, against 2.671 mm in the fine reference. All commands and measurements kept the 200 µs public clock.

| Left pad on return | Ordinary 200 µs | Adaptive | Continuous 1 µs |
| --- | ---: | ---: | ---: |
| Peak normal force | 45.31 N | 152.01 N | 151.14 N |
| Time of peak | 230.600 ms | 230.326 ms | 230.334 ms |
| Native solves in the full 290 ms | 1,450 | 2,946 | 290,000 |

During the shove and return, four public ticks used 200 native solves, 100 used eight and 1,346 used one. Both smooth carries stayed at one native solve per tick, including while the parcel slid. Existing contact and friction laws reproduced the slip boundary and the shove reference in this fixture. The next phase will test whether the gripper can detect the developing slip and tighten in time to arrest it while the lift keeps moving.

## Run locally

From the simulator checkout, record either carry or the separate perturbation. The recording guide includes launch commands for gz-web-rs.

```sh
export RUSTC_WRAPPER=
cargo run --locked --release --no-default-features -p gz-sim-cli --bin gz-sim-local-replay -- \
  examples/worlds/local_offset_gripper_carry.sdf examples/replay/offset_carry_slip.json \
  --record target/my-offset-carry --record-every 5
cargo test --locked --release --no-default-features -p gz-sim-cli --test local_cpu_replay \
  accelerating_carry_crosses_the_predicted_friction_limit
cargo test --locked --release --no-default-features -p gz-sim-scene --test two_finger_load \
  offset_carry -- --nocapture
```

[Strong grip](../../../examples/replay/offset_carry_hold.json) · [Reduced grip](../../../examples/replay/offset_carry_slip.json) · [Strong grip with a shove](../../../examples/replay/offset_carry_rock.json)

## Cost and verification

Three rotated release batches ran on CPU 5 of a Ryzen 9 9950X3D. Each benchmark timed 1,450 public simulation steps. The table shows the median batch p95 for a public step and the wall time for the complete 290 ms maneuver.

| Maneuver | Ordinary p95 | Adaptive p95 | Fine p95 | Adaptive wall time | Fine wall time |
| --- | ---: | ---: | ---: | ---: | ---: |
| Strong grip | 15.85 µs | 16.35 µs | 3.93 ms | 21.35 ms | 3.44 s |
| Weakened grip | 15.90 µs | 16.63 µs | 2.30 ms | 21.49 ms | 3.30 s |
| Shove | 16.16 µs | 96.39 µs | 2.39 ms | 39.82 ms | 3.33 s |

All 23 public replay CLI tests passed. The native regressions checked linear and angular momentum, the mirror-image load distribution, and the impact time and force against the fine run. LLVM covered 434/434 changed executable test lines and 868/869 changed source regions. This phase qualified the existing solver with new fixtures and tests. [Timing batches, coverage and raw controls](offset-carry.json)
