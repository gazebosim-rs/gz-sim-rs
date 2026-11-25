<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Repeated handoffs on the five-link serial arm

The arm completed 10,000 full park/wake cycles in one 20-second simulation. Each cycle contained two confirmed native tier changes and one advancing physics step in each tier. A second simulation ran all 20,000 steps in full dynamics. Both sequences repeated after a world reset. Changing the Bevy pool from one to eight threads produced the same digest of all observed physical states and held actuator targets, `b621fff2040a10d829f31df9a6f0cf5e4cc9fda01f836efa3f2391058a378bf9`. Four runs exercised 80,000 native transitions in total. [Results for both pools and resets](serial-switch-soak.json)

The isolated soak starts after delivery. It loads the obstacle workcell, places the 1 kg parcel on its receiver, retires the empty pickup tray, disengages the spindle and wrist position drives and advances both simulations for four seconds. The arm keeps its original rail, spindle, two wrist axes, spring-loaded jaws, masses, contacts and joint limits. The fingers close on empty space and the screw settles against its lower stop. Both controllers retain their setpoints for re-engagement. The complete pickup, approach, placement, return and support-loss exercise is separate. [Stock workcell](../../../examples/worlds/local_robot_serial_obstacle.sdf)

| Quantity over 10,000 cycles | Maximum difference |
|---|---:|
| Link position jump at a tier cut | 0 m |
| Whole-arm linear momentum jump at a cut | 0.887 µN·s |
| Whole-arm angular momentum jump about the origin at a cut | 0.496 µN·m·s |
| Kinetic energy discarded at a cut | 1.62 pJ |
| Link or parcel center position vs continuous full dynamics | 3.27 µm |
| Orientation vs continuous full dynamics | 18.22 µrad |
| Linear momentum vs continuous full dynamics | 3.19 µN·s |
| World mount impulse on the resumed step vs continuous dynamics | 0.047 µN·s |

Both runs reported 0.0241326 N·s upward at the mount on the first resumed step. The mount impulse comes from the measured change in the five links’ total linear momentum minus the gravity impulse on their 2.46 kg mass. The independent receiver held the parcel, with a reported cached normal impulse of 0.00981000 N·s. Each sleeping arm link kept its parked pose and exactly zero linear and angular velocity. The qualifier rejects any unexpected automatic wake or missing transition. Its comparison bounds are 0.1 mm and 1 mrad for the trajectory, 0.001 N·s and 0.001 N·m·s for the momentum cut, and 0.00025 N·s for resumed mount impulse.

## Why the handoff needed a solver correction

Setting each body's sleeping flag could leave the arm in the native awake solver set. A nominally parked arm then picked up a spurious downward velocity of 9.81 mm/s while its pose stayed fixed. The new handoff uses Rapier's existing connected-component sleep and wake machinery. It includes the private reference bodies that enforce the joint speed limits, verifies live contact and joint connectivity, and reconciles any pending split left by the earlier grasp. A partial connected group is refused before either the actuator or the solver state changes.

The earlier spindle workcell also parks above its stationary receiver. Rapier kept speculative rows across jaw-to-receiver and jaw-to-parcel gaps of 6.65–19.76 mm at zero impulse. The handoff now checks current shape clearance and the native graph, then retires only verified separated rows. The foreign parcel stays active with its own mass and support contacts. The parked group monitors its neighbors and repeats the interval in full dynamics if they move. An independent 0.5 mm gap test applies an impulse to the free neighbor and reproduces the continuous run's first contact tick, impulse and postimpact body states. Eligibility checks and the handoff itself require no world copy. Per-step rollback snapshots apply to the guarded proximity and speed-limit cases. [Native tests](../../../crates/gz-physics-rapier/src/joints/parked_robot.rs) · [Spindle replay and checks](../../../crates/gz-sim-cli/examples/parked_spindle.rs)

The original equilibrium check could also keep a freely hanging wrist awake indefinitely. In the passive standby probe it still refused at 30 seconds while the wrist was 7.44 µrad from vertical. The added test for a stable hanging configuration requires a center of mass below the pivot and both axes to span the horizontal plane. It bounds the combined gravitational and spring energy by the displacement equivalent of 20 µrad. An upright wrist with damping alone or weak springs stays in full dynamics. At exact torque balance, each free axis needs restoring stiffness above the inverted assembly’s gravitational stiffness. A preloaded limit can still carry a directed load. The regressions also cover a slightly tipped inverted wrist, off-vertical motion, an unbalanced spring and contact on both fingertips. The robot’s other speed, input and contact checks still apply. [Physics regressions](../../../crates/gz-sim-cli/examples/serial_switch_soak/tests.rs) · [Native island qualification](../../../crates/gz-physics-rapier/src/joints/parked_robot.rs)

## Cost on the qualification host

Each row contains 20,000 public Bevy steps on an AMD Ryzen 9 9950X3D. Times exclude state sampling, hashing and rendering. The elapsed switched cost includes the calls that park or wake the robot. The p95 and p99 values are for individual advancing steps. The report also records the cost of individual switches. Lane order alternated every cycle. These timings were taken on a shared development machine.

| Bevy pool and run | Switched / dynamic elapsed | Switched / dynamic p95 | Switched / dynamic p99 |
|---|---:|---:|---:|
| 1 thread | 1.70 / 2.52 s | 153.7 / 160.2 µs | 179.7 / 193.0 µs |
| 1 thread after reset | 1.61 / 2.41 s | 157.2 / 164.9 µs | 206.0 / 276.5 µs |
| 8 threads | 1.93 / 2.89 s | 151.6 / 170.1 µs | 284.5 / 376.3 µs |
| 8 threads after reset | 2.55 / 3.66 s | 188.9 / 229.9 µs | 467.8 / 884.2 µs |

From the simulator repository, run each pool size in its own process. The executable checks all bounds and verifies its full post-reset digest before writing its report.

```bash
RUSTC_WRAPPER= cargo build --locked --release --no-default-features -p gz-sim-cli --example serial_switch_soak
target/release/examples/serial_switch_soak target/serial-soak-one.json --threads 1
target/release/examples/serial_switch_soak target/serial-soak-eight.json --threads 8
```
