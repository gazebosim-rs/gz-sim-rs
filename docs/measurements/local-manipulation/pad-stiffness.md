<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Normal stiffness of two gripper pads

A pad with stiffness 30,000 N/m compresses by 0.100 mm when it carries 3 N. The 40, 80 and 160 g fingers each reproduced that value on a 200 g sphere. The default 80 g finger reached 1.4776 mm. These are geometric overlaps measured from body poses. [Hold, slip and release in gz-web-rs](pad-stiffness-view.md) · [Numerical results](pad-stiffness.json)

## Specify the material

Add this plugin to the gripper model. Each link must carry a collision and be the child of a bounded, reduced-coordinate prismatic joint. The value is a finite positive stiffness in newtons per metre. Restitution can be authored alongside it. [Complete SDF](../../../examples/worlds/local_stiff_pad_grasp.sdf)

```xml
<plugin name="gz::sim::systems::ContactMaterial" filename="gz-sim-contact-material-system">
  <normal_stiffness link="left">30000</normal_stiffness>
  <normal_stiffness link="right">30000</normal_stiffness>
</plugin>
```

The solver shares the stiffness across the selected points in each contact manifold. If both touching links have an authored stiffness, their normal compliances add in series. A pair with no authored pad follows the existing native contact path. Reset reapplies the pad material from the initial joint recipe. [Native regressions](../../../crates/gz-physics-rapier/tests/slider_pad_stiffness.rs) · [Scene regressions](../../../crates/gz-sim-scene/tests/two_finger_load.rs)

## Load, friction and isolation

| Test | Measured result |
| --- | --- |
| Sphere, 3 N, 15,000 N/m | 0.200 mm per pad |
| Sphere, 4.5 N, 60,000 N/m | 0.075 mm per pad |
| Flat block, four points per face, left/right 15,000 and 60,000 N/m | At 3 N, 0.200 and 0.050 mm. At 6 N, 0.400 and 0.100 mm. Returning to 3 N restored the original overlaps. Swapping the materials reversed the result. |
| Sphere slipping for 110 ms, grip reduced to 1.5 N, friction coefficient 0.5 | 13.975051 mm downward against 13.975500 mm predicted. Opening the fingers removed the contact impulses. |
| Two contacting authored layers, 10,000 and 30,000 N/m | Equivalent stiffness of 7,500 N/m |
| Gripper beside an independently loaded gripper | The unchanged neighbor reproduced all 2,917 observations and all 291 visual frames. |

The one- and eight-thread hold and release recordings were byte-identical. The native WebGPU viewer matched 1,812 independently sampled link poses across 453 frames. The default grasp, inclined groove, powered slider and serial robot also reproduced their pre-change replay traces after the build header. The four-point test uses flat parallel faces in a purely normal load. More complex patch geometry and transient damping require separate qualification. [Four-point and cost tests](../../../crates/gz-sim-scene/tests/support/pad_patch.rs)

## Loading speed and step cost

Starting with the pads touching the sphere, each motor held 3 N for two seconds. At 1 ms observation intervals the first sample at which **both** pads reached 95% of 0.100 mm depended on the integration step.

| Step (µs) | Both pads at 95% (s) | Left / right compression at 100 ms (µm) |
| ---: | ---: | ---: |
| 200 | 0.230 | 73.25 / 72.88 |
| 100 | 0.373 | 58.58 / 54.49 |
| 50 | 0.657 | 41.08 / 35.19 |
| 25 | 1.224 | 25.42 / 20.27 |

The solver's relaxation pass clears the compliant velocity target after each integration substep. Next, preserve the physical normal response through that pass and compare both loading and release to an independent mass-spring-damper solution at multiple steps. Recheck sliding friction, impact restitution and loaded travel stops.

On the local host, five alternating batches measured 500 individual Bevy steps per interval after 200 warm-up steps, with the public clock fixed at 100 µs. The table gives the median of the five batch p95s. Other work shared the host. The raw batch results show the variation between runs. [All timing samples by batch](pad-stiffness.json)

| p95 step cost (µs) | Hold | Slip | Release |
| --- | ---: | ---: | ---: |
| Default | 12.63 | 12.72 | 13.00 |
| 30,000 N/m | 13.28 | 12.92 | 12.88 |

## Verification

LLVM covered 90/90 changed native solver and adapter production lines and 148/149 regions, including the local Rapier changes. Scene loading and validation covered 43/43 changed production lines and 77/78 regions. The four-point and cost tests covered 87/87 lines and 172/173 regions. The grasp suite passed all nine tests. Adjacent world spawning, groove, impact, travel and replay checks passed, along with formatting and strict Clippy on the changed integration targets.

```sh
export RUSTC_WRAPPER=
cargo test --locked --release -p gz-physics-rapier --test slider_pad_stiffness
cargo test --locked --release -p gz-sim-scene --test two_finger_load -- --test-threads=1 --nocapture
```

For the two-second sweep, use the supplied hold journal, set its step count to two seconds divided by the SDF step size, and record at a 1 ms interval. The published measurements use steps of 200, 100, 50 and 25 µs. [Journal](../../../examples/replay/pad_stiffness_hold.json) · [Record, serve and inspect the two grips](pad-stiffness-view.md)
