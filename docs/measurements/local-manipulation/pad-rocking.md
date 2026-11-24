<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Force and torque as a gripped block rocks onto its edges

Two powered fingers squeeze a 200 g parcel at 3 N per side using 30,000 N/m pads. A short turning impulse rocks it away from full contact and the two jaws bring it back. The 24 mm block exposes the real impact on return. The 160 mm block also takes unloaded corners across the solver's 20 mm collision prediction distance while the opposite edge stays loaded. Both tests isolate normal contact with zero gravity and friction. [Watch both experiments and replay them locally](pad-rocking-view.md) · [Measurements at each step size](pad-rocking.json)

## Steady grip as distant corners leave and return

On the tall block, two corners on each pad remain loaded while the solver changes how many nearby corners it considers. The previous solver transferred the two distant corners' shares of the spring and damper to the supporting edge. A controlled 3 N test halved the edge compression from 0.200 mm to 0.100 mm when those corners passed outside prediction range. Keeping four equally weighted sites on the geometric box contact preserved the expected 0.200 mm. The moving block revealed a second error. The point where the normal force acted could lag behind the rotating corner until a collision update refreshed it. The solver now uses the current vertex as the line of action on frictionless box face–vertex contacts and reports the corresponding lever for torque readout. [Native implementation and scope](../../../third_party/rapier3d-f64-0.36.1/PATCHES.md) · [Native controls](../../../crates/gz-physics-rapier/tests/private/tilted_pad.rs)

| Tall parcel at 100 µs | Earlier left-pad force change | Revised | Earlier torque change | Revised |
| --- | ---: | ---: | ---: | ---: |
| Outward at 216.9 ms, four selected corners become two | +0.3370 N | +0.000417 N | −26.17 mN·m | −0.0211 mN·m |
| Return at 246.3 ms, two selected corners become four | −0.2538 N | −0.000368 N | +19.66 mN·m | +0.0172 mN·m |

Across the entire interval where the two corner levels were separated by 19–21 mm along the squeezing direction, the maximum pad force rate was 4.40 N/s at 100 µs and 4.39 N/s at 25 µs. A second test held the compact parcel at nine orientations. The high and low corners shared the load up to 0.47°; at 0.49° only the high edge carried load. From −14° to +14°, the measured strokes and signed restoring moments met the independent geometric tolerances of 0.1 µm and 1 µN·m. [Static and moving balance tests](../../../crates/gz-sim-scene/tests/support/pad_rocking_balance.rs)

## Force when the opposite edge strikes

A 6 N·m pulse for 200 µs makes the compact parcel rock 14.48° before it returns. Two previously unloaded corners close at about 0.6 m/s. The independent check uses that incoming speed, the parcel, slider and wrist inertias, and the normal damping coefficient to predict the force at contact onset. The left-pad peak at 1 µs was 193.91 N against 196.00 N predicted. The 200 µs clock captured 71.36 N. Both jaws returned to four loaded corners. The tall model uses a 75 N·m pulse for the same duration. [Compact world](../../../examples/worlds/local_rocking_pad_grasp.sdf) · [Tall world](../../../examples/worlds/local_tall_rocking_pad_grasp.sdf) · [Independent impact regression](../../../crates/gz-sim-scene/tests/support/pad_rocking.rs)

| Compact parcel | 200 µs step | 1 µs reference |
| --- | ---: | ---: |
| Largest pitch | 14.4067° | 14.4828° |
| Left-pad peak return force | 71.36 N | 193.91 N |
| Left-pad normal impulse, 38–40 ms after start of pulse | 28.528 mN·s | 28.423 mN·s |
| Left-pad angular impulse in the same window | 0.29763 mN·m·s | 0.29694 mN·m·s |

The regression checks the trajectory, both pads’ impulses, and the parcel’s linear and angular momentum. Across eight step sizes from 200 to 1 µs, errors in pitch and millisecond-averaged per-pad force and torque decreased with refinement. The largest integrated discrepancy across both jaws was 0.37% for normal impulse and 0.67% for angular impulse. The short return strike remains the priority for contact-local refinement.

## Timing and compatibility

On a Ryzen 9 9950X3D, nine interleaved release runs of each build on CPU 2 timed single Bevy steps at the 100 µs clock. Each run loaded the world and preloaded it for 200 ms, then measured 2,500 steps with commands submitted outside the timed calls. These are medians of the nine batch p95s on a shared host. [Raw timing batches](pad-rocking.json) · [Portable Rust timing harness](pad-rocking-view.tar.xz)

| Scene | First 60 ms, before / after | Full 250 ms, before / after |
| --- | ---: | ---: |
| Compact | 15.63 / 15.51 µs | 15.35 / 15.42 µs |
| Tall | 15.01 / 15.12 µs | 15.23 / 15.32 µs |

The material weighting applies to four-site geometric contacts between box colliders, including links with extra colliders; a modifying contact hook retains control of its selected contacts. The current-vertex lever correction is qualified for frictionless normal contact. A separate compact grasp with gravity and friction 0.5 slipped 3.814 mm and reproduced all 4,505 pre-change replay observations. Four earlier unmarked scenes reproduced another 4,429 observations. The final visual recordings agreed with independently sampled poses and matched across one and eight compute threads. [Viewer receipts and recordings](pad-rocking-view.md)

All 13 gripper regressions and 20 CLI replay tests passed. The native, friction and travel controls passed locally. LLVM covered 47/47 changed solver lines and 69/69 regions, 177/177 added native test lines and 292/292 regions, and 301/302 changed scene-test lines and 612/615 regions.

```sh
export RUSTC_WRAPPER=
cargo test --release --locked -p gz-sim-scene --test two_finger_load rocking -- --nocapture
cargo test --release --locked -p gz-physics-rapier --lib tilted_pad -- --nocapture
```
