<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# A free jaw struck by a free parcel

A 200 g parcel approaches an 80 g finger-sized block at 0.75 m/s across a 50 µm gap. Both boxes can move and rotate in all directions. The centers and face normals align, gravity is zero, and both bodies have zero friction and restitution. The mirrored scene reverses the incoming velocity and contact normal. [Watch the left approach](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10093%2F) · [Watch the right approach](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10094%2F) · [Both recordings and world files](free-jaw-replays.tar.xz)

Dividing the gap by the closing speed gives the exact impact time of **66.666667 µs**. Conservation of momentum and the plastic impact condition give both boxes the same outgoing speed of **0.535714286 m/s** and zero spin. Multiplying the initially stationary jaw’s mass by that speed gives its impulse magnitude of **0.042857143 N·s**. The impulse on the parcel has the opposite sign. Reversing the approach reverses both velocities and impulses.

## Measured at one millisecond

The first loaded time is the end of the first native solve that reported positive normal impulse. The final two columns show the maximum absolute departure from the analytic answer across both mirrors and, for speed, both bodies.

| Solve | First loaded end | Impulse error (N·s) | Axial speed error (m/s) |
| --- | ---: | ---: | ---: |
| 64 uniform calls, 1 ms public step | 78.125 µs | 3.31 × 10⁻¹⁵ | 1.12 × 10⁻¹⁶ |
| 25 ns early window, 1 ms public step | 66.675 µs | 1.77 × 10⁻¹³ | 1.93 × 10⁻¹² |
| Ordinary 100 ns public steps | 66.700 µs | 4.38 × 10⁻¹⁵ | 5.49 × 10⁻¹⁴ |
| Ordinary 25 ns public steps | 66.675 µs | 1.12 × 10⁻¹² | 6.22 × 10⁻¹⁵ |

All four solves located impact within one sampling interval and conserved total axial momentum to 8.4 × 10⁻¹⁷ N·s. The early window reduced the timestamp error from 11.458 µs to 8.33 ns. The coarse solve already resolved the total impulse accurately in this centered two-body case. Maximum measured spin on the early window was 5.18 × 10⁻¹¹ rad/s. In the two native recordings, each box center stayed within 1.07 × 10⁻¹⁰ m of its analytic trajectory across all 12 recorded times per mirror through 320 ms.

The Rust regression compares each body's measured momentum change with the signed contact impulse. The independent fine controls remove the refinement plugin, start from the original poses, and sum native contact impulses on ordinary public steps. The full robot also has off-center contact, joints and actuator loads, so it needs a separate articulated impact calculation.

~~~sh
RUSTC_WRAPPER= cargo test --release --locked --no-default-features \
  -p gz-sim-scene --test free_jaw_impact -- --nocapture
RUSTC_WRAPPER= cargo run --release --locked --no-default-features \
  -p gz-sim-cli --bin gz-sim-rs -- examples/worlds/local_free_jaw_impact.sdf \
  --physics-engine multifidelity --iterations 320
~~~
