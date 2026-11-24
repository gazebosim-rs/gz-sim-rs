<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# How fast the gripper pads compress and release

Two 80 g fingers squeezing a 200 g sphere reached 95% of their 0.100 mm static compression in **82.525–82.600 ms** across four simulation step sizes. The independent spring-and-damper prediction was **82.497 ms**. The earlier solver took 230–1,224 ms on the sustained grip. [Measured gz-web-rs replays and instructions](pad-transient-view.md) · [Tabulated results and timing batches](pad-transient.json)

## Loading and opening

The solver now preserves each authored pad's compressive velocity target and softness during the velocity relaxation pass. The previous pass imposed a rigid normal and removed part of the pad's velocity each substep. On a flat face the solver also shares the damping as well as the stiffness across the sampled contact points. In a separate check, two guided coated sliders compressed in series. The maximum error over 100 ms was 0.01674 µm at one point and 0.01682 µm at four points. A 50 µm starting gap carried zero load at rest and before contact in free approach. [Solver convention](../../../third_party/rapier3d-f64-0.36.1/PATCHES.md) · [Point, face and gap tests](../../../crates/gz-physics-rapier/tests/slider_pad_stiffness.rs)

For the normal-motion reference, the centered sphere starts at rest between two fingers in zero gravity. Equal commands keep it centered. Each finger obeys the closed-form overdamped equation with pad mass 0.08 kg, stiffness 30,000 N/m and damping 828.079 N·s/m, calculated from the normal effective mass of a finger and sphere and the native damping ratio of 10. The drive holds 3 N for 120 ms, steps down to 1.5 N for 8 ms, then reverses to −3 N for 6 ms. The reference locates the zero contact force during opening, then integrates free motion. The test compares both pads at every public tick and uses momentum change to predict the impulse over that tick. [Rust reference and regression](../../../crates/gz-sim-scene/tests/support/pad_transient.rs)

| Public step (µs) | Both pads at 95% (ms) | Worst loading position error (nm) | Worst opening position error (µm) | Worst opening impulse error (µN·s) |
| ---: | ---: | ---: | ---: | ---: |
| 200 | 82.600 | 34.50 | 3.593 | 27.23 |
| 100 | 82.600 | 16.97 | 1.844 | 12.97 |
| 50 | 82.550 | 8.42 | 0.887 | 6.95 |
| 25 | 82.525 | 4.20 | 0.433 | 2.71 |

The analytic contact unloaded 39.016 µs after the opening command. The first public tick reporting zero impulse on both pads ended at 200, 200, 100 and 75 µs respectively. At the coarsest step, the first opening tick missed the short residual load. The worst error in mean force over a single tick anywhere in the sweep was 0.139 N.

## Grasping under gravity

The gravity run uses the same pads and a friction coefficient of 0.5. Each finger presses at 3 N for 50 ms, then eases to 1.5 N for 110 ms. During the transition the sphere remains stuck until friction can no longer support its weight. Integrating the changing Coulomb limit from the independent normal-force solution predicts a drop of 13.962500 mm. The final solver gave 13.962255, 13.962124, 13.962284 and 13.962370 mm at the four step sizes. At 100 µs the prior solver gave 13.975051 mm. The corrected run reproduced after a loaded reset, including impulses and subsequent free fall. The visual replay extends the initial hold to show the approach to steady compression. [Grasp regression](../../../crates/gz-sim-scene/tests/two_finger_load.rs) · [Visual replay timeline](pad-transient-view.md)

## Step cost and compatibility

Each revision ran the same Bevy grasp benchmark on CPU 2 of a Ryzen 9 9950X3D. Each cell is the median of 15 batch p95s. A batch collects 500 individual steps after 200 warm-up steps at a 100 µs simulation clock. The batches alternate default and authored material order. Executions of the two binaries were interleaved on a shared host. [Timing batches](pad-transient.json)

| p95 step cost for authored pads (µs) | Hold | Slip | Release |
| --- | ---: | ---: | ---: |
| Before | 12.87 | 12.99 | 12.88 |
| After | 12.86 | 12.81 | 13.01 |

Four unmarked worlds reproduced all 4,429 post-header replay records byte for byte. An adjacent unmodified gripper also reproduced all 2,917 observations and 291 visual frames while the coated grip changed. Tests covered restitution at a loaded travel stop, equilibrium across three masses, slip, full release and reset. LLVM covered 15/15 changed solver lines and 27/27 regions, 64/64 native test lines and 140/140 regions, and 154/154 scene test lines and 306/306 regions. The separate frictionless seated-stop impact solve retains its hard impact target. These measurements qualify centered loading and guided flat faces. The next test is a tilted parcel rocking from face to edge and back under load.

```sh
export RUSTC_WRAPPER=
cargo test --locked --release -p gz-physics-rapier --test slider_pad_stiffness
cargo test --locked --release -p gz-sim-scene --test two_finger_load -- --test-threads=1 --nocapture
```
