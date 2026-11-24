<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Rigid wrist and sliding jaw at first impact

A 200 g sphere strikes an 80 g finger at 0.75 m/s, 20 mm from its center. The finger can slide freely along a guide. Its other five relative degrees of freedom are locked to a wrist that can turn freely about its pivot. The optional `ReducedScalarJoints` SDF model plugin builds these joint constraints as native Rapier multibody coordinates. Both links turn together immediately under the off-center impact. [Left strike](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10101%2F) · [Right strike](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10102%2F)


In the ordinary simulator's 200 ms recordings, the sphere's final position from either direction was within 0.914 µm of the analytical free-flight endpoint. Separate impulse joints left a 7.943 mm error at the same instant. The 200 ms check measures the sphere, and the 24 ms figures below measure the complete mechanism. [Both corrected worlds, recordings, browser checks and sphere positions](reduced-wrist-replays.tar.xz)

## First native contact

For offset `d`, restitution `e` and incoming normal speed `v`, the rigid impact impulse is `J = (1+e) v / (1/m_sphere + 1/m_finger + d²/(I_wrist + I_finger))`. The two moments of inertia about their centers are 0.000426666667 and 0.000032086667 kg·m². The first native impulse reached 65.317041434 mN·s at restitution 0.6, or 40.823150896 mN·s at zero restitution. The frictionless contact gave the predicted signed moment about the finger center, the equal/opposite parcel momentum change, and the predicted public hinge and slider velocities. The 0.6 trial had one loaded tick. With zero restitution the rotating finger stayed in light contact, receiving another 0.000499 mN·s by 200 µs at the 10 µs step.

| Step | Original impulse rows, e=0.6 | Original + 128 fixed-time PGS sweeps, e=0.6 | Reduced tree, e=0.6 | Original impulse rows, e=0 | Reduced tree, e=0 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 10 µs | 57.372 | 55.789 | 65.317 | 42.355 | 40.823 |
| 1 µs | 54.636 | 53.333 | 65.317 | 39.619 | 40.823 |
| 100 ns | 43.720 | 43.517 | 65.317 | 28.703 | 40.823 |
| 25 ns | 41.039 | 41.019 | 65.317 | 26.022 | 40.823 |

All impulse entries are first-contact mN·s. Each default impulse constraint has finite native joint softness. At shorter time steps an impulse can rotate the finger during that first solve before the joint rows transfer the motion to its wrist. Additional inner solver sweeps at the same simulation time also left an error. The reduced tree uses the constrained mass directly.

## Trajectory, controls and cost

The independently integrated two-coordinate free flight from the physical impact predicts finger center (0.019516200187, 0.098910817676) m and sphere center X −0.013866272625 m at 24 ms for the positive strike. The native peak position errors through 24 ms for both mirrors were **2.290 µm at 10 µs** and **0.230 µm at 1 µs**. The original impulse rows gave 2.383 mm and 3.202 mm for the same steps. The first impulse and angular speed also matched at 100 ns, 25 ns and 1 ns. The original impulse-conditioned oracle's 1 µm trajectory tolerance was exceeded at 10 µs, and its exact-zero centered angular pulse check measured 2.7×10⁻¹⁵ rad/s. The new regression uses an explicit step-scaled position bound and floating-point bounds on the public joint readings. The independent reference source is commit `2adb0b7`.

The checks reverse incoming direction and strike offset, hit through the finger center, miss the finger, and apply a measured point force without a sphere hit. They verify the public joint coordinates and repeat after a cold replay load and reset. A detached control kept its independent free-finger response. The native replay checks compared 108 and 100 visual poses and shapes across the 200 ms left and right recordings, respectively.

On an AMD Ryzen 9 9950X3D, the median across five 201-impact release runs pinned to core 2 gave the following wall times. Each sample times the advancing 10 µs Bevy step that first reports the collision. Reset and approach were outside that timer. Host load and scheduling affect the tails.

| Native representation | p50 | p95 | p99 |
| --- | ---: | ---: | ---: |
| Existing impulse rows | 7.86 µs | 9.42 µs | 11.56 µs |
| Reduced scalar tree | 9.28 µs | 11.33 µs | 14.87 µs |

## SDF admission and checks

Place the following marker on a local multi-fidelity model with one world mount, a connected passive tree of unbounded revolute or prismatic joints, and at least one prismatic joint. Model-local contact material and joint-state publication are supported. The loader rejects controls, elastic joint settings, finite travel stops, additional constraint owners and other joint types in the marked model. Paint, shapes, inertias, friction and restitution come from the ordinary scene. The public point-force input and native body/joint samples stay available.

```xml
<plugin name="gz::sim::systems::ReducedScalarJoints"
        filename="gz-sim-reduced-scalar-joints-system"/>
```

```sh
RUSTC_WRAPPER= cargo test --release --locked -p gz-sim-scene --test reduced_wrist_slider
RUSTC_WRAPPER= cargo run --release --locked -p gz-sim-cli --bin gz-sim-rs -- \
  examples/worlds/local_reduced_wrist_slider.sdf --physics-engine multifidelity --iterations 20000
```

Changed production coverage against `108ef65` came from the instrumented `reduced_wrist_slider` executable alone. Each line is an executable changed line in the LLVM LCOV export. Each code region is identified by its source start/end coordinates and intersects a changed line. Of the 24 release regression tests, five directly exercised this feature and 19 covered existing robot controllers, joints, mounted guides, tier switching and joint admission. Strict Clippy passed for the three affected libraries and the new integration test.

| Crate | Changed lines | Changed regions |
| --- | ---: | ---: |
| gz-physics-rapier | 24/24 | 32/33 |
| gz-physics-multifidelity | 11/11 | 9/9 |
| gz-sim-scene | 60/61 | 109/113 |

The local evidence directory `artifacts/local-wrist-slider-native` contains the native recordings, browser checks, fixed-time PGS controls, five pinned timing runs, analytical comparison and scoped LLVM exports.
