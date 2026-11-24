<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Powered slider at the end of its travel

A 2 N drive seats the jaw of a freely rotating wrist against a travel stop. At the first frictionless impact from a 200 g sphere, the 80 g jaw and wrist must share the reaction immediately. The ordinary SDF loader now accepts a finite prismatic travel range and a persistent `ApplyJointForce` on that coordinate in a `ReducedScalarJoints` model. The passive hinge remains free. The controller can reverse the force and release either stop. Reset clears the command and preserves the authored range.

At a 1 µs public step and restitution 0.6, the upstream native stop delivered 66.534227 mN·s. The coupled solve delivered the analytical 72.993188 mN·s, held the seated jaw to floating-point precision and gave the expected wrist angular velocity of −6.958617 rad/s. Moving the hit to the other side of the loaded jaw produced 65.317041 mN·s as it opened. The independent sweep used 60 combinations of mirror direction, restitution, preload, initial clearance and timestep. A 25 ns run resolved clearance as small as 50 nm before the incoming strike. A 1 µs public sample can combine two physical events occurring less than a microsecond apart. [Native replay and measured motion](powered-slider-view.md)

## Native solve

The marked finite slider uses its remaining travel and native substep duration to bound its generalized velocity. For a single loaded frictionless contact point on that tree, the solver first tries an unconstrained jaw. If the impulse would cross the stop, it solves the point reaction with the slider velocity fixed at the allowable bound and applies the resulting native stop reaction. The normal inverse effective mass changes from `1/a` to `1/(a − b²/c)`, where `a` is the normal mobility, `c` is the slider mobility and `b` is their cross term from the native mass-weighted Jacobians. The original normal compliance is preserved during integration. The outgoing target uses the originally measured approach speed after compression. The existing solver records the contact impulse once.

The block applies to one finite slider, one loaded frictionless point and one responding reduced tree. Multiple marked sliders, simultaneous loaded manifolds, contacts between two reduced trees, frictional or multi-point patches and effectively singular constrained masses retain the regular contact iteration. They can still move, carry loads and report native reactions. The two-stop control compared against the ordinary frictional path with negligible friction. Further controls swapped face and point, reversed native insertion order, delivered two simultaneous projectiles, applied genuine tangential sliding, and exercised unmarked gearing, motor, travel and dry friction.

## Cost of an observed impact tick

Five alternating release runs each warmed up for 20 trials, then timed 401 native 1 µs Bevy ticks with the same scene, public input and state publication. The table gives the median of the five run quantiles in wall-clock microseconds on a Ryzen 9 9950X3D pinned to CPU 2. Host scheduling influenced the upper tail. Reset and approach were outside the timed region.

| Native path | Before impact p50 / p95 / p99 | Impact p50 / p95 / p99 |
| --- | ---: | ---: |
| Upstream Rapier 0.36.1 with the same scene admission | 8.73 / 10.70 / 12.82 | 8.81 / 10.86 / 14.13 |
| Coupled native stop | 8.60 / 10.93 / 13.27 | 9.10 / 11.29 / 13.24 |

The median loaded tick increased by 0.29 µs. The identified two-row pair requires a bounded number of native normal and limit projections. No whole-island iteration count changes.

## Reproduce and audit

```sh
RUSTC_WRAPPER= cargo test --locked --offline --release -p gz-sim-scene --test reduced_slider_travel
RUSTC_WRAPPER= cargo test --locked --offline --release -p gz-physics-rapier --test native_slider_isolation
RUSTC_WRAPPER= cargo test --locked --offline --release -p gz-sim-scene --test reduced_slider_travel -- --ignored --exact measure_the_powered_slider_impact_tick --nocapture
git diff 2467dff 6b202dc -- third_party/rapier3d-f64-0.36.1/src
```

The baseline commit imports all 245 files from the published Rapier crate. The subsequent Apache-2.0 license text accompanies the source package. The vendored solver remains outside the 22 workspace members. Frozen production `6b202dc` passed all 30 scoped tests across six instrumented release objects. The scoped strict Clippy checks and both workspace and changed native-source formatting checks passed. Coverage was computed by exporting each test object with its own profile and taking the union of covered source coordinates within changed executable lines. [Raw run quantiles, object and profile hashes, and changed coverage](powered-slider-native.json)

| Production crate | Changed lines hit | Changed regions hit |
| --- | ---: | ---: |
| Vendored native Rapier delta | 135/137 (98.54%) | 206/210 (98.10%) |
| gz-physics-rapier | 6/6 | 8/8 |
| gz-sim-scene | 13/13 | 23/23 |
| gz-physics-multifidelity | Documentation only | Documentation only |
