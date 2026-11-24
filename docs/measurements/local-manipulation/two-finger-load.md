<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Load sharing in a two-finger grasp

A 200 g sphere weighs 1.962 N. Two opposed vertical pads with friction coefficient 0.5 must each squeeze with at least 1.962 N to hold it. At 3 N per finger, the measured upward force was 0.98117 N on the left and 0.98083 N on the right. At 1.5 N per finger the two faces can supply only 1.5 N upward in total. The sphere should drop 13.9755 mm in 110 ms. Both faces were still in contact when the motors opened. [Watch the measured grip, slip and release](two-finger-view.md)

| Public physics step | Measured drop in 110 ms | Error in drop | Residual sphere spin about the transverse axis |
|---:|---:|---:|---:|
| 200 µs | 13.974207 mm | 1.293 µm | −0.0520 rad/s |
| 100 µs | 13.974695 mm | 0.805 µm | −0.0405 rad/s |
| 50 µs | 13.975060 mm | 0.440 µm | −0.0253 rad/s |

The sphere held with the motors 2% above the calculated threshold and slipped 2% below it, including with gravity reversed. Swapping native body insertion order preserved the measured support. With unequal drives of 6 N and 2.5 N, the stronger finger translated the parcel sideways until its closing stop took 3.4994 N. Swapping the commands transferred the load to the other stop. Stop force here and below is inferred from each pad’s change in momentum, applied drive and contact impulses. Positive values denote force away from the engaged limit. [Flat-pad regression](../../../crates/gz-sim-scene/tests/two_finger_load.rs)

## Both stops carrying a sliding parcel

A second world puts the sphere in a 90° V-groove and starts it moving at 0.5 m/s along the groove. Each jaw pushes inward with 0.5 N. During sliding, symmetry gives a normal force of 1.38734 N per jaw, lengthwise friction of 0.69367 N, and an inward reaction from each opening stop of 0.481 N. The sphere spins up until its surface and both pads move together along the groove. For the specified solid sphere the rolling transition is predicted at 32.0356 ms. The regression detects the first tick where relative lengthwise speed at **both** measured contact points falls below 1 mm/s. [Watch the V-groove in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11434%2F) · [Portable replay and browser captures](groove-grasp.tar.xz)

| Physics step | Mean inferred opening-stop forces, left / right | First rolling sample | Maximum lengthwise trajectory error over 80 ms | Maximum vertical drift |
|---:|---:|---:|---:|---:|
| 100 µs | 0.479625 / 0.479628 N | 32.200 ms | 13.38 µm | 29.26 µm |
| 50 µs | 0.480231 / 0.480234 N | 32.100 ms | 7.33 µm | 15.77 µm |
| 25 µs | 0.480592 / 0.480594 N | 32.025 ms | 3.85 µm | 8.20 µm |

The mean forces use steps ending from 2 to 10 ms, while the sphere is sliding. At 25 µs both jaws stayed within 0.47 nm of their opening stops in that window. At 100 µs their sampled speeds could still reach roughly 0.53 mm/s there. Reversing the incoming motion reproduced the trajectory error. Removing friction preserved the supporting normal force and eliminated spin-up. In the frictionless control the maximum vertical drift fell from 57.85 µm to 16.53 µm as the step decreased from 100 to 25 µs. The existing native generalized contact rows resolved these cases. [Groove geometry](../../../examples/worlds/local_groove_grasp.sdf) · [Independent momentum, spin, travel and convergence regression](../../../crates/gz-sim-scene/tests/groove_grasp.rs)

## What happens during a longer hold

Under a constant 3 N squeeze for ten seconds, the parcel stayed within 8.5 nm of its starting height at the 100 µs step. Each pad nevertheless advanced about **1.48 mm** into the nominally rigid sphere. Peak jaw travel at ten seconds was 1.478, 1.477 and 1.449 mm at 200, 100 and 50 µs. Halving the step alone made little difference to sustained penetration. This is the next local contact problem to resolve. To repeat the 100 µs hold, keep only the first three commands of the flat-jaw journal and set the step count to 100000. [Flat-jaw world](../../../examples/worlds/local_two_finger_load.sdf) · [Command journal](../../../examples/replay/two_finger_load.json)

Three 20 s holds identified the source. Rapier's dynamic contact has a 30 Hz natural frequency. For this centered squeeze the normal effective mass is the reciprocal of the sum of the inverse pad and sphere masses. The equilibrium compression of one contact is its 3 N load divided by the effective mass times the squared angular frequency. Substituting 40, 80 and 160 g pads predicts 2.53303, 1.47760 and 0.94989 mm. Each pad’s measured overlap agreed within 1.2 nm. These are numerical contact stiffnesses. The next comparison should use an authored stiffness in N/m and check load versus compression across all three pad masses. [Twenty-second measurements and inputs](two-finger-compliance.json) · [Native contact spring coefficients](../../../third_party/rapier3d-f64-0.36.1/src/dynamics/integration_parameters.rs)

## Run the comparisons

From the simulator checkout, run the following commands with fresh output paths.

```sh
export RUSTC_WRAPPER=
cargo test --locked --release --no-default-features -p gz-sim-scene \
  --test two_finger_load --test groove_grasp -- --nocapture
cargo build --locked --release --no-default-features -p gz-sim-cli \
  --bin gz-sim-local-replay --bin gz-sim-rs
target/release/gz-sim-local-replay examples/worlds/local_groove_grasp.sdf \
  examples/replay/groove_grasp.json --trace target/groove.jsonl \
  --record target/groove-replay --record-every 10
target/release/gz-sim-local-replay examples/worlds/local_groove_grasp.sdf \
  examples/replay/groove_grasp.json --threads 8 --verify target/groove.jsonl
target/release/gz-sim-rs --server --replay --web-viewer 11434 target/groove-replay/epoch-0
```

The ordinary local replay trace now reports world-axis normal and friction impulse vectors in N·s on the lower-numbered body in each pair. The other body receives their negatives. An unreported grouped friction vector is JSON null for a loaded pair. An unloaded point contributes zero. A refined tick uses the impulses accumulated across the full tick, including any collision that ended before the final substep. The earlier scalar pressure totals remain available. [Replay observation](../../../crates/gz-sim-cli/src/bin/local_replay/observation.rs)
