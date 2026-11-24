<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Catching a returning edge between gripper commands

The parcel rocks onto one edge of each finger, then strikes the fingers with its returning corners. A 200 µs simulation step averaged most of that brief force peak away. The new policy caught the strike on both fingers by subdividing three of 2,250 public steps. All three runs held each jaw at 3 N and applied one 6 N·m pulse for exactly 200 µs. [Watch the three measured runs in gz-web-rs](pad-impact-refinement-view.md) · [Measurement data](pad-impact-refinement.json)

| Measured quantity | Ordinary 200 µs | Adaptive | Continuous 1 µs |
| --- | ---: | ---: | ---: |
| Left / right peak force (N) | 71.36 / 71.65 | 193.40 / 193.39 | 193.91 / 193.91 |
| Time of peak (ms) | 238.800 | 238.563 | 238.700 |
| Left / right normal impulse (mN·s) | 28.528 / 28.529 | 28.529 / 28.529 | 28.423 / 28.423 |
| Left / right angular impulse about the parcel’s Y axis (mN·m·s) | 0.29763 / 0.29892 | 0.29825 / 0.29832 | 0.29694 / 0.29694 |
| Native solves during 450 ms | 2,250 | 2,847 | 450,000 |

Peak force is the largest normal impulse divided by the duration of its native solve. Normal and angular impulses are integrated over 238–240 ms. The continuously fine simulation started from rest and kept the same public commands throughout. A fourth control replayed the same coarse approach up to 238.400 ms, checked the incoming body states for equality, and switched to continuous 1 µs steps at that point. From that matched approach, adaptive refinement reproduced both peak forces and their time. The largest parcel pitch difference against the matched control fell from 0.00977° to 0.00047°. Against the run that was fine from rest, it remained 0.21125°, and the peak was 137 µs early. Refining the impact resolves the local force pulse while preserving the timing error acquired during the approach. [Matched-state regression](../../../crates/gz-sim-scene/tests/support/pad_edge_impact.rs)

## When refinement runs

Each enrolled pair names the parcel, a compliant slider pad and the pad’s local face normal. While that face carries load, the policy remembers which parcel corners have separated. Each public tick it uses the relative linear and angular velocities to predict when a returning corner will cross the plane inside the finite pad footprint. A predicted crossing requests fine stepping and one follow-through tick. Overlapping requests from the two pads share the same solves. The refinement applies to the local native world for those ticks. A detached slider stops qualifying. The enrolled world uses the configured native-step grid. Replay reports a null angular impulse if a native contact does not expose the friction impulse needed to compute the full moment. This experiment uses two box pads and a rigid box parcel, with gravity and friction set to zero. [World and enrollment](../../../examples/worlds/local_refined_rocking_pad_grasp.sdf) · [Public commands](../../../examples/replay/refined_rocking_pad_grasp.json)

A reversed pulse and two changes of strength shifted the return in time. Across all three trials each jaw’s peak was within 0.7% of its continuous fine reference. Each trial selected three ticks. Holding the grip without a pulse selected none. A separate 1 kg body received 100 N on a selected tick. Pausing kept that force queued. Its subsequent 8 µm of travel over two public ticks confirmed that the force lasted exactly 200 µs. Five existing replay fixtures, including one with another refinement policy, reproduced all 4,448 prior observations byte for byte. [Public replay regression](../../../crates/gz-sim-cli/tests/local_cpu_replay.rs)

## Reproduce the measurements

From the repository root, run the comparisons and the replay. The replay writes the native pad peaks, their offsets within the current public tick and angular impulses on the parcel to standard output. The recording guide includes a portable three-run comparison and the gz-web-rs launch commands.

```sh
export RUSTC_WRAPPER=
cargo test --locked --release --no-default-features -p gz-sim-scene --test two_finger_load \
  refined_returning_edge_resolves_peak_at_the_original_actuator_clock -- --nocapture
cargo test --locked --release --no-default-features -p gz-sim-scene --test two_finger_load \
  compare_edge_refinement_step_cost -- --ignored --nocapture --test-threads=1
cargo run --locked --release --no-default-features -p gz-sim-cli --bin gz-sim-local-replay -- \
  examples/worlds/local_refined_rocking_pad_grasp.sdf examples/replay/refined_rocking_pad_grasp.json
```

## Step cost

The timing run pinned an optimized build to one CPU on the Ryzen 9 9950X3D. Each batch timed 1,250 public Bevy steps after the pulse. It excluded test-side sampling and kept the adaptive policy’s normal peak and impulse reporting enabled. The table reports medians across nine batches with rotating run order.

| Wall time | Ordinary | Adaptive | Continuous fine |
| --- | ---: | ---: | ---: |
| 95th percentile per public step | 18.05 µs | 15.80 µs | 3,220.06 µs |
| All 1,250 steps together | 17.24 ms | 24.53 ms | 2,993.65 ms |

The three selected steps per trial took 2.24–4.71 ms each (median 2.29 ms). Across paired batches on this shared machine, adaptive p95 was 2.4% higher than ordinary p95 at the median. A run-wide p95 largely measures the steps between impacts. The next phase will trace the timing drift during the loaded approach and test whether the return time can be corrected while keeping the command clock fixed.

## Verification

All 21 local replay integration tests and all 25 primary-contact tests passed. The 15 gripper tests passed including the separately invoked timing benchmark. The packaged adaptive recording reproduced byte for byte on one and eight Bevy threads. LLVM measured changed executable lines and source regions against the preceding phase. Each of the four changed production crates exceeded the 95% target on both measures. Scene integration covered 173/178 changed lines and 326/333 regions. The new returning-corner module and the changes to the solver, multifidelity backend and replay CLI covered every changed executable line and region. The JSON lists the numerator and denominator for each changed source file. [Coverage counts, control checksums and benchmark measurements](pad-impact-refinement.json)
