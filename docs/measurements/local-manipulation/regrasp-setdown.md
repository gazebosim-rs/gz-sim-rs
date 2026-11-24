<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Recovering a lost finger contact and setting the parcel down

A rising two-finger gripper recovered its 200 g parcel after a finger was pushed away. When the regrasp attempt timed out, a placement controller braked the carriage, lowered the parcel onto a selected pedestal, opened the fingers and lifted them clear. Both fingers carried at least 4.20 N of normal load throughout the 10 ms leading up to the nominal landing. The pedestal carried 1.962 N at the end. [Watch the twenty phases](regrasp-setdown-view.md) · [Native measurements, references and momentum audit](regrasp-setdown.json)

## Contact loss during a moving lift

A 12.2 N outward disturbance on a finger link can separate the surfaces even while its motor is trying to close. The default recovery controller qualifies one millisecond of low pressure, increases squeeze by up to 100 N/s per finger, and requires three milliseconds of settled bilateral contact to complete a repair. A missing sensor packet has its own state. The first contact-speed sample after a gap sets a fresh displacement baseline. If recovery takes 24 ms, the controller hands the job to the lift. Each measurement below uses a 200 µs public actuator clock. The reference uses 200 native solves per public tick. [Rust controller](../../../crates/gz-sim-scene/src/grasp_guard/contact_recovery.rs)

| Phase | Physical test | Adaptive result |
| --- | --- | --- |
| 1 | Undisturbed grasp | Held the parcel with 4.2 N per finger and no measurable slide. |
| 2 | Left finger pushed away | Recovered at 219.4 ms after 0.133 mm of slip. |
| 3 | Right finger pushed away | Recovered at 218.8 ms after 0.129 mm. |
| 4 | Sensor packet withheld for 5 ms | Marked the reading unavailable. Motion matched the undisturbed grasp. |
| 5 | Contact returns before the debounce interval | Registered the loss and resumed holding without increasing force. The parcel moved 0.006 mm. |
| 6 | Bounded squeeze through repeated disturbances | Reached the 5.2 N cap and recovered. Peak slide was 1.161 mm. |
| 7 | Reacquisition after a longer right-side disturbance | Established a new bilateral baseline and confirmed the grasp at 223.6 ms. |
| 8 | Contact interrupted again while settling | Discarded the partial settling window. Both grids confirmed the grasp at 225.0 ms. |
| 9 | Left and right faults on the same lift | Confirmed the two recoveries at 219.4 and 253.0 ms. |
| 10 | Prolonged left-side loss | Requested lowering at 234.2 ms, one public tick later than the fine run. |

These ten recordings measure regrasp on an accelerating carriage. The second experiment below starts from rest and applies an opening drive to one jaw during the lift. Its six guided placements restore closing effort on both jaws when lowering starts. The extra jammed-jaw control keeps the fault applied.

## Emergency placement

The controller uses vertical carriage velocity to brake and descend. It measures the signed upward impulse on the parcel from the designated receiver. Permission to open requires 20 uninterrupted milliseconds at 80–150% of the parcel weight, a horizontal center-of-mass margin over 1.5 mm and vertical parcel speed below 18 mm/s. Retreat requires both jaws to unload and physically clear the rotated parcel. Completion requires another 20 ms of supported, quiet placement after at least 10 mm of retreat. [Placement controller](../../../crates/gz-sim-scene/src/grasp_guard/recovery_placement.rs) · [Release gate](../../../crates/gz-sim-scene/src/grasp_guard.rs)

| Phase | Change during placement | Adaptive / continuously fine outcome |
| --- | --- | --- |
| 11 | Nominal guided set-down | Receiver loaded at 519.4 / 519.6 ms. Fingers opened at 542.2 / 542.0 ms and cleared at 548.8 ms on both grids. |
| 12 | Slower descent | Opening at 722.6 / 722.4 ms. The slower approach reduced the measured landing peak. |
| 13 | Pedestal 3 mm higher | Opening at 510.2 / 509.6 ms. |
| 14 | Pedestal below the available stroke | Both stopped with closed jaws at the travel limit. |
| 15 | 260 g parcel with proportionately larger inertia | Opening at 527.2 / 523.2 ms. The receiver ultimately carried 2.551 N. |
| 16 | Mirrored mass offset and opposite failed finger | Opening at 542.0 ms on both grids. |
| 17 | Receiver measurement unavailable for 7 ms | The confirmation window restarted. Opening at 552.2 / 552.4 ms. |
| 18 | An unrelated obstacle beneath the parcel | Measured obstacle contact and held. The designated receiver carried zero load. |
| 19 | Receiver out of reach | The bounded search ended in a hold without opening. |
| 20 | Contact too close to the deck edge | Both grids registered support and withheld release. |

### When the failed finger stays open

A persistent −6 N opening drive on the left finger let the parcel fall freely for the last 19.8 ms before impact. Both grids detected the same landing at 312.6 ms, at 0.576 and 0.578 m/s downward. The guided nominal approach was about 0.075 m/s downward. The original controller would have authorized opening because the stand was carrying weight. The corrected controller also checks for five continuous milliseconds of bilateral guidance **before** first receiver load. It reported an unguided landing, stopped the lift and made no release or placement claim. Reaching the intended stand alone cannot establish that the robot carried the parcel there.

## Receiver impact and the fine reference

The largest guided trajectory discrepancy was **0.146 mm** for the 260 g parcel. The error crossed 0.025 mm at 256.2 ms with only one pad loaded, crossed 0.100 mm at 279.6 ms and reached 0.145 mm by 400 ms, before either run touched the pedestal. Across the full nominal placement the largest difference was **0.0525 mm**. The two grids declared nominal placement complete on the same public tick at 687.2 ms.

We also tried adding the pedestal’s top face as a prospective impact pair to the existing gripper refinement policy. In the heavy case the first 5 ms peak improved from 77.17 to 69.42 N against 70.22 N in the fine run, but the 4.0 ms release-time difference persisted. In the nominal case the peak improved from 78.09 to 69.31 N against 71.78 N, while the opening decision shifted from 542.2 to 566.8 ms. The coarse follow-up intervals measured load below the release threshold and restarted the confirmation window. The qualified configuration keeps the original pad policy. The comparison used matched commands and initial states on the same local rig. [Time, peak force and solve counts](regrasp-setdown.json)

## Run locally

From the simulator checkout, use a fresh directory for each recording. Each example writes per-tick observations, a summary and a native replay under `epoch-0`. Both accept `fine` as the second argument and `--threads N` for the Bevy worker count.

```sh
export RUSTC_WRAPPER=
cargo run --release --locked --no-default-features -p gz-sim-cli --example local_regrasp -- \
  repeat adaptive target/my-double-regrasp
cargo run --release --locked --no-default-features -p gz-sim-cli --example regrasp_setdown -- \
  nominal adaptive target/my-supported-placement
cargo run --release --locked --no-default-features -p gz-sim-cli --example grasp_impulse_balance -- \
  --max-error 1e-9 target/my-double-regrasp target/my-supported-placement
```

The momentum check compares the change in parcel mass times vertical velocity with gravity and the signed contact impulses on the parcel. It starts at the first recorded velocity, rejects missing public ticks, and includes intervals with one finger in contact, both fingers, a receiver or an unrelated obstacle. The reported residual is in N·s. The viewing guide includes commands for starting gz-web-rs and replaying the portable recordings. [Viewer and recording bundle](regrasp-setdown-view.md)


## Cost and verification

Three rotated release batches on logical CPU 5 of a Ryzen 9 9950X3D measured the public Bevy step. Median batch p95 was 16.30 µs for the double regrasp, 26.73 µs for nominal placement and 26.94 µs for the heavier parcel. Continuously fine physics took 2.616, 3.766 and 4.166 ms respectively. The timing excludes recording and the feedback calculations that run between simulation steps. [Per-batch times](regrasp-setdown.json)

The independent momentum check covered 149,958 intervals across 42 recordings. The largest per-tick discrepancy was 2.71 × 10⁻¹⁴ N·s. The ten adaptive regrasp trials reproduced their trajectories and commands with one and eight Bevy threads, as did the nominal, heavy, obstructed-edge and jammed placements. Both new scene controllers reached 100% line and region coverage (244 lines, 303 regions). Every changed executable Rust file exceeded 95% in both measures. The 374 scene unit tests and the local replay, gripper and momentum regressions passed. The next local physics phase is to trace the heavier parcel through braking and jaw reacquisition and reduce the opening-time error that originates before the receiver impact.
