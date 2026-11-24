<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Catching a parcel while the gripper is moving

The 200 g parcel slipped 10.090 mm when the fingers stayed at 2.3 N. With feedback it slipped 0.462 mm and continued up with the carriage. The continuously fine run gave 0.464 mm. This moving-gripper example now drives the scene’s squeeze controller with either a relative-height reading or two simulated tactile pads. [Watch the ten phases in gz-web-rs](slip-catch-view.md) · [Numerical results and timing batches](slip-catch.json)

| Phase | Development and measured outcome |
| --- | --- |
| 1. Measure slip | Measured parcel height relative to the moving pads. The strong grip held, and the weak grip slid with the acceleration predicted from the forces on the fingers and parcel. |
| 2. Detect and catch | A sustained 0.1 mm drop and at least 5 mm/s of slide triggered tightening at 219 ms. The pose-based controller confirmed the catch at 236.2 ms. The contact-based controller confirmed it at 236.0 ms and estimated relative travel within 4.5 µm throughout the slide. |
| 3. Slow the sensor | Increasing the sample period from 0.2 to 5 ms delayed detection to 220 ms and increased slip from 0.462 to 0.534 mm. The actuator kept its 0.2 ms clock. |
| 4. Slow the fingers | Reducing the force ramp from 120 to 30 N/s per finger increased slip to 1.617 mm. The parcel settled at 261.2 ms. |
| 5. Cap the squeeze | The controller reported continued slip when the requested squeeze reached the 2.55 N ceiling. Holding lift effort fixed let the parcel slide 5.928 mm by 290 ms. |
| 6. Ease the lift | After three continuous milliseconds of sliding at the applied force ceiling, the controller cut lift effort from 10.4956 to 7.4556 N. The moving gripper caught the parcel after 0.468 mm, or 0.545 mm with the slower sensor. |
| 7. Increase the load | With a 260 g parcel at the original lift effort, the fingers arrested the slide after 1.240 mm. The final upward contact forces balanced the heavier accelerating load. |
| 8. Change the contact | Lowering the friction coefficient from 0.5 to 0.4 increased slip to 1.549 mm. Separately, mirroring the 4.5 mm mass offset swapped the load between the fingers and kept the catch distance within 1 µm of the original. |
| 9. Catch a second slip | Raising lift effort to 18.5 N at 270 ms caused another slide. The controller detected it from the reseated height and confirmed a second catch at 301.2 ms. This trial used a 6.5 N jaw limit. |
| 10. Open on command | At 265 ms an explicit release opened the fingers. Both contact forces vanished and the free parcel accelerated downward at 9.81 m/s² while the lift continued upward. |

## Independent mechanical check

During the weak-grip slide the fingers and parcel exchange 2.3 N of friction. The gripper has mass 0.56 kg and the parcel 0.20 kg, so their relative downward acceleration is (10.4956 − 2.3)/0.56 − 2.3/0.20 = 3.135 m/s². An ideal force ramp at 120 N/s per finger with a friction coefficient of 0.5 reduces that acceleration by 814.29 m/s³. Starting the ramp when the drop passes 0.1 mm plus the 1 ms confirmation time predicts a 218.987 ms trigger, a stop at 232.004 ms and a 0.460 mm drop. The controller requires a further 4 ms of measured settling to declare the catch. This check uses commanded forces, masses and friction independently of the contact solver. [Executable force-balance and trajectory checks](../../../crates/gz-sim-cli/tests/closed_loop_carry.rs)

## What the robot observes

The controller tightens only after both thresholds persist, limits squeeze demand, and holds the recovered force until commanded to release. The actuator applies increases on the next public tick and limits how quickly force can rise even when the sensor samples slowly. The lift interlock checks the **applied jaw command** and continued measured slip. A requested maximum can arrive several actuator ticks before the fingers reach it. The carry’s fallback lift effort equals the known loaded weight, so the carriage can keep moving while the upward acceleration reduces. [Squeeze controller](../../../crates/gz-sim-scene/src/grasp_guard/slip_catch.rs) · [Lift interlock](../../../crates/gz-sim-scene/src/grasp_guard/slip_lift_interlock.rs)

The tactile sensor measures relative surface velocity at each loaded contact, including the motion caused by rotation. It projects onto the load direction in the tangent plane, weights by the positive normal impulses, and requires contact on both fingers. The tactile run integrates this velocity from the initial grasp while separately recording the true parcel motion. All 401 samples from arming through the end of this run had bilateral contact. The measurement represents the last native solve of each public tick. Local rocking can move a surface differently from the parcel’s center of mass. The qualified fixture uses a vertical lift and independent dynamic links. Missing contact after arming latches a sensor fault and preserves the last squeeze. [Tactile sensor](../../../crates/gz-sim-scene/src/grasp_guard/contact_slip.rs)

## Run and verify

Use a new output directory per command. Replace `tactile` with `catch`, `open`, `unload`, `delayed-unload`, `repeat` or another case in the viewing guide to compare runs. Each output contains the native replay, public-tick observations and a summary.

```sh
export RUSTC_WRAPPER=
cargo run --release --locked --no-default-features -p gz-sim-cli --example closed_loop_carry -- \
  tactile adaptive target/my-tactile-catch
cargo run --release --locked --no-default-features -p gz-sim-cli --example closed_loop_carry -- \
  tactile fine target/my-tactile-reference
cargo test --release --locked --no-default-features -p gz-sim-cli --test closed_loop_carry -- \
  --include-ignored --test-threads=1
```

All 14 scenarios used the same 200 µs public tick in both modes. The fine reference made 200 native solves per tick. Across these smooth lifts the adaptive solver used one. The largest difference in the complete slip trajectories was 3.54 µm during retention and 18.80 µm during intentional release. Five adaptive cases reproduced their commands, measurements and replay archives with one and eight Bevy threads across 7,650 public ticks. [Replays and reproduction](slip-catch-view.md)

## Step cost and local verification

Three rotated release batches ran on CPU 5 of a Ryzen 9 9950X3D. Each batch measured 1,450 public Bevy steps for each case. The median batch p95 was 16.19 µs for the pose-based catch, 16.14 µs for the tactile catch and 15.75 µs for lift relief. The corresponding continuous-fine values were 2.252, 2.229 and 2.252 ms. These timings cover the Bevy simulation step. The example reads the feedback sensor and writes its recording outside that timed interval.

The ten carry integration tests passed, including the full fine-reference matrix. The 363 scene unit tests, 23 replay tests and 18 active two-finger tests also passed. LLVM exercised all 184 executable lines and all 228 regions of the three new scene modules. The runnable example and its fixture adapter reached 333/337 lines (98.81%) and 616/642 regions (95.95%). The new CLI integration tests reached 100% line and region coverage. A simulated short write discarded its partial recording and the same output name succeeded on retry. [Coverage by file](slip-catch.json)

The next robot phase is retaining a grasp when one finger briefly loses contact, then making a supported set-down when bilateral sensing cannot be recovered.
