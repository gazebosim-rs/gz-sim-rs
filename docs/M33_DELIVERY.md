<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M33: mobile-robot IMU and lidar coexistence

M32 is accepted at Main `679c99902469fdc9a39499a34b7d0222689734df`;
release `439342072` (`m32-preview`) is verified Latest. Continue directly under
D30–D32. This phase is budgeted at most five working days including review and
mandatory checks. Scope and exclusive leases are recorded before authoring.

Deliver a separate `examples/m33_robot_sensors.sdf`: retain every M19 robot,
altimeter, pressure, magnetometer, JointStatePublisher, DiffDrive and OBJ setting;
add one centered base-link IMU at 100 Hz, one offset planar CPU lidar at 20 Hz,
and an ordinary box target. Keep the old example and installed M32 robot exact.
Use existing admitted IMU acceleration-X Gaussian mean 0.1/stddev 0.01 and
angular-Z quantized mean 0.02/precision 0.01, with other noise parameters default;
use the existing lidar Gaussian mean 0/stddev 0.01, minimum zero, three beams
across ±pi/4 and maximum 10 m. Do not introduce new noise models or admission.
Existing six-channel IMU Gaussian/quantized behavior is already implemented and
is not credited as new work. IMU translation remains zero; all retained backend,
frame, sensor-count, noise and planar-lidar refusals remain intact.

## New acceptance

Exercise real canonical Twist forward and turn commands while receiving actual
`gz.msgs.IMU` and `gz.msgs.LaserScan` on separate keys alongside retained streams.
Bind packets to current acquisition samples and public component identities;
check complete headers, stamps, sequence, covariance and frames, valid finite or
positive-infinite lidar ranges, independent rates, pause/no-demand, loaded reset
with seed replay, reconnect/removal/weak leases and atomic topic-collision/reset
refusal. New public controls cover initial topic-collision refusal and successful
loaded reset; the unchanged private typed-family regression retains the atomic
reset-collision refusal proof. Do not add a private mutation hook solely for this
example. This does not claim cross-engine RNG or robot-trajectory equality.

A new real Primitive socket case must retain the old complete catalog plus the
target, with actual scan data/pose under publication credit, Ready, reset and
reconnect. Add no IMU widget, telemetry facade or edit permission.
A small canonical C++ Zenoh peer must consume both actual streams in the combined
Rust-world workflow using the unchanged qualified SDK. Preserve native model
proof identities through exact source reuse; this is new interoperability, not a
new whole-Gazebo trajectory oracle or a replay of old NoiseTest acceptance.

Complete implementation, meaningful regression source and documentation, batch
findings and fixes, and obtain independent complete source review before ONE
consolidated full normal hook, strict workspace/per-crate/actual-parent coverage,
exact review CI and independent local/official DATA with durable originals.
Require fresh peer/socket proof before qualified Main acceptance. M33 does not
refresh the installed package or rerun accepted browser qualification; M32 stays
Latest. Any real production defect or additional shared edit returns to Root for
bounded scope and ownership before editing. No test or lint runs per edit.

## Exclusive source leases and parallel work

Messages owns the new example, new
`crates/gz-sim-server/tests/robot_sensors.rs`, and new
`crates/gz-sim-server/tests/native_robot_sensors.cc`.
Light owns new `crates/gz-sim-cli/tests/private/web_robot_sensors.rs` and
`docs/M33_ROBOT_SENSORS.md`. These lanes may author and review in parallel.
Root alone owns the private-module attachment in
`crates/gz-sim-cli/tests/private/web_lifecycle.rs`, maintained
`scripts/zenoh_peer.py` registration, `scripts/native-peer.py` explicit-profile
dispatch, `scripts/verify.py` required-peer sets,
`tests/test_zenoh_peer.py` CLI dispatch/refusal regressions and
`tests/test_selected_native_gate.py` retention controls,
shared metadata, integration, serial heavy execution, publication and tracker.
SDF independently reviews both lanes and complete integration; authors may
cross-review disjoint files without changing another owner's files.

Reuse current models, Scene, Server, IO and renderer. Exclude sensor multiplicity,
new backends/dependencies/schema/transport/runtime or packaging frameworks,
dynamic plugins, body teleportation, sensor widgets, native-engine distribution,
renderer rebuild and performance/legal/cold-start claims. Keep all frozen trees,
original failures and Fuel work unchanged.

The first full normal hook failed after 1,492.264 seconds because the maintained
peer CLI's fixed choices rejected `robot_sensors` before any C++ compilation.
The other three new Rust/socket controls passed in the headless profile;
renderer and final coverage qualification were not reached. Preserve that
failed staged tree and packet. Batch the CLI registration, explicit Zenoh-only
profile refusal and meaningful real-dispatch regression, then independently
review the full correction before one required consolidated retry. No pass or
acceptance follows from the failed attempt.
