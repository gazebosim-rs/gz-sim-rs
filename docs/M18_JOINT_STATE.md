<!-- Copyright (C) 2019, 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M18: bounded joint-state publication

This increment activates the built-in system configured by model plugin
`gz::sim::systems::JointStatePublisher`, filename
`gz-sim-joint-state-publisher-system`. The contract follows retained Gazebo Sim
`71c60db3`, `src/systems/joint_state_publisher/JointStatePublisher.{cc,hh}` and its
integration tests (OSRF, Apache-2.0). Source implementation is separate from
qualification and qualified Main acceptance.

One top-level dynamic Rapier model with already admitted single-axis revolute
joints may own the publisher. It adds no solver, joint, force application or
controller. `tests/fixtures/m18_joint_state.sdf` supplies two existing revolutes.
Regressions retain supported world-parent effort and use the existing bounded
velocity motor for real moving-parent joint motion; generalized effort on that
moving-parent Rapier joint remains an explicit, atomic refusal. Other engines, nested
models, multiaxis joints, unknown requested joints and unsupported plugin fields
are refused before the new scene is returned. Parent/child names are direct
local link names or `world`, without alias or nested-model expansion.

The only fields are optional `topic`, repeated `joint_name`, and optional
`update_rate`. Without a filter all admitted model joints are selected. Repeated
known names are deduplicated; public joint IDs determine ascending output order,
matching native `std::set<Entity>`, rather than filter or lexical order. The
bounded selection contains at most 128 joints. Configuration XML is at most
64 KiB. Scalar fields cannot contain nested markup, namespaces, attributes or
CDATA. A nonempty valid explicit absolute topic is used; otherwise an omitted
`topic` means `/world/<world>/model/<model>/joint_state`. An invalid explicit topic
is refused rather than silently redirected. Only one publisher model is admitted.

The canonical packet is `gz.msgs.Model`, with model name/ID and current model
pose, header timestamp, and selected joint name/ID, parent/child names, construction
joint pose in its child frame, joint-frame axis direction, lower/upper limits,
damping, measured position and measured velocity. The retained parser's
`Sdf::joint_pose(name, "")` follows native joint semantic-pose resolution into
the child frame. This is not the moving world/model-relative joint pose.
An analytic nonidentity child-pose / model-expressed-axis regression stays inside
the existing revolute admission and distinguishes those frames numerically.
The current model pose comes from the model's local pose, as native `Pose` does.
Unrestricted native lower/upper limits are negative/positive infinity, not invented
finite limits. Measured position/velocity and poses must be finite. Force, effort
limits, velocity limits, acceleration, axis2 and unrelated Model fields are not
populated. The backend does not expose a measured joint force; no zero measurement
or complete native packet parity is claimed. The model header has a timestamp,
not a fabricated sensor sequence or frame ID.

Rate zero publishes on every simulation tick, including paused equal-time ticks.
A positive finite rate gives an integral nanosecond period truncated from
`1/rate`, matching native `duration_cast`; the reciprocal must fit the bounded
signed `i64` nanosecond domain of the pinned Linux native steady clock. Negative/nonfinite rates are refused; periods below one
nanosecond become zero. Native last-publication time starts at zero, so 100 Hz
first publishes on the advancing 10 ms tick, not the 1 ms tick. The exact guard
suppresses only `0 < time-last < period`: equal times and rewind are eligible.
This is deliberately different from sensor pause suppression. Sampling does not
depend on external subscriber interest, matching native's lack of a
`HasConnections` publication guard. A valid Server output lease owns publication;
a new internal `(epoch, revision)` distinguishes equal-time packets. Revision is
checked for overflow and is not a wire sequence number.

The public Scene components are `JointStatePublication` and `JointStateSample`,
with `JointStateJointPublication` and `JointStateValue` entries. Ownership and
physical sample bindings are checked before publication; invalid state faults the
simulation and leaves no new packet. An absent family owns no callback or marker.
Last-model removal retires its scheduling, while loaded reset reconstructs owned
identities/configuration. Direct core reset clears the public sample and retains
live publisher scheduling/revision. Loaded reset reconstructs the initial
publisher (last time zero, revision zero) and backend state. Both allow the next
zero-time paused tick; `epoch` keeps pre-reset packets distinguishable. Neither
reset path emits a packet inside the reset phase itself.

`native_joint_state.cc` is a finite same-backend component/packet observer. Modes
`default`, `filter`, `rate` consume the exact fixture on stdin and a live-maps path.
One paused-zero discovery tick and a bounded wait for its actual canonical
callback (timestamp zero and matching model metadata) precede the measured
100 advancing plus two equal-time paused ticks. Each paused tick uses native
`Server::RunOnce(true)`: ordinary `Run` counts advancing iterations and does not
bound a paused run. The two final paused ticks are separate `RunOnce` calls.
Every component observation must match its advancing or paused position in that
sequence; the two paused packets bind their own paused observations, not the last
advancing observation.
The observer requests native position/velocity components for both observed
joints, including a joint excluded by the publication filter. Physics fills
those arrays from actual joint state; the observer still requires complete
single-axis values and does not alter the publisher's filtered joint set.
It verifies default/custom topic, filter dedup/order, every populated native
metadata/value field, timestamp cadence
and nonzero real joint motion, while the simulator/plugins still own their maps.
The native observer may observe native force data, but Rust does not claim it.
This evidence is not a cross-engine trajectory comparison, performance result,
browser widget or general system-migration completion. New source must pass the
full normal and hosted gates, actual native/canonical publication proof and
independent review before acceptance.
