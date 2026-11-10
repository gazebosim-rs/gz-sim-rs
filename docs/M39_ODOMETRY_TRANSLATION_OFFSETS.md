<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Ground-truth odometry translation offsets

M39 admits finite local/body `xyz_offset` and global/reference
`xyz_global_offset` translations for the existing noiseless OdometryPublisher,
in both dimensions 2 and 3. Rotational offsets, noise, backend, topology,
message layouts and transport remain unchanged. The Source batch is unqualified;
M38 is now accepted at exact Main `00382fbc`, including hosted CI, independent
DATA and normal Main readback. M39's scoped native acquisition and independent
native DATA passed; Rust and distribution qualification remain pending and require
the complete independently reviewed M39/M40 candidate. M37 remains installed Latest.

[The example](../examples/m39_odometry_translation_offsets.sdf) retains M38's
one free `default::rig::body`: a centered 0.2 m box, mass 2 kg, diagonal inertia
1 kg m², zero gravity, 1 ms step and model XYZ `(2,-1,3)` with roll/pitch/yaw
`(0.2,0.3,0.6)` radians. It selects dimensions 3 at 50 Hz, local translation
`(0.4,-0.3,0.2)` and global translation `(-0.7,0.5,-0.6)`. These translate the
odometry point/reference, not the model or physics body. Without commands, the
finite installed load/step example still reports the raw body at `(2,-1,3)`.

The three canonical topics are `/m39/odometry`, `/m39/odometry_covariance` and
`/m39/transform`; frame `m39/odom` and child frame `m39/base` keep their existing
header order. The existing ApplyLinkWrench world plugin supplies one-shot,
persistent and clear inputs on its default topics. Filtered pose-derived Twist
is distinct from instantaneous backend Velocity and encoder integration.
No dimension, offset or epoch field is added to the wire. Complete covariance
matrices remain zero, and all old finite/unit-quaternion/time/revision guards
remain in force.

## Configuration and identity

`GroundTruthOdometryPublication.offsets` stores a focused immutable
`GroundTruthOdometryOffsets`. Its local/global `[u64;3]` representations are
private. The checked `from_translations(local, global)` constructor rejects
nonfinite components with `GroundTruthOdometryError::Offset`; getters return
f64 triples. XML still rejects malformed/nonfinite configuration through the
existing loader error, before entity/physics/publication mutation.

For EACH vector independently, all components comparing equal to zero produce
the default positive-zero bits. This preserves the old parser's treatment of
omitted and explicit all-zero vectors, including signed-zero spellings. When a
vector has any nonzero component, EVERY component bit is preserved, including
negative zero in its remaining components. Exact bit equality participates in
Scene ownership and Server prepared/stream/reset identity; approximate float
equality cannot silently reuse a changed nonzero configuration. Changing a
live offset, even just the sign of a zero in a nonzero vector, faults and removes
stale samples instead of reusing the previous history.

Healthy loaded reset reconstructs the original offsets, mode, history, samples
and public IDs. Internal ECS entity handles may change; reacquire them from the
loaded world before reading the restored public identity and metadata. After a latched fault, reset refuses with `Clock(Faulted)` before
observation or mutation; it is not a recovery operation. Removal retires the
owner and discards pending work without retargeting. The plugin-bearing model
reconstructs through loaded reset, not newly admitted template cloning.

## Native arithmetic

Pinned original `OdometryPublisher.cc` at Sim `71c60db` has SHA-256
`0c4ccd7cff05a18bbe0c88b9a3769bb2c0d3243ee6677dfba085c5dde0500f77`.
Configure assigns XYZ to two Pose3 positions. Update computes, left-associated,
`globalOffset.Inverse() * WORLD * localOffset`. Both offset rotations stay the
native zero-constructor quaternion. Local translation follows current model
orientation; global translation changes the world reference origin.

The implementation retains the pinned Math Pose3/Quaternion scalar order.
Global inverse computes quaternion inverse and negated position, then its
optimized vector rotation: first cross `uv`, second cross `uuv`, scale `uv`
by `(2*w)`, scale `uuv` by `2`, and evaluate `(vector + uv) + uuv`.
Each subsequent Pose3 product instead evaluates quaternion multiplication
`left.q * (vector_quaternion * inverse(left.q))`, left-position addition,
and quaternion product. Do not replace this with a subtract/rotate shortcut:
identity operations can change signed-zero bits. Missing/all-zero offsets take
the accepted zero helpers and original scalar path exactly.

Finite configuration is not a promise that every finite composition fits f64.
Either nonfinite intermediate product faults with `Arithmetic` before filter
history, timestamp, revision or sample publication changes. There is no clamp,
new floating tolerance or successful nonfinite packet. Invalid input pose and
clock still follow their existing validation and first-call/microsecond order.

Live inputs are already WORLD poses. The native reference records MODEL-local
poses under an identity parent; scalar fixture replay applies that native
identity-parent composition exactly once, before the production WORLD seam.
Requested and effective native offset vectors must both be retained: each
wholly-zero requested vector is normalized before configuring the original DSO.
That witnesses the selected legacy normalization, not equivalence with an
unnormalized native negative-zero XML vector.

The first unpaused call, including time/dt zero, records time without adopting
the current pose as the history baseline. Later elapsed intervals at or below
1 microsecond do not acquire. Every eligible composed pose updates the existing
ten-value, oldest-to-newest histories before publication throttling. Two keeps
its original planar f32 yaw arithmetic, three means and published Z/roll/pitch
zeros; Three keeps its full quaternion body-frame XYZ and six histories.
Pause holds local samples/history. Scene history does not depend on subscriber
demand; Server demand selects the existing bounded queued publications.

## Meaningful controls and remaining proof

Three exact old refusal inputs become stronger positives: omitted/default Two
with local `(0,0,1)`, explicit Two with local `(0,1,0)`, and explicit Three with
local `(0,0,1)`. EACH now loads its original input, receives real force/torque,
checks rotated point geometry and healthy reset with exact metadata/history.
The omitted case is not discharged by an explicit Three test. Old negative
slots now reject nonfinite vectors; wrong length, invalid text, duplicates,
scalar smuggling, noise, nonzero rotation and backend refusals remain strict.

Additional controls cover local/global/combined offsets in both modes,
independent zero-vector normalization and mixed signed bits, equal physical
body motion with different point feedback, history-before-throttle, pause,
bit-only stale configuration, terminal reset refusal, removal/reconstruction
and finite composition overflow before history. Public physical integration
and independent body-point geometry use the existing `1e-12` bounds; they do
not replace exact native-bit and canonical-byte comparisons.

The fresh original-system acquisition returned actual Root exit 0 in 106.975
seconds: 34 modes, 1,152 inputs, 986 triples and 2,958 canonical packets across
36 configurations, including 166 suppressed inputs. Independent native DATA
`ecd05da4eeffb8e52066b43bf621f6c97953f587847fd493ea52ac2ce9eb013f`
passed with a separate actual Root exit 0 using reader `581984a9`. Its helper-only
capture tree `4592f063a11d20deaa7d2cb9f65b1a3ba5b54f05` is distinct from the
final integrated Rust candidate. The unchanged original DSO and exact providers
were reused, compiling only the new dual-role helper. No native Physics or old
native campaign was executed. Actual literal extraction returned 0 and produced
fixture SHA-256 `1f9223e9fae7472152112c7e016412ebbfaa4c1f188ad4da51795b8d66e7887a`
(3,389,979 bytes). Independent literal DATA
`effbc94f92e6c9ca4563fe91ecb4918f23706d4c71a08d6d18c91bd1d494915d`
then passed with actual Root exit 0 using reader `bb5ec0d4`. The exact fixture
and reviewed Scene replay control are integrated. The first partial headless
run passed fifteen new controls, including all 986 numeric and 2,958 canonical
comparisons. One new healthy-reset test failed because it expected a stable
internal ECS handle; its correction reacquires the model and retains every
public-ID, metadata, empty-sample and exact history assertion. The failure
occurred before that test's later loop iterations, which receive no credit.
The complete normal hook, headless coverage and renderer qualification remain
pending; no guessed numeric value or partial pass supplies acceptance.
The accepted M34 301/221/663 and M38 510/429/1,287 fixtures and controls remain
unchanged. The real three-stream C++ lane separately requires actual motion,
headers/poses/twists/covariance, demand/pause/reset/collision/removal and owned
shutdown, within the existing clock/FIFO boundary; no general network barrier
or wire epoch is invented.

Full Source review, all new native/literal proof, the consolidated normal hook,
unchanged strict workspace/per-crate/actual-parent coverage, exact hosted CI and
independent local/official DATA remain required. M40 also needs fresh release,
notices/ELF/package/relocation/install/browser/shutdown and publication readbacks
before either phase is accepted. Finite installed commands provide load/step
credit only; this guide claims no Rust or distribution qualification,
performance, cold-start or broader Gazebo compatibility.

## Reproducing the reference

[The compact generating provenance](../crates/gz-sim-scene/tests/fixtures/m39_native_odometry_translation_offsets.provenance.json)
records the actual full compile argv/working directory, all 34 run commands,
compiler identity, reused DSO/object, five critical mapped Sim/Math/Msgs/
Transport/Zenoh provider hashes, actual result/audit/extraction/fixture bindings
and requested/effective-vector policy. The generating program is the tracked
`crates/gz-sim-server/tests/native_odometry_translation_offsets.cc`, compiled
with explicit `GZ_SIM_NATIVE_ODOMETRY_TRANSLATION_ORACLE`; its default role is
the separate canonical C++ peer. Exact historical paths describe the actual
commands, not installed dependencies. Reproduction requires matching providers
and a fresh exclusive output directory; no old file or capture is overwritten.
The complete 349-node SDK inventory remains bound by native DATA without
copying it into this compact file. Reference generation is component proof,
not a new SDK build, native Physics campaign or production implementation.
