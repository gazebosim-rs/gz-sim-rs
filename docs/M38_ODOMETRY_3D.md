<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Noiseless 3D ground-truth odometry

M38 adds explicit `<dimensions>3</dimensions>` to the existing built-in
OdometryPublisher on its admitted top-level, flat Rapier model. Omitted dimensions
and explicit `2` retain the accepted planar branch. This is pose-derived ground
truth with a ten-acquisition velocity filter, not encoder integration or the
backend's instantaneous Velocity component. The example is
[examples/m38_odometry_3d.sdf](../examples/m38_odometry_3d.sdf).

The Source batch is not yet qualified. Fresh scoped original-system reference
and independent native DATA have passed. Actual numeric fixture replay,
three-channel C++ interoperability, full normal, strict coverage, exact hosted CI
and independent local/official DATA remain required. M1–M37
are accepted at Main `6d2d5320`; M37 remains the installed preview. M38 does not
refresh the package, launcher or viewer.

## Scene and messages

The example has one free `default::rig::body`: a 0.2 m box, mass 2 kg,
diagonal inertia 1 kg m², zero center-of-mass offset and zero world gravity. The
model starts at world XYZ `(2,-1,3)` with roll/pitch/yaw `(0.2,0.3,0.6)` radians;
the fixed step is 1 ms. The existing world ApplyLinkWrench plugin accepts its
unchanged one-shot, persistent and clear topics. Publication is 50 Hz:

| Key | Canonical message |
| --- | --- |
| `/m38/odometry` | `gz.msgs.Odometry` |
| `/m38/odometry_covariance` | `gz.msgs.OdometryWithCovariance` |
| `/m38/transform` | `gz.msgs.Pose_V` |

The frame is `m38/odom`, and the child frame is `m38/base`. Stamp and existing
header ordering, all three components, full pose orientation and complete zero
covariance matrices use the existing packet schema. No dimension or epoch field
is added to the wire. The local sample's epoch/revision remains separate from
transport sequence and the canonical stamp.

`GroundTruthOdometryDimensions::{Two,Three}` is owned publication metadata.
Changing a live publication's mode is an identity fault that removes the stale
sample; it is not an instruction to reuse the old filter under a new mode.
A healthy loaded reset reconstructs the original mode, clears history and
samples, retains public IDs and starts a fresh local epoch. A latched fault is
terminal under the existing core contract: loaded reset returns
`SceneResetError::Clock(StepError::Faulted)` before observation or scene mutation;
it does not recover a deliberately corrupted live mode. A model with plugins
is reconstructed through healthy loaded reset; template cloning is not newly
admitted.

## Arithmetic and timing

The pinned original System is
`gz-sim/src/systems/odometry_publisher/OdometryPublisher.cc` at gz-sim commit
`71c60db31fddd5c98d2aa689e9a2cccb3548a0a3`, file SHA-256
`0c4ccd7cff05a18bbe0c88b9a3769bb2c0d3243ee6677dfba085c5dde0500f77`.
Math quaternion, pose, vector and RollingMean sources are traced at commit
`1316ed334f4d5bda7be59aae07cd6e8ff31cfe13` and the qualified Math10 headers.

An eligible WORLD input executes native zero global-offset inverse, left pose
composition and right zero local-offset composition. These scalar operations are
retained even for identity offsets, because IEEE signed-zero behavior matters.
Native reference inputs are MODEL-local under an identity parent; replay converts
that MODEL pose to WORLD exactly once before feeding the same estimator. Live
WorldPose already represents WORLD and receives no extra parent conversion.

For dimensions 3, displacement is current minus previous composed world XYZ.
The current quaternion reverse-rotates that displacement using
`inverse(q) * (vector_quaternion * q)`, then each resulting component is divided
by elapsed seconds. There is no planar yaw/f32 sine/cosine projection in this
branch. Angular velocity uses the shortest quaternion sign selection, native
w-first Dot order and `current * inverse(previous)`. Native AxisAngle compares
squared XYZ length with `1e-12`; its small-angle branch is angle +0 and axis
`(1,0,0)`. Scaling by angle/dt occurs before reverse rotation with the ORIGINAL
current composed quaternion; sign selection affects rotation difference only.
There is no quaternion normalization, acos clamp, generic zero canonicalization
or visibility epsilon added to this calculation. Invalid finite/domain arithmetic
is a typed fault rather than a successful reading.

Six independent histories hold body linear XYZ and angular XYZ. Each retains the
last ten eligible acquisitions and recomputes its sum oldest to newest before
dividing by count. The native zero-noise addition of +0 is retained for the
new spatial outputs; it does not draw random values. Initial history starts from identity pose, matching the native
startup transient. The first unpaused call, including time 0/dt 0, records time
and returns without adopting the current pose as its history baseline. Later
elapsed intervals at or below 1 microsecond are suppressed without changing
history. Integer time and checked duration bounds retain the existing contract.

Every eligible acquisition updates history before publication throttling; a
50 Hz sample therefore reflects intervening 1 ms history. Pause skips the
producer and preserves its held reading and time. Scene acquisition is local
and does not depend on subscriber demand. Server demand controls queued
publication on the existing streams without rewriting the producer's history;
absence, removal, endpoint retirement and shutdown retain their separate bounds.

The default/explicit dimensions 2 function retains its original f32 yaw
trigonometry, three means, zero published Z and roll/pitch rates. All accepted
M34 fixture inputs and expected bits/wire bytes remain unchanged. The old
`dimensions=3` refusal is intentionally replaced by positive 3D controls;
invalid 0/1/4/-1/text, duplicate/scalar-smuggled metadata, nonzero noise/offsets,
unsupported backend/topology and old topic/frame bounds remain refusals.

## Source controls and required evidence

The new public Scene controls use real Rapier force and torque on all three
axes, with independent mass/inertia velocity checks at the retained `1e-12`
physical integration bound. They compare equally driven throttled and
unthrottled sample histories, held pause state, loaded reset identity/initial
sample, stale mode fault, pending-command removal and absence. Their comparison
of actual Velocity is distinct from the filtered Twist. Private controls cover
zero-time startup, the inclusive microsecond boundary, six filter windows,
current-body/sign/near-zero angular branches and typed invalid math.

Fresh reference acquisition is a finite original-system DSO/ECM pose-history
oracle, not native Physics trajectory evidence. The agreed native helper case
plan completed 15 modes with 510 inputs, 429 observations and 1,287 canonical
packets in 54.958 seconds with actual Root exit 0. Independent native DATA
`42b68ad50f20197dc1066d60b86ee61af52e05ba3a1e077177feb1e371b5817c`
passed using reader `149ca999`, with a separate actual Root exit 0. The capture
tree `f5759834015f7b0e48aea7efb435d3840b675661` contains only the helper over
the accepted parent; it is not the final Rust candidate. The retained extractor
returned 0 and generated only actual literals. Independent literal DATA
`747702765ff6f7ffce9034cae0d241b0f0c69ca559c341e0650eb82e763a8629`
then passed with actual Root exit 0 using reader `45694fd7`, comparing all 510
input rows, 429 observed numeric tuples and 1,287 packet byte strings. Root
copied the exact fixture and inserted both independently reviewed replay controls
in the central candidate; Rust compilation and replay execution remain pending.
No placeholder fixture or guessed numeric constant supplies credit. The original 301-input/221-observation/
663-packet 2D fixture and its controls continue to run.

The separate Server/C++ lane must consume all three actual outputs for real
force/torque motion, match complete pose/twist/header/covariance to Scene samples,
and prove demand/reconnect/reset/removal/shutdown with existing bounded queues.
A same-session clock/FIFO fence is used only at its reviewed owned queue
boundary; it is not a general network cross-publisher barrier. Reset drains old
messages before phase-local comparisons because canonical messages carry no
wire epoch. No new dependency, backend, transport, public math hook, renderer,
noise, offset or generalized publisher framework belongs to this slice.

## Reproducing the retained reference

[The compact reference provenance](../crates/gz-sim-scene/tests/fixtures/m38_native_odometry_3d.provenance.json)
records the actual complete helper compile argv and working directory, each of
15 original-system run argv/working directories, compiler identity, original
plugin/object and five critical mapped Sim/Math/Msgs/Transport/Zenoh provider
hashes. It also binds the native result, independent DATA, extractor and actual
generated fixture hash. The tracked C++ generating program is
`crates/gz-sim-server/tests/native_odometry_3d.cc`, built with its explicit
`GZ_SIM_NATIVE_ODOMETRY_3D_ORACLE` role. Its default role is the separate real
canonical peer; neither role is a second production implementation.

To reproduce, use those exact compiler flags, original pinned headers/libraries
and case commands, with a new exclusive output directory substituted for the
historical packet paths. Verify the recorded compiler, plugin and provider
hashes before attributing results to this reference. The retained historical
absolute paths identify the actual generating command and are not runtime
requirements of the installed preview. The compact file deliberately omits the
349-node SDK inventory; complete retained capture DATA binds it separately.
No new SDK mutation, dependency build or broad native campaign is implied.
