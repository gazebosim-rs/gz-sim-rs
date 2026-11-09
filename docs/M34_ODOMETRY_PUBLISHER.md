<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M34 noiseless 2D ground-truth odometry

This is the bounded Source implementation under [M34/M35](M34_M35_DELIVERY.md),
over accepted M33 Main `17dd62f27234eab1482abb233389d167dafa31b4`. The bounded
implementation and reference helper passed independent Source review. Actual
native captures and literal-fixture DATA are now available. The fifth normal
attempt passed the new exact Rust numeric and canonical-byte replay controls,
but its whole headless hook failed. Full normal and exact hosted qualification
remain required. M34 is not accepted.
M32 remains the installed Latest. This feature does not change DiffDrive's
wheel-encoder odometry, the viewer, physics, dependencies or transport schemas.

## Actual reference evidence and Rust replay checkpoint

The first combined native batch exited 1 at the separate wrench helper's missing
`sdf/SDF.hh` include. Both original plugins and the odometry helper compiled;
all nine odometry mode children exited 0 beforehand. Their retained observations
contain 299 unpaused inputs, two paused inputs, 221 three-channel acquisitions
and 663 canonical packets. The failed wrapper remains failed. A first retained
reader also exited 1 after completing its body and saving output, because its
final self-hash reserialized mutable resource minima. A distinct metadata-only
finalization audit (`7623b056`, report `abece406`) actually exited 0 and preserves
both failures; it does not claim a fresh SDK, source or raw-body replay.

The separately reviewed wrench header correction and affected-only continuation
compiled that helper and ran eight wrench modes with an actual zero exit. It
reused the original plugin builds and did not repeat any odometry mode.
That separate wrench proof does not establish Rust odometry equivalence.

Literal generation then exited 0. Independent fixture-versus-raw audit
`1cf02da1` actually exited 0 (report `0c7e5ef2`), checking all input/output bits
and 663 packet byte arrays. The 672,265-byte unformatted artifact has SHA-256
`f506881a201153edd5204dcc98fd2b768192850f4763ea22874e30a1dac75f78`.
Its 301 tuples distinguish 221 observed outputs, 15 delayed-subscription
unobserved outputs and 65 suppressed calls; negative-zero bits are retained.
The final candidate includes this typed data in
`crates/gz-sim-scene/tests/fixtures/m34_native_odometry.rs` for private Scene
estimator and Server serializer tests. The fifth normal attempt passed all 301
dispatch inputs and 221 exact native numeric comparisons, all 663 exact packet
byte comparisons, and the separate direct-world signed-zero regression. No
epsilon or ULP allowance, cross-engine trajectory or performance claim follows.

## Admitted configuration

Recognize only the model plugin name `gz::sim::systems::OdometryPublisher` and
filename `gz-sim-odometry-publisher-system`. Admit at most one configured model
per world, a top-level flat model in the existing Rapier loader domain; reject
nested models, unsupported frame/topology metadata and other engines before
physics/ECS/endpoint mutation. Existing supported direct links/joints and static
models need no new physics behavior: sample the actual model `WorldPose`, not a
chosen link, wheel angle, rendered transform or cached initial pose.

The closed plugin fields are `dimensions` (omitted or exactly 2),
`odom_publish_frequency` (finite, default 50), `odom_frame`, `robot_base_frame`,
`odom_topic`, `odom_covariance_topic`, `tf_topic`, and the four optional
`xyz_offset`, `rpy_offset`, `xyz_global_offset`, `rpy_global_offset` vectors plus
`gaussian_noise`. All offsets must be finite exact zero and noise finite exact
zero. Reject 3D/nonzero values, duplicates, unknown fields, attributes/namespaces,
nested scalar content, scalar comments/PIs and CDATA; permit ordinary root-level
comments and surrounding scalar whitespace. Plugin XML is bounded to 64 KiB.
Use existing strict absolute-topic validation and 65,535-byte frame/topic limits,
nonempty frames without whitespace/control characters; no silent sanitization.

Default frames are `{model}/odom` and `{model}/base_footprint`; default topics are
`/model/{model}/odometry`, `/model/{model}/odometry_with_covariance` and
`/model/{model}/pose`. Raw topics must be pairwise distinct; their final remapped
keys must also be distinct from each other and every existing endpoint, including
PosePublisher and DiffDrive. Initial creation and loaded-reset preparation must
validate all three keys before any declaration, state or epoch change.

Finite frequency <= 0 means period zero (every eligible acquisition), matching
native. For positive frequency compute `(1.0 / hz) * 1e9`, truncate to integer
nanoseconds as native duration_cast does; reject nonfinite/unrepresentable signed
nanoseconds. Sub-nanosecond period zero is allowed. This is not SensorSchedule:
publication throttling and odometry history have different clocks.

## Exact native computation and timing

The authority is original `OdometryPublisher.cc`, not its broader header claims.
PostUpdate returns immediately while paused. The first **unpaused** call records
only its timestamp and sets initialized; it does **not** capture pose, publish,
fill a filter or update last-publication time. This includes an unpaused call at
time zero with dt zero. Previous pose remains identity.
Thus a stationary model initially at nonzero x/y/yaw has a real first-estimate
transient relative to identity; do not silently subtract the initial pose.

Subsequent calls compute dt from the last accepted history timestamp. Native
`math::equal(0.0, dt_seconds)` is absolute difference <= 1e-6: positive intervals
of at most 1 microsecond return without changing pose, time, filters or sample.
Later calls accumulate elapsed time from that retained timestamp. After
initialization, zero-duration calls likewise do nothing. Checked integer time
subtraction rejects backward or
unrepresentable time in Rust; native negative-time rewind is not supported.

For each accepted history call, even when publication is not due:

1. Read finite model WorldPose with a valid unit quaternion. Zero offsets still
   follow the native identity pose-composition semantics; do not recenter it.
   The production input is already world-relative. Evaluate
   `globalOffset.Inverse() * worldPose * offset` using the pinned scalar
   Pose3/quaternion operation order even when both offsets are zero. Pose3
   multiplication uses its quaternion sandwich for translation; inversion uses
   the distinct optimized vector cross/cross/add rotation. Use the composed
   pose for displacement, angular velocity, retained history and publication.
   These operations can change signed-zero bits and cannot be replaced by a
   pose copy or zero canonicalization.
2. Compute world x/y displacement from previous pose. Compute native Yaw from
   the normalized quaternion using Quaternion::Euler, including its 1e-15 pitch
   singularity branches. Convert yaw to f32 **before** `cos`/`sin`, then promote
   their f32 results to f64. Native 2D linear velocities are
   `(cosf(yaw)*dx + sinf(yaw)*dy)/dt` and
   `(cosf(yaw)*dy - sinf(yaw)*dx)/dt`.
3. Angular velocity uses the native quaternion-difference algorithm, not wrapped
   Euler subtraction: if current dot previous < 0 negate current, calculate
   `current * previous.Inverse()`, extract AxisAngle, divide angle by dt and
   reverse-rotate into the current body frame; use its z component. AxisAngle
   treats vector squared length <= 1e-12 as zero rotation; otherwise its angle
   is `2*acos(w)`. Native inverse, scalar operation order and reverse rotation
   are the reference. Preserve q/-q continuity and yaw crossing +/-pi. Reuse glam
   types, but any needed scalar compatibility expressions stay private in this
   leaf; no general quaternion or filter framework. Do not add an epsilon,
   normalize away a behavior difference or clamp a nonfinite result into a pass.
4. Push body linear x/y and angular z into three bounded ten-element histories.
   Mean is oldest-to-newest f64 summation starting at 0, divided by actual count;
   initial windows divide by 1..9, not ten. Evict before summation at count 11.
   Do not use a running-sum recurrence, whose rounding differs from native.
5. Update previous pose and history timestamp, then throttle publication. Native
   skips only when `0 < time-last_publication < period`. At or beyond period it
   publishes the current estimate and sets last-publication time to current time;
   no catch-up frames or idealized rate-grid timestamps. All three outputs use
   this one acquisition, timestamp and revision.

Dimensions=2 does not flatten orientation: native keeps the full current pose
quaternion, forces published position z to zero, and publishes only linear x/y
and angular z (remaining twist coordinates zero). Nonzero model height is valid.
The full native orientation/body-z projection remains the contract for tilted
valid poses; no undocumented planar-pose admission restriction is proposed.
Finite/unit-pose, dt, filter-sum, result and revision-overflow failures are typed
schedule faults with owner identity and diagnostic. Invalid current results
must not leave a local sample that can be republished as current. These safety
refusals are explicit Rust policy, not a claim that native always rejects them.

## Canonical outputs, demand and lifecycle

One sample encodes `gz.msgs.Odometry`, `gz.msgs.OdometryWithCovariance` and one
`gz.msgs.Pose_V`. Native Odometry header contains the acquisition stamp and exactly
`frame_id`/`child_frame_id` data entries. Covariance message has that same outer
header, the same pose/twist and both 36-entry covariance arrays all zero. Pose_V
has one pose containing the same stamp/frame header and pose, with default empty
name/id zero; its outer header remains absent. Preserve native nested-field
presence as observed from the focused helper; no fabricated sequence field.
Local epoch/revision/owner identities are bookkeeping, not additional wire data.

Native does not gate history or publication on HasConnections. While configured,
advance history independent of channel subscribers; late subscription must not
restart the initial transient or empty the filters. Scene can retain the latest
due sample. IO publishes each new (epoch,revision) at most once per channel;
paused/non-due updates and reconnect cannot replay an old sample as newly acquired.
Test no-demand history and independent real subscribers for all three channels.
No configured publisher means no system callback, resource, per-tick polling,
worker or endpoint. Last-owner removal retires schedules and weak output leases;
reconstruction happens outside stepping, preserving the existing absence policy.

Loaded reset is an explicit deterministic Rust policy: atomically prepare all
three endpoint reservations first; rebuild original public model identity and
metadata, remove old sample, clear histories/previous pose back to identity,
reset both clocks/revision and publish nothing at reset time. First unpaused
PostStep call initializes time again (including an explicit unpaused zero-time
call), and the same initial transient reappears. Paused
reset leaves epoch time zero until resumed. Native PreUpdate has a rewind TODO;
there is no native reset-parity claim. Removed/live peer endpoint collisions must
refuse reset atomically without consuming epoch, queues, leases or samples.

## Typed integration (Root-owned shared attachments)

Keep all implementation in the eight leased files; no additional dependency,
shared-noise or pure sensor-model module is needed. Root owns every attachment.

Scene public values:

- `GroundTruthOdometryPublication { model_name, odom_frame, robot_base_frame: String,
  odom_topic, covariance_topic, tf_topic: String }`, a component;
- `GroundTruthOdometrySample { time: Duration, epoch: u64, revision: u64,
  pose: WorldPose, linear: [f64; 3], angular: [f64; 3] }`, a component whose pose
  already has the wire's z=0; and typed `GroundTruthOdometryScheduleFault` carrying owner
  public ID and `GroundTruthOdometryError::{Identity,Pose,Time,Arithmetic,Revision}`.

Scene private hooks follow existing PosePublisher:
`is_plugin(&sdformat::Plugin)->bool`, `parse(&Model)->Result<Option<Plan>,LoadError>`,
`Plan::validate(&Model,&Simulation,Option<&str>,(bool,bool))->Result<(),LoadError>`,
`build(&Sdf,&mut Simulation,&Model,Entity,&PhysicsBackend,&Plan)->Result<(),LoadError>`,
`install(&mut Simulation)`, and
`retire_if_unused(&mut Simulation)->Result<(),ScheduleError>`.
Private `Publisher` owns three fixed ten-value arrays/clocks; `Installed` is
`pub(super)` for Root lifecycle guards. Root attaches parsing/admission/build,
PostStep sampling after authoritative poses, loaded initial metadata and reset
snapshot/reconstruction, removal/reconstruction and the no-publisher fast path.
Root's public getter
`initial_ground_truth_odometry_publications(&Simulation)->Result<impl
Iterator<Item=(u64,&GroundTruthOdometryPublication)>,SceneResetError>` mirrors
the existing initial-pose getter exactly. The initial-node field is
`ground_truth_odometry_publication: Option<GroundTruthOdometryPublication>`.

Server leaf follows the existing PosePublisher stream/reset API:
`streams(&Simulation,&world_io::WorldIo,&ServerOptions,&BTreeSet<String>,
Option<&Prepared>)->Result<Vec<Stream>,Error>` and
`prepare_reset(&Server,&BTreeSet<String>)->Result<Prepared,Error>` are leaf-local
functions. `Prepared` is a `BTreeMap<String,(u64,GroundTruthOdometryPublication,
GroundTruthOdometryChannel,Arc<GroundTruthOdometryOutput>)>` keyed by final
endpoint, with all three channels validated together before declarations.
Root adds `Server::publish_ground_truth_odometry(&mut self)` and
`Server::retire_ground_truth_odometry(&mut self)`, Server storage
`ground_truth_odometries: Vec<odometry_publisher::Stream>`, initial/reset/removal
hooks, reserved-key validation, and `Error::GroundTruthOdometryIdentity`.
New counters are `ground_truth_odometry_queued`,
`ground_truth_odometry_covariance_queued`, `ground_truth_odometry_tf_queued`,
`ground_truth_odometry_failures`, `ground_truth_odometry_invalid_samples`.
Counts are queue/failure facts, not proof of downstream delivery. Existing
DiffDrive `odometries_queued`, `odometry_failures`, `invalid_odometry_samples`
and its output methods retain their current meanings.

The private IO leaf proposes
`GroundTruthOdometryChannel::{Odometry,Covariance,Transform}`,
`GroundTruthOdometryOutput`, `GroundTruthOdometryJob`,
`GroundTruthOdometryDeclarations`; one ordinary IO owner maintains three
channel-tagged weak leases/jobs. Root integrates these exact methods on the
existing `WorldIo` type, using its existing boxed-error `Result` alias:

```rust,ignore
pub(super) fn ground_truth_odometry_output(
    &self, key: String, channel: GroundTruthOdometryChannel,
) -> Result<Arc<GroundTruthOdometryOutput>>;
pub(super) fn publish_ground_truth_odometry(
    &self, output: &Arc<GroundTruthOdometryOutput>, bytes: Vec<u8>,
) -> Result<()>;
```

Root adds `WorldWork::GroundTruthOdometry` and
`WorldWork::InstallGroundTruthOdometry` using the existing epoch-tagged work and
setup-receipt patterns. Three disjoint failure-slot constants are
`GROUND_TRUTH_ODOMETRY_FAILURE_INDEX`,
`GROUND_TRUTH_ODOMETRY_COVARIANCE_FAILURE_INDEX`,
`GROUND_TRUTH_ODOMETRY_TF_FAILURE_INDEX`; Root assigns their numeric values with
M35's shared array update. The leaf's observer/bookkeeping fields use the same
`ground_truth_odometry_*` namespace. Existing `WorldIo::publish_odometry` is
DiffDrive's API and is untouched; Rust method overloading is not proposed.
Keep existing bounded queue/capacity behavior and no network wait in stepping.
Root approved these signatures and shared additions. The implementation needs
no additional generic IO framework. Shared integration is reviewed with the
complete feature batch; these Source declarations alone are not a passed check.

## Bounded reference and regression plan

`native_odometry_publisher.cc` has two explicit roles. Its default build links
only canonical Messages/transport and is a real C++ consumer of the Rust server;
it asks canonical WorldControl for 200 steps and checks all three actual streams,
their identical pose/twist/header values, zero covariance and actual model motion.
The Rust integration test separately compares those wires to actual Scene values;
no native/Rust trajectory-equality claim is made.

Only `GZ_SIM_NATIVE_ODOMETRY_ORACLE` selects the independent reference role.
It loads the original packet-only plugin via native SystemLoader, queries its
actual Configure/PreUpdate/PostUpdate interfaces and supplies a minimal native
ECM world/model/Name/Pose/ParentEntity plus declared UpdateInfo/pose inputs.
This Root-approved direct-System construction witnesses unpaused time0/dt0 and
microsecond suppression exactly without a physics rebuild, modified clock,
copied odometry algorithm or new runtime framework. It is not body teleportation
in the Rust product. Source-bound original input poses, every input timestamp,
the two explicit paused calls, all three serialized canonical wire messages and
live provider maps are retained. Real subscribers must prove exact provider/type
and consumer readiness before observed phases; all three channel completions
share a bounded five-second wait. Rows are keyed by time, never callback order.

The nine declared reference modes are `identity`, `initial`, `motion`, `rate`,
`micro`, `paused`, `delayed`, `tilt`, `sign`. Their actual three-channel
acquisition counts are respectively 32, 32, 32, 1, 11, 32, 17, 32, 32 (221 total,
663 canonical packets). There are 299 unpaused input records and two additional
paused-input records. Retained native and fixture DATA bind these observations;
they do not count as Rust test or integrated qualification passes.

The helper and Rust regression Source cover these distinct cases:

- Identity stationary: paused first call leaves initialization absent; first
  unpaused call at time0/dt0 initializes silently; a repeated zero-time call
  changes nothing; eligible subsequent history update,
  all-zero pose/twist/covariance fields and three equal stamps.
- Nonzero initial x/y/z/yaw: first identity-relative transient and its exact
  ten-history eviction, full orientation and wire z=0.
- Translating/turning pose sequences: non-axis body yaw, f32 trig witness,
  nonconstant finite differences, window filling/eviction and yaw +/-pi plus
  q/-q sign continuity. Include a tilted unit-pose case for full-quaternion
  output/body-z semantics rather than silently restricting it.
- Rate: native 50 Hz versus period-zero per-step history at 1 ms; period boundary
  and no catch-up. Rust admission tests also bind finite nonpositive frequency,
  the sub-nanosecond zero period, reciprocal overflow and native operation order.
  Native micro mode supplies 500 ns increments from zero. Prediction uses the
  same `chrono::duration<double>(nanoseconds).count()` division as the original
  system, with an explicit exact-1000-ns suppression witness; multiplying integer
  nanoseconds by 1e-9 can round this boundary in the wrong direction. Accepted
  ticks are 3,6,...,33: eleven acquisitions every 1.5 us; the intervening
  zero/0.5/1 us calls leave history unchanged. The actual retained nine-mode
  capture contains those 221 acquisitions.
- Pause and delayed demand: queued subscribers absent/present do not change
  histories or estimate. Paused calls change no time/filter/sample/publication.
- Rust-only safety/lifecycle: invalid/duplicate XML and unsupported engine,
  fields/topology; nonfinite/invalid quaternion and arithmetic overflow; absent
  callbacks/resource/IO; real motion versus DiffDrive wheel odometry; three-way
  initial/remap/reset collisions, weak leases, removal, reset replay and shutdown.

Native Movement/MovementCustomTopic/OdomFrameId/OdomCustomFrameId intent maps to
pose/velocity/cadence/frame/channel controls; quaternion continuity regression
3830 maps to actual signed quaternion/wrap controls without weakening its
original 0.5 yaw-rate tolerance. Movement3d/3d-singularity, nonzero OffsetTag,
GlobalOffsetTag and GaussianNoise remain explicitly unsupported/rejected, not
passing or retired upstream tests. No native RNG-bit, 3D/full-plugin parity,
performance, package/browser, legal or cold-start claim is included.

## Exact source and reference provenance

All paths below are local, read-only inputs. Native source root is
`/home/nkoenig/work/gazebo/rust/gz-sim`, HEAD
`71c60db31fddd5c98d2aa689e9a2cccb3548a0a3` (target files clean):

| Relative source | SHA-256 |
| --- | --- |
| src/Util.cc (model component to world pose) | 6e35af3fa7c0a13d420a6cabae954177c16938d27d543d3e9f432fd980ade270 |
| src/systems/odometry_publisher/OdometryPublisher.cc | 0c4ccd7cff05a18bbe0c88b9a3769bb2c0d3243ee6677dfba085c5dde0500f77 |
| src/systems/odometry_publisher/OdometryPublisher.hh | 0630bf9e789d7c99e57d0e78f51ec582b0a0a8809800a41834427d01f339af0c |
| test/regression/3830_odometry_angular_velocity_continuity.cc | 2d1436333f36cdd08469e5975eeb6119698330f89c8813694317d1ee293bab0b |

Native math root `/home/nkoenig/work/gazebo/rust/gz-math`, HEAD
`1316ed334f4d5bda7be59aae07cd6e8ff31cfe13`:

| Relative source | SHA-256 |
| --- | --- |
| include/gz/math/Helpers.hh | 24aeb75ca8b1ee502c654f045901d39befcbd84dfce4bd7fd93c8e2ca1e439f5 |
| include/gz/math/Quaternion.hh | 9170601444da93735b4d14b369cc4266fafc7655657a6af7613e31206382b873 |
| include/gz/math/Pose3.hh | 6bfc53f0bd22a3e1596bbdb07c24bacdac9e17bd8178b9373d8df1b01e4fc6f6 |
| include/gz/math/RollingMean.hh | c37ffe6506e7b57de9bb3e9c238dca9f59ef0f4ec0be846e99751add00ecd611 |
| src/RollingMean.cc | 2075aa1b3598f6f36077a3181c7848e62d97f3f55a579b9a3c8700ea253120f6 |

Existing generated Clang recipe root is
`/home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/reference/gz-sim-clang`.
Under `src/systems/odometry_publisher/CMakeFiles/gz-sim-odometry-publisher-system.dir/`,
flags.make is `bdbd166ad52133dc8f600d8990e2d76b76d5cb4a2b653d80b7209cb7be7f9b00`
and link.txt is `921484e9211b4b4874987f6d788d683452f323483c133fc54aa9fd4b7d7c512a`.
The maintained focused helper recipe `src/CMakeFiles/UNIT_ServerConfig_TEST.dir/`
flags/link hashes are `ff0735982101a89e7296195417691346537623e8d7369d983693451c64db9aaa` /
`f95bf6385c09408c2a3ab039ed6ee1a5e7d1c50ff776901e6c006f61df049970`;
CMakeCache.txt is `126125e4c953bd16ac6177025baa833938ce22242ece11b9efd1f722fb402b68`.
One packet-only original object/DSO plus one focused helper must use these
reviewed recipes; preserve native sources/SDK and do not rebuild Sim/physics.

The saved current dependency closure is
`gz-sim-rs/artifacts/m13-m14-delivery/native-altimeter-current-dependency-closure.json`,
SHA `370f49a779cbcb32bda7a46698ef2b82b6a13764c898287e0178524e66d7a4fb`.
Its existing provider records, **not newly rehashed or executed in this review**, are:

| Path | Recorded SHA-256 |
| --- | --- |
| reference build lib/libgz-sim.so.11.0.0~pre3 | 4ff55a5ed5ee65eca4c3c7cc32ae8f31866869ce11cb95f87d645deed6f11e8a |
| /home/nkoenig/local/lib/libgz-common.so.8.0.0~pre2 | 93d9c64716d8b5efdb5862630cc33e62da97785d55e401ed19e4571c9b03df8d |
| /home/nkoenig/local/lib/libgz-math.so.10.0.0~pre2 | 288c6f52fac45359f5de67d97e0069b7f37420f5f78888e61406b8f565656c4f |
| /home/nkoenig/local/lib/libgz-msgs.so.13.0.0~pre2 | 0504be34705191623dce9654e7b659663d87cdb92a3b99c83db13a7f60f13659 |
| /home/nkoenig/local/lib/libgz-transport.so.16.0.0~pre2 | bc9b06e4337e638b4f15d2397c974d34dd99a33957a7756e2fc424f78f08309a |
| /opt/ros/jazzy/opt/zenoh_cpp_vendor/lib/libzenohc.so | de63c0072f4186cf55d325773962ea6621d391629c8b33edf276bb09062b68f5 |

The reviewed procedure compiled the original plugin and helper into an owned
packet without changing the SDK. Source/header/tool/recipe bindings, current
dependency bytes, generated objects and live provider maps were checked by the
retained partial-reader body; the distinct successful finalizer preserves that
historical validation and the original failed whole-batch outcome. The absence
of a host-installed target DSO was addressed by this packet-only build, not an
SDK installation or a historical-image availability claim. Full-precision raw
rows and actual serialized packets bind the generated fixture. No native mode
replay is needed for Rust comparison; new Rust and integrated gates remain
mandatory before acceptance.

The literal fixture records the native model's local Pose component. The private
Rust replay first left-composes that input with the native identity parent,
matching `Util::worldPose`, before passing the resulting world pose to both the
scalar estimator and ECS dispatcher. Production world poses receive no extra
parent composition. A separate direct-world negative-zero regression checks
the production offset composition, retained pose and publication bits without
that test-input conversion. The immutable 301 input records, 221 actual output
triples and 663 canonical byte strings remain unchanged, with exact comparisons
and no epsilon or ULP allowance.

This source correction follows a confirmed omission found by native-source and
literal-fixture inspection. The fourth normal attempt was interrupted after
both Clippy profiles passed; no numerical test failure is inferred from that
interruption. Earlier failures and native reference evidence remain retained.
The fifth normal attempt then exited 1 after 1,464.264 seconds with ten failures
across four headless targets, despite those three exact replay controls passing.
The M34 failures were two unqualified IO-test endpoint keys and a default-peer
compile warning promoted to an error. The successor uses properly qualified
test keys and braces the existing readiness return, separating the unchanged
deadline check and yield. That C++ control-format correction changes neither
native numerical inputs nor readiness predicates, deadlines or observations;
the original capture identity and literal fixture remain unchanged. Other
failures are handled in the complete integrated correction batch. The mandatory
full hook must run again, including the numerical controls; no separate math
probe or native replay replaces it. Full normal, strict coverage and hosted
qualification remain pending.
