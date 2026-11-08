<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M26/M27: independent AirSpeed and PosePublisher source lanes

D30–D32 continue from accepted M25 Main `08ae5525` in isolated
`gz-sim-m26-m27`. M25 remains the checked installed download and Latest. These
are two new bounded features, each at most five working days including review
and mandatory checks. No successor implementation or qualification is inferred
from M25. Finish exact native Source agreement before implementation; finish
implementation, meaningful regression sources, guides and independent Source
review before one consolidated verification pass.

## M26 ideal loaded AirSpeed

Support exactly one loaded direct-link, top-level Rapier AirSpeed sensor with
zero world wind, wind disabled, default atmosphere and exactly default no-noise
metadata. Refuse unsupported metadata, backend, multiple/nested sensors and
nonfinite/domain-invalid inputs explicitly before mutation. No new SDF parser,
noise model, atmosphere configuration, service, worker or transport framework.

Use actual sensor-point velocity: link-origin velocity plus world angular
velocity crossed with the resolved sensor offset; rotate once into the composite
sensor frame. Native Sim AirSpeed passes relativeVel(sensor), whose utility uses
the inverse composite sensor-world rotation and sensor WorldLinearVelocity;
Physics fills this velocity with LinkFrameDataAtOffset. Do not substitute link
center velocity or rotate the velocity twice.

Match pinned native AirSpeedSensor's f32 atmosphere and pressure evaluation:
absolute sensor world Z, 288.15f and .0065f, powf exponent 4.256f, density 1.225f,
signed sensor-X quadratic differential pressure, f32 assignment and final native
100.0f multiplication. Native published numeric scale is Pa despite the protobuf
comment saying hPa; document the discrepancy and validate actual native numbers.
Negative reverse-flow pressure is valid. Temperature is Kelvin. Reject nonfinite
intermediates and nonpositive atmosphere-domain values; a runtime acquisition
failure replaces stale data with a typed fault and publishes no old valid frame.

Reuse existing finite sample scheduling, demand, canonical timestamp/frame/seq,
weak IO ownership, reset epochs and immediate Rust retirement. Preserve actual
native paused/due/connection gates. Do not claim native rewind or removal timing
parity where native code has TODOs or returns early without consumers. Native
proof includes actual SensorFactory numeric cases and a finite real Sim/DART
rotated-offset witness, with original component velocities/poses and canonical
messages. This is not a cross-engine trajectory or native RNG claim.

## M27 flat-model PosePublisher

Support one named built-in PosePublisher on one top-level flat Rapier model,
with at most 128 selected model/link records and vector messages only. Match
native publish_link_pose default true and publish_model_pose default false;
require nonempty selection. Explicitly refuse nested/visual/collision/sensor pose
publication, static publisher/frequency, multiple publishers, runtime plugin
spawning and unsupported/duplicate XML/options. Existing visual/collision/sensor
entities remain present; only their pose records are omitted. No dynamic plugin
host is introduced.

Native poses are actual parent-relative components::Pose, not viewer world
poses. For world default/model rig, model frame is default and child rig; links
use frame rig and child rig::link. Each Pose has its own timestamp and ordered
frame_id/child_frame_id headers, name equal to child frame, wire ID zero. Pose_V
outer header is absent. Native uses unordered_map: preserve captured order but
compare native records by unique child-frame key. Rust may use documented stable
lexical order; never call it canonical native order.

Match native unpaused cadence: last publication starts at zero; a positive
period skips only when 0 < time difference < period. Equal-time unpaused and
rewound times are eligible; all paused publication is suppressed. Nonpositive
finite frequency means every unpaused tick. Reject nonfinite frequency and positive
reciprocals outside a representable signed nanosecond duration. Truncate admitted
positive periods to nanoseconds like native, including an admitted zero period
above 1 GHz; do not apply the sensor scheduler's 1 ns clamp. Do not use sensor scheduling,
viewer wall-clock throttling or a subscriber-interest gate for acquisition.
Native world/time-only reset reloads this plugin because it has no ISystemReset,
restarting its last-publication time at zero. Loaded Rust reset restores that
initial timer/cache; direct core Reset retains the timer as a documented Rust
convenience, and must not be labelled native world-reset parity. Native proof
observers must survive actual reset and measure the reloaded plugin cadence.
Direct and loaded reset semantics must be explicit, with fresh handles and
all-or-nothing initial/live metadata, source-kind and topic collision guards.
A malformed/missing pose retires stale data instead of publishing a partial frame.

Use canonical existing Pose_V definitions and the maintained WorldIO worker,
weak leases, bounded queue and global generation fence. An absent publisher owns
no family-specific runtime work/resources. Native proof uses the same flat
loaded fixture and actual component Pose witness for default links, model-only,
both, cadence, paused and rewind cases. The selected SDK lacks both AirSpeedSystem and PosePublisher system DSOs.
Build only each standard native plugin's one object/DSO in the owned proof packet
from its actual qualified compile/link recipe and pinned upstream Source. Bind
the existing SensorFactory AirSpeed provider and all SDK inputs; do not rebuild,
install into or mutate the SDK. These are native reference plugins, not a Rust
dynamic plugin host or new runtime feature.

## Ownership and qualification

Light owns the new AirSpeed model, Scene and Server leaves, private/integration
regressions, native helper/fixture, example and AirSpeed guide. Messages owns the
new PosePublisher Scene and Server leaves, private/integration regressions,
native helper/fixture, example and PosePublisher guide. No author changes shared
files or runs per-edit checks. Root owns all shared exports, admission/build,
producer guards, initial/reset/removal/rollback, WorldIO integration, all live and
initial reserved keys, fixtures shared by existing suites and canonical plans.
SDF independently reviews native contracts, source, regressions and integration.

Use one integrator and serial Root-released local heavy work. After the complete
batch and independent Source clearance, run one full normal hook, strict
workspace/per-crate line AND region coverage and actual-M25-parent gates; retain
focused real native/canonical wire proofs and independent DATA. Publish the exact
candidate to normal review CI, pass every job and independently audit durable
original evidence. Qualified Main promotion is required before accepting either
phase; the old M25 check is not a new pass. A future updated installed archive is
separate scope and requires its own fresh build/package/install/browser evidence.

Preserve Fuel, accepted demos, all frozen trees/captures, old original failures,
capacity floors and native SDK bytes. No optional backend, renderer rebuild,
body teleport, telemetry facade, transport/framework rewrite, full compatibility
inventory, broad noisy AirSpeed or unrelated system work is authorized here.
