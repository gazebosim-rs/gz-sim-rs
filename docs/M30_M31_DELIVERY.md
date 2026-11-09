<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M30/M31: ideal LogicalCamera and static-vector PosePublisher

M1–M29 are accepted at Main `ab9660ee`. The verified installed Latest is
[m29-preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m29-preview).
[M28/M29 acceptance](M28_M29_ACCEPTANCE.md) records all actual qualification and
publication proofs. These next two independent source lanes use that exact
parent. Each is budgeted at most five working days including review and mandatory
checks under D30–D32; neither inherits acceptance from the parent.

## M30: ideal CPU LogicalCamera

Admit one loaded top-level direct-link Rapier logical camera in a flat world of
at most 128 uniquely named top-level models. Require four explicit, unique finite
parameters: `0 < near < far`, `0 < horizontal_fov < pi`, and `aspect_ratio > 0`.
Require finite, nondegenerate derived vertices/planes and unit finite input
quaternions; finite parameters alone do not prevent overflow or degenerate
normals. Reject noise, nested/duplicate model names, unsupported metadata, invalid
poses and configuration drift before creation. The initially consumed SDFormat revision `a44bf8be` recognizes the kind but
rejects its configuration child before raw retention. The separately qualified
`82c35f24` correction admits that singleton child only for kind `logical_camera`,
preserving `SensorConfig::Unconfigured` and exact `Sensor.configuration_xml`.
Its independent review, normal/coverage hook, exact hosted run `42614152237` and
independent local/official DATA passed before the consumer pin changed. Scene consumes only this bounded raw element with existing
roxmltree, requiring its effective kind and Unconfigured typed metadata to agree.
Do not edit the paused SDFormat candidate or add parser infrastructure or libraries.

Pinned native LogicalCameraSensor checks **model origins**, not bounding boxes,
geometry or occlusion. Use the existing Scene world poses, public identities,
attached sensor pose and glam double transforms. Implement only the small,
attributed f64 six-plane/point predicate needed for the pinned native inclusive
frustum contract; record its exact native source before edits. Do not build a
geometry framework or use renderer f32 frustums. A model's own geometry does not
affect inclusion. Include the owning model if its origin is inside; do not invent
an owner exclusion. Preserve native lexical bare-model-name ordering and output
full camera-relative model poses plus the sensor's world pose.

One acquisition publishes both existing canonical `LogicalCameraImage` at the
base topic and `LogicalCameraSensor` at raw base + `/frustum`. Reserve and map
both endpoints before IO, including all live/initial cross-family collisions.
Image header has native stamp/frame/sequence; frustum header has stamp/frame and
no sequence. Either listener activates acquisition once; no demand, paused and
non-due states do no acquisition or stale replay. Reuse SampleSchedule, bounded
owned state, existing upstream Zenoh and weak IO leases. Invalid due input must
invalidate the whole frame before publication. A zero-visible-model image is
valid. Absent sensors own no callbacks, buffers, worker or endpoint. Preserve
fresh reset handles, queue epochs, sequence restart and immediate removal.
Document Rust reset/removal guarantees separately from native TODO/early-return
limits; do not claim unproved native rewind parity.

Meaningful controls cover inclusive planes and just-inside/outside points,
behind/near/far, offset/rotated camera, full relative rotation, finite/norm/name
and model-count refusals, lexical order, own-model inclusion and size independence.
Move camera and target with existing velocity APIs and compare due samples to
actual current WorldPose while frames stay held between due ticks. Cover no
interest, image-only, frustum-only, both, reconnect, pause, reset, removal,
absence, both reserved keys/remaps, no-IO atomic refusals and actual canonical
headers on both streams. The example/fixture is `m30_logical_camera.sdf`. While a logical camera is
installed, runtime template spawning must preflight the flat, unique-name world
and final 128-model bound before any ID, ECS, native-body or IO mutation. Preserve
ordinary spawning behavior when no logical camera is installed.

A focused native SensorFactory helper must use the actual retained logical-camera
provider and same pose/config inputs, prove both canonical messages and edge/
rotation/ordering behavior after real readiness, and retain actual exits and
loaded-library identities. Preserve the original 19 nominal plane inputs; add
12 exact side-face-center and adjacent witnesses (31 plane acquisitions, 36
acquisitions across four modes). Assert actual native face-center distance is
exactly zero before inclusion checks. Nominal geometric edges may have a nonzero
computed plane distance, so record all actual planes/distances and preserve their
native classifications in a typed Rust replay fixture. This bounded reference
fixture generation precedes the final complete Rust source/review and one normal
hook; it is not an exploratory native probe or a visibility tolerance. Source/provider/hash/ABI/maps agreement is reviewed
before execution; no SDK rebuild, mutation or fallback is authorized. Native
system source supplies model-origin semantics; do not require a missing Sim
system DSO or start a broad SDK campaign. No renderer, pixels, AABB/occlusion,
noise, second camera, new backend, UI widget, service or performance work.

## M31: static-vector PosePublisher

Extend the existing one flat top-level Rapier model/vector publisher, retaining
all ordinary M27 modes and defaults. New `static_publisher=true` requires all
direct link poses (`publish_link_pose=true`), with model pose optional and at
most 128 selected frames total. Require a valid direct canonical link and bare
direct joint link names (`world` may be the parent), retaining the current joint/
backend domain. The all-link requirement avoids native selected-count minus
omitted-dynamic-link reserve underflow; refuse model-only split mode explicitly.
No nested, visual, collision, sensor or individual-Pose output is added.

Native appends `_static` to the raw chosen base topic before mapping each key.
Check derived length, map both independently and reject aliases and all live/
initial peer or cross-family collisions before IO. Static rate inherits dynamic
rate unless explicitly supplied; accept only finite static rate in static mode.
Reuse native `(1/f)*1e9` truncation and the existing nonpositive/unthrottled policy,
including a derived zero-ns period. Keep independent initial-zero last clocks.
Skip only `0 < difference < period`; paused suppresses both. Static publishes
first when both are due: this is call/enqueue order, not guaranteed cross-topic
callback arrival order. Check received vectors per topic. Both endpoints remain
declared even when a group is empty. There is no native listener-interest gate,
latching or empty-vector packet. Re-read local poses every due static publication; static
classification does not imply an immobile world pose.

Classify direct link endpoints of non-fixed/non-invalid joints as dynamic,
except the canonical link. Selected model/canonical/fixed-only records are
static. Preserve native per-Pose stamp, two ordered frame entries, name/id0,
no sequence and no outer header. Compare native vectors by child-frame key,
without imposing incidental hash-map order. Separate channel sample/revision/
last state prevents a static-only tick replaying dynamic data. Reuse the current
IO registry, worker, weak leases, queue epochs, counters and fault owner; no new
transport family or worker. Invalid due vectors clear stale samples and fault;
reset identity/metadata/two-channel preflight stays atomic. Preserve M27 direct
core reset retaining both live last clocks; loaded/native world or time-only
reload restores both clocks to zero. Do not conflate those reset boundaries.

Meaningful native/wire/lifecycle evidence includes both real endpoints and
readiness, a moving noncanonical joint with actual component/keyed comparisons,
unthrottled split, independent 100 Hz/25 Hz rates (10 dynamic/2 static over
100 one-ms advancing ticks), inheritance, pause/equal-time/no-interest behavior,
and an actual reset witness followed by 40 ticks (4 dynamic/1 static). Include
a moving jointless model with refreshed static model poses, no dynamic packets,
canonical/fixed classification and no empty packet. Cover held deduplication,
both-direction remap/collision/initial reservations, drift/reset refusal without
IO/clock mutation, closed IO, bounded queues, epoch cancellation, weak retirement
and absence. Existing negative intent remains; any newly admitted old case needs
a stronger positive plus the model-only-static refusal. Use
`m31_pose_static_publisher.sdf` and `native_pose_static_publisher.cc`.

## Native source and small shared API

Read-only reconnaissance pinned Sensors `8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`,
Sim `71c60db31fddd5c98d2aa689e9a2cccb3548a0a3` and Math
`1316ed334f4d5bda7be59aae07cd6e8ff31cfe13`. Relevant source SHA-256 values:

- Sensors `src/LogicalCameraSensor.cc`: `1ca85b3f431061d45de4fc102060e206ba017af027ad0033c56c73ea7a216ee4`;
  public header: `d73b48d4585bea6f424ce2c0164dea439dc06d96c0e7c8aee1d7cca35d88e795`.
- Sim `src/systems/logical_camera/LogicalCamera.cc`: `2c7179d4279da388639c8495bf58e200d3a843e6ddd10c0733d9bb875e25d297`.
- Math `src/Frustum.cc`: `d899e16789a7a1a2fb6c08ab5e3f5ba6c1a2f49bf8af961b83ca1ec7139f97f6`;
  `include/gz/math/Plane.hh`: `c70b0f0b5cbdd26de7a724e5949fbe34564e7e66ad9eabdc3f51254f5d98f73f`.
- Sim `src/systems/pose_publisher/PosePublisher.cc`: `e70f34969f0fa128da64e60803423d83327ab7a6e9ca9d1f6caf6761582e705b`.

Math Plane::Side excludes only negative distance; equality is inside, with no
added visibility epsilon. The retained local LogicalCamera provider file is
`libgz-sensors-logical_camera.so.11.0.0~pre2`, SHA-256
`5da77b29c3aa2cc7b2bedcc8a51d5d72697f3192a90082cbe2d8d1f2bb882925`;
its pc metadata declares Sensors11.0.0. These are source/file identities, not
loader, ABI or runtime proof. Record actual helper inputs/maps/dependencies before
claiming native execution; no mutation or broad SDK build follows from this pin.

M31 may add `PosePublication.static_publication: Option<StaticPosePublication>`,
where the new metadata contains a topic and existing `PoseFramePublication`
values. Existing `poses` are dynamic in split mode; groups are disjoint and total
at most128. Ordinary metadata remains None with old nonempty behavior. A
`StaticPoseSample(PoseSample)` component provides independent time/epoch/revision,
reusing the current frame and value types and one optional static clock in the
existing publisher. Keep `initial_pose_publications`' iterator signature; its
snapshot must carry both channel metadata. Root owns the exports and both-key
live/initial/reset reservation integration. Report other shared API needs before
editing shared files; this is not a new runtime abstraction.

## Ownership and integration

Light owns new `gz-sensors/src/logical_camera.rs` and matching pure tests; new
Scene logical-camera leaf/private/public tests, `m30_logical_camera.sdf` fixture,
`native_logical_camera.cc`, example and `docs/M30_LOGICAL_CAMERA.md`; new Server
logical-camera leaf/IO leaf and dedicated private wire/lifetime tests. Root owns
module declarations/exports and all shared admission, initial state, activation,
reset, lifecycle, schedule and topic-reservation integration.

Messages owns Scene `src/pose_publisher.rs`, private pose-publisher tests and new
`pose_static_publication` private/public tests, fixture/native helper/example
and `docs/M31_POSE_STATIC.md`; Server `src/pose_publisher.rs` and its three private
pose/IO/world-IO test leaves. The existing `pose_publisher_io.rs` production file
may be changed only if a concrete need is reported to Root first; its multi-key
registry is reused. Root delegates shared Server `lib.rs`, `reset.rs`, `world_io.rs`, options/removal
integration and all-family reservation tests to Messages in the Root integration
worktree after the leaf review. Root retains shared Scene and Sensors integration
and final coordination. The file leases remain disjoint.

Authors use separate worktrees over exact accepted `ab9660ee`; Root integrates
in `gz-sim-m30-m31`. Report needed shared API signatures before implementation.
Do not edit each other's leaves or any frozen accepted tree. SDF independently
reviews scope, leaves and complete integration. Finish implementation, meaningful
regression source, examples and guides; batch all findings before one consolidated
normal required pass. No per-edit check/lint/test/build/native executions.

Root alone serializes local heavy jobs. Each focused native contract and full
normal hook, strict per-crate/workspace line AND region and actual-parent gates,
exact hosted CI, durable original evidence and independent local/official DATA
are mandatory before qualified Main acceptance. Source lanes and later remote
CI/local work may overlap only with actual prerequisites; corrections require
review and affected checks. No old whole gate or browser proof transfers. These
are source-feature phases, with no new installer/archive/browser refresh; M29
remains the installed Latest. Preserve all old failures, warnings with unknown
causes, accepted demos, Fuel and resource floors. After acceptance, continue the
next useful bounded retained slice; no broad compatibility or performance claim.
