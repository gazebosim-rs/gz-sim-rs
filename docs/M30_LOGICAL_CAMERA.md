<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M30 ideal LogicalCamera

This source feature admits one top-level direct-link Rapier logical camera in a
flat world of at most 128 uniquely named models. The fixture and example are
`m30_logical_camera.sdf`. Supply four explicit unique parameters under
`<logical_camera>`: finite `0 < near < far`, `0 < horizontal_fov < pi` and positive
finite `aspect_ratio`. Derived vertices and planes must remain finite and
nondegenerate. Poses use finite unit quaternions. Noise, rendering, nesting,
second cameras and unsupported metadata/configuration drift are refused.

The pinned native Sensors `8ddf4c17`, Sim `71c60db3` and Math `1316ed33` contract
checks **model origins**, including the owner when visible. Model size, visual or
collision geometry and occlusion do not affect detection. The geometry-size
regression keeps valid colliders on static models, so geometry changes cannot
move the tested origins through physics. The six f64 planes
include zero signed distance and exclude negative distance, without an added
visibility epsilon. Native Vector3's normalization threshold is retained as an
arithmetic operation, not a visibility tolerance. Output names are bare and
lexically sorted. The image contains the sensor world pose and each detected
model's full camera-relative pose, including rotation. This is not an AABB camera.

Each demanded due acquisition generates canonical `gz.msgs.LogicalCameraImage`
at the chosen base topic and `gz.msgs.LogicalCameraSensor` at raw base plus
`/frustum`. Both are mapped/reserved independently before IO. Either listener
activates the acquisition; both outputs are published. Images carry stamp,
frame_id and sequence. Frustum messages carry stamp/frame_id and no sequence.
An empty image is valid. Invalid due poses, identities or metadata invalidate
the whole pair, with no stale successful replay. Queue admission/transport is
bounded independently per endpoint; this does not promise atomic delivery of
both topics to remote receivers.

No demand, paused polls and non-due ticks do not acquire or advance deadlines.
Frames remain held between acquisitions. Loaded reset restores the original
world, fresh entity handles, epoch and sequence restart. Removal immediately
retires Rust state, callbacks and weak IO leases. These Rust guarantees are
separate from native Sim's rewind TODO and removal early-return behavior; no
native world-rewind parity is claimed. Runtime model additions must preserve
flat topology, unique names and the model cap before mutation. Absent cameras
own no acquisition state, callbacks, endpoint or IO worker.

The focused SensorFactory helper uses the existing logical-camera provider and
both actual canonical subscribers after bounded readiness. Modes `planes`
(31 acquisitions, including the original 19 nominal inputs and 12 exact
side-face-center/adjacent witnesses), `rotated` (3), `empty` (1), and `order` (1) retain input poses,
received poses, both header contracts and live library maps. The 36 acquisitions
produce 36 image/frustum pairs. Native face-center witnesses must have exactly
zero computed plane distance; nominal geometric edge coordinates are characterized
separately because floating-point construction can give them a nonzero signed
distance. Preserve all original nominal inputs and compare their actual native
visible names in a typed Rust replay fixture; do not add a visibility epsilon. Pure and loaded regressions cover boundaries,
refusals, real motion, cadence/demand, reset/removal, wire/lifetime and collision
contracts. These are source controls, not execution or acceptance claims. Actual
native execution, full mandatory qualification, strict line/region coverage,
exact hosted CI and independent DATA review remain required before acceptance.
M29 remains the installed preview; this phase does not refresh that archive.
