<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M64: cylinder visuals in RGB camera snapshots

Current acceptance: M61–M64 passed local and exact hosted qualification,
independent local/official DATA and ordinary Main/readbacks at192ede. M1–M64/197
are accepted; M60 remains Latest/install33. See
[the actual acceptance record](M61_M64_ACCEPTANCE.md). The feature contract and
retained qualification checkpoints below describe their historical scope.

## Runnable scope and preserved domain

Admit opaque, positive finite SDF cylinder VISUALS in the existing calibrated
64×64 RGB camera path, mixed with existing boxes. This makes the already retained
Z-axis visual primitive available to cameras; it changes no physical collision,
solver, body pose, frame conversion, shader, material, dependency or transport.
Keep existing opaque/default material and no-world-light restrictions, 128 TOTAL
visuals, 131072-byte protocol frames, one capture/client slot and 64×64 images.
No native Ogre pixel, physics, performance or physical-GPU equivalence is claimed.

The generic public box-only `extract` remains strict. RGB `extract_camera` may
produce the new explicit shape request; box-only worlds keep their old request
and exact field bytes. Depth extraction retains its box-only behavior and refuses
cylinders before any output/worker starts. No paired camera/depth or paused metric-
depth-worker scope is selected. Existing RGB snapshots have no depth calibration;
the new RGB geometry setter must preserve existing NoDepthSnapshot refusal.

## Closed wire and renderer interfaces

- Internal worker protocol becomes version4. Old client/worker versions fail the
  existing exact Ready handshake; there is no negotiation, downgrade or shape
  fallback. Preserve all valid old Box request forms and public `Visual` struct
  literals. Explicitly reject unknown fields on Visual and Request in version4
  so geometry cannot be smuggled into an old Box message and ignored.
- Add required tagged `Geometry::{Box { size:[f64;3] }, Cylinder { radius:f64,
  length:f64 }}` and `ShapeVisual { id, translation, rotation, geometry, color }`.
  New records and geometry tags reject unknown/missing/duplicate fields and
  unknown variants; no default shape. `Request::CalibratedShapeSnapshot` has the
  existing request_id/stamp/origin/camera and `visuals:Vec<ShapeVisual>`. It is RGB
  only. Keep all old Request variants and all Reply/frame shapes unchanged.
- All Request getters, wire validation, client stamp/request/capture-kind
  correlation, worker snapshot ordering/cancel/Busy paths and CameraOutput RGB
  classification must handle the new variant using the existing state machines.
  Enforce the same combined count/message limits; malformed/unknown geometry
  fails closed instead of drawing a proxy box or skipping a visual.
- Preserve renderer BoxVisual/Snapshot structs and old setters/box pixels. Add
  `CylinderVisual { id:u64, pose:WorldPose, radius:f64, length:f64, color:[f32;3] }`,
  `PrimitiveVisual::{Box(BoxVisual),Cylinder(CylinderVisual)}` and
  `PrimitiveSnapshot { epoch:u64, time:Duration, origin:DVec3,
  visuals:Vec<PrimitiveVisual> }`. Add only the calibrated unlit RGB entry
  `set_calibrated_primitive_snapshot(&PrimitiveSnapshot, Calibration, WorldPose)`.
  It shares existing snapshot/readback lifecycle, not another renderer or queue.
- Use one cached unit Cuboid and a lazily created centered, capped Bevy0.20
  cylinder of radius0.5/fullheight1, 32 radial sectors, one height segment and
  MidPoint anchor. No user tessellation or per-size mesh cache. Maximum two mesh
  assets after first accepted cylinder; box-only usage allocates no cylinder.
  Scale the cylinder by `(2*radius,length,2*radius)` in Bevy coordinates. Existing
  Gazebo+Z→Bevy+Y pose conversion already maps its axis: NO extra90° rotation.
- Validate every kind/dimension/ID/material/pose/calibration before asset/entity/
  camera/stamp mutation or cancelling readback. Reject zero/negative/nonfinite
  radius/length, diameter overflow and dimensions that narrow to zero/infinity.
  Preserve shared unique nonzero IDs and total128 across both shapes. Same-ID
  Box↔Cylinder replaces/selects the actual Mesh3d handle; readiness checks each
  selected mesh, not just the old cuboid. Removal/clear/cache bounds and old
  replacement/cancel/generation/Busy semantics remain unchanged.

## Ownership and evidence

Root owns protocol/codec tests, client and worker conversion/state-match seams,
shared qualifier/current guides/planning and integration. Light owns renderer
GPU implementation and private/software regressions. Messages owns render-scene
RGB extraction and tests, CameraOutput's narrow new-variant match and tests,
new cylinder fixture, existing canonical subscriber's optional cylinder mode,
and real software-worker RGB integration regression plus guide appendix. SDF
independently reviews complete batches and shared integration. Separate author
trees start at accepted M60; only Root edits shared files. Read pinned consumed
SDF/native camera and locked Bevy0.20 sources before implementing.

Source-declared controls include cylinder_dimensions_and_mixed_ids_preflight_before_scene_mutation,
unit_cylinder_axis_caps_and_shape_replacement_reuse_bounded_meshes,
software_cylinder_side_endcap_rotation_and_box_occlusion_are_real_pixels,
loaded_rgb_cylinder_geometry_preserves_boxes_material_and_origin_contract,
worker_cylinder_snapshots_keep_shared_stamps_atomic_refusal_and_epoch_ordering,
and canonical_rgb_cylinder_frame_preserves_dimensions_stride_frames_and_sequence.
Preserve every old control/assertion/tolerance; narrowly move the exact admitted
RGB-cylinder refusal to a meaningful positive and retain unsupported Sphere/
other-geometry negatives. Old box-only extract and depth-cylinder refusals stay.

Real software-worker proof must discriminate a box proxy: end-on cap interior is
coloured while an AABB corner outside the circular silhouette shows background;
side/end/rolled views show the actual axis; near/far Box+Cylinder occlusion is
independent of IDs/order; same-ID Box→Cylinder→Box restores the old full-frame
box pixels. Include actual local visual pose and large-origin subtraction,
invalid-input purity, bounded mesh assets, selected-mesh readiness and pending
readback replacement/cancel/reset/stale controls. Keep samples strictly off MSAA
edges and state the fixed32-facet approximation honestly. Existing calibration,
canonical Image/CameraInfo dimensions/stride/frame/sequence/stamp and actual
owned C++ peer exits remain required. Worker exit codes remain UNKNOWN where the
existing application discards them; no invented zero or reap proof.

The existing single-camera application is sufficient; M63 pair APIs are not a
dependency. Keep the native camera client key and no-argument behavior; add an
explicit optional cylinder mode with an actual main dispatcher, not a helper
that relies on main-only implicit return0. Reuse the current software worker and
canonical consumer recipe, not a new SDK/native Manager/Ogre capture campaign.
Installed M60 and its protocol3 binaries/viewer/package remain unchanged.

Implementation, regression Source, fixture/docs and independent integration review
are complete; the consolidated formatter exited zero. Actual real
software/canonical execution, full unchanged normal hook and strict workspace/
per-crate/actual-parent coverage, independent local DATA, exact hosted CI,
official DATA and ordinary Main/readbacks remain mandatory before acceptance or
publication. No source-readiness or formatter result is release acceptance.

## Cylinder fixture and canonical consumer Source

`examples/m64_rgb_cylinder.sdf` declares a static camera rig at the origin,
red cylinder at `(4,0,0)`, blue backdrop at `(7,0,0)` and green occluding box at
`(2,0,0)`. Each model has link `body`; the camera uses `/m64/rgb/image`,
`/m64/rgb/camera_info`, frame `rig/body/camera`, 64×64 RGB and 10 Hz. Gravity is
zero. Only the red visual uses cylinder geometry; all collision geometry stays
in the old Box domain. Its radius is 1 and full length is 2. Local visual pitch
π/2 points its authoritative SDF Z axis toward the camera without altering the
model pose or adding a renderer axis correction.

The declared real software-worker controls require green at image centre, red
inside the cap and blue at pixels inside its AABB but outside its circular
silhouette. A second genuine loaded world removes only the green visual and
requires red at centre. Samples avoid MSAA boundaries; the renderer uses the
fixed capped 32-sector approximation, not an analytic circle or native Ogre.
The tests also require actual 1/100/200 ms acquisitions, unchanged calibration,
RGB stride, loaded frame and sequence; paused holds; and reset cancellation of
a buffered shape frame before fresh epoch-1 sequence-0 output. They use the
accepted scalar CameraServer API, without an M63 pair dependency.

The existing `camera` canonical native client retains its no-argument M5 mode.
Its explicit `cylinder` mode subscribes to the two M64 routes, decodes actual
upstream Image/CameraInfo, checks cap/occlusion pixels and retains their complete
serialized messages. The Rust control compares both decoded messages to its own
actual delivered packets and requires the owned C++ child to exit 0. That is a
canonical transport consumer proof, not native rendering parity. CameraServer
output retirement is checked separately; its discarded worker exit code remains
UNKNOWN, and no reap/exit success is invented.

These are Source declarations awaiting consolidated execution and all mandatory
normal, coverage, independent DATA, hosted and Main gates. Protocol 4 requires a
matched client/worker pair; immutable installed M60 protocol-3 binaries are not
replaced or claimed compatible with cylinder requests. Generic extraction and
depth remain Box-only, while all-Box RGB requests retain their old wire form.


## Finite execution and pending qualification

The shared native-free qualifier registers the 33 accepted M60 examples plus
`m64_rgb_cylinder.sdf`: 34 future finite executions. The four static body poses
must remain exact after two 1 ms steps. Parser and retained-evidence regression
Source rejects altered time, pose, identity, count, command, status or fixture
claims. Finite loading and stepping does not prove camera pixels or canonical
transport; the real-worker and C++ controls above are separate obligations.
Installed M60 still contains 33 examples. M61/M62 and M63 remain independently
frozen; their registries, APIs and results are not copied into this candidate.
