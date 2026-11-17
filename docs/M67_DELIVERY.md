<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M67: ellipsoid visuals in calibrated RGB snapshots

M66 is accepted at Main `1b0c3be1af7c751987df53b4b96269236f7ae9c5`,
tree `fd7edcdd920fb780551a3a23022b97b167138668`, with 199 accepted slices
across M1–M66. M65 remains public Latest with 35 installed examples.
See [M66 acceptance](M66_ACCEPTANCE.md). Never amend qualified predecessors.

## Bounded result and closed interfaces

Within at most five working days including independent review and mandatory
checks, admit opaque positive finite canonical SDF ellipsoid VISUALS in the
existing calibrated 64×64 RGB path. Mix them with accepted boxes, cylinders and
spheres. Preserve physics/collisions, authoritative poses, shaders/materials,
no-world-light/default-opaque restrictions, shared 128 visuals, 131072-byte
frames, one capture/client slot and current calibrated image fields. Generic
and depth extraction remain box-only and refuse ellipsoids before IO.

Internal protocol version 6 adds only required tagged
`Geometry::Ellipsoid { radii: [f64; 3] }`. Preserve all prior shape payloads;
version-5 and older peers fail exact Ready without fallback. Malformed fields,
unknown variants, invalid dimensions and shared IDs fail before mutation.
The codec preserves finite-only wire validation; positivity, diameter and f32
representability remain semantic renderer admission, as in M66.
Client/worker epoch, stamps, capture kind, Busy, cancellation, stale-response and
resource ownership rules remain unchanged.

Renderer adds `EllipsoidVisual { id: u64, pose: WorldPose, radii: DVec3,
color: [f32; 3] }` and `PrimitiveVisual::Ellipsoid`. Preserve old public structs
and setters. Reuse the EXACT existing lazy `MeshKind::Sphere` radius-0.5 UV
32-sector/18-stack handle, including ellipsoid-first allocation. Local diameters
`(2rx, 2ry, 2rz)` map to Bevy scale `(2rx, 2rz, 2ry)` through unchanged WorldPose.
No extra rotation, fourth mesh, per-size cache or tessellation option.
Equal radii must produce identical RGBA to the accepted Sphere path. Same-ID
replacement preserves entity/material ownership and updates actual mesh/scale.

Every semiaxis and diameter must be positive finite and narrow to positive finite
f32 before changing assets, entities, cameras, stamps or pending readback.
Validate all IDs, colors, poses, calibration and total limits as one preflight.
The retained consumed SDF parser defaults missing radii to `[1,1,1]`, rejects
malformed/nonfinite/wrong-length triples, and normalizes ANY nonpositive component
to the WHOLE unit triple. Preserve that policy; canonical API/component fault
controls must not claim invalid XML survived parsing.

## Disjoint Source ownership

Use only the new `gz-sim-m67-rgb-ellipsoids` checkout over accepted M66. Each
shared file has one writer. No tests/lint/builds/native/API qualification during
individual edits. Root serializes all operational releases.

- Root: protocol/codec tests; client and worker version/shape/state conversion;
  CLI refusal controls, finite/RGB registries and Python registry regressions;
  shared planning/guides, qualification and tracker.
- Light: `crates/gz-renderer/src/gpu.rs`, `tests/private/gpu.rs`,
  `tests/software.rs` and renderer README only; `lib.rs` already reexports types.
- Messages: `crates/gz-render-scene/src/lib.rs`, `tests/camera_extraction.rs`;
  append-only focused CameraOutput private tests if needed, no production there;
  new `crates/gz-render-worker/tests/rgb_ellipsoid.rs`;
  append optional mode/dispatcher in `tests/fixtures/camera_subscriber.cc`;
  new `examples/m67_rgb_ellipsoid.sdf` and `docs/M67_RGB_ELLIPSOID.md`.
- SDF: independent contract, completed Source and cross-lane integration review.

## Meaningful evidence

Preserve all old test bodies and assertion strength except reviewed exact
protocol-version (including positive fixture Ready literals), exhaustive
enum-match and finite/RGB registry inverse updates. Registry counts advance
38→39 and11→12 only for actual appended obligations; old tail slices become
bounded slices, and synthetic success includes all required names. Preserve old
missing/ignored/forged-result negatives and append ellipsoid controls. Add each-axis invalid radii/diameter/narrowing,
shared IDs/count128/129 and pending-state refusal purity; ellipsoid-first bounded
cache; same-ID Sphere/Ellipsoid/Box/Cylinder mesh/scale/readiness and material
ownership; removal, empty/reset/retirement, Busy/cancel/stale behavior. Verify
local/world/camera/common-rigid and large-origin equivalence. Restore full old
Sphere RGBA for equal radii and full old Box pixels after shape roundtrip.

Use an asymmetric red ellipsoid at X4 with radii(.5,1.5,.75), blue backdrop at X7,
near green box at X2, camera at origin, gravity0, 64RGB/10Hz/FOVpi2 and explicit
`/m67/rgb/image` and `/m67/rgb/camera_info`. Collisions remain boxes. A second
control rolls only the VISUAL local pose 90 degrees about Gazebo X, swapping the
horizontal/vertical silhouette without moving body/collision/camera. Off-edge
red/blue samples must distinguish a single-radius sphere and AABB proxy and
prove near-box occlusion. Proposed samples (42,32)/(32,27), blue(43,37)/(32,41)
are analytical Source premises, NOT observed evidence; review them and require
genuine software-rendered proof. The finite UV approximation is not analytic,
native Ogre, physical-GPU or performance parity.

Append an explicit ellipsoid dispatcher/mode while preserving no-arg, pair,
cylinder and sphere C++ bodies. Retain full canonical Image/CameraInfo bytes,
calibration/frame/sequence/stamp and actual owned C++ wait/exit. Worker exit stays
UNKNOWN where the application discards it. Reuse maintained bounds and deadlines.
Count the new target's two unchanged selected_native helper tests separately
from authored feature definitions in the INITIAL source census.

## Qualification

Complete all implementation, meaningful regression Source, documentation and
independent whole review; batch findings before one affected formatter and one
consolidated unchanged normal hook. Strict workspace/per-crate/actual-parent
coverage, independent local DATA, exact hosted CI, durable originals, independent
official DATA, ordinary Main promotion and readbacks precede acceptance.
Batch genuine failures before affected reruns; no weakened gates or extra trials.
Register the new fixture and actual RGB case once (proposed finite39/RGB12 are
obligations, not passes). No package refresh or worker distribution, physics,
lighting/material/depth expansion, dependencies/SDK/provider/framework changes,
new transport or broad compatibility inventory. Accepted viewer bytes and both
14MB budgets, M65 public Latest/install35 and all original evidence stay intact.

The complete contract and batched corrections passed independent Source review.
Implementation, meaningful regressions, guides and whole independent lane/cross-seam
review are complete. The single affected formatter passed. Final staged bindings
precede full normal/strict qualification; no M67 product checks have run.
