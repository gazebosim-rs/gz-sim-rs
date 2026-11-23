<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M109 — true capsule visuals in calibrated RGB and depth

This selected lane carries explicit capsule radius and cap-center distance
through the existing calibrated shape protocol, Scene extraction, renderer and
worker. Generic legacy extraction remains box-only. No arbitrary Mesh camera
visuals, new renderer, transport or protocol framework is introduced.

## Geometry and ownership

A capsule is centered on Gazebo local Z. Its segment length is the distance
between the hemispherical cap centers; its complete axial size is length plus
twice the radius. Both radius and segment length must be finite and strictly
positive in the admitted f64/f32 domain. Diameter and complete extent must also
remain finite and positive after narrowing. Existing ID, pose, color, material,
visibility, global 128-visual, calibration and snapshot ordering gates apply.
Hidden visuals still undergo complete validation before camera filtering.

The mesh is constructed with Bevy0.20's existing `Capsule3dMeshBuilder` using
the actual narrowed radius and segment length, 32 longitudes and 18 latitudes.
The mesh has genuinely spherical caps; it is never an anisotropically stretched
fixed capsule. Gazebo Z maps to Bevy Y through the existing pose conversion.
Capsule instance scale is one. Dimension-specific mesh handles are shared only
for identical narrowed dimensions, at most 128 after a completed replacement;
obsolete variants are removed after old instances have been replaced. A valid
replacement may transiently hold the prior and next bounded sets. This is an
ownership bound, not a measured allocation or performance claim.

All input validation and the existing busy check precede mesh/entity mutation
or readback cancellation. Invalid later visuals preserve the prior snapshot.
Existing box, cylinder, sphere and ellipsoid branches, legacy framing and worker
retirement remain. Metric depth uses the actual rendered depth attachment and
inverse projection, with M110's selected geometric near-plane preflight.

## Regression Source

New protocol controls roundtrip capsule radius/length in RGB/depth requests and
retain finite/wire/global limits. New Scene controls check exact capsule geometry,
world pose, legacy extraction refusal and invalid hidden geometry. Real software
renderer controls compare interior axial depths to an independent analytic
finite-cylinder/two-sphere union across lengths and rotations, bound the known
fixed-mesh approximation, observe changed silhouettes, preserve prior images on
invalid late input, and exercise 128 dimension-specific meshes, replacement and
empty retirement. Actual worker tests cover RGB/depth/stamps and semantic refusal
with prior-frame preservation and explicit worker shutdown.

Root owns retained valid-Capsule refusal-to-positive rows in protocol and render
Scene tests, and exhaustive-match attachments in the old software renderer
fixtures. Other malformed Capsule records, old shape cases, assertions and
native reference fixtures remain. No assertion or tolerance is weakened.

The retained depth-output refusal case keeps its original Capsule dimensions,
visual target and depth fixture as a calibrated depth-shape positive. It checks
exact geometry, frame and zero-time stamp, lazy no-demand admission and ownership
of two writer keys until explicit retirement and drop. A valid Plane at the same
target retains the exact unsupported-geometry refusal before worker startup or
writer reservation. This is a Source correction to an obsolete Capsule refusal;
no current renderer, worker or full-normal pass is claimed.

## Leases and acceptance

Owned production paths are render-protocol `src/lib.rs`, render-scene
`src/lib.rs`, renderer `src/gpu.rs` and render-worker `src/main.rs`. New tests
are protocol `tests/capsule.rs`, render-scene `tests/capsule.rs`, renderer
`tests/capsule_depth.rs` and worker `tests/capsule_worker.rs`. The renderer test
leaf also contains the separately identified M110 controls. M109 and M110 share
one writer and are sequenced in that file. The lane budget is five working days
including review and all required checks.

All current results are Source-only. No test, lint, formatter, worker, GPU or
native execution has run for this authored lane. Root must independently review
the complete combined candidate and release its single consolidated verification
cycle after actual resource admission. Exact local/hosted/original-artifact
qualification is required; there is no acceptance or Main promotion from Source
completion or inherited evidence. Frozen trees, Fuel and the live demo stay intact.
