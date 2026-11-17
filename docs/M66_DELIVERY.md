<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M66: sphere visuals in RGB camera snapshots

M65 is accepted at Main `069f315c1466c40120d85f6944a5f478ef6e9c71`.
Its installed 35-example preview is public and Latest at `m65-preview`.
M1–M65 contain 198 accepted slices. The immutable receipt is
`artifacts/m65-delivery/acceptance-main-promotion.json`; this new candidate
does not amend that qualified commit or repeat its demonstration.

## Bounded result

Within at most five working days including review and mandatory gates, admit
positive finite, opaque SDF sphere VISUALS in the existing calibrated 64×64 RGB
path. Mix spheres with accepted boxes and cylinders. Preserve physics/collisions,
body pose, frame conversion, shaders, materials, dependencies and transport.
Retain no-world-light/default-opaque restrictions, 128 total visuals, 131072-byte
protocol frames, one client/capture slot and current calibrated image fields.
Generic `extract` and depth extraction remain box-only and refuse spheres before
IO. All-box requests retain their old wire form and pixel behavior.
These admission rules apply to canonical loaded geometry: the retained upstream
SDF parser may normalize a nonpositive raw sphere radius to its default before
render extraction. This phase does not change or claim rejection by that parser.

## Closed interfaces

The internal worker protocol becomes version 5; version-4 and older peers fail
the exact Ready handshake without negotiation or fallback. Add only required
`Geometry::Sphere { radius:f64 }` to the current tagged shape request. Preserve
existing Box/Cylinder fields and request/reply variants. Unknown/missing/duplicate
geometry fields and unknown variants fail closed. Client/worker stamp, request,
capture-kind, epoch, Busy, cancellation and stale-response rules stay unchanged.

Add renderer `SphereVisual { id:u64, pose:WorldPose, radius:f64, color:[f32;3] }`
and `PrimitiveVisual::Sphere(SphereVisual)`. Reuse `PrimitiveSnapshot` and
`set_calibrated_primitive_snapshot`. Keep old structs and setters intact.
Use one lazy cached Bevy 0.20 sphere mesh: radius 0.5, UV 32 sectors/18 stacks,
uniform scale `2*radius`, no extra axis rotation. Read the locked upstream API
before authoring. At most three mesh assets exist after all three shapes are
used; old box/cylinder paths allocate no sphere. No user tessellation or per-size
cache is introduced.

Validate every dimension, shared nonzero unique ID, material, pose and calibration
before mutating assets/entities/camera/stamp or cancelling readback. Reject zero,
negative/nonfinite radius, diameter overflow and f32 narrowing to zero/infinity.
Same-ID replacements among all three kinds must choose the actual mesh handle;
readiness checks the selected mesh. Preserve removal/clear, bounded assets and
pending replacement/cancel/reset behavior. RGB shapes continue to refuse depth.

## Parallel ownership

All authors use the new isolated `gz-sim-m66-rgb-spheres` checkout over accepted
M65; never edit accepted/frozen checkouts. Each shared file has one writer.

- Root: protocol/codec controls; client and worker conversion/version/state seams;
  finite and RGB qualifier registries, integration, shared guides/planning and
  all serial operational releases.
- Light: renderer `gpu.rs`, renderer exports if needed, private/software renderer
  regressions and renderer README. No protocol/client/worker edits.
- Messages: render-scene extraction/tests; narrow CameraOutput integration/tests
  if needed; `examples/m66_rgb_sphere.sdf`; optional sphere mode in the existing
  canonical C++ camera subscriber; new real-worker RGB integration test and a
  separate guide appendix. No Root or renderer files.
- SDF: independent contract, completed Source batches and integration review.
  Findings are batched before one consolidated verification cycle.

## Required meaningful evidence

Controls must cover mixed shared IDs/counts and invalid-input purity, bounded lazy
mesh assets, selected-mesh readiness, same-ID Box/Cylinder/Sphere replacement,
removal, pending capture replacement/cancellation/reset/stale behavior, visual
local poses and large-origin/camera equivalence. Preserve all old controls and
tolerances; narrowly replace the admitted RGB sphere refusal with positive
controls while retaining generic/depth and other unsupported-geometry refusals.

Real software-rendered pixels must distinguish a sphere from an AABB proxy:
red interior versus blue background at a corner inside the AABB but outside the
silhouette, plus a near green box occluder. Choose samples away from MSAA edges.
Require full-frame old box equality after a shape round trip. State the finite
32×18 UV approximation; no analytic, native Ogre or physical-GPU parity claim.

The static fixture has a camera at the origin, red sphere at X=4, blue backdrop
at X=7 and green near box at X=2, zero gravity, 64×64 RGB/10 Hz and explicit
`/m66/rgb/image` and `/m66/rgb/camera_info` routes. Collisions remain old boxes.
Use the maintained software worker and current scalar CameraServer API; paired
camera behavior is unchanged. Preserve the subscriber's default and cylinder
modes, adding an explicit sphere dispatcher. Decode actual canonical Image and
CameraInfo, retain full messages, verify dimensions/stride/frame/sequence/stamp
and actual owned C++ child exit. This proves a canonical consumer, not native
rendering parity. Report worker codes UNKNOWN where the unchanged application
discards them; never invent reaping or exit zero.

Register the new fixture in the maintained finite qualifier and genuine sphere
RGB control in the closed RGB registry. Existing finite fixtures/control bodies
remain intact. Reconcile exact authored IDs/counts from completed Source before
binding evidence; declarations are not measured passes.

## Qualification and exclusions

Complete implementation, meaningful regression Source, guides and independent
whole review before one affected formatter and consolidated full unchanged
normal hook. Strict workspace/per-crate/actual-parent coverage, independent local
DATA, exact hosted CI, durable originals, independent official DATA and ordinary
Main/readbacks precede acceptance/publication. Reuse unchanged evidence only
where existing checks permit it. Inspect all available genuine failures and
batch corrections before affected reruns; no gate weakening or extra trials.

M65 Latest/install35 and its accepted six viewer bytes/both 14 MB budgets remain
unchanged. No package refresh, RGB/depth-worker distribution, SDK/provider build,
new transport/runtime/packaging framework, physics/solver, lighting/material,
depth geometry, sensor expansion or broad compatibility inventory is selected.
Implementation, meaningful regression Source, guides and independent whole
review are complete. The single affected formatter passed; final staged bindings
precede the unchanged full normal/strict gates. No M66 test, lint, build, native
or qualification has run yet.
The complete contract passed independent Source review before implementation
was released to the disjoint author lanes.
