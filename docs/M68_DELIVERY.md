<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M68: curved primitives in calibrated metric-depth snapshots

M1–M67 and200 reviewed slices are accepted at exact Main
`cba09f1c088b5b8b328516c8f31aaf554229f1db`, tree
`d03ef2094795c732f12a9183d912547e602c693a`. Full normal/strict coverage,
local DATA, [exact hosted44024955758 attempt1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/44024955758),
durable original collection, independent official DATA and Main/readbacks passed.
Read [M67_ACCEPTANCE.md](M67_ACCEPTANCE.md). Its failed original retained reader
and successful method2 remain distinct. M65 stays Latest/install35.

## Current signed-input delivery repair

The original exact `103fd927` runtime completed full normal and independent local
DATA. Hosted attempts 1 and 2 of run44049236320 both stopped before product
checks on a signed `Packages.xz` HTTP503. Their logs and failure audits remain
immutable. A fresh diagnostic subsequently verified all required signed inputs;
it did not qualify CI. See [the narrow delivery repair](M68_SIGNED_INPUTS.md).

This separate repair checkout adopts all original source blobs, then changes
only input delivery, the safe stager/regression Source, workflow preparation and
current guidance. All Rust/Cargo/fixtures/protocols and all eight SDK input files
remain exact. The feature's 21 Rust controls and four Python declarations below
retain their original scope; eleven stager and one workflow-order Python
regressions are additional Source declarations, not observed passes. Complete
Source/integration review precedes a fresh full normal and all mandatory gates.
The original runtime qualification does not qualify this changed source tree.

The sections below preserve the original feature contract and historical
qualification checkpoints. The repair checkpoint above supersedes their old
next-action prose without relabeling any failure or partial result.

## D31 Source overlap and bounded result

This separate checkout permits M68 Source preparation during M67 hosted/official
closeout. It supersedes only the read-only proposal's acceptance-before-edits
sequence. No M68 formatter, tests, lint, build, native operation, qualification
or publication may run before actual M67 acceptance and complete independent
Source review. If the predecessor changes, adopt/review the affected correction
before checks; no result transfer. Budget at most five working days including
review and all mandatory gates. Preserve frozen M67 and all old evidence.

Admit opaque Box/Cylinder/Sphere/Ellipsoid visuals to ONE calibrated64×64 metric
depth camera. Reuse current meshes and Depth32Float readback, no-world-light and
default-material restrictions, authoritative f64 poses/origin,128 total visuals,
131072-byte IPC frames and one shared capture/client slot. Generic extraction
remains box-only. Existing RGB, old all-Box depth payload/pixels and two-RGB-camera
behavior remain unchanged; mixed/depth pairs remain refused. Preserve consumed
SDF dimension normalization and canonical API fault semantics.

## Closed interfaces and mutation boundary

Protocol7 adds required `CalibratedDepthShapeSnapshot { request_id, stamp, origin,
visuals: Vec<ShapeVisual>, camera: Box<Camera> }`. Existing geometry and reply
variants stay unchanged; no new DepthImage message. ExactReady rejects version6
and earlier, without negotiation. Preserve finite-only wire validation; semantic
positive dimensions/diameters/f32 narrowing remain renderer/extraction checks.
All old request forms remain identical except exact Ready-version expectations.

Renderer adds `set_calibrated_primitive_depth_snapshot(&PrimitiveSnapshot,
Calibration,WorldPose)->Result<(),RenderError>`. Construct at most129 bounded
metadata boxes: CylinderXYZ(2r,2r,L), Sphere(2r,2r,2r), Ellipsoid2rXYZ. Apply the
UNCHANGED `depth::validate_near` oriented-eight-corner guard. Render ACTUAL current
primitive meshes, never the enclosing boxes. The conservative OBB rule may refuse
a safe curved surface; no analytic near-range relaxation is claimed.

Every fallible camera/count/dimension/diameter/f32/ID/color/pose/near/busy check
must precede asset/entity/camera/depth-mode/stamp mutation or pending cancellation.
The existing RGB primitive installer has no fallible branch after its commit
boundary. After a successful install, restore MsaaOff, Camera3d attachment
RENDER_ATTACHMENT|COPY_SRC, and depth_calibrationSome exactly like the old Box
depth setter; the shared RGB installer clears the prior depth mode. Keep the
old Box setter, near validator, shader/material/prepass and decode path intact.
Reuse at most the same three cached meshes and actual selected-mesh readiness.
Depth→RGB restores existing camera/MSAA behavior; invalid depth preserves mode
and any real pending frame. Readback still requires64²,Depth32Float,sample1 and
COPY_SRC,256-byte row alignment and the accepted inverse narrowed projection.

Client, worker and CameraOutput recognize BOTH depth requests in explicit kind
matches. Preserve correlation, epochs, request IDs, Busy/no-retick, stale replies,
cancel/retirement and ownership. Scene emits the old Box depth form for all-Box
worlds and the new form only for curved visuals. Keep prepare_pair RGB-only;
application/scheduled production need no new behavior. Sensor/image demand stays
lazy; absent demand owns no worker, endpoint or per-tick callbacks.

## Disjoint ownership

All authors use only `gz-sim-m68-depth-primitives-source`. Each file has one writer.

- Root: protocol/codec controls; client state/kind/version and fixtures; worker
  conversion/dispatch and direct controls; CLI and finite/RGB registries with
  meaningful Python controls; shared planning/guides, integration and all releases.
- Light: renderer gpu.rs, private/gpu.rs, software.rs and renderer README.
  Production depth.rs stays unchanged; the new wrapper can reuse its near guard.
- Messages: render-scene src/lib.rs +camera_extraction.rs;
  CameraOutput src/lib.rs ONLY the two explicit depth request-kind/snapshot-phase
  matches, and private/camera_output.rs; new worker integration depth_primitives.rs;
  append optional depth mode to existing camera_subscriber.cc; new
  examples/m68_depth_primitives.sdf and docs/M68_DEPTH_PRIMITIVES.md.
- SDF: independent completed contract, Source batches and cross-seam review.

No per-edit checks. Batch all findings and fixes before the consolidated cycle.

## Exact eight old-test admission inverses

Preserve historical test IDs and every unrelated assertion/input/threshold.
Only valid single-camera curved-depth refusal tails change:

1. Scene `cylinder_geometry_stays_refused_by_generic_and_depth_box_only_boundaries`:
   depth tail becomes exact Cylinder depth-shape request; keep generic/time/epoch.
2. Scene `sphere_rgb_refuses_invalid_radius_material_and_depth_without_mutation`:
   depth tail becomes exact Sphere admission; keep bad dimensions/material/generic.
3. Scene `ellipsoid_rgb_refuses_each_invalid_axis_material_and_depth_before_mutation`:
   depth tail becomes exact Ellipsoid admission; keep each-axis faults/purity.
4. CameraOutput `cylinder_invalid_and_depth_geometry_fail_before_publication_admission`.
5. CameraOutput `sphere_invalid_and_depth_geometry_fail_before_publication_admission`.
6. CameraOutput `ellipsoid_invalid_and_depth_geometry_fail_before_publication_admission`.
   In4–6 keep initial invalid-dimension/scope-empty guards. Valid depth must now
   acquire the actual owner scope, stay client-absent without image demand, then
   drop/retire and release it; do not keep the obsolete always-empty premise.
7. CLI `cli_sphere_rgb_keeps_worker_lazy_and_depth_and_owner_refusals`.
8. CLI `cli_ellipsoid_rgb_keeps_worker_lazy_and_depth_and_owner_refusals`.
   In7–8 only valid single-depth command becomes success with exact prior physics
   stdout/empty stderr/no absent worker spawned; keep owner/metadata-only guards.

There is NO ninth single-depth clause or dedicated old Cylinder CLI clause.
`cli_two_camera_path_preserves_single_and_headless_refusal` MUST remain entirely
unchanged, including its mixed two-camera depth refusal. Keep every old M6 Box
depth positive, near/word/numeric constraint and tolerance. Narrow exact protocol
positive-version/Ready fixtures and append-only finite/RGB registry inverses are
also required; preserve prior prefixes, negative/identity controls and all bounds.
No broad old-test retirement or blanket assertion waiver.

## Meaningful finite-mesh and canonical evidence

Use one static64×64 depth fixture: sensor type `depth_camera` with the accepted
M6 default/R8G8B8 input-format metadata, never floating SDF input format to select
R_FLOAT32 output. Camera at0,near.1/far10/FOVpi2/10Hz and ordinary
Sensors intent; Cylinder body(4,-1.5,0),r.5/fullL1 with VISUAL-only pitchpi/2;
Sphere(4,0,0),r.75; Ellipsoid(4,1.5,0),radii(.5,.6,.35); nearBox(2,0,0),size.4³;
backdrop(7,0,0),size(.2,8,8). All links body, old box collisions, gravity0.
Use `/m68/depth/image` and `/m68/depth/camera_info`, rig/body/camera frame.
Finite qualifier observes exact six unchanged body poses; it is not image proof.

Create a small TEST-ONLY ray/triangle reference for a few declared interior rays.
Independently build locked Bevy0.20 unit Cuboid1, capped Cylinder.5/fullheight1/
resolution32/segments1/MidPoint, and Sphere.5 UV32×18 shared with Ellipsoid.
Read only those independently created CPU vertices/indices. Apply explicit
contract WorldPose/semiaxes/basis and narrowed f32 calibration/pixel-centre rays;
never read production EntityTransform/installed mesh/output as the oracle or call
production conversion/decode helpers to derive expected. In camera-local Gazebo axes, parameterize each pixel ray with forward X=1;
transform that declared ray by the explicit camera WorldPose so hit parameter t
is optical-forward axial metres, not normalized-ray Euclidean range. Select the
minimum hit eligible under the current near/far clip interval; no eligible hit
or clear/far gives exact+INF. Preserve the existing1e-4metre
bound; do not assert ideal analytic sphere/ellipsoid depth at that tolerance.
This bounded reference is not a CPU rendering framework or physics solver.

Independent planar anchors: centre occluder1.8m, cylinder cap sample(44,32)3.5m,
backdrop6.9m. Candidate proxy traps(40,35) and(16,34) should reach backdrop, and
corner(0,0) should be exact+INF. These are reviewed Source PREMISES, not observed
pixels. Require actual software proof away from edges, curved off-axis nonflat
depth versus AABB, far clipping/+INF, equal-radii Sphere/Ellipsoid full-depth byte
equality, old Box/RGB restoration, local/common-rigid/large-origin equivalence.
Cover lazy/cache3/selected-mesh/shape replacements, invalid dimension/near/IDs/
color/calibration/pending purity,128/129,RGB↔depth transitions,actual cancellation,
Busy/stale/reset/seq0/no-retick and real owner retirement.

Canonical output is existing `gz.msgs.Image` R_FLOAT32(13),64²,stride256,16384 LE
f32 axial-metre bytes plus CameraInfo; exact stamps/frame/sequence/calibration,
positive+INF for clear/far, no NaN/zero/negative successful depth words. Add one
explicit optional `depth-primitives` C++ consumer mode preserving all old mode bodies. There is no
old C++ depth proof to transfer. Require Image callback≤32768 and Info≤16384 AND
combined hex IMAGE+INFO+labels≤65536 before writing. Retain full bytes, explicit
LE decode and actual C++ child wait/reaping/stream exits. Worker exit stays
UNKNOWN where the unchanged application discards it. Keep existing20speer/90stest/
20sreadback deadlines and caps; no new telemetry or fake depth/stamps.

Count the new target's two unchanged selected_native support tests separately
from authored definitions in the INITIAL census. Complete implementation,
meaningful regressions, guides and independent whole review, then actual accepted
M67 parent before ONE affected formatter and full unchanged normal/strict
workspace/per-crate/actual-parent coverage/local DATA cycle. Exact hosted CI,
durable originals, independent official DATA and ordinary Main/readbacks precede
acceptance/publication. Batch real failures before affected reruns; no weakening.

No noise, pointcloud,RGBD,multiplicity,lights/materials,physics/dependency/SDK/provider/
framework expansion, package refresh or worker distribution. No analytic/native
Ogre/physicalGPU/performance parity. M65 Latest/install35, accepted viewer bytes,
both14MB budgets, old failures/originals/caches and Fuel work remain protected.

The complete implementation, meaningful regressions, guides and initial census
passed independent whole/cross-seam review. Exactly21newRust(H1/R21/overlap1),
4richPython definitions and2separate borrowed support instances are Source
obligations;40finite/RGB13 remain unexecuted for this candidate. M67 is now
actually accepted and its immutable acceptance is adopted in final provenance.
The one affected15Rust formatter passed; full unchanged normal/strict/local
qualification is next. The first M68 normal failed as detailed below; all future exact hosted/official/
durable/Main gates remain mandatory.


## First normal failure and bounded correction

The first full normal failed after290.178seconds: both strict lint profiles and
documentation phases completed, then484Python tests ran with one failure.
`test_rgb_requires_every_named_case_without_failure_or_skip` still expected12
registered/unique CLI controls after the thirteenth was added. This separate
correction updates both exact cardinalities to13, preserving every name and
negative control. Full headless/renderer, finite/RGB qualification and coverage
were not reached. Failedtree3f00, Source/index and all originals stay frozen.
One reviewed correction batch precedes ONE full unchanged normal retry. No Rust,
fixture, tolerance, runtime behavior, formatter or extra trial is introduced.


## Second normal failure and exact malformed-input correction

The second full normal failed after2050.908seconds. Both strict lint profiles,
484Python tests,40finite examples and13auxiliary RGB controls passed. Headless
runtime recorded2049passed/0failed, renderer1292passed/1failed (overlapping,
not summed). The sole failed new protocol control tried to duplicate request ID1
while its inherited valid fixture actually uses3, so its assert-changed guard
correctly stopped before the intended decoder check and nonfinite matrix.

The new correction reads the fixture's actual valid request ID, requires exactly
one serialized member, and duplicates that SAME valid member once. It preserves
both changed-input and malformed-decode refusal assertions; two individually
valid duplicate values avoid a false proof from an otherwise invalid last value.
No production, prior controls, semantic tolerance or payload cap changes. All
other20new renderer controls, including real geometry/canonical consumer proofs,
passed in the failed run; this partial evidence is not qualification. Renderer
coverage was not produced. Preserve failed3f00/d446 and all originals, review the
complete correction, format only the affected leaf once and run the mandatory
full normal/strict cycle. No isolated trial or success reader on failed packets.
