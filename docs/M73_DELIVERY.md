<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M73: explicit monocular camera projection

Budget: at most five working days including review and required checks. The owner
requested the next phase on November 19, 2026. D31/D32 permits this bounded local
successor over clean, locally qualified M72 `2f3e458f8257e72358a607b999bdd4e66a93872a`.
Its full normal/strict gate, independent retained audit and consistency review
passed. M70–M72 remain private and unaccepted while final parent reconciliation,
exact hosted qualification, official evidence and Main acceptance are held.
The historical Ubuntu snapshot HTTP 401 failure has no demonstrated recovery.
No CI retry, snapshot probe or workaround is selected.

## Scope and dependencies

Admit an explicit SDF lens projection P for existing 64 by 64 RGB/depth cameras.
Require finite positive p_fx/p_fy, finite p_cx/p_cy and exactly zero tx/ty.
Select K from explicit intrinsics when present, otherwise preserve exact HFOV
selection. Select P from explicit projection when present, otherwise derive
P=[K|0]. Rendering and its ray/depth interpretation use P; CameraInfo publishes
the separately selected K and P, identity rectification and existing zero
distortion. With no P, preserve exact M72 request and canonical message bytes.
Both selected calibrations retain the current f64/f32 representability check
before external IO. Dormant absent fields remain ignored.

Pinned gz-sensors 8ddf4c17be8a434c51b51d5be7c866fb69aaa54f
CameraSensor.cc UpdateLensIntrinsicsAndProjection selects K before explicit P
replaces the rendering matrix; PopulateInfo publishes each separate DOM matrix.
DepthCameraSensor uses the same selection and publication methods. Its native
rendering ignores tx/ty, so this phase refuses nonzero terms and makes no stereo
claim. Existing native CPU projection and CameraInfo captures retain their
original scopes. The failed Ogre/Mesa precedence helper remains failed: no native
image or native runtime precedence result is claimed. Source precedence, analytic
math, real software-worker images and actual canonical C++ consumers provide the
new scoped evidence. Nested camera.pose is not a new extrinsics offset: native
camera classes use Sensor::Pose, which our canonical WorldPose already models.

Keep all existing camera count/type/rate/64x64/format/AA4/HFOV/clip/noise/distortion/
pose/topic/ownership bounds. Nonzero K skew stays refused. No parser, schema,
protocol version, shader, renderer algorithm, physics, transport, dependency,
framework, SDK, viewer or package-distribution expansion is selected.

## Ownership

- Root: Scene admission and metadata/reset controls; CLI case and closed finite/
  CLI registries; shared guides, planning and integration; qualification releases.
- Extraction/output lane: render-scene calibration selection/extraction and its
  controls; camera-output message constructors/request path and focused packet
  controls. One pure shared selector avoids duplicate HFOV or precedence logic.
- Evidence lane: new M73 SDF fixture; additive actual-worker and canonical C++
  consumer mode; additive application preflight/lifecycle controls. Reuse existing
  M72 custody helpers through small explicit entry points, without copying a
  second harness or altering M72 test bodies/oracles.
- Independent reviewer: complete integrated Source and all assertion transitions.
  Batch every confirmed finding before one consolidated verification cycle.

## Required evidence

1. Cover all four K/P presence combinations and unequal K/P. All four RGB/depth
   box/shape extraction variants must select P while retaining K independently.
2. Check malformed/nonfinite/nonpositive P, nonzero tx/ty and unrepresentable
   selected K/P before IO and without state mutation. Preserve every other
   refusal; only old tests specifically refusing now-supported valid P change.
3. Prove absent-P requests and canonical packet bytes exactly match the existing
   path. Keep the old single-K message API as a compatible constructor, with a
   focused distinct-K/P API for the new path and complete invalid-field controls.
4. Actual software RGB/depth workers must follow P-based analytic rays rather
   than K/HFOV, including distinct focal lengths/principal points. Require exact
   RGB only for homogeneous full pixel footprints, disclose excluded boundaries,
   and preserve all4096 axial-depth/+INF checks and numerical bounds. A canonical
   C++ consumer must check unequal K/P, image format, frame, sequence and time.
5. Retain info-only laziness, first1ms acquisition, independent demand/rates,
   pause, reset restoring original K/P/IDs/sequences, removal and metadata-drift
   stale-frame retirement. No pose teleport or invented native exit result.
6. Register one finite Source example and one exact CLI control, maintaining
   closed result-name/once-only/all-ok validation. Source run counts do not imply
   installed distribution evidence; M65 Latest remains35 examples.

## Qualification and acceptance

Finish implementation, meaningful regression Source and guides, then independent
whole Source review. Batch findings before one formatter and one full required
normal/strict local gate plus independent retained audit. No per-edit test/lint,
product trial or repeated completed parent qualification. Real failures are
inspected together and corrected before affected checks rerun. Never weaken
assertions, deadlines, coverage thresholds or mandatory jobs.

Local qualification is provisional. Final accepted-parent reconciliation and all
exact hosted, durable official and Main gates remain mandatory before M73 source
publication or acceptance. Preserve frozen parents, historical evidence, Fuel
ownership, Rapier f64/TPE and D33 retirement. M1–M69/202 slices remain accepted.
Exclude stereo translations, arbitrary extrinsics, skew/distortion/noise, RGBD
registration/synchronization, point clouds, larger images/camera fleets,
performance claims, packaging and old browser/installed qualification replay.
