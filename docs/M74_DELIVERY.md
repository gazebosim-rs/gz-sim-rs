<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M74: signed pinhole camera skew

Budget: at most five working days including review and mandatory checks. The
owner requested the next phase on November 19, 2026. D31/D32 permits this bounded
local successor over clean, locally qualified M73
`03f3fc9dfc594ca7ebccb9af46ba8968b83ca342`. Its first full normal/strict gate,
independent retained audit and consistency review passed. M70–M73 remain private
and unaccepted pending final parent reconciliation and exact hosted/official/Main
gates. The earlier Ubuntu snapshot HTTP 401 has no demonstrated recovery; no CI
retry, snapshot probe or workaround is selected.

## Scope and dependencies

Support finite signed SDF intrinsic skew for the existing 64 by 64 RGB/depth
camera domain. Preserve positive fx/fy, finite principal points and all current
image, clip, HFOV, pose, noise, rate, count and ownership bounds. Active K selects
its supplied skew; absent K ignores dormant skew and retains exact HFOV behavior.
Absent P renders with K, including skew. Explicit zero-translation P still wins
and selects rendering skew zero; both selected calibrations are validated before
IO. Stereo translations and arbitrary projection forms remain unsupported.

Pinned gz-sensors `8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`
CameraSensorUtil builds clip matrix entry [0,1] = +2*s/width. Under the existing
optical convention (+X right, +Y down, +Z forward), the native-compatible rule is
u = fx*X/Z - s*Y/Z + cx, v = fy*Y/Z + cy. The inverse ray uses
Y/Z = (v-cy)/fy and X/Z = ((u-cx)+s*(v-cy)/fy)/fx. Preserve this sign explicitly.

A retained actual CPU utility capture contains skew35 matrix [0,1]=0.109375 at
640 pixels: artifacts/render-camera-calibration-worktree/artifacts/
camera-native-projection.log, SHA cbdd8a6ef7ba857f469d3a2ac7d7ed478044ce0fccddc1ac3564a25639359f99.
The adjacent original identity record pins the exact utility source and fixture.
It is native projection math evidence, not a native renderer or sensor image.
Reuse this unchanged capture with a tiny tracked numeric fixture and provenance;
do not rebuild the SDK or replay the old failed Ogre/Mesa precedence helper.

The pinned CameraSensor::PopulateInfo explicitly publishes K[1] and P[1] as zero,
even when rendering consumes intrinsic skew. Preserve this native message
limitation: CameraInfo remains zero-off-diagonal while skew affects rendering.
Do not add an IntrinsicValues skew field or invent a nonzero native packet.
The old 325-byte native capture used zero skew; nonzero-skew omission is derived
from pinned source, not an observed new native capture. New actual canonical C++
consumer checks establish interoperability of our packets, not native images.

Projection, optical projection and Bevy frustum corners must agree. Extend
representability checks to all sheared near/far corners in f64 and f32, including
coupled overflow missed by independent unsheared edge checks. Reject nonfinite
skew or a nonzero shear that is not a normal representable f32 coefficient, and
invalid narrowed inverses where necessary, before IO. Preserve the existing
zero-skew matrix/corner/request/message path exactly. Use one small shared corner
calculation for admission and renderer frustum corners; no projection framework.

## File ownership

- Root: Scene finite-skew admission and narrow old refusal transitions; extraction
  selection controls; CLI and closed finite/CLI registries; shared guides,
  planning, protocol/message comments, integration and qualification releases.
- Projection lane: render-frame camera calibration and focused tests/numeric
  fixture; renderer CalibratedProjection corner delegation and focused private
  controls. Preserve all shader, depth-inverse and protocol algorithms.
- Evidence lane: new M74 fixture, additive actual-worker/canonical C++ modes,
  small existing custody helper entries and additive application controls.
  Reuse the existing harness without copying it or changing M72/M73 oracles.
- Independent reviewer: the entire implementation, regression source, guides,
  math sign/bounds and every supported-domain assertion transition. Batch all
  findings before one consolidated verification cycle.

## Required evidence

1. Compare skew35 to the retained native matrix and exercise positive/negative
   skew. Check projection/project_optical/inverse rays, sheared near/far corners,
   crop/frustum agreement, coupled corner overflow and exact zero-skew behavior.
2. Cover absent K, explicit K without P and explicit P overriding nonzero K skew.
   Reject nonfinite and unrepresentable selected K before IO even if P overrides;
   retain every translation and other camera-domain refusal.
3. Actual software RGB/depth images must follow independent inverse-sheared rays,
   with signed/zero/HFOV discrimination. Require homogeneous full pixel footprints
   using all four inverse-projected corners, not an axis-aligned front bbox or
   an assumed MSAA pattern. Keep exact colors and meaningful pixel counts, disclose
   excluded boundaries, and check all 4,096 depth values/+INF with existing bounds.
4. Actual explicit-P workers must keep the existing M73 pixels despite nonzero
   K skew. Canonical C++ consumers check actual complete image/CameraInfo bytes,
   zero offdiagonals, K/P, frame, sequence and first 1 ms time, with actual exits and
   retained bounded custody evidence. No new native image equivalence claim.
5. Retain info-only laziness, demand/cadence/pause, buffered-skew-drift retirement,
   loaded reset restoring original skew/K/P/IDs/sequences, selective removal and
   clean owned worker retirement. No fake pose or new telemetry framework.
6. Add one finite Source fixture and one exact CLI case. Preserve the once-only,
   all-name/all-outcome closed registry and all old negative controls. Source
   finite runs are not installed examples; M65 Latest remains 35 examples.

## Qualification and acceptance

Complete implementation, meaningful regressions, guides and independent whole
source review. Batch findings before one formatter and one full required local
normal/strict gate, then one reviewed retained-evidence audit. No per-edit tests,
lint, native probes or product trials. Inspect real failures together before any
affected rerun; never weaken numeric assertions, coverage, deadlines or gates.

Local qualification is provisional. Final accepted-parent reconciliation, exact
hosted success, durable official evidence and qualified Main promotion remain
mandatory before M74 source publication or acceptance. Preserve all frozen
parents, older failures, Fuel ownership, Rapier/TPE and D33 retirement. M1–M69 and
202 slices remain accepted. No shader, schema, protocol version, transport,
dependency, SDK, solver, viewer, package, larger camera fleet/image, noise,
distortion, RGBD/point-cloud or performance expansion is selected.
