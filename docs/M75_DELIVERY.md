<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M75: per-camera visual visibility masks

Selected before implementation on November 19, 2026 under the owner's next-phase
instruction and D30–D32. Budget: at most five working days including independent
review and mandatory checks. Base is locally qualified M74
`487f0eab28b9fa505ae02becd71e74f69f54c65f`, now available on the owner-requested
GitHub review branch. M70–M74 remain unaccepted; the new automatic M74 hosted run
44627469212 was in progress at selection. No hosted success is inferred, and no
retry, snapshot probe or workaround is selected.

## Contract

Use the existing 64 by 64 RGB/depth camera paths and existing camera-count,
calibration, material, geometry, lighting, pose, rate and ownership bounds.
Admit camera visibility masks and visual flags in the low 28 bits
(`0x0FFFFFFF`), plus the existing `u32::MAX` default all-visible sentinel. Other
reserved-bit inputs remain unsupported. Select a visual exactly when
`visual_flags & camera_mask != 0`. Default values preserve the original behavior.

Validate every visual in the admitted world before filtering, including identity,
geometry, material, pose and reserved flags. Retain the global 128-visual bound
before filtering; a zero camera mask cannot hide invalid data or excessive scene
size. Preserve sorted identities. Generic noncamera snapshots remain box-only and
retain their current strict default-visibility requirement and error ordering.
Avoid new protocol fields: filter the immutable acquisition snapshot before it
enters the existing worker request. An empty selected snapshot still renders
the unchanged Bevy RGB background [43, 44, 47] or positive-infinity depth under
image demand; it is not no-demand. This default comes from pinned Bevy 0.20.0
ClearColor, not the native test's explicitly selected black background. The new
whole-frame software controls must establish its actual bytes.

Camera metadata changes retain existing stale-owner retirement semantics.
Visual metadata belongs to each immutable acquisition snapshot. Loaded reset
restores original camera masks, visual flags, IDs, calibration and sequences.
Preserve info-only laziness, independent cadence, pause, selective removal,
session ownership and clean owned worker retirement.

Native references are source-derived: gz-sensors
`8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`, CameraSensor.cc and
DepthCameraSensor.cc forward SDF masks; gz-rendering
`fb8e38cee4dad491f95f205afe6697b6a5d0f5fa`, RenderTypes.hh defines the low-28-bit
all mask, Ogre2Camera/RenderTarget reserve higher bits, and CameraTest.Visibility
selects differently colored co-located visuals using masks 1 and 2. These are not
newly executed native renderer or image-equivalence evidence.

## File leases and required evidence

- Root owns Scene admission and parsed-metadata/reset controls, CLI/closed
  qualification registries, planning and guides. No production change outside the
  two existing admission/extraction modules is intended.
- Extraction lane owns render-scene collection/filtering and focused controls:
  exact IDs/order for disjoint, overlap, zero and default masks; exact default
  compatibility; all shape routes; generic refusal; hidden-invalid and global
  128-visual boundary checks. Do not edit Scene or shared guides.
- Evidence lane owns one additive SDF fixture, actual software-worker/canonical
  C++ modes and application lifecycle controls. Reuse the existing harness and
  its numerical/full-pixel/depth oracles; keep older test bodies unchanged except
  tiny shared event/entry plumbing. Prove selective RGB occlusion and all 4,096
  depth values/+INF, empty rendering, actual packet K/P/header/time/sequences,
  demand/cadence/pause/removal and mask-drift/reset behavior. No fake pose or new
  observer framework. Preserve actual exits and unobservable-exit limitations.
- Independent reviewer reads the entire completed implementation, meaningful
  regression source and documentation, including every transitioned old refusal.
  Batch findings before one formatter and one consolidated normal/strict gate.

Add one finite source fixture and one exact CLI case, retaining the closed
all-names/all-outcomes registry and every old negative control. Source runs are
not installed examples; M65 Latest remains 35 examples. Preserve all retained
thresholds, deadlines, strict coverage gates and native interoperability controls.

## Qualification and acceptance

Finish implementation, regressions, guides and independent whole-source review
before checks. Run no per-edit tests, lint, native probes or product trials.
One required full local normal/strict cycle and a reviewed retained-evidence audit
follow the completed source batch. Inspect real failures together before any
affected rerun; retain originals and never weaken assertions.

Local qualification is provisional. Final accepted-parent reconciliation, exact
hosted success, durable official evidence and qualified Main promotion remain
mandatory for acceptance or release. The owner's source-push instruction permits
review-branch availability after required local qualification; it does not imply
acceptance. Preserve frozen parents, all failures, Fuel ownership, Rapier/TPE and
D33 retirement. No shader, renderer algorithm, schema/protocol, SDK, dependency,
physics, transport, framework, viewer, package, camera-count/resolution, RGBD,
noise, distortion or performance expansion is selected.
