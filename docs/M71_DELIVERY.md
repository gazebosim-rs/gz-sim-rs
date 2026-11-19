<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M71: independent RGB and depth camera pair

Budget: at most five working days including review and mandatory qualification.
The owner requested the next phase on 2026-11-19. M70 has completed its local
normal, release, package, installation, camera-component, RGB and actual browser
proofs with independent audits. Its exact candidate is `7094a31890e78bc83c65f12a056dfb3918b7401a`.
It remains private and unaccepted: hosted qualification is blocked by the retained
Ubuntu snapshot HTTP 401 failure. No CI retry or snapshot probe is selected here.
M1–M69 and 202 slices remain accepted; DART and MuJoCo remain retired.

## Scope and dependencies

Develop in a new isolated successor over that locally qualified M70 candidate.
One configured world owns exactly one supported calibrated 64 by 64 RGB camera
and one supported calibrated 64 by 64 depth camera. Each retains independent
image/info demand, rate, sequence, worker, failure and lifetime. Reuse the existing
camera state machines, worker protocol, extraction, canonical messages and
unmodified upstream Zenoh. No image registration or shared exposure is promised.

M71 development, independent Source review and local qualification may proceed
while M70 hosted closeout is held. The base is genuinely locally qualified;
its old results do not qualify M71. Finish the complete Source batch and review
before one consolidated local normal/strict pass. All results remain provisional.
Final parent acceptance, reconciliation of any parent correction and all exact
hosted, official and Main gates remain mandatory before M71 acceptance or
publication. A changed parent requires affected review and checks. Do not mutate
or replay the frozen M70 release, installation, camera or browser proofs.

## Interfaces and ownership

Add explicit `CameraServer::new_mixed_pair` and
`CameraServer::new_mixed_pair_with_server_session` constructors, and
`Server::with_external_mixed_camera_outputs(..., [u64; 2])`. IDs are an unordered
complete set; loaded sensor kinds determine RGB versus depth. Validate exactly
one of each kind, all IDs, bindings, calibration, rates and four distinct remapped
image/info keys before opening external IO. Existing scalar, RGB-pair, depth-pair
and automatic default constructors retain their admission and refusal semantics.
The new shared-session constructor must never close the world's session when
one output or the complete output family retires.

Add explicit CLI `--camera-mixed-pair`, requiring `--camera-worker` and server
mode. A non-renderer build rejects it as it rejects camera-worker. Without this
opt-in the current CLI continues to reject mixed camera worlds. No installed
launcher/package extension is selected.

Root owns Server admission, CLI, qualification registry, planning and integration
in `gz-sim-m71-mixed-camera-pair`. The camera-output lane owns only its application,
error enum and private regression Source in `gz-sim-m71-camera-output`. The
camera-evidence lane owns a new mixed fixture, real-worker integration target and
narrow canonical consumer support in `gz-sim-m71-camera-evidence`. Independent
review examines the complete integrated batch before any verification.

## Required evidence

The fixture has separate rigs and routes, RGB at 10 Hz and depth at 20 Hz,
calibrated 64 by 64 images, existing near/far limits and distinct box targets.
Keep all existing test bodies, numerical assertions, timeouts and queue limits.

- Reject wrong cardinality, two RGB/two depth, unsupported types/calibration,
  duplicate/zero/foreign IDs and cross-role topic aliases before IO.
- No demand and info-only demand start no worker; each image demand starts only
  its selected owner. Verify canonical RGB_INT8 and R_FLOAT32 bytes, complete
  calibration/header/frame/time/sequence fields and distinct actual scene views.
- Require independent rates and first 1 ms stamps, paused silence and unchanged
  committed time during capture. Busy or a failed owner cannot skip its peer or
  authorize a repeated physics tick.
- Removal retires the selected real worker while its peer remains usable. Last
  removal releases the family; loaded reset reconstructs the same mixed intent,
  original IDs and fresh sequences, with no stale in-flight publication.
- Exercise the explicit CLI and unchanged default refusals. Use actual bounded
  canonical upstream consumers and actual owned exits, plus real software workers.
  Worker identity disappearance is observable; discarded worker returncodes stay
  unknown. Preserve the selected interval scope of no-worker assertions.

After the complete Source batch and independent review, run one consolidated full
normal hook with strict workspace/per-crate/actual-parent coverage and independent
local audit. Local checks may overlap the parent hosted hold. Final parent
acceptance, exact hosted success, durable originals, independent official audit
and qualified Main promotion remain required for acceptance or publication.
No Source declaration or old M70 result is an M71 pass.

Exclude RGBD registration, synchronized exposures, point clouds, noise, dynamic
registration, greater camera counts, physics, renderer/shader/lighting changes,
new dependencies/SDK/framework, viewer changes, package distribution, native Ogre
pixel equivalence and performance or universal memory-fit claims.
