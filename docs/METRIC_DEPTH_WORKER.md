<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Metric depth through the bounded renderer worker

M6 adopts the focused protocol, client and worker source from the preserved
`metric-depth-worker-worktree` onto accepted M5 `60a3a710`. The original candidate
and its failed formatting attempt remain historical. Its renderer dependency is
already accepted in Main through `386482e5`; no renderer production code is
copied or changed by this adoption. The adopted worker extension has not run
tests, lint, compilation, GPU work or subprocess controls yet.

The private protocol is version 3. `CalibratedDepthSnapshot` selects calibrated
metric depth; `DepthFrame` carries exactly 64×64 little-endian IEEE f32 words in
`metres_le`. Values are optical-forward axial metres, ordered from the upper-left
pixel. Positive infinity preserves background/far pixels through Serde framing.
NaNs, zero, negative values, wrong lengths and invalid IDs or stamps are refused.
The existing message-size, visual-count, queue and one-capture limits remain.
Older protocol versions fail explicitly; the version-1 refusal and a separate
version-2 refusal are retained alongside current-version successful handshakes.

Snapshot kind commits only after admission. Client correlation binds the accepted
kind to each capture ID and stamp, rejecting a valid but wrong-kind reply.
Cancellation and snapshot replacement retire the actual old RGB/depth operation
using its original kind. They cannot switch readback methods because a newer
snapshot selected another kind. Both modes share the accepted renderer slot,
submission fence and cancellation lifecycle; no second worker or blocking
operation enters critical simulation stepping.

The established renderer uses the actual stored single-sample Bevy depth
attachment and inverse projection. It conservatively refuses any box with a
corner at or behind the camera near plane before mutating the accepted scene.
This includes boxes wholly behind the camera. It does not reproduce native
near-range negative-infinity semantics. The admitted rendering domain remains
calibrated 64×64 opaque boxes with no world lights; meshes, physical-GPU parity,
RGB-D registration, camera arrays and depth noise are outside this slice.

Authored controls exercise little-endian float words and infinity, invalid
metadata and bytes, valid wrong-kind peers, actual software-worker depth at both
central and off-axis pixels, rejected near-plane admission preserving RGB,
RGB/depth restoration across epochs, and warm in-flight replacement in both
directions. The calibrated small-box f32 pixel bound remains `1e-4` metres;
existing RGB assertions and numerical tolerances are preserved. Changed existing
successful handshake assertions require the current protocol constant rather
than the former version 2.

M5's terminal failed-spawn admission closure, worker dev-only Zenoh introspection
feature, four real RGB worker controls and executable handoff remain unchanged.
Loaded SDF admission, canonical `R_FLOAT32` image publication, scheduling and
CLI integration belong to the coordinated M6 application batch. Worker source
alone proves none of those application outcomes. Independent source review of
the complete feature precedes the single required verification batch; no M6
qualification or acceptance is claimed here.
