<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M6: one loaded metric-depth camera

Implementation and qualification are pending. Budget: five working days including
independent source review and required checks. M1–M5 remain accepted.

The bounded delivery selects one loaded SDF `depth_camera` through the existing
CameraServer owner. The admitted input is the same 64×64 opaque box/no-world-light
camera metadata domain as RGB. The producer's accepted default image metadata is
retained; the sensor kind selects depth output. Do not declare an unsupported
floating-point SDF input format to select it. One RGB or depth camera is admitted;
arrays, RGB-D registration, point clouds and depth-noise parity are deferred.

Reuse the metric-depth renderer already accepted in Main. The paused worker
candidate supplies a focused private IPC v3 extension: calibrated depth snapshots,
explicit little-endian float payloads, exact capture-kind correlation and bounded
replacement/cancellation. Frozen candidates are preserved. Existing RGB requests,
outputs and controls remain required; mismatched private protocol versions fail
explicitly. See METRIC_DEPTH_WORKER.md for the adopted worker boundary.

Depth images use canonical `gz.msgs.Image` with `R_FLOAT32` (13), dimensions 64×64,
stride 256 and 16384 bytes of little-endian f32 axial metres. Positive infinity
represents background or far clipping. Zero, negative and NaN words are rejected.
CameraInfo, frame and sequence use the existing messages. Acquisition timestamps
come from committed simulation state; canonical Image has no invented epoch field.
Reset invalidates old local-epoch buffers before sequence zero restarts.

The renderer conservatively refuses any box with a corner at or before the near
plane, including boxes behind the camera. This explicit limited domain is retained;
no native near-plane or negative-infinity parity is claimed. Calibrated centre and
off-axis pixel checks use the existing small f32 depth bound of 1e-4 metres.

Use the existing opt-in `rgb-camera` build feature and `--camera-worker` CLI flag
for this owner; the feature name is retained for compatibility. Build the CPU CLI
with `--no-default-features --features rgb-camera` and use the matching GPU-enabled
worker binary. The loaded fixture is `crates/gz-camera-output/tests/fixtures/m6-depth.sdf`.
The separate camera peer preserves outgoing connect/scouting configuration and
has no listening endpoint; configure a reachable observer/router explicitly for
isolated canonical delivery. Headless default paths create no renderer.

Tick and capture remain separate. Output Busy cannot repeat committed physics or
acquisition. GPU, network and retirement waits remain outside critical stepping.
The complete Source, regressions and documentation must receive independent
review before one consolidated required verification pass. No M6 execution or
acceptance is claimed by this document. Retained RGB assertions, numerical bounds,
native-free closure checks and both strict coverage profiles remain mandatory.
