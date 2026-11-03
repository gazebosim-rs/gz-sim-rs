<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M4: live Rapier IMU delivery

Accepted and published on 2026-11-03 at simulator
[eae6f3b8](https://github.com/gazebosim-rs/gz-sim-rs/commit/eae6f3b81ed5e52d74648dfd1634ae95a8e9a41b),
with [all sixteen exact hosted CI steps passing](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41659117101).
The accepted viewer `5b67f45f` and its bundles are unchanged. M1–M3 remain
preserved. [Launch and sensor contract](M4_IMU.md).

This delivery qualifies the existing IMU implementation through one runnable
SDF example and four focused regression controls. It adds no production code,
transport layer or Messages functionality. The example uses a two-kilogram
Rapier box, a centered sensor rolled 90 degrees, a 1 ms physics step and a
100 Hz ideal IMU on `/mvp/imu`, frame `imu_box/body/imu`.

## Sensor and lifecycle evidence

The passing controls prove:

- SI specific force and sensor axes using a known force/mass input; rad/s and
  orientation using a known nonzero angular velocity.
- Exact end-step acquisitions at 1, 10 and 20 ms, paused acquisition, core and
  loaded reset epochs, and restored initial state.
- A no-IMU world's callback absence, two loaded consumers through last-consumer
  retirement and reset restoration, and stable sensor identities.
- Server subscriber-interest suppression, explicit activation and sample counts,
  pause/reset and closed-world behavior, with no invalid samples or IMU failures.
- Real two-session upstream Zenoh delivery using actual listener locators and
  maintained Prost decoding: canonical attachment, entity/frame, sequence,
  timestamps, zero-noise covariance and ideal free-fall values across two resets.

Local epochs are asserted on the simulation/sample. Wire timestamps and sequences
restart; no extra epoch field is claimed in the canonical IMU message. Structural
last-consumer removal is distinct from deactivating a still-loaded registration
or having no subscriber. Existing lifecycle and overload assertions remain intact.
This is not a new C++ sensor-parity capture or full noise-model qualification.

## Required checks and independent review

Implementation, regression code, docs and independent source review preceded one
consolidated normal staged pass: 910.222 seconds, exit zero, no capacity stop.
Exact hosted CI and independent local/official source/data audits passed, as did
independent review of actual browser observations and binary/source bindings.
No assertion, tolerance, denominator, threshold or mandatory check was weakened.

| Separate owned profile | Local and hosted lines | Local and hosted regions |
| --- | --- | --- |
| Headless | 12044 / 12264 | 18345 / 18908 |
| Renderer | 6256 / 6382 | 9233 / 9588 |

Every required crate exceeds 95% lines and regions separately. Profiles are not
combined. Actual parent-relative production changed coverage is **0/0** because
only tests, fixture and documentation were added; this is not new production
coverage credit. Hosted zero/full-source changed results separately pass at
11570/11720 headless and 6116/6172 renderer, with every crate passing its gate.

Actual logs contain 1020 headless, 559 renderer and 276 tooling passes, zero
failures, and all four new IMU controls passing. Five/three ignored parent entries
remain delegated helpers. Ten native-off cases pass with four retained native-free
executables, thirteen actual raw profiles, disabled-arm observations and a
system-library-only closure. Eight retained C++ Zenoh peer scenarios preserve
existing compatibility evidence; the new IMU wire observer is Rust Zenoh/Prost.

Audits checked 2150 local artifact hashes, 558 distinct source/input bindings
including 78 production files, and all 1227 official source blobs and Git modes.
The 275 full-profile Rust executable records are metadata-only; no independent
binary re-export is claimed. Hosted SDK DSOs are remotely metadata-bound,
rather than claimed locally rehashable copies. Four native-off ELFs are retained.

## Actual browser observations and next delivery

[Four original captures and source/binary bindings](evidence/m4-browser/README.md)
show the IMU-bearing cube move onto ground at current time 26.414 seconds,
committed pause at 30.220 seconds, and reset to epoch 1/time zero with original
apparent placement at the same camera. The exact qualified instrumented
native-free executable and accepted viewer bundles were reused without a build.

The browser is not an IMU sample inspector. Units, frames, sample counts,
covariance and wire values are established by the passing tests, not screenshots.
Background 1 fps is visible; no performance or separately inspected renderer
backend result is claimed. Startup initially showed Waiting before the scene.

M5 now connects one SDF RGB camera to existing offscreen Bevy rendering and
bounded canonical image delivery, targeted within five working days including
review and mandatory checks. Camera arrays, distortion, thermal, segmentation
and broader sensor/noise compatibility remain deferred.
