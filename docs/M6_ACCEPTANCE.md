<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M6: bounded live metric-depth camera delivery

Accepted and published on 2026-11-03 at simulator Main
`7fccd049f373aef5c2ea6c8feb41339ea955e62e`, tree
`6ece516d5de38a277783aaa2090e697b27e0317e`. The final normal staged gate,
[exact hosted CI 41726703723](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41726703723)
and independent source, local and official retained-data reviews passed.
M1–M5 and viewer `5b67f45f` remain preserved and unchanged.
The application contract is documented in `docs/M6_DEPTH_CAMERA.md`.

## Delivered application boundary

One loaded SDF `depth_camera` uses the existing CameraServer, accepted metric-depth
renderer, bounded worker IPC and canonical upstream Zenoh/Prost delivery. The
admitted domain remains calibrated 64×64 opaque boxes with no world lights.
One RGB or depth camera is admitted. The producer's accepted default camera image
metadata is retained; the depth sensor kind selects the output, without inventing
support for a floating-point SDF input format.

The existing opt-in CLI feature name `rgb-camera` and `--server --camera-worker`
interface are retained. The worker path is an operator-trusted absolute path;
the CPU CLI can use `--no-default-features --features rgb-camera`, with rendering
in the separate GPU-enabled worker. The existing absence and demand paths, split
tick/capture, listener ownership and retirement order remain. Busy does not repeat
committed physics. No additional transport, sensor scheduler or Messages runtime
is introduced.

The focused preserved worker extension is adopted onto accepted M5. Private IPC
version 3 adds `CalibratedDepthSnapshot` and typed `DepthFrame` little-endian f32
bytes. Each capture retains its admitted kind through cancellation, replacement
and retirement; valid but wrong-kind replies are rejected. Older private protocol
versions fail explicitly. The renderer itself was already accepted through
`386482e5`; its production code is unchanged in M6.

## Sensor, wire and lifecycle evidence

Five passing real-worker controls use the loaded `m6-depth.sdf` fixture and actual
upstream Zenoh consumers with maintained Prost decoding:

- A nonidentity rotated camera and box yield a known 4-metre planar depth at
  central and off-axis pixels within the existing `1e-4` metre f32 bound.
  Background corners retain positive infinity. Canonical Image is `R_FLOAT32`
  (13), 64×64, stride 256 and 16,384 bytes in upper-left row-major order.
  CameraInfo K/P retain the existing `1e-12` comparison, exact frame and stamp.
- Exact 10 Hz acquisition timestamps at 1, 100 and 200 ms, then paused silence.
- Busy after one committed tick without another physics advance; the original
  capture timestamp survives delayed completion. Reset discards buffered depth,
  requires IO preparation and restarts sequence zero in the new local epoch,
  restoring the same static scene bytes. Epoch is not invented as an Image field.
- Actual model-removal RPC cancels buffered output and releases camera ownership.
  A box wholly beyond the far plane yields only positive infinity; a near-plane
  intersection returns the existing typed InvalidSnapshot error and no image.
- A Cargo-bound real CLI and worker publish depth across pause/resume, then exit
  with exactly 10,000 advancing ticks and 10 seconds of simulation time. The
  existing namespace, process-group and timeout controls bound cleanup.

All four accepted real RGB worker controls also pass. CPU message controls prove
raw little-endian words, infinity, canonical headers and Prost roundtrip, and
reject invalid lengths, NaNs, signed zero, negative values and timestamp overflow.
Protocol/client/worker controls retain exact kind, request/stamp, legacy-version,
near-admission atomicity and bidirectional in-flight replacement assertions.

This renderer conservatively refuses any box with a corner at or before the
near plane, including geometry wholly behind the camera. Background/far values
are positive infinity. These are documented bounded semantics, without a native
near-range negative-infinity claim. No image value is reconstructed from physics
or substituted for an actual renderer readback.

## Required local checks and retained bindings

The completed normal pass took **1138.283 seconds**, exit zero, without a capacity
stop; minimum recorded filesystem free space was 3,469,410,304 bytes. It records
**1029 headless, 663 renderer and 289 tooling passes**, zero failures, plus
formatting, Clippy and documentation checks. The logs separately record five
headless and four renderer ignored helper entries; these are the existing
delegated build-script/source-audit scopes, not ignored depth cases. Independent
local review confirmed the retained packet; exact hosted CI and its official
audit also passed.

| Separate owned profile | Local lines | Local regions |
| --- | --- | --- |
| Headless | 12120 / 12343 | 18445 / 19020 |
| Renderer | 7503 / 7650 | 11102 / 11539 |

Every required crate passes its separate >95% line and region gates. CLI remains
independently covered in both profiles at **1069/1089 lines, 1633/1709 regions**;
the profiles are not combined. Parent-relative changed executable lines pass:
Scene **3/3** in both profiles, plus camera-output **42/44**, render-client
**18/18**, protocol **14/14**, render-scene **30/30** and worker **98/98** in the
renderer profile. Unchanged renderer production receives no new changed-source
credit. No coverage denominator, threshold, assertion or old tolerance is relaxed.

The unchanged feature-disabled native-off proof passes **10 cases**, with four
retained Cargo-bound ELFs and 14 raw profiles, including actual disabled-backend
return-line and executable-region hits. The distinct native-free camera proof
passes **nine cases**, retains its CLI and test ELFs and 33 raw profiles. It
includes depth no-interest/lazy admission alongside retained RGB/error/lifecycle
controls and binds the feature/dependency closure without native SDK or GPU
dependencies. These auxiliary proofs are not whole-source native-free coverage.

The renderer evidence binds the actual CLI and worker to build-only Cargo JSON
and pre/post-execution SHA-256 values:
CLI `9099c68b15df9428fd857f012e32487da8e6369ed3747223f981867129ab0db3`;
worker `554a3daa0d5faecec7b6ca43908b69fbe43c8a92e187e6e1569f01f5cd6a488c`.
Those full-profile executable records are metadata bindings, not retained copies
of every ELF. The six auxiliary native-off ELFs are retained separately.
Test-temporary depth images are not claimed as archived pixel blobs.

The packet records **2365 artifact hashes, 78 production source bindings and
272 profile-index input bindings**. Across the profile index and auxiliary proof,
there are 566 distinct source/input paths. Full-profile raw retention contains
1746 headless and 354 renderer profiles with their separate merged coverage.
Authoritative local paths are:

- Primary `artifacts/m6-delivery/staged-gate-support-corrected/qualified-result.json`
  and `qualified-gate.log`.
- M6 worktree
  `artifacts/staged/6ece516d5de38a277783aaa2090e697b27e0317e-d74flhyz/`:
  `stage-result.json` reports passed/complete, with separate profile summaries,
  source/input and raw/merged coverage records, native-reference evidence and
  `rapier-runtime/` auxiliary configurations.

## Review history and retained failures

Source and independent review preceded consolidated verification. The first
1013.249-second gate failed in the renderer support harness: 9 of its 34 cases
passed and 25 failed on stale successful version-2 handshake assertions or three
manually framed version-2 fixtures. Headless and auxiliary precursors passed,
but that attempt has no completed renderer/whole-gate acceptance.

The corrected four-path support batch uses the current protocol constant for
all 16 successful handshake assertions and typed current-version fixture frames.
The closed-input fixture still closes stdin before publishing Ready; original
delay, flush, stall, cancellation, backpressure, ownership and deadline assertions
are preserved. Intentional bad/legacy-version refusal controls are unchanged.
The passing continuation does not retroactively pass the first attempt or the
historical paused worker candidate.

## Official hosted evidence and limits

Exact run `41726703723`, attempt 1, passed all sixteen steps. Independent audit
matched all 1248 official Git blobs/modes and 566 distinct source/input bindings.
Hosted results are 1029 headless, 663 renderer and 289 tooling passes, including
five real depth, four retained RGB and 34 support-worker controls. Owned and
parent-relative changed coverage matches the local table above in each separate
profile; no profile union or new renderer changed-source credit is claimed.

The official packet is retained at `artifacts/m6-delivery/hosted-41726703723`.
Its artifact ZIP SHA-256 is
`e881f555d1b8ec87629d82ae6c45a0d809cc5127038bf95c29e56da63a4a8889`;
source ZIP SHA-256 is
`9ced38fe8346b49c21341a37eaaa8f426b9abf65e5ee2c18200e18f00dc145d0`.
Data-only recombination reproduces all four official merged profiles byte for
byte from 1746 headless, 358 renderer, 14 original native-off and 33 camera raw
profiles. The hosted renderer raw count differs from local 354 and remains
reported separately. Nine existing C++ peer provenance checks pass; hosted SDK
DSOs remain metadata-bound. This adds no C++ depth-image parity claim.

There are 285 full-profile executable metadata records, including prepared
CLI/worker identities; the original full-profile binaries are not retained.
Six auxiliary native-free ELF copies are retained and rehashed. Actual depth
and RGB values are established by passing real rendering/assertion controls;
pixel buffers were not saved. No browser depth display, performance result or
general GPU/backend qualification is claimed. Near-plane limitations remain
explicit. RGB-D registration, point clouds and depth-noise parity remain deferred;
M7 CPU ray lidar is the next named delivery.
