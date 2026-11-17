<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M69: two independently demanded calibrated depth cameras

Budget: at most five working days including review and all mandatory gates.
M1–M68 and 201 reviewed slices are accepted. M68 commit
`24ea62519d0a671f789e6b3fc98bd2b72de18ae2`, tree
`10101d116ecbd694f528cb9989f1a0014794d718`, passed its full normal hook,
strict coverage, independent local DATA, [exact hosted CI](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/44077798140)
attempt 1, durable original retention and independent official DATA before the
ordinary Main promotion/readback. See [M68 acceptance](M68_ACCEPTANCE.md).
Both original hosted HTTP 503 failures remain retained; their snapshot
suite/component and provider cause remain unknown. The signed-input delivery
repair preserves canonical pins, signature verification and all SDK inputs.
M65 remains Latest with 35 installed examples; no distribution refresh is selected.

This isolated successor has the actual accepted repair parent. The first normal
on `733835b6` failed at the strict registry count, now corrected to 41. The
second normal on `b04a0c3b` genuinely failed after 2054.903 seconds: Python 498
and headless 2052 passed, while renderer tests had 1304 passes and one new
Busy-control failure. Full qualification and renderer coverage did not pass.
The existing standalone 41-example and RGB component outputs are retained
partial observations; they do not qualify this failed normal.

The new control incorrectly demanded a Busy error after `capture_all()` first
pumps its deliberately invalid worker. A real asynchronous error can precede
admission; the exact observed variant was not printed and remains unknown.
The bounded correction uses the accepted RGB control's ordering: require typed
Busy at the owner's actual admission boundary before pumping, then require its
error and the peer's exact publication, stamps and unchanged simulation time.
The same no-image, demand, retirement, lifetime and shutdown assertions remain.
Production code, fixtures, tolerances and accepted test bodies stay unchanged.
The complete correction has independent Source review. The one selected Rust
leaf formatter passed with all source bytes and modes unchanged. Bind the final
snapshot, census and procedure for one affected full normal; do not repeat the
formatter. No old result transfers; both failed trees and originals remain.

## Source overlap and ownership

Under D31–D32 this isolated checkout may author and review M69 while frozen M68
completes hosted and official qualification. This explicitly selects Source
overlap; it transfers no result or acceptance. Actual accepted M68 and complete
independent M69 Source review are prerequisites for every formatter, test, lint,
build, native operation, qualification or publication. Adopt and review any
predecessor correction before checks. Root alone integrates and releases heavy
work; frozen M68 and all old failures remain unchanged.

Light owns application pair intent and its private controls. Root owns separate
Server admission, CLI routing, qualification registries and current guidance.
Messages owns the fixture, worker integration, bounded canonical consumer proof
and feature guide. SDF independently reviews the contract and integration seams.
Finish implementation, regression Source and guides, batch all review findings,
then execute the single consolidated verification cycle. No per-edit checks.

## Closed interfaces

Add `CameraServer::new_depth_pair` and the separate
`Server::with_external_depth_camera_outputs(..., [u64; 2])`. Their complete loaded
selection must contain exactly two supported calibrated 64 by 64 depth cameras.
Existing `new_pair` and `with_external_camera_outputs` remain RGB-only; scalar
constructors and their old refusal controls remain unchanged. CLI selects the
new factory only for a configured homogeneous two-depth selection. Mixed
families, zero/one/three selections at the new API, duplicate/zero/foreign IDs,
invalid metadata and invalid transport keys fail before external effects.

Preserve explicit depth-pair intent through reset and two-to-one-to-zero owner
retirement. A surviving owner keeps its family identity and cannot enroll a new
camera. Validate complete identities, four output keys, namespace, rates and
publisher tokens before starting a family session or worker. Reuse the existing
scope-two session, per-owner scheduler and workers, single capture slot,
protocol 7, calibrated primitive depth math and canonical output messages.
No protocol, renderer, scheduling or transport framework change is selected.

## Required feature evidence

Use common-forward, separated-Y cameras with independent 10 and 20 Hz rates.
All visual oriented-bound corners must remain ahead of both near planes under
the unchanged conservative admission rule. A-only, B-only, dual-image and
information-only demand must prove laziness, independent clocks and sequences,
calibration and frame identity. Preserve full little-endian R_FLOAT32 metric
bytes and empty/far behavior. Compare against independently constructed expected
geometry and packets rather than production conversion helpers.

Actual worker integration must prove two owned worker identities, Busy-peer
progress without reticking, pause/reset/stale replies, owner removal and owned
shutdown. Record only observable exits; a discarded worker exit stays UNKNOWN.
Use two simultaneous independently owned canonical upstream consumers, each
retaining its actual wait/exit and complete image/calibration streams. Each keeps
stdout at 65,536 bytes, stderr at 4,096, Image at 32,768 and CameraInfo at 16,384.
The two-consumer stdout receipt bound is explicitly 131,072. Do not silently
widen old pair, IPC or proof limits.

## Acceptance and exclusions

After complete Source review and actual predecessor acceptance, the unchanged
full normal hook, strict workspace/per-crate/actual-parent coverage, independent
retained local DATA, exact hosted CI, durable originals, independent official
DATA and qualified Main promotion are all required. Source declarations are not
observed passes. M68 evidence does not qualify M69. Keep inherited tests,
thresholds, tolerances, assertions, notices and failures intact.

Exclude RGBD registration, point clouds, noise, dynamic registration, physics,
dependencies, SDK changes, new frameworks, worker/package distribution and
analytic, native Ogre, performance or physical GPU parity claims.
