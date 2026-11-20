<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M76: explicit camera projection offsets in CameraInfo

Selected before implementation under D30–D32 on November 19, 2026 (local).
Budget: at most five working days including review and mandatory gates. Base is
accepted Main `616cc3e2105a7ef9479ac823a6b05e832a92defc`. M70–M75 individual
acceptance and the M70 public Latest release are complete; their exact proof
records remain under the primary artifacts/m75-delivery. M70 installed payload
is still exact `7094a318` with 41 examples; this Source cycle adds no distribution.

Use only this isolated successor. Preserve all qualified/frozen trees, indices,
packages, archives, original failures, pending worktrees and Fuel ownership.

## Behavior and dependencies

Admit finite, signed explicit SDF `lens.projection.tx` and `ty` for the existing
64 by 64 RGB/depth domain. Publish their exact f64 values in canonical CameraInfo
`P[3]` and `P[7]`. They affect metadata only. Explicit P still selects rendering
focal lengths/principal point with zero skew; absent P retains selected K/HFOV
and zero offsets. Selected K/P retain their existing finite, positive focal and
render-representability guards. Dormant values behind absent presence flags stay
ignored. Finite metadata offsets need no f32 representability restriction.

The local pinned gz-sensors checkout is
`8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`:
`src/CameraSensor.cc:788–864` expressly ignores tx/ty when building rendering
projection; `:673–719` publishes them in PopulateInfo. This is source evidence,
not newly executed image/native proof. Retain the existing CPU-only
PopulateInfo/PublishInfo harness; no Ogre or new renderer prerequisite is needed.
Do not change `gz-render-frame::Calibration::new`: direct worker requests with
nonzero offsets remain refused.

## File ownership

All paths below are relative to the new successor; they are leases for this isolated candidate; the qualified base stays frozen.

| Owner | Files | Responsibility |
| --- | --- | --- |
| Root/admission | `crates/gz-sim-scene/src/lib.rs`; `crates/gz-sim-scene/tests/camera_metadata.rs` | Remove only selected finite-offset zero restrictions; parsed presence, dormant-invalid and loaded reset controls. |
| Calibration/output | `crates/gz-render-scene/src/lib.rs`; `crates/gz-render-scene/tests/camera_extraction.rs`; `crates/gz-camera-output/src/lib.rs`; `crates/gz-camera-output/src/camera_messages.rs`; `crates/gz-camera-output/tests/private/camera_messages.rs` | Separate metadata offsets from render Calibration, validate before IO, pass metadata to an additive full-P message constructor. Existing message APIs delegate with zero offsets and preserve errors/bytes. |
| Evidence | New `crates/gz-camera-output/tests/private/projection_offsets.rs`; existing `tests/private/explicit_intrinsics.rs` and `projection.rs` under that crate for transitioned selected nonfinite refusals; `crates/gz-render-worker/tests/explicit_intrinsics.rs` for one module attachment and tx-only/ty-only drift callback arms; `crates/gz-camera-output/tests/fixtures/native_camera_info.cc`; `capture-camera-info.py` beside it; genuinely captured `native_camera_info_projection_offsets.bin` and its provenance | Actual worker/canonical/lifecycle controls; add a nonzero-offset CPU-native mode without changing the old zero-offset invocation, fixture or oracle. The capture wrapper gains failure-only signal cleanup of its owned child and an optional actual-exit receipt; no harness/framework redesign. |
| Root/registry | New `examples/m76_camera_projection_offsets.sdf`; `crates/gz-sim-cli/tests/rgb_camera.rs`; `scripts/qualify-rapier-runtime.py`; `tests/test_rapier_runtime_coverage.py`; `tests/test_rapier_runtime_retention.py` for the two demonstrated positive-fixture census corrections | One finite source example and exact CLI case; complete closed names/outcomes registries and guards. Source example counts remain distinct from installed M70's 41 examples. |
| Root/docs | New `docs/M76_DELIVERY.md`; `docs/DECISIONS.md`; `docs/DELIVERY_PLAN.md`; `README.md`; `AGENTS.md`; `docs/M70_INSTALL.md`; `docs/M70_CAPABILITIES.md`; `docs/M70_PREVIEW.md`; `planning/delivery-phases.json`; the existing guide control in `tests/test_preview_package.py` | Selection, boundaries and actual qualification state; maintained M70 corrections below; preserve the exact pre-closeout product M70 record as historical metadata; current tracker updated only from observed state. |
| Independent reviewer | Read entire completed diff and affected callers/tests/docs; no overlapping implementation lease | Audit metadata/render separation, each old refusal transition, error order, exact compatibility and complete acceptance coverage before checks. |

Production scope is exactly the four files in admission/calibration/output rows.
Any additional production dependency requires Root review before expansion.
Registry/module plumbing is bounded to the files named above. Preserve old
negative assertions by transitioning only now-admitted finite-offset cases and
keeping equivalent selected nonfinite refusal controls. The parsed loader
refusal keeps a finite nonpositive projection focal value because the pinned
SDF parser rejects nonfinite scalar text before constructing sensor metadata.

## Acceptance

1. Exercise all four K/P presence combinations for RGB and depth. Check signed,
   zero and large finite offsets exactly in metadata; absent-P dormant invalid
   values are ignored. Selected NaN/+INF/-INF in each offset fails before
   publisher/worker IO, including info-only and CLI admission paths.
2. Compare all four calibrated request families (RGB/depth, boxes/shapes) against
   their zero-offset baseline: unchanged selected render calibration, pose,
   geometry and encoded request bytes. Preserve all old zero-offset API and
   canonical wire bytes, complete matrices, header, stamp and error ordering.
   Direct worker nonzero-offset refusal remains unchanged and covered.
3. Use the real software worker to compare complete RGB and metric-depth frame
   bytes with zero-offset controls at identical acquisition state. Require
   canonical CameraInfo to differ only at P[3]/P[7]; assert all 4,096 depth
   values/background, image header/time/sequence and RGB bytes. Reuse existing
   oracles and bounds. The CPU-native harness must independently publish signed
   offsets and complete CameraInfo; keep old captured originals untouched.
4. Preserve info-only/no-demand worker laziness, per-owner cadence, metadata
   drift retirement (including tx-only and ty-only changes), loaded reset of
   original offsets/IDs/sequence, pause, selective removal and M75 masks. Retain
   all existing scalar, homogeneous-pair and mixed-pair bounds.
5. Integrate maintained-guide corrections from
   `artifacts/m75-delivery/m70-public-documentation-overlay-held`: public archive
   filename remains `gz-preview-m70-linux-x86_64.tar.gz`, but extracted root is
   `gz-preview-m70-websocket-close-handshake-provisional`; ellipsoid RGB uses the
   observed native paused server/packaged worker route, not unsupported web
   Ellipsoid export. Correct corresponding M70_PREVIEW wording and stale current
   checkpoint prose using actual Root closeout evidence. Preserve historical
   checkpoint/failure evidence and package bytes; do not retroactively rebuild
   or relabel the M70 payload as M71–M76.
6. Complete implementation, meaningful regressions, docs and independent whole
   source review; batch all findings/fixes before one consolidated ordinary
   normal/strict verification cycle. No per-edit test/lint/probe rounds. Keep
   required final staged hook, coverage gates, retained-evidence audit, exact
   hosted CI, official evidence and Main gates before acceptance/publication.
   Inspect and batch any real failures before affected reruns; retain originals.

## Boundaries and schedule

No physical stereo, baseline inference/SetBaseline, paired-camera pose coupling,
new poses, count/resolution expansion, RGBD, distortion, noise, point clouds,
matrix/ray/frustum/shader changes, protocol/schema/worker/framework changes,
dependency/SDK/backend work, native Ogre parity, package rebuild or new browser
feature is selected. Rapier f64 and TPE remain; DART/MuJoCo stay retired.

Days 1–2: source, regression controls and maintained guides; day 3: independent
whole-source review and one correction batch; days 4–5: consolidated mandatory
qualification, independent evidence review and actual hosted/official closeout.
If a real prerequisite threatens the five-day bound, report it and re-scope
explicitly without weakening gates or claiming partial evidence as acceptance.


## Native reference checkpoint

The single reviewed endcycle CPU capture returned actual Root 0 in
20.047634856 seconds. All five command stages and the actual native child
returned 0; the child was reaped and its PID and group were absent. The new
325-byte wire is SHA-256
`a84edd3c39ff29cb2deffc4aa304b3ce4fc53dd0fc6fcfad3462b5559f211e31`.
The checked-in provenance beside the fixture records actual commands, tool and
library hashes and retained execution records. The old 325-byte zero-offset
fixture was not recaptured. Independent retained native DATA passed without
replay; its report is pinned in the checked-in provenance. Full ordinary
normal/strict, hosted and official qualification remain required; this checkpoint
claims only CPU CameraInfo reference capture and retained native DATA.

## First normal failure and bounded correction

The first full normal returned Root/Git 1 after 100.359962 seconds at one
workspace Clippy type-complexity error in the new test helper return type.
This isolated correction factors that identical tuple into a named type; test
bodies, assertions and all production code remain unchanged. Product tests and
coverage were not reached. Preserve the failed tree and logs, reuse the unchanged
native reference evidence, and run the full affected normal hook after Source
review; no standalone Clippy or native retry is selected.

## Second normal failure and retained-fixture correction

The second full normal returned Root/Git 1 after 277.707559 seconds. Both
lint stages passed; all 481 Python checks ran with one error and one failure.
Two retained test fixtures still expected the prior 20-case CLI summary and
46-example registry. This isolated batch changes those positive expectations to
21 and 47, preserving every negative assertion and runtime/native byte. Full
Rust profiles, coverage and the auxiliary runtime qualifier were not reached.
Review the complete correction before the full affected hook retry; no standalone
Python test, lint, native capture or accepted evidence replay is selected.
