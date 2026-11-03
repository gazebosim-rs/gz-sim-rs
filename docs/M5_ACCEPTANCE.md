<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M5: bounded live RGB camera delivery

Accepted and published on 2026-11-03 at simulator Main
`60a3a71096e5eff19c0e09392dc5b9c0348ae350`, tree
`d86a448566eaa11e50bd519a5b83ce965970ae9c`. The final normal staged gate,
[exact hosted CI 41703668173](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41703668173)
and independent source, local and official retained-data reviews passed.
M1–M4 and viewer `5b67f45f` remain preserved and unchanged.
The application contract is documented in `docs/M5_RGB_CAMERA.md`.

## Delivered application boundary

The opt-in CLI feature `rgb-camera` connects one loaded SDF camera to the existing
CameraServer, bounded render-worker IPC, offscreen Bevy renderer and canonical
upstream Zenoh/Prost image delivery. Run with `--server --camera-worker` and an
operator-trusted absolute worker path. The admitted domain is 64×64 RGB_INT8,
calibrated opaque box visuals and no world lights. The CPU CLI can be built with
`--no-default-features --features rgb-camera`; rendering remains in the separate
GPU-enabled worker.

Without the feature, the flag is explicitly refused before world/resource IO.
Without a configured camera, the ordinary headless path leaves the unused worker
untouched. With a configured camera and no image demand, the renderer stays lazy.
The CLI camera peer uses no listening endpoint, preserving the world Server's
configured listener and outgoing camera connect/scouting configuration. Existing
programmatic CameraServer construction and unwind ordering remain intact.

Camera tick and capture are separated so output Busy cannot repeat committed
physics. The CLI reports a backpressure transition once and continues its finite
tick budget. If spawning the configured worker executable fails, its original IO error is
published before request admission closes, allowing capture failure to terminate
the run truthfully.
No transport replacement, sensor runtime or Messages implementation is added.

## Sensor, wire and lifecycle evidence

Four passing real-worker controls use the exact loaded `m5-rgb.sdf` fixture and
actual upstream Zenoh subscribers with maintained Prost decoding:

- Known red/green/blue pixels and bounded box footprint, RGB format, 64×64 size,
  192-byte stride, canonical Image/CameraInfo attachments, frame and sequence;
  calibration matrices retain their existing `1e-12` comparison.
- Exact 10 Hz acquisition timestamps at 1, 100 and 200 ms, then paused silence.
- Busy after a committed tick without another physics advance; original capture
  time retained; reset discards buffered pixels, requires IO preparation and
  restarts sequence zero in the new local epoch. Epoch is not invented as a
  canonical image wire field.
- Camera model removal cancels pending output and retires its resources. A real
  CLI plus real worker publishes an image across pause/resume and exits with
  exactly 10,000 advancing ticks and 10 seconds of simulation time.

Eight CPU CLI controls separately cover flag/admission errors, no configured
camera, lazy no-interest behavior, default network configuration, fixed listener
release and viewer-setup cleanup. Actual image interest with a silent operator
worker proves bounded Busy reporting and advancing ticks without claiming an
image. A nonexistent worker proves the original asynchronous startup error
reaches the CLI, with no successful simulation summary. Process-group guards and
bounded watchdogs preserve failure cleanup.

## Required local checks and retained bindings

The completed normal pass took **1131.278 seconds**, exit zero, with no capacity
stop (minimum recorded filesystem free space 4,691,906,560 bytes). It records
**1028 headless, 637 renderer and 289 tooling passes**, zero failures, plus
formatting, Clippy and documentation checks. Independent local review confirmed
the retained packet; exact hosted CI and its official audit also passed.

| Separate owned profile | Local lines | Local regions |
| --- | --- | --- |
| Headless | 12119 / 12342 | 18443 / 19018 |
| Renderer | 7339 / 7485 | 10888 / 11320 |

Every required crate passes its separate >95% line and region gates; profiles
are not combined. Parent-relative changed executable lines pass at CLI **92/92**
in both profiles, plus camera-output **24/24** and render-client **1/1** in the
renderer profile. Existing strict/source/native and coverage policies remain
mandatory; no assertion, tolerance or denominator was weakened.

The unchanged feature-disabled native-off proof passes **10 cases**, with four
retained Cargo-bound ELFs and 14 raw profiles, including actual disabled-backend
return-line and executable-region hits. The distinct native-free RGB proof
passes **eight cases**, retains its CLI and test ELFs and 31 raw profiles, and
binds feature selection, source/inputs and dependency closure without native SDK
or GPU dependencies. These are auxiliary proofs, not a merged whole-source
native-free coverage claim.

The renderer evidence binds the CLI and worker actually used by the real image
test to build-only Cargo JSON and pre/post-execution SHA-256 values:
CLI `d066f08ebb6a48b9f13a9ae3a71e7c581321e626faf721aa9b8bb3c0993908ad`;
worker `7d5019bba7555133f8a709070c2473fe6ac1d8fc3893582add089e4630343652`.
Those full-profile executable records are metadata bindings; this record does
not claim retained copies of every full-profile ELF or test-temporary image.
The six auxiliary native-off ELFs are retained separately.

The packet records 2340 artifact hashes and 562 distinct Git source/input
bindings overall, including 78 production files; its profile index binds 272
inputs. Its authoritative local paths are:

- `artifacts/m5-delivery/staged-gate-contract-preserved/qualified-result.json`
  and `qualified-gate.log` in the primary checkout.
- M5 worktree `artifacts/staged/d86a448566eaa11e50bd519a5b83ce965970ae9c-yo8pjj0v/`:
  `stage-result.json` reports passed/complete with no missing artifacts;
  separate profile summaries, raw/merged coverage and native-reference evidence;
  and `rapier-runtime/` retains both auxiliary configurations.

## Review history and retained failures

Source and independent review preceded consolidated verification. Earlier
attempts remain failed evidence: initial API/error-conversion, formatting and
tooling-fixture mismatches; the 407-second camera-admission assertion failure;
the 786-second strict changed-coverage failure; and the 1010-second constructor
error/unwind contract failure. Corrections preserve exact unsupported-width
admission, separately exercise f64-to-f32 calibration refusal, add meaningful
default-network/backpressure/fatal controls, and preserve the original
CameraServer constructor contract while configuring the CLI's camera peer
without a listener. The final pass does not retroactively pass those attempts.

## Official hosted evidence and limits

Exact run `41703668173`, attempt 1, passed all sixteen steps at the accepted
feature SHA/tree. Independent review matched 1239 official Git blobs and modes
and 562 distinct source/input bindings. Official test counts are also
1028 headless, 637 renderer and 289 tooling, with the four real RGB worker
controls passing. Hosted headless totals are 12119/12342 lines and 18443/19018
regions; renderer totals are 7340/7485 and 10889/11320. The legitimate single
worker line/region difference from local counters is kept separate. Every owned
crate and parent-relative changed gate passes independently in both profiles.

Data-only recombination of 1744 headless, 337 renderer, 14 original native-off
and 31 RGB raw profiles reproduces all four official merged profiles byte for
byte. The official packet is retained at
`artifacts/m5-delivery/hosted-41703668173`. Its artifact ZIP SHA-256 is
`944853bf6ab29e11e98832b5a3523af51dba081550cc614889461fb18fa0aadd`;
source ZIP SHA-256 is
`862faef31dd8fed600d848b6c0d666ffe0f63c3ae82ad947ab10bbab7dc030ca`.
Hosted base-zero/full-source checks and parent-relative changed checks retain
their distinct scopes. Nine existing C++ peers and native provenance checks
remain qualified; this is not new C++ RGB image parity.

The full-profile CLI/worker ELF bindings are metadata records, not retained
binary copies. Six auxiliary native-free ELFs are retained. Actual RGB values
are established by passing real rendering/assertion controls; pixel buffers
were not saved. No M5 browser observation, browser RGB display, C++ image parity,
performance result or general GPU/backend qualification is claimed. Camera
arrays, other geometry/formats, distortion, thermal and segmentation remain
outside M5; metric-depth integration is the next named delivery M6.
