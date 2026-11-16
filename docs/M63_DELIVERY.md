<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M63: two automatic RGB cameras

Current acceptance: M61–M64 passed local and exact hosted qualification,
independent local/official DATA and ordinary Main/readbacks at192ede. M1–M64/197
are accepted; M60 remains Latest/install33. See
[the actual acceptance record](M61_M64_ACCEPTANCE.md). The feature contract and
retained qualification checkpoints below describe their historical scope.

## Runnable scope

Extend the existing camera application and `--camera-worker` path to exactly two
loaded RGB cameras. Keep the ordinary headless Server renderer-free. Preserve
existing zero/single application semantics, including the one-depth-camera path.
The new pair path admits only two supported 64×64 RGB cameras; third cameras,
mixed RGB/depth and paired depth are refused before IO. Scene metadata and the
manual per-ID CameraOutput path already support multiple cameras; do not narrow
those APIs or label accepted IMU multiplicity a new feature.

Reuse the existing per-ID extraction, calibration, frame precedence, message
encoding, lazy renderer client, request/reply limits, worker protocol, shaders and
software renderer. No SDK, viewer, package or paused depth-worker feature is
selected. Each camera has one existing schedule, sequence and acquisition slot;
there are at most two demanded workers and two in-flight captures. This is not a
memory/performance or native Ogre/pixel-equivalence claim.

## Closed API and ownership

- Preserve `CameraServer::new`, scalar capture/pump/demand/update and
  `ScheduledCameraOutput::prepare` as zero/single APIs; their existing multiple
  camera refusal tests stay meaningful. Add `CameraServer::new_pair` requiring
  exactly two supported RGB owners. Scalar output methods refuse pair mode
  before side effects rather than silently selecting one owner.
- Add `capture_all` returning bounded, ID-bearing per-camera outcomes. A concrete
  `CameraCapture` contains `sensor_id` and a typed result with pumped/requested
  observations (`CameraFrameProgress`). Global closed/reset/identity failure is
  returned before any output; per-owner Busy or worker error is recorded while
  the other owner still gets one bounded pump/request. Keep no catch-up backlog
  and never repeat an already committed physics tick. Add per-ID pump and demand
  observation only as needed; keep the old scalar APIs unambiguous.
- Reuse a crate-private selected-ID scheduled constructor; do not expose a new
  unbounded scheduler or broaden paired depth. The old scheduled prepare checks
  zero/single count before calling the same constructor.
- Root adds `Server::with_external_camera_outputs` accepting `[u64; 2]` and exact
  complete-set admission. Retain the old singular API and headless refusal.
  Both IDs must be distinct, nonzero, actual Sensor entities and exactly the two
  current loaded RGB cameras under the configured world intent. Wrong IDs,
  missing/extra cameras, unsupported metadata or no intent fail closed.
- Pure whole-set preflight covers both identities/hierarchies, frames, calibration,
  operator settings and all four post-namespace/remap image/info keys before the
  first lookup/declaration. Reject same-role and cross-role aliases. This is not
  an atomic network-advertisement guarantee after an actual IO failure.
- The family owns one shared Session and writer scope of capacity two. Individual
  camera retirement must not close the peer's Session; the last family owner
  retires outputs before closing it. All GPU/IPC/setup/retire/drop work stays
  outside the Server's critical tick. Absent family retains no settings/session,
  scope/worker/declaration or callbacks.
- Validate the entire current camera set before pumping/requesting any member.
  Genuine removal retires only that owner; replacement/drift/unknown extra owner
  fails closed without publishing a peer's stale frame. Pending reset invalidates
  both old generations, full loaded reset restores original IDs/sequence/phase,
  and no in-flight old frame may escape. Existing CameraWorldEdit restrictions
  remain; use admitted Scene lifecycle test seams, not new camera CRUD.

CLI selection is automatic under the existing `rgb-camera` feature and explicit
worker option: configured exactly-two RGB intent uses the new pair constructor;
zero/single and metadata-only/no-intent paths keep their old behavior. Process
all per-camera outcomes after one tick; Busy cannot skip a healthy peer or retry
physics. Preserve the default native-free distribution and one-camera stderr.

## Ownership and acceptance

Root owns Server's narrow external-ID admission, CLI dispatch/regression Source,
shared finite qualifier/current guides/planning and integration. Light owns
camera-output application/scheduled modules, focused exports/error and their
private regressions in an isolated worktree. Messages owns the two-RGB fixture,
existing canonical subscriber's optional pair mode, new real software-worker
integration regression and guide appendix. SDF independently reviews completed
batches and shared integration. Read the retained camera contracts and pinned
upstream Source before edits; do not replay their successful original campaigns.

Planned controls: `two_loaded_rgb_owners_preflight_all_bindings_and_four_keys`,
`independent_image_info_demand_keeps_two_clocks_and_bounded_acquisitions`,
`busy_camera_does_not_starve_its_peer`,
`camera_pair_removal_and_loaded_reset_retire_only_owned_generations`,
`two_real_rgb_views_keep_pixels_stamps_sequences_and_owned_shutdown`, and
`cli_two_camera_path_preserves_single_and_headless_refusal`. Preserve all existing
controls. Exercise third/late-invalid owner and all four cross-key aliases before
IO, no-demand/info-only/single-image/two-image demand, distinct 10/20 Hz clocks,
paused/no-duplicate timestamps, independent actual pixel views and canonical
Image/CameraInfo dimensions/stride/frame/sequence/time, in-flight reset/removal,
last-owner absence and actual owned child exits. Synthetic rows are not receipts.

Complete implementation, regression Source, fixture/guide and independent review;
batch all findings before one consolidated formatter/verification cycle. Fresh
real existing software-worker and canonical four-route evidence is required;
old CPU metadata/clock evidence is reused only within unchanged scope. No new
SensorsManager/Ogre reference campaign is required by this RGB ownership change.
Full unchanged normal hook, strict workspace/per-crate/actual-parent coverage,
independent local DATA, exact hosted CI, official DATA and ordinary Main/readbacks
remain mandatory before acceptance/publication. Installed M60 stays33 examples.

## Existing worker diagnostics and removal evidence

The application may pass through the existing Client's read-only PID and
non-owning ClientLifetime using `camera_worker_diagnostics(sensor_id)`. This
starts no worker, advances no state and adds no telemetry/protocol/registry.
Real worker tests retain PID/starttime identity and require actual original
identity disappearance plus existing thread-lifetime retirement. A zombie is
not absence. The application's existing retirement discards the child exit
code; that code remains UNKNOWN, not a fabricated zero or independent wait
receipt. Actual canonical-peer child waits and exits are separate evidence.

Genuine camera-owner removal is exercised through an existing one-shot Scene
Finalize system that despawns the actual Sensor entity. Removing CameraSensor
from a still-live owner is drift and fails the whole pair preflight. The former
proves camera-output retirement while the model/body remain; it is not a new
camera CRUD endpoint, model-removal RPC or backend-body-removal claim. Full
loaded reset restores the original Sensor identity and both generations.

The renderer integration test names the existing ECS World type to use that
Finalize seam. Its crate adds only the already locked workspace `bevy_ecs` as a
dev-dependency and its corresponding lockfile edge; no package/version/checksum
or production dependency is added. The isolated Source qualifier appends M63 to
accepted M60's 33-example list (34 finite load/step examples). M61/M62's frozen
35-example candidate is unchanged; integration must reconcile the registries.
Finite stepping is not the required dual-image/software-worker proof.

## Two-view example and renderer integration (Source preparation)

`examples/m63_two_rgb_cameras.sdf` keeps the M5 opaque-box, zero-gravity,
64×64 RGB domain and its explicit bare world Sensors intent. Static `rig_a`
and `rig_b`, each with link `body` and sensor `camera`, view from `(0,-2,1)`
and `(0,2,1)` towards the red and blue targets at `(4,-2,1)` and `(4,2,1)`.
Their image/info pairs are `/m63/a/image`, `/m63/a/camera_info` and
`/m63/b/image`, `/m63/b/camera_info`; loaded frames are
`rig_a/body/camera` and `rig_b/body/camera`. Rates are 10 and 20 Hz. Calibration
retains horizontal FOV pi/2, near/far 0.1/10, RGB_INT8 and anti-aliasing 4.
Opaque directional markers retain camera +Y-left/+Z-up checks. No light,
shader, depth, rendering backend or worker protocol extension is introduced.

A build with the existing `rgb-camera` feature selects the pair automatically
under `--server --camera-worker /absolute/path/to/gz-render-worker`. The existing
`--run`, pause/step/reset and bounded Busy behavior apply. An output Busy result
never authorizes another physics tick. Fast or delayed rendering may coalesce
missed periods to one current acquisition; the CLI does not promise to retain
every elapsed camera period. The accepted M60 installed package is unchanged.

`crates/gz-render-worker/tests/two_rgb_cameras.rs` is a new integration target;
the existing automatic-camera tests and their bodies remain unchanged. It uses
the exact Cargo-bound existing software worker, the maintained network-isolation
helper, and the new pair application API. An existing workspace `bevy_ecs`
dev-dependency supports its admitted Finalize lifecycle system only.

The declared controls are:

- `two_real_rgb_views_keep_pixels_stamps_sequences_and_owned_shutdown`: both
  actual pixel views, all four canonical routes and complete calibration/header
  fields, independent first sequences and 10/20 Hz acquisition grids, paused
  silence, unchanged committed time during capture, and owned shutdown.
- `pair_info_only_then_independent_image_demand_starts_only_selected_workers`:
  no-demand and info-only create no worker; image demand starts only that camera,
  the second can start independently, and retiring the first preserves the second
  Session, worker, sequence and subsequent frame. Info-only output remains lazy.
- `pair_busy_reset_and_sensor_retirement_preserve_the_other_real_camera`: a held
  camera-A acquisition is Busy at a later due time while camera B still acquires;
  A's delivered frame retains its original timestamp. Both buffered images are
  cancelled before reset reconstruction. A one-shot admitted Scene Finalize
  system despawns the actual sensor entity, not just its metadata; its model and
  native body remain. Removing one owner cancels its held image while the peer
  completes, and removing the last releases output. Loaded reset restores original
  sensor IDs and fresh sequences. CameraWorldEdit remains explicitly refused.

The existing C++ camera subscriber retains its no-argument contract. Optional
`pair` mode receives exactly the first Image and CameraInfo from each new route,
checks actual dimensions, stride, pixel type, distinct centre colours, frame,
sequence, timestamp and all calibration arrays, and emits actual encoded bytes.
The Rust control independently joins those bytes to its real Zenoh receipts.
This reuses the selected `camera` client/SDK recipe and adds no registry entry or
native Sensors/Manager/rendering campaign. Its actual owned child wait and stderr
are retained separately from compiler provenance.

Worker retirement has an explicit observation limit. Tests obtain each real PID
from the existing Client diagnostic, record its `/proc/PID/stat` start identity,
and require that identity to disappear and the existing weak Client lifetime to
retire within five seconds. A zombie is not absence. The application discards
worker exit codes, so those codes remain **UNKNOWN**; no exit-zero or separate
raw reap receipt is invented. The C++ consumer's actual exit is checked separately.
The existing 20-second readback, one acquisition per owner, bounded channels and
five-second native cleanup remain in place. A new two-owner test watchdog bounds
each whole isolated case at 180 seconds; no old test timeout is changed.

All of these are Source declarations awaiting the single reviewed verification
cycle. They establish no native Ogre pixel equivalence, general camera support,
performance result, distribution refresh or M61/M62 qualification.

## Source-complete checkpoint

All 22 intended implementation, regression Source, fixture, guide and shared
integration paths passed independent whole Source review. The private pause API
and C++ main-dispatch findings were corrected before any execution. One
consolidated `cargo fmt --all` exited zero; it is not product verification.

The real software-worker/canonical tests remain unrun, as do Rust/Python product
checks, strict coverage, local DATA, exact hosted CI, official DATA and acceptance.
Host SSD free space was 8,708,616,192 bytes, below the unchanged 14 GiB initial
normal guard; other operating floors held. The practical target remains 35 GiB
free, and the exact six-target recovery still needs explicit owner approval.
No cache deletion, heavy job, native capture, SDK, viewer or package rebuild ran.
