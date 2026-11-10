# Current delivery: M43 and M44

M1–M42 are accepted at qualified Main `73e03b9f741689b1538898bde96bf975a49f068e`.
M41 constant-wind AirSpeed and M42 joint-force feedback passed the complete
normal hook, strict coverage, exact hosted CI 42808865907 attempt 1 and
independent local/official DATA, followed by normal Main push and readback.
M40 release 440119258 (`m40-preview`) remains the latest installed preview.

M43 adds named IMU orientation references; M44 refreshes the preview with
eighteen examples and the installed joint-command helper. Source work proceeds
in parallel with explicit leases; Root owns shared integration and serial heavy
operations. Neither phase is accepted. Complete implementation, regression
source, guides and independent review before consolidated checks. Fresh native,
Rust, distribution, clean installation and browser proofs remain required. Each
phase is bounded at five working days including review and mandatory checks.

Read [M41/M42 acceptance](M41_M42_ACCEPTANCE.md) and the [M43/M44 contract](M43_M44_DELIVERY.md).

## Historical planning and closeout checkpoints (superseded)

# Current closeout and next source preparation

M1–M38 remain accepted; 42 phases are selected including the two next source-only lanes. M39/M40 complete normal, independent local DATA, exact review CI42778327635 and fresh package/install/browser DATA have passed. Independent official DATA and Main/tag/release/Latest readbacks remain pending; neither M39/M40 nor M41/M42 is accepted. M37 remains installed Latest.

The inherited earlier current-state blocks below are historical checkpoints; their outstanding gates remain requirements until actual completion. The new M41/M42 contract and source-only boundary above them govern this worktree.

## Next source preparation: M41 and M42

[The reviewed-before-code contract](M41_M42_DELIVERY.md) selects constant-wind
AirSpeed and a joint-force/JointState feedback example as disjoint source lanes.
M39/M40 closeout remains the priority; neither next phase inherits acceptance.
Each is at most five working days including review and all required gates.
Actual accepted-parent binding precedes next-wave native or consolidated checks.

<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

## Current delivery: M39 translation offsets and M40 preview

M1–M38 are accepted at qualified Main `00382fbc`; [M38 acceptance](M38_ACCEPTANCE.md)
records actual normal, exact hosted CI, independent DATA and Main readback.
M37 release 439888006 remains installed Latest. Forty phases are selected and
38 are accepted.

[Atomic M39/M40](M39_M40_DELIVERY.md) adds finite local/global XYZ translations
to existing noiseless odometry in both dimensional modes, and refreshes the
native-free Ubuntu preview with fifteen finite installed examples. Feature,
canonical/native and packaging Source lanes are independently reviewed and
integrated. Fresh scoped component capture and native/literal DATA passed;
new Rust, normal/strict/exact hosted and distribution qualification remain
required. Budget: at most five working days per phase including review and
mandatory gates. Root alone releases serial heavy checks. Neither phase is
accepted until every feature, package, install, browser/shutdown and publication
criterion passes. Continue the next useful bounded increment after acceptance.

The following earlier current-state statements are historical and superseded.

<!-- Copyright (C) 2026 Open Source Robotics Foundation -->

## Historical M36/M37 qualification checkpoint (superseded)

M1–M35 are accepted at qualified Main `1982ce86`; M32 remains installed Latest.
M34/M35's full normal, exact hosted CI42715967423 attempt1, independent
native/local/official DATA and exact Main promotion are recorded in
[M34_M35_ACCEPTANCE.md](M34_M35_ACCEPTANCE.md).

[M36/M37 are one atomic delivery](M36_M37_DELIVERY.md) over that accepted parent.
Both complete functional/package source batches, meaningful regressions and
guides have independent Source review. A new combined candidate must pass the
full normal hook, strict workspace/per-crate/actual-parent coverage, exact hosted
CI and independent local/official DATA. Fresh source-bound release inputs,
notices/ELF/archive/relocation DATA, clean Ubuntu nonroot installation with all
thirteen examples, actual default-robot browser controls and owned shutdown,
and Main/tag/assets/body/Latest readbacks remain required before either phase
is accepted. No M36 or M37 acceptance is claimed.

This supersedes only the intermediate M36-acceptance-before-M37 sequence. The
already-running M36-only normal gate may finish and is retained as provisional
historical evidence; it cannot qualify the combined candidate. Every individual
M36 canonical/physics-feedback/lifecycle criterion and M37 distribution criterion
is preserved. Root releases serial heavy operations after complete Source and
procedure review. No old gate, test, installation or browser success transfers.

<!-- SPDX-License-Identifier: Apache-2.0 -->


# Weekly delivery plan

## Historical M33 pending checkpoint (superseded)

Prior qualification: [M33 mobile-robot IMU/lidar coexistence](M33_DELIVERY.md),
locally qualified at `17dd62f2` with exact hosted/official/Main acceptance pending.
M1–M32 remain accepted, and M32 stays installed Latest.

[Selected M34/M35 contracts and leases](M34_M35_DELIVERY.md) open disjoint source
preparation during the remote qualification wait. Each scope is at most five
working days including review and mandatory checks. Detailed native/lifecycle
and typed-interface review precedes code release; accepted-parent binding
precedes native execution or full consolidated verification. No phase inherits
acceptance from another or changes M32's package.

> M1–M32 are accepted at qualified Main `679c9990`; see
> [M32 acceptance](M32_ACCEPTANCE.md) and the verified Latest
> [m32-preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m32-preview).
> Current work is [M33 mobile-robot IMU/lidar coexistence](M33_DELIVERY.md).
> New source, functional proofs and mandatory gates remain required.

## Retained delivery policy

Owner decision D30 (2026-11-02) replaces the previous full-rewrite-first sequence.
Ship a useful MVP first, then small runnable releases. The initial planning
update left implementation paused; the owner subsequently explicitly resumed
M1 implementation and its continuation automation.

## Schedule and acceptance

M1–M6 were accepted on 2026-11-03, M7–M9 on 2026-11-04 and M10–M11 on
2026-11-05 UTC; see [M1 evidence](M1_ACCEPTANCE.md),
[M2 evidence](M2_ACCEPTANCE.md), [M3 evidence](M3_ACCEPTANCE.md),
[M4 evidence](M4_ACCEPTANCE.md), [M5 evidence](M5_ACCEPTANCE.md) and
[M6 evidence](M6_ACCEPTANCE.md), [M7 evidence](M7_ACCEPTANCE.md) and
[M8 evidence](M8_ACCEPTANCE.md), [M9 evidence](M9_ACCEPTANCE.md) and
[M10 evidence](M10_ACCEPTANCE.md) and [M11 evidence](M11_ACCEPTANCE.md). M12 was accepted on 2026-11-07 UTC after final qualification, clean installation and actual preview publication; see [M12 evidence](M12_ACCEPTANCE.md). Its original **at most five working days** scope/budget is retained. Each later named increment is
also at most five working days, including source review, fixes, documentation,
mandatory staged checks and hosted CI. These are scope budgets and planning
targets, not guaranteed completion dates. No clock runs while paused. Re-estimate after a real integration checkpoint; report any risk immediately.
If work exceeds a budget, split unfinished optional scope into a new named
phase. Never label a broken mandatory exit criterion accepted to meet a date.
Do not carry a broad family indefinitely as one weekly phase.

Every phase ships a runnable increment of the previously accepted subset. Keep
retained assertions, numerical tolerances, per-crate/workspace >95% line and
region coverage, independent review, staged hooks and required CI. Complete the
bounded feature and source review first, then run one consolidated verification
pass. Reuse unchanged evidence where current checks permit it. New C++ parity
work is required when a changed retained contract needs it, not a new campaign
to complete every backend before a primitive MVP can ship. Rapier-versus-DART
results must never be described as same-engine performance improvement.

## Delivery sequence

| Phase | Budget | Runnable result | Acceptance boundary |
| --- | --- | --- | --- |
| M1: Runnable primitive MVP | 5 working days | One local SDF world, Rapier f64 box/sphere dynamics and static box ground; live gzweb-rs scene and poses; pause, resume and reset. | On a clean supported Linux setup, documented launch commands start the Rapier-only runtime and gzweb-rs. The real browser shows at least 20 moving primitive bodies and correct transforms; pause/resume/reset, reconnect and clean shutdown work. No C++ engine or GPU is required by the server. |
| M2: Local mesh visualization | 3 working days | Render local glTF 2.0 mesh visuals in gzweb-rs; keep primitive collisions. | A moving body with a local mesh visual has correct scale, orientation and resource paths in the actual browser; missing or invalid assets fail clearly. |
| M3: Mesh collisions | 5 working days | Rapier static triangle-mesh and dynamic convex-hull collision from the admitted local mesh input. | Static mesh contact and one dynamic convex hull pass geometry/contact and reset tests; unsupported nonconvex dynamic input is explicitly rejected. |
| M4: IMU sensor model | 3 working days | Wire the existing IMU model through loaded SDF demand and the live Rapier world; expose its canonical messages. | One configured IMU publishes correct units, frame, sample count and timestamps; pause/reset and last-consumer teardown pass. A no-IMU world owns no IMU callbacks or buffers. |
| M5: RGB camera model | 5 working days | Activate one SDF RGB camera using existing offscreen Bevy rendering and bounded image delivery. | A known primitive scene produces real timestamped images at the requested rate; frame, pause/reset, removal and overload controls pass. |
| M6: Depth camera model | 5 working days | Connect the existing metric-depth renderer to one loaded SDF camera and a bounded consumer stream. | A calibrated primitive scene yields metric depths and documented background/clipping behavior; reset epochs, removal and bounded delivery pass. |
| M7: CPU ray lidar model | 5 working days | One planar ray lidar using Rapier queries against admitted primitive and mesh collisions. | Analytic range fixtures, min/max range, frame, acquisition timestamps and pause/reset pass; visualization can inspect the scan. |
| M8: Differential-drive system | 5 working days | One two-wheel Rapier robot, velocity command and odometry, visible in gzweb-rs. | A simple robot drives and turns from real commands; wheel limits, odometry frames, pause/reset and absent-system teardown pass. |
| M9: Joint controller systems | 5 working days | One admitted Rapier revolute chain with position and velocity controllers. | Commands move the displayed joints with documented limits and stable IDs; bad commands, reset and removal pass. |
| M10: Interactive world lifecycle | 5 working days | Expose focused spawn/remove and entity-state operations to the live browser for the admitted MVP scene. | Spawn and remove primitives and one local mesh without stale IDs or visuals; rejected requests are atomic; reconnect obtains a current snapshot. |
| M11: Focused recording and replay | 5 working days | Record and replay the admitted scene, poses and controls for one local session. | A short recorded session reproduces its timestamps and entity lifecycle in gzweb-rs; malformed input and bounded buffering pass. |
| M12: Installable integrated preview | 5 working days | Package the accepted subset and document supported capabilities, setup and a runnable robot demo. | A clean-machine installation runs the integrated demo; required coverage, CI, licenses and retained regressions pass at the release commit, with a published limitations matrix. |
| M13: Ideal altimeter | 5 working days | One loaded noise-free Rapier altimeter with canonical publication. | Native/wire/offset-point velocity, cadence, absence, pause/reset/removal and stale-generation controls; all gates. |
| M14: Local STL meshes | 5 working days | Local ASCII/binary STL collision and immutable GLB visuals using the accepted viewer. | Bounded admission, geometry/scale/contact/lifecycle controls and actual browser evidence; all gates. |
| M15: Ideal air pressure | 5 working days | One loaded ideal Rapier pressure sensor with canonical FluidPressure publication. | Actual native geopotential model, absolute sensor height, wire/frame/time/cadence/demand/reset/removal and atomic-refusal controls; all gates. |
| M16: Local OBJ meshes | 5 working days | Bounded local triangle OBJ collision and immutable GLB visuals using the accepted viewer. | Maintained parser, bounded/index/resource admission, geometry/contact/lifecycle/immutable serving and actual browser proof; all gates. |
| M17: Ideal magnetometer | 5 working days | One loaded ideal Rapier magnetometer with a finite constant world field and canonical Tesla publication. | Native/model and resolved-pose vectors, frame/time/sequence/cadence/demand/pause/reset/removal, atomic refusals and canonical wire; all gates. |
| M18: Joint-state publication | 5 working days | Actual single-axis Rapier joint position/velocity and supported metadata in canonical Model packets. | Native packets and backend components, topic/filter/order, equal-time/pause/reset cadence, lifecycle/endpoint and moving-joint wire controls; all gates. |
| M19: Refreshed installed preview | 5 working days | Ubuntu 24.04 native-free archive through M18, copied local OBJ robot, unchanged viewer and a nonconflicting explicit loopback HTTP port. | Fresh release binaries and notices, actual manifest/archive/relocation, clean non-root supported installation and real installed robot/browser controls and shutdown; all gates. |
| M20: Explicit installed lifecycle and publication credit | 5 working days | Explicit existing lifecycle/credit opt-in with the complete M19 robot and unchanged defaults/viewer/wire. | Real socket full-world credit/catalog controls, guides, all mandatory gates and actual supported installed-browser controls/shutdown; see M20_DELIVERY.md. |
| M21: Loaded altimeter Gaussian noise | 5 working days | Existing single Rapier altimeter with seeded Gaussian/quantized position and velocity channels. | Native zero-elapsed noise application, noise-free reference, deterministic streams, demand/rate/pause/reset/fault and actual wire controls; focused native evidence and all gates. |
| M22: Loaded air-pressure Gaussian noise | 5 working days | Existing single Rapier pressure sensor with seeded Gaussian/quantized output and native double variance. | Native zero-elapsed application and sqrt(stddev) variance, positive-output/invalid suppression, deterministic streams, demand/rate/reset and actual wire controls; focused native evidence and all gates. |
| M23: Loaded magnetometer Gaussian noise | 5 working days | Existing single Rapier magnetometer with seeded sensor-axis X/Y/Z Gaussian/quantized noise. | Native post-rotation XYZ/zero-dt, demand/reset/fault/wire controls and all gates; see M23_M24_DELIVERY.md. |
| M24: Planar CPU-lidar white Gaussian noise | 5 working days | Existing single planar Rapier lidar with zero-minimum finite-hit mean/stddev noise. | Native activation, beam-order/skipped misses/post-noise clipping, stale-scan fault retirement, unchanged ideal behavior and all gates; see M23_M24_DELIVERY.md. |
| M25: Updated installed preview through M24 | 5 working days | Fresh native-free Ubuntu archive with the four parent noise examples and dedicated guides, preserving the default robot and viewer. | All gates, fresh binaries/notices/archive, clean non-root installation with four actual example loads, new installed browser controls/shutdown and actual publication readbacks; see M25_DELIVERY.md. |
| M26: Ideal loaded AirSpeed | 5 working days | One top-level Rapier sensor, zero wind/default atmosphere/no noise, sensor-point and frame-correct native numeric pressure and temperature. | Actual model/native/wire/demand/cadence/reset/removal/atomic refusal proofs and every mandatory gate; see M26_M27_DELIVERY.md. |
| M27: Flat-model PosePublisher | 5 working days | One built-in vector publisher for selected model/link parent-relative poses, exact headers and native unpaused cadence. | Native keyed frame/stamp records, real wire and lifecycle/atomic-collision controls and every mandatory gate; see M26_M27_DELIVERY.md. |
| M28: Loaded AirSpeed pressure noise | 5 working days | Existing AirSpeed with Gaussian/quantized noise on native pre-100 float pressure. | Numeric/seed/demand/reset/fault/wire controls, focused native evidence and every mandatory gate; see M28_M29_DELIVERY.md. |
| M29: Updated installed preview through M28 | 5 working days | Fresh native-free Ubuntu preview with all seven finite examples and unchanged robot/viewer. | Full gates, fresh package/closure/relocation, clean non-root install, seven actual example runs, new installed browser/shutdown and actual release readbacks; see M28_M29_DELIVERY.md. |
| M30: Ideal CPU LogicalCamera | 5 working days | One loaded Rapier origin-frustum camera with canonical image and frustum streams. | Native inclusive point predicate/full relative poses, both-stream demand/wire/lifecycle/refusal proofs and all gates; see M30_M31_DELIVERY.md. |
| M31: Static-vector PosePublisher | 5 working days | Existing flat-model publisher with native dynamic/static channels and independent cadence. | Native classification/refreshed poses/rates, both-topic reservations and real wire/reset/retirement proofs plus all gates; see M30_M31_DELIVERY.md. |

M1–M29 are accepted. [M28/M29 acceptance](M28_M29_ACCEPTANCE.md) records Main
`ab9660ee`, exact CI42586254208 attempt 1, all independent DATA and the fresh
seven-example installation, actual browser/shutdown and verified publication.
The M29 preview is checked Latest; M25 and every older release remain preserved.
There are 31 selected phases and 29 accepted. M30/M31 now continue under
[their explicit contracts and disjoint source leases](M30_M31_DELIVERY.md).
No new check, native proof or source-feature acceptance is inherited; local
heavy jobs remain serial and every required gate remains unchanged.

### Historical atomic M21–M25 integration checkpoint

The following original pending/Latest statements are superseded by the actual
M25 acceptance above; the old failures and capture identities remain protected.

M1–M20 are accepted and published; [M20 acceptance](M20_ACCEPTANCE.md) remains
the checked installed preview. [M21_M25_DELIVERY.md](M21_M25_DELIVERY.md) now
combines the complete reviewed M21–M25 source into one new candidate over M20,
avoiding three serial qualification cycles. Both frozen M21/M22 hosted attempts
failed at the same LLVM Scene-link SIGBUS of unknown cause; no third unchanged
retry is authorized. Old local/provisional passes and native captures remain
separate, correctly bound evidence.

The atomic candidate must pass new full normal/strict actual-M20-parent coverage,
exact hosted CI and independent Source/local/official DATA. All separate sensor
contracts and fresh M25 binaries/notices/archive/clean installation/four actual
installed examples/new browser/shutdown/publication proof remain required before
accepting M21–M25 together. The CI correction aligns debug information with the
passing local configuration and adds bounded diagnostics; it does not establish
the cause or weaken checks. All original starts/budgets and accepted history are
retained. There are still 25 selected phases and 20 accepted; M20 remains Latest.

D31 permits independent bounded increments to run in parallel with explicit file
ownership, one integrator and one complete release candidate. A dependency may be reused from existing
accepted code, but no phase inherits acceptance merely from an older test run.
Existing broad P0–P7 gates remain historical and long-term coverage inventories;
they are not this release schedule and do not block M1 as entire families.
The following M15–M19 preparation paragraphs are historical checkpoints; current
M25 acceptance and M26/M27 execution policy are recorded above.

`planning/delivery-phases.json` is the machine-readable schedule mirrored by the
tracker. M1–M16 are accepted under their bounded criteria, independently of total
rewrite effort. M15/M16 Main is `5f53c92e`; its public `m15-m16-features` receipt
records actual qualification. M12's installer remains unchanged. Current M17 ideal
magnetometer reuses accepted sensor infrastructure; M18 joint-state publication
reuses admitted M9 joints. See [M17_M18_DELIVERY.md](M17_M18_DELIVERY.md) for exact
contracts and parallel ownership. Neither inherits acceptance from an older run.

[M19_DELIVERY.md](M19_DELIVERY.md) selects the next useful bounded delivery:
refresh the installed preview, which otherwise remains the older M12 payload.
Its source, regressions and guides may proceed separately while the exact M17/M18
parent qualifies. After that parent passes its local normal hook, native proof
and independent local audits, reviewed M19 local checks may overlap parent
hosted CI. Local heavy work stays serial. Acceptance and publication still require
the finally qualified parent, every exact M19 gate and its own installed artifact
proof. A parent source correction requires rebasing and affected required checks.

Owner decision D32 directs automatic continuation: after accepting a phase,
select and start the next useful bounded delivery while retained work remains.
Do not stop merely because a successor was unselected. Each scope still needs
its dependencies, fixtures, unsupported cases and exit criteria recorded before
implementation, and every acceptance gate remains mandatory. Stop for an explicit
owner pause, completion of retained scope or a genuine blocking owner decision.



## M1: smallest real end-to-end product

Deliver one example world with a static box floor and at least 20 dynamic boxes
and spheres. Reuse the current Rapier f64 adapter, scene loading, IDs, clock and
upstream Rust Zenoh integration. Add a selectable Rapier-only runtime build/run
path that does not require native DART/MuJoCo libraries or the C++ reference SDK
on the user's machine. Preserve the existing backend implementations and gates.
The server remains usable without a window or GPU. The browser requires an available supported graphics backend; prefer WebGPU
and provide the maintained viewer's WebGL2 path where supported. Test an actual
browser; a successful WASM build alone is not visualization evidence.

The maintained viewer is `gazebosim-rs/gzweb-rs`, checked out alongside this
repository at `../gzweb-rs` (baseline `3c3ca32`, Bevy 0.20). It already has live
Gazebo WebSocket networking, scene rendering and controls. The initially
inspected `/home/nkoenig/work/gazebo-web/gzweb-rs` is an older untracked Bevy 0.18
prototype; its isolated work and probe evidence are preserved, but it is not the
M1 delivery target. Reuse the maintained viewer's `SimBackend`, scene and UI
interfaces. Add an explicit primitive-backend selection without changing its
default Gazebo protocol. Viewer and simulator remain separate processes.

Use the narrow versioned scene snapshot, committed clock and pause/resume/reset
contract in MVP_BROWSER.md for this bounded local bridge. Simulator networking
continues to use unmodified upstream Zenoh. Do not create a new transport
repository or general protocol framework. Preserve D28's selected Gazebo Zenoh
compatibility at the simulator boundary. This browser bridge is not proof of
C++ Gazebo scene-service compatibility.
The snapshot must include stable world/entity IDs, box dimensions or sphere
radius, simple color, explicit units and frame conversion. Pose updates carry
simulation timestamps and reset epochs. Reconnect replaces stale state from a
fresh snapshot. Bound update/command queues; show connection/error state rather
than freezing silently. Browser pause/resume/reset acknowledgements reflect the
actual server state. A hard-coded or replayed browser animation does not qualify.

Day 1: prove one live body in the actual browser and agree the minimal contract.
Day 2: complete the primitive scene, poses and coordinate conversion.
Day 3: connect controls and reconnect snapshots.
Day 4: complete reset/shutdown/error regressions, clean-start commands and docs.
Day 5: finish independent review, batch fixes, run required final checks and ship.
This is a work breakdown, not permission for daily broad test/lint reruns.

M1 excludes meshes, sensors, jointed robots, remote Fuel downloads, native-engine
feature extensions and full Gazebo service compatibility. They are deferred,
not retired or credited as complete. M2 visuals still use primitive collisions;
M3 adds only the named collision shapes. Each sensor phase activates its own
model on demand and preserves zero per-system runtime cost when absent.

## Later cycles and retained scope

After M12, choose bounded items by dependency: one additional sensor/noise model,
one Gazebo system, one asset format, one native backend capability, or one
packaging/performance improvement. Independent items may share a parallel source
wave under D31; shared runtime interfaces and heavy qualification have one owner. Before starting, name its fixtures, commands,
unsupported cases and exit evidence and budget no more than five working days.
Split general DART/MuJoCo/TPE parity, remaining 78-system behavior, GPU lidar,
advanced cameras, distributed worlds, historical logs and Python workflows into
such increments. Keep the full legacy packet/test ledger for traceability.
Accepted phase counts do **not** set a whole-rewrite completion date. Continue
selecting bounded deliveries while retained work remains under D32.

M30 needs a narrow SDFormat correction over its consumed revision: admit the
matching LogicalCamera raw child without interpreting it. That dependency must
pass its own review, staged hook and hosted CI before the simulator pins it.
This does not resume the paused automatic-mass candidate.

Do not resume paused SDFormat mass-override or metric-depth-worker qualification
as a competing priority. Preserve their trees, evidence and accepted Main pins.
Adopt or finish them only when a weekly delivery actually depends on them. Never
edit the independent Fuel owner's work.
