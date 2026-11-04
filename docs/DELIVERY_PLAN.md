<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Weekly delivery plan

> M1–M9 are accepted, including live Rapier joint position and velocity control.
> Current M10 delivers focused interactive spawn/remove and entity-state operations
> for the admitted MVP scene, budgeted at most five working days including review
> and required checks. Preserve accepted demos, paused qualifications and Fuel work.


Owner decision D30 (2026-11-02) replaces the previous full-rewrite-first sequence.
Ship a useful MVP first, then small runnable releases. The initial planning
update left implementation paused; the owner subsequently explicitly resumed
M1 implementation and its continuation automation.

## Schedule and acceptance

M1–M6 were accepted on 2026-11-03 and M7–M9 on 2026-11-04; see [M1 evidence](M1_ACCEPTANCE.md),
[M2 evidence](M2_ACCEPTANCE.md), [M3 evidence](M3_ACCEPTANCE.md),
[M4 evidence](M4_ACCEPTANCE.md), [M5 evidence](M5_ACCEPTANCE.md) and
[M6 evidence](M6_ACCEPTANCE.md), [M7 evidence](M7_ACCEPTANCE.md) and
[M8 evidence](M8_ACCEPTANCE.md) and [M9 evidence](M9_ACCEPTANCE.md). M10 now targets **at most five working days**. Each subsequent phase is
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

The sequence is ordered around user-visible features, not parallel teams each
starting a week of unrelated work. A dependency may be reused from existing
accepted code, but no phase inherits acceptance merely from an older test run.
Existing broad P0–P7 gates remain historical and long-term coverage inventories;
they are not this release schedule and do not block M1 as entire families.
`planning/delivery-phases.json` is the machine-readable schedule mirrored by the
tracker. M1–M9 are accepted; M10–M12 remain unaccepted. These counts are independent of
historical P2 acceptance.

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

After M12, choose one bounded item at a time: one additional sensor/noise model,
one Gazebo system, one asset format, one native backend capability, or one
packaging/performance improvement. Before starting, name its fixtures, commands,
unsupported cases and exit evidence and budget no more than five working days.
Split general DART/MuJoCo/TPE parity, remaining 78-system behavior, GPU lidar,
advanced cameras, distributed worlds, historical logs and Python workflows into
such increments. Keep the full legacy packet/test ledger for traceability.
The twelve listed phases are **not** a promise that the entire rewrite finishes
in twelve weeks. No whole-rewrite completion date is currently scheduled.

Do not resume paused SDFormat mass-override or metric-depth-worker qualification
as a competing priority. Preserve their trees, evidence and accepted Main pins.
Adopt or finish them only when a weekly delivery actually depends on them. Never
edit the independent Fuel owner's work.
