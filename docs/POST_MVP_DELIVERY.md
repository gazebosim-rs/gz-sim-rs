<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Post-MVP delivery: M13 and M14

> Completed contract: M13/M14 were accepted at Main `2f32bc7e` and published
> in `m13-m14-features`. The record below preserves that wave's original scope.
> Current work is [M15 air pressure and M16 local OBJ](M15_M16_DELIVERY.md).
> D32 directs continuing to the next useful bounded phase after acceptance.

The owner requested continuation and parallel delivery after M12 on 2026-11-07.
M12 canonical Main is `c68b03fcd48682082a12cfc9ad2c78cae27553f3`; its preview,
qualified source, acceptance evidence and original failures remain unchanged.

## Two independent increments

| Phase | Budget | Runnable result | Acceptance |
| --- | --- | --- | --- |
| M13: Ideal altimeter | At most five working days, including review and gates | One loaded-world, noise-free Rapier altimeter publishes canonical `gz.msgs.Altimeter` through existing upstream Zenoh. | Native model comparison, actual offset-point velocity, frame/topic/time, fixed SDF cadence, demand, pause, loaded reset, removal and bounded stale-publication fences; no sensor means no callbacks or transport declarations. |
| M14: Local STL meshes | At most five working days, including review and gates | Local ASCII/binary STL visuals and existing Rapier static triangle / admitted closed-convex collisions. The maintained viewer consumes an immutable generated GLB. | Bounded malformed-input rejection, analytic geometry/scale/pose and contact controls, immutable served geometry, reset/removal, and one actual browser visualization/control session. |

Both depend on accepted M12. Neither depends on the other's new implementation.
The budget is a scope limit, not a promised completion date. Split optional work
before candidate freeze if either increment will exceed its budget.

## Frozen boundaries

M13 reports world-Z sensor position minus its resolved initial world-Z reference,
world-Z velocity at the sensor point, and that fixed initial reference. Point
velocity includes the parent's angular-velocity cross offset term. Sensor rotation
does not rotate the measurement axis. Reuse `SampleSchedule`, current frame/topic
conventions, typed faults, epochs and demand-installed system retirement.
Only one loaded-world altimeter is admitted. Reject nondefault noise, unsupported
metadata and unrepresentable rates atomically. Runtime altimeter spawning, rate
setters, new services, UI widgets and native rewind-parity claims are excluded.
Loaded reset follows the simulator's existing reconstruction contract.

M14 reuses maintained `stl_io` 0.11.0 (MIT, Rust minimum 1.85, no dependencies),
pinned by the Cargo index checksum
`567641995c51a3b8befddb13e1826187bcf7eb7ca8d13746bfd1cc5a22e89fa8`.
Use streaming triangle iteration with bounded indexing; do not eagerly allocate
from a claimed triangle count. Existing limits remain 16 MiB per file, 64 MiB
total resources, 131072 unique vertices and 262144 triangles. Validate finite
coordinates and nonzero area; preserve winding and recompute visual normals.
Handle binary headers beginning `solid` without interpreting them as ASCII.
Reject unsupported color/attribute payloads, malformed/trailing data, unsupported
submeshes/optimization and resource escapes. Positive finite SDF scale is applied
exactly once. STL is Z-up; generated GLB uses the existing glTF basis conversion.
No new viewer code, wire format, mesh repair/decomposition, remote asset loading,
general exporter or backend capability is introduced. Existing dynamic-convex
admission limits are retained rather than silently repairing arbitrary surfaces.

## File ownership and integration

- Altimeter owner: new `gz-sensors/src/altimeter.rs`, its export and focused tests;
  new `gz-sim-scene/src/altimeter.rs`, new Server altimeter formatter/IO modules,
  new exclusively named tests/fixtures and `docs/M13_ALTIMETER.md`.
- STL owner: `gz-assets/src/mesh.rs`, new STL decoder/GLB leaf modules, exclusively
  named asset/mesh tests and fixtures, `gz-sim-cli/src/web/assets.rs`, and
  `docs/M14_STL.md`. Shared behavior must use one decoder, not two importers.
- Root integrator: Cargo manifests/lock, Scene `lib.rs`, `initial.rs`, `lifecycle.rs`,
  `world_spawn.rs`, `world_systems.rs`, Server `lib.rs`, `world_io.rs`, `reset.rs`,
  common test attachments, example coexistence, canonical planning and release.
  Owners propose small hooks/API signatures; Root applies shared-file changes.
- Independent reviewer: review each completed lane and Root's shared integration;
  batch all confirmed findings before the consolidated verification pass.

Root explicitly leases the three central Server files to the Messages owner for
this wave; that owner coordinates with the altimeter leaf author. Root retains
Scene integration and independently reviews the Server integration. Each shared
file has exactly one writer; the lease creates no additional acceptance authority.

These are leases for one isolated development checkout. Accepted/frozen trees,
old services, source evidence, caches and independent Fuel work are preserved.
No dependency, schema or global configuration change is delegated to a lane.

## One candidate and one qualification cycle

Complete both implementations, meaningful regressions, API/capability documents
and independent Source review before freezing one integrated candidate. Exercise
coexistence as well as each feature. Serialize local heavy builds/native proofs
under the existing resource guards; source authoring and review can run in
parallel. Keep the full unchanged staged hook, separate strict workspace/per-crate
line and region gates, exact hosted CI and independent retained-data audit.
Never union coverage, relax assertions/tolerances or substitute a prior result.
Inspect available real failures and batch fixes before affected reruns.

Source worktrees and review refs do not confer acceptance. A final candidate may
contain the target `complete` phase status, but it becomes effective only when
that exact candidate is promoted to Main after actual feature-specific proofs,
all mandatory checks, independent review/data clearance and Root acceptance.
Until then the published Main and owned live status remain authoritative.

Do not invent future commit IDs, run IDs or acceptance times in tracked files.
Record actual values after qualification in the existing immutable release /
promotion receipt and owned status. Each phase links its acceptance contract to
that published receipt. A later feature candidate may incorporate known prior
receipt details into retained history. This avoids a second metadata-only full
qualification cycle merely to insert identifiers generated by the first cycle.
It adds no gate bypass, result-reuse framework or automatic acceptance path.

Each phase needs its own actual functional evidence even if both share the same
qualified candidate. Defer an unfinished lane before freeze; the ready lane can
ship after all its exit criteria pass. One combined updated preview is preferred
when both are ready, using the accepted viewer and existing packaging procedure.
Broader sensors, format families, rendering-worker distributions and full rewrite
compatibility remain outside these two increments.

The machine-readable candidate records target M13/M14 completion and no active
successor. Its legacy `active` / `current_phase=M14` fields identify the last
named phase for the existing tracker schema; `active_phases=[]` and the owned
live status distinguish actual ongoing work. This is conditional on the Main acceptance boundary above,
not a claim that source authoring has executed the proofs. Root publishes the
actual immutable `artifacts/m13-m14-delivery/acceptance-main-promotion.json`
receipt and owned tracker status after qualification. If any mandatory criterion
fails, preserve the failure and keep the candidate off Main.
