<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M15 air pressure and M16 local OBJ

The owner directed continuous delivery on 2026-11-07: after accepting a phase,
select and start the next useful bounded increment while retained work remains.
Do not stop because a successor was previously unselected. Stop for an explicit
owner pause, completion of retained scope, or a genuine blocking owner decision.
M1–M14 are accepted at Main `2f32bc7e906286ad16d9ec513c4bac659e21f996`;
the [combined release](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m13-m14-features)
contains the actual acceptance receipt. M12's installer and accepted viewer
`c40418cd` remain unchanged.

## Current parallel wave

Each phase is bounded to at most five working days including independent review,
fixes, documentation and all mandatory checks. Root owns integration and acceptance.

| Phase | Dependency | Runnable increment | Required evidence |
| --- | --- | --- | --- |
| M15: Ideal air pressure | Accepted M13 | One loaded, noise-free Rapier pressure sensor publishes canonical `gz.msgs.FluidPressure` through upstream Zenoh. | Actual pinned native sensor comparison, absolute resolved sensor world-Z including offsets, pressure/units/frame/time/sequence, cadence/demand/pause/loaded reset/removal, stale publication and atomic setup/reset refusals; no sensor means no callbacks or endpoints. |
| M16: Local OBJ meshes | Accepted M14 | Bounded local triangle-only OBJ visuals and existing Rapier static triangle or admitted closed-convex collisions. | Maintained parser source review, malformed/bounds/index/no-external-resource controls, analytic geometry/scale/basis/contact/reset, immutable generated GLB serving and one actual combined robot/browser control proof. |

## M15 boundary

Use the actual pinned `gz-sensors` air-pressure implementation, not the altimeter
relative-height formula. It applies its RotorS-derived geopotential formula to
absolute sensor world Z, with zero reference altitude and implicit standard
atmosphere. `gz-sensors` does not read SDF `reference_altitude`; reject nonzero
values explicitly for this increment. Reject explicit/nondefault atmosphere,
noise, visualization/metrics/plugins/extensions, multiple sensors, unrepresentable
rates and unsupported engines before creating simulation state or declarations.
Keep fixed SDF `SampleSchedule`, canonical frame/topic selection, acquisition
integer timestamps and reset epochs. Canonical `variance=0` denotes unknown
variance, not statistical certainty. Reject initial invalid computation domains;
later invalid acquisitions retain a typed model/sample error for Server to count
and suppress. Only sampling-deadline faults are terminal. Do not invent
atmospheric layers or clamps.

The immutable canonical schema already provides `FluidPressure`; no Messages,
transport, schema or generator change is needed. Reuse the existing typed CPU
lease/weak queued-job/IO-owned declaration pattern and bounded shared queue.
Reserve every existing endpoint family plus live and initial altimeter/pressure
keys in both directions before declarations. Loaded reset reconstructs original
sensor IDs/configuration/schedule and restarts sequence; native rewind parity is
not claimed. No runtime spawn, rate/reference services, new UI widget, native
backend expansion, environmental model or sensor framework is included.

## M16 boundary

Use maintained `tobj` 4.0.5, exact Cargo checksum
`6eb8e04167c1c0c76b5de63226fd733485ad63ef71e40de31e16272f47b099e2`.
Its published source is MIT, edition 2021, with optional dependencies and no
stated MSRV. Disable default features and use its ordinary f32 geometry, matching
the existing STL/glTF input precision before promotion to the f64 solver. Refuse
overflow or degeneracy after rounding; do not promise decimal f64 preservation.
Qualify it on the unchanged Rust 1.97.1 toolchain. The parser is eager, so preflight
bytes, lines, directives and declaration/face counts before calling it.

Retain 16 MiB/file, 64 MiB resources, 131072 unique vertices and 262144 triangles.
Admit local triangle faces with bounded position/UV/normal declarations and
validated positive/relative indices. Preserve winding; canonicalize signed zero
without approximate welding. Generate deterministic geometry normals through the
same GLB emitter as STL. Reject unknown/freeform/point/line/nontriangle syntax,
vertex colors, homogeneous coordinates, material/texture references and external
resource reads. Enforce finite, nondegenerate geometry and representable visuals.
Do not silently ignore unsupported syntax or turn arbitrary polygons into a new
triangulator. Positive finite SDF scale applies once; physics remains Z-up and
GLB uses the existing Y-up basis. Existing dynamic-convex and backend admission
limits remain unchanged. No repair, decomposition, materials, remote loading,
viewer/protocol changes or broader format parity is included.

## One writer per file

- SDF owner: new Sensors air-pressure model/export/tests; new Scene air-pressure
  leaf/private tests/fixture; native observer source and `M15_AIR_PRESSURE.md`.
- Messages owner: new Server pressure formatter/IO/private tests and the explicitly
  leased Server `lib.rs`, `world_io.rs` and `reset.rs` integration/attachments.
  Root independently reviews these shared Server changes.
- Asset owner: OBJ decoder, minimal shared STL GLB extraction, asset tests, CLI
  immutable capture and new private tests, OBJ collision tests/fixtures and
  `M16_OBJ.md`. Existing STL/glTF behavior and test intent stay intact.
- Root: Scene shared modules/exports/dispatch/initial/reset/removal/system guards,
  central CLI test attachment, Cargo manifests/lock, coexistence world, canonical
  planning, tracker and publication. Authors propose hooks rather than editing
  another owner's file. No changes to independent Fuel or paused backend work.

Finish implementation, meaningful regression source, docs and independent Source
review; batch findings before one consolidated normal hook. Batch real failures
before affected reruns. Preserve strict separate workspace/per-crate/actual-parent
coverage, exact hosted CI and independent DATA; no additional process or gate
bypass. Serialize local heavy work; source/review and remote CI may overlap.
Reuse unchanged accepted evidence where allowed, with each new phase's own
actual functional proof. Reuse the accepted viewer without rebuilding it.

Source worktrees/review refs are unaccepted. Qualified Main promotion remains the
acceptance boundary under D31, with actual generated IDs/times in the existing
release/promotion receipt and owned status, not a second metadata-only gate.
After acceptance, immediately select and start the next bounded wave under the
owner's standing continuation direction, keeping the web tracker accurate.

The final candidate records target M15/M16 `complete` status and no active
successor. This is conditional on the Main acceptance boundary above; it does
not claim that source authoring has executed the proofs. Until qualified Main
promotion, the web tracker and owned live status keep M15/M16 in progress.
Actual generated commit/run IDs, timestamps and native/browser evidence belong
in the existing immutable release/promotion receipt. If any required criterion
fails, retain the failure and keep the candidate off Main.
