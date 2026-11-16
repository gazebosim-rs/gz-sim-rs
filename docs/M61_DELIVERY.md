<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M61: eight independent differential-drive robots

Current acceptance: M61–M64 passed local and exact hosted qualification,
independent local/official DATA and ordinary Main/readbacks at192ede. M1–M64/197
are accepted; M60 remains Latest/install33. See
[the actual acceptance record](M61_M64_ACCEPTANCE.md). The feature contract and
retained qualification checkpoints below describe their historical scope.

## Ownership and dependencies

Root integrates and owns Scene drive loading/identity/reset code and its private
and public tests, shared qualification files, planning and current handoff records.
Light owns Server drive production/IO code and private/unit regression Source.
Messages owns the canonical C++ drive peer, Server public integration regression
Source, eight-robot example and feature guide details. Each author uses a separate
worktree over accepted M60. Shared-file changes require Root coordination.
SDF independently reviews semantics, native input scope and the complete batch;
Root independently reviews all delegated work and SDF reviews Root's Scene work.
Read pinned upstream implementation/tests and retained M59 reference inputs
before editing. No SDK rebuild or old accepted qualification replay is selected.

## Acceptance and verification cadence

Meaningful controls must cover zero, one, two, three, seven and eight loaded
owners, ninth-owner refusal and the unchanged manual singleton path. Validate all
current/original owners, wheels and sixteen topic keys atomically before motor
writes, IO allocation or lease acquisition. Preserve removed-owner reservations,
peer roles/remaps, error priorities, old weak/epoch/UUID rejection and independent
owner behavior. Keep a ninth sentinel in every bounded scan. Prove aggregate
budget fairness with malformed, stale and full queues and correct retirement.
Healthy reset restores eight fresh encoder references/sequence zero; failed reset
still performs required zero-command/sample cleanup. Use supported canonical
blocking removal RPCs, not unsupported editable-model removal.

Fresh scoped canonical native evidence must cover all eight actual routes, frames,
cadence, demand, reset and removal, plus real independent wheel/body response.
Keep all numerical limits and existing two-robot tests. Native component and wire
observations do not imply matched trajectories, native Physics or performance.

Finish implementation, meaningful regression Source, guide and independent review;
batch findings before one consolidated affected verification. Retain the complete
normal staged hook, strict workspace/per-crate/actual-parent coverage, independent
local DATA, exact hosted CI and independent official DATA before Main/acceptance.
No tests or lint between edits. Existing unchanged evidence may be reused only
where the maintained checks allow it. Test IDs and final commands are recorded
with the complete implementation, rather than invented future pass counts.

No new backend, transport/framework, controller math, general drivetrain support,
viewer/package/browser changes, SDK rebuild, performance claim or broad compatibility
inventory is included. Fuel, old releases, frozen trees and failures remain preserved. Implementation,
independent review and one formatter passed. Three full attempts failed: the old
admission assertion, service-context namespace setup, then one changed-line
coverage gap. Attempt3 passed2040headless/all24new controls/Python462/RGB9/finite35,
but Server14/15 changed executable lines did not exceed95%; renderer was not run.
One meaningful wildcard presence-identity refusal regression is being completed
without production or gate changes. See [verification retry](M61_M62_VERIFICATION.md).
The exact six-cache recovery is complete and consumed; no broader eviction.

## Planned Scene regression controls

Before Scene edits, Root inspected the pinned `71c60db3` DiffDrive source and
integration test through the retained upstream Git repository. No original-System
execution or native-reset parity is inferred. Planned new IDs are
`fleet_counts_and_ninth_refusal_preserve_loaded_identity_domain`,
`eight_loaded_owners_keep_real_motion_pause_reset_and_retirement_independent`, and
`all_eight_bindings_are_preflighted_before_cache_updates`. The existing third-owner
refusal assertion moves to the new ninth-owner boundary; existing zero/single/two
controls and all numerical assertions remain. New Source controls are not passes.

The Scene batch also adds
`ninth_counterfeit_reset_clears_every_bounded_command_and_sample_before_refusal`
to preserve required failure cleanup through the ninth sentinel. It checks every
bounded drive's zero command and absent sample, with unchanged physical state.

Root's finite-source qualification extension retains the first 33 fixtures and
adds the eight-robot fixture as number 34. Planned parser/retention controls bind
all 24 dynamic links and the exact static ground, refuse missing/extra/duplicate
identities and nonfinite coordinates, and reject tampered retained run metadata.
The installed package list remains the unchanged 33-example M60 distribution.

## Canonical fleet fixture and public controls (Source preparation)

`examples/m61_eight_diff_drives.sdf` contains one unchanged M8 world/ground and
exactly eight copies of its robot. Models `robot_0` through `robot_7` occupy two
columns at X=-1.25/+1.25 and four rows at Y=-2.25/-0.75/+0.75/+2.25 metres.
Only model identity, planar placement, topics and frame names change. Initial
Z=0.2, orientation, mass/inertia, collision/visual geometry, wheel axes,
wheel radius 0.2 m/separation 0.5 m, torque 5 Nm/target speed 10 rad/s and command bounds are retained.
Each owner keeps the original 50 Hz encoder publication cadence.

For owner N in 0..7, use canonical `gz.msgs.Twist` on
`/m61/robot_N/cmd_vel` and `gz.msgs.Odometry` on `/m61/robot_N/odom`, with
`frame_id=m61_robot_N_odom` and `child_frame_id=m61_robot_N_base`. Qualify raw
keys with the actual Server partition/namespace/remap options. All sixteen keys
must be distinct from each other and all fixed/peer reservations, including the
original keys of removed owners. A fresh loaded ninth robot is refused; it is
not an extra supported motor whose commands are silently dropped.

The current source finite `drive_command` helper already supports `--topic` and
retains finite duration, discovery, final zero command and session close. Use it
with the selected Server endpoint and partition; this does not make the existing
M60 installed package support eight drives. M60 and its 33 copied examples remain
immutable. This source example is a future qualification input, not a measured
run or new installed distribution.

Two appended public controls are declared, with all three accepted M59 public
control bodies retained unchanged:

- `genuine_fleet_counts_and_ninth_owner_refusal_preserve_public_admission` loads
  real 0/1/2/3/7/8 graphs, observes only their actual command subscriptions, keeps
  paused native state/time/sample bits fixed, and checks the genuine ninth-load
  refusal. It also byte-joins the eight-model generator to the finite example.
- `canonical_cpp_eight_motor_owners_keep_aggregate_budget_wire_demand_reset_and_retirement`
  uses the maintained upstream C++ client and actual process exit. One output is
  demanded initially; all eight motors, body poses and measured encoders still
  respond to independent real commands. Each paused update shares the original
  aggregate budget 2. Three fixed one-second windows drain all known demanded
  rows before the next window; actual signed chassis displacement and wheel
  states are checked independently of encoder packets. All eight outputs then
  bind complete raw wire, real samples, owner frames, cadence and sequences.
  Quiet drain/unwatch precedes real reset; fresh Server streams establish sequence zero.
  Supported blocking-removal RPCs exercise eight → seven → zero, preserve survivors,
  and reset restores all eight original identities/references. Final Server drop
  retires all routes before the owned peer is reaped with actual exit 0.

The peer keeps its existing argv 1/2 legacy routes and outputs. Counts 3..8 select
this fixed fleet topic/frame convention; no new command or ACK exists. Bounds
remain 4096 phases, 128-byte input lines, 128 queued rows per owner, 512-byte actual
wire, 1 MiB stdout/stderr and one original five-second phase deadline including
completion flush. Rust retains the existing owned child, bounded capture and
five-second cleanup; all failure paths preserve diagnostics. `SENT` is typed
publication, discovery is metadata, and neither is an actuation acknowledgement.

This proves canonical transport against Rust's actual wheel-encoder samples and
separate native Rapier wheel/body response. It is not an original DiffDrive
System/RNG/Physics/reset equivalence, global wire completeness, matched trajectory,
velocity-times-duration estimate or performance claim. No M61 test, native client,
physics step or check has run during this Source preparation. Root adds the
finite example to its shared qualification lists during reviewed integration.
