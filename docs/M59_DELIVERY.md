<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M59: two independent loaded differential-drive robots

Provisional local qualification is complete at M59 `7f849bd3`, tree `0184e824`,
over M58 `173b31d2`. The full normal passed in 2,455.201 seconds with 2,013
headless tests, 1,176 overlapping renderer tests, 449 Python checks, strict
coverage and 33 finite examples. Independent retained DATA passed, including
all four byte-exact RAW comparisons (1,831/401/47/33 original profiles) and
573 headless/262 renderer named-control joins. All six failed normal originals
remain preserved. Hosted CI, accepted predecessors, official audits and Main
readbacks still gate acceptance and publication; M59 remains unaccepted.

Retained originals and exact receipts are under `artifacts/m59-delivery`;
`provisional-local-qualification.json` binds the commit, full normal and DATA.

## Historical Source and failed-run checkpoints (before qualification)

M59 is selected under D30–D32 on provisionally locally qualified M58
`173b31d2d13889f851c58dfff1ea81a34680ede8`. Its full normal and independent
retained DATA passed; hosted/official qualification, accepted predecessors and
Main promotion remain required. The isolated Source preparation over M57 was
reviewed under the explicit disjoint leases. Its twelve reviewed leaves are now
adopted onto M58 without replacing shared files or dropping M58 repairs.

The implementation candidate is `gz-sim-m59-integrated-multiple-diff-drives`.
Scene, Server/WorldIO and canonical peer integration have independent Source
review. Root serializes all heavy work and SDF reviews completed corrective
batches. The budget remains at most five working days including required checks.

The sixth M59 full normal retired with actual exit 1 after 1,798.811 seconds.
It completed 449 aggregate Python checks and native-free/RGB qualification,
including 33 finite examples. Headless execution passed 2,011 tests with zero
failures and five existing ignored controls. Both corrected removal paths and
the three-second motion/wire observations passed. Renderer was not reached.

The strict changed-line gate stopped at Scene's 363 covered lines out of 391;
it requires strictly greater than 95 percent. All twelve headless crate-wide
coverage gates passed. Server's 700/721 changed-line result is derived only from
retained coverage, not an executed later gate or qualification. Full coverage,
normal success and independent retained DATA have not passed for M59.

All six failed originals remain retained. Attempts 1–3 stopped at lint/compile;
attempt 4 failed the one-second motion observations; attempt 5 passed the
three-second observations but used the unsupported local removal API. Attempt 6
passed the supported blocking-removal RPC and all headless runtime assertions.
None of these partial results is a full M59 pass or hosted acceptance.

Two additional Scene regression cases for the coverage gap are Source-complete
and independently reviewed.
Their current Source obligation is 30 feature headless/15 overlapping renderer
controls plus two separate headless support controls: 32/15 combined. These are
Source declarations, not new measured passes; run 6 used the earlier 30/13 set.
Complete final batch review and one affected formatter before another full
unchanged normal and independent retained DATA audit.
No gate is active. Existing resource floors, assertions and limits are unchanged;
no deletion, eviction, focused trial or native campaign replay is authorized.

M60's package, launcher, regression and guide Source is independently ready.
Its proof-only procedures remain held for genuine qualified inputs and results;
final integration waits for actual M59 normal and independent DATA success.

## Bounded behavior

Extend M8 from zero or one to zero through **two** genuine loaded top-level,
dynamic Rapier models. Each has exactly the existing distinct left/right wheel
pair: at most four wheel joints, two canonical `gz.msgs.Twist` command inputs and
two canonical `gz.msgs.Odometry` outputs. Duplicate the admitted
`examples/m8_diff_drive.sdf` robot at separated, supported planar placements,
with distinct model names/topics/frames. Preserve the existing supported chassis,
wheel geometry, axis convention, force/speed/command limits and loading domain.
Neither count is an observed pass total.

Keep `diff_drive/config.rs` and `diff_drive/model.rs` unchanged. For +X forward,
+Z yaw and both wheel axes +Y, retain exactly
`left=(v-omega*separation/2)/radius` and
`right=(v+omega*separation/2)/radius`, existing uniform bounded-target scaling,
finite planar admission and last-valid-command behavior. Native Rapier motors
own physical response. Targets are not physical speed caps under external force.
Encoder odometry uses actual continuous wheel coordinates, the existing measured
ten-sample velocity window, local XY/yaw integration and configured cadence
(default 50 Hz). It is not commanded velocity times time or body ground truth;
wheel slip and native ABS/trajectory equivalence are not promised.

Each owner retains independent command, encoder reference/history, sample
schedule, output sequence and last-sample state. Paused/equal-time updates do not
actuate or integrate. Demand controls publication rather than driver existence.
Absent drives allocate no drive callbacks, queues or declarations. Keep global
ingress/message limits and existing weak endpoint ownership; introduce no IO
thread or transport abstraction.

## Original ownership and failure semantics

Root supplies a borrowed original model identity view containing stable ID,
kind, scoped name, parent and original configuration. Preserve the legacy
configuration iterator, missing-initial-state behavior and removed originals.
The existing InitialScene blueprint already stores private Drive, model and
wheel identities/parents/handles/definitions. A narrow internal borrow of those
records supports a pure Scene per-active-owner binding witness; do not expose
private native handles publicly or create another registry.

Validate current private owner/public configuration and original two native
joint bindings, immutable geometry/limits, stable names/parents and actual
current wheel/body/definition relationships against that blueprint. Global
current joint consistency alone does not prove original ownership after a
coordinated swap. For genuine loaded owners, Server candidate and retained-live checks consume
the witness before IO allocation, queue reads or counter mutation. Preserve the
existing MissingInitialState manual empty/single-metadata path and its eligibility;
it cannot admit a second owner or bypass original identity on a loaded world.
Mutable command/history,
schedule and samples are not immutable identity fields.

PreStep validates the complete bounded owner set and every target/native motor
operation before writing any motor. Preserve singleton error ordering and typed
command errors. PostStep prepares each owner's existing encoder/history/cadence
result before committing the owner set, without changing its numeric routines.

Reset has an explicit existing failure-cleanup contract: commands become zero
and stale DiffDriveSample components disappear **even when reset fails**.
The old `encoder_and_clock_failures_are_terminal_and_reset_discards_stale_samples`
assertions and fault messages remain. Do not replace them with a cache-unchanged
rollback premise. Validate all native clears before native writes; distinguish
required cache/fault cleanup from successful reset. Successful loaded reset
restores original models/wheels/IDs, encoder origins, schedules and fresh input
epochs. Real full-model removal retires only its streams and callbacks when the
last owner disappears; it is not evidence of live corruption or a global fault.

## Disjoint Source ownership

- Scene/Light: `crates/gz-sim-scene/src/diff_drive.rs`, new
  `tests/private/multiple_diff_drives.rs` and `tests/multiple_diff_drives.rs`,
  plus only the obsolete genuine-second-model subsection in existing
  `tests/private/diff_drive.rs` under the same old test ID.
- Server/Messages: `crates/gz-sim-server/src/diff_drive.rs`, `diff_drive_io.rs`,
  new private/public multiple-drive regressions and a bounded maintained
  canonical peer if required by the reviewed transport proof. Agree its exact
  path/protocol/registry before implementation; no native artifact is assumed.
- Root: shared initial identity/internal blueprint access, Scene exports,
  Server shared stream storage/reset/options/full-key reservations, one finite
  example, maintained registries/guides/planning, integration and evidence.
  Existing lifecycle code changes only if the reviewed two-owner contract needs
  one, under a separate explicit shared lease.
- SDF/reviewer: independent whole API/premise/Source review and M8 reference
  authority review. Review the full completed batch before corrections; Root
  alone releases serial heavy work.

The reviewed preparation leases and shared API remain in force during integration. No lane edits another
lane's shared files. A parent correction requires overlap/rebase review and fresh
affected required checks; old qualification is never transferred.

## Meaningful acceptance obligations

Use two real admitted loaded graphs, not cloned private components, to prove
opposite independent commands, actual wheel motors/chassis motion and each
encoder output, timestamps, frames, sequences and independent cadence. Retain
the original geometric/limit/invalid-command/clock/encoder negatives and all
existing M8 assertions. Extend only the exact old genuine-second-model refusal
subsection to genuine-two admission plus third-owner refusal under the same ID;
preserve its neighboring roll/pitch/rate/nested/plugin-conflict negatives.
The separate counterfeit private-Drive clone refusal remains unchanged.

Prove paused receipt without actuation, repeated-time behavior, global terminal
fault behavior before any healthy peer actuation, and all-owner preflight with a
late invalid owner. Live binding corruption must retain the entity and witness
the corrupted component/native binding. Full despawn separately proves retirement,
no cached output/requeued command, healthy-owner continuity and loaded restoration.
Preserve the existing reset-error command/sample cleanup explicitly.

Exercise actual canonical Twist ingress and Odometry output through maintained
serialization/transport. Check all four command/output keys, cross-family and
world-clock collisions before IO, owner-distinct bounded queues/global budgets,
old-epoch rejection and reset/removal endpoint retirement. Legacy M8 WorldIO owned
one active declaration and one pending install. M59 extends those to a whole bounded
two-owner installation, validate the complete set before lookup/clone/allocation,
and retain the existing worker, actual receipt fencing and global command budget
rather than multiplying that budget per owner. Legacy M8 Output lacked an immutable
endpoint pair; M59 stores the actual leased command/odometry keys as an ownership witness. A remap proof retains
the OLD live endpoint/lease while preparing NEW keys; same-key distinct-Arc swaps
and duplicate leases require actual distinct witnesses. No fabricated getters or
private Server calls from an external integration target. Preserve the existing
IO mismatched-job negative (different keys); new same-key swaps are separate
meaningful cases. Keep all old Server/IO and robot_sensors IDs/assertions, including
the manual/absent stream path, counterfeit extra metadata without original
identity, and direct retirement without queued odometry. Do not turn direct
retirement into an invented global-terminal or all-cache rollback premise.

Root reservation integration must exclude only Drive-owned keys. The present
generic legacy exclusion also drops controller inputs: explicitly retain both
current and original JointController keys when excluding Drive. Include actual
WorldIO clock/pose/control/removal endpoints, /clock and all configured peer input
and output roles, including IMU set_rate. Preserve simultaneous-error priorities:
existing global and controller selectors retain earlier constructor wins; Drive
constructor collisions cover its actual earlier IMU/lidar data, /clock and three
stored WorldIO keys; Force, set_rate and removal roles remain in full reservations
without moving a later typed peer's error. During reset, typed peer preparation precedes
restored Drive preparation. Pure Drive identity/own-key checks must not replace
these peer-specific winners with a blanket DiffDriveIdentity error. Constructor
uses current live eligibility; loaded reset uses the complete restored original
set. Record exact precedence in the selected shared API before implementation.

M8 explicitly chose a Source-informed contract, not a fresh C++ DiffDrive campaign
(`docs/M8_DIFF_DRIVE.md`, native Source revision 71c60db3 and math 1316ed33).
Retain those authority distinctions and any historical interoperability evidence.
There is no claimed available original DiffDrive executable/capture, native reset
interface, lifecycle/tick-order/solver/trajectory parity or source-binary proof.
If a fresh original-System campaign is required, close its actual input/ABI/link
dependency in a separately reviewed scope before promising a five-day exit.

Complete implementation, meaningful regression Source, finite fixture and guides,
independent whole review and batched fixes before one consolidated formatter and
the unchanged full staged normal hook. Require strict workspace/per-crate and
actual-parent changed coverage, maintained canonical/finite controls and an
independent retained results audit. Derive actual census/profile totals from
reachable Source and real outputs; forecast no pass counts or 100% result.
Full normal success alone is not independent DATA or local qualification.

Budget at most five working days including review and mandatory checks: one day
for dependency/API/fixture premises, two for disjoint complete Source, and two
for integration/review and serial verification. This is a working budget, not an
ETA or permission for daily broad reruns. Exclude new drivetrains/topologies,
acceleration/jerk/TF/enable/namespace expansion, motor/backend/solver changes,
new dependencies/frameworks, package/browser refresh or a compatibility sweep.
Accepted predecessors, exact hosted CI, official audit, ordinary Main/readbacks
and publication remain mandatory before acceptance. M1–M42/175 slices, M40 Latest,
and every locally qualified but unaccepted predecessor retain their exact status.
