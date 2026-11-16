<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M62: eight independent loaded joint-controller chains

## Scope and sequencing

M62 is an independent Source preparation lane on accepted M60
`a1a01192661281fc4691fe5c048a65e1986aefa4`, selected by the owner's request to
continue the next phase. M61's reviewed/formatted candidate remains unchanged
and unaccepted after three failed full verification attempts. M62 does not
use M61's unverified APIs or results. Its budget is at most five working days,
including review and mandatory gates, not a promised completion date.

Extend actual M58 loaded-controller admission from two owners/four inputs to
eight owners/sixteen inputs. Each top-level dynamic Rapier model retains exactly
the supported M9 two-link, two world-rooted serial +Z revolute-joint graph, with
one or two configured controllers. Keep configuration, native witness, velocity
motor and bounded damped-position-servo arithmetic and limits unchanged. Add
one finite eight-chain fixture. Position mode is not native force-PID/ABS parity.

Source work may proceed in separate worktrees under D31. Root later integrates
against the actually qualified predecessor and reviews all coexistence/shared
changes before one final complete Source census and verification cycle. If both
independent phases are integrated in one candidate, each retains its own actual
functional evidence and acceptance boundary. No M61 result or acceptance is
transferred. No product verification starts while capacity or Source review is held.

## Preserved contracts

- Keep the existing budget B **per input stream**, with a family maximum of
  sixteen times B; every stream still gets its own bounded visit. This is not
  the Drive family's aggregate-budget policy. Preserve old singleton/two-owner
  budget assertions and malformed, nonfinite, full and old-epoch behavior.
- MissingInitialState does not authorize a runnable manual controller. Preserve
  bare metadata key reservations, empty behavior and unrelated Drive singleton
  rules; genuine controller owners require the original loaded blueprint.
- A one-controller model still has the same two joints. Its unconfigured joint
  keeps original infinite effort/velocity and zero-dynamics admission; removing
  a plugin alone from a finite-limited joint is not a valid one-controller case.
- Enforce eight owners separately from sixteen inputs. Reject a genuine ninth
  **one-controller** owner even below sixteen inputs, plus a seventeenth input
  in the appropriate private/IO negative controls. Scan with a seventeenth sentinel.
- Validate all owners, parent/native joint witnesses, current/original endpoint
  reservations, remaps and retained/prepared leases before queue/motor/IO changes.
  Leases bind endpoint, active state, Arc/Weak and epoch; add no UUID registry.
  Narrow IO work may preflight all endpoint syntax, concreteness, duplicates and
  presence before the first declaration, preserving existing reason priorities.
- Preserve the controller reset's ordered native preflight and atomic restore.
  Earlier missing-joint errors still beat later overflows. Do not transplant
  Drive's failure-zero/sample-cleanup behavior. Use the existing blocking
  EntityRequest/Boolean RPC for public removal; local editable removal stays refused.
- Double is command input only. There is no controller output topic, 50 Hz
  output cadence or invented ACK. Observe real joint/backend coordinates and body
  orientation, exact advancing 1 ms sample time/epoch, paused bit preservation,
  fresh reset targets, retirement and absence. No trajectory/no-slip oracle.

## Leases and planned controls

Root owns Scene joint_controller.rs and Scene public/private multiple-controller
regressions, shared hooks/qualifier/planning/current guidance. Light owns Server
joint_controllers.rs, joint_controller_io.rs and their private controller,
multiple-controller, boundary and IO regressions. Messages owns the existing
canonical C++ controller peer, Server public integration regressions, the new
M62 fixture and an appended feature-guide section. Separate author worktrees
start at accepted M60; shared files require Root coordination. SDF independently
reviews Root and both leaf batches; Root reviews integration. Read the pinned
upstream Source and retained M58 native/component contract before editing.

Root plans loaded_controller_fleet_counts_preserve_blueprints_and_reject_ninth_owner,
eight_controller_chains_keep_physical_response_and_loaded_reset,
all_sixteen_controllers_preflight_before_any_motor_write, and
controller_fleet_failure_preserves_ordered_reset_validation. Light's controls
cover genuine 0/1/2/3/7/8, sixteen native/key/lease witnesses, per-stream bounded
queues, reset/removal/OLD reservations, whole-set IO and final-owner typed errors.
Messages plans genuine_loaded_controller_fleet_counts_mixed_slots_and_ninth_refusal
and sixteen_canonical_doubles_drive_eight_independent_chains_and_reconstruct_after_retirement.
These are Source plans, not observed passes.

Preserve all old test IDs, fixtures, assertions and numerical bounds. Old five-input
refusal moves to the seventeenth-input boundary while four valid inputs remain
explicitly covered. The real C++ peer must send all sixteen canonical Double
routes, preserve old one/two-owner modes and all wire/deadline/queue/exit bounds,
and join input counters with actual eight-owner physical/sample/lifecycle evidence.
The retained M58 isolated original-component captures remain scoped to their
original two families/four slots; no eight-original-System or Source/binary,
native Physics/reset/trajectory/performance equivalence is claimed.

## Completion

Complete implementation, meaningful regression Source, guides and independent
whole review; batch findings before formatting and one consolidated final pass.
Keep full normal staged hook, strict workspace/per-crate/actual-parent coverage,
independent local DATA, exact hosted CI, official DATA and ordinary Main/readbacks
before acceptance. No gates or tolerances weaken. Existing exact unchanged
reference evidence may be reused only where maintained checks permit it.
M60's installed 33-example preview/viewer remains immutable; a new Source fixture
is not an installed-package refresh. No SDK rebuild, original-System campaign,
new backend/math/controller topology/transport/framework, schema work, browser,
package, renderer or broad compatibility inventory is selected. Preserve Fuel,
frozen M61, all originals and failures. The original feature implementation/review/formatter are complete; the coverage
regression needs complete review and one affected formatter.
The first normal failed an old admission assertion; the second stopped at namespace
setup. Attempt3 passed2040headless/all24new controls/Python462/RGB9/finite35, then
failed strict Server changed-line coverage14/15. Renderer was not reached. A real
wildcard presence-identity refusal regression targets that concrete gap without
production/assertion/gate changes. See [verification retry](M61_M62_VERIFICATION.md).

## Root Scene Source preparation

Root inspected pinned upstream `71c60db3` JointController/JointPositionController
configuration and retained M58 Scene/Server controls before editing. The Scene
batch extends only owner/input bounds, seventeenth sentinels and fixed prepared
slots; model/configuration and motor arithmetic stay unchanged. New public counts
exercise all-two/all-one/mixed fleets, including ninth one-controller refusal.
New eight-owner controls keep the accepted one-second position bound 0.1 and
signed velocity response product bound 0.03; the existing thirty-second 0.01
precision controls remain exact. Private controls validate all sixteen witnesses
and preserve earlier-native-fault versus overflow reset ordering without clearing
valid pending targets. These are unrun Source controls, not product results.

## Canonical eight-chain fixture and public controls (Source preparation)

`examples/m62_eight_joint_controllers.sdf` contains eight M9 chains named
`arm_0` through `arm_7`. Their centres have X=-1.5/+1.5 and
Y=-4.5/-1.5/+1.5/+4.5 metres, with the original Z=1 and identity rotation.
The three-metre spacing separates their swept geometry. The world-rooted joints
and zero gravity do not require the outer chains to be supported by the unchanged
ground. All link geometry, inertia, +Z axes, local anchors, motor parameters,
1 ms step size and configuration modes remain the M9 values. Both configured
joints retain effort 3 and velocity 4; position retains `cmd_max=2`.

Each owner N consumes canonical `gz.msgs.Double` commands on:

- `/model/arm_N/joint/base_hinge/0/cmd_pos` for position in radians;
- `/model/arm_N/joint/tip_hinge/cmd_vel` for velocity in radians/second.

The velocity route has no joint-index segment. Actual Server options qualify and
remap these keys once. A one-controller model still contains both physical joints;
its unconfigured tip joint has the original implicit infinite effort/velocity
bounds and zero dynamics. Merely removing its plugin while retaining finite
limits does not create a valid one-controller model.

The maintained C++ peer retains argv `1`/`2` and their existing `arm_left`/
`arm_right` routes. Counts `3` through `8` select the fixed `arm_N` fleet convention.
Slot `2*N` is position and `2*N+1` is velocity. Existing `send`, `matched`,
`retired` and `finish` commands and limits remain unchanged: 4096 phases,
128-byte input lines, 32-byte actual Double wire, five-second typed discovery,
1 MiB per Rust-captured channel, eight-second Rust phase waits and five-second
owned exit/cleanup. `SENT` carries actual encoded bytes, not an actuation ACK.
The existing `multiple_joint_controllers` client key and compile/support route
remain sufficient; no new registry or transport is introduced.

Two public controls are appended, preserving both accepted public test bodies
and all preceding helpers byte for byte:

- `genuine_loaded_controller_fleet_counts_mixed_slots_and_ninth_refusal` admits
  actual 0/1/2/3/7/8 graphs with two, one and mixed configured joints per owner.
  It observes exactly their canonical command subscriptions, unchanged paused
  native state and samples, then full input retirement. A ninth one-controller
  owner is refused with only nine configured inputs, separating the owner bound
  from the sixteen-input bound. Its eight-model generator byte-matches the fixture.
- `sixteen_canonical_doubles_drive_eight_independent_chains_and_reconstruct_after_retirement`
  sends real finite binary64 commands over all sixteen upstream C++ routes and
  checks actual protobuf bytes and application counters. Three messages per
  stream exercise budget B=2 per stream, with at most 32 consumed per paused
  update; private controls retain the per-stream malformed/stale/full-queue proofs.
  Actual 1 ms physics steps join every configured sample's time, epoch and q/v
  to backend coordinates. Independent child-body orientations follow the measured
  native joints. Distinct signed position/velocity targets keep the existing
  one-second physical response bounds, with no output subscribers or pose writes.
  Pause preserves complete native/sample bits. Real reset restores identities,
  original zero targets and poses without replaying old targets. Supported blocking
  model-removal RPCs exercise eight → seven → zero, retain survivor actuation,
  then reset restores all eight and genuine commands work again. Final Server
  drop retires all sixteen inputs before actual owned peer exit is accepted.

These are Source declarations, not measured passes. There is no controller output
or 50 Hz publication cadence: cadence here is the actual 1 ms physics/sample clock.
The retained M58 original-component observations remain scoped to their original
families and four slots; they are not eight original Systems, a source/binary
match or a Rust servo/native ABS/Physics/reset/trajectory equivalence. No new
reference capture, SDK build, performance result or installed distribution is
claimed. M60's accepted 33-example package and M61's frozen candidate are unchanged.
Root integrates this finite Source example after M61 in the maintained qualifier
and regression lists: the combined registry has 35 examples; installed M60 stays33.

## Atomic M61/M62 qualification amendment

Root adopts the independently reviewed D31 co-integration plan: both independent
features use genuinely accepted M60 `a1a0119`. Preserve the separate frozen M61
`e4a559f16b85b8767a321e64e8394412bf4f3752` candidate and all prior evidence.
Compose its reviewed Drive leaves with reviewed M62 leaves in this new candidate.
Retain every own functional/native/wire/lifecycle control and add a scoped
maximum-count coexistence control before final whole integration review. This is
not a qualified-parent or pass claim for the frozen M61 candidate.

One fresh combined exact head must pass the full normal hook, strict workspace,
per-crate and actual M60-parent coverage, independent local DATA, exact hosted CI,
independent official DATA and ordinary Main readbacks. Only then may M61 and M62
be accepted atomically, with each feature's own functional proofs. This replaces
separate intermediate qualification timing only; no test, tolerance, gate,
original reference or actual-result requirement is removed or transferred.

The coexistence world retains M8 gravity -9.81/ground and shifts only controller
model XY far away, retaining Z1/RPY0 and joint topology. It proves coexistence,
not equivalence to the untouched zero-gravity M62 native/physical fixture. Check
eight drives, eight controller owners, twenty-four inputs, thirty-two distinct
reserved command/odometry keys and thirty-two joints, both independent ingress
policies (Drive total B, controllers at most16B), real wheel/body/joint response,
paused native/sample/time purity and loaded reset/restored identities. Existing
original/current/remapped/OLD and lease tests stay intact. The appended Source control is
`maximum_drive_and_controller_fleets_keep_disjoint_ingress_and_loaded_reset`.

The shared finite registry keeps the exact first33 fixtures and appends M61 then
M62 (35 Source examples); the installed M60 package remains33. Source/context
and actual-parent census are recomputed for the combined candidate, with no
double-counted retained reference results. Capacity and exact cleanup approval
remain held; this Source amendment releases no verification or recovery action.

## Source-complete checkpoint

The combined implementation, regression Source, fixtures, finite registry and
independent whole integration review are complete. All confirmed findings were
batched before one consolidated `cargo fmt --all`, which exited zero. The copied
M61 private helper uses idiomatic integer parity; the frozen M61 donor is unchanged.
The coexistence regression joins every one of its 32 actual joint samples to
current backend state, time and epoch after advancing, as well as both families'
input budgets, paused purity, physical response and loaded reset.

These are reviewed Source declarations. No M61/M62 product tests, coverage,
native execution, hosted CI, local DATA or acceptance has run. Full verification
is held by capacity: the fresh host SSD reading was 8,901,894,144 bytes free,
below the unchanged 14 GiB initial guard. Other operating floors held. The
practical target is 35 GiB based on the preceding normal run's scratch use.
The exact six-target compiler-cache recovery still needs the owner's explicit
approval; no cleanup, new native capture, SDK, package or viewer build ran.
