<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M92: posed singleton primitive runtime templates

M92 copies a translated or rotated Box, Sphere, Cylinder or Capsule collision
through the existing passive Rapier runtime-template APIs. It shares one D31
candidate with [M93](M93_DELIVERY.md). Each phase has at most five working days,
including independent review and mandatory verification.

## Basis and current checkpoint

The genuine locally qualified, unaccepted parent is
`e264c6494d69ccacf41e0b21ddf3827d1f12cd40`, tree `19187aea`, parent `1e54399a`.
Normal `1e23ed5a`, independent local DATA `9948aa52` / report `59a9b368`, review
publication `a3324402` and exact hosted CI 44925640602 attempt 1 passed.
Receipt `6f169244` and collector `08d22772` / collection `3e53233e` preserve the
original terminal metadata and 33 successful collection requests. Official audit
and acceptance remain pending. Parent failures `f9d43298` / `fc5dce53` and its
separate current-three/intermediate-world_spawn-one/original-four formatter
custody stay intact; none qualifies this wave.

The first M92/M93 full normal returned Root/Git 1 after 1,607.763467844 seconds.
Retained failure `005bf028` records 1,965 headless passes, two failures and two
ignored cases across 192 summaries: fifteen of seventeen new cases passed in
headless only. No renderer or strict coverage ran, and no qualified commit was
created. Resource, signal and finalization guards held. Freeze tree `6b82b5e3`,
its packet and qualification-attempt ref; preserve the original preformat,
seven-file formatter `12ccb626`, final review `e538a563` and snapshot `e45e22af`.

After all independent diagnoses, Root selected two test-only physical-control
repairs. Both are authored and independently Source-reviewed: M92 `63716999`
(completion `2a7d26e7`) and M93 `dbf52c78` (original completion `63364729`, narrow
named-field WorldPose correction `7d676fae`). Reciprocal, Scope and Root whole
code reviews are CLEAR. The WorldPose `.0` error was caught and corrected before
execution; LocalPose's wrapper remains exact. Final authored eight-doc review
is pending at this Source checkpoint. No formatter or retry has been released.
The unchanged Source census is 1,811 files / 15 owned / four new / two production /
106 cumulative versus accepted Main. M92 eight plus M93 nine
Rust identities remain 17 headless and 17 overlapping renderer declarations,
zero Python. These counts and failed-profile observations are not corrected
passes. Corrected final tree, qualified commit and all execution outcomes are
unknown. Preserve the original seven-file formatter only for five unchanged
Rust copies; the two corrected test leaves need separate current formatting.

## Closed behavior

Keep the existing prepare/spawn signatures, one body/collider/BodyId and four
logical nodes: model, identity link, collision, visual. Visual ID remains
model ID plus three. Existing Box/Sphere/Mesh visual admission is unchanged;
collision support does not add Cylinder/Capsule visuals.

Resolve caller-DOM collision pose in the existing link frame. For a newly posed
primitive, pass that local pose, the requested model/link world pose and resolved
link inertia to the existing shape-specific posed constructor. Collision center
is link position plus link rotation times local translation; local collision
rotation must not rotate that translation again. Cylinder's existing constructor
owns its single axis correction. Sphere local rotation is geometrically symmetric
but remains validated and reported metadata. Link/native origin and visual frame
remain distinct from the collision center.

The new branch requires supplied/resolved centered inertia and the existing
constructor's finite, normalized-pose and representability domain. Requested
mass and link inertia remain the physical mass/COM/tensor inputs; child density
does not infer a shifted mass frame. Static and dynamic cases retain their
existing body semantics. No offset-COM or tiny-COM expansion is selected.

The newly admitted posed branch requires an actually passive source, including
when caller DOM removes a real VelocityControl plugin. Keep existing valid
loaded and owned-runtime-singleton source ownership routes; do not add a
loaded-only predicate. Put the actual controller-owner veto only in the new
branch. Preserve the exact identity primitive
`spawn_link` dispatch, its mature COM behavior, and the mesh/compound caller-DOM
policies. Caller-resolved parameters are intentionally copied; do not assert
they equal the original loaded geometry. Actual runtime compounds remain refused
independently of supplied DOM count. This phase does not add immutable singleton
child-geometry stamps or broaden existing ownership/binding guarantees.

Preflight all newly reachable pose/inertia/numeric/native construction failures
before publishing ECS nodes or consuming logical IDs. Preserve existing foreign
world/template ownership, name limits, pending inputs and camera capacity rules.
Use the existing world_edit, initial/reset and removal paths without widening
their admission domains or public APIs.

## Acceptance and retained tests

The corrected physical inertia witness retains the original mixed force
`[2,-4,6]`, torque `[6,8,4]`, pause and strict linear response, with complete
native-state equality between density twins. Its finite-step angular response
includes default gyroscopic coupling; the old `torque * dt / I` vector was an
invalid multi-axis post-step premise. Require finite positive `torque dot omega`,
then independently restore both twins to the same stationary ModelState before
each world-principal-axis torque: `[6,0,0]`, `[0,8,0]`, `[0,0,4]` under the existing
cyclic rotation. World inertia is `(4,2,3)`, so expected angular velocities are
`[.0015,0,0]`, `[0,.004,0]`, `[0,0,.0013333333333333333]` at `1e-11`. Keep the original
linear vector, full density twins and isotropic resolved-default tail. Do not
disable gyro, loosen a bound or alter product physics.

- Cover all four shapes, static and dynamic bodies, centered explicit inertia,
  noncommuting link/collision rotations, copied parameters, density variation,
  and independent link/collision/visual frame composition. Preserve valid passive
  loaded and owned-runtime-singleton source preparation; actual runtime-compound
  sources remain refused regardless of caller DOM count.
- Read and set a committed six-axis target through the real public APIs; check
  the returned committed target and fresh native state without stepping. Query
  immediate geometry after spawn and edit, accounting for every body and child.
  Retain the cylinder 3.6 chord and curved-surface bounds at `1e-11`, nearest hit
  and whole-body exclusion. Use wide finite bounds for generic transformed rays.
- Use a separate exactly representable Sphere endpoint: identity body/child
  rotations, binary radius/positions, inclusive max 1 and miss at .999. Inside
  distance zero still needs a positive finite query maximum. Do not assume an
  arbitrarily transformed mathematical endpoint is bit-exact.
- Prove real positive-gap contact against a separated countercontrol with a
  justified existing horizon. Preserve source removal/template survival,
  ordinary and loaded reset, removal/survivors/high-water IDs and both native
  and logical atomicity. Exercise the real 128-model logical-camera capacity
  refusal and recovery without numeric allocator seed APIs.
- Follow the first reachable Source guard for every typed refusal, including
  TPE's existing `InvalidScene` route. Check invalid/late inputs, foreign worlds,
  actual controlled sources with stripped caller plugins, COM and closed visuals.
  Faulting steps stay terminal; repair before a step in an independent healthy
  world rather than assuming reset recovers a faulted simulation.

Root alone changes exactly the two existing translated/rotated collision-pose
refusal rows in `tests/primitive_runtime_spawn.rs` to same-XML positive
prepare/spawn/geometry/ownership witnesses. Keep their case identities and every
other negative, numerical bound and assertion strength. No other old test body
is leased to this phase.

## Source leases and gates

Current corrective leases are test-only: Package may change only the selected
inertia case in `tests/posed_primitive_runtime_spawn.rs`; Server may change only
the selected physical traction case in `tests/compound_wheel_diff_drive.rs`.
No new declaration, product edit or retained-row edit is selected. All other
fifteen new case bodies, helpers and earlier input dispositions remain exact.
The original feature leases below are historical scope, not new authority to
change production during this correction.

Package owns only `crates/gz-sim-scene/src/world_spawn.rs` and the new
`crates/gz-sim-scene/tests/posed_primitive_runtime_spawn.rs`. No fixture is planned.
Server independently owns the M93 lib predicate and new wheel tests. Root owns
the three shared retained-test files across both phases, integration, records and
serial heavy releases. Scope owns this guide, M93's guide, AGENTS.md, README.md,
DELIVERY_PLAN.md, HANDOFF.md, IMPLEMENTATION_TASKS.md and delivery-phases.json.
Authors reciprocally review whole lanes; Scope independently reviews joined code
and Root reviews authored docs and integration.

Complete implementation, meaningful regressions, docs and all independent Source
reviews; batch findings before one affected formatter and one consolidated full
normal/staged-hook/strict workspace, per-crate, actual-parent and retained-ancestor
line-and-region coverage cycle. Every production coverage gate remains strictly
greater than 95 percent. Independent local DATA, ordinary review publication,
exact hosted CI and independent official DATA remain required, with all original
predecessor and M77 browser/shutdown/release criteria. No per-edit checks or old
qualification replay. Failures require a complete diagnosis and reviewed batch
before any affected rerun; no predicted pass is evidence.

No world_edit/initial/Physics/Server/transport/public API, renderer, package,
posed LINK, visual-shape, sensor, controller, TPE, mesh/plane or native-reference
campaign expansion is selected. Rapier f64 and TPE remain; DART/MuJoCo are retired.
Accepted phases/slices remain 76/202, catalog 93, finite examples 47, CLI cases 21
and Latest M70's original 41-example package. Keep no-expiry demo 86363 and Fuel
untouched. No stop, restart or automatic expiry is authorized.
