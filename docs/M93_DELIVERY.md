<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M93: primitive compound differential-drive wheels

M93 admits flat 2–16 primitive collisions on either or both wheel links of the
existing validated DiffDrive model, alongside its compound canonical chassis.
It shares one D31 candidate and consolidated qualification with
[M92](M92_DELIVERY.md); each phase has at most five working days including review
and mandatory checks.

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

Change only Scene's compound Drive admission predicate. Allowed wheel links come
from existing `Plan.names` through the already validated model joints; do not
guess names or admit arbitrary links. Preserve the canonical chassis path.
One body/collider/BodyId still owns each link's complete compound.

All existing controller validation remains authoritative: exactly three links,
two Revolute joints, canonical frame, +Y wheel axes, identity wheel-link rotations
and joint poses, finite symmetric wheel translations, positive effort/speed
limits, owner bounds, topics, rate and command limits. Wheel radius and separation
are supplied controller parameters, not inferred from collision geometry.
Existing motors, encoder sampling, odometry and lifecycle bodies remain unchanged.
This composition adds no every-child immutable metadata-binding promise.

Keep the flat 2–16 existing four-primitive compound domain, resolved centered
inertia and one link mass frame. No offset COM, topology, axis, wheel-pose,
runtime-joint-template, sensor, plugin or public API expansion is selected.
VelocityControl still refuses joints, world-edit still refuses jointed setters,
and runtime template preparation still refuses drive models. TPE stays closed.

## Acceptance oracles

- Cover left-only, right-only, both-wheel and all-three-link compounds with
  2/4/16 members. Preserve the current singleton wheel parameter controls.
  Vary member density against fixed explicit centered inertia and finite effort.
- Preserve the original Cylinder rim radius .2 / half-length .02 and local
  Rx(-pi/2), centered interior Sphere hub and every additional interior member.
  Keep all original 2/4/16 fixtures, all three commands and 1,000-step windows.
  Forward/reverse native and odometry driven-minus-quiet displacement remains
  greater than .01; suspended translation remains below `1e-7`. The original
  pure-turn result remains a finite contact observation, with binding, frame and
  sample checks; the exact contact cause is UNKNOWN. Do not claim masked wheel
  encoders, odometry or later counts passed. Do not tune width, gains or horizon.
- Add a separate round Sphere rim radius .2 with a strictly interior Cylinder
  hub, retaining requested mass/tensor, link/joint frames, declared radius,
  pure-turn command `(0,1)` and 1,000-step horizon. For every 2/4/16 count, preserve
  strict native
  yaw, odometry and opposed-encoder turn response greater than .01 against quiet
  and suspended controls. This mechanically justified point-contact witness is
  distinct from the original finite-width Cylinder fixture; no pass is predicted.
  Keep strict forward/reverse controls in the original Cylinder loop. Quiet contact may
  settle; do not assume ideal motor speed or no-slip odometry/body equivalence.
- Keep the independent spatial all-four-shape geometry matrix. Interior hubs
  need not have separately visible outside surfaces; do not reuse their traction
  fixture for all-child surface assertions.
- Observe finite signed native encoder increments. Derive independent SE(2)
  odometry and publication cadence from those measured increments; retain strict
  `1e-11` numerical assertions. Use equivalent angular-target native-state and
  encoder twins for declared-radius, clamp, latest/zero and invalid-command
  controls, rather than ideal target trajectories under soft finite-force motors.
- Use a separate spatially observable 2/4/16 all-four-primitive matrix for child
  frames, immediate global queries, strict 3.6 cylinder chord at `1e-11`, nearest
  hit, positive-bound inside zero and whole-wheel-body exclusion. Account for
  chassis, both wheels, all children and ground. Generic transformed endpoints
  use wide finite caps and tolerance; exact max 1/.999 is a separate binary
  identity Sphere witness. Excluding one wheel does not exclude the robot.
- Preserve pause/latest/pending/invalid timing, healthy ordinary and loaded
  resets, backend prepared-reset geometry/ID scope, partial removal, surviving
  owners, high-water IDs, eight owners and ninth refusal. A bare backend reset
  does not prove Scene pending commands cleared. Binding drift stays within
  existing config/joint/native-owner checks. Faulting steps are terminal;
  before-step repair in a healthy independent world is a different control from
  reset after fault. Include late-owner atomicity with existing good pending work.

Root alone changes the remaining same-XML right-wheel extra-Sphere row in
`tests/compound_joints.rs` and the first left-wheel extra-collision tuple in
`tests/compound_diff_drive.rs` into real binding, isolated-query and causal
encoder/native positives. The old right-wheel Sphere starts .05 into ground and
has an eccentric arm: compare its same-XML quiet physical response, without
claiming that fixture is a round traction rim. Preserve all earlier branches,
later VelocityControl/state refusals, case identities, geometry and every other
closed-domain tuple. M92's two old-row dispositions are separate Root leases.

## Source leases and remaining gates

Current corrective leases are test-only: Package may change only the selected
inertia case in `tests/posed_primitive_runtime_spawn.rs`; Server may change only
the selected physical traction case in `tests/compound_wheel_diff_drive.rs`.
No new declaration, product edit or retained-row edit is selected. All other
fifteen new case bodies, helpers and earlier input dispositions remain exact.
The original feature leases below are historical scope, not new authority to
change production during this correction.

Server owns only `crates/gz-sim-scene/src/lib.rs` at the compound Drive predicate
and the new `crates/gz-sim-scene/tests/compound_wheel_diff_drive.rs`. No fixture is
planned. Package owns M92 world_spawn.rs and its separate new test file. Root
owns the three retained-test files, integration, records and serial heavy jobs;
Scope owns the two new guides and six current canonical docs/planning files.
Reciprocal whole-lane review and independent joined Source review precede checks;
Root independently reviews authored docs and the integrated candidate.

Finish the complete implementation/regression/docs batch, gather findings, then
one affected formatter and consolidated full normal/staged-hook/strict workspace,
per-crate, actual-parent and retained-ancestor line/region coverage cycle. Keep
every production threshold strictly greater than 95 percent. Independent local
DATA, ordinary review publication, exact hosted CI and independent official DATA
remain mandatory; no acceptance, predecessor or M77 browser/owner-shutdown/release
criterion changes. No per-edit tests, trials, native campaign, solver or custom
framework is selected. No Physics, Server, transport, renderer, package, controller
runtime or public API changes are leased. All future results remain unknown.

Preserve all first 91 historical phase objects and prior groups. Accepted phases
and slices remain 76/202; catalog 93, finite examples 47, CLI cases 21 and Latest
M70's original 41-example package are unchanged in scope. Rapier f64 and TPE stay;
DART/MuJoCo remain retired. Keep demo 86363 live without expiry and preserve Fuel
and frozen qualification trees. No stop or restart is authorized.
