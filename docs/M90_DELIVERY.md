<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M90: Passive runtime primitive-compound templates

The new DOM-compound route prepares and spawns one passive, original loaded Rapier
compound through the existing public `prepare_model_template` /
`spawn_model_template` signatures. A model has one
identity-local link, two to sixteen admitted primitive collisions, one already
supported visual and centered resolved link inertia. One native body/collider/
BodyId owns the entire compound. This is a bounded ownership/lifecycle increment,
not a new public template protocol or general ECS hierarchy editor.

## Basis, cadence and qualification

The genuine development parent is locally qualified, unaccepted M88/M89
`1e54399a4b1fbf565b64f3ddc8e35a1a8439137f`, tree
`e648b4bfaedb9dcfe6b6937c83ea1b8647f30397`, parent M86/M87
`ee4df6c5851cc40b77a15b19c6e992434ee7bfff`. Corrected normal `887048c8`
and independent retained DATA `a7c8e550` / report `c82e1e4f` genuinely passed:
1933 headless and 1469 overlapping renderer passes, two ignored each, all fourteen
own controls in both profiles, 494 aggregate Python, 47 finite examples, 21 CLI
cases and eleven strict coverage bases; Scene changed coverage is 9/9.
Ordinary review publication `1a2931b5` passed. Exact automatic simulator CI
44908259208 attempt 1 genuinely succeeded (`7ea4fda3` and three saved API responses).
Root's collection and independent official audit now passed once: collector
`839dba87` / collection `bce9739a`, official actual `e3402e39` / report `fb08e276`.
Four prescribed end-only RAW comparisons covered 446/738/57/168 originals once.
This completes the parent's own qualification, not acceptance or M90/M91 results. Preserve the failed first run `0361f70b`, its typed
TPE-error premise diagnosis, and separate original-five/reused-four/current-one
formatter custody. Parent M86/M87 completed its own official audit
(`d781710d` / actual `6492532b`) and remains unaccepted.

Each phase has at most five working days including review and mandatory gates.
M90 and M91 form one D31 candidate with disjoint product/new-test leases. Complete
implementation, meaningful regression Source and docs; perform reciprocal and
whole independent Source review, batch all findings/fixes, then one affected
formatter and one consolidated mandatory staged-hook/full normal/strict workspace,
per-crate and required-ancestor coverage cycle. Independent local DATA, ordinary
review publication, exact own hosted CI and independent official DATA remain
required. Root alone integrates and releases serial heavy operations. No per-edit
tests, lint, product imports, trials, probes or extra check rounds.

The first full normal failed compilation at six nonexistent `Simulation::step`
uses (`f9d43298`): Root/Git 1, 99.52633646101458 seconds, no tests or coverage.
Author, reciprocal, whole Source and Root reviews all missed those calls. The
six-call test API repair and its two-file formatter passed Source review, but the
second full normal also returned Root/Git 1 (`fc5dce53`) after
1591.1281296920206 seconds: 1,942 headless passes, eight failures and two ignored
cases. Ten of seventeen new controls passed that profile; seven new and one
retained case failed. Renderer and strict coverage were not reached; there is no
qualified successor. Both frozen candidates, raw packets and actual failures stay
intact. Partial passes do not qualify the feature or transfer to a correction.

All three test corrections are complete and integrated. Reciprocal, Scope and
Root whole independent code, API, mathematical, lifecycle and exact inverse
reviews are CLEAR; Root independently reviews this final eight-doc batch.
Package reviewed the whole M91 drive file and retained row; Server reviewed the
endpoint and retained row; Scope reviewed all three complete patches and the
joined Source. No corrected executed result or predicted pass is claimed.

The endpoint control uses a separate caller-DOM sphere with identity orientation,
retaining the original mixed rotated template and exact max/short-bound, inside,
exclusion and strict 1e-11 oracles. Drive tests use finite measured encoder travel
against quiet worlds, actual two-encoder SE(2) odometry with 150 publications and
ten-interval twist at 1e-11, equivalent latest/clamped/zero and declared-radius
command twins, terminal-fault refusal plus healthy-before-step repair, and signed
ground response against quiet/suspended controls. A separate airborne bumper
witness begins with a positive .006 gap and measures motor reaction and contact
within the unchanged 1500-step horizon, with quiet/distant controls at 1e-7.
The same-XML retained chassis row adds paired quiet encoder travel and independent
physical response without attributing settling solely to the motor.

These are explicit reviewed dispositions of new, never-passed invalid oracles:
one-second setpoint convergence, ideal wheel symmetry/absolute quiet immobility,
reset after terminal fault, and the unsupported ground-contact premise. All
other assertions, strict geometry/tolerances, case identities, the three already-
passing M91 bodies, old helpers and closed-domain tail remain exact. No tolerance,
production behavior, public API, solver or mandatory gate is weakened or changed.

Actual joined Source is 1807 files, 16 owned paths, four new paths, three production
paths and 102 cumulative paths. M90 retains seven public plus one inline private
control; M91 retains nine public controls: seventeen unique H/R declarations,
overlap seventeen, zero Python. Finite examples remain 47 and CLI cases 21. These
are Source obligations; the old partial passes remain bound only to failed 1ddb.
The complete correction has no executed test, coverage or qualification result.

The current affected formatter is ONLY the three test leaves
runtime_compound_spawn.rs, compound_diff_drive.rs and compound_joints.rs, and has
not run. Intermediate `e15f2983` formatting is reused ONLY for the unchanged
world_spawn.rs copy, with its `53b83b86` review and `0c42c616` snapshot. Original
`d52bee60` formatting is reused ONLY for four unchanged copies: lib.rs,
world_edit.rs, compound_world_edit.rs and primitive_runtime_spawn.rs, with
`ab497fbb`/`3eda5af6` custody. These are disjoint physical copies, not a formatter
union or qualification claim. Current formatter, full retry, local DATA and hosted
outcomes are UNRUN/NULL; final corrected tree and qualified commit remain NULL.

No parent qualification or acceptance transfers. Preserve the first 89 canonical
phase objects and old parallel groups as historical observations. Accepted phases
and slices remain 76/202, catalog becomes 91, finite/CLI stay 47/21 and Latest
M70 keeps its original 41-example package. Every predecessor and M77 browser/
owner-confirmed shutdown gate remains. Demo 86363 has no expiry: no stop, restart,
new demo or port change is authorized. Preserve Fuel and frozen trees.
DART/MuJoCo remain retired; no unrelated backend work is selected.

## Closed template and ownership contract

Use only existing Box, Sphere, Cylinder and Capsule `CollisionSpec` shapes and
resolved local poses; the private template collider may hold their bounded vector.
Reuse the existing backend `spawn_compound_link` and M80 cylinder child-query
route unchanged. Preserve singleton primitive and mesh branches, centered
singleton identity-pose admission, public signatures, and singleton
`admitted_entities -> [Entity; 4]` exactly. No Physics, lib.rs, initial.rs, core,
CLI, renderer, dependency or query-kernel edit is selected.

Only the NEW caller-DOM compound branch (2–16 collisions) requires an actually
admitted original loaded passive compound in this Scene. Use existing immutable
original-domain checks and refuse actual VelocityControl ownership even when a
caller removes its plugin from DOM. Preserve the mature caller-DOM SINGLETON
route over actual LOADED compounds exactly as parent, including pre-existing
stripped-plugin/caller-parameter semantics; no global singleton passive veto is
added. An actual RUNTIME compound is a newly possible source family and refuses
retemplating after `model_state` independently of caller collision count. The
regression uses a genuinely resolvable generated singleton DOM to prove this path.

No new compound-route jointed/multilink/nested model, sensor, plugin/controller,
TPE, compound Mesh/Plane, offset COM, automatic inertia computation or visual
shape is admitted.
The one visual remains existing Box/Sphere/Mesh. Do not impose new XML-presence
rules: honor existing resolved mass/inertia defaults within the centered domain.
Requested link mass and supplied link inertia own physical mass/COM/tensor;
child density is metadata, never a newly inferred mass sum.

Retain existing caller-resolved SDF parameter semantics. The DOM supplies copied
geometry/inertia while the admitted Scene supplies its visual Arc/identity; do
not claim all supplied parameters equal original loaded geometry. Commit and pin
the actual immutable template parameters used to construct each runtime body.
Model/world poses remain mutable through the admitted state API; link-local
identity uses the existing predicate, not a new bitwise synchronization pin.
Child local poses, membership and ownership metadata remain exact.

Preflight EVERY child, pose/density, generated name, native numeric boundary,
logical u64/u32 range, all world presentations and existing 128-model camera demand
before native or public-ID allocation. N members require N+3 nodes, allocated as
model, link, first collision, visual, then remaining collisions. Thus
`visual_id == model_id + 3` and the four-node singleton prefix stay compatible
with existing CLI lifecycle/snapshot code. No fallible ordinary refusal may leave
a body, partial node set, consumed logical IDs, changed survivors or time behind.

A private immutable runtime-compound stamp must be discoverable from the original
model independently of current hierarchy counts. Pin exact model/link/body and
logical EntityId/ReservedId-to-entity mappings, complete member set, kinds,
parents/names, child LocalPose/AttachedPose/leaf status, center and visual Arc.
Reject shrink-to-singleton, relabelled sensors, foreign/reparented/fabricated
children, swapped native handles, lost reservations/markers or altered geometry
identity. Route original loaded compounds first, stamped runtime compounds next,
old singleton binding last: damaged originals must not fall through to a weaker
domain. Runtime singleton stamps and COM-shift behavior remain unchanged.

Use a separate private bounded owned-node validator for runtime-compound reset
preflight. Keep the fixed singleton helper intact. Existing loaded-reset variable
BTreeSet/removal handling consumes the validated set; no initial.rs helper or new
reset API is selected. Ordinary reset preserves committed geometry/IDs and clears
motion as before. Loaded reset removes validated runtime clones and restores
loaded originals without ID reuse. Removal must retire the whole owned hierarchy
and native body while preserving unrelated worlds, survivors and high-water IDs.

## Meaningful acceptance and exact old transitions

`tests/runtime_compound_spawn.rs` now has seven public declarations, with one
inline private ownership/marker control in `world_spawn.rs`: eight H/R controls,
zero Python. These are Source obligations; table rows are not extra tests.
Cover 2, 4 and 16 mixed
members with translated/rotated children, distinct model orientations, static and
dynamic link mass/inertia, stable prefix IDs/visual alias, source removal and
subsequent template use. Check six-axis read/edit state and immediate independent
per-child/global-nearest geometry, strict 3.6 chord at 1e-11, cap/inside/inclusive
maximum/exclusion and concave gaps. Account for every body/child in each ray.

Use actual contacts with a separated/quiet countercontrol and a justified common
horizon, ordinary and loaded resets, removal, repeated clones, survivor/ID/native
ownership, pending input, foreign world and late invalid-child atomicity. Exercise
real camera capacity exhaustion/recovery without consuming IDs. Retain existing
numeric allocator tests; no seed API, unsafe private access or new core lease.
Private stamp guards may need bounded inline controls in the already leased
product files; report all actual declarations in the final census.

Root alone owns these same-XML retained dispositions, at their original strict
numerical tolerance and same case identities: the compound-template refusal in
`compound_world_edit.rs` (static/dynamic table) and ONLY the extra-collision closed
row in `primitive_runtime_spawn.rs` (Cylinder/Capsule table). Replace each with
positive prepare/spawn/read/geometry/ownership evidence; preserve every other
negative and assertion strength. Source-trace every typed refusal to its FIRST
reachable guard. In particular TPE preparation currently reaches missing
BodyHandle/`InvalidScene` before backend state validation; do not invent an
`UnsupportedModel` ordering or broaden production to satisfy a test.

## Disjoint leases and review

Package owns Scene `src/world_spawn.rs`, narrow `src/world_edit.rs`, and new
`tests/runtime_compound_spawn.rs` only. Those two ownership files are one product
lane; do not split their binding/stamp contract across independent feature owners.
Server owns M91's separate lib.rs predicate/new tests. Root owns the three shared
old-row dispositions and integration; Scope owns these two guides plus current
AGENTS/README/DELIVERY_PLAN/HANDOFF/IMPLEMENTATION_TASKS/planning blocks.
Each author reviews the opposite complete lane; Scope independently reviews the
whole joined code, mathematical oracles, lifecycle, first-error paths and old-test
inverses. Root independently reviews authored docs and final integration. No
native parity campaign, solver, framework, Server/transport or browser work is
selected. No weakening of existing gates or future acceptance inference.
