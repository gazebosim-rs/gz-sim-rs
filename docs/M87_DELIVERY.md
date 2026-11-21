<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M87: Passive revolute graphs with Rapier compound links

Deliver the EXISTING passive revolute-joint semantics when participating Rapier
links use two to sixteen admitted primitive collision children. Reuse the current
body, joint and query construction; add no joint type, actuator or solver path.

## Basis and shared delivery

M86 and M87 are disjoint Source lanes in one D31 candidate. Each is budgeted at
most five working days including independent review and mandatory verification.
The actual parent is locally qualified, unaccepted M85
`22cffebd3d1f9f0be82e7ac35e557b42d08edea9`, tree
`bf83012768dde6feb1af7e402853d3dd841e3d50`, parent M84 `ba2d85ee`.
Its corrected full normal and independent retained DATA passed: 1904 headless,
1440 overlapping renderer, two ignored each, all seven own controls in both
profiles, Scene changed lines 15/15 in both, 494 aggregate Python, 47 finite
examples, 21 CLI cases and nine strict bases. Preserve the first M85 normal's
14/15 strict-coverage failure and its original Source/packet; do not relabel it.

M85's ordinary review branch is published. Its own hosted run 44883777137
attempt 1 genuinely succeeded (`bd7e54ea`). Original collection then returned
Root zero (`b1495abe`), with all 30 requests started, reaped and zero and durable
individual original ZIPs (`10dc6f99`). Independent official DATA then genuinely passed: report `bab7fc31`, receipt
`300b40da`, four original RAW groups 442/734/57/168 compared once and nine strict
bases, with the actual all-zero new-branch CI base decoded from the original
log. M85 completed its own qualification but remains unaccepted. M84 also
completed its own qualification (`6304ac77`/`6fc783e1`); no proof or acceptance
transfers to this shared candidate.

The first shared candidate completed implementation, regression Source,
independent code/docs review and one six-path formatter, then FAILED its only
full normal: Root/Git 1 after 1405.724398 seconds, 1915 headless passes, four
failures and two ignored. Eleven of 15 new cases passed, including all eight
M87 cases; Root's three retained-test transitions also passed headless. Renderer
and strict qualification were not reached. This does not qualify either phase.
Preserve frozen tree `ca8a0614`, failure receipt `865ca8d0`, original Source,
packet and formatter `78be4b89`; no qualified commit was created.

The first isolated four-failure correction batch retained: compare sorted complete
identity/entity pairs rather than archetype iteration order; retain the posed
cylinder's distance-one oracle at 1e-11 under a wider existing query bound while
an axis-simple binary sphere proves exact inclusive 1 and short 0.999; use the
EXISTING 1e-12 identity predicate for computed link-local poses, keeping every
attached-child local and ownership/member pin exact; retain the 0.25 COM loader
refusal and use an admitted 1e-13 singleton COM with an exact nonzero-shift oracle.
Change no generic synchronization, setter, API, loader admission, numerical
tolerance or coverage threshold. All M87 and other feature Rust stays unchanged.

The second full normal on corrected tree `2d65c793` then failed Root/Git 1
in 1399.021763 seconds (`b1619de7`): 1918 headless passes, one failure and two
ignored. Fourteen of 15 new cases passed, including all other M86 controls,
both private controls and every M87 case. Renderer and strict qualification
were not reached. Preserve that frozen Source, packet and failed result alongside
the first candidate; neither has a qualified commit.

The sole remaining cause is the strict cylinder chord ray's placement. For four
or sixteen children, its origin lies inside the adjacent box: a solid-union query
correctly returns zero. One existing-case correction casts along world Z at
radial Y offset 0.3, retaining the same collider geometry, analytic distance 3.6,
maximum 5 and 1e-11 tolerance. Keep the old ray as exact Some(0) for counts four
and sixteen, and near 3.6 for count two. All later case oracles were independently
reviewed. Package's one-file patch is complete; Scope's independent exact inverse
recovers the entire frozen file. No production code, other case, tolerance,
coverage threshold or admission changes.

Confirmed inventory remains 1799 files, 14 owned paths, four new paths, three
production paths and 93 cumulative paths; seven M86 (five external/two private)
plus eight M87 declarations remain 15 unique H/R obligations, all overlapping,
zero Python. These are Source counts, not executed corrected results. Root
independently reviews this eight-doc checkpoint before one affected single-file
formatter and the required full retry. Reuse previous formatter `b075001c` ONLY
for unchanged `initial.rs`; reuse original `78be4b89` ONLY for its four unchanged
Rust copies. Preserve both complete receipts without claiming a formatter union.
Current one-file formatting, tests, lint and retry are UNRUN; final tree, commit
and outcomes remain NULL. One consolidated full staged-hook, normal/strict
workspace/per-crate, local DATA, review publication, exact own hosted and official
DATA cycle must qualify the shared successor. Every original own, predecessor
and M77 gate remains mandatory. The possible next development wave is unselected.

Accepted Main remains M76 `dddf1f42`: 76 accepted phases and 202 slices. Catalog
is 87; Latest M70 retains its original 41-example package. Finite/CLI inventories
stay 47/21. Keep no-expiry M77 demo 86363 live: formal browser evidence and an
owner-confirmed shutdown still hold acceptance. Preserve all frozen trees,
failed originals and Fuel work; DART/MuJoCo remain retired. Root alone releases
serial heavy operations. No check or runtime operation is released here.

## Closed joint composition

The existing Scene compound branch rejects every model containing joints. Replace
that blanket restriction with the selected passive composition boundary. Existing
joints::validate_model remains authoritative: Revolute only, no axis2, supported
top-level dynamic non-nested models, current world/local endpoint resolution,
one parent per child, no self endpoint or cycle, and the existing axis, limits
and zero-dynamics rules. Preserve supported passive link poses/model anchors,
centered resolved compound inertia, self-collision groups and one owned body /
collider / BodyId per link. A compound at either endpoint does not create child
bodies or change the joint origin.

Removing the no-joints check alone would also admit unselected actuator chains.
Explicitly require both existing JointController and DiffDrive plans absent for
EVERY model containing any compound link, including mixed singleton/compound
endpoints. The lib.rs controller_plan variable denotes JointController, not
VelocityControl. Keep VelocityControl's separate no-joints rule unchanged; M85's
free compound controller remains valid. No motor/reduced-controller expansion,
command route, limits/axis/dynamics change or new native parity is selected.

Use unchanged joints.rs/backend construction, committed sample/binding,
removal ordering and initial geometry/joint restoration. Existing torque/impulse
capability boundaries remain authoritative: a world-anchored effort witness does
not qualify moving-parent effort support. Add no Physics, joint public API or
Server IO change. Plane/Mesh/nested compounds, offset COM, TPE joints/compounds,
static joint models and other unsupported domains remain refused.

## Meaningful acceptance and exact old transition

Root replaces only the compound model's passive world-revolute refusal row in
compound_primitives.rs with a stronger same-XML positive constraint, sample and
geometry control. Keep every other old negative, case identity, existing analytic
oracle and tolerance; M86 owns a separate getter/setter transition in that same
file through Root integration. Do not erase old coverage through table broadening.

New controls must exercise a world-anchored compound body and supported
parent-child passive links with compound and mixed endpoints, resolved/nontrivial
frames, signed angle/axis/sample time and independent real motion/constraint
witnesses. Verify unchanged explicit inertia with a supported torque/impulse
control and retain all unsupported effort boundaries. Check per-child analytic
rays/strict cylinder chord/concave gaps as bodies move, real contact plus a
countercontrol, same-model exclusion versus distinct models, and no shape-cache
mutation. Select justified physical observation windows before execution.

Cover ordinary, prepared-geometry and loaded resets without inventing new Scene
or native reset modes; retain geometry, joint/body/logical IDs and high-water
marks. Prove removal ordering and unrelated survivors, malformed graph/endpoints,
unsupported joint types/axis2/limits/dynamics, late invalid child admission and
foreign binding atomicity. Explicitly refuse JointController and DiffDrive on
compound or mixed endpoints, including attempts to hide ownership by link order;
keep VelocityControl joint refusal, compound world-edit joint refusal and
runtime-spawn refusal intact. Existing singleton graphs and all retained M84/M85
controls remain unchanged outside Root's exact three-assertion disposition lease.

## Independent work and unchanged boundaries

Package owns M86 Scene `src/world_edit.rs`, the narrow `src/initial.rs` blueprint
helpers, and new `tests/compound_world_edit.rs`. Server owns M87 Scene
`src/lib.rs` admission and new `tests/compound_joints.rs`. No new fixture file is
selected. Root alone owns the disjoint old-test transitions in
`tests/compound_primitives.rs`: the passive compound getter/setter pair for M86
and the passive compound revolute refusal for M87. Keep the same XML, existing
analytic oracles/tolerances and every unrelated negative. Neither lane edits
that shared file. Root integrates; Scope owns these two guides and the current
README/AGENTS/DELIVERY_PLAN/HANDOFF/IMPLEMENTATION_TASKS/planning blocks.

Authors independently review the opposite lane; Scope reviews the complete
integrated code, mathematics, lifecycle and retained-test integrity; Root
independently reviews authored docs and the whole release batch. No per-edit
formatter, test, lint, probe or targeted trial. Preserve the real strict coverage
thresholds and all required jobs. Retain unchanged cylinder/capsule and original
M79 component Source references only within their original limitations; add no
native campaign, solver parity or native manager-reset claim.

No new Physics/backend API, Server/transport/protocol, renderer/package/browser,
geometry, automatic inertia, dependency, framework, DART/MuJoCo or Fuel expansion
is selected. Compound runtime spawning stays refused. The live demo is protected
from stop/restart, port or process changes. Runtime/controller changes are not an
implicit lease of either phase.
