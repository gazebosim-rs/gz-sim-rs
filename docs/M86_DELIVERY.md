<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M86: Loaded Rapier compound model-state edits

Deliver fresh state reads and explicit passive state edits for a loaded Rapier
model with one identity-local link, two to sixteen admitted primitive collisions
and exactly one existing supported visual. Extend the existing ModelState API;
engine stepping and physical actuator commands retain their separate semantics.

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

## Closed original-membership admission

Keep existing singleton behavior, including COM velocity shifts, exactly intact.
The new domain is loaded M84-style flat primitive compound geometry, one original
body/collider/BodyId, centered original inertia, a top-level model with one
identity-local link and one visual, no sensors, nested entities or joint
participation. Existing singleton visual kinds remain the supported presentation
subset; add no visual or geometry kind. Preserve all current representability,
backend-owner, body-owner, public-ID, parent, leaf and anchor checks.

Widening the current descendant count alone is unsafe. A live sensor relabelled
Collision must not promote an originally singleton body into this capability;
a removed/relabelled compound member must not downgrade into the old singleton
path. Route by the IMMUTABLE loaded original domain, then verify current
membership. A narrow pub(super) InitialScene helper/classification uses existing
Node records; do not expose Node or build another registry/snapshot framework.

Require the original model/link relationship and BodyId, original 2..16 Collision
children plus exactly one Visual and no other original child kinds. Match the
COMPLETE live logical child-ID set to that original membership, not merely its
count. Check original reservation/World/allocator continuity and current
entity_by_id/EntityId mapping, kind, parent, Name/LocalName, AttachedPose and leaf
semantics. Both original and current LINK retain the exact EXISTING identity
predicate (POSE_TOLERANCE 1e-12). Its LocalPose is a legitimately mutable computed
value: ordinary reset/commit recomposes inverse(parent world) times link world
and can introduce f64 residuals. The new bitwise LINK-local pin was therefore
incorrect; add no tolerance and change no synchronization or setter. Pin every
AttachedPose child LOCAL translation/quaternion field EXACTLY to the original.
Require the original centered COM representation and the current matching value.
Bind visual geometry to the captured immutable Arc clone. InitialPhysics owner
and the original BodyId retain the actual loaded native compound construction;
public ECS labels cannot substitute for that evidence.

Do not compare model LocalPose or any WORLD pose with its initial value: legitimate
motion and edits change them. Do not require original ModelAnchor presence:
editing a loaded fixed model legitimately adds an identity anchor. Keep existing
anchor checks and resolve current entities by restored logical IDs after loaded
reset, never by an obsolete Bevy Entity generation.

## Reads, writes and spawn boundary

Reads may include an exactly admitted sensorless VelocityControl compound,
consistent with existing singleton getter semantics. Writes remain passive-only:
retain set_model_state's immutable blueprint/live owns_model veto, including when
public controller configuration is removed or changed. Prove getter-positive /
setter-UnsupportedModel separately from direct-velocity ownership protection.
The existing M85 sensor-rich getter refusal remains valid and unchanged.

Reuse the existing read/visual-projection/transactional setter. Precompute EVERY
attached world pose, finite/f32 presentation bound and native state validation
before mutation; commit one native state and all matching child frames without
stepping, query-cache refresh, IO or allocator changes. Keep ordinary failure
atomicity, unrelated bodies, clocks, identities and initial reset records.

RuntimeBodyOwner models and admitted_entities's fixed [Entity;4] result remain
EXPLICITLY singleton-only; never truncate a generalized compound attachment list.
prepare_model_template's existing collision-count guard continues refusing
compound templates even when a loaded compound state read is now valid. Change
no world_spawn implementation or template/public API. Prove that existing
singleton spawn and loaded-reset cleanup still work and compounds stay refused.

## Meaningful acceptance and exact old transition

Root replaces only the passive owner-domain compound model_state and
set_model_state UnsupportedModel assertions in compound_primitives.rs with
stronger same-XML positive read/edit/read controls. Keep the singleton positive,
VelocityControl ownership, all adjacent negatives, current case identities and
existing 1e-11 oracles. Add no test retirement or numerical relaxation.

New controls must include counts two and sixteen with all four primitive kinds,
noncommuting/offset child frames and concave gaps; static and dynamic six-axis
read/edit/read state; immediate independent child rays, inside/inclusive maximum
and whole-body exclusion without a step. Preserve the strict cylinder 3.6/1e-11
oracle and all-child query-error behavior. Test late attached-pose/native numeric
failure atomicity, foreign/misclassified/extra/missing/reparented/re-IDed members,
local-pose/COM/visual-Arc/attachment/leaf drift and both original-domain upgrade
and downgrade forgeries. Every refusal preserves native/ECS/IDs/time.

Cover ordinary and loaded reset, fixed-model anchor creation, removal/survivors,
logical-ID restoration and high-water behavior. Test the sensorless controlled
read/write distinction and configuration-removal veto; do not use a sensor-rich
getter outside this selected domain. Compound construction, controller command
scheduling, joint semantics and standalone/runtime-spawn hierarchy stay unchanged.

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
