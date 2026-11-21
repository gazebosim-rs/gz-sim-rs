<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M91: Existing DiffDrive on a compound canonical chassis

Compose the retained differential-drive system with a primitive compound ONLY
on its already validated canonical chassis link. Wheel links remain in their
current singleton domain. Existing configuration, graph, motors, timing,
odometry and command APIs stay unchanged; this is one Scene admission predicate.

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

## Closed chassis composition

In Scene lib.rs, replace the blanket compound DiffDrive refusal only with the
predicate: no drive plan, OR this compound link is the existing validated model's
canonical chassis. Existing DiffDrive parsing and validation already establish
three distinct links, two distinct Revolute wheel joints parented to the canonical
link, canonical identity frame, allowed wheel translations/rotations and +Y axes,
finite positive wheel effort/speed, top-level dynamic Rapier and planar initial
model frame. Keep every actual current bound/domain exact. Do not invent a wheel
collision-shape or collision-radius rule absent the existing parser; supplied
DiffDrive radius/separation retain their present meaning and validation.

All compound wheel links remain refused, including a compound on either wheel
in a mixed model. Reuse retained wheel-velocity motors, effort limits and odometry
behavior and existing bound ownership/metadata identities. No new planar solver,
velocity teleport, wheel kinematics, backend joint API, friction model or native
parity claim. JointController composition remains its existing domain; preserve
actuator-owner conflicts, VelocityControl no-joints, jointed ModelState and
runtime-joint/template refusals. M90's one-link passive template rules still
refuse every three-link/plugin/jointed DiffDrive model; these lanes have no
admission dependency on each other.

## Meaningful acceptance and retained transition

`tests/compound_diff_drive.rs` now contains nine H/R Source declarations, zero
Python; no executed pass is claimed. Use 2, 4 and 16 chassis members and retained
singleton wheels with explicit centered mass/inertia. Prove commands remain
queued/latching on their existing boundaries without pose/velocity teleport;
exercise forward, turn, reverse/zero, pause/latest command/invalid input, finite
force/speed bounds and unchanged odometry cadence/message semantics.

Require actual ground traction and chassis/native body motion against suspended
or clear-contact controls. Odometry is wheel-derived and need not equal body
travel under slip; never use their equality as a new physical oracle. Account for
all chassis members, wheel bodies and ground in immediate frame/global-ray/contact
queries; retain strict geometry tolerances, inclusive bounds and whole-body
exclusion. Use a justified contact horizon and independent quiet control before
checks, not parameter trials or threshold relaxation after failure.

Cover ordinary/prepared geometry and loaded reconstruction within their actual
reset contracts, command/pending clearing only where the Scene reset provides it,
removal/partial retirement/survivor IDs/high-water reservations, foreign/native and
metadata binding drift, late invalid geometry/configuration atomicity, and exact
closed wheel/nesting/owner/topology/axis/finite domains. Preserve current motors,
binding checks and survivor state when a refusal occurs. Inspect first reachable
errors in the real parser/loader order rather than guessing enum variants.

Root alone changes ONLY the remaining M8 canonical-chassis mutation row in
`tests/compound_joints.rs` to same-XML positive binding, real motor/contact/body
motion and independent child geometry evidence at its existing tolerance. Preserve
the exact wheel mutation refusal, other M9/VelocityControl/ModelState controls,
all XML targets and the old case identity. No broad table removal or test census
invention. All inherited M80 precision and M83–M89 controls stay intact.

## Disjoint leases and review

Server owns Scene `src/lib.rs` ONLY the compound drive predicate and new
`tests/compound_diff_drive.rs`; no fixture is currently planned. No changes to
DiffDrive config/model/runtime, joints, initial state, Physics, public APIs,
Server/transport, renderer/viewer, packages, dependencies or native reference
inputs are selected. Package owns M90 world_spawn/world_edit/new tests; Root owns
the two runtime-template old rows and this one chassis row. Scope owns the eight
current documents. Keep those explicit Source leases disjoint; do not expand
production seams silently.

Each author independently reviews the opposite complete lane and Root's shared
rows; Scope independently reviews the whole joined semantics, every mathematical
oracle, first-error path and exact old-file inverses. Root independently reviews
authored docs/final integration. Batch the complete finding set before corrections
and one consolidated verification cycle. Fresh native capture, new frameworks,
DART/MuJoCo and browser/demo changes are excluded.
