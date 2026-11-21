<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M85: VelocityControl for Rapier primitive compounds

Deliver the existing model-frame VelocityControl on one Rapier link containing
two to sixteen existing primitive collisions. This composes the M79/M83
controller with M84 geometry without a new backend, command or query mechanism.
The budget is at most five working days including review and mandatory gates.

## Basis and current state

The true parent is locally qualified, unaccepted M84
`ba2d85ee2a0989ad192d224fbf76ba6ab27a51d0`, tree `6e720e7b`, parent M83
`11afe37a`. Its corrected full normal and independent retained DATA passed:
1897 headless/1433 overlapping renderer, two ignored each, 494 aggregate Python
and finite/CLI 47/21. Its ordinary review branch is now published; own hosted
CI 44876073592 attempt 1 passed. Independent official audit remains pending at
this Source checkpoint. Its first failed normal and
original evidence remain preserved. M83 completed its own local, hosted and
independent official qualification but stays unaccepted. No result transfers.

M85's first normal genuinely returned Root/Git 1 after 1394.356940572994 seconds.
Headless completed 1904 passes, zero failures and two ignored; all seven new
integration controls passed in that failed run. The strict changed-line gate
then rejected Scene 14/15 executable lines, below strictly greater than 95
percent. Renderer and strict qualification were not completed. No resource,
signal or finalization fault occurred. Freeze tree `cfa8913f`, original four-file
formatter, failed packet and actual failure `4aa09613`; no M85 commit qualified.

The sole missed changed line is `velocity_control/config.rs:181`, the error
propagation from resolving a child's pose. Public XML is already checked by
`validate_non_frame_poses` before Plan admission. The selected test-only repair
therefore strengthens the existing private configuration-boundary case: prove a
valid two-child parsed DOM, change only the final relative_to to a unique unknown
frame, require typed SDF UnknownReference and preserve the first child, empty
world and unrelated live native/controller/pending/identity state. The complete
additive block is independently Source-reviewed. It changes no production or
integration test body, old assertion, case identity, tolerance or coverage rule.

Source inventory is 1795 files, twelve own paths, two new, 89 cumulative and one
unchanged production leaf. New Rust declarations remain seven (H7/R7/overlap7),
zero Python; the strengthened old private case adds no declaration. The original
four-file formatter covers its four byte-identical Rust copies. One separately
affected private-file formatter remains unrun; this is not an invented combined
five-file formatting result. Root final authored docs review precedes that one
formatter and a complete mandatory normal/strict retry and retained audit. Current
correction tree, outcomes and qualified commit remain unbound; no pass predicted.

Initial Source preparation and the earlier solid-inside maximum0-to-0.125
correction retain their original scope. Accepted Main remains M76 `dddf1f42`,
with 76 accepted phases/202 slices; catalog 85 and Latest M70's original 41-example
package do not add acceptance. M77 browser and owner-confirmed shutdown gates
remain mandatory; keep no-expiry manual demo 86363 live. Preserve Fuel and all
frozen candidates. This phase remains budgeted at most five working days.

## Closed admission and unchanged execution

The feature changes only Scene `velocity_control/config.rs` admission. Preserve the existing
singleton branch for both retained engines. Additionally admit counts 2..=16
only when the explicit selected engine is Rapier. Resolve and validate EVERY
collision's local pose and require each geometry to be Box, Cylinder, Capsule or
Sphere. Do not validate only collision zero, infer the engine from resources,
add a new public API, or bypass the existing loader/backend numeric guards.
Empty, seventeen-member, TPE compound, mesh, plane and other unsupported domains
remain refused. Pinned SDFormat's compound automatic-inertia refusal stays
unchanged; documented resolved default Some inertia remains admitted.

The owner remains one top-level dynamic free model with exactly one link,
identity local LINK pose, centered resolved COM, no nested models or joints,
one VelocityControl owner and no conflicting controller. The passive M84
nonidentity-link domain does not widen this controller frame contract. Keep
explicit mass/inertia behavior, every child's density metadata and the same one
BodyId/collider. Child placement never changes the model/link velocity origin.

Keep the existing model-frame initial and commanded linear/angular vectors,
current model rotation into WORLD, old committed target in advancing PreStep,
new pending target latched in advancing PostStep, paused retention and finite
command refusal. Engine stepping owns physical motion; no pose teleport or demo
motion shortcut. Runtime binding, immutable initial blueprint, body ownership,
COM checks, error atomicity, reset, removal, absent-system and Server IO/epoch/
lease/queue behavior stay unchanged. No controller runtime or transport edit is
selected. Existing direct-link velocity ownership protection remains required.

Keep all M84 child-query behavior: maintained cylinder precision kernel, all-child
traversal without a shrinking maximum or early success, finite inclusive nearest
hit, solid-inside zero, whole-body exclusion, current poses after reset and
child-domain error propagation. No custom solver, compound GJK shortcut or new
universal precision claim. No Physics, Server, renderer, package, dependency,
SDK/native campaign, automatic COM/inertia, joint or sensor expansion is selected.
DART/MuJoCo remain retired; preserve Fuel and frozen qualification trees.

World-edit and world-spawn remain closed to compounds. A compound model_state
getter is structurally UnsupportedModel in that existing domain: it cannot serve
as a positive getter precondition or demonstrate the separate controller setter
ownership guard. Observe native BodyState/WorldPose and controller binding for
compound controls; retain the existing singleton getter/ownership positives.

## Exact retained-test transitions

Only TWO existing Source sites change intentional compound refusals to stronger
same-XML positive controls at the existing 1e-11 tolerance:

- `crates/gz-sim-scene/tests/velocity_control_posed.rs`: the extra-collision row
  in the four-primitive admission table. Preserve every other invalid row and
  all case identities; cover each old input as a positive Rapier compound with
  binding, actual advancing steps and independent child geometry/frame checks.
- `crates/gz-sim-scene/tests/compound_primitives.rs`: the controlled owner-domain
  load refusal. Preserve the same XML, passive control and world-edit refusals;
  require controller binding, old/new latch behavior, actual motion and child
  rays/poses at the retained numerical bound.

The generic public VelocityControl matrix has no compound refusal row. Its
singleton, private mesh/COM tests, all TPE assertions, all four passive posed
compound positives and M80/M84 strict precision controls remain unchanged. Add
no artificial calls, weakened negatives or tolerance changes. Authors and
independent reviewers must inspect the complete related assertion inventory.

## Independent leases and meaningful acceptance

Server owns exactly Scene `src/velocity_control/config.rs`,
`tests/velocity_control_posed.rs` and `tests/compound_primitives.rs`; the two test
files are leased only for the transitions above. In this isolated correction,
Server additionally owns ONLY the additive existing configuration case in
`tests/private/velocity_control.rs`; no new declaration or other old-body edit.
Package owns only new Scene
`tests/velocity_control_compounds.rs`. Root integrates; Scope owns this contract
and the current blocks in README, AGENTS, DELIVERY_PLAN, HANDOFF,
IMPLEMENTATION_TASKS and planning/delivery-phases.json. Root independently reviews
docs, authors review the opposite code lane and Scope reviews the whole integrated
product/regression and retained-test batch. No additional source lease is implied.

Meaningful new controls must cover mixed four-primitive and sixteen-child owners,
late invalid child/count/engine/frame/COM/joint/owner refusal atomicity, identical
explicit mass/inertia singleton-versus-compound six-axis trajectories, and
noncommuting model rotation. Prove initial/old-Pre/new-Post timing, pending
replacement and pause retention without borrowing a sensor-rich world-edit
getter outside its domain. Independently check advected child geometry and
concave gaps, inclusive/excluded queries and the strict cylinder 3.6/1e-11 chord.
Real contacts need a countercontrol and a justified fixed observation horizon;
do not assume a solver-dependent separation threshold from a short window.

Ordinary Simulation reset and loaded-world reconstruction must restore configured
initial targets, clear pending commands and retain child geometry/metadata IDs,
ownership and survivors. Preserve the existing backend prepared-geometry reset
contract without inventing a new controller reset mode. Cover removal, high-water
IDs, invalid commands and binding drift without redirecting another body.
Existing Server all/time-only loaded-reconstruction and model-only boundaries
remain inherited and unchanged, not new native manager-reset parity.

Complete implementation, meaningful regression Source, docs and independent whole
review; batch all findings before one affected formatter and the consolidated
full staged hook, normal/strict workspace and per-crate coverage and independent
retained local DATA. Then ordinary review publication, exact own whole hosted CI
and independent official DATA remain mandatory. Retained original M79 component
reference keeps its exact prior limitations; neither native compound-controller
parity nor a fresh capture is claimed. Acceptance, Main, release and Latest still
require every own and original predecessor gate, including M77 browser and
owner-confirmed shutdown. Root alone releases serial heavy operations after
Source and final parent/snapshot/procedure review; this contract runs nothing.
