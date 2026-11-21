<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M83: VelocityControl with one posed Rapier primitive

M83 composes the maintained model VelocityControl with one translated or rotated
Box, Cylinder, Capsule or Sphere collider on its existing free single-link Rapier
owner. A robot can command its link while its displaced collision geometry moves
and contacts at the correct frame. The budget is at most five working days,
including independent review and mandatory qualification.

## Basis and state

Source preparation under D31 uses locally qualified but unaccepted M82
`039a1e5142dd48705a23acb6f817613f509408f0`, tree `42019718`, parent
`6ef85e9f`. Its corrected normal/strict and independent retained DATA passed:
1872 headless/1417 overlapping renderer passes, two ignored each, 494 aggregate
Python, 47 finite/21 CLI and six strict bases. Review publication and exact hosted
run 44856006215 attempt 1 succeeded; its official audit remains pending. Preserve
its failed first normal and all earlier proof; no qualification is inherited.

M83's first full normal returned Root/Git 1 after 1385.3434725689876 seconds:
1879 headless passed, one failed and two ignored. Seven new cases passed; the
invalid-command case asked the unchanged world-edit getter to inspect a
collision+visual+IMU hierarchy, outside its collision+visual domain. Renderer and
strict coverage were not reached; there were no resource, signal or finalization
faults. The failed `16203ff3` tree, packet and receipt remain frozen.

The isolated correction removes exactly one IMU only in that case, retains a
successful getter precondition, and asserts the specific ownership setter refusal
plus unchanged native state/binding. Production, the shared fixture and other
seven case bodies remain exact. This is a test-input correction, not an expansion
of world-edit support. Independent whole correction review precedes one affected
new-test formatter and a full mandatory retry. The previous eight-file formatter
is retained for the seven unchanged Rust leaves; no correction check has run.

Source inventory remains eight new Rust declarations in both profiles (overlap
eight), zero new Python, 1788 files, 15 own paths, two new files and two production
leaves. Source finite/CLI inventories remain 47/21. No qualified M83 commit exists.
Accepted Main remains M76 `dddf1f42`: 76 phases/202 slices, catalog 83 and Latest
M70 original 41-example package. M77 formal browser access and owner-confirmed
shutdown remain acceptance gates. Keep no-expiry demo session 86363 live.

## Closed capability and interface

Retain one top-level dynamic, unjointed model with one identity-pose link, one
collision and a centered resolved inertia. Admit an already supported nonidentity
collision pose only when the actual backend is Rapier and its geometry is Box,
Cylinder, Capsule or Sphere. Existing centered Rapier/TPE paths remain unchanged;
TPE posed geometry, offset link/COM, meshes, planes, compounds, nested/static or
multiple owners and conflicting controllers remain refused.

No public API is added. Privately pass the actual `Option<&str>` engine from the
local backend in `build_model` to `velocity_control::config::Plan::validate`, as
adjacent validators already do. The backend has not been installed as a Simulation
resource at that admission point; do not infer the engine from missing resources.
Group the existing top_level/is_static booleans as one unchanged flags tuple
when forwarding the new engine argument; no private-call assertion changes.
Resolved SDF and all shape/quaternion/inertia/bounds admission remain unchanged.
A valid absent LinkCenterOfMass component remains valid absence, not binding drift.

Preserve the existing default/custom topic and canonical six-axis Twist protocol,
initial values, queue bounds, ownership/identity/epoch leases, pre-IO collision
preflight and invalid-command behavior. Commands are model-frame linear/angular
velocities at the link origin. Existing PreStep rotates the committed target by
the current model rotation, then uses the maintained engine velocity operation;
advancing PostStep latches pending input. Paused steps neither apply nor latch.
The collider rotation must not rotate commands or its translation again. Its
world center follows linkP + linkQ * localP through ordinary physics stepping;
do not add an offset correction to commanded link velocity or teleport a pose.

Retain ordinary and loaded reset restoration of configured initials, pending
clearance, geometry and IDs; removal retires the old owner. Existing Server
all/time-only loaded-reset paths, model-only behavior, stale callback leases and
shutdown remain unchanged and retain their existing tests. No Server/transport
production change or new endpoint is selected. Existing direct velocity/state
bypass refusals for the controlled link remain intact; demos and other systems
gain no direct-motion shortcut.

## Exact old assertion dispositions

Only four current Source sites change disposition:

- `tests/velocity_control.rs`: the collision-local rotation entry in the generic
  invalid matrix becomes a positive Rapier case while its TPE refusal remains.
- `tests/posed_cylinder.rs`, `tests/posed_capsule.rs` and `tests/posed_sphere.rs`:
  only each final controlled_pose refusal becomes a positive owned-controller
  geometry/motion control. Preserve the entire centered-coexistence prefix.

These paths are relative to `crates/gz-sim-scene`. `tests/posed_box.rs` contains no
controller refusal and remains unchanged. Private controller negatives cover
mesh, singleton/static/nested/overflow domains and remain unchanged; direct
private validate calls receive only the new engine argument and grouping of
the unchanged top_level/is_static flags. Retain every other
assertion, negative domain, test identity and tolerance. In particular preserve
M80's strict cylinder 3.6/1e-11 and five ray-precision controls. Review the complete
four-site inventory and reversible changes before checks.

## Disjoint Source leases

Server owns Scene `src/velocity_control/config.rs`; `src/lib.rs` only the engine
argument at this validator call; `tests/private/velocity_control.rs` only engine
argument forwarding and unchanged flags-tuple grouping at existing direct calls;
and the four public disposition files listed above. Controller execution, binding, reset and input modules are
not implementation leases; any demonstrated need requires one batched scope
review before expanding them.

Package owns only new Scene `tests/velocity_control_posed.rs`, with inline
test-owned SDF and no producer changes. It independently reviews the admission
and retained-test changes; Server cross-reviews the new regression batch.
Scope owns this contract, README, AGENTS, DELIVERY_PLAN, HANDOFF, IMPLEMENTATION_TASKS and
`planning/delivery-phases.json` in the integration tree. Root independently reviews
docs and owns shared integration and serial heavy qualification. Scope reviews
the complete integrated code/test batch; authors do not approve their own changes.

## Acceptance and exclusions

Exercise all four shapes with noncommuting model/collider transforms and centered
explicit inertia. Contact-free posed/centered controls with equal inertia must
preserve the same link-origin six-axis motion, initial/default targets and old
PreStep/new PostStep timing. Use independent moving ray/contact/frame oracles to
prove the collision moved with the link, including offset angular motion; actual
contact responses need not match a centered control that misses. Preserve sphere
rotation invariance and each shape's existing axis and numerical query boundaries.

Cover pause/latest pending command, ordinary/loaded reset, removal/survivors and
stale/foreign/drift refusal without lowering tolerances or weakening assertions.
Keep unchanged Server protocol, all/time-only/reset/removal and absent-system
controls as inherited obligations; add no new protocol mechanism merely to retest
them. Reuse the unchanged M79 CPU component reference only with its original
component-only timing/reset/callback limitations. No new native physics parity,
SDK campaign, callback receipt or full SystemManager-reset proof is claimed.

Complete implementation, meaningful regression Source and docs; batch independent
whole-review findings before one final affected formatter and consolidated full
normal/strict/local DATA/exact own hosted/official qualification. Current-provider
and Source custody remain mandatory when reusing evidence. Accepted ancestry and
all predecessor gates remain required for Main/acceptance; no earlier result
qualifies M83. No new physics API/query kernel, dependencies, package/examples,
renderer/viewer/worker, sensors, DART/MuJoCo, Fuel or demo operation is selected.
`LOAD-M83` maps this composition to existing LOAD-01, SYS-CONTROL and PHY-01.
