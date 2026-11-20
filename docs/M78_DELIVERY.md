<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M78: one posed box collision in Rapier

## Current shared M78/M79 qualification

Root selected one D31 candidate over qualified but unaccepted M77
`57b44de10713dc6132c5549d769dac00dc041f56`, with accepted Main ancestor M76
`dddf1f4284557c420e384d263a4365b2153dfde2`. This candidate includes the reviewed
M78 posed-box scope below and [M79 VelocityControl](M79_DELIVERY.md), with no
new feature expansion. M78's own scope has 16 Rust declarations H16/R7; M79 has
23 H23/R14. The combined 39 (H39/R21, overlap21) and zero new Python declarations
are Source obligations, not passes. Finite/CLI inventories stay 47/21.

The standalone M78 normal failed with actual Root/Git 1 after
4169.970975892 seconds at the unchanged 3600-second strict-headless timeout.
The retained reset test was unfinished; cause UNKNOWN. Frozen tree `9774cd21`,
HEAD57, the failure receipt and all partials remain unchanged. No qualified
M78 commit, standalone pass or intermediate Main is claimed. The new candidate
uses only the existing test isolation helper for that local-query fixture;
its assertions/timeouts are unchanged, and this is not a confirmed causal fix.

Complete merged implementation/regression/docs and independent whole Source
review, fresh M79 component capture/DATA and final native adoption before one
new full normal/strict/coverage/hosted/official chain proving both scopes.
Measure fresh combined source/modes/tree and the CameraInfo retained-reference
domain; old M78-only Source cards cannot stand in for the merged candidate.
Both phases remain unaccepted. M77 predecessor acceptance, qualified Main and
every feature-specific proof remain mandatory. The original bounded contract
below retains the reviewed behavior and exclusions; its initial preparation
status is historical, superseded by this paragraph. No demo stop is authorized.

The owner authorized the next development phase on November 20, 2026. M78
selects one useful retained geometry gap: a single box collider may have a
finite translated and rotated link-local SDF pose in Rapier f64. The link and
its inertia keep their own frames. The budget is at most five working days,
including independent Source review and mandatory qualification.

Implementation, additive regression Source and documentation are complete. The
Source census is 16 distinct Rust test declarations: nine physics and seven Scene,
all 16 scheduled headless and seven also scheduled in the renderer profile. There
are zero new Python declarations. These are Source counts, not observed passes;
whole independent Source review is underway. No M78 formatter, lint, test, native
operation or qualification has run.

Source preparation uses isolated `work/m78-posed-box-collision` at qualified
M77 `57b44de10713dc6132c5549d769dac00dc041f56`. That immediate parent remains
**unaccepted**; accepted Main ancestor is M76
`dddf1f4284557c420e384d263a4365b2153dfde2`. M77's remaining interactive proof
and current manual demo are separate work. No M77 acceptance, new Main or
published package is implied. Reconcile actual parent facts before M78
qualification; Root may release
its own completed qualification under D31 while M77 interactive closeout
continues. Acceptance/Main still require the accepted predecessor and every
M78 gate. A changed parent requires affected Source review and checks. Preserve
frozen M77 trees, all evidence and Fuel ownership.

## Behavior and interface

Retain exactly one collision per link. Existing centered primitive, mesh and
plane routes stay unchanged. Only a nonidentity resolved box pose in Rapier
selects the new route; TPE and other primitive shapes keep their existing
nonidentity-pose refusal. Use the existing SDFormat semantic resolver, including
`relative_to`, rather than manually adding Euler angles or translations. Direct
backend API quaternions must be unit-valued. Loaded XML follows pinned SDFormat:
finite positive quaternion norms are normalized upstream; zero and nonfinite
norms are refused. A nonunit XML quaternion is not itself an invalid-input case.

Add `PosedBoxBodySpec` with `half_extents`, `collision_position`,
`collision_orientation`, `position`, `velocity` and `mass`, following the
existing mesh specification's SI, link-local and XYZW conventions. Add
`DynamicBackend::spawn_posed_box_link(spec, rotation, inertia)` with a default
explicit Unsupported refusal and a Rapier implementation. Old construction
methods retain their behavior. Dynamic posed boxes require explicit validated
inertia centered on the link origin at the API boundary (`None` is refused).
The Scene loader supplies the resolved SDF inertial values, including the pinned
SDFormat default when `<inertial>` is omitted: mass 1, identity tensor and
identity inertial pose. It does not require XML element presence and never
derives mass or a displaced center of mass from the collision. Static posed boxes keep the original reported link
frame and fixed-body velocity rules. Any supplied inertia remains centered.

Validate the complete local/world transform, dimensions, mass, rotation,
inertia and representable bounds before allocating a body or consuming an ID.
Retain one body/public link identity, self-collision groups, query exclusion and
existing world-force/torque semantics. Clone the admitted collider in ordinary
and prepared reset; do not reload assets or derive a different collider.

## Dependencies and evidence

The existing Cargo lock pins `rapier3d-f64` 0.36.1 with enhanced determinism and
SDFormat Rust revision `82c35f24f36d0b443cb078c02e0b844804d454e5`.
Rapier's maintained `ColliderBuilder::position` / parent-local collider pose
and the existing posed-mesh construction/reset path provide the implementation
mechanism; no dependency change is selected. Upstream SDFormat checkout
`1249805d9fceeff94ca1fe63e26ab9c051cea24e` exposes collision SemanticPose.
Upstream gz-physics `b44cd0d0ea6606b6c98320c0c87a5670d4fc6be6`,
`bullet-featherstone/src/SDFFeatures.cc`, resolves Collision.SemanticPose in the
owning link frame. These are Source references, not a new native execution or
numerical parity claim. D33 keeps DART and MuJoCo cancelled.

Acceptance uses independent analytic box geometry and actual Rapier behavior.
No new SDK, native engine, renderer or telemetry framework is required. Keep
every old regression body and oracle, including offset-sphere and compound
refusals in `compound_collisions.rs`; new assertions belong in additive tests.

## File ownership and acceptance

Root reviewed this scope and released the disjoint Source leases below. Physics
and Scene implementation may proceed in parallel; the documentation lane owns
this contract and canonical planning. Shared API spelling and semantics above
are the integration contract; coordinate changes before touching another lease.

| Lane | Owned files |
| --- | --- |
| Physics | `crates/gz-physics/src/lib.rs`, `crates/gz-physics/src/backend.rs`, additive default-refusal control in `crates/gz-physics/tests/private/backend.rs`, `crates/gz-physics-rapier/src/lib.rs`, optional focused `crates/gz-physics-rapier/src/posed_box.rs`, new `crates/gz-physics-rapier/tests/posed_box.rs` |
| Scene | `crates/gz-sim-scene/src/lib.rs`, new `crates/gz-sim-scene/tests/posed_box.rs`, new `crates/gz-sim-scene/tests/fixtures/m78_posed_box.sdf` |
| Canonical documentation | This contract, `planning/delivery-phases.json`, `docs/DELIVERY_PLAN.md`, `docs/HANDOFF.md`, `docs/DECISIONS.md`, `README.md`, `AGENTS.md` |
| Root | Integration, independent review assignment, serial formatter/heavy execution, final bindings and publication evidence |

Meaningful controls must establish:

- Signed translation and nontrivial local rotation compose with a separately
  rotated/translated model/link. Independent analytic rays hit the expected
  faces and miss the old centered box; ray exclusion remains body-scoped.
- Static and dynamic boxes retain link-origin state and explicit inertia.
  A contact countercontrol distinguishes the offset from a centered collider;
  zero-gravity force/torque or impulse controls catch an accidental shifted COM.
- Identity pose matches old construction. Ordinary and prepared reset,
  same-ID loaded reset, removal and survivor behavior retain geometry and IDs.
- SDF `relative_to` resolution produces the expected local collider without
  shifting link/visual/sensor frames. Existing groups remain effective.
- Nonfinite transforms, direct-backend nonunit quaternions, invalid dimensions/
  mass/inertia and transformed-bound overflow fail atomically. Loaded XML has an
  additive finite-positive-norm normalization equivalence control and genuine
  zero/nonfinite-norm refusals. Missing dynamic inertia and
  offset inertia are explicit API refusals; omitted SDF inertial data uses the
  documented centered default. TPE posed boxes, offset non-box shapes,
  multiple collisions, custom surface parameters and unsupported inputs retain
  their refusals. Existing successful centered inputs remain successful.

Use a test-owned fixture; this phase adds no public finite example or CLI case.
The Source finite/CLI inventory stays 47/21. M70 public Latest remains the
original 41-example package; M77's provisional 47-world package stays separate.
No test-count result is claimed from declaration Source.

Finish implementation, additive regression Source and concise guides, then
independently review the whole candidate and batch findings. Only then may Root
release one final formatter and full ordinary staged normal/strict hook,
coverage gates, exact hosted CI, retained local/official DATA and qualified Main
promotion. Reuse unchanged evidence only where the existing gate permits it;
M78 needs its own results. Keep existing thresholds, offline/toolchain/job and
resource guards. No per-edit tests, lint, format or duplicate parent checks.

## Bounds and delivery budget

Days 1–2: API/backend and Scene/additive regressions in parallel. Day 3:
integration, docs and independent Source review with batched corrections. Days
4–5: the consolidated mandatory qualification, retained DATA and publication.
This is a work budget, not permission for daily verification rounds.

Compound geometry, arbitrary primitive local poses, offset inertial centers,
automatic mass, new joint domains, TPE capability expansion, solver tuning,
rendering/camera/network protocols, transport changes, package refresh, Fuel,
performance claims and retired backends are excluded. Existing upstream Zenoh
and accepted viewer behavior remain unchanged. No manual demo may be stopped or
restarted by this phase.
