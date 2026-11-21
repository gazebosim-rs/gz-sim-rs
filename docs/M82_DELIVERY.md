<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M82: one posed sphere collision

M82 admits one translated or rotated sphere collision per Rapier f64 link, with
at most five working days for implementation, review and mandatory verification.
The useful increment is a loadable displaced sphere with real contacts, rays and
unchanged link-centered inertia, completing the remaining existing primitive-pose
gap after boxes, cylinders and capsules. Sphere orientation is geometrically
symmetric; collision pose metadata still retains the caller's rotation.

## Basis and state

The selected basis is locally qualified M81
`6ef85e9f2fbc0b371b2e0db3c366230ac8d9106e`, tree `e720a221`, parent
`ac75646a`. Its complete normal/strict hook, independent retained DATA and own
hosted run 44840183277 attempt 1 and independent official DATA passed. Accepted
predecessors remain required; this basis is unaccepted and qualifies no changed
M82 code. Accepted Main is M76 `dddf1f42`: 76 phases/202 slices, catalog 82,
Source finite/CLI inventories 47/21 and Latest M70 original package 41.
M77 formal browser access and owner-confirmed shutdown remain acceptance gates.
Keep its manual demo session 86363 running without expiry.

The first full normal at frozen tree `05716f75` ended Root/Git 1 after
1390.567 seconds: 1870 headless passes, two failures and two ignored controls.
Both failures are older offset-sphere refusal expectations; all 19 new M82
controls explicitly passed in headless. Renderer and strict coverage were not
reached. No qualified commit, capacity/signal/finalization fault or M82 pass is
claimed. The original tree, formatter and failure receipt remain preserved.

An isolated Source correction adds the two missed generic Scene dispositions
to the original three posed-suite transitions. Production and all 19 new control
bodies remain unchanged. Current Source has 1786 files, 20 own changes, five new
files and five production leaves; 19 headless/nine overlapping renderer Rust
declarations and zero new Python are obligations, not full-run results. Complete
whole correction review before one affected two-file formatter and full retry;
retain the unchanged original 11-file formatter evidence. Root alone launches
heavy qualification; no per-edit checks or native trials.

## API and capability mapping

Add `PosedSphereBodySpec { radius, collision_position, collision_orientation,
position, velocity, mass }` and
`DynamicBackend::spawn_posed_sphere_link(spec, RotationSpec, Option<Inertia>)`.
Use meters, SI values and XYZW quaternions, like the existing posed APIs.

| Boundary | Selected behavior |
| --- | --- |
| Default DynamicBackend | Explicit Unsupported before numeric validation/allocation |
| Rapier | Existing ball geometry plus caller local rigid pose; no axis correction |
| Scene | Resolved nonidentity sphere pose routes through the new Rapier operation |
| Existing identity construction | Preserve the centered path and behavior |
| TPE nonidentity sphere pose | Remains unsupported |

The world sphere center is `link_position + link_rotation * collision_position`.
Local collision rotation does not rotate that translation again and cannot change
the spherical geometry. Validate both backend quaternions even though local
rotation is geometrically symmetric. Preserve the original link, visual, sensor
and inertia frames, collision metadata, body IDs and ownership.

Validate finite positive representable radius, poses, velocities, composed local
and world bounds and geometric mass properties before inserting bodies or
consuming IDs. Direct dynamic callers require supplied, validated link-centered
inertia; None and nonzero COM remain unsupported. Keep static-velocity refusal,
malformed-input/identity-exhaustion atomicity and pending-command preservation.
Use existing construction/reset/group handling and zero collider density with
explicit inertia; do not infer an offset mass center or parallel-axis tensor.

Scene passes pinned SDFormat resolved Some inertia, including defaults and
supported automatic values. At pin `82c35f2`, missing/nonpositive sphere radius
becomes 1; finite nonzero XML quaternions normalize. Plain local rotation-only
automatic sphere inertia is supported; translated auto derives nonzero COM and
remains refused by the existing centered-COM gate. Auto pose attributes and
relative_to remain upstream-refused. There is no XML-presence parser, producer
change or new automatic mass computation. Do not copy capsule identity-only auto
or cylinder/capsule axis and tiny-segment assumptions into sphere tests.

Keep existing Ball RayCast and all other query math unchanged. No precision
adapter, custom solver, universal precision claim or new native campaign is
selected. Retain historical references with their actual Source and scope;
independent analytic oracles prove this new route without relabeling native
physics parity. Any genuine gate prerequisite must be reviewed explicitly.

## Disjoint Source leases

Package owns `crates/gz-physics/src/{lib,backend}.rs`, its
`tests/private/backend.rs`, and Rapier `src/lib.rs`, new `src/posed_sphere.rs`
and `tests/posed_sphere.rs`. Scene owns `crates/gz-sim-scene/src/lib.rs`, new
`tests/posed_sphere.rs`, `tests/fixtures/m82_posed_sphere.sdf`, and only the
five retained Sphere-refusal dispositions below, including `tests/compound_collisions.rs`
and `tests/scene.rs` for the bounded correction. Scope owns this contract,
AGENTS, DELIVERY_PLAN, HANDOFF, IMPLEMENTATION_TASKS and delivery-phases planning.
Root reviews docs and shared integration; authors cross-review opposite code
lanes and Scope independently reviews the whole integrated code batch.

The five intentional old assertion dispositions are the Sphere-refusal blocks
in Scene `tests/posed_box.rs`, `tests/posed_cylinder.rs`,
`tests/posed_capsule.rs`, `tests/compound_collisions.rs` and `tests/scene.rs`.
The original three posed-suite positives retain independent center/chord geometry
at their existing `1e-11` tolerance. The two missed generic rows become positive
Rapier geometry controls; retain the compound and TPE negatives and the existing
simulation's time/state atomicity assertions. Keep every other body, assertion,
negative domain and tolerance; independently review all five dispositions before
the full retry. All M80 strict 3.6/1e-11 and five ray-precision controls and all
other M81 controls remain unchanged. This correction adds no test declaration or
production behavior and does not reinterpret the first failure as a pass.

## Acceptance and exclusions

Require independent transformed-center and local-rotation-invariance controls,
axial/off-axis 3-4-5 chords, tangency/miss/inside/surface, max-distance,
closest-hit and body exclusion. Exercise explicit anisotropic inertia and world
linear/angular impulses without collider-derived COM, real displaced contact
against a centered miss, same-model exclusion/distinct-model contact, ordinary,
prepared and loaded reset/removal/IDs/survivors, and centered identity equivalence.
Cover finite/overflow/radius/inertia refusals atomically and actual SDF default
and auto boundaries. A separate centered M79 owner may coexist; posed-target
VelocityControl refusal and all controller behavior remain unchanged.

One collision per link only. TPE posed geometry, compounds, offset COM, other
new shapes, new controllers/sensors, Server/transport, renderer/viewer/worker,
package/dependency/workflow changes, DART/MuJoCo and Fuel are excluded. Use a
test-owned SDF and existing visuals; 47 finite/21 CLI inventories do not expand.

Finish implementation, meaningful regression Source and docs, then whole
independent review and one batched finding correction before a final affected
formatter. Complete the ordinary full staged hook, strict workspace/per-crate
coverage and all required changed-line bases, independent local DATA, exact own
hosted CI and official evidence. Reuse unchanged evidence only where its checks
permit it. D31 permits Source/qualification alongside predecessor closeout;
Main and acceptance require all own gates and accurate accepted ancestry.
No frozen tree, old proof or persistent demo is changed by M82.
