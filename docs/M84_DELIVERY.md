<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M84: Bounded Rapier primitive compounds

Deliver one link with two to sixteen primitive collisions, represented by one
Rapier rigid body, one collider and one BodyId. This makes concave arrangements
of existing primitives runnable without new physics or query solvers. The budget
is at most five working days including independent review and mandatory gates.

## Basis and current state

D31 Source preparation starts from locally qualified, unaccepted M83
`11afe37a4747cd7c4c0ad9a2f466721b86b1a156`, tree `3a22a633`, parent
`039a1e51`. Its corrected full normal and independent retained DATA passed:
1880 headless/1425 overlapping renderer, two ignored each, all eight new controls
in both profiles, 494 aggregate Python, 47 finite examples, 21 CLI cases and seven
strict changed-line bases. Ordinary review publication and exact hosted run 44865727539 attempt 1 passed.
Original collection and independent official audit also returned actual zero;
four original RAW groups (439/732/57/168) were compared byte-exactly once. Preserve its first
failed normal and all earlier evidence. No result or acceptance transfers to M84.

M84's first full normal genuinely returned Root/Git 1 after 1389.975346346997
seconds: 1896 headless passed, one failed and two ignored. Sixteen of seventeen
new controls passed; renderer and strict coverage were not reached. The only
failure required x < -0.1 and y > 0.7 after 100 fixed steps; observed positions
were approximately -0.06980455 and 0.66980455. No resource, signal or finalization
fault occurred. Freeze the original 4292c982 tree and all retained evidence.

The isolated test-window correction changes both before/after-reset observations
in that one case from 100 to 1500 fixed steps, the existing 1.5-second compound
contact window. Keep both separation inequalities, 1e-11 equality, fixture,
timestep, mass, solver and reset assertions unchanged. No production defect,
exact solver cause or successful retry is inferred. All six production leaves,
the other sixteen new cases and every other assertion remain exact.

Inventory remains 1793 Source files, 22 own paths, five new files, six production
leaves and 87 cumulative paths from accepted Main. Seventeen new Rust declarations
cover headless and eight renderer declarations overlap; zero new Python. These
are Source obligations, not qualified results. The exact two-window repair is independently Source-reviewed; final authored
docs review precedes one affected-test formatter and the full mandatory retry. Reuse
the original fourteen-file formatter only for thirteen unchanged Rust leaves;
no correction formatter or check has run. Final tree, commit and outcomes are unbound.
Accepted Main remains M76 `dddf1f42`: 76 phases/202 slices, catalog 84, and Latest
M70 original 41-example package. M77 formal browser access and owner-confirmed
shutdown remain acceptance gates. Keep no-expiry manual demo 86363 live.

## Closed API and geometry

Reuse the existing Copy `CollisionSpec { shape, position, orientation }` and its
four closed Shapes: Box, Gazebo-Z Cylinder, Gazebo-Z Capsule and Sphere. Add:

```rust
pub struct CompoundBodySpec<'a> {
    pub collisions: &'a [CollisionSpec],
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub mass: Option<f64>,
}
// DynamicBackend, alongside the existing singleton construction methods:
fn spawn_compound_link(
    &mut self,
    spec: CompoundBodySpec<'_>,
    rotation: RotationSpec,
    inertia: Option<Inertia>,
) -> Result<BodyId, PhysicsError>;
```

The default implementation returns Unsupported before numeric validation or
allocation. Rapier accepts exactly 2..=16 direct primitive children; no recursive
Shape variant is introduced. Borrowed input is consumed during admission into an
owned maintained SharedShape compound, retained through existing Construction
collider cloning. One link owns one body/collider/ID; child collision declaration
identities and local frames remain Scene metadata, never extra bodies or masses.

Compose each caller child pose with the existing cylinder Y-to-Z correction
exactly once; the other three shapes require no axis fix. Preserve existing
passive link poses and model anchors: world child geometry is body/link pose
multiplied by child local pose. Do not add an identity local-link restriction.
The existing one-collision dispatch stays byte-identical.

Dynamic calls require Some validated link-centered inertia; None is Unsupported.
Static calls permit None, require zero velocities, and any supplied inertia must
also be centered and validated. The requested spec.mass supplies link mass; supplied link inertia supplies its
COM and tensor. With supplied Some(Inertia), insertion zeroes collider density,
so child geometry and density do not infer physical COM or tensor. A static call
with mass None and inertia None retains geometric collider bookkeeping on a
fixed body only. Child density stays metadata, never an automatic mass sum. Scene supplies its
existing resolved inertia, including documented defaults. Pinned SDFormat's
compound automatic-inertia refusal remains unchanged; add no XML-presence parser
or mass/inertia computation.

Validate the complete list before native insertion, public-ID advance or pending
state mutation: count, dimensions, finite local/world translations, unit direct
quaternions, every child and aggregate AABB with representable positive extents,
representable aggregate AABB centers and maintained BVH arithmetic (including
its existing coincident-centroid fallback) before compound construction, existing geometric mass/inertia representability, requested
mass/inertia, and ID
availability. A final invalid child must leave bodies/colliders/IDs/state intact.
No constructor domain, numeric threshold or singleton refusal is weakened.

## Query and lifecycle contract

Do not use Parry's Compound ray cast: its cylinder child takes support-map GJK
and its strict nearest comparison can exclude the inclusive maximum. Traverse
all at most sixteen direct children without early successful return or shrinking
the caller maximum. Compose current body/collider/child poses, use the unchanged
M80 cylinder_distance for Cylinder, and maintained Ball/Cuboid/Capsule kernels
for the other children. Return the finite inclusive nearest hit, solid-inside
zero, and whole-body exclusion. Any admitted child's query-domain error remains
an error even if another child already hit. Keep the M80 strict 3.6/1e-11 chord,
all five precision controls and their numeric refusal boundaries unchanged.

Ordinary, prepared-initial and loaded resets reuse owned Construction geometry;
queries immediately after reset must use current poses without stepping or
mutating contacts. Preparation completes all fallible reconstruction before
apply; apply retains owner checks, stable body IDs and Scene child metadata identities (no native child IDs),
pending-command
clearing, self-collision groups and body/joint ID high-water marks. Removal removes
all owned compound geometry and preserves unrelated bodies and their IDs.

Scene admits new compound links only with Rapier, centered resolved inertia and
no joints anywhere in that model. Preserve existing passive multi-link grouping;
this phase adds no Scene compound-joint domain or joint parity claim. Existing
M83 VelocityControl singleton/identity-link restrictions and world-edit/world-spawn
singleton gates remain unchanged. Exclude Plane, Mesh, nested compounds, TPE
compounds, offset COM, custom surfaces, new renderer/Server/package/dependency or
sensor capability, new native campaign, DART/MuJoCo and Fuel changes.

## Exact retained-test transitions

Only five existing Rapier compound-refusal sites become stronger positive
geometry controls using the same XML and existing 1e-11 tolerance:

- tests/compound_collisions.rs: the two-member FIRST+SECOND row; keep its single
  posed sphere and every TPE input/assertion unchanged.
- tests/posed_box.rs: its extra-box compound row.
- tests/posed_cylinder.rs: its extra-sphere compound row.
- tests/posed_capsule.rs: its extra-sphere compound row.
- tests/posed_sphere.rs: its extra-sphere compound row.

All paths above are under crates/gz-sim-scene. Preserve adjacent COM, surface,
nonprimitive and other negative rows; zero-collision controls in scene.rs and
body_joint.rs remain refused. Keep M83 controller-compound and offset-COM
refusals, all prior case identities/assertion strength, and every strict query
oracle. Do not relocate fixture link poses to evade frame coverage.

## Independent leases and acceptance

Package owns gz-physics src/lib.rs, src/backend.rs, tests/private/backend.rs;
and gz-physics-rapier src/lib.rs, new src/compound.rs (including private ID control),
src/ray.rs and new tests/compound.rs. Server owns gz-sim-scene src/lib.rs, new
tests/compound_primitives.rs and tests/fixtures/m84_compound_primitives.sdf plus
only the five old-row dispositions above. Root integrates; Scope owns this
contract and current README/AGENTS/DELIVERY_PLAN/HANDOFF/IMPLEMENTATION_TASKS/
planning blocks in the integration tree. Root independently reviews authored
docs; authors cross-review the opposite lane and Scope independently reviews
the complete integrated API, mathematics, lifecycle and retained-test batch.

Meaningful controls cover mixed all-four geometry and concave gaps, counts two
and sixteen versus one/seventeen, last-child failure atomicity/ID exhaustion,
independent per-child rays with noncommuting frames and cylinder precision,
inside/inclusive maximum/exclusion/current reset poses, real contacts plus
countercontrols and self groups, explicit centered-inertia impulses, collision
metadata/stable identities, removal/survivors and ordinary/prepared/loaded reset
with high-water preservation. Retain negative backend/count/shape/COM/joint/
controller/world-edit/auto-inertia and numeric-domain controls.

Finish implementation, regression Source, docs and independent whole review;
batch findings before one affected formatter and consolidated full normal,
staged hook, strict workspace/per-crate coverage and independent local DATA.
Then ordinary review publication, exact own whole hosted CI and official DATA
remain mandatory. Reuse unchanged evidence only where the mature gates permit;
no fresh native reference or solver parity is claimed. Main, phase acceptance,
release and Latest require every original predecessor and own gate. No checks
are authorized by this document; Root serializes operations after Source closure.
