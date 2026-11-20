<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M80: one posed cylinder collision

M80 admits one translated or rotated cylinder collision per link on Rapier f64.
The budget is at most five working days, including review and mandatory gates.
The link, visual, sensor and inertial frames keep their existing meaning; a
collision pose changes geometry without moving the reported link origin.

Source preparation starts from locally qualified M78/M79 commit
`8ad64532489ffbb9bf62badb4a0835c34af81eaf`, whose complete normal/strict hook
and independent retained DATA passed. Its exact hosted run 44806268854
attempt 1 and independent official DATA have also passed; the initial running
observation at selection remains historical. This is an unaccepted development
basis: accepted Main remains M76 `dddf1f4284557c420e384d263a4365b2153dfde2`.
M77 browser closeout and all predecessor/own gates remain required before
M80 acceptance or Main publication. No earlier result qualifies changed code.

## Interface and boundaries

Add `PosedCylinderBodySpec` with `radius`, `half_height`,
`collision_position`, `collision_orientation`, `position`, `velocity` and
`mass`, using the same units and XYZW convention as `PosedBoxBodySpec`.
Add `spawn_posed_cylinder_link(spec, rotation, inertia)` to `DynamicBackend`.
Its default returns explicit `Unsupported` before numeric admission or allocation;
only Rapier implements the operation.

Compose the collision's local rotation with Rapier's existing cylinder-axis
correction from Y to Gazebo Z. Setting a new collider pose must not discard that
correction. Retain preallocation checks for finite dimensions, poses, transformed
bounds, mass properties, centered inertia and identity exhaustion. Dynamic
cylinders follow the existing posed-box rule: direct backend callers supply
validated link-centered inertia. Scene passes SDFormat's resolved values,
including existing defaults or centered automatic-inertia results; it does not
compute new mass properties or require XML-presence metadata that the parser
does not retain. A resolved nonzero center of mass remains unsupported.

Scene accepts the resolved local collision pose for one cylinder on Rapier and
routes it through the additive interface. Identity cylinders retain their existing construction
path; the cylinder query correction below applies to their actual shape too. TPE, compounds, offset COM and other posed primitives keep
their current refusals; new automatic-inertia computation is excluded.
Preserve SDF normalization semantics;
malformed XML that becomes a valid resolved default is not a refusal oracle.

One coexistence control may place a posed cylinder beside a separate centered
model using M79 VelocityControl. M80 does not broaden M79's admitted target shape
or geometry domain and does not replace ordinary physics with pose edits.

## First gate and selected query correction

The original 18-declaration feature completed whole Source review and formatting,
then one full normal on frozen tree `ce5d01fc` failed Root/Git 1 after
1384.4462833660073 seconds. Only the strict off-axis Scene cylinder chord failed:
3.600000000170603 versus 3.6 at `1e-11`. Remaining headless targets completed;
renderer and strict coverage were not reached. No resource, signal or finalization
fault occurred. Preserve this original tree, raw transcripts and terminal receipt.

The isolated correction admits a cylinder-only query adapter in
`crates/gz-physics-rapier/src/ray.rs`, using existing Parry analytic Capsule ray
casting restricted to the cylinder local AABB. The clipped segment stays in the
cylinder axial slab, where capsule and cylinder solids coincide; hemispherical
cap-only intersections are outside the selected segment. Apply the adapter to
all actual cylinder shapes, including existing centered cylinders, posed cylinders
and restored geometry. Keep finite transforms, interval/rebase/span/addition
checks, solid-origin zero hits, inclusive tangent/endpoints, max-distance limits,
closest-hit and body-exclusion semantics. All other shape routes remain unchanged.

Constructors and shape admission stay unchanged. The cylinder query rejects
unrepresentable analytic intermediates with `InvalidValue`, without mutation:
axis squared, radius squared and their product must be normal positive values;
actual dot/cross squares must stay finite, and a nonzero cross product must have
a normal squared norm. Zero, subnormal or overflowing scale intermediates are not a miss
or a reason to use approximate roots. Public controls retain admitted thin shapes
that the query refuses, and a finite `1e60` scaled positive cast. No rescaling or
new universal precision promise is selected.

The adapter changes no collider/contact geometry, solver, inertia, allocation,
reset, identity or VelocityControl admission. Preserve the original chord, fixture
and `1e-11` assertion. Add independent finite/overflow, cap/radial/oblique,
inside/surface/tangent, parallel/receding, cap-only false-positive, max-distance,
transformed and reset controls before whole Source review. No hand-written
quadratic, dependency, precision parameter search or new native campaign is
selected. The exact old GJK termination branch and transform contribution remain
unobserved; no universal precision guarantee is claimed.

The completed correction adds five headless-only Rust declarations: four public
query tables and one private arithmetic-boundary control. Together with the
unchanged original 18 declarations, M80 has 23 unique/headless and nine overlapping
renderer Source obligations, zero new Python declarations, 1776 Source files,
18 own changed paths and six production paths. The original 18 and the added
five are declaration counts, not pass claims; the failed gate is not qualification.
The final staged tree, whole integration review, two newly affected Rust-file
formatter and actual full retry remain required before qualification. The nine
previously formatted Rust paths retain exact Source and existing formatter evidence.

## Required evidence

Complete implementation, meaningful regression Source and documentation, then
independent whole Source review before one consolidated verification pass.
No tests, lint, formatter or native probes run after individual edits.

Regression coverage must include signed translation and composed link/local
rotations; axial, radial and tangent analytic rays and exclusion; offset contact
with a centered countercontrol; linear/angular impulse with link-centered inertia;
identity equivalence; ordinary, prepared and loaded reset, removal, groups and public identities;
and atomic refusal for invalid dimensions, quaternions, transformed bounds,
inertia and exhausted identities. Preserve existing controls and tolerances.

The retained pinned cylinder reference and M78 mechanisms are scoped inputs,
not fresh native solver-parity results. Any new parity fixture must include its
generator, exact command and library identities and reproduce byte for byte.
The final normal hook, strict coverage, independent retained DATA, exact hosted
success and official evidence remain mandatory. Batch genuine failures before
affected reruns; never weaken assertions or gates.

Source finite and CLI inventories remain 47 and 21; use test-owned fixtures.
No Server, protocol, viewer, worker, package, dependency, SDK or backend expansion
is selected. DART and MuJoCo remain retired; preserve TPE, Fuel, frozen trees and
all retained evidence. The owner's M77 demo remains running without expiry until
explicit stop confirmation. M80 does not stop or restart it.
