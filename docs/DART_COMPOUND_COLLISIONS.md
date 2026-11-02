<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Bounded DART compound link construction

This candidate extends the reviewed serial-three-body Scene base `f000707`.
Actual C++ observations are captured below. Rust/native comparison, coverage,
normal staged qualification and hosted acceptance remain unrun; no B03 or
performance acceptance is claimed.

`CollisionSpec` holds an existing primitive and its link-local translation and
XYZW rotation. `CompoundBodySpec` borrows one to three collision records and
supplies world position, COM velocity and optional dynamic mass.
`DynamicBackend::spawn_compound_link` takes that spec, the existing world rotation
and angular velocity, and explicit link `Inertia`. Unsupported engines return a
typed error without allocating a body. DART requires a zero local COM for this
increment. No shape masses, density-derived inertia or parallel-axis assembly is
invented. Existing `BodySpec`, `spawn_link` and the dedicated static plane API
retain their contracts.

One native BodyNode owns all ShapeNodes, collision and dynamics aspects, default
friction1 and local transforms. Construction order follows the input slice.
Shapes have unique internal names and share one BodyId, inertia and world twist.
No registration framework, runtime callback, additional solver or package edge is
introduced. The legacy path retains an empty extra vector and its original shape
and translated-plane construction.

Admission retains the original body-origin grid domain and validates every local
pose, primitive dimension and transformed world extent. Huge opposing world and
local translations cannot qualify through floating-point cancellation. The existing contact-response formula uses the shared principal inertia,
with each primitive's support transformed into those axes and its local offset
included. Hinge reach includes the most distant primitive. Thresholds and the
packaged DART/ODE library checks stay unchanged. The existing native grid loop
already visits every ShapeNode; contacts are still checked by BodyNode, including
all existing articulated-contact and active-limit refusals.

The construction record owns all primitive records, so captured initial state
and prepared reset rebuild them together. Native construction cleans up the
whole skeleton on an exception, including failure before any body exists.
Post-construction contact/read rejection removes the new body before publishing
its state or consuming a public ID. Remaining bodies, pending wrenches/efforts and
initial-state ownership remain governed by the existing backend transactions.
No allocator-exhaustion branch is claimed executed.

The new backend test source covers old-versus-new single-body state with explicit
rotated inertia, transformed three-shape contact against separately constructed
fixed primitives, invalid final records and ID/pending-work preservation,
principal-axis response, composed world-grid bounds, final-shape hinge reach,
prepared reset/removal and an unsupported engine. It leaves every prior negative
assertion intact. These are domain and metamorphic controls, not a substitute for
an independently compiled native oracle.

## Captured native reference and consolidated qualification

`crates/gz-physics-dart/tests/fixtures/compound_collisions.cc` uses the actual
pinned Gazebo dartsim plugin's `ConstructSdfWorld`. The selected successful
capture produced six exact SDF inputs and thirty complete thirteen-field body
states: collision counts one, two and three, each in free motion and against a
static floor, at ticks 0, 1, 10, 100 and 600. The primitive families are sphere,
box, rotated cylinder and capsule. All links use centered COM, mass 2, principal
inertia (1, 1.5, 2) rotated by RPY (.1, .2, .3), and body RPY (.2, .1, -.3).
Free cases start at z=2 with world twist (.2, -.1, .3)/(.1, .2, -.3). Floor cases
start at rest at z=1 in gravity (0, 0, -9.81). All steps are one millisecond.
Every floor case actually departs from ballistic vertical motion by tick 600.

The actual fixture is `tests/fixtures/compound_collisions.json` in the DART
crate. `reference/compound_collisions.json` records its actual generator,
compiler, ELF, compiler-input digest, expanded compile/run arguments and explicit
environments, source pins, selected image/manifest/plugin, and all 37 observed
mapped files before and after the run. There were 672 compiler inputs. These
are captured native identities, not a fresh homogeneous SDK or performance
qualification. The unchanged qualified hybrid SDK provenance still applies.
The original selected library hashes remain enforced, including DART 6.13.2,
ODE, the plugin and build-tree physics core.

The first attempt failed at link time because the inherited simulator-observer
link inputs omitted the direct plugin-loader dependency. The second compiled
and ran, but failed library qualification: its install-first DT_RPATH selected
the installed physics core instead of the previously qualified build-tree core.
Neither attempt is accepted evidence. The third uses the explicit qualified
loader and build-first DT_RPATH; it passed the original map checks and captured
all finite rows and actual floor responses. No numeric input, assertion or
threshold was weakened. Failed logs remain separate from the successful data.

To reproduce, mount the observer at `/probe-source/probe.cc`, the checked-in
reference record at `/input/reference.json` and a fresh writable output directory
at `/evidence` in the recorded immutable image. No image or SDK rebuild is needed.
Run its `commands` in order: `compiler-version`, `compile`, `readelf`, `reference`,
using the exact recorded argv and environment. The reference command creates a
new `/evidence/observations` directory with all six SDFs and process maps. Rehash
the selected compiler, manifest, plugin and mapped files against the record;
independent ELF bytes are not assumed identical after a different compilation.
The record contains executed arguments, rather than an unexecuted command shape.

`scripts/materialize-compound-collisions.py` repackages only actual reference
stdout, empty stderr, exact SDF inputs/maps and successful provenance. It rejects
failed provenance, duplicate or malformed JSON, missing/extra observations,
nonfinite states, changed source/input hashes and absent floor response. It
never synthesizes expected physics values. The Rust regression binds the exact
checked-in provenance bytes and generator source, validates every input/row, then
compares all thirty states and thirteen fields against direct API construction
at the unchanged 1e-12 bound. That Rust comparison remains unrun at this point.
The backend and Scene regression source, retained suite, lint, strict staged gate
and exact hosted qualification must pass before acceptance.

The unchanged B03 fixture still requests nonzero link COM and remains outside
this increment. Full B03 trajectories, general compound contact admission,
Scene renderer support and equal-work performance are not qualified here.
