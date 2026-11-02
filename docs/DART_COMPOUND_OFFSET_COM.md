<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Offset COM for bounded DART compound links

This source batch is isolated on accepted parent `6e330abc`; its source and index
were not modified. The earlier borrowed baseline was advanced without losing
the offset-COM delta, including the parent's exact-decimal Clippy correction.
The batch includes coordinator-owned Scene admission and point-velocity handling.
Actual C++ observations are captured below. Rust/Scene comparison, mandatory
staged coverage and hosted acceptance are separate requirements, not implied by
the native reference.

The existing public API remains unchanged. `Inertia.center` is in the body/link
frame; inertia orientation rotates only the moment tensor about that COM.
`CompoundBodySpec.position` is the world body origin, and its velocity is world
COM velocity. The native BodyNode already owns this COM and its origin inertia;
no new mass synthesis or solver is needed.

The compound admission now reuses the same single-body limits for COM magnitude,
`mass*|center|²/minimumMoment`, and `|COMvelocity|+|angularVelocity|*|center|`.
All existing thresholds, anisotropy, principal-moment and world-grid checks stay.
For every primitive, the contact-response lever is measured from COM:
`localCollisionPosition-center`, projected into the link's principal inertia axes;
local collision rotation still transforms primitive support into those axes.
Hinge bounds already use COM in parallel-axis inertia and all primitive offsets
in reach, so no native or hinge algorithm change is required.

Source controls compare a one-shape compound against the existing single-body
COM path, preserve initial world shapes and compare each body-origin
representation against its own native trajectory, separate origin conditioning from near-COM collision response, reject an invalid
final primitive lever and excessive origin velocity without losing pending work,
and restore all shapes/COM/twist after prepared reset and removal. Four new
backend controls passed before the failed origin-change experiment described
below. The corrected comparison and later Scene/native controls still require
the consolidated gate; partial execution does not establish feature acceptance.

## Prior-reviewed retained assertion transitions

Independent source review approved the exact backend case7 stronger-positive
transition. Its original input remains in the same thirteen-case table: center
(0,0,.5), three coincident local radius.2 spheres, origin0, mass1, identity
moment/orientations and zero initial twist. Only this input now takes a positive
branch after the actual construction call. It checks all initial BodyState
fields, one new body ID, unchanged preexisting state, preserved pending wrench,
removal and the next monotonic ID. The duplicate prepared test wrapper was removed
after these predicates were integrated. The other twelve negative inputs and
their indices, typed errors, state, ID and pending-work checks remain unchanged.
No execution or red/green result is claimed.

The coordinator's Scene change admits offset COM only for DART solid links,
including the existing single-collider API path. Other engines and planes retain
their prior COM refusal. Source controls cover all thirteen direct-backend state
fields, COM versus origin velocity, inertial relative-frame/model-pose resolution,
static and dynamic attachments, reset, public identities and removal. Scene
velocity/commands are link-origin values, converted to/from backend COM velocity
using optional captured local-COM metadata; IMU finite differences remain at the
link point. The unchanged zero-COM path avoids extra arithmetic/allocation.
The separately prior-reviewed exact Scene COM rejection now has its stronger
same-input positive. The exact B03 fixture retains the complete sun DTO and
an explicit joint-admission negative. Its proposed construction/reset positive
failed in the consolidated gate and was never accepted; the reviewed disposition
below records the actual next boundary. B03 construction, trajectories, native
parity, general contacts and performance remain open.

## Explicit native origin-versus-COM reference

Pinned gz-physics `dartsim/src/KinematicsFeatures.cc` fills FrameData from
`frame->getWorldTransform()` and `frame->getLinearVelocity()`: these describe
the link-frame origin. `FreeGroupFeatures.cc::SetFreeGroupWorldLinearVelocity`
also commands root-frame origin velocity. In contrast, the existing adapter's
`bridge.cc::read` returns `BodyNode::getCOMLinearVelocity()`, and
`setFreeVelocity` subtracts `angular.cross(worldRotation*localCOM)` before
commanding FreeJoint origin velocity.

The NEW `tests/fixtures/compound_com.cc` uses the actual plugin's SDF constructor.
It loads six literal inputs (one/two/three primitives, free/floor), with COMs
(.03,-.02,.05), (.1,-.05,.2), and(0,0,.5), explicit rotated link inertia and the
same declared shapes as the centered reference. It records native origin pose,
origin velocity and angular velocity; local COM comes from the actual loaded
native SDF Inertial object. Eigen transforms that actual observation to world
COM position and velocity. The primary13 fields contain body-origin pose and
COM velocity, with nine extra measured/defined frame-witness components. This
conversion is labeled; it is not a claim that native FrameData directly contains
COM velocity.

Free cases command angular velocity first, then the declared origin velocity;
the observer checks that native origin command was applied. Rust independently
converts the declared command and COM with maintained nalgebra before calling its
COM-velocity API. It never initializes from an expected native row. Contact cases
start at rest at bodyz1. Native finite/unit checks and an actual tick600 COM
departure from ballistic free fall were checked in the actual selected capture.

The selected observer ran once after complete independent source review, using
the unchanged qualified image/compiler/SDK/plugin and build-first library map.
It captured six exact SDFs and thirty rows with all twenty-two numeric fields,
including all frame witnesses. The retained record binds 675 compiler inputs,
compiler/ELF/source identities and all 37 mapped files before and after the run.
Independent read-only audit verified those inputs and all 660 numeric values
against the raw output; no native execution or compiler rerun was used by the
reviewer. This is reference qualification, not a Rust parity result.

`scripts/materialize-compound-com.py` reuses the parent's pure schema validation,
adds exact COM XML/frame checks and repackages only actual successful observations.
It does not change helper globals or execute the parent's main function. Actual
numeric observations are in the DART crate's `tests/fixtures/compound_com.json`;
`reference/compound_com.json` contains the exact actual compact provenance bytes.
The reference record's executed compile/run argv and whitelisted environments can
be replayed in its immutable image with the observer mounted at its recorded
source path and a fresh writable evidence directory. No image/SDK rebuild or
assumption of identical independently recompiled ELF bytes is needed.

`tests/compound_com_native.rs` binds those exact provenance/generator bytes,
validates all six XMLs/thirty rows and both velocity conventions, then compares
every primary field and frame witness at the unchanged 1e-12 bound over six
hundred individual one-millisecond steps. No quaternion-sign rewrite, tolerance
change or synthesized expected value is used. Full Rust/Scene checks, strict
staged coverage and exact hosted qualification remain required for acceptance.

## Origin-change experiment: actual SDK result and corrected regression

After an initial new-test compile error was corrected, the next ordinary
offset-COM gate failed after253s in the new
`changing_body_origin_preserves_world_shapes_com_motion_and_pending_wrenches`
experiment (four other new controls passed). Its13-field state comparisons did
not label the tick; the original log is retained, not retrospectively claimed
to contain a full trajectory. Production code, inputs and numerical bounds were
not changed in response.

One source-reviewed native diagnostic now observes the exact two mathematical
representations through the selected SDK plugin: zero gravity, mass1/I1, identical
initial COM twist/body rotation/world geometry, force(1,2,3) at the local COM and
world torque(.2,-.1,.3) only before the first1ms step. Angular velocity is set
before converting the declared COM command to origin velocity. C++ input
quaternion construction and actual rounded values are retained; no unobserved
Rust-input bit identity is asserted. The source is
`tests/fixtures/compound_com_gauge.cc`, raw22-row trace is
`tests/fixtures/compound_com_gauge.jsonl`, materialized values/exactXMLs/input
transcript are in `tests/fixtures/compound_com_gauge.json`, and the exact executed
provenance record is `reference/compound_com_gauge.json`.

The capture used the unchanged qualified image,677 actual compiler inputs and37
mapped files, checked before/after, with empty stderr. Independent read-only
review verified the raw values, selected libraries and source/header identities.
Initial world COMx is1.086580018004557 in both representations. At tick1, native
COMx is1.086781027461546 versus1.086781018004557, numerically identical to both
reported Rust failure values. Their x difference grows from approximately
9.45699e-9 at tick1 to9.45699e-8 at tick10. This establishes an observed SDK
post-step difference, not its numerical mechanism. It neither proves a bridge
bug nor justifies assuming representation-invariant discrete trajectories.
The diagnostic's `time_ns` is requested step duration times counted steps, not
an independently queried simulator clock; it is separate from the six-case
native Scene/physics reference.

With prior independent review, ONLY this new false post-step equality is
replaced by per-representation native comparison. Exact input geometry, force,
center/twist, ten1ms steps and1e-12 remain. The new regression binds the actual
generator, provenance, raw trace and input transcript with SHA256, checks every
one of13BodyState fields and the origin/COM witnesses at all22 rows, preserves
strong initial shape/COM/twist equality, and explicitly checks the captured
cross-representation COM delta. The zero-COM representation additionally checks
that the first force/torque increment occurs once and later twists remain at the
first-step value. No idealized constant-COM-velocity assumption is imposed on
the offset representation. This corrected Rust/native comparison passed in the
next ordinary gate. Its later B03 failure leaves full coverage, renderer and
hosted qualification pending.

The standard-library materializer `scripts/materialize-compound-com-gauge.py`
repackages only the existing successful raw capture. It rejects failed provenance,
changed generator bytes, malformed/nonfinite rows, wrong input/XML domain,
missing ticks, unexpected stderr and a changed requested-clock contract. It does
not execute physics or synthesize expected state. Reproduction uses the actual
compile/run argv and whitelisted environment in the checked-in provenance with
this exact generator mounted at `/probe-source/probe.cc` in the recorded image;
use a fresh output directory and retain the new depfile/maps/compiler identities.
B03 motion, continuous-time invariance and performance are outside this result.

## Remaining B03 joint-admission boundary

The corrected native-backed COM regression and six-case native comparison passed
in the next ordinary gate. Its sole reported runtime failure was the proposed
B03 construction positive: the unchanged fixture returned
`LoadError::Physics(PhysicsError::InvalidJoint)`. No final coverage or renderer
qualification followed that failure. The complete failed gate is retained.

Independent review approved restoring the original negative test structure,
retaining its exact fixture, caller URI, DART backend and full sun DTO. Its old
COM-unsupported expectation changes only to the actual typed joint refusal;
the never-accepted construction/reset assertions are withdrawn. All existing
backend contact, rollback, conditioning and numerical guards remain unchanged.

Source inspection identifies multiple possible joint-admission rejection paths.
A transient contact with a still-independent lower link during sequential
assembly is plausible, but no actual branch diagnostic establishes that cause.
Transactional multi-edge construction is a separate feature; this COM batch
does not silently defer contact validation or admit unqualified trajectories.
