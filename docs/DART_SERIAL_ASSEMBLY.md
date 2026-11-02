<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Bounded atomic two-revolute assembly source

The original serial candidate completed source review and one selected native
reference capture, then failed its mandatory qualification on B03 motion. This
isolated snapshot-correction source batch is unexecuted; no runtime, lint,
coverage or hosted pass is claimed for the correction.
It is isolated from the qualifying COM and B03 worktrees. The initial seed was
reviewed COM tree4d962a2276e12d0f4f3f5115535cfb7d0d2c415a over6e330abc;
COMe553 is now separately accepted. Root owns final parent adoption, Scene
integration and qualification of this feature. No Fuel, solver, dependency,
inertia formula, threshold or performance change is claimed. The only old
assertion transition in this feature is Root's separately prior-reviewed exact
B03 construction/reset positive; all other old predicates are retained.

## Contract and ownership

`DynamicBackend::add_serial_revolutes([root,middle,leaf], [firstAnchor,secondAnchor],
[firstAxis,secondAxis]) -> Result<[JointId;2],PhysicsError>` is one specific
construction operation. DART accepts exactly three distinct isolated dynamic
free bodies, before the first advancing step, and returns root→middle then
middle→leaf joint IDs. Anchors and axes are world-frame; the existing finite,
representable-anchor, normal axis-length, normalization and child hinge-response
checks remain. Both IDs are reserved with checked arithmetic before native work;
no failed call consumes either ID. Other backends' default implementation returns
Unsupported before argument validation or mutation. Existing single-joint methods
keep their exact old admission/rollback behavior.

The operation prepares an independent DART world through the maintained
`Skeleton::cloneSkeleton` contract. The selected world constructor keeps the
same guarded solver, ODE detector/filter, contact limit and gravity. Native
skeleton state, shape-node/dynamics properties, body names and the slot/hole map
are cloned; the existing principal-inertia map is rekeyed to the matching cloned
body names. Current timestep/time are retained. Bodies and joints outside the
three participants remain in the candidate and take part in final guards; they
are not omitted to simplify contact admission. Geometry is owned through DART's
clone/lifetime contract, not raw pointers escaping to Rust.

Only the candidate performs the two existing moveTo revolute frame/projection
operations consecutively. There is no global defer flag and no generic
transaction/scheduler framework. It checks each projected rate, then the complete
world's final generalized velocities, every shape's grid bounds and the UNCHANGED
contact/conditioning restrictions. In particular, an articulated body touching
an external dynamic body is still refused; this feature does not open dynamic
external contact. Final static response still uses the selected coupled-mass
conditioning/inverse-response bound.

A partial second-edge refusal or exception destroys the candidate. The live
world never needs a second allocation to roll back a moved body. On success,
all Rust body/joint metadata and readbacks finish before the live swap. Exact
nonparticipant cached state is checked rather than silently changed. Existing
world-wrench and joint-effort maps remain on the backend, with the same slots
and IDs; they are not cleared by preparation/commit. The old native world can
then retire normally. Preparation is cold O(world-size) cloning and makes no
allocation, absence or performance improvement claim.

## Reset and retirement

The accepted pair of joint IDs is retained as assembly grouping. A child-frame
group also retains its two local anchor/axis inputs; world-anchor groups need no
additional input record. Captured initial/reset records rebuild such a pair
together through the same final-state operation and original frame mode,
avoiding an intermediate pair-versus-intended-child contact check. No runtime
state snapshot or public registry is added. Removing a joint/body
clears an incomplete grouping; surviving edges use the existing single-joint
reconstruction path. Existing refusal of an unsafe partial retirement remains;
this feature does not make removal through a genuine external dynamic contact
safe. Reset still clears pending commands under its existing contract and
restores original IDs/configuration, while later IDs remain monotonic.

## Source controls and actual native reference

New public controls cover overlapping intended children, the retained old
single-edge refusal, two-ID/endpoint identity, zero/advancing-step admission,
final external dynamic contact refusal/recovery with real queued world force,
invalid final edge/axis/anchor/missing/duplicate/fixed/nonisolated endpoints,
unrelated preexisting articulation/twist/pending effort/wrench, compound shapes
with offset COM, reset/prepared reset, all three body retirements and partial
joint retirement. A static-contact case retains the previously characterized
high-condition mass/inertia family and asks the unchanged final guard to refuse.
These are source expectations pending actual consolidated execution; they do not
assert that a guessed guard branch already ran.

Private native controls exercise a genuine candidate first-edge move followed
by second-edge refusal, exact original readouts and future applied wrench,
candidate lifetime after original destruction, removed slot holes, static shape
ownership and complete skeleton retirement. Deliberately raw private records
also exercise final generalized-velocity/grid refusal; they are not claimed as
admitted public BodySpecs and the off-grid original is never collided/stepped.
A private ID-exhaustion case checks actual queued work and state without fake
counters or allocator exhaustion injection. Existing backend test bodies remain
unchanged; the exact B03 transition is separately reviewed and Root-owned.

`tests/fixtures/serial_assembly.cc` is a NEW selected gz-physics SDK observer,
not a custom dynamics model or a whole Gazebo Server benchmark. The public SDF
constructor builds the complete final root→middle→leaf tree, with three overlapping
radius.2 spheres, mass1/I1, local COM(0,0,.1), identity initial rotation/twist and
self-collision disabled. Cases use zero-gravity free motion atz1 and gravity
−9.81 with actual static-plane contact atz.19. A world force(1,2,3) at the explicit
local COM and world torque(.2,-.1,.3) are applied only before tick1. It records
all13 body fields plus9 origin/COM witnesses and both joint coordinates/rates at
ticks0,1,10,100, saving both exact XMLs and maps. Recorded requested timestamps
count ForwardStep calls; they are not a queried Gazebo simulation clock.

After complete independent source review, Root completed ONE capture with the
unchanged qualified SDK/compiler/plugin/build-first selection. It retained
actual expanded compiler argv/environment, source/ELF/depfile/header identities,
stdout/stderr, both exact XMLs and before/after runtime maps. The capture records
672 compiler inputs and eight rows containing560 body/frame/joint values.
Independent read-only audit verified those raw values against the materialized
fixture, all672 dependencies, six native source files, compiler/manifest/plugin
identities and all37 mapped SDK libraries before and after. The reviewer did not
rerun a compiler, native observer or Rust test.

The measured native FrameData supplies body-origin pose/velocity and world
angular velocity. The observer derives world COM position and COM velocity with
the EXPLICIT INPUT local COM(0,0,.1); it does not independently query the native
inertial center. The local-COM row entries are declared inputs, not additional
measured native state. Row timestamps count requested ForwardStep durations,
not a separately queried simulator clock. These limits also apply to the560-value
tally; that tally counts state/frame/joint values, not row descriptors/timestamps.

Actual data are in `tests/fixtures/serial_assembly_native.json` (SHA256
`f8030cc5729b1f2771b8f38b17a490f8d6c2316ce1c530eb350bbe26c5313b25`),
with the exact capture provenance in `reference/serial_assembly.json` (SHA256
`04a96f4cf5977e8c0f8743d5d81027951cb04336d266fe214db988e29bda8c49`).
`scripts/materialize-serial-assembly.py` repackaged only the actual successful
capture, including its raw trace and exact input XMLs. No expected trajectory
was synthesized. This is native reference qualification, not Rust parity.

`tests/serial_assembly_native.rs` binds those actual generator/provenance/raw
bytes, exact2XML/8row schema and first-step wrench protocol, and compares every
body/joint/frame field at the existing1e-12 bound over100 individual1ms steps,
then repeats against a prepared initial reconstruction. The captured floor case
passed the materializer's actual COM-departure-from-ballistic-fall predicate;
a declared floor alone would not be a passing contact witness. For the snapshot correction, all Rust comparisons and reconstruction controls
remain unrun; the earlier failed candidate context is retained separately. No tolerance/sign normalization or synthesized
trajectory is permitted. A failed comparison is a real feature qualification
failure to preserve, not a reason to loosen the final guard.

The actual B03 whole-world Server baseline lives separately. The prior COM
ordinary gate returned InvalidJoint during Rust B03 construction; a transient
partially assembled contact remains a source hypothesis, not a proven cause.
Root applied only the independently prior-reviewed same-input B03 construction/
reset positive for this candidate. That source transition is not a Rust passing
result, and it does not qualify B03 stepping. This feature still requires its
one consolidated mandatory Rust gate and exact hosted qualification. Neither
the selected native reference nor the unrun candidate proves B03 whole-world
motion, sensors, rendering or an equal-work rewrite speedup. Root's Scene
routing/tests/docs are independently owned and reviewed alongside this batch.


## Pre-mutation frame snapshot correction (SOURCE only)

This narrow correction is isolated from failed reviewed tree
`2d720de905c8dbb5260c49d1a4efb24e7e98a970` over accepted `e55356c`. The original
serial worktree, native reference, failed logs, diagnostic captures, Scene/B03
source and accepted single-edge method remain unchanged. In the new serial
candidate join only, both the parent and child `getWorldTransform()` values are
saved before `moveTo`; parent-relative joint placement then uses the saved
parent inverse. This matches pinned `SDFFeatures::ConstructSdfJoint`, which saves
`T_parent` and `T_child` before constructing/reparenting the joint. Reading a
parent frame after topology mutation permits a cache invalidation/recomputation
that the native source avoids. No child frame is forced to identity and no
inverse/product, axis, velocity projection, guard or atomic swap is removed.

This is a principled construction sequence correction, not a proven explanation
or fix for the B03 divergence. The authoritative diagnostic reproduces all101
old cached native frames and reuses saved Rust data without a rerun; fresh state
first exceeds the unchanged1e-12 comparison at tick37, with direct joint
velocity/position differences later. Native itself creates free bodies with
`FreeJoint::setTransform` before joining; absence of that round trip is not a
native/Rust distinction. Producer Euler construction, world-versus-child-local
anchor semantics, cloning/registration and contact numerical ordering remain
separate hypotheses. No threshold, fixture or expected data is altered.

The existing exact two XML/eight-row/560-value native comparator remains
byte-identical. A new appended cold-chain control uses nonidentity orientations
for root, middle and leaf, distinct positions and offset COM. It compares every
13-field actual state before/after assembly, retains zero incoming joint state
and monotonic IDs, and uses the retained single-edge API on a noncontact chain
as an independent construction control. Both receive the same captured-protocol
COM wrench once, step three1ms frames, and retain the unchanged1e-12 state and
joint bounds. Prepared reset restores the same measured initial states and
discards actual pending work; final body retirement checks ownership release.
These new actual-readout controls do not synthesize a native trajectory, prove
a solver branch ran, or qualify the nonidentity case against a new C++ capture.
The complete correction requires independent source review before any execution.

## Child-frame serial API (SOURCE only)

`add_serial_revolutes_in_child_frames(bodies, child_anchors, child_axes)` is a
separate fixed-array operation: three body IDs, two translations and two axes.
Each joint frame has identity orientation relative to its child. The local
axis is joint-local; it is also child-local for this bounded identity-orientation
contract. The operation does not accept general joint rotations or reinterpret
caller-supplied world anchors. Other backends return Unsupported before argument
validation. Existing single-edge and world-anchor public contracts remain.

Pinned `JointAxis::ResolveXyz` resolves to the XML parent joint when its target
is omitted (`sdformat/include/sdf/JointAxis.hh` and `src/JointAxis.cc`). Pinned
`dartsim::ConvertJointAxis` uses that default and copies the result into the
DART single-axis properties. Root's Scene selection therefore resolves the axis
to the child only when the resolved child-relative orientation is exactly
identity (or its equivalent negative quaternion). Nonidentity joint orientation
retains the existing world-anchor path; no near-identity tolerance is added.

The new candidate mode saves both LIVE body transforms before reparenting and
forms `T_joint = T_child * localPose`, where Eigen's local isometry has identity
rotation and the admitted local translation. It evaluates the saved child inverse
product and then the saved parent inverse product, as pinned
`SDFFeatures::ConstructSdfJoint` does, before assigning the parent and child joint
transforms. Even zero local translation does not replace an inverse product with
an invented exact identity. The validated local unit axis is copied directly
to the native joint property. The original world mode retains its existing
world-anchor/axis construction, with the separately reviewed pre-mutation parent
snapshot correction.

Admission retains the same cold/distinct/isolated/dynamic domain, checked pair
of IDs, finite and representable anchor limits, normal axis-length admission,
normalization, hinge conditioning and final whole-world velocity/grid/contact
guards. A maintained Rust rotation computes conservative world guard inputs
before preparation. Native construction uses its own live transforms. After
all native guards succeed, the candidate returns the exact world anchor/axis
values it used. Rust rechecks their anchor and hinge bounds and completes all
metadata/readbacks before swapping live worlds. A rejected readback or guard
still drops only the candidate and consumes no IDs or queued work.

An intact child-frame group reconstructs from its two local inputs. The native
resolved world edges are retained in the existing per-joint metadata; after
partial retirement, the surviving edge reconstructs through the old world path.
This prevents a local input from being silently treated as a world anchor. There
is no new public pose type, generic transaction framework, live registry or
unchecked FFI ownership conversion.

Appended source controls reuse the unchanged actual two-case/eight-row/560-value
selected-plugin reference for the matching identity-relative XML inputs through
the new API, with the same single COM wrench and unchanged 1e-12 comparison,
then repeat prepared reconstruction. A separate nonidentity-body control
compares actual readouts and local axis/anchor construction against the retained
world API; it is not a new native reference. Additional controls retain typed
invalid/fixed/nonisolated/closed/overflow refusals, final-contact refusal with a
real pending wrench, partial-edge reset, and native second-edge failure/candidate
lifetime. All original test bodies and native fixture bytes remain intact.
These new source controls, the API and Root's paired Scene/fresh-reference batch
remain unexecuted pending complete independent source review and the consolidated
required verification. No known cause or guaranteed B03 trajectory fix is claimed.
