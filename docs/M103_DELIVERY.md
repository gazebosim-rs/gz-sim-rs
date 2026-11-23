# M103: rotated local link frames for JointController

Status: selected Source implementation, not a qualified or accepted delivery.
Budget: at most five working days including review and mandatory verification.
Source parent: locally qualified `08a0eeae` / tree `9e1f92e1`.

## Contract

Extend only the existing two-link/two-Revolute world-rooted serial
JointController configuration. For a newly rotated local LINK require finite
f64/f32 full-pose components and quaternion norm squared within 1e-12 of one.
Do not normalize, repair or mutate caller DOM. Preserve the literal identity
LINK path's existing finite-f64 translation domain. LINK relative_to remains
empty; retain the separately admitted local JOINT-frame rule.

Keep arbitrary unit-axis admission, empty expressed_in/no axis2, canonical
child/topology, modes, gains, limits, topics, eight-owner limits and all existing
model/plugin/static/backend gates. Keep centered articulation inertia COM and
the native compatible initial/reset velocity checks. No named LINK frames,
new topology, rotated inertia/COM composition, solver, protocol or public API.

## Existing frame machinery

Only `joint_controller/config.rs` changes production behavior. Existing Scene
joint resolution already computes world anchors and axes. Native construction
already transforms the world joint frame by each actual body's inverse pose.
Do not alter those paths or add a parallel frame implementation.

Regression references must compose model, LINK and JOINT rotations separately.
Use measured joint coordinates with independent Rodrigues transforms and
transported anchors/axes; compare full physical bases, native twists, canonical
model origin and collision/visual frames. Respect moving-parent Jacobians and
origin velocity lever arms. Never substitute ideal commanded motor speed for a
measured physical result or require an arbitrary quaternion bit pattern.

## Regression and delivery evidence

Cover first-only, second-only and both rotated links, noncommuting frames,
signed axes and equivalent sign/quaternion inputs, both controller modes,
child geometry, wide ray caps and separate exact identity endpoints. Retain
limits/latest/zero/invalid input, pause/time/epochs, force/inertia/density twins,
causal contact controls, both resets, native/joint identities, high-water IDs,
eight-owner late faults, repair and terminal latching.

Two retained former LINK rotation inputs use the exact [0,0,1,0] quaternion:
the external private configuration case and the M101 inline DOM matrix. Root
moves these same inputs to positive parse/immutability/parameter witnesses;
all neighboring frame, axis, type and malformed-input refusals remain.

New Scene public and private controls run in both retained profiles. Record
exact declarations only after complete Source review; they are obligations,
not executed passes. Finish implementation, regressions and documentation,
batch independent Source findings, then run one consolidated final cycle.
Acceptance requires full mandatory local/hosted checks and original evidence
audits and predecessor/browser gates; no result transfers from the parent.
