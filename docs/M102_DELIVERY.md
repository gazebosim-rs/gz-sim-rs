# M102: translated mass frames for runtime primitive compounds

Status: selected Source implementation, not a qualified or accepted delivery.
Budget: at most five working days including review and mandatory verification.
Source parent: locally qualified `08a0eeae` / tree `9e1f92e1`.

## Contract

Use the existing `prepare_model_template` and `spawn_model_template` APIs.
The source may be an already supported valid loaded primitive compound or an
owned runtime primitive compound. Validate actual source ownership before
interpreting the copied request. The new output is one passive link containing
2–16 Box, Sphere, Cylinder or Capsule collisions with explicit translated COM.
The LINK and newly translated inertial orientation remain identity. Preserve
all existing mass, inertia, density, pose and primitive dimension validation.

A compound source remains a compound even when a copied request supplies one
collision. That mature one-collision route retains its centered-COM restriction.
No loaded initial offset-COM body, Mesh COM, new singleton source class, TPE,
controller, articulation, sensor, plugin or nested-compound support is added.
Existing centered inertia-orientation and tiny singleton COM domains remain.

## Native and ownership boundaries

The only production seams are Rapier `compound.rs` and Scene `world_spawn.rs`.
Replace the native nonzero-center refusal with bounded nonzero-only admission:
finite center and center norm squared, actual BODY-pose world COM and its norm
squared, and finite compound bounding-sphere distance to COM plus radius.
Check before allocating native identifiers or mutating native state. Reuse
explicit inertia properties and zero collider density; never infer the tensor
from geometry or apply a second parallel-axis shift. Constructor admission does
not promise universal safety for arbitrary future force or simulation steps.

The immutable compound stamp must retain the exact optional original center.
Centered None and translated Some are distinct. Match the value and presence
through both original Arc stamps. Removed/changed/forged COM, missing stamps,
foreign identity, late child corruption and stripped actual controllers must
remain refused through model state, reset, removal and retemplating. Preserve
all existing ID, native-body, member, visual ownership and pending-work checks.

## Regression and delivery evidence

Cover all primitive kinds and 2/4/16 children, rotated body/collider geometry,
independent COM/origin position and velocity equations, explicit inertia and
density twins, static/dynamic contacts, finite-step force/torque references,
six-axis edits, both resets, source retirement, high-water IDs and late faults.
Native BodyState velocity is COM velocity; the Scene model-origin velocity is
a distinct quantity. Initialize schedules without callbacks before a complete
cold paused-state baseline; respect the existing delayed controller latch.

Keep the four exact native +/-0.1 X COM times fixed/dynamic inputs as physical
positives. Keep the exact Scene +0.25 X retemplate request as a positive in a
separate world, preserving the original atomicity baseline and every other
refusal. Do not change tolerances or remove historical declaration identities.

Native private library cases are headless only. Required renderer constructor
branches need public Scene witnesses through the existing dependency; do not
expand the renderer package plan. The exact authored inventory is recorded only
after Source completion and independent review, before the single final cycle.

Acceptance requires the unchanged staged hook and strict coverage, independent
retained local DATA, exact review-hosted CI and original-artifact audit, plus all
predecessor/browser acceptance gates. No current test result or count is implied.
