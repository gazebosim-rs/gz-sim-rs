<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M104: loaded passive primitive models with translated mass frames

This is unqualified Source in the selected M104–M113 wave. The genuine Git
parent is `08a0eeae`; the copied M102/M103 tree is reviewed Source only. Budget:
at most five working days including review and mandatory verification.

The selected production leases are Scene `lib.rs` and the narrow prerequisite
in `initial.rs` recorded in `concrete-prerequisite-source-scope-addendum.json`.
Loaded passive Rapier singleton and 2–16 primitive-compound links may consume
valid resolved translated COM and inertia. Resolve full BODY/LINK pose, keep
collider and visual frames separate, and use existing M100 posed constructors
for an ordinarily translated singleton COM even when its collision pose is
identity. Compounds use the exact M102 native constructor. The old centered and
tiny-COM constructor route remains unchanged. Newly translated inertial
orientation must be literal identity; no new mass-properties algorithm,
parallel-axis shift or automatic-inertia computation is added.

The explicit prerequisite policy consumes either explicit resolved inertia or
outputs of the pinned existing automatic primitive-inertia computation. The
normalized SDF Inertial has no automatic/explicit provenance discriminator;
Source does not invent one. The initial ten-phase selection's exclusion of
automatic COM consumption is overridden only by the recorded addendum. Existing
automatic producer limits and failures remain authoritative.

Mesh/plane COM, jointed or actuator-controlled offset models and rotated-offset
inertial orientation remain excluded. Loaded LINK poses already resolve to the
native BODY frame and remain allowed. This does not broaden world-edit or
runtime-template identity-LINK restrictions. Existing identity-collider runtime
singleton COM policy is untouched: cloning witnesses use already admitted
posed-singleton and compound output routes, never a new runtime branch.

The only initial ownership change removes the categorical refusal of an
honestly captured nonzero COM in loaded-compound world-edit preflight. All
remaining live-versus-captured optional Some/None/value comparisons, IDs,
hierarchy, original body identity and child geometry ownership remain exact.
A private Scene-library regression loads a genuine translated compound, removes
or changes its original COM metadata, checks model/state/template preflight
refusals and unchanged allocator/native/time state, restores the honest stamp,
then compares previously queued physical work with a healthy twin. It never
steps a corrupt world or clears a fault resource.

Public regressions cover all four primitive constructors, identity and posed
colliders, 2/4/16 compounds, wide/excluded collision queries, explicit mass and
principal torque controls with density twins, measured COM versus model-origin
velocity, six-axis edits within the mature edit domain, full LINK pose loading,
both resets, independent source retirement/cloning and contact/fall controls.
Automatic versus resolved-explicit twins use the unchanged parser's actual
computed mass/inertia. Original inputs, strict tolerances and retained case
identities belong to Root's separate same-input dispositions.

No tests, lint, formatter, build, probes or new native campaign run during
Source authoring. Complete regressions, docs and independent Source review,
batch findings, then use the one complete-candidate mandatory cycle: full staged
hook, strict workspace/parent/ancestor coverage, local and exact hosted DATA
qualification and original-artifact audit. Capacity admission is separate; no
old targets/evidence/Fuel/demo changes or acceptance transfers are authorized.
