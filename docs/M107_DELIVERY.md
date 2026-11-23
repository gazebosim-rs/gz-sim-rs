<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M107: posed and compound TPE collision bounds

This is unqualified Source in the selected M104–M113 candidate over genuine Git
parent `08a0eeae`. The M102/M103 foundation is reviewed Source, not a passed
predecessor. Budget: at most five working days including review and verification.

Production is limited to Scene `lib.rs` and `tpe.rs`. Loaded TPE links may hold
one posed primitive or a union of 2–16 Box, Sphere, Cylinder or Capsule
collisions. Validate dimensions and finite local poses using the retained TPE
state domain, compute each collision-local primitive AABB, then union them.
The SDF cylinder is Z-axis; Parry's Y-axis cylinder needs the explicit fixed
axis conversion. Check finite, nondegenerate local and initial world envelopes
for this new domain before publishing a TpeLink. The old identity singleton
path still calls the exact mature validator.

The existing LINK-to-model and model-to-world AABB envelope stages remain.
These are conservative model envelopes, not exact oriented-shape contacts.
Motion remains kinematic and contacts remain bounded AABB-overlap reports with
existing masks, static filtering, contact limits and terminal overflow faults.
No mass/inertia dynamics, force resolution, Mesh, planes, controller expansion,
new physics API or custom contact solver is added. A later arbitrary motion can
still exceed finite bounds and fault the run; constructor checks are not a
universal future-step safety guarantee.

Public regressions use independent transformed box corners and analytic
90-degree round-shape extrema, then explicitly compose both envelope stages.
They cover four primitive kinds, 2/4/16 unions, noncommuting collision/LINK/model
rotations, contacts just inside/outside boundaries, measured constant angular
and linear motion, pause, static/mask filtering, contact-limit terminal failure,
ordinary reset, loaded reconstruction and removal. Malformed/overflow and
Mesh/plane inputs stay refused. A centered singleton witness preserves the old
contact and constant-velocity behavior. Exact generic curved binary ray
premises and physical-resolution claims are excluded.

Root owns retained TPE posed/compound refusal-to-positive dispositions; authors
leave those bodies unchanged. No tests, lint, formatter, build, native campaign
or probes run during Source authoring. Batch all independent Source findings,
then use the single complete-candidate verification cycle with unchanged gates,
local/hosted qualification and original evidence audits. No capacity admission,
cleanup, old-target reuse, Fuel change or demo retirement is authorized here.
