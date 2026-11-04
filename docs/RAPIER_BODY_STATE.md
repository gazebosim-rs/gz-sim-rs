<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Focused Rapier body-state replacement

M10 adds `DynamicBackend::validate_body_state` and `set_body_state` for one
existing, unjointed Rapier body. Other engines return typed `Unsupported` before
validating IDs or values. This is source implementation pending the complete M10
feature's independent review and normal required verification; no runtime result
is claimed here.

The existing `BodyState` specifies the body-origin position, xyzw orientation,
world **center-of-mass** linear velocity and world angular velocity. Scene callers
that expose origin velocity must convert it using the requested orientation and
the declared center of mass. A fixed body admits pose edits only with exactly zero
linear and angular velocities. Every impulse- or reduced-joint participant is
refused, preserving accepted motors and continuous joint coordinates.

Both operations use the same immutable preparation: require an existing ID,
refuse joints, check all finite scalar inputs and the existing construction
quaternion tolerance (squared length within `1e-12` of one), then compute each
maintained collision shape's AABB under requested body pose times its admitted
local collision pose. Nonfinite bounds are refused before mutation. Quaternions
are not silently normalized. The setter then calls Rapier 0.36.1's maintained
`RigidBody::set_position`, `set_linvel` and `set_angvel`. No fallible admission
remains between those native setters.

`set_position` updates native world mass properties, marks position changes and
wakes a dynamic body. This is an explicit state edit that bypasses collision
resolution, not controller actuation. Contacts resolve on the next ordinary
physics step. Collider cached world poses may lag until that step; the accepted
read-only ray path already composes the current parent-body and local collider
poses, so a ray immediately observes the edit without a query-cache update or a
hidden physics step.

Success preserves IDs, admitted collider ownership, collision settings, spawn
records, other bodies and queued world forces/torques. Failure preserves all
state. Ordinary reset still restores surviving bodies to their spawn records;
captured-initial reset restores initial bodies, removes runtime additions and
retains the public ID high-water mark. A state edit does not redefine either
reset contract.

Standalone primitive and mesh spawn/remove already work after an advancing step.
Joint construction and runtime self-collision-group editing retain their existing
closed-construction refusals. Mesh admission retains static triangle surfaces and
already-convex dynamic solids; M10 introduces no hull/decomposition path or asset
format expansion. Scene must finish resource and public-identity admission before
its synchronous backend/ECS commit. No reset or cloned-world transaction is used.

Authored controls cover post-step insertion/removal, all admitted primitive ray
frames, an offset and rotated mesh, fixed-body velocity refusal, invalid numeric
and transformed-bound atomicity, COM velocity/rotated inertia, pending world and
generalized work, both native joint kinds, both resets and default-backend
capability refusal. These controls remain unexecuted until the whole M10 batch
is complete and reviewed. Existing M8/M9 motor policies and assertions are intact.
