<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Read-only Rapier collision ray queries

M7 source adds `DynamicBackend::cast_ray(origin, direction, max_distance, exclude)`.
Inputs use world axes and metres. Origin and endpoint must be finite; direction
must have unit squared length within `1e-12`; maximum distance is finite and
positive. Numeric refusal is `PhysicsError::InvalidValue`; a missing excluded
body is `MissingBody`. Other engines retain the default `Unsupported` capability
error before input validation.

The result is the nearest collision distance on the inclusive segment, including
zero when the origin is inside a solid, or `None` for no hit. Only the explicitly
excluded body's colliders are omitted. Solver collision groups do not hide other
bodies from these sensor queries. Minimum-range clamping, no-return infinity,
scan angles and cadence belong to the lidar model, not the physics API.

The adapter iterates existing colliders and calls the maintained Parry `RayCast`
implementation on each admitted primitive or mesh. It uses current rigid-body
position/orientation multiplied by collider-local pose for attached shapes;
standalone colliders use their world pose. It does not rely on the collider world
pose cache or the broad-phase tree, both of which can lag construction or body
edits until a pipeline update. No physics step, collision detection, BVH rebuild,
contact mutation, body allocation or pending-command consumption is introduced.
This bounded implementation has linear cost in collider count per ray and makes
no performance claim. M7 separately caps its planar scan at 128 rays.

New regression source covers all admitted primitive families, rotated frames,
inclusive maximum/inside/nearest/miss semantics, parent-only exclusion, collision
groups, malformed rays, pending work and IDs, finite static triangle surfaces and
owned dynamic convex meshes. Both ordinary and captured-initial reset and removal
are queried immediately without a preparatory step. Private controls demonstrate
a stale collider/BVH cache after maintained body/local-pose setters, and verify
the read-only query uses fresh composed poses while leaving that cache untouched.

Implementation and regression source are unexecuted pending whole-M7 completion,
independent source review and the consolidated required verification. Existing
physics tests and all IMU/RGB/depth behavior remain unchanged. No native lidar
parity, visual/material reflectance model or broader engine query support is
claimed.
