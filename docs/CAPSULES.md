# Capsule geometry

A capsule is a Gazebo Z-axis segment with a hemisphere at each end. SDF length
is the straight section length; total height is length + 2 * radius. Shared shape
values retain half the straight length. Rapier uses its existing f64 capsule_z
primitive, including mass properties and contact solving. TPE retains native
model-AABB contacts with local half-extents [radius, radius, length/2 + radius].

Physical shape construction requires positive finite dimensions. TPE also
rejects overflow of the combined cap extent. Rapier rejects unrepresentable
mass properties or bounds before inserting a body. A tiny positive straight
section may remain a valid near-sphere; it is distinct from a radius whose mass
properties underflow. SDF's native dimension defaulting precedes these checks.

The pinned native Shape.CapsuleShape original and eight touching, separated,
rotated and static TPE controls passed before production edits and in an
independent replay. The fixture records exact source/compiler/library hashes.
The default native CapsuleShape zero bounds and mutable getter/setter API
remain unported. Installed dependency/build provenance remains qualified.

Author and independent selected checks passed seven cases covering Z-axis mass
response, transformed ground support, invalid-state atomicity, native TPE
contacts and loaded IMU/reset/IDs. Three production-only controls fail the
original axis, contact and overflow predicates; exact source restoration passes.
The exact staged source gate passes 253 Rust tests, 75 tooling tests and all
fifteen unchanged real C++ client scenarios. Every production crate exceeds 95%
line and region coverage: workspace 3569/3605 lines and 5446/5565 regions; changed
Rapier lines are 6/6 and scene lines 13/13. Final metadata review, the mandatory
normal hook and exact hosted qualification remain separate requirements. Offset/multiple collision geometry, general physics
conformance, other backends and formal P2 remain open.


Reproduce the pinned native original and controls with
`scripts/probe-capsule.py --help`, providing the pinned physics source, its TPE
build and the qualified dependency prefix. Keep raw stdout, stderr, XML and
identity records. The public reference is `planning/reference/capsule.json`;
`planning/reviews/capsule.json` records independent evidence and remaining scope.
