<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Unbounded reduced-coordinate revolutes

`DynamicBackend::add_reduced_revolute` is an additive programmatic API for
unbounded acyclic articulations. Rapier uses its maintained `MultibodyJointSet`;
existing impulse constructors and the SDF loader keep their existing solver.
`None` parent anchors to the world; otherwise the distinct parent is a registered
fixed or dynamic body. The child is dynamic. This initial API requires centered local inertias; signed
zero is accepted as zero. Anchor and axis are world-frame
values, the construction pose is zero, and the reported coordinate retains full
turns. Construction closes on the first advancing step and reopens after reset.

Each child has at most one parent. The constructor uses maintained Rapier tree
identities and parent-link indices to reject cycles and second parents before
insertion. Endpoint constraint membership rejects mixed components in both
constructor directions; this invariant is maintained on every insertion. This preflight matters: an upstream failed multibody insertion
can allocate roots before returning `None`. Contacts between separate components
remain supported. Independent world roots receive separate fixed anchors so
unrelated models do not become one multibody tree.

Both current endpoint velocities and the immutable spawn velocities used by
reset must satisfy the articulation. Their joint
anchor velocities must agree and relative angular velocity must lie on the free
axis, within a relative numerical admission tolerance of 1e-10 with scale at
least one. Incompatible or unrepresentable derived velocities return explicit
`Unsupported` without mutation. The constructor does not silently project away
motion. Generalized velocities are initialized from those admitted body values
before the first step, including after both reset paths. Rapier's default 0.1
multibody angular damping is disabled because these constructors specify no
joint damping or root drag.

After a step, the adapter uses Rapier's maintained body Jacobians and current
generalized velocities to refresh body velocity caches; the upstream body cache
otherwise precedes the last multibody solver update. IMU and joint sampling must
observe the same completed step. Direct world-velocity edits and impulses on
multibody endpoints are explicitly unsupported before mutation; world wrenches
remain supported through the engine's articulated dynamics. The accepted impulse
endpoint behavior is unchanged.

A free-root offset-COM probe currently violates world center-of-mass conservation
while the identical impulse case passes. The new constructor therefore rejects
nonzero local centers of mass on either endpoint atomically. This is a
conservative initial scope, not a claim every offset configuration fails. The
original physical input and assertions remain retained as unaccepted future
qualification evidence; accepted impulse/full-inertia behavior is unchanged.

Public joint IDs survive upstream tree splits and internal reindexing. Removing
a joint preserves surviving world poses and velocities; removing a body retires
its incident joints. After splitting, remaining generalized velocities are
resynchronized from body state so new free roots retain their absolute motion.
Ordinary reset restores remaining construction records;
prepared reset restores its captured bodies, frames and IDs with monotonic later
allocation. Unused fixed anchors are retired.

DART and MuJoCo explicitly reject this additive capability before validating
arguments. Their existing world hinges remain unchanged. TPE has no dynamic
joint API. This slice does not add SDF loading, native body trees, finite limits,
motors, damping controls, or upstream test-case completion.

Finite limits remain unqualified. Separate retained prototypes demonstrate
impulse winding loss near a full turn and multibody first-crossing overshoot.
Correcting initial velocities and removing hidden damping exposed failures even
with eight solver iterations. Those assertions remain intact; neither clipping
nor wrapping sampled coordinates is an acceptable fix. This unbounded slice
keeps default solver iterations and per-body iteration settings unchanged.

Fifteen public physical/lifecycle tests and a private forest/solver-setting control
supplement all prior tests. They exercise continuous relative motion, torque and
momentum, a changing world axis, nonidentity frames and an offset anchor, atomic topology and velocity
rejection, live splitting, endpoint removal, both resets and ID retirement.
Coverage, independent review and exact hosted CI are still required. No speedup
or cross-engine numerical parity is claimed. Initial synchronization and the
final Jacobian refresh have a cost that needs representative articulation
benchmarks before a performance claim.
