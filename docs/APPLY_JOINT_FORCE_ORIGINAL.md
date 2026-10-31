<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Original ApplyJointForce command case

The assertion port covers the Linux
`ApplyJointForceTestFixture.JointVelocityCommand` case at pinned simulator
`71c60db31fddd5c98d2aa689e9a2cccb3548a0a3`. Despite its name, it publishes a force.
Both checked-in copies of `test/worlds/apply_joint_force.sdf` retain the exact
original bytes. The free parent, ground plane, inertias, geometry, gravity and
joint remain unchanged. No world hinge or replacement fixture is introduced.

The Rust server test retains ten initial component observations within 1e-4 of
zero, then a real native C++ Double publisher on the canonical topic, at most
1000 steps paced at 1 ms, the original 1e-6 arrival predicate and four-ULP final
equality to 0.001. Construction checks no initial or pending steps because Rust
runs only on explicit ticks; this maps the original no-running observation
without recreating the C++ Running API. A persistent JointForceCommand sampled
before each tick is the input applied on that tick. Native PreUpdate observes
its additive JointForceCmd before Physics clears it. The representation differs;
the selected single-command observations are preserved.

The unchanged native simulator original already passed 1/0, with no skip, in
immutable DART reference image
`sha256:4ac74e616369164eaba07149e9b98f7594a33cb3ddbbd6a45e0dd43cb688b1d5`.
Its [selected runtime evidence](../planning/reference/apply-joint-force-original.json)
and [original XML](../planning/reference/apply-joint-force-original.xml) are kept
separately from the fresh plugin physical observations and C++ to Rust server
replay. They do not imply a new execution of the whole native reference suite.

Independent review reproduced the real C++ server control and qualified the
bounded static-contact physics and exact assertion translations. The original
case is one test mapping; full system-family behavior, zero-cost absence, dynamic
external contacts, world-root articulation contacts, active multibody limits and
performance remain open. Strict staged and hosted gates remain required before
acceptance. See [the contact domain](../crates/gz-physics-dart/docs/ARTICULATED_CONTACTS.md).
