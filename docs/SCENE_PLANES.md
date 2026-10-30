<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Static plane scene loading

This follow-up to DART_PLANES.md lowers a centered, local +Z plane on
an effectively static link through `DynamicBackend::spawn_static_plane`.
The existing Bevy body handle, pose synchronization, model removal and loaded
reset paths own its lifetime. No plane system, callback, worker or substitute
solid shape is introduced.

The bounded loader accepts only default plane inertia (mass 1, identity tensor
and coincident inertial frame). It explicitly rejects custom inertia rather
than discarding metadata that the current backend plane contract cannot carry.
Existing collision-count, surface, collision-offset, fluid-mass and link-property
guards continue to apply. Dynamic planes, other normals and other engines remain
unsupported; native DART's finite cube and size-independent collision behavior
are retained. Normalization of input normals remains the SDF parser's contract.

The new scene controls replay all 49 recorded native frame/trajectory samples
at absolute tolerance 1e-14, exercise model removal and loaded reset, and reject
unsupported input configurations. Existing scene assertions are retained.
The complete upstream ApplyJointForce world remains unqualified because
moving-parent joints and other required behaviors are still open. This increment
does not change consumer pins or claim broader plane/backend or performance parity.

Independent review and the full normal staged and hosted gates are required
before acceptance. Local test outcomes are recorded in ignored artifacts.
