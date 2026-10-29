<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Top-level model contact defaults

Native dartsim disables a model's internal contacts by default, independently of
whether it contains joints. The earlier Rapier scene loader only assigned contact
exclusion groups to articulated models. Two overlapping, unconstrained links in
one ordinary model therefore separated in Rust while the native reference stayed
at its initial positions.

The candidate extends the existing group assignment to top-level models with
multiple direct links. Each actual model keeps its own backend-local group;
contacts with other models remain enabled. Single free-link model behavior and
native adapters are unchanged. Nested-model contact policy and explicit
self-collision declarations remain outside this qualification.

The regression uses the same free sphere positions, dimensions, default inertia,
gravity and step duration as native mode0 in planning/reference/model-contacts.json.
The existing distinct-model mode1 provides the positive contact control. Only
contact presence is compared across engines; solver response values are not
claimed equal. Direct backend velocities also verify independent link motion,
then model removal and loaded reset preserve the original identities and groups.
Existing scene input admission is unchanged.

The exact same-model regression fails before the group assignment correction and
passes afterward. Existing assertions and numerical tolerances are unchanged.
Independent review and the normal strict staged/hosted checks remain required
before this candidate is accepted. No performance improvement is claimed.
