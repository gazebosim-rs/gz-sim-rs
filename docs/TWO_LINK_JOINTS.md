<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Initial two-link SDF qualification

The initial qualification at `9e8e056` admitted one revolute connecting two direct links of a
single top-level dynamic model on Rapier. Both endpoint names must resolve to
those links. SDF graph transforms resolve the initial world anchor and axis;
the backend constructs both local frames. The existing one-link world revolute
remains supported on all three dynamic engines.

Body-joint samples report relative angle and axis velocity. Their runtime joint
pose follows the child's construction frame, while ECS ownership stays with the
model. Both endpoints, their velocity/pose components, and the original model owner
are checked before
stepping or model removal. Loaded reset restores the original constraints,
public identities, frames and zero joint coordinates.

DART and MuJoCo explicitly reject body-to-body construction until their adapters
implement it. TPE rejects dynamic joints. There is no world-anchor fallback.
More than two links, multiple joints, static or nested articulated models, and
frame/model aliases as endpoints remain unsupported. Limits, dynamics, motors
and closed loops remain unsupported. Model-wide self-collision filtering must
be implemented before admitting larger trees: the current primitive disables
contact between its two adjacent bodies only.

New tests exercise both endpoints, translated and rotated construction frames,
relative motion, child-attached joint poses, model removal/reset, corruption
before stepping, native capability errors and topology boundaries. Existing
world-joint/native-reference tests are retained. These are supplemental
capability tests, not completion of an upstream articulation test family.
No new cross-engine accuracy or equal-work performance claim is made.

That slice passed independent review and the normal strict staged/hosted checks.
Expanded local-link graph support is described in
[ARTICULATED_SCENES.md](ARTICULATED_SCENES.md); the restrictions above record
this earlier qualification's scope.
