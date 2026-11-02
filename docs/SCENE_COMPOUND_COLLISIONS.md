<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Bounded DART compound collision loading

This source candidate admits one to three supported primitive collisions on a
DART link. A single centered primitive keeps the existing construction path.
Compound or offset primitives use one native body with explicit resolved link
inertia. Shape masses are not synthesized from density. Link COM must remain
centered, as before. Other backends retain their single centered collider domain;
a plane keeps its existing dedicated static path and cannot join a compound.

The loader validates every collision's surface, geometry and semantic local pose.
Collision entities retain their individual density metadata, local/world poses,
declaration identities and AttachedPose lifecycle. Initial-state capture and
reset must reconstruct every shape without assigning extra body identities.
No optional-system schedule, worker or endpoint is introduced.

Scene controls compare all body-state fields against the direct DART API, check
individual collision metadata and attachment transforms, reset and removal, and
retain explicit negative cases for other engines, unsupported final collisions,
surfaces, planes, counts and offset COM. Backend controls supply native shape,
contact/bounds and failure-atomicity coverage. These controls are source only
until the complete feature passes independent review and consolidated gates.

The unchanged B03 world still contains offset link COM and remains unsupported.
No B03 trajectory, general contact or equal-work performance acceptance follows.

Prior independent source review approved only the unchanged B03 assertion's test
name and expected first unsupported entity/feature transition to upper_link and
`offset center of mass`. Its literal XML, URI, DART invocation and complete sun
metadata assertion remain intact. The centered base's two primitives are within
the reviewed backend bounds; an unexpected earlier runtime failure must remain
a failure of this candidate, rather than being relabeled to pass that test.
