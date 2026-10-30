<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Owned scene visual geometry

Loaded visual entities retain their parsed `sdformat::Geometry` in the Bevy
`VisualGeometry` component. A missing geometry remains absent. This is scene
description data for consumers such as rendering and scene publication; physical
collision construction uses its existing collision declarations independently.

Frame resolution borrows the parsed SDF. After construction completes, the loader
moves geometry values into the matching visual entities in construction order:
models, their links and visuals, then child models. A temporary entity list
preserves that association without another name registry or reparsing XML. It
includes visuals without geometry, so later entries retain their correct owners.
The existing producer revision stays pinned; no geometry cloning API is required.

Each value is held in an `Arc`, shared with the immutable loaded-reset blueprint.
Replacing a live component's Arc leaves the initial value intact. Loaded reset
restores the original value and public entity ID after edits or model removal.
The blueprint retains reset data until the simulation is dropped. No asset or
mesh resource is opened by this transfer, and no renderer dependency, worker or
scheduled callback is added.

The controls preserve all existing assertions and cover distinct visual/collision
shapes, ownership after the input string is dropped, replacement/reset/removal and
final release on all four engines. Nested model controls also retain missing
geometry and unresolved mesh URI/scale/source metadata in the correct order.
This prerequisite does not implement rendering, material effects or mesh loading;
those consumers must validate and support the geometry they use.
