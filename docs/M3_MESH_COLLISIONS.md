<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M3: bounded Rapier mesh collisions

Accepted on 2026-11-03; see [M3_ACCEPTANCE.md](M3_ACCEPTANCE.md).
The phase budget was at most five
working days including independent source review and mandatory verification.
M1/M2 remain the accepted baseline. This phase adds static triangle-mesh contact
and one admitted dynamic convex hull, with geometry, contact and reset tests.
Unsupported nonconvex dynamic input must fail explicitly; no silent hull
approximation, automatic decomposition, heightmaps or general format conversion.

## Reuse and missing semantics

Use pinned glTF 1.4.1 maintained readers with only the `utils` feature, and
Rapier f64 0.36.1 / Parry f64 0.31.1 mesh constructors. Do not add an asset
runtime, transport facade or generic physics framework. Decode only loaded mesh
collisions; absent mesh collisions create no decoded buffers, workers or IO.

The maintained glTF readers assume valid count/stride arithmetic; preflight
nonzero counts, checked offsets/ranges and decoded-size budgets before calling
them. Sparse collision accessors, skin/morph/animated collision input and required
extensions remain explicitly outside this bounded subset. CPU collision decoding
does not read images or create Bevy resources.

`convex_hull` computes an enclosing hull for concave point clouds, and
`convex_mesh` assumes convex input. Their success is not convex-only admission.
Parry's geometry checker asserts rather than returning typed input errors.
This leaves a specific semantic gap: bounded typed rejection of nonclosed,
misoriented or nonconvex dynamic input before the maintained hull constructor.
Implement only the needed closed/oriented topology and supporting-plane guards,
with independent analytic convex/concave/open/degenerate controls. Do not catch
panics or silently repair/decompose geometry. Static planar triangle surfaces
must not inherit the primitive positive-volume/dynamic-inertia requirement.

## Frame and ownership contract

For a glTF vertex, bake full parent-times-child node matrices without TRS
re-decomposition, then apply `Rx(+90 degrees)` and SDF geometry-axis scale.
Apply the existing collision-local rigid pose and body pose afterward. Physics
stays Z-up; do not apply the viewer's global root conversion. This matches M2's
preserved node transforms and outer rotation/scale permutation. Reflected node
matrices require matching triangle winding reversal. Select the default scene,
otherwise the first, and preserve repeated node instances.

Use typed mesh construction separate from the existing Copy primitive shape
contract where practical. Other engines must return their existing explicit
unsupported capability outcome before mesh IO. Retain original mesh construction
and ownership for atomic reset/removal; never substitute a primitive or lose
manual link mass/inertia semantics.

## Implementation state

The first source increment adds a bounded CPU decoder in `gz-assets::mesh`:
local versioned glTF/GLB, declared local/base64 binary buffers, full f64 node
matrix baking, checked accessors and triangle/index validation. Current budgets
are 16 MiB per file, 64 MiB total data, 128 buffers, 131072 decoded vertices,
262144 triangles, 1024 node instances and depth 32.

The source increment also wires a borrowed `MeshBodySpec` into Rapier and
Scene loading. Rapier retains an immutable admitted collider and spawn kinematics
for reset, including the collision-local frame, without reading the resource
again. Other backends refuse before IO. Mesh optimization/decomposition metadata
is explicitly refused. `examples/mvp_mesh_collisions.sdf` reuses M2's local
pyramid with matching visual and collision poses/scales; the accepted M2 demo
itself is unchanged. Regression source covers planar contact, dynamic inertia,
resource ownership, failure atomicity and loaded reset.

Dynamic input is bounded to 512 distinct positions and 1024 triangles before
its O(vertices × faces) support-plane proof. Exactly equal finite coordinate
triples (including normalized signed zero) identify normal/UV seams without
moving geometry or merging nearby positions. The boundary must be connected,
closed, outward-oriented and manifold, with every vertex used. Any computed
positive support-plane distance or nonrepresentable arithmetic is refused;
this conservative f64 subset is not an exact-predicate qualification of arbitrary
transformed meshes. No numerical tolerance, hull approximation or decomposition
is applied.

Implementation and regression source are complete. Independent source review
cleared the implementation and its batched corrections before the consolidated
verification pass. Required local and exact hosted checks, independent evidence
reviews and actual browser motion/contact/reset observations passed; exact
results and limits are recorded in M3_ACCEPTANCE.md.

Use the viewer setup in [MVP_BROWSER.md](MVP_BROWSER.md), with accepted M2
viewer code. The candidate native-free simulator command is:

```sh
cargo run --locked --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  examples/mvp_mesh_collisions.sdf --physics-engine rapier --server --web-viewer 9005
```

Open `http://127.0.0.1:8081/?backend=primitive&ui=full&url=ws://127.0.0.1:9005/`.
The green local pyramid now uses its mesh as the collision boundary, with the
same local pose and scale as its visual; the red sphere and box ground remain
primitive controls. The existing gzweb-rs mesh renderer is reused without a
viewer source change. The acceptance captures reused the exact qualified CLI
and accepted viewer bundles; see the acceptance record for their bindings.

Complete the whole bounded feature, regression code, docs and independent source
review before one consolidated required verification pass. Preserve every
retained assertion, tolerance and quality gate.
