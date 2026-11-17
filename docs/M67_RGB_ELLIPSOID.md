<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M67: opaque ellipsoid visuals in calibrated RGB snapshots

The calibrated RGB path admits ellipsoid visuals alongside boxes, cylinders and
spheres. `examples/m67_rgb_ellipsoid.sdf` places the camera at the origin, a red
ellipsoid at `(4,0,0)` with local semiaxes `(0.5,1.5,0.75)`, a blue backdrop at
`(7,0,0)` and a green occluding box at `(2,0,0)`. Each model has link `body`.
The static world has zero gravity and ordinary box collisions. This visual
extension changes no body, inertia, collision shape or solver behavior.

For an already-built RGB-enabled CLI and its matching worker, follow
[AUTOMATIC_CAMERA.md](AUTOMATIC_CAMERA.md):

```sh
GZ_PARTITION=m67-ellipsoid target/debug/gz-sim-rs examples/m67_rgb_ellipsoid.sdf \
  --server --run --camera-worker "$(pwd)/target/debug/gz-render-worker"
```

The canonical routes are `/m67/rgb/image` and `/m67/rgb/camera_info`, under the
ordinary partition mapping, with frame `rig/body/camera`, 64×64 RGB and 10 Hz.
The primitive web viewer is a separate output, not a display of these RGB bytes.
Image interest starts the software worker lazily; Info-only interest does not.

## Geometry and compatibility

SDF radii are local XYZ semiaxes. The extractor keeps their order and the
composed authoritative visual pose. The renderer reuses the accepted cached
radius-0.5 UV sphere mesh (32 sectors, 18 stacks), with local diameters
`(2rx,2ry,2rz)` mapped to Bevy scale `(2rx,2rz,2ry)` through the unchanged frame
conversion. There is no additional axis rotation or fourth/per-size mesh cache.
This finite approximation is not an analytic-geometry, native Ogre, physical-GPU
or performance parity claim.

All semiaxes and diameters must be finite, positive and narrow to finite positive
f32 before renderer state changes. Existing default opaque materials, no-world-
light policy, shared 128 visuals, calibration, frame-size and single-slot bounds
remain. Generic and depth extraction stay box-only and reject ellipsoids before
IO. The codec retains its existing finite-only validation; semantic positivity
and representability are enforced by extraction and renderer admission.

The consumed SDF parser defaults missing radii to `(1,1,1)`, requires exactly
three finite numeric values, and replaces the entire triple by `(1,1,1)` if any
component is nonpositive. This policy is unchanged. Invalid canonical-component
faults are tested separately; a negative XML input is not described as an invalid
canonical ellipsoid surviving that normalization.

Internal client and worker must agree on protocol version 6. Version 5 and older
Ready records fail without fallback. Old shape and box-only request payloads,
legacy setters and existing box/cylinder/sphere behavior remain preserved.

## Required proof

The baseline pixel obligations are green `(32,32)`, red `(42,32)` and `(32,27)`,
and blue `(43,37)` and `(32,41)`. A local visual roll of 90 degrees about Gazebo X
swaps the unequal silhouette axes while leaving body, collision and camera
unchanged. The red/blue patches discriminate a single-radius sphere and a box
proxy, with blue backdrop visible outside the ellipse and green in front.
These analytical interior-pixel premises require actual software-rendered proof;
Source declarations alone are not observed output.

Controls require full-frame equality between an equal-radii ellipsoid and the
accepted Sphere path, shared large-origin/camera equivalence, and renderer shape
round trips restoring old pixels. They retain actual cadence, pause, Busy,
cancel/reset and stale-frame assertions. The C++ ellipsoid mode consumes the real
software-worker Image and CameraInfo, retaining complete canonical bytes,
calibration/frame/sequence/stamp and its actual owned wait/exit zero. It does not
render a native scene. Worker exit remains UNKNOWN where CameraServer retirement
discards it. Two unchanged selected-native support tests instantiated in the new
integration target are counted separately from authored feature definitions.

This Source feature does not refresh M65's installed 35-example package or
distribute an RGB worker. Accepted viewer bytes and prior evidence remain intact.
