<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M66: opaque sphere visuals in calibrated RGB snapshots

The bounded RGB path accepts sphere visuals together with boxes and cylinders.
`examples/m66_rgb_sphere.sdf` places a static camera at the origin, a red sphere
of radius 1 at `(4,0,0)`, a blue box backdrop at `(7,0,0)`, and a near green box
at `(2,0,0)`. Each model has link `body`. Collisions remain boxes and gravity is
zero; admitting the visual changes no physical body, inertia or solver behavior.

Use the maintained RGB-enabled CLI and its matching worker, following
[AUTOMATIC_CAMERA.md](AUTOMATIC_CAMERA.md). For an already-built pair:

```sh
GZ_PARTITION=m66-sphere target/debug/gz-sim-rs examples/m66_rgb_sphere.sdf \
  --server --run --camera-worker "$(pwd)/target/debug/gz-render-worker"
```

The camera emits canonical Image on `/m66/rgb/image` and CameraInfo on
`/m66/rgb/camera_info`, under the ordinary partition/topic mapping. Both carry
frame `rig/body/camera`; the configured image is 64×64 RGB at 10 Hz. The
primitive web viewer is a separate output, not a display of these RGB bytes.
An image subscriber is required before the software worker starts. Info-only
interest retains lazy worker behavior.

## Geometry and compatibility

The sphere is an opaque SDF visual with a positive finite canonical radius.
The extractor preserves its composed local/world pose and f64 origin; the
renderer subtracts origin and uniformly scales one cached radius-0.5 mesh by
`2*radius`. There is no cylinder-style axis correction. The mesh uses 32 UV
sectors and 18 stacks: it is a finite approximation, not an analytic sphere or
a native Ogre/physical-GPU parity claim.

Generic and depth extraction remain box-only and reject sphere geometry before
publication or worker creation. RGB retains the existing no-world-light,
default-material and 128-total-visual limits. Invalid canonical radii, diameter
overflow, f32 narrowing to zero/infinity and invalid poses fail before a request
is admitted. The consumed SDF parser normalizes some nonpositive raw radii;
this feature preserves that parser policy and validates the resulting canonical
component/API boundary independently.

The internal worker protocol is exactly version 5. Client and worker must match;
older peers fail the Ready handshake without fallback. Box-only requests retain
their original request form and pixel behavior. Image/calibration fields, Busy,
reset, cancellation, stale-response and ownership rules remain unchanged.

## Required evidence

The real software-worker controls check green occlusion at `(32,32)`, red sphere
interior at `(32,26)` and `(38,32)`, and blue at `(24,24)` and `(39,39)`. The blue
samples are inside the projected box proxy but outside the sphere silhouette,
away from antialiasing edges. Whole-frame equality under a shared large world
translation/origin guards the camera-frame conversion. Separate renderer
controls retain exact old-box pixels after a shape round trip.

Canonical controls decode complete Image/CameraInfo fields, acquisition stamps,
frame IDs, sequence, stride and calibration; retain the actual C++ peer messages
and streams; and require its owned exit zero. The peer consumes Rust software
pixels, not a native rendering scene. CameraServer retirement discards the worker
return code, so that code remains UNKNOWN. Reset/Busy and paused cadence controls
retain the existing assertions. A finite fixture load or a Source declaration
alone is not a rendered, canonical or qualification pass.

This Source feature does not refresh the M65 native-free installed preview or
distribute an RGB worker. Its accepted viewer bytes remain unchanged.
