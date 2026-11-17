<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M68: calibrated depth for existing opaque primitives

One calibrated depth camera can observe the existing box, cylinder, sphere and
ellipsoid visuals. `examples/m68_depth_primitives.sdf` uses a static camera at
zero, a cylinder at `(4,-1.5,0)` with local radius 0.5/full length 1 and a visual-only
pitch of 90 degrees, a radius 0.75 sphere at `(4,0,0)`, and an ellipsoid at
`(4,1.5,0)` with local semiaxes `(0.5,0.6,0.35)`. A near box at `(2,0,0)` occludes
part of the sphere; a backdrop at `(7,0,0)` bounds the visible background.
All links are `body`, gravity is zero and collisions remain boxes. This extension
changes no body, inertia, collision or physics solver.

For an already-built matching RGB-feature CLI and worker, follow
[AUTOMATIC_CAMERA.md](AUTOMATIC_CAMERA.md):

```sh
GZ_PARTITION=m68-depth target/debug/gz-sim-rs examples/m68_depth_primitives.sdf \
  --server --run --camera-worker "$(pwd)/target/debug/gz-render-worker"
```

The sensor type is `depth_camera`; its SDF image metadata retains the accepted
R8G8B8/default input format. Sensor kind selects metric-depth output: canonical
`gz.msgs.Image` on `/m68/depth/image` is R_FLOAT32 (13), 64×64, stride 256, and 16384 bytes
of little-endian f32 optical-forward axial metres. It is not Euclidean range.
CameraInfo is on `/m68/depth/camera_info`; both use frame `rig/body/camera` and
10 Hz acquisition stamps. Background/far clipping is positive infinity. Successful
frames reject zero, negative and NaN words. The primitive web viewer remains a
separate output; this does not add a depth display or RGB-D registration.

Image interest starts the owned software worker lazily; Info-only interest does
not. Keep lifecycle and network configuration from the existing camera guide.
The retained `rgb-camera` feature name also enables the existing depth owner.
Generic extraction remains box-only, RGB behavior remains unchanged, and mixed
or depth pairs are refused by the two-RGB-camera API.

## Geometry and bounded admission

Depth reuses the actual accepted primitive meshes, including the shared fixed
UV 32×18 sphere/ellipsoid mesh; at most the same three mesh assets are cached.
The renderer keeps authoritative poses, f64 origin subtraction, explicit camera
calibration, default opaque material/no-world-light policy, 128 total visuals,
131072-byte IPC messages and one shared RGB/depth capture/readback slot.

The near restriction is deliberately conservative. Every oriented corner of each
primitive's enclosing box must be strictly beyond the near plane: cylinder local
XYZ dimensions `(2r,2r,L)`, sphere `(2r,2r,2r)`, ellipsoid `(2rx,2ry,2rz)`.
The old oriented-box guard is unchanged. A safe curved surface can therefore be
refused if its enclosing box crosses near; there is no analytic near-range or
native negative-infinity claim. The image renders the actual meshes, not those
bounds. The stored depth attachment is single-sample Depth32Float with COPY_SRC.
All fallible validation precedes renderer state/cancellation changes.

Existing consumed SDF dimension normalization is unchanged. Canonical semiaxes,
radii, lengths and doubled dimensions must remain finite, positive and narrow to
finite positive f32. Invalid canonical API/component tests do not claim that
nonpositive XML dimensions survived the parser's defaults. Exact worker protocol
version 7 adds a required calibrated depth-shape request. Older Ready versions
fail without fallback; old box-only depth and RGB request forms are preserved.

## Required proof

The fixture declares independent planar anchors: centre near-box 1.8 m, cylinder
cap at `(44,32)` 3.5 m, backdrop 6.9 m at proxy-trap pixels `(40,35)` and `(16,34)`,
and exact positive infinity at uncovered corners. Curved off-axis samples must
have nonflat depth rather than AABB-front depth. These are reviewed Source
premises, not observed images until the actual qualification run.

Renderer software controls use a few independent intersections with the exact
locked meshes, explicit contract transforms and narrowed calibration. Rays are
parameterized with camera-forward X=1 so hit t is axial metres. They do not derive
expectations from production EntityTransforms, installed meshes, decode helpers
or depth output. The retained finite-depth bound is 1e-4 m; ideal analytic curved
surfaces are not asserted at that tolerance. Separate controls cover origin and
common camera/scene transforms, exact equal-radii Sphere/Ellipsoid bytes, old
Box/RGB restoration, invalid-state purity, cache/readiness and RGB/depth reuse.

Real worker controls retain actual canonical Image/CameraInfo/calibration/stride/
frame/sequence/stamps,10 Hz/pause, Busy without retick, reset/stale cancellation,
far infinity and typed near refusal. The optional C++ `depth-primitives` mode
receives and decodes all canonical depth bytes and metadata, with explicit LE
words, bounded callbacks and a 64 KiB aggregate stdout guard. It retains its real
owned wait/exit and streams. The peer consumes Rust software pixels, not native
rendering. Worker exit stays UNKNOWN where unchanged CameraServer retirement
discards it. Imported selected-native support cases are counted separately from
new authored tests. An SDF load or Source declaration alone proves no pixels.

No noise, pointcloud, lighting/material, sensor-multiplicity, dependency/provider,
package refresh or worker distribution is added. M65 Latest/install 35 and accepted
viewer bytes remain preserved. No native Ogre, physical-GPU or performance parity
is claimed. All M68 checks remain held until M67 acceptance and completed review.
