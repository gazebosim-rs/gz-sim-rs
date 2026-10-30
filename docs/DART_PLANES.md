<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# DART static planes: bounded adapter

This PHY-03 increment adds `DynamicBackend::spawn_static_plane` for DART.
Bounded scene loading and its lifecycle are described in SCENE_PLANES.md.
Other engines, arbitrary collision offsets and plane normals, moving-parent
joints and the complete original ApplyJointForce fixture remain open.

`PlaneSpec` describes a fixed link frame with a local +Z plane and two positive
size values. The DART adapter reproduces the pinned native engine's
`ConstructPlane`: a 2,100 m cube translated -1,050 m along collision-local Z.
SDF size does not change that physical cube. Rotation moves the cube with the
link; readback continues to return the link frame. This is finite native geometry,
not an infinite-plane approximation. Other adapters return an explicit unsupported
capability before allocating an identifier.

Admission retains decision D25. Both size values must lie in [1e-60, 1e60],
the orientation must satisfy the existing unit-quaternion contract, and collision
bounds must stay within ±5e8 m on each world axis. The implementation conservatively
checks both the centered cube and its translated physical bounds. It can reject
a frame near the positive boundary even when the translated cube alone fits;
asymmetric and rotated boundary tests preserve this explicit restriction.
Validation precedes all native mutation and identifier allocation.

The existing native shape tag carries the translated-cube distinction through
removal and both reset paths. No new per-body allocation or simulation callback
is added. Fixed-body velocity commands retain the existing accepted no-op behavior.
No solver, contact, friction or numerical tolerance is changed.

## Evidence and reproduction

Six adapter controls cover frames, settling, reset, removal, ignored size,
finite extent, invalid input, identifier preservation and unsupported engines.
They include 49 native samples across seven trajectories: center, small and large
size, outside the cube, its footprint edge, a tilted frame and a translated frame.
Every ground and ball position, quaternion, linear velocity and angular velocity
component is compared at absolute tolerance 1e-14 without quaternion sign relaxation.

Run the native generator from the repository root:

```sh
python3 scripts/probe-dart-plane.py \
  --physics-source ../gz-physics --prefix /home/nkoenig/local \
  --work-dir artifacts/plane-native
```

The default mode compares the recorded fixture. `--write-fixture` is only for
deliberate regeneration and requires independent review of any changed evidence.
The runner checks the pinned source revision, selected source and installed header
hashes, DART 6.13.2 and actual loaded native engine/DART/ODE library paths. Tracked
evidence records source, generator and loaded-library hashes; local provenance
also records compiler, command, binary hash and paths. This qualifies an installed
native engine, not a fresh source build or a performance improvement. Python
controls reject stale generator/baseline identities and missing trajectory cases.
The C++ helper uses the reference engine loader; Rust simulation systems remain
built into the workspace under D26.
