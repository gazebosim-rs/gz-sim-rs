<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M69: two independently demanded metric-depth cameras

This Source increment selects exactly two homogeneous calibrated depth cameras.
`CameraServer::new_depth_pair` uses the separate
`Server::with_external_depth_camera_outputs` admission path. The existing
`new_pair` remains RGB-only; scalar constructors and mixed-pair refusals keep
those meanings. A depth pair retains its original two-owner intent after one
or both sensor entities disappear and through loaded reset. It never enrolls a
replacement or silently becomes scalar selection.

The fixture `examples/m69_two_depth_cameras.sdf` has five static `body` links:
`rig_a` at `(0,-2,1)`, `rig_b` at `(0,2,1)`, `backdrop` at `(6,0,1)`, and
`occluder_a` / `occluder_b` at `(2,-2,1)` / `(3,2,1)`. The backdrop is a
`(0.2,16,16)` box; both occluders are `(0.4,0.4,0.4)` boxes. Gravity is zero,
no world light is configured, and both cameras face world +X. Every visual's
oriented bounds lie strictly ahead of both near planes. There is no solver,
geometry, material or renderer change.

Both depth sensors are 64 by 64, horizontal field of view pi/2, near 0.1 m and
far 10 m. Their consumed SDF image format is the existing R8G8B8/default input.
Depth kind selects canonical `gz.msgs.Image` R_FLOAT32 (13): stride 256 and
16384 bytes of little-endian binary32 optical-forward axial metres, not
Euclidean range. A uses 10 Hz and `/m69/a/image`, `/m69/a/camera_info`, frame
`rig_a/body/camera`; B uses 20 Hz and the corresponding `/m69/b/` routes and
`rig_b/body/camera`. CameraInfo retains the full intrinsic, projection,
rectification, distortion and acquisition-stamp contract.

For an already-built matching CLI and worker, follow
[AUTOMATIC_CAMERA.md](AUTOMATIC_CAMERA.md). The homogeneous depth-pair path uses
the existing worker option:

```sh
GZ_PARTITION=m69-depth target/debug/gz-sim-rs examples/m69_two_depth_cameras.sdf \
  --server --run --camera-worker "$(pwd)/target/debug/gz-render-worker"
```

This is a Source-checkout example, not an installed-worker distribution or a
claim that this command has been run. M65 Latest and its 35 installed examples
remain unchanged. All M69 formatting, checks, native execution, qualification
and publication wait for actual M68 acceptance and complete independent Source
review.

## Independent expected geometry and actual delivery

The worker integration declares camera poses and the three visual boxes as
literal expected geometry. Its slab intersections use pixel centres, camera
forward X=1 and the existing projection basis, so the intersection parameter is
axial depth. No ECS transform, production extractor, readback decoder or observed
pixel supplies its own expected geometry. Every one of 4096 LE words per frame
must be finite and positive and within the retained 1e-4 m bound of that expected
field. Centre depths are 1.8 m and 2.8 m; the broad backdrop is 5.9 m and covers
every fixture ray. The two occluder silhouettes also have distinct widths. This
fixture is planar; it claims no new curved-surface numeric or native-renderer
parity. Existing primitive-depth controls remain inherited obligations.

Two upstream C++ consumers run simultaneously, one for each owner. The registered
camera peer has an explicit `depth-pair 0` / `depth-pair 1` mode; all earlier
modes and the no-argument body remain unchanged. Each consumer checks every LE
word against its separately declared planar projection, both typed headers and
full calibration. Rust additionally compares the entire reserialized canonical
Image and CameraInfo packets with the exact consumer receipts. Each stdout is
bounded to 64 KiB, stderr to 4 KiB, Image to 32768 bytes and Info to 16384 bytes;
the combined stdout/summary receipt is bounded to 128 KiB. Real child PIDs,
start times, actual wait status and original channels are retained, including
partial failure channels. Actual C++ wait0 is separate from worker retirement.

The three new real-worker controls cover two distinct physical worker processes,
A-only and B-only demand, Info-only laziness, independent 10/20 Hz stamps and
sequences, a busy owner's peer progressing without another physics tick, pause,
reset with no stale image, two-to-one-to-zero sensor retirement and original-ID
restoration. Sensor removal uses the admitted one-shot Finalize despawn seam;
the camera's model and body remain, and the public camera world-edit refusal
stays enforced. Worker retirement requires the actual `/proc/PID/stat` path to
be absent and the existing client lifetime to be retired. Zombies or reused PID
paths do not count as absence. Worker exit status remains UNKNOWN where the
application discards it.

The private application Busy control checks the outstanding acquisition's typed
`Busy` admission error before pumping. A subsequent `capture_all` may observe
that error or a real asynchronous worker failure; either error must leave the
other owner's exact camera-info clock progressing without another tick. Its
non-Ready test worker supplies no rendered frame. This is a CPU-only ownership
and scheduling control, separate from the real-worker metric-depth proofs.

These are Source-declared controls until the complete mandatory qualification
actually runs. No RGBD registration, point clouds, noise, dynamic registration,
new dependency, SDK/framework expansion, package refresh, analytic/native Ogre,
physical-GPU or performance comparison is added. Protocol 7, the shared family
session and two per-owner lazy workers/slots retain their existing bounds.
