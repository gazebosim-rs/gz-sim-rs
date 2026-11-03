<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M3: Rapier mesh collisions

Accepted and published on 2026-11-03 at simulator
[ea8048f1](https://github.com/gazebosim-rs/gz-sim-rs/commit/ea8048f1bf8c1ac3067bb29b07caa62651b6e202),
with [all sixteen exact hosted CI steps passing](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41644722929).
The unchanged accepted viewer is `5b67f45f`; its bundles were reused. M1/M2
remain preserved. [Launch and admitted input contract](M3_MESH_COLLISIONS.md).

Static triangle surfaces and one admitted closed, outward-oriented convex
boundary use maintained glTF 1.4.1 and Rapier/Parry f64 constructors. Complete
node matrices, axis conversion, SDF scale and collision/link frames are retained.
Dynamic admission checks manifold topology, connectivity and supporting planes
before construction; nonconvex/open input is explicitly refused. Exact-position
seam canonicalization does not move vertices or merge nearby positions. The
bounded f64 check conservatively refuses positive computed plane distances and
nonrepresentable arithmetic; arbitrary transformed meshes are not qualified by
an exact mathematical predicate. No hull approximation or decomposition occurs.

Named passing tests cover static and dynamic contact, anisotropic support,
explicit mass/inertia and world twist, resource lifetime, failed-spawn atomicity,
reset/removal/captured restore, original link frames and multi-link contact
groups. Asset boundary tests cover typed metadata/resource/accessor failures and
aggregate decode limits. Other engines refuse before mesh IO, preserving the
retained collision-geometry diagnostic. Old assertions and tolerances remain
unchanged.

## Required checks and independent review

Implementation, regressions, docs and independent source review preceded the
consolidated mandatory checks. Actual failures were corrected as reviewed
batches: an old diagnostic-priority regression, then missing asset and multi-link
coverage cases. The successful normal staged hook took 897.212 seconds with no
capacity stop. Exact hosted CI, independent local and official archive audits,
and independent review of actual browser/source/binary evidence all passed.
No assertion, denominator, coverage threshold or required job was weakened.

| Owned profile | Local lines | Local regions | Hosted lines | Hosted regions |
| --- | --- | --- | --- | --- |
| Headless | 12046 / 12264 | 18348 / 18908 | 12044 / 12264 | 18345 / 18908 |
| Renderer | 6256 / 6382 | 9233 / 9588 | 6256 / 6382 | 9233 / 9588 |

Every required crate passes both strict metrics separately. Profiles are not
combined. Assets pass at 505/515 lines and 819/847 regions; Rapier at 1300/1307
and 1998/2013. Actual headless parent-relative changed-line results are assets
327/328, physics 11/11, Rapier 236/238 and Scene 52/52. The renderer's changed
owned source result is separately Scene 52/52. Hosted CI uses a
separate zero/full-source base; its per-crate DA thresholds also pass.

Actual logs record 1016 headless, 559 renderer and 276 tooling passes, with zero
failures. The five/three ignored parent entries remain delegated helpers.
Ten native-off cases pass with four retained native-free executables, thirteen
actual raw profiles, disabled-arm observations and a system-library-only closure.
Eight retained real C++ Zenoh peers bind selected SDK/library identities; Zenoh
itself remains upstream Rust and unmodified.

Audits verified 2149 local artifact hashes, 557 distinct Git source/input bindings
including 78 production files, and all 1217 official source blobs/modes. The
274 full-profile executable records are metadata-only; no binary re-export is
claimed. Four native-off executables are retained. Hosted SDK DSOs are bound
through remote metadata, rather than claimed locally rehashable files.

## Actual browser observations and boundary

[Four original captures and source/binary bindings](evidence/m3-browser/README.md)
show the green mesh and sphere move onto the ground at current time 9.875 seconds,
committed pause at 19.765 seconds, and reset to epoch 1/time 0 with original
apparent placement at the same camera. The instrumented native-free executable
from the mandatory gate and accepted viewer bundles were reused without another
build. Background 1 fps is visible. This is not performance acceptance or
screenshot-derived quantitative geometry evidence; the M3 capture did not
separately inspect the renderer backend. Startup initially showed Waiting and
Connecting before the scene appeared.

M3 admits bounded local glTF/GLB collision input, static triangle surfaces and
already-convex dynamic solids. Dynamic proof is limited to 512 distinct positions
and 1024 triangles. Optimization/decomposition metadata, automatic decomposition,
deformables, heightmaps, arbitrary formats and other engine mesh support remain
outside acceptance. Broad asset/backend compatibility remains partial.

M4 now wires one demand-activated IMU model into the live Rapier world and
canonical messages, targeted within three working days including review and
mandatory checks. Other sensors and full noise-model parity remain deferred.
