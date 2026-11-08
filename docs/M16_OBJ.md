<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M16: bounded local OBJ geometry

This increment adds local triangle-only OBJ to the existing mesh collision and
immutable GLB visual paths. Source/review worktrees are unaccepted; qualification
and actual combined browser evidence are required before qualified Main promotion.
M12's installed distribution and accepted viewer are unchanged.

## Admission and precision

The maintained `tobj` 4.0.5 parser is MIT, edition 2021, with no declared MSRV.
Its exact crate checksum is
`6eb8e04167c1c0c76b5de63226fd733485ad63ef71e40de31e16272f47b099e2`.
Default features are disabled. It parses finite **f32** coordinates, matching
existing STL/glTF input precision; decoded geometry is promoted to the f64
collision solver. Decimal f64 source precision is not preserved. A face that
collapses after f32 rounding is refused. Positive finite SDF scale applies once,
and nonfinite or degenerate scaled geometry is refused too.

`tobj` eagerly builds vectors. Before invoking it, the admission envelope caps
UTF-8 input at 16 MiB, individual lines at 2048 bytes, position/UV/normal
declarations at 131072 each, faces at 262144 and object/group directives at 128.
Flattened output retains at most 128 models, 131072 unique positions and 262144
triangles. Indices are checked before upstream optional-channel arithmetic:
positive and relative negative indices must refer to existing declarations;
zero, forward, missing and out-of-range references are refused. No callback opens
materials or files. Original source bytes are never changed; a bounded temporary
buffer strips comments/whitespace for the maintained parser.

Admitted commands are comments, `v x y z`, `vt u v`, `vn x y z`, triangle `f`
records with `v`, `v/vt`, `v//vn` or `v/vt/vn` corners, bounded single-word ASCII
`o`/`g` names, and `s off` or an unsigned smoothing identifier. Finite checks
include unused declarations. Winding is retained; exact f32 position equality
with signed zero normalized shares an index, as in STL. No approximate welding,
repair or polygon triangulation occurs.

Normals, UVs, groups and smoothing are admitted as validated geometry metadata;
imported shading is not promised. The shared STL/OBJ GLB emitter computes
area-weighted geometric normals, using the deterministic first incident normal
when contributions cancel. It converts Z-up to existing glTF Y-up; the accepted
viewer reverses that basis and applies the visual's SDF scale once.

Unknown commands, free-form geometry, point/line faces, quads/ngons, homogeneous
vertex coordinates, vertex colors, line continuations, NUL/binary text,
`mtllib`/`usemtl`, textures and external resources are refused explicitly rather
than ignored. Filename/path, symlink, mesh options, recentering and scale admission
remain the same as existing local STL. There is no network or MTL loader.

## Existing integration boundaries

`gz_assets::mesh::decode_obj` returns unscaled Z-up `TriangleMesh`;
`obj_visual_glb` returns a self-contained generated GLB through the same emitter
as STL. `mesh::load` performs existing positive finite scaling for collisions.
The CLI captures `/assets/<scope>/<filename>.obj.glb`; raw OBJ and MTL files are
not served. Existing 128-file, 16 MiB/file and 64 MiB immutable resource limits,
alias retirement, recording/playback validation and HTTP lifetime remain intact.

Physics admission does not expand: existing Rapier static open surfaces and
admitted outward closed-convex dynamic meshes work; open dynamic surfaces,
unsupported backends, decomposition and optimization remain refused. No new
renderer, viewer build, schema, wire message, asset service or sensor widget is
introduced.

## Required evidence

New controls cover all approved index forms, relative indices/group boundaries,
f32 collapse, malformed/unsupported commands, explicit external-resource refusal,
input/declaration/face/line limits, nonuniform scale and existing glTF basis
round-trip. CLI controls exercise immutable captured bytes after source deletion,
recording admission, alias retirement, real HTTP bytes and atomic resource limits.
Scene controls exercise posed/scaled open-surface ray/contact/reset and closed
asymmetric tetrahedron ray/freefall/reset, using an admitted static box floor;
open dynamic geometry stays refused. Existing STL/glTF assertions remain intact.

One combined M15/M16 live scene must display an asymmetric OBJ on the real robot
and a fixed OBJ landmark, demonstrate forward/turn and Pause/Reset, and shut down
its owned processes cleanly. Quantitative mesh/collision assertions and canonical
pressure wire/native evidence remain separate from screenshots. This does not
claim materials, general OBJ parity, exact pixel-derived pose or performance.
