<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M2: local glTF visuals

Accepted on 2026-11-03 on both public Main branches. See
[M2_ACCEPTANCE.md](M2_ACCEPTANCE.md) for exact commits, required checks and actual
browser evidence. M1 remains preserved.
M2 keeps Rapier primitive collisions and adds local glTF 2.0 visual geometry.
No mesh collision, remote download, general format conversion or new system
plugin is introduced. Target: three working days including independent review
and the consolidated required checks.

Use the same build and viewer setup as [MVP_BROWSER.md](MVP_BROWSER.md), with
`examples/mvp_local_mesh.sdf` in place of the primitive world. Both simulator
and viewer must include M2. The simulator command is:

```sh
cargo run --locked --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  examples/mvp_local_mesh.sdf --physics-engine rapier --server --web-viewer 9002
```

Open `http://127.0.0.1:8081/?backend=primitive&ui=full&url=ws://127.0.0.1:9002/`.
The green pyramid has an external vertex buffer, a texture, a rotated and offset
visual and nonuniform scale. Its collision is a box. The red sphere retains a
sphere collision. Embedded glTF materials remain authoritative for mesh visuals. Omit the SDF
material element on a mesh visual; SDF mesh material overrides are outside M2
and are explicitly rejected. Primitive visuals retain their existing required
opaque SDF diffuse materials.

## Local resources

Mesh URIs resolve against the original SDF resource's file URI, including an
included model's original source. Relative paths and explicit local file URIs
are admitted; `model://`, remote sources and non-glTF suffixes fail explicitly.
Submesh selection and recentering are outside M2. Filenames and declared relative
dependency paths use ASCII letters, digits, dot, dash, underscore and `/`, with
no empty, `.` or `..` segments, percent escapes, query, fragment or backslash.
This deliberately bounded filename subset is not a general Gazebo URI resolver.

The maintained glTF parser validates `.gltf` and `.glb` document structure before
serving; M2 additionally requires version `2.0`, with absent or `2.0` minVersion.
Only their declared buffer/image files are copied into an immutable resource
snapshot; symlinks cannot escape the document's canonical directory. Relative
subdirectories are preserved. Embedded data URIs remain the maintained loader's
responsibility. Malformed documents and missing or unreadable declared dependencies produce a
startup error with the resource path. Buffer content and image decoding are validated
by the maintained viewer loader; corrupt binary/image contents produce visible
asset loading errors rather than a startup success claim.

Limits are 128 declared files, 16 MiB per file and 64 MiB total. Repeated visual
instances share matching document resources. A mesh-containing viewer owns one
loopback ephemeral HTTP listener and one joined worker. Primitive-only worlds
and runs without `--web-viewer` create no asset listener, worker or retained file
snapshot. One active HTTP request, 32 headers, 8 KiB header buffer and a one-second
connection deadline bound slow peers and shutdown. Only declared GET routes are
served. Browser origins are the existing localhost/127.0.0.1 ports 8080/8081;
responses do not enable remote origins. Native requests without Origin are allowed.

## Scene and frame contract

M1 primitive-only scene messages remain version 1. A complete scene containing
meshes is version 2 and adds this shape:

```json
{"kind":"mesh","uri":"http://127.0.0.1:12345/assets/0/pyramid.gltf","scale":[1.5,0.75,2]}
```

Controls and acknowledgements remain version 1. IDs, timestamps, complete scene
bounds, reset epochs and lifecycle semantics are retained. The wire pose is the
raw SDF world visual pose in Z-up coordinates, using XYZW quaternions. Positive
finite scale is expressed in SDF geometry axes. The viewer converts the glTF
Y-up geometry once, then applies the world visual transform and the existing
Gazebo-root conversion. Specifically, mesh rotation is `worldQ * Rx(+90deg)`
and mesh node scale is `[sx,sz,sy]`, equivalent to
`worldQ * Scale(sx,sy,sz) * Rx(+90deg)`. The same correction applies to later pose
frames. Intrinsic glTF node transforms and materials are preserved; unchanged
URI/scale layout retains loaded asset handles.

## Required acceptance

Finish implementation, regression code, documentation and independent source
review before one consolidated verification pass. Retain all existing gates.
An actual browser must show correct orientation, nonuniform scale, declared
resource paths and continued motion, pause/reset and reconnect. Missing and
invalid asset cases must fail clearly. No M2 acceptance is inferred from source
review or an M1 browser image.
