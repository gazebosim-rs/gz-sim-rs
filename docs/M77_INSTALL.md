<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M77 installed camera preview

M77 prepares an Ubuntu 24.04 x86_64 preview with 47 finite examples through M76.
The earlier frozen `353dc9ae` package passed archive/install and finite-load
checks, but its first camera startup failed at the web Scene visibility guard.
That package and failure remain preserved. The current isolated correction is
unqualified and requires fresh same-source distribution/runtime evidence;
these commands do not assert its success. No M77 public release is claimed.
The public M70 package remains unchanged with 41 examples.
The next reviewed correction (`95ef6ced`) failed its one full normal in a
wrench-test presence-query EOF assumption, before Renderer. Its frozen Source
and logs remain preserved. This successor corrects the test helper's finite-query
completion/timeout handling without extending deadlines or replaying real puts;
new qualification and fresh same-source distribution remain pending.
Use the package's manifest and release checksums to identify actual delivered
bytes; a later source checkout does not update an installed package.

The producer requires Python 3.11 or newer and ordinary OS runtime libraries.
It needs no compiler, development checkout or Gazebo SDK. Optional camera output
also requires the system Vulkan loader and Ubuntu lavapipe provider. Install
`libvulkan1` and `mesa-vulkan-drivers` through ordinary signed Ubuntu APT when
using that profile; the package supplies no graphics driver or physical GPU.
Read CAPABILITIES.md for the bounded camera domain and qualification state.

## Extract and verify

The M77 archive name and its top-level directory are deliberately consistent.
After obtaining the archive and checksum from the same delivered release, use a
new directory without replacing another installation:

```sh
set -eu
sha256sum --check gz-preview-m77-linux-x86_64.tar.gz.sha256
preview_parent="$HOME/gz-preview-m77-install"
mkdir "$preview_parent"
tar -xzf gz-preview-m77-linux-x86_64.tar.gz -C "$preview_parent"
preview_prefix="$preview_parent/gz-preview-m77-linux-x86_64"
"$preview_prefix/bin/gz-preview" verify
```

Keep the complete prefix together when relocating it. Verification checks all
four executables, the launcher, examples, viewer assets, guides and notices by
bytes, size and mode. Do not edit the manifest or substitute a worker binary.

## Default robot

```sh
export GZ_PARTITION="gz-preview-m77-$$"
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" run --http-port 8082 --ws-port 9014 --lifecycle
```

Open the complete fresh token URL printed by this launch. The robot starts
paused. Use Play before sending finite commands from another terminal with the
same prefix and partition:

```sh
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" drive 0.4 0 3
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" drive 0 0.8 3
```

Pause and Reset are separate controls. Stop the owned launcher with Ctrl-C and
wait for its processes and listeners to retire before another launch. Occupied
ports fail; do not stop an unrelated service. HTTP choices are 8080–8082.

## Mixed RGB and depth cameras

Camera output and mixed-pair selection are both explicit. Stop the previous
launch, then run one of these commands at a time:

```sh
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" run --http-port 8082 --ws-port 9014 --camera-output --camera-mixed-pair --example m76_camera_projection_offsets.sdf
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" run --http-port 8082 --ws-port 9014 --camera-output --camera-mixed-pair --example m74_camera_skew.sdf
```

`--camera-mixed-pair` is available only in M77 and requires `--camera-output`.
Neither option takes a worker path. The launcher selects the manifest-verified
packaged worker and ordinary system lavapipe. Camera output cannot be combined
with `--lifecycle`; filenames never implicitly enable camera options.

Use Play or a canonical world-control client to advance the paused world. External
upstream Zenoh consumers use the same `GZ_PARTITION` and endpoint
`tcp/127.0.0.1:7449`. The M76 topics are `/m76/rgb/image`,
`/m76/rgb/camera_info`, `/m76/depth/image` and `/m76/depth/camera_info`;
M74 uses the corresponding `/m74/` topics. These are canonical `gz.msgs.Image`
and `gz.msgs.CameraInfo` streams. RGB is RGB_INT8 and depth is R_FLOAT32 axial
depth. The browser scene does not include a camera image or depth panel.

The default viewer uses an implicit all-bits mask, so the corrected Scene
export accepts nonzero visual flags including the fixture's 4/1/2. Zero visual
flags and transparent or unsupported materials are still refused. This does not
change per-camera visibility selection or add a camera image panel. The changed
Source has not yet supplied fresh installed-camera evidence.

The M76 pair has independent 10 Hz RGB and 20 Hz depth rates, visibility masks,
distinct intrinsic/projection matrices and signed metadata offsets. M74 instead
exercises signed intrinsic skew. Information-only interest starts no renderer
worker; image demand belongs to its camera. See PREVIEW.md for finite loads and
CAPABILITIES.md for limits. The ellipsoid example keeps its native-only route;
it is not a supported web Scene export.
