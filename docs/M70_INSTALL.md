<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M70 optional installed-camera preview

This is Source preparation, not an available M70 release. No M70 installed,
frame, browser or runtime-fit result is claimed for this corrected candidate.
M1–M69 and 202 slices are accepted.
M65 remains public Latest with 35 installed examples. M69 at
`d19d5a69b94ed959bf1348665783996a531a3c36` is accepted on Main after local,
hosted and independent evidence gates. The preceding candidate's retained
qualification, distribution and setup records remain separate from this
launcher cache/custody correction and its failed browser attempts. Complete
Source and independent review before the corrected candidate's consolidated
checks and fresh distribution/runtime evidence. Publication remains held.
The five-working-day budget includes review and mandatory gates; it is not a
release or resource-fit guarantee.

## Install the matching qualified artifact when published

The admitted target is Ubuntu 24.04 x86_64, with Python 3.11 or newer and the
ordinary OS libraries identified by the actual release dependency record. The
optional camera profile also requires the specifically admitted ordinary system
lavapipe Vulkan ICD and its libraries. This package does not bundle graphics
providers, a compiler, source checkout or a Gazebo SDK. A fresh clean non-root
installation must prove those boundaries before release.

Obtain the archive and matching checksum from the same actual release. These
are proposed artifact names, not a published checksum or download claim:

```sh
set -eu
sha256sum --check gz-preview-m70-linux-x86_64.tar.gz.sha256
preview_parent="$HOME/gz-preview-m70-install"
mkdir "$preview_parent"
tar -xzf gz-preview-m70-linux-x86_64.tar.gz -C "$preview_parent"
preview_prefix="$preview_parent/gz-preview-m70-linux-x86_64"
"$preview_prefix/bin/gz-preview" verify
```

Use a fresh directory. Preserve all four ELF files, the Python launcher, guides,
41 copied examples, both M33 mesh copies, six viewer files and complete notices.
Missing, changed, linked, unlisted or wrong-mode files fail manifest verification.
Do not edit the manifest, substitute a worker or fetch missing payload from a
checkout. Relocation preserves the complete prefix; it does not rebuild anything.

## Default robot and camera mode are separate

The default robot remains the existing no-camera path. Set the same partition
in every terminal; the installed Zenoh endpoint remains `tcp/127.0.0.1:7449`.
No launcher changes variables in the calling shell.

```sh
export GZ_PARTITION=gz-preview-m70
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" run --http-port 8082 --ws-port 9014 --lifecycle
```

Open the complete viewer URL printed by this launch, including its query, for
the default-robot forward/turn and Play/Pause/Reset demonstration. Each launch
prints a fresh `/__gz_preview__/<launch-token>/` path; a previous launch's path
is refused. Use the new printed URL instead of a bookmark or a reconstructed
root URL. The six viewer files stay unchanged, and static responses use
`Cache-Control: no-store` rather than a cached 304 response.

Stop the owned launcher with Ctrl-C and wait for its processes and listeners
to retire before starting another world. Occupied ports fail; never evict an
unrelated listener. Existing HTTP choices remain
8080, 8081 and 8082.

Camera output requires the explicit boolean `--camera-output` on the M70
generation. The launcher derives only the manifest-verified
`bin/gz-render-worker`; this flag takes no worker-path argument. Without the
flag there is no camera worker. Do not combine it with `--lifecycle`: that
combination is refused before spawning. The launcher already supplies the
ordinary simulator web-viewer connection; there is no launcher `--web-viewer`
option to add.

```sh
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" run --example m69_two_depth_cameras.sdf --http-port 8082 --ws-port 9014 --camera-output
```

The launcher requires one admitted ordinary system lavapipe ICD whose
`ICD.library_path` is an allowed absolute LVP provider path or exactly
`libvulkan_lvp.so`, as used by ordinary Ubuntu packages. For that basename,
admission checks `/usr/lib/x86_64-linux-gnu` and `/lib/x86_64-linux-gnu` and
requires one canonical regular provider; usrmerge aliases count once. Missing,
ambiguous or escaping providers and every other relative path are refused.
The original ICD and controlled environment stay unchanged: the Vulkan loader
performs normal OS resolution, and actual mapped-provider/package ownership
must be proved separately. The launcher controls a private 0700 runtime directory.
Nonempty inherited graphics/device-selection and loader
overrides in the closed camera-admission policy are rejected before effects,
even if they appear to select the same provider. They are not silently discarded
or used as a fallback. Default robot environment behavior remains unchanged.
Hardware passthrough and arbitrary worker or SDK paths are outside this profile.
Read the actual runtime/dependency record before claiming an OS provider fits.

The private runtime directory is removed only if no child was spawned or owned
child retirement is confirmed. If retirement is uncertain, the launcher retains
the directory, reports its path and records UNKNOWN; it does not claim cleanup.
Inspect the retained owned process/listener evidence. Manual removal is safe
only after confirming ownership and actual retirement, never by evicting or
signalling an unrelated owner.

A separate qualified external consumer must use the same `GZ_PARTITION` and
endpoint. Its SDK is not installed in the producer. Camera A is 10 Hz on
`/m69/a/image` and `/m69/a/camera_info`, frame `rig_a/body/camera`; B is 20 Hz on
`/m69/b/image` and `/m69/b/camera_info`, frame `rig_b/body/camera`. Canonical
`gz.msgs.Image` R_FLOAT32 carries 4096 little-endian axial-depth floats, with
full typed CameraInfo calibration and acquisition stamps. Actual image demand
starts only the selected owner; Info-only demand does not start its worker.
Discovery, a receipt or an accepted control request does not prove a frame or
physical response.

After stopping that launcher and confirming owned retirement, the separate RGB
ellipsoid smoke uses:

```sh
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" run --example m67_rgb_ellipsoid.sdf --http-port 8082 --ws-port 9014 --camera-output
```

It uses `/m67/rgb/image`, `/m67/rgb/camera_info`, frame `rig/body/camera`, at
10 Hz. It is a canonical RGB smoke, not a new browser camera/depth panel or a
re-observation of every shape/output combination. Camera pause/reset proof uses
the existing ordinary web-control messages through the separately reviewed
proof consumer, not a new launcher service or a claimed framebuffer UI.

See [PREVIEW.md](PREVIEW.md) for 41 finite loads and
[CAPABILITIES.md](CAPABILITIES.md) for the exact proof and resource boundaries.
