<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M70 installed preview: current standalone installation guide

This maintained guide follows the standalone INSTALL.md published with the
[M70 release](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m70-preview).
It supersedes the checkpoint INSTALL.md inside the archive.
The archive and its manifest are unchanged audited bytes from Source
`7094a31890e78bc83c65f12a056dfb3918b7401a`. The accepted final Main
integration is `616cc3e2105a7ef9479ac823a6b05e832a92defc`; it is not the
Source used to build these binaries. M71–M75 features are not added to this package.
M70 release 452158796 is public and Latest with 41 installed examples. Root
retained actual Main/tag, seven-asset download, public/Latest and prior-release
readbacks, then recorded separate immutable acceptance. Published assets and
manifest-bound payload bytes stay frozen; this maintained Source guide does not
rewrite them. M76 development does not add its features to this package.

The supported target is Ubuntu 24.04 x86_64 with Python 3.11 or newer and the
ordinary OS runtime libraries listed in the package. The producer needs no
compiler, development checkout or Gazebo SDK. The optional camera worker needs
an admitted ordinary Ubuntu lavapipe ICD and provider, not a bundled graphics
stack or physical-GPU passthrough. See CAPABILITIES.md for measured scope.

## Extract the exact published bytes

Get the archive and checksum from the same actual M70 release. Verify before
extraction, and use a new directory without replacing an older preview:

```sh
set -eu
sha256sum --check gz-preview-m70-linux-x86_64.tar.gz.sha256
preview_parent="$HOME/gz-preview-m70-install"
mkdir "$preview_parent"
tar -xzf gz-preview-m70-linux-x86_64.tar.gz -C "$preview_parent"
preview_prefix="$preview_parent/gz-preview-m70-websocket-close-handshake-provisional"
"$preview_prefix/bin/gz-preview" verify
```

The public archive name is `gz-preview-m70-linux-x86_64.tar.gz`, but its actual
unchanged top-level directory is
`gz-preview-m70-websocket-close-handshake-provisional`. Do not rename files
inside the prefix or edit its manifest. The word provisional is part of that
retained directory name; it is not a claim about later publication status.

Keep all four ELFs, the launcher, 41 examples, two OBJ copies, six accepted
viewer files and notices. The manifest checks every payload file's bytes, size
and mode. Relocate the complete prefix together. This standalone guide is a
public overlay, not a replacement file inside the manifest-bound archive.

## Default robot in the browser

The ordinary default robot uses no camera-worker option:

```sh
export GZ_PARTITION="gz-preview-m70-$$"
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" run --http-port 8082 --ws-port 9014 --lifecycle
```

Open the complete URL printed by that owned launch, including its fresh token
path and query. Old token URLs are refused. Use Play, the finite drive helper,
Pause and Reset; stop the owned launcher with Ctrl-C and wait for retirement.
Never evict an unrelated listener. The accepted HTTP choices remain 8080–8082.
The retained demonstration used actual owner-approved access to 8082 and passed
forward/turn, stable Pause, Reset and owned launcher/PID/listener retirement.
Its browser reused the camera-provisioned 8 GiB container; it does not establish
a fresh 1 GiB live-browser fit. The simulator-grandchild return code is UNKNOWN.

## Paired metric-depth route

Camera output is an explicit M70 launcher option and cannot be combined with
`--lifecycle`:

```sh
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" run --example m69_two_depth_cameras.sdf --http-port 8082 --ws-port 9014 --camera-output
```

The launcher derives the verified packaged worker path. It supplies the existing
web-viewer connection; there is no launcher `--web-viewer` option to add.
Do not combine it with `--lifecycle`; this flag takes no worker-path argument.
Separate upstream consumers use the same `GZ_PARTITION` and endpoint
`tcp/127.0.0.1:7449`. Camera A publishes at 10 Hz on `/m69/a/image` and
`/m69/a/camera_info`; B publishes at 20 Hz on `/m69/b/image` and
`/m69/b/camera_info`. Info-only demand starts no worker. Actual image demand
selects only its owner. Canonical R_FLOAT32 contains 4,096 axial-depth values,
including positive infinity background, with complete CameraInfo and stamps.
The browser does not acquire or display a new depth panel.

The launcher checks one ordinary system lavapipe ICD/provider, rejects the
closed set of nonempty inherited loader/graphics overrides, and uses an owned
private runtime. Its ICD library path may be an admitted absolute provider or
exactly `libvulkan_lvp.so` with one canonical allowed system provider. Admission
is not proof of actual mapped libraries; the retained camera proof establishes
that separately. Preserve uncertain owned runtimes until actual retirement is
established, and do not signal unrelated processes.

## RGB ellipsoid: the observed native-only route

Do **not** launch the ellipsoid through the web route above: that exporter does
not admit Ellipsoid. The successful installed RGB proof used a paused native
server, with no HTTP/WebSocket/Scene exporter. Its retained exact argv was:

```text
/opt/gz-preview/bin/gz-sim-rs /opt/gz-preview/share/examples/m67_rgb_ellipsoid.sdf --server --camera-worker /opt/gz-preview/bin/gz-render-worker
```

After relocation, substituting only the prefix gives:

```sh
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m67_rgb_ellipsoid.sdf" --server --camera-worker "$preview_prefix/bin/gz-render-worker"
```

The observed run used an owned 0700 runtime as HOME and XDG_RUNTIME_DIR, the
ordinary `/usr/share/vulkan/icd.d/lvp_icd.json`,
`MESA_SHADER_CACHE_DISABLE=true`, a fresh GZ_PARTITION and the packaged
`share/zenoh.json5` as ZENOH_CONFIG. Those are retained environment facts, not a
claim that arbitrary host drivers or inherited overrides are qualified. Set up
the supported OS profile and matching environment before using this direct
route; it does not invoke the launcher's environment admission for you.

The native server begins paused. An external canonical subscriber and the
existing WorldControl/Clock service are needed to request the first 1 ms step;
this package does not ship the separate SDK test helper. The successful proof
kept SDK dispatch live using an external in-process pair interlock, validated
actual RGB_INT8/CameraInfo, then retired the worker and native server. It was
not a browser image proof, continuous video or an automatic user-facing smoke
command. The routes are `/m67/rgb/image` and `/m67/rgb/camera_info`, frame
`rig/body/camera`, at 10 Hz.

## Evidence and limits

The 7094 code, four-role build, package relocation, SDK-free clean install and
all 41 finite loads passed their retained audits. Paired-depth/GPU components,
separate native RGB and default-browser/visual proofs are distinct successful
records. The original combined camera controller operation remains failed;
there is no full-camera-operation success alias. Its lost depth host wait
ledger is explicitly Source-flow inference only. SDK callback quiescence and
discarded worker/simulator-grandchild exit codes remain UNKNOWN where not
observed. Finite loads alone are not image evidence.

No package, SDK, rendering or browser replay is required merely to publish these
unchanged bytes. Internal checkpoint guides remain historical. Read this
attached guide, CAPABILITIES.md and the qualification proof together with the
actual release readbacks; none substitutes for final release acceptance.

## Historical Source checkpoint

The following retained narrative describes the prequalification correction,
not the current release or an instruction to repeat those failed attempts.
Original failed trees, raw outputs and approval history remain preserved.

The parent's local package and clean-installation checks passed, while its
hosted run failed before product checks on Ubuntu snapshot HTTP 401 responses.
Its first installed camera trial failed after the one-millisecond world step
without retained `PACKET` receipts before the host barrier timeout. The host
stopped observation after 10 seconds, before the SDK's original 20-second
deadline; actual matching and callback delivery remain UNKNOWN. The previous
CLI isolated its camera peer under the package's listener-only,
multicast-disabled network configuration.
The correction uses the existing world session for CLI camera publications,
without changing the installed endpoint, viewer, worker or camera wire format.
No extra peer, endpoint inference or user configuration is required.

Earlier parent and frozen `fcd5a30dc0a8431b250cf96720db27ae597a6585` records,
including genuine failures, remain separate. Complete Source and independent
review before consolidated checks and fresh source-bound distribution/runtime
evidence. Reuse unchanged evidence only where its exact bindings permit it; this
changed package needs a fresh ordinary Ubuntu installation and GPU profile,
without changing the old package or provisioned container. Publication remains
held. The five-working-day budget includes review and mandatory gates; it is not
a release or resource-fit guarantee.

Live browser acceptance is also held for explicit approval to access
`http://127.0.0.1:8082` and its launcher-generated URLs after the retained automatic
approval rejections. The commands below document the intended installation and
use; they do not release a browser retry or an alternative surface.

The first correction normal stopped at an auxiliary registry mismatch: all 15
CLI camera cases passed, while the strict registry still expected 14. Clippy and
466 Python checks passed; full headless/renderer coverage was not reached. Its
failed tree and logs remain preserved. The bounded registry correction needs
independent review and one consolidated full retry before fresh package proof.

