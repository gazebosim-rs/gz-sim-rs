<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M60 Ubuntu preview: installation and current examples

This is provisional Source documentation for a future Ubuntu 24.04 x86_64
preview. No M60 installed or browser result is claimed. M40 remains Latest;
M1–M42 and 175 accepted slices are unchanged. M60 integration waits for genuine
M59 full normal and independent retained DATA. The requested closeout date is a
target, not permission to bypass Source review, checks, installation evidence or
hosted/official/Main release gates. Do not treat the names below as a published
archive or an invented checksum.

## After the exact artifact is qualified and made available

Obtain `gz-preview-m60-linux-x86_64.tar.gz`, its matching `.sha256` and provenance
from the same actual release. On supported Ubuntu, Python 3.11 or newer and
ordinary system runtime libraries are required. Consult that artifact's actual
ELF/package evidence for exact runtime dependencies. No SDK/compiler/source
checkout is intended at installed runtime; this remains an acceptance obligation,
not a fresh dependency-closure result. Do not downgrade security updates.

Choose a new directory and keep the complete prefix together:

```sh
set -eu
sha256sum --check gz-preview-m60-linux-x86_64.tar.gz.sha256
preview_parent="$HOME/gz-preview-m60-install"
mkdir "$preview_parent"
tar -xzf gz-preview-m60-linux-x86_64.tar.gz -C "$preview_parent"
preview_prefix="$preview_parent/gz-preview-m60-linux-x86_64"
"$preview_prefix/bin/gz-preview" verify
```

Never extract over an existing installation or edit the manifest to hide a
mismatch. Verification requires the listed ordinary files, exact hashes/modes,
notices and both viewer flavors; it refuses missing, linked, changed or unlisted
files. The archive checksum and manifest do not supply missing OS libraries.
Preserve `bin/`, `share/`, `web/`, `INSTALL.md`, `CAPABILITIES.md`, `PREVIEW.md`,
`LICENSE`, `NOTICE`, `notices.json`, `notices/` and `manifest.json`.
M33's relative OBJ requires `share/examples/meshes/m16-asymmetric.obj`; the default
M19 robot separately uses `share/meshes/m16-asymmetric.obj`. Both copies must stay
byte-identical and ordinary. No checkout lookup or download repairs either one.

## Default robot and explicit copied example

The existing command still starts the paused complete M19 robot when `--example`
is absent. Existing ports, lifecycle opt-in and default command topic are intact:

```sh
"$preview_prefix/bin/gz-preview" run --http-port 8082
```

To select the copied two-robot world instead, use one manifest-bound basename:

```sh
"$preview_prefix/bin/gz-preview" run --example m59_multiple_diff_drives.sdf --http-port 8082
```

The selector admits only a direct listed `share/examples/BASENAME.sdf`. It does
not accept an arbitrary path, directory, remote resource or outside checkout.
Open the exact URL printed by the launcher; wait for its actual Ready state and
use the existing pause/run controls. The headless server needs no GPU; browser
operation still requires a supported WebGL2 or WebGPU implementation. Both viewer
bundles are retained, not newly rebuilt or universally qualified.

Use the same partition in every terminal (`gz-preview` by default). For the
selected M59 world, after unpausing, these finite commands independently address
the left and right robots using canonical Twist:

```sh
"$preview_prefix/bin/gz-preview" drive --topic /m59/left/cmd_vel 0.4 0 1 &
left_sender=$!
"$preview_prefix/bin/gz-preview" drive --topic /m59/right/cmd_vel -0.25 0.5 1 &
right_sender=$!
wait "$left_sender"
wait "$right_sender"
```

Each helper publishes for the finite duration then sends the existing zero/brake
completion. An actual match is discovery, not an application/motor acknowledgment.
While paused, receipt does not advance physics. There is no automatic unpause or
invented topic argument for old installations. Without `--topic`, the helper
retains `/model/robot/cmd_vel`; use that default with the default M19 robot.
The installed `gz-joint-command` and its zero-coasts force behavior remain intact.
Use Ctrl-C in the launcher terminal for its owned shutdown; never kill an
unrelated port owner. HTTP accepts 8080, 8081 or 8082, WebSocket defaults to 9014,
and Zenoh TCP7449 remains fixed. Occupied ports must fail without eviction.

## Finite examples and evidence boundary

[PREVIEW.md](PREVIEW.md) lists the exact33 direct load/step commands. Each uses the
ordinary installed simulator, Rapier and two one-millisecond steps. Finite loads
are separate from sensor noise, canonical wire, physical response and demand
proofs. [CAPABILITIES.md](CAPABILITIES.md) distinguishes inherited feature evidence
from fresh M60 package/installation/browser evidence, all of which remains held
until it actually completes. No native Gazebo System replay is promised.
