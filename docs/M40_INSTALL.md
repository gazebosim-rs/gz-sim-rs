<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Updated Linux preview: install and run

These commands apply to the M40 Ubuntu 24.04 x86_64 preview. Obtain the archive,
external checksum and provenance from the same published release. Its acceptance
receipt records the actual builds, installed system libraries and browser proof.
See [CAPABILITIES.md](CAPABILITIES.md) for the profile and limits, and
[PREVIEW.md](PREVIEW.md) for fifteen copied sensor, publisher and wrench examples and finite commands.

## Prerequisites

Use Ubuntu 24.04 x86_64 with Python 3.11 or newer. On a minimal installation,
install ordinary runtime prerequisites from its signed Ubuntu repositories:

```sh
sudo apt-get update
sudo apt-get install --no-install-recommends python3 libc6 libgcc-s1
```

The executables are dynamically linked. Consult the matching release's actual
ELF and installation evidence for resolved libraries and package versions;
do not downgrade security updates to reproduce an observed version. Execution
requires no source checkout, Rust/Cargo, compiler, Gazebo/DART/MuJoCo SDK or
camera worker. The server needs no GPU or display. The viewer needs a browser
with a working WebGL2 or WebGPU implementation; supplying both bundles does not
qualify every driver or browser.

## Verify and extract

Download `gz-preview-m40-linux-x86_64.tar.gz` and its matching `.sha256` file.
Keep them together, verify before extracting, and choose a new parent directory.
Do not extract over an existing installation.

```sh
set -eu
sha256sum --check gz-preview-m40-linux-x86_64.tar.gz.sha256
preview_parent="$HOME/gz-preview-m40-install"
mkdir "$preview_parent"
tar -xzf gz-preview-m40-linux-x86_64.tar.gz -C "$preview_parent"
preview_prefix="$preview_parent/gz-preview-m40-linux-x86_64"
"$preview_prefix/bin/gz-preview" verify
```

Verification checks the complete ordinary-file inventory, hashes and modes;
linked, missing, changed or unlisted files are refused. Keep binaries, `share/`
(including both `share/meshes` and `share/examples/meshes`), `web/`, all three guides (`INSTALL.md`, `CAPABILITIES.md`, `PREVIEW.md`), `LICENSE`, `NOTICE`, `notices.json`,
`notices/` and `manifest.json` together. Do not edit the manifest to conceal a
mismatch. The external checksum checks the downloaded archive; the manifest
checks the extracted payload and does not remove OS dependencies.
The M33 example's unchanged relative OBJ URI needs
`share/examples/meshes/m16-asymmetric.obj`; the default robot separately uses
`share/meshes/m16-asymmetric.obj`. Both are ordinary byte-identical copies.
Keep both locations even though their bytes match; no checkout lookup or asset
download repairs a missing installed file.

## Finite installed examples

The fifteen self-contained SDF examples are ordinary copied files under
`share/examples/`. Run the installed verifier first, then use the fifteen explicit
`bin/gz-sim-rs` commands in [PREVIEW.md](PREVIEW.md). Each uses Rapier and
`--iterations 2 --step-ns 1000000`, loads world `default`, and exits after two
simulation steps. No source checkout, Cargo, SDK, browser, model download or new
launcher scene-selection option is needed. Do not run these commands through
`gz-preview run`: that command intentionally keeps the complete default robot.
Loading and stepping alone does not measure noisy output or canonical delivery;
at 100 Hz the PosePublisher first positive-time frame is at 10 ms, beyond these
two steps. The M31 static channel at 25 Hz first publishes at 40 ms. Separate
M34/M36/M38/M39 ground-truth odometry at 50 Hz first publishes at 20 ms. These finite
runs do not send wrench commands, verify motion feedback or demonstrate sensor
demand. Separate model and wire evidence qualifies acquisition. The examples
add no browser telemetry widgets.

## Run and drive

In the first terminal:

```sh
preview_prefix="$HOME/gz-preview-m40-install/gz-preview-m40-linux-x86_64"
"$preview_prefix/bin/gz-preview" run --http-port 8082
```

The world starts paused. Open the printed URL and click Play before sending
commands. This command selects loopback HTTP 8082; defaults remain HTTP 8081,
WebSocket 9014 and Zenoh TCP 7449.
The launcher serves only `web/` and selects the maintained primitive backend/UI.
HTTP admits only 8080, 8081 or 8082; `--http-port 8080 --ws-port 9014` selects
another supported HTTP port. WebSocket ports must be 1024..=65535 and distinct from HTTP
and Zenoh. Occupied ports fail without touching their owners. Zenoh 7449 is fixed;
a different partition does not permit two launches to share it. This is a local
preview, not a remote or multi-instance service.

### Optional lifecycle and publication credit

Ordinary `run` keeps the default publication behavior. To opt in, stop that
launcher and restart it explicitly:

```sh
"$preview_prefix/bin/gz-preview" run --http-port 8082 --lifecycle
```

Use the URL printed by this invocation. The launcher supplies both the native
`--web-lifecycle` option and the viewer's outer `lifecycle=1` query. Adding the
query to an ordinary run is not a substitute for this choice. The existing
viewer then uses its acknowledged publication-credit contract; this option
also exposes the existing local lifecycle controls in the full UI.

Those controls can spawn only catalog-offered local templates and can remove,
read or set state only where the existing model contract admits the action.
They do not accept arbitrary XML, filesystem paths or remote models. The
articulated robot remains visible and drivable, but its catalog reports
`can_remove=false` and `can_set_state=false`: jointed/sensor-bearing robot
editing is unsupported. Keep its joints, sensors and built-in systems intact.
Do not interpret a disabled action as permission to alter the installed world.

The `--lifecycle` option belongs only to `run`; `drive` and `verify` are unchanged.
Use actual Play/Pause acknowledgements before acting, and the maintained Retry
control after a connection failure. This explicit mode does not promise to fix
an unknown browser stall or provide higher throughput. Ports, ownership refusal,
queue bounds and cleanup do not change.

While the world is playing, in a second terminal:

```sh
preview_prefix="$HOME/gz-preview-m40-install/gz-preview-m40-linux-x86_64"
"$preview_prefix/bin/gz-preview" drive 0.4 0 3
"$preview_prefix/bin/gz-preview" drive 0 0.8 3
```

Arguments are linear speed in m/s, yaw rate in rad/s and wall-clock duration in
seconds. Inputs must be finite and duration must be 0..=3600 seconds. The finite
sender waits at most five seconds for a matching subscriber and uses canonical
Twist messages. A successful send ends with zero targets; braking is bounded by
motor torque, not instantaneous rest. A failed send cannot guarantee delivery of
its final brake packet. Pause and investigate errors before continuing.

The default partition is `gz-preview`. A custom nonempty partition without
whitespace must match in both terminals, before the subcommand:

```sh
"$preview_prefix/bin/gz-preview" --partition demo-local run --http-port 8082
"$preview_prefix/bin/gz-preview" --partition demo-local drive 0.4 0 3
```

The model command is `/model/robot/cmd_vel`. The sender connects outward to the
explicit local endpoint, without listeners, multicast or gossip. The installed
world includes ideal altimeter, pressure and magnetic sensors, a joint-state
publisher and local OBJ visuals/collision. These canonical publications do not
add sensor widgets to the browser; see the capability guide for their contracts.

## Pause, reset, stop and relocate

Use the maintained browser's Pause and Reset controls. Reset returns to the
initial scene and clock; the matching acceptance receipt records the actual
observed acknowledgements and poses, without promising cross-engine trajectories.
Let a finite sender finish, then press Ctrl-C in the `run` terminal. The launcher
stops its owned simulator group and HTTP server. Closing the browser does not
stop the simulation.

To relocate, stop the launcher, move the entire unchanged prefix, and run
`bin/gz-preview verify` at its new location before restarting. All runtime paths
are derived from the installation; its copied local mesh needs no asset server.
