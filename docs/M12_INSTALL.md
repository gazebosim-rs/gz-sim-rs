<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M12 Linux preview: install and run

These instructions apply to the M12 Ubuntu 24.04 x86_64 preview. Obtain the
archive and checksum from the same published release. See the matching release's
provenance and [capability matrix](M12_CAPABILITIES.md) for its profile and limits.

## Supported setup

The supported target is **Ubuntu 24.04 x86_64**. Installed execution requires
Python 3.11 or newer and the usual Ubuntu system libraries. The clean installation
used Ubuntu 24.04.5 and Python 3.12.3, with these actual native dependencies:

| Library | Observed Ubuntu package | Observed installed version |
| --- | --- | --- |
| `libgcc_s.so.1` | `libgcc-s1:amd64` | `14.2.0-4ubuntu2~24.04.1` |
| `libm.so.6`, `libc.so.6`, x86-64 loader | `libc6:amd64` | `2.39-0ubuntu8.9` |

These are observations from the supported installation, not permission to remove
other OS packages or a requirement to downgrade Ubuntu security updates. On a
minimal supported Ubuntu installation, install the named runtime prerequisites
using its ordinary signed repositories:

```sh
sudo apt-get update
sudo apt-get install --no-install-recommends python3 libc6 libgcc-s1
```

The package is dynamically linked, not fully static. Neither native executable
requires a dynamic OpenSSL, Gazebo, DART, MuJoCo or renderer SDK library. Running
the package requires no source checkout, Rust/Cargo, compiler, native physics
SDK, camera worker or external asset service. The headless server needs no GPU
or display. Viewing requires a graphics-capable browser; the actual installed
robot demo used WebGL2. The retained WebGPU bundle is also supplied, without a
claim of support for every browser or driver.

## Download, verify and extract

From the matching published release assets, obtain:

- `gz-preview-m12-linux-x86_64.tar.gz`
- `gz-preview-m12-linux-x86_64.tar.gz.sha256`

Keep both downloaded files in the same directory and verify the external
checksum before extraction. The archive contains the single prefix
`gz-preview-m12-linux-x86_64/`. Choose a new parent directory; never extract over
an existing installation.

```sh
set -eu
sha256sum --check gz-preview-m12-linux-x86_64.tar.gz.sha256
install_parent="$HOME/gz-preview-m12-install"
mkdir "$install_parent"
tar -xzf gz-preview-m12-linux-x86_64.tar.gz -C "$install_parent"
preview_prefix="$install_parent/gz-preview-m12-linux-x86_64"
"$preview_prefix/bin/gz-preview" verify
```

`verify` checks manifest paths, file bytes and modes without launching the
simulator. Missing, changed, unlisted or linked payload files are refused. Do not
edit or regenerate the manifest to conceal a mismatch. Keep the whole prefix
together: binaries, `share/`, `web/`, both guides, `LICENSE`, `NOTICE`,
`notices.json`, `notices/` and `manifest.json`. Distributed files are ordinary
files, not development RAM symlinks. Source, feature and notice provenance are
retained with the matching release; a manifest does not remove OS dependencies.

## Start the live robot

In the first terminal:

```sh
preview_prefix="$HOME/gz-preview-m12-install/gz-preview-m12-linux-x86_64"
"$preview_prefix/bin/gz-preview" run
```

The robot starts **paused**. Open the printed URL in your browser, then click
Play. Defaults are HTTP `127.0.0.1:8081`, WebSocket `127.0.0.1:9014` and Zenoh
`tcp/127.0.0.1:7449`. The URL selects `backend=primitive&ui=full` and the matching
WebSocket. The launcher serves only static files under `web/`, with no CGI or
separate Gazebo service.

HTTP accepts **8080 or 8081 only**. For example:

```sh
"$preview_prefix/bin/gz-preview" run --http-port 8080 --ws-port 9014
```

An occupied port fails without terminating its owner. Resolve the conflict
yourself or choose the other supported HTTP port. Zenoh 7449 is fixed; changing
the partition does not allow two launches to share it. A WebSocket port must be
1024..=65535 and distinct from HTTP and Zenoh. These are loopback preview
endpoints, not a supported remote or multi-instance deployment.

## Drive and turn

While the world is playing, use a second terminal:

```sh
preview_prefix="$HOME/gz-preview-m12-install/gz-preview-m12-linux-x86_64"
"$preview_prefix/bin/gz-preview" drive 0.4 0 3
"$preview_prefix/bin/gz-preview" drive 0 0.8 3
```

Arguments are linear speed in m/s, yaw rate in rad/s and duration in wall-clock
seconds. Values must be finite; duration is 0..=3600 seconds. The finite sender
waits at most five seconds for a matching subscriber and uses canonical Twist
messages. A successful command ends with zero targets; braking is torque-limited.
Failed sending does not promise delivery of the final brake packet. Use Pause
and investigate any error before continuing.

The default partition is `gz-preview`. A custom nonempty partition without
whitespace must match in both terminals, with the global option before the
subcommand:

```sh
"$preview_prefix/bin/gz-preview" --partition demo-local run
"$preview_prefix/bin/gz-preview" --partition demo-local drive 0.4 0 3
```

The model is `robot`; its command topic is `/model/robot/cmd_vel`. The launcher
passes the packaged configuration and partition to its simulator. The sender
uses an explicit outgoing-only Zenoh endpoint, with no listeners, multicast or
gossip. See [the capability matrix](M12_CAPABILITIES.md) for frames and limits.

## Pause, reset, stop and relocate

Pause and Reset are the maintained viewer controls. The observed installed demo
drove and turned, acknowledged Pause, and acknowledged Reset to epoch 1/time
zero and the initial pose. These observations establish integrated behavior,
not exact odometry, trajectory, browser startup time or performance.

Let the finite sender finish, then press Ctrl-C in the `run` terminal. This stops
the launcher-owned simulator group and HTTP server, not unrelated processes.
Closing the browser alone does not stop the server.

To relocate, first stop the launcher, move the complete prefix unchanged, then
run `bin/gz-preview verify` at the new location before starting it. Paths are
derived from the installed prefix. A changed or incomplete prefix must pass
`verify` before it is started.
