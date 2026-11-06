<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Integrated preview: install and run

**M12 is pending; no release archive or download URL is published by this guide.**
The commands below describe the agreed relocatable prefix and authored launcher.
Package assembly, actual binary dependencies, required gates/CI and clean-machine
installation/browser acceptance must still complete. They are not passing
results. See the repository package contract at `docs/M12_PREVIEW.md` and the
[profile/limitations matrix](M12_CAPABILITIES.md), also copied into the prefix.

## Supported setup

The initial target is Ubuntu 24.04 x86_64. Installed execution needs Python 3
with `hashlib.file_digest` (Python 3.11 or newer), the selected ELF files' actual
system-library closure, and a browser with working WebGL2 or WebGPU. The
headless native-free server itself needs no GPU or display. A user can open the
viewer in a local graphics-capable browser.

The package must not require a source checkout, Rust/Cargo, a native physics SDK,
a renderer worker or an external asset service to run the fixed robot demo.
This is not a fully static distribution. Exact M12 runtime libraries and any
Ubuntu package prerequisites are **pending actual simulator and sender builds**;
no guessed `apt` command is supplied.

For reference only, the accepted `101f1ae8` native-free CLI closure records
`libgcc_s.so.1`, `libm.so.6`, `libc.so.6` and the
`ld-linux-x86-64.so.2` loader. This describes that retained CLI, not the future
M12 simulator or drive-helper closure. Both selected M12 binaries must have their
actual libraries and platform requirements recorded before an installation pass
is claimed. Do not infer runtime libraries merely from Cargo package names.

## Produce or obtain a prefix

A release archive's exact filename, download location and digest remain pending.
Use only the actual source-bound artifact and notices supplied with a qualified
release. Choose a new empty parent directory; do not extract over an existing
installation. Every archive member is rooted under `PREFIX_BASENAME/`, where
`PREFIX_BASENAME` is the assembler's `NEW_PREFIX` directory basename, not
necessarily the archive filename. Extraction therefore yields
`NEW_PARENT/PREFIX_BASENAME`; no extra strip or nesting step is needed.

After an actual qualified archive is available, replace these placeholders with
its path, a new parent directory and its recorded prefix basename:

```sh
mkdir NEW_PARENT
tar -xzf ARCHIVE.tar.gz -C NEW_PARENT
preview_prefix="NEW_PARENT/PREFIX_BASENAME"
"$preview_prefix/bin/gz-preview" verify
"$preview_prefix/bin/gz-preview" run
```

These are installation instructions for that future artifact, not commands that
have been executed or a claim that an archive already exists.

The agreed source-side assembler interface is:

```sh
python3 scripts/package-preview.py --spec package-inputs.json --output NEW_PREFIX
```

An optional new archive can be requested with `--archive NEW_ARCHIVE.tar.gz`.
`NEW_PREFIX`, `NEW_ARCHIVE.tar.gz` and `package-inputs.json` are operator-selected
paths, not artifacts that already exist. The assembler's `--help` describes the flags; the package contract supplies the
full input schema/example. Its source requires `schema_version: 1`, target
`ubuntu-24.04-x86_64` and exact 40-hex simulator/viewer revisions. Do not substitute
placeholder paths, digests or unselected Cargo graphs for actual build records.

The explicit spec binds repository identities, simulator and finite sender paths
and expected hashes to their Cargo compiler-artifact transcripts, all six
accepted viewer files, selected Cargo metadata and explicit extra notices.
Both outputs must be new, with existing real parent directories and simple
basenames made from letters, digits, underscore, hyphen or dot, starting with a
letter or digit and at most 128 characters. The optional archive must be outside
the prefix. Existing output, linked inputs and unsupported file types are
refused; distributed output contains ordinary files and directories, not
symlinks. The intended native profile is `--no-default-features`, without
`rgb-camera`. Build commands, selected artifacts
and release/assembly evidence remain pending required source review and gates;
this guide does not authorize a standalone build or an unreviewed alternative.

The intended installed layout is:

```text
PREFIX/
  manifest.json
  M12_INSTALL.md
  M12_CAPABILITIES.md
  LICENSE
  NOTICE
  notices.json
  bin/gz-preview
  bin/gz-sim-rs
  bin/gz-drive-command
  share/robot.sdf
  share/zenoh.json5
  web/index.html
  web/startup.mjs
  web/target/wasm32-unknown-unknown/release/webgpu/gzweb.js
  web/target/wasm32-unknown-unknown/release/webgpu/gzweb_bg.wasm
  web/target/wasm32-unknown-unknown/release/webgl2/gzweb.js
  web/target/wasm32-unknown-unknown/release/webgl2/gzweb_bg.wasm
  notices/
```

Keep the complete directory together, including relative viewer imports and
notices. Distributed payloads are ordinary files, not development RAM symlinks.
The manifest records source/profile/license provenance and payload hashes,
sizes and modes. It does not make the package independent of system libraries.

## Verify and start

After a qualified prefix is available, set its actual location, for example:

```sh
preview_prefix="$HOME/gz-preview"
"$preview_prefix/bin/gz-preview" verify
"$preview_prefix/bin/gz-preview" run
```

`verify` checks installed manifest paths and recorded file bytes/modes without
launching the simulator. Missing, changed or linked payload files are refused.
It is not a physics, graphics or clean-install acceptance test. Do not hand-edit
or regenerate a manifest to hide a mismatch.

`run` starts the packaged robot **paused** and prints the actual viewer URL.
Defaults are HTTP `127.0.0.1:8081`, WebSocket `127.0.0.1:9014` and Zenoh
`tcp/127.0.0.1:7449`. The printed URL explicitly selects
`backend=primitive&ui=full` with that WebSocket address. Open the printed URL in
the browser; no separate local Gazebo service is required. HTTP serves only
static files under `web/`; CGI execution is not enabled. It does not serve the
prefix's binaries or notice files.

The HTTP choice is limited to 8080 or 8081. For example:

```sh
"$preview_prefix/bin/gz-preview" run --http-port 8080 --ws-port 9014
```

The launcher refuses occupied ports without killing their owners. Port 7449 is
fixed for the supplied Zenoh configuration and drive helper; a different
partition does not permit two launches to share that listener. The WebSocket
port must be in 1024..=65535 and distinct from HTTP and Zenoh. Do not expose or
forward these local endpoints as a supported remote deployment.

## Send real robot commands

Click Play so simulation time advances. In a second terminal use the same prefix:

```sh
preview_prefix="$HOME/gz-preview"
"$preview_prefix/bin/gz-preview" drive 0.4 0 3
"$preview_prefix/bin/gz-preview" drive 0 0.8 3
```

Arguments are linear speed in m/s, yaw rate in rad/s and duration in wall-clock
seconds. Values must be finite and duration must be in 0..=3600 seconds. The
helper waits at most five seconds for a matching command subscriber and sends
through the existing canonical Twist path. A successful finite command finishes
with zero targets; physical braking remains torque-limited. Failed sending is a
failure, not a promise that a final brake packet was delivered. Use viewer Pause
and investigate the reported error before continuing.

The default partition is `gz-preview`. To choose another nonempty partition
without whitespace, put the global option **before** the subcommand in both
terminals:

```sh
"$preview_prefix/bin/gz-preview" --partition demo-local run
"$preview_prefix/bin/gz-preview" --partition demo-local drive 0.4 0 3
```

The launcher sets the packaged Zenoh configuration and partition for its child
and helper. The unchanged demo model is `robot`, with command topic
`/model/robot/cmd_vel`. Its configured bounds and frames are documented in the
[capability matrix](M12_CAPABILITIES.md). The wrapper does not add teleoperation,
world editing or recording/replay subcommands.

## Pause, reset, stop and relocate

Use viewer Pause/Reset for the maintained robot state controls. Ctrl-C in the
`run` terminal stops this launch's simulator process group and HTTP server. It
does not terminate an unrelated process that owns a conflicting port. The
finite drive command should finish before intentional shutdown. Closing the
browser alone does not stop the server.

To relocate the package, first stop its launcher, move the entire prefix without
changing its contents, then invoke `bin/gz-preview verify` and `run` at the new
path. The launcher derives file locations from its installed prefix and supplies
the packaged `ZENOH_CONFIG` to the simulator and the matching `GZ_PARTITION` to
both processes. The finite helper uses its explicit connection endpoint and
outgoing-only upstream Zenoh configuration, with no listeners, multicast or
gossip; it does not read `ZENOH_CONFIG`. The drive wrapper replaces itself with
that helper so a wrapper-only signal cannot leave a separate helper running.
This authored behavior still needs actual clean-install/relocation evidence;
no relocation qualification has run as part of writing this guide.

## Qualification still required

Before calling M12 accepted, finish the complete source/regression/docs batch
and independent review; pass normal required gates and exact hosted CI; build
and bind the actual native-free binaries, viewer files, notices and manifest;
then install outside the development checkout on supported Linux. Prove actual
forward/turn motion, browser Pause/Reset and clean shutdown with the packaged
files and recorded actual dependencies. Keep all previous acceptance evidence,
failures, assertions, coverage policies and independent Fuel work. There is no
claim of a published package, new metrics or clean-machine pass yet.
