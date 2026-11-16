<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M65 installed fleet preview

This is Source preparation, not an available M65 release. No M65 installed or
browser result is claimed. M60 remains Latest with 33 installed examples;
M1–M64 and 197 slices are now accepted at exact Main
`192ede6369ab6d83ecf88230944d2061f74b5e94`, M65's accepted parent. The repository
records this boundary in `docs/M61_M64_ACCEPTANCE.md`. The single affected
formatter passed; M65 tests, build, installation, browser and release evidence
remain unrun. A five-working-day
budget includes review and mandatory gates and is not a release guarantee.

## Install the actual qualified artifact when published

Use supported Ubuntu 24.04 x86_64, Python 3.11 or newer and ordinary OS libraries.
The actual release ELF/dependency evidence determines the required libraries.
The intended runtime needs no compiler, source checkout or Gazebo physics SDK;
that must be proved by a fresh supported clean installation, not assumed here.
Obtain the archive and matching checksum from the same actual release. Names
below are proposed artifact names, not a published checksum or download claim:

```sh
set -eu
sha256sum --check gz-preview-m65-linux-x86_64.tar.gz.sha256
preview_parent="$HOME/gz-preview-m65-install"
mkdir "$preview_parent"
tar -xzf gz-preview-m65-linux-x86_64.tar.gz -C "$preview_parent"
preview_prefix="$preview_parent/gz-preview-m65-linux-x86_64"
"$preview_prefix/bin/gz-preview" verify
```

Use a new directory and preserve the whole prefix. Missing, linked, unlisted,
changed or wrong-mode files must fail verification; do not edit the manifest to
hide them. Keep both copies of M33's OBJ under `share/meshes` and
`share/examples/meshes`, all notices, three guides and both viewer flavors.
Never repair the installation by fetching resources from another checkout.

## Start a fleet and address its actual topics

The launcher does not export variables into your shell. Set the SAME partition
in each terminal, and pass it explicitly to the launcher. Its installed Zenoh
endpoint is `tcp/127.0.0.1:7449`; occupied ports fail without eviction.

```sh
export GZ_PARTITION=gz-preview-fleet
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" run --example m61_eight_diff_drives.sdf --http-port 8082 --ws-port 9014 --lifecycle
```

Open the printed viewer URL, wait for actual Ready, and use Play before expecting
motion. In another terminal set `preview_prefix` and the same `GZ_PARTITION`, then
address a selected robot, or visit all eight sequentially:

```sh
export GZ_PARTITION=gz-preview-fleet
for owner in 0 1 2 3 4 5 6 7; do
  "$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" drive --topic "/m61/robot_${owner}/cmd_vel" 0.3 0 0.2
done
```

The drive helper retains its finite repeat and terminal zero/brake. A matching
subscriber or helper exit is not a motor/application acknowledgement. Odometry
is measured encoder output, not chassis world pose or commanded velocity × time.
Pause stops physics despite command receipt. Use Pause/Reset to observe the
actual loaded world; no command automatically unpauses it.

Stop that launcher with Ctrl-C and wait for its owned shutdown before starting
the controller world on the same ports. Do not kill an unrelated port owner:

```sh
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" run --example m62_eight_joint_controllers.sdf --http-port 8082 --ws-port 9014 --lifecycle
```

After Ready and Play, call the installed joint helper DIRECTLY with the matching
partition and endpoint. There is no `gz-preview joint` subcommand. These two
commands select each arm's position and velocity routes; the velocity topic has
no `/0` segment:

```sh
export GZ_PARTITION=gz-preview-fleet
for owner in 0 1 2 3 4 5 6 7; do
  "$preview_prefix/bin/gz-joint-command" tcp/127.0.0.1:7449 position 0.4 0.2 --topic "/model/arm_${owner}/joint/base_hinge/0/cmd_pos"
  "$preview_prefix/bin/gz-joint-command" tcp/127.0.0.1:7449 velocity 0.3 0.2 --topic "/model/arm_${owner}/joint/tip_hinge/cmd_vel"
done
```

Position retains its selected target; velocity and force send terminal zero.
The duration is finite, from 0 through 3600 seconds; zero sends one target.
Targets must be finite. Omitted `--topic` preserves all old mode defaults;
force is not an added M62 input. Selection does not enlarge the loaded controller
domain or create a command acknowledgement. Distinct owners still require actual
application/sample and cross-owner-isolation evidence for qualification.

Omitting `--example` still loads the complete M19 robot; the default partition
is still `gz-preview`. HTTP supports 8080/8081/8082, WebSocket defaults to 9014,
and lifecycle mode remains explicit and paired with the printed viewer URL.
The accepted viewer bytes are unchanged; browser graphics support is still needed.
The installed simulator is native-free and has no RGB/depth-worker runtime.
See [PREVIEW.md](PREVIEW.md) for 35 finite loads and
[CAPABILITIES.md](CAPABILITIES.md) for the evidence boundary.
