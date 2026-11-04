<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M8: one bounded differential-drive robot

M8 is accepted and published at simulator Main
`fc9673881bae5b0c28e8e22fe2ab013d3f87f117`. Required local verification,
[exact hosted CI41776261368](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41776261368),
and independent source, local/official data and actual browser reviews passed.
See [M8_ACCEPTANCE.md](M8_ACCEPTANCE.md) for evidence and limitations. M9 is the
current weekly delivery; accepted M1–M7 manual qualification was not replayed.

The five-working-day delivery is one two-wheel Rapier robot, real canonical
velocity commands, measured encoder odometry and a live gzweb-rs view. It must
prove driving and turning, wheel target/actuation bounds, odometry frames,
pause/reset and absence/last-system teardown. Ackermann, tracks, general drivetrain
parity, additional sensors and unrelated geometry/backend work remain excluded.

## Reused architecture and explicit admission

Use the exact built-in model configuration `gz::sim::systems::DiffDrive` with
filename `gz-sim-diff-drive-system`, never a dynamic plugin loader. Admit one
unambiguous distinct direct-model left/right revolute wheel pair and finite
representable geometry. Radius and separation are explicit controller/odometry
parameters, not silently inferred from collision shapes. Native defaults are
0.2m and1m. The initial domain is one top-level model without a namespace;
custom topics are valid absolute names. Unknown or unsupported fields/layouts
must fail explicitly. Existing ApplyJointForce admission and generic unsupported
model-plugin negatives remain preserved.

Existing effort commands cannot drive a moving chassis: the Rapier effort API
explicitly refuses moving-parent joints. A small reviewed Source dependency adds
maintained native velocity motors instead of a custom solver or body teleport.
The backend API takes finite target radians/s and a finite nonnegative torque
cap in Nm, validates before mutation, supports zero-target braking and explicit
disable, and clears runtime motor state on both reset paths/removal. Root's
configured wheel target bounds are not hard physical speed caps against external
impulses. Other backends explicitly refuse the new motor capability.

## Commands, measured state and canonical output

The canonical command topic is `/model/{model}/cmd_vel`; output is
`/model/{model}/odometry`. Defaults for header metadata are
`frame_id={model}/odom` and `child_frame_id={model}/{canonical_link}`. Do not invent
an epoch field in canonical Twist/Odometry; epochs fence internal queues.

For +X forward/+Z yaw, left wheel at +Y and both wheel axes +Y:

`left=(v-omega*separation/2)/radius`,
`right=(v+omega*separation/2)/radius`.

Consume finite planar `Twist.linear.x` and `angular.z`. The admitted Rust boundary
rejects malformed/nonfinite commands and unsupported nonzero axes without
replacing the last valid command; this is stricter admission than the inspected
native implementation. Hold the last valid command until replacement/reset.
Velocity limit support must remain distinct from acceleration/jerk fields, which
are refused unless actually implemented. Actuator torque caps come from the
admitted wheel configuration, not an invented native DiffDrive XML max_force.

Odometry integrates actual continuous wheel coordinates, never commanded-v*dt
or a body ground-truth pose. Use local XY/yaw encoder integration and the admitted
ten-sample measured velocity window, with simulation-time stamps and a default
50Hz publication cadence. Physics remains authoritative for the actual body's
world motion/contact. Encoder odometry is not a promise of zero wheel slip.

The configured driver/encoder lifecycle does not follow odometry subscriber
matching. Command input stays active for a configured driver; publication may
follow real matching. Reuse the existing bounded world IO owner, command budget,
weak output leases and lazy declarations. No configured driver means no drive
callbacks, owned history, endpoints or per-tick polling. No extra IO thread or
transport facade is introduced.

Paused ticks do not actuate, integrate or publish new odometry; receipt can update
pending command state. Reset must zero command/motors/odom/history, restore joint
coordinates and replace input generation so old queued commands cannot return.
The inspected native system has no reset interface and warns about rewind:
these fresh-epoch rules are an explicit Rust lifecycle contract, not native reset
parity. Native command calculation/application is pipelined; this delivery must
document its actual boundary instead of claiming complete native tick ordering.
TF Pose_V, enable controls and namespace/general drivetrain semantics are deferred.

## Source-informed contract, not a new native parity campaign

Read-only assessment inspected local simulator revision `71c60db3` at
`src/systems/diff_drive/DiffDrive.cc` and math revision `1316ed33` at
`src/DiffDriveOdometry.cc`. Harmonic `9d9cd030` agrees on the core wheel formulas
but differs on custom-topic namespace handling; admission remains explicit.
The maintained Rapier0.36.1 motor and solver source is the backend authority.
No C++ DiffDrive executable or new parity/performance evidence was captured.
Retained C++ interoperability controls remain historical compatibility evidence.

## Run the robot and send commands

Build the native-free simulator and the finite command sender:

```sh
cargo build --locked -p gz-sim-cli --no-default-features
cargo build --locked -p gz-sim-server --no-default-features --example drive_command
```

Use an explicit upstream Zenoh listener so the sender has a known endpoint.
Create `/tmp/m8-zenoh.json5` with:

```json
{
  "listen": {"endpoints": ["tcp/127.0.0.1:7448"]},
  "scouting": {"multicast": {"enabled": false}}
}
```

Start the simulator from this repository root. It starts paused; use the viewer's
Play control before sending commands.

```sh
GZ_PARTITION=m8 ZENOH_CONFIG=/tmp/m8-zenoh.json5 target/debug/gz-sim-rs examples/m8_diff_drive.sdf --server --web-viewer 9008
```

Serve the maintained `gzweb-rs` app on loopback HTTP port 8080 or 8081 as described
in [MVP_BROWSER.md](MVP_BROWSER.md). Select its existing primitive backend and
connect it to `ws://127.0.0.1:9008/`, for example:
`http://127.0.0.1:8081/?backend=primitive&ui=full&url=ws://127.0.0.1:9008/`.
The scene contains the ground, one box chassis and two sphere wheels. The
simulator requires no native physics SDK, window or GPU; the browser requires a
supported graphics backend.

In another terminal, send forward and turning commands using the same partition:

```sh
GZ_PARTITION=m8 target/debug/examples/drive_command tcp/127.0.0.1:7448 0.4 0 3
GZ_PARTITION=m8 target/debug/examples/drive_command tcp/127.0.0.1:7448 0 1 3
```

Arguments are endpoint, forward metres/second, yaw radians/second and duration
seconds. Each successful invocation ends with a zero command for bounded motor braking;
this is not an emergency stop or an instantaneous physical velocity clamp.
Commands apply at the next advancing step. The viewer consumes existing live
world poses; wheel actuation does not replace body poses or velocities.
The documented viewer reset requests `pause: true`, restores the loaded pose
and clears pending drive state in a new epoch. It leaves the world paused;
resume is a separate action.

The command sender waits up to five seconds for a matching command subscriber
before starting its finite send interval. Matching confirms declaration discovery;
it is not an actuation acknowledgment. Encoder integration starts at the actual
loaded or restored wheel coordinates at simulation time zero, including the first
advancing physics tick. The demo retains the existing centered single-box chassis
and sphere-wheel admission; compound Rapier links and offset centers of mass are
not introduced by M8.
