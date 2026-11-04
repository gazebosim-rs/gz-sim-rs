<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M8: one bounded differential-drive robot

Implementation is in progress in an isolated tree. No M8 compilation, tests,
lint, native captures or runtime observations have run. The complete feature,
regression source, documentation and independent source review must precede one
consolidated required pass. M1–M7 remain accepted; no earlier manual qualification
is replayed. This is not an acceptance record.

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

## Command sender

The bounded `gz-sim-server` example `drive_command` connects directly to an
upstream Zenoh endpoint and publishes canonical Twist commands, followed by a
zero command. Set the same `GZ_PARTITION` as the simulator. For example, after
starting the M8 world with a Zenoh listener at `tcp/127.0.0.1:7447`:

```
GZ_PARTITION=m8 cargo run -p gz-sim-server --no-default-features --example drive_command -- tcp/127.0.0.1:7447 0.4 0 3
GZ_PARTITION=m8 cargo run -p gz-sim-server --no-default-features --example drive_command -- tcp/127.0.0.1:7447 0 1 3
```

The command applies at the next advancing simulator step. The viewer consumes
the existing live world poses; wheel actuation does not replace body poses or
velocities. These commands and the new robot remain unqualified until the single
complete M8 verification and independent evidence review.

The command sender waits up to five seconds for a matching command subscriber
before starting its finite send interval. Matching confirms declaration discovery;
it is not an actuation acknowledgment. Encoder integration starts at the actual
loaded or restored wheel coordinates at simulation time zero, including the first
advancing physics tick. The demo retains the existing centered single-box chassis
and sphere-wheel admission; compound Rapier links and offset centers of mass are
not introduced by M8.
