<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Two independent differential-drive robots

Provisional local qualification is complete at M59 `7f849bd3`, tree `0184e824`,
over M58 `173b31d2`. The full normal passed in 2,455.201 seconds with 2,013
headless tests, 1,176 overlapping renderer tests, 449 Python checks, strict
coverage and 33 finite examples. Independent retained DATA passed, including
all four byte-exact RAW comparisons (1,831/401/47/33 original profiles) and
573 headless/262 renderer named-control joins. All six failed normal originals
remain preserved. Hosted CI, accepted predecessors, official audits and Main
readbacks still gate acceptance and publication; M59 remains unaccepted.

Retained originals and exact receipts are under `artifacts/m59-delivery`;
`provisional-local-qualification.json` binds the commit, full normal and DATA.

## Historical Source and failed-run checkpoints (before qualification)

M59 is selected on locally qualified M58 `173b31d2`. Scene, Server/IO, the
finite example and canonical C++ peer sources are adopted in the integration
candidate. Whole integration Source review and formatting passed. The sixth full
normal passed all 2,011 headless tests (zero failed, five existing ignored) and
449 aggregate Python checks, then failed Scene changed-line coverage at 363/391.
Renderer and independent retained DATA were not reached; M59 is not qualified.

## Configure the two owners

The bounded extension admits zero through two genuine loaded top-level Rapier
robots, each using the existing M8 chassis and distinct left/right revolute wheel
pair. The `examples/m59_multiple_diff_drives.sdf` fixture has one world and ground,
with model `robot_left` at Y=-2 and `robot_right` at Y=+2. Keep the M8 model's
remaining pose, mass, collision geometry, wheel axes and motor limits unchanged.
The sixth normal exercised actual loading, stepping and canonical transport;
that partial run does not establish full normal or independent DATA success.

Use the built-in configuration name `gz::sim::systems::DiffDrive` and filename
`gz-sim-diff-drive-system`; there is no dynamic plugin loader. Each plugin keeps
`left_wheel_joint`, `right_wheel_joint`, radius 0.2m and separation 0.5m.

| Model | Twist input | Odometry output | frame_id | child_frame_id |
| --- | --- | --- | --- | --- |
| robot_left | /m59/left/cmd_vel | /m59/left/odom | m59_left_odom | m59_left_base |
| robot_right | /m59/right/cmd_vel | /m59/right/odom | m59_right_odom | m59_right_base |

These are raw absolute topics. Use the same Server partition and existing
namespace/remap configuration for each publisher and subscriber; qualify keys
through `ServerOptions::topic_key`. All four resulting keys must remain distinct
and avoid every reserved world and peer input/output role. Removed original
owners still reserve the channels a loaded reset restores. A key cannot be
borrowed from another original owner or exchanged between command and odometry.

The original M8 limits remain: each wheel has 5Nm torque and a 10rad/s target
bound; configured linear commands lie in [-1,1]m/s and yaw commands in [-2,2]rad/s.
Existing uniform wheel-target scaling preserves curvature. These are motor target
bounds, not hard physical speed caps against contact or external impulses.
The default odometry cadence is 50Hz per owner. Supported existing rate settings
remain independent; the authored Server regression uses 50Hz and 25Hz.

## Send canonical commands and read measured output

Connect an upstream Zenoh session using the maintained profile and qualify the
chosen input key with the simulator's options. Wait for actual publisher matching
within a finite deadline before sending. That proves subscriber discovery, not
command application or motor response. Encode a canonical `gz.msgs.Twist` protobuf
and attach its exact type name, `gz.msgs.Twist`.

Set `linear.x` in metres/second and `angular.z` in radians/second. The other four
axes must be zero; absent vectors mean zero. For example, left (+0.4,0) and right
(-0.25,+0.5) request independent forward and reverse-turning motion. Nonfinite,
unsupported-axis, malformed, wrong-type and oversized packets are rejected without
replacing the last valid command. Invalid and old-epoch packets still consume the
one family-wide command budget; the budget is not multiplied by the owner count.

The existing M8 `drive_command` example has four arguments and hardcodes
`/model/robot/cmd_vel`. It cannot select these two routes. Do not invent a topic
argument for it. The maintained two-owner peer uses the fixed routes documented in
[M59_CANONICAL_PEER_PROTOCOL.md](M59_CANONICAL_PEER_PROTOCOL.md); it is a bounded
interoperability control, not a new standalone sender CLI. Existing M8 usage in
`docs/M8_DIFF_DRIVE.md` remains unchanged.

Subscribe to each output with canonical `gz.msgs.Odometry` decoding. Header data
contains the selected frame pair and that owner's sequence; timestamps use integer
simulation time. Position is local encoder X/Y/yaw projected to a planar pose,
and Twist is measured encoder forward/yaw velocity. The same M8 continuous-wheel
integration and ten-sample velocity window are retained. This is not commanded
velocity multiplied by time, body ground truth, or a slip-free estimate. There is
no invented wire epoch or new transform/covariance stream.

Commands are consumed at the existing post-tick Server boundary and reach motors
on a subsequent advancing Scene step. The two owners keep separate last commands,
encoder references/history, cadence, sequences and output leases. Output demand
may suppress publication; it never disables a configured driver's command input
or native actuation. No additional IO worker is introduced.

## Pause, reset and removal

The loaded simulator starts paused. Unpause through the existing public world
control before waiting for a newly acquired odometry sample. While paused,
canonical receipt may update pending commands, but zero-time/repeated updates do
not actuate motors, integrate encoders or create new odometry acquisitions.

A loaded-world reset through the existing Server control restores the original
robots, wheel identities and encoder origins, zeros pending drive state and
creates fresh input epochs and output sequences. Use `prepare_io` outside the
critical tick, or the existing `update` method that includes that preparation.
The normal paused reset leaves the world paused; resuming is a separate control.
A bare clock reset does not recreate removed models.

For an articulated M8 robot, use the existing `/world/default/remove/blocking`
RPC with canonical `EntityRequest` and `Boolean` messages. The local
`remove_editable_model` API correctly refuses this graph as `UnsupportedModel`
under its existing M10 domain; it is not the positive-removal path. The regression
first asserts that refusal and unchanged whole state, then requires the RPC's
actual true response, removal counters and original ID absence. Full model
removal retires only that robot's streams and weak output lease; the other owner
remains active. Removing the last robot releases the drive callbacks
and declarations. A loaded-world reset can restore removed originals. A queued
weak job cannot keep a removed drive alive, although a transport put already
admitted before retirement may finish.

Live original identity, public/private configuration or native wheel corruption
is a different condition: whole-set preflight refuses it before a healthy peer's
queued command or motor write. Constructor and runtime admission stay strict.
During reconstruction, valid metadata drift reports the retained reset error
only after structural, original/native, lease and complete key checks. Failure is
terminal for that run. Preserve the existing failed-reset cleanup: pending
commands become zero and stale samples are discarded even when reset fails;
there is no promise of a general rollback of earlier effects.

## Current physical-test observation boundary

The fifth full normal reached and passed both three-second physical observations
with the original position baselines, canonical commands, five-centimetre
assertions, complete wires and cadences. Initial queue totals were 225 in the
Rust 50/25 Hz case and 150 in the C++ case with only the left output watched.
Both tests then failed at the unsupported local-removal call. The earlier
one-second production cause remains unknown. The sixth normal passed the
corrected removal-RPC branches and all headless runtime controls, then stopped at
Scene changed-line coverage (363/391; strictly greater than 95 percent required).
Server's 700/721 is derived only from retained coverage, not a later executed
gate. All six failures remain retained. Full normal, renderer, strict complete
coverage and independent retained DATA remain pending; M59 is not qualified.
Two new Scene cases are independently reviewed Source, not measured passes.

## Scope and evidence

This keeps the M8 admitted geometry, wheel math and Rapier motor authority.
Additional drivetrains, nested or namespaced model layouts, acceleration/jerk,
TF/enable controls, motor/backend changes and new dependencies are outside M59.
The old manual metadata-only path remains limited to zero or one owner; it cannot
stand in for either genuine loaded robot.

M8's inspected original DiffDrive Source and odometry math remain Source-informed
authority, as documented in `docs/M8_DIFF_DRIVE.md`. No fresh original C++ DiffDrive
System, reset/tick-order, solver, trajectory or performance parity is claimed.
The maintained C++ Twist/Odometry peer supplies interoperability evidence only
after its agreed protocol and actual result pass the required checks. Source review and the consolidated formatter have passed. The full mandatory
normal and independent retained DATA are still required for M59;
hosted, official and publication gates remain separate.
