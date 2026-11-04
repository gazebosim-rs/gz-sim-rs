<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M9 joint-controller command ingress

This Source implementation adds command input only. Scene owns controller
admission, continuous joint targets and the opt-in torque-capped Rapier native
position/velocity servos described in RAPIER_JOINT_MOTORS.md. The newest position
correction prepares a damped reduced-coordinate waypoint each advancing step;
the earlier clipped ABS velocity policies failed the retained coupled test.
The accepted M8 compliant motor API remains unchanged. Default force-PID modes
and actuator messages are outside the admitted subset.
There is no custom solver, body-velocity edit, output family, dependency or IO
thread. The newest position correction is Source-only and still unqualified.

The loaded world admits at most two distinct configured joint identities. Each
owns one canonical `gz.msgs.Double` subscription on the existing world IO owner.
For the authored `examples/m9_joint_controllers.sdf`, the topics are
`/model/arm/joint/base_hinge/0/cmd_pos` (radians) and
`/model/arm/joint/tip_hinge/cmd_vel` (radians/second). Existing explicit topic
remapping and qualification apply once. Subscriptions do not depend on pose or
sensor subscribers; commands can drive the chain with no output interest.

The IO callback requires the exact canonical type attachment, bounded payload,
successful maintained Prost decode and finite scalar. It hands off only an
internal `(epoch, Option<f64>)` pair; malformed packets consume one rejection
slot. Each stream has `ServerOptions.capacity` slots and consumes at most the
existing command budget per tick. Thus this family has at most two queues and
consumes at most twice that budget, in addition to unchanged older families.
Overload increments the existing world ingress-drop metric. New received,
rejected and applied counters do not change existing counters or the six
publication-failure indices. No command acknowledgement or reset epoch is
invented on the Double wire.

Command polling follows the existing DiffDrive poll after the completed tick.
An admitted command affects the next advancing Scene PreStep. Paused receipt can
replace a pending target, without moving physics or advancing time. This is not
a claim of complete native system-order parity. Nonfinite or unrepresentable
commands preserve the last target. Scene target-speed limits are not hard
physical speed caps, and position targets do not introduce physical joint stops.

Server setup pins each loaded joint ID and immutable configuration to the initial
Scene blueprint. It checks the typed joint owner, ID lookup, current joint handle
and child, unique endpoints, and collisions with world clock/pose/control/removal,
IMU/rate, lidar, DiffDrive and joint-force endpoints. Live public metadata, kind,
ID or handle drift is terminal: leases and queues retire, the first error is
`JointControllerIdentity`, and later frame admission returns the existing
terminal `ResetStreams` error. Scene checks deeper native ownership and state;
all command errors except `InvalidCommand` are terminal rather than silently
treated as bad user input. Legitimate owner removal or pending reset removal
retires the corresponding stream; the other stream can remain live.

Reset uses the original `(joint ID, configuration)` blueprint. Old CPU leases
and receivers drop before a replacement generation is installed outside the
critical tick. The IO owner undeclares retired endpoints and waits for old
callback captures to drain before admitting the next generation. A callback
checks its active flag before decode and again before handoff. Its acquisition
epoch is never relabeled. Queued old commands cannot enter new receivers;
Scene restores configured initial targets. Reset metadata drift is terminal
`ResetJointControllers`. Shutdown drops leases before joining the existing IO
owner. An absent controller system has no controller state, queues, declarations
or additional IO polling timer.

Authored controls cover actual upstream Zenoh Double delivery for both fixture
topics, physical movement without output interest, pause/reset/stable IDs,
malformed/type/size/nonfinite input, internal invalid/stale-epoch handoffs,
per-stream budgets, old queued work, independent lease retirement, removal and
absence, public-owner drift, Scene native-owner refusal, disconnected ingress,
setup bounds/duplicates, world endpoint collisions, callback capture retirement,
and a refused overlapping reset installation. All prior tests and assertions
remain unchanged. Prior gates compiled the Server scope but stopped at Scene's
real-chain convergence control; those partial outcomes do not qualify M9.
The newest position correction and these Server runtime controls remain unrun;
whole-correction Source review must finish before the affected required gate.
