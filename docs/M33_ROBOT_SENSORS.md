<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M33 mobile-robot IMU and planar lidar coexistence

`examples/m33_robot_sensors.sdf` is a separate source example. It retains the
complete M19 robot, wheels, DiffDrive, altimeter, pressure, magnetometer,
JointStatePublisher and OBJ geometry/settings. One centered base-link IMU, one
offset CPU lidar and an ordinary static box target are added. The accepted M32
installed robot and archive remain unchanged; this phase does not refresh a
package or add an installed scene selector.

The world has four models and seven visuals: all three old models and six old
visuals, plus `m33_target::body::box`. Both copied OBJ visuals remain present.
Sensors do not create new visuals or numeric widgets. The articulated robot
retains `can_remove=false` and `can_set_state=false`; no lifecycle edit permission
or remote model loading is added.

## Existing sensor and command contracts

| Interface | Configuration | Topic / frame |
| --- | --- | --- |
| IMU | 100 Hz, zero local translation and rotation. Acceleration X uses Gaussian mean 0.1 and standard deviation 0.01; angular Z uses Gaussian-quantized mean 0.02 and precision 0.01. Other noise fields/channels stay default. | `/m33/imu`, `robot/base_link/imu`, canonical `gz.msgs.IMU`. |
| CPU lidar | 20 Hz, local offset `(0,0,0.35)`, one planar three-beam scan across ±pi/4, range 0–10 m. Existing Gaussian range mean 0 and standard deviation 0.01. | `/m33/scan`, `robot/base_link/lidar`, canonical `gz.msgs.LaserScan`. |
| Differential drive | Existing 0.2 m wheel radius, 0.5 m separation, wheel velocity/torque and linear/angular bounds. | `/model/robot/cmd_vel`, canonical `gz.msgs.Twist`; `/model/robot/odometry` retains its existing frames. |

The target is a static 1×1×1.1 m box at `(3,0,0.55)`, with ordinary collision and
visual geometry. Lidar returns derive from the actual acquisition pose and
Rapier raycasts. Finite hits are noised before clipping to the admitted range;
positive-infinite misses remain misses and consume no draw. No new occlusion,
intensity, multi-layer or point-cloud model is added.

The existing six-channel IMU Gaussian/quantized implementation is reused, not
credited as new noise-model work. IMU noise retains its existing elapsed-acquisition
interval arithmetic (zero on its first acquisition); lidar noise uses native zero
elapsed time. Private seed/public identity roles and precision rounding remain intact. Covariance,
orientation, frame, sequence and integer acquisition stamps retain their existing
canonical contracts. Paused, non-due and uninterested acquisitions consume no new
draws. Loaded reset rebuilds the documented Rust seed state; this does not promise
native RNG bits or cross-engine robot trajectories. Invalid acquisition cannot
publish a fabricated or stale successful sample.

## Bounded integration proof

The new Server regression and focused canonical C++ Zenoh peer consume both
actual streams while real Twist commands drive and turn the complete robot.
They bind packets to public entity identities and current acquisition samples,
with header/frame/sequence/covariance, separate rates, pause/demand, loaded-reset
seed/public-identity replay, reconnect, Scene model removal, Server shutdown and
atomic initial remap-collision refusal controls. With no demand, the IMU holds
its last local reading without new acquisition/publication; lidar removes its
stale local scan. Existing accepted private typed-family reset-collision and
weak-lease controls remain mandatory retained evidence, rather than new M33
public-test coverage or a reason to add private hooks. The peer uses the maintained qualified SDK and registration path. It is new
coexistence interoperability, not a replay of old native noise or trajectory
acceptance.

The new Primitive socket regression uses the maintained local WebSocket protocol
with `scan=1&lifecycle=1&lifecycle_credit=1`. It keeps every old catalog entry and
visual plus the target, including unchanged robot edit flags. The test explicitly
selects the existing local `Always` acquisition policy; canonical subscriber
demand is covered separately by the Server controls. Canonical Twist commands
arrive through the existing command subscriber, without teleportation or a new
mutable simulation API. Command admission during pause leaves time and samples
unchanged.

While publication credit is held, actual IMU and lidar schedules and physics keep
advancing. Real Ping/Pong FIFO barriers check that no normal scene/scan flight
escapes before Ready. Ready releases the current scene/catalog/scan pair: the
scan's ID, acquisition time, pose, geometry and complete ranges must match the
actual `LidarSample`, including null encoding of infinite misses. Pose changes
come from the real drive motors, not UI-only transforms. The 100 Hz IMU advances
while the 20 Hz lidar sample stays held between its due acquisitions.

A reset ACK must describe epoch 1, zero time and paused state before Ready
releases the restored original catalog/visuals and cleared scan. Disconnect and
reconnect retire the socket's scan interest and restart its credit token, with
the reset scene intact. The next first acquisition must reproduce the original
IMU values and lidar ranges under the same loaded seed and public identities.
These are component/sample and real-socket assertions, not new IMU telemetry.

The source example can be selected using the existing CLI with Rapier and the
usual local viewer/scan options. It is not the default `gz-preview run` world.
All required native/wire/socket proofs, the complete normal hook, strict
workspace/per-crate/actual-parent coverage, exact hosted CI and independent
local/official DATA must pass before qualified Main acceptance. Source authoring
alone claims no execution. No new runtime, schema, dependency, renderer, widget,
packaging, performance or legal-compliance contract is introduced.
