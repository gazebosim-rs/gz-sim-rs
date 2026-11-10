<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Installed sensor, publisher and wrench examples

These commands use ordinary immutable copies under `share/examples/` in the M44
Ubuntu 24.04 x86_64 preview. They need only the installed executables and ordinary
system runtime libraries from INSTALL.md. No development checkout, Cargo,
compiler, native physics SDK, remote assets, browser or new helper is required.
Keep the whole installation together and verify it before running:

```sh
preview_prefix="$HOME/gz-preview-m44-install/gz-preview-m44-linux-x86_64"
"$preview_prefix/bin/gz-preview" verify
```

Run each self-contained `default` world through the existing copied binary:

```sh
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m21_altimeter_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m22_air_pressure_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m23_magnetometer_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m24_cpu_lidar_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m26_air_speed.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m27_pose_publisher.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m28_air_speed_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m30_logical_camera.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m31_pose_static_publisher.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m33_robot_sensors.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m34_odometry_publisher.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m35_apply_link_wrench.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m36_wrench_odometry.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m38_odometry_3d.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m39_odometry_translation_offsets.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m41_air_speed_wind.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m42_joint_force_feedback.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m43_imu_reference_frames.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
```

All eighteen commands load and step twice with a 1 ms step, then exit. This establishes
finite installed execution, not measured noise statistics, canonical packet
acquisition, trajectory accuracy or browser sensor visualization. Check its
actual exit and diagnostics. `gz-preview run` still starts the complete default
robot; it has no scene selector. Do not edit the manifest or fixtures to conceal
a verification error, and do not treat the two steps as a publication test.
At 100 Hz the PosePublisher first positive-time frame is at 10 ms, beyond this finite run.

| Copied example | Configured canonical topic / schema | Bounded behavior |
| --- | --- | --- |
| `m21_altimeter_noise.sdf` | `/m21/altimeter`, `gz.msgs.Altimeter` | 100 Hz world-Z position and offset-point velocity, Gaussian/quantized channels, noise-free load reference. |
| `m22_air_pressure_noise.sdf` | `/m22/air_pressure`, `gz.msgs.FluidPressure` | 100 Hz pressure after the default absolute-height atmosphere model; zero reference altitude, finite positive pressure, native variance `sqrt(stddev)`. |
| `m23_magnetometer_noise.sdf` | `/m23/magnetometer`, `gz.msgs.Magnetometer` | 100 Hz constant world field inverse-rotated into sensor axes, then distinct X/Y/Z noise. No geographic field model. |
| `m24_cpu_lidar_noise.sdf` | `/m24/scan`, `gz.msgs.LaserScan` | 100 Hz planar three-beam CPU scan, Gaussian mean/stddev only, minimum zero; finite hits noised before clipping, infinite misses unchanged. |
| `m26_air_speed.sdf` | `/m26/air_speed`, `gz.msgs.AirSpeed` | 100 Hz ideal sensor-point/frame-correct differential pressure and temperature; zero wind and default atmosphere. |
| `m27_pose_publisher.sdf` | `/m27/pose`, `gz.msgs.Pose_V` | 100 Hz model and two link local poses in an unchanged articulated fixture, with per-pose frame headers and absent outer header. |
| `m28_air_speed_noise.sdf` | `/m28/air_speed`, `gz.msgs.AirSpeed` | 100 Hz Gaussian/quantized pressure noise in native pre-100 units; noise-free temperature and valid negative pressure. |
| `m30_logical_camera.sdf` | `/m30/logical_camera`, `gz.msgs.LogicalCameraImage`; `/m30/logical_camera/frustum`, `gz.msgs.LogicalCameraSensor` | 100 Hz paired origin-in-frustum image and configuration publications; either channel's demand activates acquisition. No rendering, occlusion or AABB test. |
| `m31_pose_static_publisher.sdf` | `/m31/pose`, `/m31/pose_static`, both `gz.msgs.Pose_V` | Moving tip at 100 Hz; model and canonical base on the static channel at 25 Hz. Independent clocks; static classification does not mean world immobility. |
| `m33_robot_sensors.sdf` | `/m33/imu`, `gz.msgs.IMU`; `/m33/scan`, `gz.msgs.LaserScan` | Complete robot plus static target, 100 Hz IMU Gaussian/quantized channels and 20 Hz planar Gaussian lidar; retained earlier sensor/joint/drive topics are unchanged. |
| `m34_odometry_publisher.sdf` | `/m34/odometry`, `gz.msgs.Odometry`; `/m34/odometry_covariance`, `gz.msgs.OdometryWithCovariance`; `/m34/transform`, `gz.msgs.Pose_V` | 50 Hz noiseless 2D ground-truth motion estimation, full quaternion, zero covariance, ten-entry body-frame filtering and no encoder sequence. |
| `m35_apply_link_wrench.sdf` | `/world/default/wrench`, `/world/default/wrench/persistent`, `gz.msgs.EntityWrench`; `/world/default/wrench/clear`, `gz.msgs.Entity` | Existing once/additive-persistent/clear semantics for centered free single-link Rapier bodies; this finite command sends no wrench. |
| `m36_wrench_odometry.sdf` | Same three wrench inputs; `/m36/odometry`, `/m36/odometry_covariance`, `/m36/transform`, same M34 output types | One mass-2 body combines commands and feedback with a 20 ms publication period; separate canonical evidence qualifies that interaction. |
| `m38_odometry_3d.sdf` | `/m38/odometry`, `/m38/odometry_covariance`, `/m38/transform`, the same three canonical types | 50 Hz noiseless body-frame linear/angular XYZ from pose history; all six channels have ten-acquisition filters. |
| `m39_odometry_translation_offsets.sdf` | `/m39/odometry`, `/m39/odometry_covariance`, `/m39/transform`, the same three canonical types | 50 Hz 3D point odometry with local translation `(0.4,-0.3,0.2)` and global origin `(-0.7,0.5,-0.6)`. Rotational offsets and noise remain unsupported. |

The schema names and topics describe canonical Zenoh interfaces (sensor
acquisition is demand-driven; PosePublisher acquisition is not). No message CLI
or new observer is bundled. Use a compatible canonical consumer in the matching partition for actual streamed measurements; acquisition
headers, frame IDs and sequence follow the established sensor contracts. The
finite commands above are not a substitute for those separate wire proofs.
The c404 browser has no numeric widgets for these noise channels.

Altimeter, pressure and magnetometer admit only finite Gaussian parameters,
nonnegative deviations/precision and signed finite means; `none` is default-only.
Native application has `dt=0`, so dynamic bias does not evolve with sample time.
Private seeded streams, no-demand/pause/non-due no-draw behavior and deterministic
Rust reset reconstruction retain their separate source contracts. Failed noisy
acquisitions cannot publish fabricated or stale valid measurements.

CPU lidar deliberately differs: only Gaussian mean and nonnegative standard
deviation are admitted. Bias, dynamic bias and precision stay default;
Gaussian-quantized is refused. Every Gaussian declaration requires minimum zero,
even at default/near-zero inactive settings; noise-free existing nonzero minima
are unchanged. The native near-zero activation boundary is preserved. Finite
noise overflow faults and clears the scan rather than becoming a clipped return.
Existing 2–128 beam, planar, one-sensor/Rapier, metadata and visibility restrictions
remain. This package adds no GPU scan, point cloud, intensity model or new sensor.

AirSpeed's signed sensor-X quadratic pressure retains native float atmosphere,
float conversion and final multiplication by 100. Its noise mean and deviation
are in the pre-100 pressure units: mean 0.1 changes the published pressure by
about 10 Pa, subject to float rounding. Noise is not applied to the published
pressure or temperature. Gaussian/quantized metadata must be finite, with
nonnegative deviations and precision; the existing precision threshold and
half-away-from-zero rounding apply. Zero-elapsed dynamic bias does not evolve.
An invalid acquisition cannot emit a stale valid packet.

The PosePublisher finite command checks the full sorted CLI world-link output:
base `(1,2,3)` and tip `(1+0.8*cos(0.6),2+0.8*sin(0.6),3)` in its zero-gravity,
zero-effort world. Its separate canonical packets contain parent-relative local
poses, not those CLI world positions. The native publisher's unordered records
are compared by name in feature evidence; no native wire order is invented.
Paused ticks never publish; subscriber absence does not stop configured pose
acquisition. The browser still shows only the default robot when launched with
`gz-preview run`, and no example acquires new lifecycle editing support.

The M30 finite command retains the entire sorted body list: `behind::body`
at `(-3,0,3)`, `rig::body` at `(0,0,3)` and `target::body` at `(4,0,3)`
(all prefixed by `default::`). Those are CLI world-link positions, not a camera
detection list. The logical camera includes visible model origins, including its
owner; it does not use collider size or occlusion. Its image carries complete
camera-relative poses, while the frustum channel carries the four configured
parameters. Zero computed plane distance is included with no visibility epsilon.
Only the separately qualified canonical acquisition proves those semantics.

The M31 command retains the same complete composed base/tip world positions as
M27 above. Static vectors use local component transforms and independent 25 Hz
cadence; their first positive-time frame is at 40 ms, beyond these two steps.
Both channels skip paused ticks, and neither requires subscriber interest.
A physical fixed-joint world remains unsupported; the copied fixture uses the
existing admitted revolute joints. No latching, individual Pose messages,
rendered camera, new widget or articulated editing is added.

M33 retains its actual robot and target SDF bytes. Its three relative mesh URI
sites use `share/examples/meshes/m16-asymmetric.obj`, separate from the default
robot's identical `share/meshes` copy. Keep both ordinary manifest-bound files.
The complete sorted CLI list is `default::ground::body`,
`default::m33_target::body`, `default::obj_landmark::body`,
`default::robot::base_link`, `default::robot::left_wheel`,
`default::robot::right_wheel`. The static positions are `(0,0,-0.1)`,
`(3,0,0.55)` and `(1.5,1.5,0)` respectively. The three dynamic robot links
require finite coordinates; their two-step positions are not a reference
trajectory. Loading the example does not prove IMU/scan demand, credit handling,
actual Twist delivery or a browser session. The default `run` robot is unchanged.

M34, M36, M38 and M39 each report the sole `default::rig::body` at `(2,-1,3)` in these
zero-gravity runs without applied commands. M35 reports both
`default::other::body` at `(0,3,0)` and `default::rig::body` at `(0,0,0)`.
The full finite-output matchers retain those complete names and positions;
they do not silently accept missing bodies. Ground-truth first publication is
at 20 ms, beyond these two steps. Its filtered body-frame Twist is not the
instantaneous world-axis backend velocity. Zero offsets still perform the
native scalar pose-composition operations, including signed-zero behavior.

Use compatible canonical consumers in the same partition for actual M34–M36
interfaces. EntityWrench targets an existing public link ID (or the admitted
name/type form), with finite world-frame force/torque and zero force offset.
Persistent inputs add; clear removes persistent entries even while paused,
but leaves queued one-shots for the next advancing step. Neither M35 nor M36
finite command sends a force/torque or proves analytic velocity increments.
Loaded reset clears configured command/history state; the plugin-bearing M36
rig is reconstructed only by loaded reset, not template cloning. Demand loss,
three canonical headers/receipts, pause/reset/removal/shutdown and command
admission are separate functional proofs. No native Physics, new noise model,
performance result or backend expansion follows from these eighteen commands.

The matching release binds fresh binaries, manifest, selected notices and actual
installed-example results. Parent model/native/wire evidence remains explicitly
scoped reuse; no C++/Rust RNG-bit, native reset, cross-engine, performance or legal
verdict is implied by these commands. INSTALL.md covers the unchanged default
robot, finite drive commands, lifecycle opt-in, Pause/Reset and owned shutdown.

M38 and M39 retain a raw model/body at `(2,-1,3)` with the same initial
roll/pitch/yaw `(0.2,0.3,0.6)`. Their two-step CLI output reports that raw body
pose. M39's canonical position instead describes the configured body-fixed
point relative to its translated global origin; do not confuse it with body
position or instantaneous backend Velocity. Both dimensional modes are
supported, but the copied example selects 3D. Healthy loaded reset reconstructs
translation configuration and filter history. A latched identity fault remains
terminal and refuses reset before mutation. The finite commands do not prove
canonical receipt, motion feedback or a native trajectory.

## New copied examples

`m41_air_speed_wind.sdf` configures finite constant WORLD wind. The AirSpeed
measurement subtracts sensor-relative airflow; the wind does not apply forces.
The existing default atmosphere, noise, acquisition and canonical `gz.msgs.AirSpeed` output
remain unchanged. See the matching source feature guide for its admitted values.

`m42_joint_force_feedback.sdf` is a zero-gravity WORLD-hinge sphere. Its existing
ApplyJointForce input is `/model/rotor/joint/hinge/cmd_force`; measured position
and velocity are in 100 Hz `/m42/joint_state` Model packets. Axis.force is zero
and unmeasured. Two finite steps do not apply commands or establish feedback.
Use the installed `gz-joint-command` walkthrough in INSTALL.md for real commands.
Zero effort coasts rather than brakes, and no native position trajectory is
promised.

`m43_imu_reference_frames.sdf` selects a NED orientation reference on a centered
IMU. NED and NWU are also admitted in the same bounded domain; named references
reject spherical-coordinate metadata, while default CUSTOM preserves its
initial-reference behavior. Only orientation reference changes; gyro and
specific force remain in sensor axes. `/m43/imu` is canonical `gz.msgs.IMU`,
not a browser telemetry widget. Finite stepping is separate from demand,
noise, canonical acquisition and physical angular-motion evidence.
