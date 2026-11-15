<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M60 copied finite examples

This is provisional Source documentation; no M60 artifact or installed execution
is claimed. After the exact package qualifies and is available, set the relocated
prefix and verify it before using these commands:

```sh
preview_prefix="$HOME/gz-preview-m60-install/gz-preview-m60-linux-x86_64"
"$preview_prefix/bin/gz-preview" verify
```

Each of the 33 ordinary copied fixture is run directly through the installed
simulator. Preserve the first 18 and append the 15 newer examples without replacing
old fixtures or inventing an M44 file:

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
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m45_imu_custom_world.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m46_odometry_rotation_offsets.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m47_odometry_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m48_imu_world_heading.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m49_multiple_altimeters.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m50_multiple_air_pressures.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m51_multiple_air_speeds.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m52_multiple_magnetometers.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m53_multiple_logical_cameras.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m54_multiple_lidars.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m55_multiple_pose_publishers.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m56_multiple_odometry_publishers.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m57_multiple_joint_state_publishers.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m58_multiple_joint_controllers.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m59_multiple_diff_drives.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
```

These are two one-millisecond Rapier steps, not noise statistics or physical,
wire, lifecycle or demand qualification. The M33 file's relative mesh comes from
share/examples/meshes; the default M19 robot has its separate share/meshes copy.
The newer 15 SDFs have no external include/URI dependency in this Source contract.
Never repair missing resources by fetching a model or using a checkout.

The M45/M48 IMU frames/headings, M46/M47 odometry configuration and M49–M57
multiple sensor/publication owners retain their existing precise domains.
At 50 Hz an odometry acquisition begins at 20 ms; the M31 static 25 Hz channel begins
at 40 ms. Two steps cannot establish those output cadences. M58's position mode is
a Rust servo, not native ABS/PID trajectory parity. M59's motor targets are not
hard physical speed caps; canonical Twist/Odometry and actual native motor/body
response require the maintained separate functional controls.

For an interactive copied world, use the manifest-bound `run --example` option
in [INSTALL.md](INSTALL.md); it does not make these finite commands serve a
browser. The default launcher, its M19 robot and old command arguments remain.
All original M40/M44 guides/evidence remain historical and are not relabeled as
M60 passes. Current installed/library/browser proof and every release gate are
still required.
