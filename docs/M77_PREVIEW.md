<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M77 installed examples

These are the 47 selected manifest-bound finite examples through M76. The
frozen `353dc9ae` package passed their finite loads, then failed its first live
camera startup before SDK consumers at the Scene visibility guard. The selected
nonzero-flag correction and wrench-harness preconditions remain unqualified;
all fresh Source/package/runtime gates still apply. These commands are required
installed checks, not results for the correction. The public M70 archive still
contains its original 41. Extract and verify the complete prefix using
INSTALL.md first.
The reviewed `95ef6ced` correction later failed its normal at a wrench-test
FIFO EOF assumption, not an installed frame check. The current helper correction
is unqualified; the original frozen failure and UNKNOWN original CI1 cause stay
separate from the successful unchanged original-head CI2.

## Finite world loads

Each command loads its copied world and advances exactly two 1 ms steps without
starting a live server, camera subscriber or renderer worker. This checks the
installed world/load path, not live sensor output, cadence or browser rendering.
The six mixed-camera worlds use the same finite path. Do not add a bare
`--camera-mixed-pair`: that CLI option requires a worker and server.

```sh
set -eu
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m21_altimeter_noise.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m22_air_pressure_noise.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m23_magnetometer_noise.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m24_cpu_lidar_noise.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m26_air_speed.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m27_pose_publisher.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m28_air_speed_noise.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m30_logical_camera.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m31_pose_static_publisher.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m33_robot_sensors.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m34_odometry_publisher.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m35_apply_link_wrench.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m36_wrench_odometry.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m38_odometry_3d.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m39_odometry_translation_offsets.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m41_air_speed_wind.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m42_joint_force_feedback.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m43_imu_reference_frames.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m45_imu_custom_world.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m46_odometry_rotation_offsets.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m47_odometry_noise.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m48_imu_world_heading.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m49_multiple_altimeters.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m50_multiple_air_pressures.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m51_multiple_air_speeds.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m52_multiple_magnetometers.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m53_multiple_logical_cameras.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m54_multiple_lidars.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m55_multiple_pose_publishers.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m56_multiple_odometry_publishers.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m57_multiple_joint_state_publishers.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m58_multiple_joint_controllers.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m59_multiple_diff_drives.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m61_eight_diff_drives.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m62_eight_joint_controllers.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m63_two_rgb_cameras.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m64_rgb_cylinder.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m66_rgb_sphere.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m67_rgb_ellipsoid.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m68_depth_primitives.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m69_two_depth_cameras.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m71_mixed_cameras.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m72_camera_intrinsics.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m73_camera_projection.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m74_camera_skew.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m75_camera_visibility.sdf" --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m76_camera_projection_offsets.sdf" --iterations 2 --step-ns 1000000
```

Require each command to succeed and report `simulation_ns=2000000 iterations=2`.
Retain the complete per-world output when qualifying an installation. A finite
load does not prove an image or native-renderer result. The source camera CLI
registry has 21 cases; that is separate from this 47-file installed inventory.

## Live mixed cameras

Use the explicit commands in INSTALL.md for `m76_camera_projection_offsets.sdf`
and `m74_camera_skew.sdf`. Both need `--camera-output --camera-mixed-pair` in
M77. The first combines distinct K/P, metadata offsets, visibility masks and
independent RGB/depth rates; the second exercises signed intrinsic skew.
The default viewer's implicit all-bits mask admits nonzero visual flags in the
correction candidate; zero flags and opacity/material failures remain refused.
This does not change camera masks or the archived failed operation.
Information-only demand remains lazy. Actual installed acceptance checks complete
canonical frames, calibration, stamps, demand, cadence, pause/reset and retirement
against the existing fixture-specific numerical oracles.

For selective model removal, use the existing canonical
`/world/default/remove` or `/world/default/remove/blocking` Entity service with
an upstream Zenoh client in the same partition. The local editable-model API and
browser lifecycle editing are different, restricted routes. Do not combine
`--lifecycle` with camera output. Canonical loaded reset restores original scene
state; an actual successful reply and subsequent output must be observed during
qualification.

The default robot browser proof is separate: use its printed fresh URL, Play,
finite forward/turn, Pause and Reset, then stop only the owned launcher and wait
for its processes and listeners to disappear. No camera panel is supplied.
The ellipsoid example remains native-only and must not be launched as a web
Scene export. Read CAPABILITIES.md for limits and the pending evidence boundary.
