<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M70 finite loads and distinct installed demonstrations

This is Source preparation, not an available M70 release. No M70 installed,
frame, browser or runtime-fit result is claimed for this corrected candidate.
M69 at
`d19d5a69b94ed959bf1348665783996a531a3c36` is accepted on Main: M1–M69 and
202 slices are accepted. Earlier candidate records and failed browser attempts
are retained separately; they do not qualify this launcher cache/custody
correction. Complete Source and independent review before consolidated checks
and fresh source-bound evidence. M65 stays public Latest with 35 installed
examples. The new generation's 41 finite entries do not change old 33/35
package generations or prove 41 rendered scenes.

## Finite load/step inventory

After actual installation, set `preview_prefix` to the complete verified M70
prefix as in [INSTALL.md](INSTALL.md). These literal commands exercise two
one-millisecond finite steps each. They request no image demand or camera worker,
assert no cadence/noise/native parity, and are not a replay of installed camera
frames. Both mesh locations and all copied Source fixture bytes remain bound.

```sh
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
```

## Separate actual evidence

Run the unchanged default robot with `--lifecycle`, the complete viewer URL
printed by that same launch and matching partition as documented in INSTALL.
The fresh token directory keeps relative startup/module/Wasm requests inside
that launch's path. Old token paths and path traversal are refused; all static
GET/HEAD and error responses use `Cache-Control: no-store`, and conditional
file requests return 200, with the full body for GET, instead of 304. Ordinary
root static routes remain available for existing tools, but browser proof uses
the actual newly printed path. No viewer payload bytes or URL query/control
behavior are changed.

Capture actual forward/turn, Play/Pause/Reset and clean owned shutdown. The six
accepted viewer files are unchanged; the canvas/AX is not a numeric camera-depth
witness.

Stop the default launcher and verify owned listener retirement before starting
`run --example m69_two_depth_cameras.sdf --camera-output` on supported free ports.
Do not combine `--camera-output` with `--lifecycle`, supply a worker path, or
borrow an unrelated listener. The explicit camera mode uses the manifest-derived
worker and one ordinary OS lavapipe ICD with an absolute LVP provider path,
under its separately reviewed resource ceiling. Bare provider names and
nonempty inherited graphics/loader overrides are refused before effects;
they are not silently discarded. A default robot run does not exercise that
ceiling. The private 0700 runtime directory is removed only after confirmed
owned retirement or no spawn. If retirement is uncertain it is retained and
its path reported with UNKNOWN; inspect ownership and retirement evidence
before manual cleanup, without signalling any unrelated process.

The installed paired-depth proof must retain full canonical Image/CameraInfo,
all 4096 LE axial-depth words, matching frames/calibration/stamps, distinct
10/20 Hz streams, Info-only and per-owner laziness, pause/reset/stale refusal,
and actual two-worker/launcher/listener retirement. Use the same GZ_PARTITION
and tcp/127.0.0.1:7449 endpoint in the separately qualified external consumer.
It is SDK-host proof with an SDK-free installed producer, not a bundled SDK or
new camera-control protocol. The existing ordinary web-control path supplies
pause/reset; no new depth browser panel or ACK-as-frame claim is introduced.

After complete owned shutdown, run the separate
`run --example m67_rgb_ellipsoid.sdf --camera-output` smoke for one actual RGB
frame and full CameraInfo. It does not reobserve every copied camera geometry
or output mode. Fresh provider/library/resource, independent numeric/visual
and true shutdown evidence remain required before acceptance/public release.
Do not invent worker exit0 when the application exposes only retired lifetime;
retain UNKNOWN and require the same PID identity gone where appropriate.
