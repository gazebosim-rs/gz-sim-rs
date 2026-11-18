<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M70 finite loads and distinct installed demonstrations

This is Source correction, not an available M70 release. This candidate starts
from `597ac3ddccf150cc1c45a60c3c72b838f943093a`, retaining the D33 retirement
on Main `8f13dc7446be4fded424b1eba83df4c8253d0cef`: Rapier f64 and TPE remain;
DART and MuJoCo do not return. M1–M69 and 202 slices are accepted. M65 stays
public Latest with 35 installed examples. No qualification or installed frame,
browser or resource-fit result is claimed for this changed Source.

The parent passed its local checks, package and clean-installation audits. Its
hosted run failed at Ubuntu snapshot HTTP 401 responses before product checks.
The first installed paired-camera trial then failed at the first camera barrier,
after canonical Clock and Scene reached exactly one millisecond. The old CLI
opened a camera peer with its listener removed; the installed configuration had
no outgoing endpoint and multicast was disabled. This candidate shares the
world's existing upstream Zenoh session with camera output. It adds no endpoint
guess, router, configuration knob or transport layer. Default headless execution
and the three existing direct camera constructors retain their contracts.

All parent, frozen `fcd5a30dc0a8431b250cf96720db27ae597a6585` and failed-trial
records remain historical evidence. They do not qualify this correction.
Complete the bounded implementation, regressions, guides and independent review
before one consolidated normal/coverage pass and fresh distribution/runtime
proof. Reuse unchanged evidence only where its exact bindings permit it; the
changed package requires a fresh ordinary Ubuntu installation and GPU profile.
Keep the old package and provisioned container intact. The generation's 41 finite
entries do not change the old 33/35 package generations or prove 41 rendered scenes.

Live browser acceptance remains held for explicit approval to access
`http://127.0.0.1:8082` and its launcher-generated URLs after the retained automatic
approval rejections. No browser retry or alternative surface is released by these
instructions; numeric camera and shutdown proofs remain separately required.

The first correction normal stopped at an auxiliary registry mismatch: all 15
CLI camera cases passed, while the strict registry still expected 14. Clippy and
466 Python checks passed; full headless/renderer coverage was not reached. Its
failed tree and logs remain preserved. The bounded registry correction needs
independent review and one consolidated full retry before fresh package proof.

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

The numeric camera proof can precede the browser demonstration after fresh
package and clean-installation DATA. Stop each owned launcher and verify listener
retirement before starting another world; the camera proof uses one ordinary
web-control peer and cannot share that connection with a concurrent browser.
Use `run --example m69_two_depth_cameras.sdf --camera-output` on supported free ports.
Do not combine `--camera-output` with `--lifecycle`, supply a worker path, or
borrow an unrelated listener. The explicit camera mode uses the manifest-derived
worker and one ordinary OS lavapipe ICD with an allowed absolute LVP path or
exactly `libvulkan_lvp.so`, under its separately reviewed resource ceiling.
The basename must identify one canonical regular provider in the fixed system
library directories; usrmerge aliases count once. Other relative paths and
missing, ambiguous or escaping providers are refused. The ICD stays unchanged
for normal Vulkan loader resolution; actual mappings and package ownership
remain separate required observations. Nonempty inherited graphics/loader
overrides are refused before effects;
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
