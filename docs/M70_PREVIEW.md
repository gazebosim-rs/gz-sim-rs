<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M70 finite loads and distinct installed demonstrations

[M70 release 452158796](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m70-preview)
is public and Latest with 41 installed examples. Its payload was built at
`7094a31890e78bc83c65f12a056dfb3918b7401a`; accepted Main
`616cc3e2105a7ef9479ac823a6b05e832a92defc` separately includes M70–M75.
Later Source features and the planned M76 example are not in this payload.
Rapier f64 and TPE remain; DART and MuJoCo product backends stay retired.

The exact package, clean installation, paired-depth/GPU components, separate
native RGB and owner-approved default-browser demonstration have successful
independent evidence. The original combined camera controller remains failed.
Its missing depth host-wait ledger remains Source-flow inference, and SDK
quiescence and discarded grandchild exits remain UNKNOWN. Follow the published
standalone guides or maintained [installation](M70_INSTALL.md) and
[capabilities](M70_CAPABILITIES.md) guides, which correct the historical archive
checkpoint documents. The archive's unchanged extracted directory is
`gz-preview-m70-websocket-close-handshake-provisional`.

## Finite load/step inventory

After actual installation, set `preview_prefix` to the complete verified M70
prefix as in [M70_INSTALL.md](M70_INSTALL.md). These literal commands exercise two
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

The default robot demonstration used its owned launcher URL and the six unchanged
accepted c404 viewer files. Actual Play, finite forward/turn, Pause with stable
clock, Reset and owned shutdown passed, with four JPEG/four AX observations and
independent process and visual review. The historical queue-full 4002 toast was
retained; its cause remains UNKNOWN. That proof reused the provisioned 8 GiB,
512-PID/four-CPU container and does not establish a fresh 1 GiB browser fit.

The separate paired-depth proof used the existing launcher route:

```sh
"$preview_prefix/bin/gz-preview" --partition "$GZ_PARTITION" run --example m69_two_depth_cameras.sdf --http-port 8082 --ws-port 9014 --camera-output
```

It retained complete canonical Image/CameraInfo, every 4,096-word axial-depth
array, matching frames/calibration/stamps, independent 10/20 Hz streams,
Info-only/per-owner laziness, paused observation and reset/stale refusal, actual
worker identity retirement, launcher0 and listener retirement. Reset has an
actual Scene epoch/ACK; CameraInfo/Image do not gain a camera-wire epoch field.
No SDK-wide callback-quiescence or FIFO-wide publication-drain claim follows.
The SDK-free producer and separately qualified external SDK consumers remain
distinct. Finite loads alone do not prove any image or native renderer parity.
The browser remains the Scene/control view, with no new depth browser panel.

The ellipsoid RGB proof used a **native paused server without a web exporter**:

```sh
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m67_rgb_ellipsoid.sdf" --server --camera-worker "$preview_prefix/bin/gz-render-worker"
```

Do not use `run --example m67_rgb_ellipsoid.sdf --camera-output`: the web exporter
refuses Ellipsoid. The successful native route required the supported OS profile,
owned runtime, ordinary lavapipe ICD, packaged Zenoh configuration, matching
external subscriber and existing Clock/WorldControl service. It retained the first
1 ms pair, full canonical RGB_INT8/CameraInfo, pixel checks, a live-SDK in-process
interlock and owned worker/native-server retirement. It is not an automatic
user-facing smoke command or browser image proof; the external QA helper is not
bundled. See M70_INSTALL.md for the exact observed environment boundary.

Each demonstration has its own actual-zero audit. Their composition preserves
the original aggregate Root1/full-camera false and all unavailable exits. Stop
owned worlds and establish their listener retirement before another launch;
never borrow or signal an unrelated listener. The final public release and
ordered phase acceptance receipts are retained in `artifacts/m75-delivery`.
Neither the failed aggregate nor any earlier manual attempt is relabeled or
replayed by these maintained instructions.

## Historical Source checkpoint

This retained narrative describes the earlier correction and failed attempts,
not a current approval hold, qualification request or instruction to retry.

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

