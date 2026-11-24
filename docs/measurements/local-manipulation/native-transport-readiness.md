<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Native transport readiness under coverage

The staged Ubuntu coverage run stopped on the finite paused CLI, gyro unsubscribe and differential-drive Twist tests. The hosted run at `59a606e` also missed the first complete native odometry, covariance and transform callbacks in two fixtures. Each test now starts its measured interval after observing its actual transport boundary.

| Scenario | Evidence required before the measured operation |
|---|---|
| Finite paused CLI | An isolated, continuously held Zenoh listener supplies its bound loopback address to the executable. A decoded paused clock verifies publication. The control queryable must match before the single request, which must return an affirmative Boolean. Both engines still exhaust exactly three advancing iterations across reset. |
| World-heading IMU | The private routing peer observes the C++ sensor subscription appear and disappear. After unsubscribe, the server must hold the last reading and its publication count across a complete due interval and another 20 advancing ticks. The transport propagation deadline uses wall time. |
| Differential-drive reset | A malformed Twist probes the rebuilt callback while physics remains paused. The server must count its rejection in the new epoch with zero motion time and zero commands applied. The actual forward and turning commands are then sent once each, and both must be applied. |
| 3D and noisy odometry | The private routing peer sees all three native subscriptions before the first measured acquisition, their disappearance at disconnect, and their return on the same Gazebo peer. The existing C++ callbacks still verify each three-message set at its authored timestamp, including after reset. |

The staged image was `gz-fold-qualification:noble-network-tools`, SHA-256 `c1fb881cdd3c52cd77b1b390acdc723ea3f9cbc24268466427e554a498d691c2`. Runs used the repository's workspace coverage compiler wrapper, the staged test optimization and overflow settings, the pinned native Gazebo transport reference, new private partitions and kernel-assigned loopback ports. The five corrected scenarios passed all 25 repetitions. The five complete instrumented test targets passed all 23 tests. Strict Clippy and formatting passed on the changed test targets. This qualification changed only test harnesses.

In the comparison runs the original heading test failed in two of three attempts at its fixed tick bound. The original unmodified paused CLI passed 12 additional runs, and the original Twist and two odometry scenarios each passed three. Their recorded staged and hosted failures were intermittent. The retained original and amended runs allow each result to be distinguished.

Run both full targets in the staged coverage environment.

```sh
cargo test --locked -p gz-sim-cli --test native
cargo test --locked -p gz-sim-server --test imu_world_heading --test robot_sensors \
  --test odometry_3d --test odometry_noise
```
