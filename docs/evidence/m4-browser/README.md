<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Actual M4 browser observations

These are byte-exact original JPEGs and their source/binary binding manifest.
The manifest retains paths and qualification status in the capture environment;
current acceptance is recorded separately in
[../../M4_ACCEPTANCE.md](../../M4_ACCEPTANCE.md). The simulator
is `eae6f3b8`; the unchanged accepted viewer is `5b67f45f`. The exact retained
native-free instrumented executable was reused without another build.

| Capture | Actual observation |
| --- | --- |
| [Initial](01-initial.jpg) | Paused at zero; blue IMU-bearing cube above ground, two visual entities. |
| [Motion](02-motion.jpg) | Current time 26.414 seconds; cube moved onto ground. Resume acknowledgement: epoch 0, 0.001 seconds, paused false. |
| [Pause](03-paused.jpg) | Current time and pause acknowledgement 30.220 seconds, epoch 0, paused true. |
| [Reset](04-reset.jpg) | Current time zero, reset acknowledgement epoch 1, paused true; original apparent placement restored at the same camera. |

The browser is not an IMU sample inspector. Units, axes, sample times/counts,
covariance and canonical wire values are proved by the separate four retained
passing regression tests, including real upstream Zenoh and Prost decoding.
The images show world motion and committed controls only. Background 1 fps is
visible; no performance or separately inspected renderer-backend claim is made.
Startup initially showed Waiting before the scene appeared. The preserved
manifest's pending qualification text describes capture-time status.
