<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# Bounded DiffDrive commands and encoder odometry

One loaded `DiffDriveConfiguration` on its original public Model identity installs
one canonical `gz.msgs.Twist` input and one `gz.msgs.Odometry` output. The loaded
absolute topics default to `/model/{model}/cmd_vel` and `/model/{model}/odometry`;
existing partition, namespace and remap rules qualify them. Configuration must
retain nonempty model/frame names and distinct valid absolute topics. Cross-type
collisions with world clock/pose/control, IMU or lidar endpoints are refused.

The existing world publication IO owner declares both endpoints. The existing
bounded queue admits odometry jobs with weak CPU leases; a queued job cannot keep
a removed controller alive. No drive thread, transport facade or dependency is
added. An absent system owns no drive stream, input channel, native declaration
or output lease. Last-owner retirement disables input immediately; the IO owner
releases declarations and waits for outstanding upstream callback captures.

Twist attachment, encoded size, Prost decoding and all six numeric axes are
checked on IO. Only finite linear.x and angular.z are supported; the other four
axes must be zero. Missing vectors mean zero. Malformed, wrong-type, nonfinite
and unsupported-axis packets consume a bounded ingress/budget slot and are
observably rejected. Overload increments the existing world-lifetime ingress
loss count. No Sample, unbounded payload or network wait enters the controller.

CPU command polling happens at the existing post-tick Server command boundary,
within `command_budget`. A valid last command becomes available to Scene's next
advancing step; receipt is not instant actuation or native full-tick-order parity.
Paused receipt can replace a pending command without stepping. Scene owns wheel
target limits, native motor targets, contact-authoritative body motion and measured
encoder integration. Command input and encoder tracking remain active without
odometry listeners. Matching may omit only output publication.

Odometry uses the sample's integer simulation timestamp and local odom x/y/yaw,
with quaternion yaw orientation. Twist is measured encoder forward speed in x
and yaw speed in z. Header data carries `frame_id`, `child_frame_id` and sequence;
there is no invented wire epoch, covariance, ground-truth or slip-free claim.
Samples must be finite, from the current epoch and not the future. Duplicate
acquisitions and paused repeats produce no additional output. Invalid samples
increment a counter; admitted output attempts advance sequence even on bounded
publication loss.

Reset runs IO preparation outside `tick`. Immutable initial metadata is verified
against the restored public Model identity. The old command receiver and output
lease retire, old callback state becomes inactive, and a new generation receives
its own bounded channel. Commands carry an IO-observed epoch and old queued
odometry is fenced by both epoch and weak-generation identity. Sequence restarts
at zero; Scene resets its last command and encoder reference. A reset handler's
metadata drift is terminal and drops prepared drive state. Shutdown joins the
existing owners; model removal releases the drive family without changing other
sensor/output ownership. A transport put already admitted before retirement can
finish, as for existing outputs.

New counters distinguish received/rejected Twist messages, applied commands,
queued odometry, invalid samples and publication failures. Publication failure
slot 5 is additive; original clock/pose/IMU/root-clock/lidar indices 0–4 are
unchanged. The inherited IMU/RGB/depth/lidar and force contracts remain intact.

Authored controls cover canonical field/time/unit projection, malformed metadata
and samples, all unsupported axes, actual Zenoh input/type/size/queue limits,
weak retirement and callback capture lifetime, shared-owner epoch/generation
fences, actual same-demo commands and encoder odometry at configured cadence,
no-listener actuation, pause/reset/removal, runtime typed-owner drift, absent
system and terminal reset drift. These controls passed the required M8 local
gate and exact hosted CI41776261368 with independent source/data review at
`fc967388`. Actual browser motion/control observations are recorded separately
in M8_ACCEPTANCE.md. No new C++ DiffDrive parity or performance claim is made.
