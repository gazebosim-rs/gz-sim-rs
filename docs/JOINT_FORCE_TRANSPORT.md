<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Bounded joint-force transport

Recognized loaded ApplyJointForce controllers subscribe to `gz.msgs.Double` on
`/model/{model}/joint/{joint}/cmd_force`. Names come from immutable initial scene
metadata. The pinned Transport topic sanitizer runs once, followed by the
configured namespace and topic remapping rules. This follows the upstream
ApplyJointForce Configure / OnCmdForce / PreUpdate source contract.

Each received finite value replaces the joint's persistent Bevy command through
`set_joint_force`; the existing physics preparation system adds it once per
advancing step. Paused ticks admit commands without accumulating physical force.
An empty protobuf Double means zero. Malformed protobuf and rejected scalar or
binding inputs preserve the preceding command and increment rejection counters.
The native system does not perform these finite-value admission checks; the Rust
backend's existing numerical boundary remains in force.

Subscriptions with the same qualified, remapped endpoint share a bounded receiver
and fan out to their configured joints. Distinct endpoints are polled round robin
with the server's finite command budget. That budget counts received packets;
fanout is bounded by the immutable loaded controller inventory. Counters distinguish
received packets, rejected packets and successfully updated joint targets. A
partially rejected fanout can update valid targets; it is not a transaction across
all joints. Transport queue drops remain visible in aggregate transport statistics.

Worker Nodes are created before stepping, with at most one worker per Transport
capacity-sized group of endpoints. Tick processing only performs nonblocking
receives and existing checked scene operations. Removal retires controller targets
and empty subscriptions. Loaded reset reconstructs the original identities with
zero commands and replaces subscriptions using the prepared workers. Old bounded
receiver queues are discarded. A packet arriving after reset has no wire epoch;
there is no claim that reset cancels a remote publisher's in-flight messages.
Shutdown drops subscriptions and joins owned workers outside stepping. Failure
while reconstructing subscriptions leaves the committed reset terminal, following
the existing server reset contract.

The C++ `force_client.cc` fixture uses a real native gz-transport Node and typed
Double publisher. Its stdin/stdout commands coordinate the test harness only;
force values cross the unchanged Gazebo wire protocol. `native-peer.py --client
force` applies the same pinned SDK, compiler and loaded-library provenance checks
as the existing clock and IMU rate clients. Author tests exercise all three dynamic
engines, physical force response, pause, default zero, nonfinite refusal, reset,
retirement and unread queue cancellation. These additions require independent
review and the normal staged and hosted gates before acceptance.

This slice does not complete the original ApplyJointForce integration fixture.
Plane geometry, moving-parent actuation and world physics plugin selection remain
unqualified. It adds no native plugin ABI, rendering, new backend, broad malformed
protobuf compatibility claim or performance improvement claim.
