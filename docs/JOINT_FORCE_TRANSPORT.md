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
all joints. Real queue-full losses remain visible in the shared world-lifetime ingress
metric, including late callbacks after CPU retirement.

Direct Zenoh subscribers and MS liveliness tokens live on the existing world
publication IO owner. CPU ingress holds only scalar receivers, active gates and
fanout identities. A server without configured controllers retains no force CPU
state and skips its polling/retirement calls; it creates no force channel or native
declaration. The shared world infrastructure remains independently active.

Tick processing performs nonblocking scalar receives and checked Scene operations.
Removal retains shared endpoints until their final target disappears, then drops
the CPU state and disables handoff. Native declaration and callback-capture cleanup
remain on IO. Loaded reset reconstructs original identities from Scene metadata,
starts zero commands and prepares replacement declarations outside raw ticking.
Old bounded receiver queues are discarded. A packet arriving after reset has no wire epoch;
there is no claim that reset cancels a remote publisher's in-flight messages.
Shutdown disables CPU ingress and joins the world IO owners outside stepping. Failure
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

The unchanged original ground-world controls are documented in
[APPLY_JOINT_FORCE_ORIGINAL.md](APPLY_JOINT_FORCE_ORIGINAL.md). This CPU ownership
follow-up adds no original-case credit. Moving-parent actuation and complete
world physics plugin selection remain outside this slice. It adds no native
plugin ABI, rendering, new backend, broad malformed-protobuf compatibility claim
or performance improvement claim.

The optional CPU-state follow-up does not replace the shared IO owner's existing
force-retirement timer or eliminate other always-active server families. It does
not establish full ACT-01 absence, no-wait shutdown, or a performance improvement.
Its focused checks and independent review must be followed by the unchanged
strict staged and hosted gates before acceptance.
