<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# DART solid-link center of mass

This unqualified candidate follows accepted centered-COM parent `6e330abc`.
Its Scene and native COM controls ran before the later B03 joint-admission
failure; full coverage, renderer and hosted acceptance remain pending.
Scene permits a resolved offset inertial center for DART solid links. A single
centered primitive still uses the existing spawn_link API; compound or offset
collision primitives use spawn_compound_link. Both receive the same resolved
explicit link inertia, including its center and orientation. No primitive masses
are synthesized from density. Plane and other-engine admission stays unchanged.

Backend body position and WorldPose describe the link origin. Backend
BodyState linear velocity describes world COM velocity; Scene Velocity instead
uses the link origin, matching native FrameData. The existing sync converts
vOrigin = vCOM - omega cross (R * localCOM). A queued link/world velocity converts
back at application using current authoritative orientation. No-COM links retain
their exact original path without this arithmetic or an allocated COM component.
Loaded reset snapshots retain the optional physical metadata and reject runtime
orphan COM state before mutation. Attached collision poses remain link-relative. A queued world wrench is applied at the
COM, and reset must reconstruct the original physical frame and attachment
configuration without changing public identities. CPU IMU finite differences use
origin velocity, including rotational point acceleration, while velocity
configuration seeds the previous origin value to avoid a false acceleration
impulse. Controls compare all thirteen backend fields with the direct API and
check origin commands, IMU sampling, semantic frames and these lifecycle rules.
This does not claim a new native continuous-acceleration IMU qualification.

Independent source review approved the exact same-input Scene COM positive.
The unchanged B03 fixture retains its complete sun DTO check and a typed
joint-admission refusal. Once COM was admitted, its attempted construction
positive failed with `Physics(InvalidJoint)` in the consolidated gate. That
never-accepted positive is withdrawn; B03 construction, reset, motion and
performance remain open. The failure does not establish which joint guard fired.
Other-engine and plane COM refusals remain. The actual native reference is
independently qualified; Rust/Scene comparison, strict coverage, mandatory staged
checks and hosted acceptance remain separate requirements.
