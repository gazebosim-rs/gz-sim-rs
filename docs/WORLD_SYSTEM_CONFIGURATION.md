<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Built-in Physics world configuration

The scene loader recognizes the empty world record with filename
`gz-sim-physics-system` and name `gz::sim::systems::Physics`. This is the exact
record in the pinned upstream
[apply_joint_force.sdf](https://github.com/gazebosim/gz-sim/blob/71c60db31fddd5c98d2aa689e9a2cccb3548a0a3/test/worlds/apply_joint_force.sdf).
It selects the already integrated physics system. It does not load a library,
create another system instance or retain a plugin host/configuration resource.

The existing explicit Rust backend argument remains authoritative. The native
[Physics::Configure](https://github.com/gazebosim/gz-sim/blob/71c60db31fddd5c98d2aa689e9a2cccb3548a0a3/src/systems/physics/Physics.cc#L959)
also gives the explicit ServerConfig engine priority over its SDF setting.
This slice accepts no engine payload. Rust's existing default remains Rapier;
no native default-DART or automatic engine-selection parity is claimed.

One canonical record is allowed, including comments and ASCII whitespace.
Duplicate records, child payloads, mixed text, namespaces and extra attributes
are rejected. Other filename/name pairs retain the existing world-plugin error.
The original XML is checked against the parsed metadata; an inconsistent owned
record cannot bypass validation. Filename aliases and filesystem paths are not
library lookup requests. Native accepts more configuration than this bounded
Rust contract; unsupported content is diagnosed instead of silently ignored.

Four-engine controls compare the configured and unconfigured paths' exact step
states, schedule counts and three loaded reset cycles. Negative public and raw
metadata controls cover the admission boundaries. Existing physics, scene and
native client assertions remain unchanged. No whole original ApplyJointForce
credit is added: plane collision geometry and moving-parent actuation remain
open. This configuration mapping also does not close ACT-01's remaining idle
callback, worker/resource teardown or equal-work performance requirements.
