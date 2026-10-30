<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Built-in Physics world configuration

The scene loader recognizes the world record with filename
`gz-sim-physics-system` and name `gz::sim::systems::Physics`. This is the exact
record in the pinned upstream
[apply_joint_force.sdf](https://github.com/gazebosim/gz-sim/blob/71c60db31fddd5c98d2aa689e9a2cccb3548a0a3/test/worlds/apply_joint_force.sdf).
It selects the already integrated physics system. It does not load a library,
create another system instance or retain a plugin host/configuration resource.

The existing explicit Rust backend argument remains authoritative. The native
[Physics::Configure](https://github.com/gazebosim/gz-sim/blob/71c60db31fddd5c98d2aa689e9a2cccb3548a0a3/src/systems/physics/Physics.cc#L959)
also gives the explicit ServerConfig engine priority over its SDF setting.
A single `<engine><filename>` may specify the canonical DART, MuJoCo or TPE name.
It is accepted compatibility metadata under this explicit override, with no
filesystem/library lookup or post-load configuration resource. Rust's existing default remains Rapier; automatic selection from
SDF and native default-DART parity are not claimed.

One canonical record is allowed, including comments and ASCII whitespace.
The optional engine has exactly one filename containing one ordinary XML text
node. CDATA, entities and lexical splits are rejected instead of concatenating
a different selector than native TinyXML. Unknown filenames, Bullet, filesystem
paths, duplicates, other child payloads, mixed text, namespaces and extra
attributes are rejected. Other filename/name pairs retain the existing world-plugin error.
The original XML is checked against the parsed metadata; an inconsistent owned
record cannot bypass validation. Filename aliases and filesystem paths are not
library lookup requests. Native accepts more configuration than this bounded
Rust contract; unsupported content is diagnosed instead of silently ignored.

The retained native input and TPE correction fixtures now load their complete
original XML without stripping the Physics record. Their fixture bytes and all
physical/error assertions remain unchanged. A former unsupported canonical
engine guard is superseded by these positive controls and unknown-engine
rejection; other admission guards remain.

Four-engine controls compare the configured and unconfigured paths' exact step
states, schedule counts and three loaded reset cycles. Negative public and raw
metadata controls cover the admission boundaries. Existing physics, scene and
native client assertions remain unchanged. No whole original ApplyJointForce
credit is added: plane collision geometry and moving-parent actuation remain
open. This configuration mapping also does not close ACT-01's remaining idle
callback, worker/resource teardown or equal-work performance requirements.
