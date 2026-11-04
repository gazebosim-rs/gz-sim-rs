<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M9: one revolute chain with position and velocity commands

M9 is not accepted. The latest completed gate passed the original continuous
three-target position/velocity checks and reversed-declaration motion/reset
checks in both required profiles. It stopped at the retained renderer Scene
region gate: 7,154/7,535 must exceed 95%. The constructor-transaction,
reset-preflight, retirement and public-admission regressions now pass both
profiles. Additional shared-joint endpoint/removal and low-level admission
regressions are undergoing source review before an affected required rerun.
No threshold or assertion is relaxed.

Two earlier required attempts failed the unchanged 0.01-radian / 30,000-step
position assertion, first with a compliant velocity motor and then a rigid
velocity servo. Both failure packets are preserved. The damped native position
policy below passed the latest functional checks; full acceptance still requires
the complete gate and independent evidence review. M1–M8 remain accepted.

The five-working-day phase admits one top-level dynamic Rapier model with two
direct links and two +Z revolute joints forming a world-rooted serial chain.
Existing Scene geometry admission still applies; the example uses centered
primitive bars and adds no geometry-lowering capability. Raw link rotations are identity and joint anchors are zero in their child
frames; the root parent is the literal `world`. The canonical link is the root
body. One or two distinct configured joints own unique absolute command topics.
Configured joint effort/velocity limits must be positive and finite. Existing
unconfigured-joint limitations, moving-parent effort refusals and unsupported
backend errors remain. General articulations, physical joint stops,
mimic/transmissions, namespaces, shared-topic fanout and other controller families
are excluded.

## Reuse and command semantics

Native simulator `71c60db3` supplies two admitted configurations:

- `gz::sim::systems::JointController`, filename
  `gz-sim-joint-controller-system`, default velocity mode or explicit
  `use_force_commands=false`.
- `gz::sim::systems::JointPositionController`, filename
  `gz-sim-joint-position-controller-system`, explicit
  `use_velocity_commands=true` and finite positive `cmd_max`. Its optional
  `joint_index` must be zero; that field is unknown for JointController.

Both consume canonical `gz.msgs.Double`. Velocity targets are radians/second,
clamped to the joint's configured target-speed bound. Position targets are
continuous radians from construction, without shortest-angle wrapping. Both M9
chain joints use maintained Rapier reduced-coordinate construction, in physical
root-first order while public IDs retain XML declaration order. M8 and general
joint construction remain unchanged.

Position actuation uses the native acceleration-based position motor with fixed
stiffness 100 s⁻² and damping 20 s⁻¹, a nominal critically damped 10-rad/s isolated
response. These are documented design constants, not searched gains. Before
every advancing step, the backend obtains the conservative zero-step ERP limit
from maintained Rapier code and bounds the continuous target waypoint so its
start-of-step positional velocity bias is at most `min(cmd_max, axis.velocity)`.
Positive native substeps have ERP no larger than this limit; authoritative
coordinates may evolve during those substeps. The backend also validates the
supplied step's coefficients, finite arithmetic and rounding before mutation. Target velocity is zero and the same finite `axis.effort` Nm cap
applies. The selected step duration must match that next physics step. Finite
torque and coupled motion can still cause tracking error or physical overspeed;
qualification must establish the retained example's actual convergence.

Velocity mode uses the opt-in maintained native velocity constraint with zero
spring stiffness and damping, and the same finite torque cap. Parent reaction
and contacts remain physics-owned. M8 retains its gain-one compliant velocity
motor unchanged. No custom solver or user-space PID is added. The recognized
SDF plugin modes and command identities are reused, but the Rust position policy
is now a damped servo: native ABS velocity-lowering/order parity is not claimed.
The position API explicitly refuses impulse-joint handles, whose quaternion
position motor aliases complete turns; admitted M9 joints use continuous reduced
coordinates. Default position force-PID and
optional velocity force-PID modes are explicitly refused, as are actuator
messages, unsupported fields and conflicting actuator ownership. Target bounds
are not hard physical speed caps or joint stops.

Commands replace pending targets without stepping, including while paused.
Invalid commands preserve the last target. Every chain target and native motor
is validated before any native target is changed. Configured initial targets
are restored on ordinary and loaded reset; motors are disabled first. Ordinary
Scene reset preserves pause state. The existing viewer reset explicitly requests
`pause: true`. Removing the model retires controller callbacks and bounded input
leases; absent families own no controller callbacks, buffers or endpoint state.
Measured joint samples and existing pose snapshots supply inspection/visuals;
there is no additional controller output protocol.

## Runnable example (unqualified until M9 acceptance)

Build the native-free simulator and finite sender:

```sh
cargo build --locked -p gz-sim-cli --no-default-features
cargo build --locked -p gz-sim-server --no-default-features --example joint_command
```

Create `/tmp/m9-zenoh.json5` containing:

```json
{"listen":{"endpoints":["tcp/127.0.0.1:7449"]},"scouting":{"multicast":{"enabled":false}}}
```

Start the world paused:

```sh
GZ_PARTITION=m9 ZENOH_CONFIG=/tmp/m9-zenoh.json5 target/debug/gz-sim-rs examples/m9_joint_controllers.sdf --server --web-viewer 9009
```

Serve the existing gzweb-rs app on HTTP port8080 or8081 and use
`?backend=primitive&ui=full&url=ws://127.0.0.1:9009/`. Press Play, then:

```sh
GZ_PARTITION=m9 target/debug/examples/joint_command tcp/127.0.0.1:7449 position 0.7 1
GZ_PARTITION=m9 target/debug/examples/joint_command tcp/127.0.0.1:7449 velocity 0.5 3
```

The sender waits up to five seconds for subscription matching before sending;
matching is discovery, not actuation acknowledgement. A successful velocity
invocation ends with zero-target braking. A position invocation retains its last
position target. Failure does not promise a final stop. Endpoints default to
`/model/arm/joint/base_hinge/0/cmd_pos` and
`/model/arm/joint/tip_hinge/cmd_vel`. Geometry, IDs and viewer protocols are reused.
The simulator remains native-physics-SDK/window/GPU free in the selected build.
The browser requires a supported graphics backend.

No new C++ controller parity, performance or general controller-family acceptance
is claimed. See M9_JOINT_CONTROLLER_IO.md for bounded ingress and reset fences.
