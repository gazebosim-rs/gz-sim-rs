# TPE loaded-world slice

`load_world` retains Rapier as its default. `load_world_with_engine` accepts
PhysicsEngine::Tpe, and the headless CLI exposes `--physics-engine tpe`.
The loader currently accepts top-level, unjointed models with centered box/sphere
collisions. Nested TPE models fail explicitly. The existing plugin, surface,
fluid-added-mass, frame, IMU configuration and source-validation guards remain.

The selected TPE path is kinematic. Gravity does not accelerate bodies; mass and
inertia do not affect motion. Gravity still supplies IMU specific force. Collision
shapes are validated, but contact reporting is not implemented in this slice.
No collision response, joint, force or link-velocity command is implied. Full TPE
capabilities, native test parity and the complete headless P2 milestone remain open.

Bevy owns a validated Motion component on each model. Direct child links retain
fixed local poses; committed model motion composes those frames before shared
attachment, local-pose and IMU observation systems run. No native World/Entity
class hierarchy or second body registry is ported. Direct model state replacement
is a low-level Rust operation with constructor reset semantics; it is not a Gazebo
world-velocity command. Static/rotated-link command policy D23 remains undecided.

Pinned World::Step updates top-level models and direct child link-local motion.
This slice implements zero link-local velocities. The native plugin reports model
velocity for that case, without adding angular-velocity cross link-offset terms.
Do not substitute a rigid-body offset-velocity formula silently. Independent link
motion and nested-model behavior need their own reference evidence.

Model-subtree removal validates the selected engine and TPE bindings before Bevy
retires descendants and identities. The immutable loaded-scene blueprint retains
Motion constructor state and a world-specific owner token. Reset preflight rejects
missing/foreign/both-engine ownership and unsupported runtime physics bindings
before mutation. Successful loaded reset restores original public IDs, poses,
Motion, sensors and epochs through the existing transaction. Core time reset only
restores surviving Motion components. Existing terminal-fault boundaries apply.

The seventh actual C++ transport scenario runs against the selected TPE world.
It checks an exactly stationary model, 9.81 m/s² IMU specific force, clock/pose
messages, controls, deferred removal, original-ID restoration, sampled reconnect,
a second live reset and restarted sequence/epoch behavior. The six existing Rapier
scenarios retain their assertions and tolerances.

The supplementary native generator tests/native_tpe_scene.cc runs the freshly
built pinned TPE World/Model/Link library for 100 × 1 ms steps, with a rotated model
and offset/rotated link, both stationary and moving. Its two rows are retained in
planning/reference/tpe-scene-motion.json with source, generator, binary and linked
library hashes. Rust compares them at 1e-12 and separately checks analytic poses,
IMU frames, reset and velocities. This is a direct model-motion reference, not a
full native gz-sim/SDF/transport run. Installed dependencies remain qualified.

Rebuild the generator against the fresh pinned TPE build, using the same include
and library setup as TPE_MOTION.md. No installed TPE SDK is required; math/utils
headers and their linked dependencies are still required. The stored compile
command and hashes identify the author's run. Independent regeneration, review,
strict staged checks and hosted CI are required before publication. No upstream
case is retired and no scene-level speedup is claimed.
