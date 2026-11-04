<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# Bounded revolute velocity motors

M8 adds `JointVelocityMotor { target_velocity, max_force }` to the existing
`DynamicBackend` boundary. Targets are finite radians per second; maximum actuator
torque is finite, nonnegative Nm. Validation is read-only; replacement is atomic
for one joint. Root's controller validates both wheel commands before replacing
either. Other backends return typed `Unsupported` before identifier or numeric
validation. Rapier reports `MissingJoint` for an absent or retired joint and
`InvalidValue` for invalid motor numbers, preserving existing state and work.

The joint direction is unchanged: child angular velocity minus parent angular
velocity, projected along the declared world axis carried by the parent-local
joint frame. Construction pose is angle zero. Impulse joints retain continuous
angle unwrapping; reduced joints retain their native continuous coordinate.
A robot with +X forward and both wheel axes +Y uses positive wheel targets to
roll forward. Opposite declared axes require an explicit controller sign mapping.

Rapier 0.36.1's maintained `GenericJoint` angular-X motor drives both existing
impulse and reduced-coordinate handles. `set_motor_velocity(target, 1.0)` selects
zero position stiffness and damping one, with explicit `AccelerationBased` motor
semantics: response coefficients scale with inertia. `set_motor_max_force` is a
separate torque cap. Pinned `JointMotor::motor_params/max_impulse` converts that
cap to `max_force * dt`; the impulse and multibody motor builders use symmetric
impulse bounds. This preserves a Nm cap even with the acceleration-based response.
The application adds no solver, explicit effort controller, body-velocity edit,
teleport, hidden step, or new dependency.

`Some` replaces a persistent target, including while paused. Zero target applies
bounded braking; zero maximum force applies no motor torque. `None` disables the
motor axis and discards its warm-start impulse. The motor is native joint-owned
runtime state, excluded from immutable construction/initial-state records. Both
ordinary reset and captured initial restoration clear all motors, including a
motor active at capture. Joint/body retirement removes its motor. Setting targets
or validating them never changes public pose, velocity, joint sample, IDs, or
queued wrench/effort. A zero-duration step has the same unchanged-state contract.

Before any advancing-step mutation, target velocity times dt and torque cap times
dt must remain finite. An unrepresentable product returns `InvalidValue` while
retaining admitted targets and pending work. Existing effort guards, including
moving-parent effort refusal, remain unchanged. Native motors provide the parent
reaction that the existing world-couple effort implementation does not qualify.

The controller's configured maximum wheel speed bounds its **target**, not physical
speed under external impulses or contact loads. The backend does not impose a
hard speed limit or angle limit. Contact, slip, motor response and measured body
motion remain authoritative solver outcomes.

Authored public controls cover signed continuous motion, first-step torque caps,
disable/coast and zero-target braking, zero-force actuation, invalid replacement
and ID/work preservation, ordinary/captured reset, removal, and actual moving-parent
rolling contact versus the same motor-driven assembly without a floor. Private
controls inspect both maintained native handle kinds and a nonidentity frame,
parent reaction, disabled warm-start state, and both step-product refusals with
one-shot wrench/effort conservation. Default-backend capability assertions are
additive. All previous assertions remain unchanged. These controls and the new
runtime implementation are unexecuted pending whole-M8 source review and one
consolidated required verification; no runtime or numerical success is claimed.
