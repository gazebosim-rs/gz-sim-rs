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
additive. All previous assertions remain unchanged. The controls passed the
required M8 local gate and exact hosted CI41776261368 at `fc967388`, with
independent source and retained-data review. See M8_ACCEPTANCE.md for exact
coverage, provenance and limits; no general native drivetrain parity is claimed.

## M9 accepted opt-in native velocity servo

`validate_joint_velocity_servo` and `set_joint_velocity_servo` add an explicit
alternative using the same `JointVelocityMotor` target/torque DTO. The accepted
M8 methods and factor-one compliant motor policy remain unchanged. Other
backends return typed `Unsupported` for this new capability before validating
the joint ID or numbers. Rapier reuses the existing validation, replacement and
disable path, then infallibly selects its maintained angular-X motor with
explicit `AccelerationBased`, zero stiffness and zero damping. There is no
custom constraint, solver, position motor, gain search or dependency change.

This choice follows pinned Rapier 0.36.1 source: `MotorModel::combine_coefficients`
uses the maintained reciprocal-zero convention, giving zero ERP and compliance
coefficients at those exact spring parameters. Both impulse and reduced motor
builders retain the enabled motor axis and symmetric `max_force * dt` impulse
bounds. Thus the native solver attempts the target velocity without the old
factor-one compliance, while retaining the same finite Nm cap. Finite torque
can still permit tracking error, overshoot and externally driven overspeed;
this is not a hard physical velocity or position limit. It is not a native
controller parity or convergence proof.

Validation remains read-only, and refusals preserve the old mode, target,
warm-start state, public physics state, IDs and pending work. `None` disables
and clears warm-start state. Both resets clear the opt-in mode even if a servo
was active when the initial state was captured. Joint/body removal retires it.
The existing pre-step finite target-times-dt and torque-times-dt guards apply
before construction closes or pending effort/wrenches can be consumed. Calls
to either explicit motor API replace the one native motor on that joint; using
the old API again restores its accepted factor-one policy.

The first M9 functional gate stopped at the second continuous position target:
12.960774009241305 radians versus 12.866370614359173, beyond the unchanged
0.01-radian assertion after 30,000 steps. The printed second joint velocity was
0.8164674376912187 versus requested 0.5; its following assertion was not reached.
The first target's position and measured-velocity assertions passed. Source
establishes a policy difference between the fixed compliant motor and the native
SERVO command route, not the sole cause of that failure. The failed packet is
retained, and the exact convergence assertions, deadlines, coordinates, target
bounds and torque caps remain unchanged for qualification of this correction.

Three new public controls cover both handle kinds' real velocity tracking,
torque cap, coast/brake/reversal, zero-force response, invalid-command/state/ID
and pending-work conservation, ordinary/captured reset and removal. Two private
controls check native zero coefficients and nonidentity parent reaction,
mode/warm-start preservation on invalid replacement, accepted-mode restoration,
and atomic overflow refusal. A default-capability control covers both methods.
All six passed in the affected normal gate, but the whole feature still failed:
the same Scene control now stopped at its first 0.7-radian target after 30,000
steps, with position 0.5764025648558753 and velocity -0.5339801774600306. The
printed tip velocity was 0.49978922837343875. That failed packet remains separate;
passing the focused native controls did not qualify the coupled controller.

## M9 accepted reduced continuous damped position request

`JointPositionServo { target_position, max_velocity, max_force }` and
`validate_joint_position_servo` / `set_joint_position_servo` explicitly prepare
for the next supplied `Duration`. The caller must use the actual upcoming step
duration and reissue before each advancing step. The caller retains the ultimate
target; the backend owns only the maintained motor and its prepared waypoint.
`Some` is supported only on reduced revolute handles, whose native coordinate is
continuous. Impulse position motors retain shortest-angle/quaternion semantics
and are refused by this API. Other engines return typed `Unsupported` before
numeric or identifier validation. `None` disables any registered joint's motor
regardless of duration or handle kind.

The fixed acceleration-based spring uses stiffness100 s^-2 and damping20 s^-1:
a nominal critically damped isolated response with10rad/s natural frequency,
chosen as a documented design policy, not from a gain search. No new SDF PID
gain fields or manual solver are introduced. Preparation reads the actual
reduced coordinate and uses public `MotorModel::combine_coefficients`. Rapier
can divide the supplied duration into solver substeps; its maintained zero-dt
ERP limit bounds all positive-substep ERPs for these positive coefficients.
The waypoint clamps continuous error by `max_velocity / ERP_limit`. Native
target velocity remains zero and the torque cap remains the same finite Nm.

The prepared start-of-step position bias is checked against the target-speed
bound using represented binary64 values. If rounding initially puts a waypoint
outside that bound, it moves one representable value toward the current
coordinate and is rechecked. Nonfinite values, zero/negative speed, zero duration,
overflowed products/error or an unrepresentable nonzero waypoint are refused
before any native mutation. Supplied-duration coefficients are checked too.
Native coordinates can evolve during solver substeps; this bounds the prepared
start-of-step request, not every later internal bias or physical speed under
loads. It creates no physical position stop.

All fallible preparation is shared by validation and setting. Refusal preserves
the previous motor mode/target/warm start, state, IDs and pending work. The native
reduced motor then uses its existing continuous `curr_pos - target_pos` spring
constraint and symmetric torque impulse cap. Old M8 factor-one methods remain
unchanged; resetting/removing native joints still clears actuator state. M9's
Scene construction opts its two joints into the already-supported reduced path;
other admitted joint construction policies remain unchanged.

Three public controls and two private controls plus a default-capability control
are authored for positive/negative multi-turn tracking and body orientation,
torque cap/coast/reset/removal, impulse refusal, invalid-command/pending-work
conservation, coefficient/rounding/representability boundaries, nonidentity
parent reaction, and mode/warm-start ownership. These controls and the unchanged
coupled chain's 0.01-radian / 30,000-step assertions passed the normal M9 gate and
exact CI41817794153 at `0f0f031e`, with independent source/local/official and actual
browser reviews. Both prior failures and all original assertions remain intact.
Nominal damping is not a general articulated convergence guarantee or native
parity claim; see M9_ACCEPTANCE.md for the accepted subset and exact evidence.
