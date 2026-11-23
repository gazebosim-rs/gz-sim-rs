<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M106: balanced effort on moving-parent revolute joints

This is unqualified Source in the selected M104–M113 candidate. Its genuine Git
parent is `08a0eeae`; its copied M102/M103 foundation is reviewed Source only.
The phase budget is at most five working days including review and mandatory
verification. No result or acceptance transfers from a parent.

The only production lease is Rapier `src/joints.rs`. Generalized effort on an
existing dynamic parent/child revolute is an internal couple: compose the joint
frame with the measured current parent rotation, apply positive world-axis
torque to the child and its negative to the dynamic parent. Accumulate every
endpoint in a temporary map and check the complete sum against existing body
wrenches before changing physics or consuming pending efforts. World and static
parents keep the existing child-only response. Scalar, ID, topology, step and
combined-wrench checks remain authoritative. No new joint, motor, servo,
controller or solver domain is introduced.

The public Scene regression leaf exercises native APIs in both retained harness
profiles: impulse and reduced joints; isolated principal-axis responses;
measured rotating axes; positive relative work and conserved angular momentum
for centered isotropic bodies sharing a pivot; shared-parent accumulation;
nonfinite and parent-accumulator overflow rejection with pending repair; both
native resets, retirement and unchanged motor/servo policy. A loaded
ApplyJointForce pair independently joins persistent and one-step commands to
committed joint feedback, pause, reset, removal and reconstruction. No oracle
substitutes ideal motor convergence or a nonprincipal Euler approximation for
measured physics. General off-center momentum includes orbital terms and is
not inferred from these concentric controls.

Root owns retained same-input effort refusals in native motor/servo/effort and
Scene joint-force/publication tests. Only the effort expectations affected by
the selected dynamic-parent extension may change; native actuation eligibility
and other invalid inputs remain unchanged. All old test bodies stay untouched
in the author lane.

Finish Source, meaningful regressions, documentation and independent review,
then run the one combined mandatory cycle. Full staged hook, strict local and
hosted coverage, independent retained local DATA and exact original-artifact
DATA remain required before acceptance. No tests, formatter, build, probes or
new native reference campaign have run during authoring. Heavy operations stay
held for fresh resource admission; old targets, evidence, Fuel and the live
no-expiry demo remain protected.
