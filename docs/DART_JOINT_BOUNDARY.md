<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# DART mixed angular constraint boundary

The retained SDF defaults include enforced position bounds of ±1e16 radians.
DART can activate these rows predictively, from the current velocity and time
step, even before the stored position reaches a bound. Tiny contacting bodies
can satisfy the contact lever-arm response bound while their angular row has an
arbitrarily larger response. Three admitted profiles aborted the packaged native
Dantzig solver with a zero LDLT pivot: box radii 1e-20 or 1e-29 m, mass 1 kg,
moments proportional to radius squared, two world hinges with different axes,
and generalized efforts of 1 or 1e-10 Nm at 1 ms. The durable subprocess cases in
`crates/gz-physics-dart/tests/joint_bounds.rs` retain those construction profiles.

Before native LCP assembly, the bridge inspects each actual constrained group.
A group containing a contact and an active JointConstraint requires that row's
own skeleton to be an admitted one-DOF world hinge. Its angular unit-impulse
response, 1/M(0,0), must be finite, positive and at most the existing 4e5 ceiling.
Moment and reciprocal must both be finite and positive. This extends the native
coordinate numerical bound to angular rows; angular response has units
1/(kg m²), whereas contact response has units 1/kg. It is not a dimensionless
condition-number argument or a general solver correctness proof.

Global velocity/contact checks retain their priority. A violating group skips
the native solve and returns a distinct typed Native diagnostic through the
existing rollback path. Body positions, velocities and joint samples remain
unchanged; pending efforts and world wrenches survive the refusal. Removing the
ill-conditioned bodies permits retained commands on a surviving normal hinge to
apply once. No state clamp, automatic retry, solver replacement or tolerance
change is introduced. Isolated scalar angular rows continue through native DART,
as do ordinary mixed groups within the inverse-response bound.

The source basis is DART 6.13.2:
[ConstraintSolver](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ConstraintSolver.cpp)
places only active constraints into groups and resets union roots before solving;
[JointConstraint](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/JointConstraint.cpp)
defines predictive position-limit activation and angular unit-impulse response.
The ownership lookup therefore uses each constraint's local root after that
reset, not the group's first participant.

The existing fuzz work/count assertions, normal resting controls and native
force-pulse oracle remain unchanged. Separate author studies cover 48 tiny-body
profiles and 36 active profiles with radii down to 1e-29 m but independently
larger moments (1e-5, 1e-3 and 1 kg m²). These probes qualify selected numerical
domains, not all diagonal ratios, physical accuracy or throughput. Finite user
limits and the separate private solver-conditioning research remain unaccepted.
An independent final source export and reviewer-owned target passed all 91 DART
package tests, including the five new controls and unchanged fuzz/bounds/pulse
assertions. A separate clean accepted baseline with the identical durable test
source independently reproduced all three native assertion terminations. These
negative baseline outcomes are failure evidence, not passing checks. Strict
staged coverage and exact hosted qualification remain acceptance gates.
