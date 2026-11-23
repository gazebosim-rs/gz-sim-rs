<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M112: DiffDrive acceleration-limited commands

This phase adds six finite `min/max_acceleration`, `min/max_linear_acceleration`
and `min/max_angular_acceleration` fields to the existing built-in DiffDrive.
Generic bounds apply first; an axis-specific value overrides that side of the
bound. Each resulting interval must be ordered. Unspecified bounds are unlimited.
Positive-only or negative-only derivative intervals retain the native semantics;
therefore a zero requested velocity can still evolve under such an interval.
Velocity bounds retain the existing requirement to include zero.

The private command limiter follows the retained `gz-math` SpeedLimiter source:
compute `(request - previous) / dt`, clamp the acceleration, reconstruct the
velocity, then apply the existing velocity and wheel-speed bounds. Limiters run
only at an advancing PreStep. A finite raw valid request is retained until then;
without temporal fields, the existing immediate velocity-clamp path is preserved.
The histories store limited twist commands before wheel saturation, not measured
wheel/body velocities. Bounded-force native motors, contact and slip determine
physical motion; encoder odometry remains measured and unchanged.

All owners and native motor values are preflighted before applying targets, and
histories commit only after all targets succeed. Arithmetic failures preserve
histories and retain terminal fault behavior. Paused ticks never advance history;
latest-valid command replacement still applies while paused. Zero becomes a
bounded deceleration request. Simulation reset clears command histories; loaded
reset reconstructs them, and removal retires only the selected owner. Immutable
limiter parameters are checked against the original loaded blueprint.

Regression Source includes eight new cases in the private acceleration module:
closed parsing and exact overrides; independently tabulated binary ramps with
reversals/variable dt; legacy default behavior; native encoder/body comparisons
to independently commanded one/eight-owner twins; pause/latest/zero/reset/removal;
late-owner atomic faults; healthy metadata repair; and malformed/overflow input.
The original six temporal-field refusal rows keep their exact inputs and case
identity and become typed parameter positives where selected; other refusals stay.

M112 shares an exclusive three-file DiffDrive lease with dependent M113. The
budget is at most five working days including review and mandatory verification.
The combined M102–M113 candidate must pass its own complete normal hook, strict
coverage, exact hosted CI and independent local/official DATA. These regression
cases are declarations: no current test, lint, formatter or build pass is claimed.
No transport, public API, solver, odometry or paused-step tolerance is changed.
