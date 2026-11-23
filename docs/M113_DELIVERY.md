<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M113: DiffDrive jerk-limited commands

This phase depends on M112 and adds the six corresponding finite `min/max_jerk`,
`min/max_linear_jerk` and `min/max_angular_jerk` fields. Generic limits precede
axis-specific overrides; each resulting interval must be ordered. Existing
velocity-only and acceleration-only behavior stays within its selected domain.

Retained native ordering is jerk, then acceleration, then velocity. With the
current step duration `dt`, latest command `v0` and preceding command `v1`, use
`a_previous = (v0 - v1) / dt`,
`j = ((request - v0) / dt - a_previous) / dt`, clamp `j`, and reconstruct
`v = v0 + (a_previous + j_limited * dt) * dt`. There is no half coefficient.
Both differences use this step's duration, even after unequal earlier durations.
The existing exact zero-duration pause behavior stays; no native near-zero-time
approximation is copied. Every intermediate must be finite before mutation.

The two command histories belong to each of at most eight original loaded
owners. They advance only after complete multi-owner/native-target preflight and
application. Reset initializes both to zero. Request replacement, pause, removal,
loaded reset and immutable metadata checks follow M112. A zero request can take
time to decelerate while jerk reduces prior acceleration. Command limits describe
command shaping, not a physical acceleration or no-slip traction guarantee.

Eight new regression declarations cover typed override/refusal matrices; literal
independent discrete-jerk tables with unequal durations; combined limit ordering;
measured native motion against integer-difference command twins for one/eight
owners; paused and reset histories; late-owner faults; wheel saturation versus
measured velocity; and malformed/overflow guards. Native references are retained
`gz-sim` DiffDrive.cc and `gz-math` SpeedLimiter.cc, pinned in the selection record;
no new native reference execution or parity result is claimed at Source freeze.

The budget is at most five working days including independent review and all
mandatory gates. Implementation and regression Source must be reviewed with M112
and the combined candidate before the one final verification cycle. The current
SSD execution hold remains; no current run, commit, qualification or acceptance
is inferred from this document.
