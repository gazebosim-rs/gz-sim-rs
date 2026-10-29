<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Body-to-body revolute primitive

The shared dynamic backend contract adds `add_revolute(parent, child, anchor,
axis)`. Rapier connects the registered bodies with its existing full-f64 revolute
constraint. The child is dynamic; the distinct parent may be fixed or dynamic.
The world-space anchor and nonzero axis are converted into each body's local
frame at construction, whose joint angle is zero. Invalid endpoints or derived
frames leave bodies, constraints and the joint-ID allocator unchanged.

Joint samples report continuous relative angle and relative angular velocity
along the current parent-frame axis. They do not report the child's absolute
rotation or speed. Both endpoint rotations remain within the retained per-step
caps, so existing nearest-angle unwrapping also bounds relative rotation.

Initial/prepared reset captures both endpoints and local frames. Removing either
body retires its connected joint records; removing a joint releases unconstrained
body activation. World-anchored and body-anchored constraints can coexist. A
world anchor is allocated only for world joints and retired when none remain.

DART and MuJoCo currently return explicit `Unsupported` before argument validation
for this new capability; their existing world-revolute behavior is unchanged.
Tests use actual native backends to verify rejection leaves both bodies unchanged.
TPE has no dynamic-joint support. There is no world-anchor fallback.

This is a programmatic backend primitive. The SDF scene loader still supports its
previous single world-anchored revolute topology. General SDF link trees, native
body-joint adapters, loop/collision policies, limits, motors and full per-backend
conformance remain separate P3 work. No upstream case is retired and no new
performance or native cross-engine numerical-parity result is claimed.

The new Rapier tests exercise moving/rotated parents, a hinge axis rotating in
world space with angle/velocity consistency, relative complete turns,
a fixed-parent pendulum, mixed world/body constraints, endpoint removal,
prepared reset, ID retirement and atomic invalid/overflowing-frame rejection.
The existing world-joint/native-reference tests remain unchanged. This candidate
requires independent review and the mandatory strict staged/hosted gates before
acceptance.
