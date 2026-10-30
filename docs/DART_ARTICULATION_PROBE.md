<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# DART body-to-body revolute investigation

The current DART adapter supports world hinges, not body-to-body revolutes.
The small native controls in `crates/gz-physics-dart/tests/native_articulation.rs`
exercise the installed DART 6.13.2 library directly. They establish native
topology behavior; they are not adapter, Gazebo plugin, collision-safety or
double-pendulum parity tests. They use no collision shapes, no contact or finite
joint limits and do not support a performance claim.

- A moving free parent and child merge into a seven-DOF skeleton. The original
  child skeleton remains registered and empty until removed from the World.
  For compatible input twists, poses and COM velocities survive attachment,
  the anchor stays coincident through 1000 steps, and restoring saved generalized
  positions and velocities restores both bodies. Splitting the child into a new
  free skeleton preserves its world pose and twist; the remaining parent steps.
- A two-hinge world-root chain preserves the moving upper and lower twists and
  anchor through 1000 steps and generalized-state restoration. A self-parent or
  descendant cycle returns null and leaves topology and generalized state intact.
- The existing adapter represents a static body with a nonmobile skeleton. A
  child merged into that skeleton also freezes. A welded root in a mobile skeleton
  holds the parent fixed while allowing the child joint to move. Removing that
  merged skeleton and its final owner releases it and both bodies.
- A free root and two hinges have eight generalized coordinates. DART reports
  six, seven and eight dependent Jacobian columns for the successive bodies.
  Expanding those columns into skeleton coordinates and solving the mass matrix
  reproduces native point-velocity changes for three unit impulses per body to
  `1e-10`. The translational response trace bounds those tested directional
  responses. This finite example is not a proof for arbitrary inertia, contacts,
  limits, or DART's native solver assertions.

The test compiles and runs the four native controls as isolated subprocesses,
reuses the adapter's build-time DART 6.13.2 prefix checks and include roots, and
requires the same compiled DART and ODE library directories. Each child checks
its actual `libdart` mappings against the compiled DART directory; a mismatched
directory is rejected. The processes have a bounded timeout. The local run used the packaged OSRF
`6.13.2+ds1-1~osrf2~noble` build. In the DART 6.13.2
[`moveBodyNodeTree`](https://github.com/dartsim/dart/blob/v6.13.2/dart/dynamics/Skeleton.cpp)
source, null or wrong-owner bodies can reach assertions, while self-parent and
descendant cycles are refused. The native subprocess controls check the latter;
the bridge must preflight inputs instead of deliberately reaching assertions.
The checksum-pinned OSRF source package has the same relevant topology checks;
its patch series comments out one contact friction-tangent orthogonality assert,
while other contact and impulse ownership assertions remain. This does not
change the selected ODE backend or qualify the remaining assertion domains.

The bridge now stores DART's reference-counted `BodyNodePtr` per slot, so lookup
no longer infers a body from a skeleton root. Current admission still creates
one body per skeleton, and removal still removes that whole skeleton. These
observations do not justify simply calling `moveTo`: velocity and contact-response
code assumes a free body or one-DOF world hinge, and the mixed contact/limit guard
only admits a one-DOF hinge. An empty skeleton also cannot be allowed into the
current zero-DOF velocity maximum check. A safe implementation needs articulated
ID and subtree ownership, atomic topology changes, compatible
velocity admission, lifecycle/reset reconstruction and a qualified multi-DOF
contact/limit boundary. Finite-limit safety, arbitrary articulation, parent or
child removal, world-wrench and impulse behavior remain open until tested through
the actual adapter against native references.
