# Cylinder geometry

Centered SDF cylinders now use the Gazebo local Z axis on both headless engines.
SDF length becomes twice the shared shape's half-height. Rapier's mature cylinder
primitive is rotated within the collider, leaving the link pose, inertial frame
and published physical state unchanged. Derived AABB validation includes this
collider-local rotation. Mass-derived inertia, world impulses, gravity, floor
contact and loaded reset use the same shape. TPE retains native cylinder AABB
contact approximation with half-extents `[radius, radius, length / 2]`; this is
not an exact curved-surface contact solver.

`scripts/probe-cylinder.py --help` reproduces pinned native dimensions/bounds,
eight touching, separated, rotated and static model controls, and the unchanged
`Shape.CylinderShape` original. `planning/reference/cylinder.json` binds source,
compiler, binary, build-cache and loaded-library hashes. Installed dependencies
and build provenance remain qualified. The original mutable Shape API, including
its empty default constructor, is not claimed as a complete Rust port. Physical
construction requires positive finite dimensions, and Rapier also rejects
unrepresentable derived mass properties or bounds before state mutation.

Tests retain the exact old `<cylinder/>` unsupported input as a positive on both
engines, including IMU timestamps, motion, original identity and loaded reset.
The old error/atomicity control now uses unsupported plane geometry with the same
assertions. Analytic inertia checks distinguish the Z axis in two body frames.
The floor test checks the transformed cylinder's lowest support point and contact
normal velocity, allowing physical rocking; it does not assume an upright,
stationary center of mass. Existing numerical tolerances remain unchanged.

Offset/multiple collision shapes, other geometry, general joints, DART/MuJoCo,
complete LOAD/PHY packets and P2 acceptance remain open. No performance or solver
equivalence claim follows from adding this geometry.
