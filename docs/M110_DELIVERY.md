<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M110 — tight curved near-plane admission

This lane removes conservative enclosing-box false refusals for curved
primitives that are wholly beyond the calibrated near plane. It does not admit
actual tangent, intersecting or behind-near geometry, and does not implement
native negative-infinity near-range sensor semantics.

## Finite support rule

The legacy box-only validator remains unchanged. For a curved primitive, express
the camera-forward axis in the shape's local frame as n, and the shape center
in the camera frame as c. The closest axial coordinate is c.x minus support:

- sphere or ellipsoid: length(radii componentwise multiplied by n);
- local-Z cylinder: radius times hypot(n.x,n.y) plus half-length times abs(n.z);
- local-Z capsule: radius times length(n) plus half-segment-length times abs(n.z).

The ellipsoid formula includes the sphere specialization. These are supports of
the actual surfaces, not the depth surface of a proxy box. Pose/origin conversion,
f64/f32 dimensions and every derived value remain finite; segment lengths and
radii retain their strictly positive domain. Admission requires the closest
coordinate strictly greater than the near distance. Equality is refused. No
caller pose or quaternion is repaired or normalized by this lane.

This preflight precedes existing primitive installation, so any failure leaves
assets, entities, stamp, cancellation and prior depth mode unchanged. All normal
projection, 64×64 output, depth attachment, readback and lifecycle code remains.
The existing typed near-clip error is retained for compatibility.

## Regression Source and acceptance

The M110 declarations in renderer `tests/capsule_depth.rs` demonstrate actual
finite depth for sphere, ellipsoid, cylinder and capsule cases whose enclosing
rotated boxes cross the near plane. Independent scalar supports choose the
fixtures. The same scene under common noncommuting camera/world rotation and a
large translated origin preserves declared depth samples within the existing
f32 raster tolerance. Separate tangent, intersecting, behind and malformed
controls keep the complete prior depth frame unchanged. Legacy box refusal is
explicitly retained on the same enclosing bounds. M109's independent capsule
surface-depth controls are separate.

Owned production paths are renderer `src/depth.rs` and `src/gpu.rs`, sequential
with the same M109 owner; no backend, Scene loader, public calibration or native
sensor seam changes. Root retains existing refusal controls and all old fixture
bodies. The lane budget is at most five working days, including review and
mandatory verification.

No current test, lint, formatter, GPU, worker or native result is claimed.
The exact reviewed M102/M103 Source foundation is unverified. Root must finish
the combined implementation/regression/documents/review batch, establish fresh
resource admission, then run one consolidated required verification cycle.
Local/hosted and original-artifact evidence must qualify this exact candidate
before acceptance or Main promotion; ancestor passes do not transfer.
