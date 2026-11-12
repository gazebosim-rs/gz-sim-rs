<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Rotated ground-truth odometry frames

The second normal attempt passed both required Clippy runs, then ran 402 Python
cases with one failure: the mocked finite-example invocation assertion still
expected 19 rather than the selected 20. The correction keeps an exact 20-run
assertion, the original nineteen-source prefix and every per-source check.
Runtime profiles and feature numeric tests were not reached. Both failed
attempts remain retained; no native/literal evidence is repeated.

The first complete normal attempt stopped at required Clippy before tests or
coverage: `clippy::neg_multiply` rejected `value * -1.0`. The reviewed correction
uses unary `-value`, preserving every admitted finite value and signed-zero bit,
with all subsequent arithmetic and assertions unchanged. The original failed
source tree, packet and logs remain retained. No lint suppression or native/data
replay is selected. The corrected full normal pass is pending separate review.

M46 adds finite local and global Euler-angle offsets to the existing noiseless
OdometryPublisher. Development starts at locally qualified, provisional M45
`e66c21e739089670b24e23b91fcd1363935b9a39`. Accepted Main remains
`73e03b9f741689b1538898bde96bf975a49f068e`; predecessor hosted and publication
closeout is separate. The original-system reference and native/literal audits have passed.
M46 Rust checks, hosted qualification and acceptance remain pending.

`examples/m46_odometry_rotation_offsets.sdf` uses the existing free mass-2 box
with unit diagonal inertia, centered geometry, zero gravity and a 1 ms step.
The initial model pose is `(2,-1,3)` with RPY `(0.2,0.3,0.6)`. The local XYZ
point `(0.4,-0.3,0.2)` has RPY `(0.3,-0.4,0.5)`, and the global origin
`(-0.7,0.5,-0.6)` has RPY `(-0.2,0.25,-0.35)`. ApplyLinkWrench is the existing
configured world system. Output is 50 Hz on `/m46/odometry`,
`/m46/odometry_covariance` and `/m46/transform`, with frames `m46/odom` and
`m46/base`. This finite example is not a package refresh or trajectory proof.

## Configuration and ownership

The existing `xyz_offset` and `xyz_global_offset` remain translation vectors.
`rpy_offset` and `rpy_global_offset` are finite roll, pitch and yaw in radians,
respectively specifying the local reporting basis and global reference basis.
Omission and every wholly zero vector, including mixed signed zeros, keep the
legacy positive-zero identity independently. Every bit of a nonzero RPY vector
is retained, including signed-zero components; numerically equal spellings can
therefore carry distinct ownership metadata.

`GroundTruthOdometryOffsets::from_translations(local, global)` and both existing
translation getters remain compatible. The additive immutable method
`with_rotations(self, local, global)` returns a new value or the existing
`GroundTruthOdometryError::Offset` for nonfinite input. The new
`local_rotation_rpy()` and `global_rotation_rpy()` getters return `[f64;3]`.
All four bit arrays remain private. There is no runtime setter, new wire field,
public transform framework or new error type. Publication metadata equality
covers the angle bits, so replacing live local or global metadata faults instead
of publishing stale samples.

Malformed or duplicate tags, wrong vector lengths, nonfinite values, attributes,
nested elements, CDATA and mixed text remain refused through the existing
configuration error. Nonzero Gaussian noise, unsupported backends and unrelated
plugin/domain refusals remain closed. Finite subnormal or large Euler values
are not assigned a new semantic meaning: they use the native scalar constructor
under the finite input contract.

## Native arithmetic and history

Pinned Sim `71c60db31fddd5c98d2aa689e9a2cccb3548a0a3`,
`src/systems/odometry_publisher/OdometryPublisher.cc`, constructs each offset's
rotation using `Quaternion(Vector3 RPY)` and computes
`globalOffset.Inverse() * worldPose * localOffset` from left to right. Pinned
Math `1316ed334f4d5bda7be59aae07cd6e8ff31cfe13`,
`include/gz/math/Quaternion.hh` and `Pose3.hh`, defines the scalar operations.

Only a nonzero rotation enters the new branch. The old default and complete
translation-only calculations stay unchanged. Euler conversion evaluates the
native half-angle sine/cosine expressions and w,x,y,z normalization sum, with
per-component division. Global pose inversion first inverts the quaternion,
then applies the optimized cross/cross vector rotation to the negated global
translation: scale `uv` by `2*w`, `uuv` by `2`, then add `(vector+uv)+uuv`.
The two pose products use the existing scalar quaternion expression
`q * (vectorQuaternion * q.Inverse())` for translation and Hamilton products
for rotation. They are not replaced by a reassociated matrix or glam shortcut.
Nonfinite inverse or composed poses produce the typed Arithmetic error before
history, means, clock, revision or publication mutation.

The composed WORLD pose enters the unchanged Two or Three estimator. Default
Two still publishes position z=0, linear z=0 and angular x=y=0, while retaining
the full composed quaternion and native planar-yaw calculation. Three retains
all six rolling means. Startup transients, the microsecond floor, quaternion
sign handling, angular threshold, current-body rotation, ten-value averaging,
history before publication throttling, pause and equal-time behavior remain
unchanged. Filtered pose-derived Twist is not instantaneous backend Velocity.

Healthy loaded reset restores the original configuration, stable public IDs and
empty histories. Internal Bevy entities are reacquired. A latched terminal fault
still refuses reset before its observer or scene mutation. Removal clears the
publication and sample; only healthy reconstruction restores the original one.
Scene pose acquisition remains independent of Server demand; Server lifecycle
and canonical stream proof are separate obligations.

## Regression and qualification boundary

The new public Scene leaf covers all four exact former finite refusals. The two
private parser inputs retain the original M34 model and raw plugin content,
omitted dimensions and no XYZ offsets: local `(0,0,1)` and global `(1,0,0)`.
Only the existing world wrench consumer is added for a real physical stimulus.
The two old public inputs use the literal original M39 XML replacement with
Three and its unchanged XYZ/topic/frame fields: local `(0,0,0.1)` and global
`(0.1,0,0)`. Each positive loads, moves under real force and torque, checks
independent frame geometry and performs healthy reset. Root owns the two old
public negative-row dispositions; this leaf replaces only the two private rows
with nonfinite/malformed rotation negatives.

Other controls cover each axis, local/global/combined rotations, both dimensions
and omitted Two, unchanged physics across reporting frames, zero/signed-zero
bit identity, stale metadata, throttled histories, equal time, pause, removal,
healthy reset, terminal refusal and overflow rollback before any means change.
Independent point/basis geometry checks pose at `1e-12`; it does not estimate
native filtered Twist. Old M34/M38/M39 fixtures and replay bodies are preserved.

The original-system capture and independent native/literal audits have passed.
The actual campaign observed 30 modes and 32 configurations: 1,020 calls,
858 odometry triples, 162 suppressed inputs and 2,574 canonical messages.
The exact 3,075,941-byte fixture is
`tests/fixtures/m46_native_odometry_rotation_offsets.rs`, SHA-256
`af48226dc4019d5326e3d3270bef80ae8be9a5f36e5727cb4d23ced273f72874`.
Its compact `.provenance.json` records the generating helper, actual compiler,
compile/run commands and provider hashes, separately from the unchanged raw
literal-generation record retained with the audit. The saved SDK closure has
349 nodes; the actual mapped-provider set has 337. Neither count is substituted
for the other. The capture reused the original system DSO without native Physics
or an SDK rebuild, over accepted capture parent 73e. Rust development remains e66.

MODEL-local capture inputs are composed into WORLD once; live Scene WORLD poses
receive no additional parent transform. Requested and effective zero XYZ/RPY
vectors remain explicit. The reviewed production scalar/dispatcher replay and
captured-value encoder controls are now attached to that actual fixture. They
have not run; native scalar bit agreement and current serializer equivalence
remain Rust-check obligations. The separate live Server proof uses actual
force/torque. No native trajectory, RNG or performance equivalence is claimed.

Complete Source review precedes Root's consolidated ordinary normal hook and
independent local DATA, retaining strict workspace/per-crate/actual-parent
coverage and every retained control. Current local work remains provisional.
Actual accepted predecessor, exact hosted success, independent official DATA,
normal Main and readbacks remain prerequisites for publication or acceptance.
No package, browser, native Physics, noise RNG, geodesy, performance or broad
compatibility credit follows from this change.
