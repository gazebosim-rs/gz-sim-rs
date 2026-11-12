<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M46: rotated ground-truth odometry frames

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

Select one bounded increment, budgeted at most five working days including
review and mandatory checks: finite `rpy_offset` and `rpy_global_offset` for the
existing noiseless OdometryPublisher in dimensions 2 and 3. This lets the same
physical robot report odometry in a rotated local or global coordinate frame.
Existing translation offsets may be combined with these rotations.

Development uses clean local M45 commit
`e66c21e739089670b24e23b91fcd1363935b9a39`, whose full normal hook and independent
local audit passed. It is provisional, not accepted. Accepted Main remains
`73e03b9f741689b1538898bde96bf975a49f068e`; M43/M44 hosted CI failed snapshot
setup before product checks. M45 and M46 cannot inherit predecessor acceptance.
The frozen M43–M45 candidates and all evidence remain unchanged.

Under D31/D32, Source work and reviewed provisional local qualification may
proceed while hosted closeout is blocked. Complete implementation, regression
Source, documentation and independent review before one consolidated local
verification pass. Preserve every normal job, strict workspace/per-crate and
actual-parent coverage gate. New native evidence requires a reviewed bounded
procedure before invocation. Actual accepted predecessor, exact hosted success,
independent official evidence and normal Main/readbacks precede publication and
acceptance. If the accepted parent differs, review the integration and affected
checks; no result transfers automatically. No CI retry or snapshot workaround is
authorized by this selection.

## Behavior

Follow pinned Sim `71c60db` OdometryPublisher Configure/Update and pinned Math
Quaternion/Pose3: `globalOffset.Inverse() * worldPose * localOffset`, evaluated
left to right. Build each offset rotation with the native Euler constructor and
normalization order. Pose inversion rotates the negated global translation by
the inverse quaternion; Pose multiplication keeps its existing scalar order.
Avoid a generic transform framework, glam Euler shortcut or changed tolerance.

All omitted and wholly zero RPY vectors, including signed-zero spellings, keep
the exact old translation-only path. Normalize each wholly zero vector to the
legacy positive-zero configuration independently. Preserve every component bit
of a nonzero vector for immutable ownership identity. Existing constructors and
translation getters remain compatible. Extend GroundTruthOdometryOffsets with
`with_rotations(self, local: [f64; 3], global: [f64; 3]) -> Result<Self,
GroundTruthOdometryError>`, plus `local_rotation_rpy()` and
`global_rotation_rpy()` returning `[f64; 3]`. Store private local/global RPY bit
arrays; use the existing Offset error for nonfinite arguments.

Preserve default Two, explicit Two and Three semantics. In Two, the full
composed orientation and native planar velocity calculation remain in force;
do not flatten the offset rotation before native pose composition. Keep startup,
microsecond floor, ten-value history, history-before-throttle, pause, demand,
sequence, headers and all three canonical streams unchanged. No new noise,
backend, topology, transport, dependency, viewer or package behavior is selected.

Malformed, duplicate, nonfinite and wrong-length input remains refused before
mutation. Finite composition overflow returns the existing typed arithmetic
fault before history/publication changes. Live configuration mutation faults
and clears stale samples. Healthy loaded reset restores exact configuration,
public identities and empty histories; terminal fault reset remains refused.

## Regression and native proof

Inventory each old nonzero-RPY negative before editing. Each exact newly admitted
input needs an independently reviewed positive load, real force/torque motion,
independent frame geometry and healthy reset. Retain malformed/nonfinite and all
unrelated negative controls; preserve the old M34/M38/M39 native fixtures and
their existing assertions unchanged.

The exact old finite negative inputs are private parser local `(0,0,1)` and
global `(1,0,0)` under omitted/default Two, and public local `(0,0,0.1)` and
global `(0.1,0,0)` under the existing Three plus XYZ fixture. Cover all four
exact inputs as stronger positives. Replace only those negative rows with new
malformed/nonfinite rotation negatives after independent disposition review.

Add local, global and combined rotations, each axis, asymmetric mixed angles,
translation/rotation combinations, both dimensions, omitted dimensions,
independent zero normalization and mixed signed-zero identity controls. Compare
unchanged physical motion across differently configured peers. Distinguish
filtered pose-derived Twist from backend Velocity. Test stale configuration,
pause, due/throttled history, removal, healthy reset and terminal refusal.

Acquire fresh finite observations from the existing original OdometryPublisher
DSO using its maintained system-only ECM helper recipe. Reuse exact original
providers; do not rebuild the SDK or run native Physics. Capture requested and
effective XYZ/RPY, dimensions, period, MODEL-local input and all canonical bytes.
Apply legacy zero-vector normalization before native configuration and disclose
that scope. Convert the identity-parent MODEL pose to WORLD exactly once in
replay. New observations and literal extraction require independent review
before tracked fixture insertion. No fabricated expected output or old-fixture
relabeling. Actual counts come from the finished case matrix.

The live C++ peer must consume all three canonical streams under real force and
torque, and verify complete fields, covariance, demand, pause, loaded reset,
collisions, removal and owned shutdown. No pose or velocity teleportation.
The new finite development example is not a distribution refresh. M40 remains
Latest, and the unchanged M44 package keeps its actual build lineage.

## Ownership

Light: Scene odometry production leaf (including its private test module), new
Scene regression leaf, new example and feature guide. Root: old-negative disposition, shared registration, normal
qualifier integration, planning/status and serial execution. Messages: new
Server regression/private peer leaves, native helper and finite case design.
SDF: independent whole contract/Source review. Shared files have one writer.
No Cargo, lock, workflow, Fuel, renderer, launcher or SDK edit is selected.

Independent review has cleared the contract, focused offsets API and all four
old-negative dispositions. Source implementation is integrated. The original-system
reference and independent native/literal audits have now passed for 30 modes, 32 configurations, 1,020 calls and 858 triples (2,574 packets).
The actual fixture and reviewed consumers are integrated. Full integration review
and consolidated Rust qualification remain pending. This file claims no M46 Rust
pass, hosted result, publication or acceptance.
