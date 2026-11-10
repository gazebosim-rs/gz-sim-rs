<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M38: noiseless 3D ground-truth odometry

Selected under D30–D32 on 2026-11-09, over accepted Main
`6d2d53209174e3398b79909553e4fc5fd7fe05e8`. Budget: at most five working
days including independent review and every mandatory check. M1–M37 are
accepted; [M37 is the installed Latest](M36_M37_ACCEPTANCE.md). M38 has no
acceptance credit yet. The reviewed implementation and regression Source are
integrated. Fresh original-system capture and independent native/literal DATA
passed: 15 modes, 510 input calls, 429 triples and 1,287 canonical packets.
The fixture and both replay controls are inserted; Rust compilation/replay,
full normal/strict coverage, exact hosted CI and independent qualification DATA
remain pending. See [the guide](M38_ODOMETRY_3D.md) for reproduction provenance.

## Bounded result and dependencies

Add explicit `dimensions=3` to the existing noiseless ground-truth odometry
built-in for its admitted top-level Rapier models. Preserve missing/explicit
`dimensions=2` behavior exactly, including existing arithmetic, signed-zero
behavior, 301 native inputs, 221 observations and 663 canonical comparisons.
Keep all existing topic, frame, rate, offset, noise, topology and backend bounds.

The useful result is actual vertical position, body-frame linear XYZ and body
angular XYZ derived from acquired pose history. The existing Server already
encodes all three components in Odometry, OdometryWithCovariance and Pose_V;
reuse those streams and the upstream Zenoh transport unchanged. This is pose-
derived, filtered ground truth, not an encoder integrator or instantaneous
backend Velocity. Covariance remains the existing complete zero matrices.

Review the pinned original OdometryPublisher system and Math quaternion/vector/
rolling-mean code before runtime edits. Its original system Source has SHA-256
`0c4ccd7cff05a18bbe0c88b9a3769bb2c0d3243ee6677dfba085c5dde0500f77`.
Agree the typed 2D/3D mode, reconstruction identity and exact 3D arithmetic with
the Scene and Server owners before implementation. Do not change the existing
2D branch merely for symmetry with the new mode.

The agreed interface for review is a public Copy/Eq
`GroundTruthOdometryDimensions::{Two, Three}` stored in
`GroundTruthOdometryPublication.dimensions`. Missing/explicit 2 selects Two;
explicit 3 selects Three. Existing publication identity equality then includes
the mode. The Server must keep its four planar-zero constraints for Two and
admit all finite XYZ pose/twist components for Three under the same common
quaternion/message bounds. There is no wire field or schema change. Root owns
the shared enum export and any non-leaf construction or reconstruction seam.

For Three, review native current-body inverse rotation of the complete
displacement before componentwise division by elapsed acquisition time, then
three linear and three angular oldest-first ten-sample means. Keep native
quaternion shortest-sign/axis-angle/body-rotation order in a separate 3D path;
do not refactor the accepted 2D arithmetic. Preserve native zero-offset pose
composition, first unpaused initialization, near-zero elapsed handling and
history updates before publication throttling. The original dimensions-3
negative case is deliberately superseded by positive 3D controls; invalid
0/1/4/-1/text/duplicate/scalar-smuggling cases remain negative. This disposition
and the finite native case plan require independent review before runtime edits.

## Parallel ownership and cadence

All three worktrees start at that exact accepted Main. Root integrates one
candidate and owns shared files, configuration/reconstruction seams, peer
registration, CLI/profile/retention contracts, planning and publication.

- Light: Scene odometry implementation and private tests;
  `crates/gz-sim-scene/tests/odometry_3d.rs`,
  `examples/m38_odometry_3d.sdf` and `docs/M38_ODOMETRY_3D.md`.
- Messages: the Server odometry leaf only if an actual new binding is needed;
  `crates/gz-sim-server/tests/odometry_3d.rs` and its C++ canonical peer;
  the scoped original-system native capture helper, case inputs and typed
  fixture. Keep existing packet layouts and old fixtures unchanged.
- SDF: independent review of the complete native/arithmetic/API contract and
  implementation, regression integrity, lifecycle and evidence.

Scene and Server/native Source work may proceed in parallel after contract
review. Complete implementation, regression source, guides and independent
review; batch findings before one consolidated verification pass. Root alone
runs heavy work. Preserve existing evidence instead of repeating unchanged
workloads. Use the maintained real gates; do not add a new process framework,
per-edit tests/lint or administrative qualification cycles.

## Reference and acceptance

Before runtime edits, review finite original-system cases for stationary
nonzero 3D poses, every translation/angular axis, tilted bodies, quaternion sign
selection, startup and ten-sample filter boundaries, cadence and pause/reset.
Derive exact row/message counts from the actual finite inputs. Acquire fresh
3D reference values through the maintained original-system DSO/ECM recipe,
binding the generating program, command and actual mapped libraries. No native
Physics trajectory claim follows from an ECM pose history. Commit the typed
fixture and independently audit the actual capture; preserve all old controls.

Require meaningful loaded Scene controls for full pose/twist, history and
current-body rotation; dimension identity through reconstruction/reset;
demand/absence, equal-time/pause, removal and retirement; invalid/duplicate
dimensions, nonfinite arithmetic and stale configuration. All old noise/offset/
backend/topology refusals remain. Any new floating-point bound requires native
and Source justification; no blanket epsilon or generic zero clamp is selected.

A real transport-only C++ peer must consume all three canonical outputs for
actual Rapier motion, keyed by acquisition stamp, and compare complete pose,
twist, headers and covariance to Scene samples. Reuse a Clock/FIFO fence only
where the actual existing queue order supports it; no general cross-publisher
barrier is claimed. Use actual force/torque motion, never pose or velocity
teleportation. Preserve explicit loaded-reset and wire-epoch limitations.

Acceptance requires independent complete Source review, the full normal hook,
unchanged strict workspace/per-crate/actual-parent line and region coverage,
exact hosted CI and independent local/official DATA before normal qualified
Main promotion. Old runs supply no M38 acceptance. This source increment needs
no package/browser refresh; M37 remains installed Latest. After acceptance,
select the next useful retained increment without waiting for owner approval.

## Exclusions

No noise/offset expansion, new backend, solver, schema, transport/runtime or
packaging framework, dynamic plugins, generalized publisher machinery, viewer
rebuild, Fuel changes, performance claims or broad compatibility campaign.
A newly required dependency or broader admission change must be re-scoped
before implementation rather than absorbed into this bounded slice.
