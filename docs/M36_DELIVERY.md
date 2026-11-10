<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M36: canonical wrench commands to ground-truth odometry

Selected under D30–D32 on 2026-11-09, bounded to five working days including
review and mandatory gates. M1–M35 are accepted; M32 remains installed Latest.
M36 source preparation overlapped frozen parent qualification in separate files.
The exact accepted parent is now `1982ce86918df9caf941f8be79b586757bfa77fb`, tree
`a2302c57b1d6ca9939bcbc77ee262c4334f46612`. All reviewed M36 source bytes survived
that parent binding. Its own source, regression controls, docs and affected
formatting are complete with independent review. Final tree/procedure binding
and the complete normal, strict coverage, exact hosted and local/official DATA
gates remain required before M36 acceptance. No parent result qualifies M36.
Preserve all frozen trees, original failures and captures.

Current integration is the atomic [M36/M37 delivery](M36_M37_DELIVERY.md) over
accepted1982. That plan supersedes a separate intermediate M36 hosted/promotion
cycle, preserving every criterion below and requiring new combined full normal,
strict coverage, exact hosted and independent DATA plus all M37 distribution
proofs. The already-running M36-only normal remains provisional historical
evidence and cannot qualify the combined tree.

## Scope and acceptance

One free top-level single-link Rapier body combines the existing configured
ApplyLinkWrench and noiseless 2D OdometryPublisher systems. World `default`,
model `rig`, link `body`, box 0.2 m, mass 2, diagonal inertia 1, centered COM,
zero gravity, 1 ms steps and initial pose `2 -1 3 0 0 0.6`. Retain the three
existing wrench topics. Output topics are `/m36/odometry`,
`/m36/odometry_covariance`, `/m36/transform`; frames `m36/odom`, `m36/base`;
publisher frequency 50 Hz. Resolve actual public IDs, never guessed IDs.

Prove one-shot, additive persistent and clear command behavior using real C++
canonical inputs and actual Rust received/queued counter fences while paused.
Verify force and torque velocity increments using the existing 1e-12 bound;
compare a matched direct-input Scene history without inventing an integrator or
comparing filtered body-frame twist to instantaneous world-axis velocity.
Consume all three real canonical outputs by acquisition stamp and compare them
to the actual Scene sample, including headers, full orientation, zero covariance
and cadence. Preserve the exact native numeric/663-byte controls unchanged.
For reconnect, receive the existing canonical world-clock marker on the same C++
connection at a following non-publication tick before restoring odometry
subscriptions. This fences preceding worker-queue work without a new runtime API;
it is not a general cross-publisher network receipt guarantee.

Cover pause, clear while paused, observer disconnect/reconnect, loaded reset
with fresh leases and no stale work, removal and owned shutdown. A reset may
repeat timestamps; fence/drain actual receipts before changing epoch rather than
inventing a wire epoch. The combined plugin-bearing model is reconstructed only
by loaded reset: existing model-template plugin refusal stays unchanged. Reuse
individual bounds/collision matrices and add combined-owner interactions only.
No production change is intended. An actual integration defect needs a narrow
recorded lease and independent review before editing shared production code.

Acceptance requires complete source/regressions/guide and independent review,
one consolidated full normal hook with strict per-crate/workspace/actual-parent
coverage, the new real canonical peer evidence, exact hosted CI, independent
local/official DATA and qualified Main promotion. Root alone releases heavy jobs.
M32 remains installed Latest; this phase has no package or browser refresh.

## File leases and parallel source work

- Light: new `examples/m36_wrench_odometry.sdf`,
  `crates/gz-sim-scene/tests/wrench_odometry.rs`, and
  `docs/M36_WRENCH_ODOMETRY.md`.
- Messages: new `crates/gz-sim-server/tests/wrench_odometry.rs` and
  `crates/gz-sim-server/tests/native_wrench_odometry.cc`.
- Root: registry and CLI guards, required-peer/retention controls, meaningful
  tooling regression source, shared phase records and integration.
- SDF: independent whole-source review. Authors do not approve their own work.

The reviewed Scene and peer contracts are retained under
`artifacts/m36-delivery/`. Source preparation overlapped parent qualification; the exact accepted parent is
now bound. Root releases all M36 execution only after the complete reviewed batch
and final tree/procedure binding.
No tests or lint run per edit. Batch findings before one consolidated pass.

## Exclusions

No new backend, dependency, solver, schema, telemetry facade, transport/runtime
framework, dynamic plugin, offset math, contact/reaction sensor, performance or
native physics trajectory claim. No native recapture or old accepted manual
qualification replay. Preserve Fuel, paused owners and frozen worktrees.
