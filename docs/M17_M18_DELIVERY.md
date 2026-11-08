<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M17 magnetometer and M18 joint-state publication

The owner directs continuous bounded delivery and parallel independent work under
D30–D32. This source wave is isolated from frozen M15/M16 candidate `5f53c92e`.
That base is now qualified Main after exact hosted run `42446229290`, independent
audits, native pressure and actual OBJ browser proofs. Starting these new sources
transfers no acceptance; the `m15-m16-features` receipt records the base evidence.
Each increment has a budget of at most five working days, including review,
regressions, documentation and all mandatory qualification.

## M17: ideal magnetometer

Admit one loaded Rapier magnetometer with a finite constant SDF world magnetic
field, including an explicit nondefault or zero vector. Match pinned native
`gz-sensors` `8ddf4c17`: inverse sensor world rotation applied to the world field,
in Tesla, with canonical `gz.msgs.Magnetometer` timestamps, sequence and frame.
Use resolved sensor poses, including link and sensor rotations. Translation alone
must not change the field. Preserve existing cadence, demand, queued-generation,
pause, loaded reset, removal and absent-family ownership contracts.

Pinned `gz-sim` `71c60db3` retains the configured field when no spherical-coordinate
lookup is available. Its geographic branch can replace that field and has separate
Gauss/NED configuration; this increment does not claim that branch. Reject
spherical coordinates, noise, unsupported metadata/configuration and engines.
Only a fully admitted typed magnetometer may permit a nondefault world field;
preserve the existing rejection for worlds without that consumer. Validate the
complete configuration before creating world state or declarations.

Required evidence: asymmetric default/explicit/zero fields; nonzero body and
sensor rotations; translation invariance; finite/domain and atomic-refusal cases;
actual pinned native model and moving-pose observations; canonical wire values,
frame/time/sequence; cadence, demand, pause/reset/removal and stale-job controls.
Native geographic warnings and limitations remain disclosed. No noise model,
geographic field calculation, sensor UI, new framework or backend expansion.

## M18: joint-state publication

Admit the built-in configuration corresponding to native `JointStatePublisher`
for one top-level model and existing admitted single-axis revolute joints. Publish
canonical `gz.msgs.Model` with actual model/joint identities, poses and supported
axis metadata, position and velocity. Reuse existing backend joint samples and
canonical schemas; do not add a solver, force application or controller.
Do not invent unavailable force values or claim complete native packet parity.

Match the pinned native topic, known joint-name filtering/deduplication and
cadence contract. Its positive-rate guard suppresses only `0 < elapsed < period`;
equal simulation time and rewind are not sensor-style pause suppression. Record
the exact public Rust contract and negative cases before implementing it. Reject
unsupported plugin options, unknown requested joints, multiaxis joints, nested
model expansion and dynamic joint creation atomically.

Required evidence: actual pinned native packets and same-backend component
observations; topic/filter/order, identities and every emitted metadata/value
field; default and positive-rate cadence including equal time, pause and reset;
finite values, removal/reused IDs, weak handles and endpoint conflicts. A new
browser widget or repeat of unchanged accepted visual qualification is not an
exit criterion; actual canonical publication from a moving joint is required.

## Ownership and cadence

- SDF owner: new Sensors/Scene magnetometer leaves, meaningful private/public
  regressions, fixture, native observer and magnetometer guide.
- Asset owner: new Scene joint-state publisher leaf, meaningful regressions,
  fixture, native observer and joint-state guide.
- Messages owner: new Server formatter/IO leaves and regressions for both phases;
  leased shared Server integration after current M15/M16 evidence closeout.
- Root: shared Scene modules/admission/reset/lifecycle dispatch, manifests,
  coexistence and central tests, canonical planning, integration and publication.
  Propose hooks across ownership boundaries; do not edit another owner's file.

Complete implementation, regression source, docs and independent source review;
batch findings before one consolidated normal gate. Reuse unchanged evidence
where permitted. Keep strict separate workspace/per-crate/actual-parent coverage,
full hooks and exact hosted CI. Root serializes heavy local work. No acceptance
or publication before all phase-specific proofs, independent audits and qualified
Main promotion. Preserve frozen trees, all genuine failures and Fuel ownership.

The final candidate records target M17/M18 `complete` status and no active
successor. Those flags take effect only at qualified Main promotion after all
actual proofs and mandatory gates; they do not label source authoring accepted.
The live tracker remains in progress until that boundary. Actual generated
commit/run IDs, timestamps and proof hashes belong in the existing immutable
release/promotion receipt and owned status, avoiding a second metadata-only gate.
After acceptance, immediately select and start the next useful bounded wave.
