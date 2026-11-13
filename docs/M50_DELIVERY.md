<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M50: independent loaded air-pressure sensors

One bounded increment, at most five working days including independent review
and mandatory checks. This selection records scope before implementation under
D30–D32. Independent complete contract review precedes leaf authorship. No M50
capture, implementation, test, coverage, package or acceptance result is claimed.

## Current M50 normal failure and test-only correction

The first complete normal hook failed after 1622.437 seconds at an existing
Server wrench-admission test's five-second wait. Its generic diagnostic cannot
identify which of nine waits failed; the actual cause remains unknown. Both
lint stages and all 403 aggregate Python tests passed earlier. The native-free
and RGB reports, 24 finite examples and new headless pressure numeric/wire
controls are retained partial evidence, not full qualification. Headless coverage
finalization, renderer qualification and independent local DATA were not reached.

The reviewed test-only batch labels every wrench wait and prints existing
counters, ingress statistics, elapsed time and epoch on failure. The existing
post-reset readiness barrier now witnesses all three publishers before they are
used, under the same five-second limit. This strengthens readiness coverage; it
does not establish the earlier failure's cause. Production code, payloads,
numerical assertions, deadlines, test-thread settings and native/literal fixtures
are unchanged. Preserve failed tree `ff22f5c4` and every original result/log.
One complete affected normal retry requires final Source/tree/procedure review;
no separate target trial, native replay or accepted result is supplied.
Earlier checkpoints below are historical.

## Current M50 retained native and literal evidence

The fresh original AirPressure System capture and both independent retained
native and fixed-literal audits returned actual Root zero. The checked-in fixture
retains all 441 sensor WORLD-pose inputs, 42 received pressure/variance packets
and 399 inputs without a matching packet, across twelve configurations and
three modes. The 104 System calls are summary and pinned-method attestations;
missing packets do not expose internal update, noise-draw or deadline state.
Reference altitude remains zero; initial sensor height is not subtracted.

See `planning/reference/m50-multiple-air-pressures.json` for the exact program,
commands, provider inputs and provenance. No SDK/provider was rebuilt or capture
repeated. Scene numeric and Server canonical-wire consumers are being completed
before final independent Source review and one consolidated normal/strict coverage
pass. No M50 Rust, coverage, hosted or release result is claimed. M1–M42 remain
accepted, M40 remains installed Latest, and all release gates remain mandatory.
Earlier checkpoints below are historical.

## Current M50 original-System reference outcome

One reviewed fresh AirPressure System campaign returned actual Root zero in
23.920 seconds, with all 33 child stages successful. It retained three modes,
twelve configurations, 441 actual sensor WORLD-pose input rows and 42 received
canonical pressure packets. The 104 System calls are helper-summary and pinned
method attestations, not 104 fabricated individual raw observations. All current
349 inventory guards, the two link-provider inputs, exact pressure component and
retained original System provider remained bound. No SDK/provider was rebuilt.

Independent native and fixed-literal DATA, observed consumers and the full M50
Rust/normal/coverage checks remain pending. This is controlled original-System
input evidence, not native Physics, RNG/demand equivalence or release acceptance.
Every prior Source checkpoint below is retained as history.

## Exact development basis and release boundary

Use locally qualified M49 `4ccd4f067ecf3438cd9a3cc445cd45d682f6075d`, tree
`b15039730d9926ae49aa3233a0d1b06c4d09a4c2`, over unaccepted M48 `a204d94b`.
M49's full normal hook returned actual zero in 2086.486 seconds. Independent
retained local DATA method2 then returned actual zero: 1791 headless and 1066
renderer profile passes with overlap, 403 aggregate Python tests and 23 finite
examples. All strict workspace, per-crate and actual-parent coverage gates passed.
Changed Scene lines are 16/16 in both profiles; Server is 106/108 headless, not
100 percent. Four fresh raw comparisons from 1811/391/37/33 inputs matched once,
after semantic checks. Native/literal DATA separately binds twelve configurations,
441 inputs and 42 received packets. Preserve both original native failures, the
first normal Clippy failure and the first reader's pre-raw assertion failure.
The named fixture alias correction preserves its entire constant value body;
method2 explicitly verifies the strengthened legacy control in both profiles.

M1–M42 remain accepted at Main `73e03b9f`; M40 remains installed Latest. M43–M49
remain unaccepted. Exact accepted predecessors, hosted success, independent
official DATA and normal Main/readbacks remain release prerequisites. A changed
predecessor requires integration review and affected checks, not credit transfer.
M43/M44 hosted snapshot TLS/select recovery is not demonstrated. No CI retry,
probe, mirror/pin/workflow/signature workaround, new eviction or old accepted
manual qualification replay is released. Keep every frozen tree and Fuel work.

## Closed behavior

Admit zero through eight otherwise supported loaded Rapier air-pressure sensors.
Eight is this increment's resource bound, not an upstream limit. Keep the existing
pressure model, direct-link pose domain, `reference_altitude == 0`, metadata
admission and supported Gaussian/quantized noise domain unchanged. Each sensor
keeps its own world-Z acquisition, schedule, seeded noise stream, demand, frame,
sequence and publication lifetime. Adding a peer must preserve the exact absent
and single-sensor ideal/noisy arithmetic and same-seed paths. Same-seed peer
addition controls must append the peer after the original hierarchy and assert
that the original public sensor ID is unchanged.

Preserve M22's pinned pressure noise behavior, including default zero elapsed
noise application, nonpositive sample handling and the native variance convention
`sqrt(stddev)`. This phase does not replace that convention with stddev squared.
Rust retains its typed sample error/no-canonical-packet behavior for nonpositive
noisy pressure. Upstream can publish its computed nonpositive value: Rust's
rejection is not a native rejection oracle, and no equivalence is claimed there.
Use independently observed pressure/variance values, not reconstructed native
RNG draws. No new reference-altitude option, atmosphere, geodesy, backend, sensor
model, schema, dependency, runtime setter or transport/registry framework.

Require distinct fully effective endpoints after normalization, partitioning and
remapping. Shared pressure endpoints are outside this increment. Validate count,
metadata, nonzero unique public IDs, unique original scoped names, kind and owner,
resolved endpoint uniqueness, cross-family occupied keys and every prepared reset
binding for the whole set before any publisher lookup, reuse or declaration.
A ninth sensor, aliases, late invalid metadata, duplicate IDs/names, wrong ownership,
reserved collisions and missing/extra/changed reset bindings refuse atomically.
Reset preflight cannot publish early entries before validating later ones.
When both the current set and any prepared set are empty, preserve the absent
family outcome before unnecessary immutable-blueprint lookup. An inconsistent
nonempty prepared set must still refuse; do not turn missing identities into an
absence shortcut.

Root adds only the analogous read-only loader query:
`initial_air_pressure_publications_with_identity(&Simulation) -> Result<impl
Iterator<Item = (u64, EntityKind, &str, &AirPressurePublication)>, SceneResetError>`.
The original query remains available. Borrow original IDs/kinds/world-scoped names
and publication metadata from the immutable blueprint, including removed entities;
perform no mutation or IO. The world-scoped name, publication entity name and
frame ID are distinct fields. Respect an actual world prefix containing `::`.

Keep per-sensor demand independent: an inactive peer cannot advance deadlines or
consume draws. Check actual acquisition times and held values, pause/equal-time,
disconnect/reconnect, same-seed reset and terminal refusal. Healthy loaded reset
restores all original identities, rates and streams with full-set validation.
Removing one model retires its own leases while a remaining owner continues;
last-owner removal releases the family. Keep existing weak-output, cached matching,
bounded queue, CPU lease, callback and session-failure behavior. No sensor-bearing
model cloning or editable-model expansion is selected.

## Original reference and observation boundary

Before implementation, independently inspect pinned original AirPressure System
and Sensor source for connection wakeup, scheduling and removal ordering. The inspected original System uses a family-wide connected-and-due wakeup,
updates every mapped owner after wakeup, and returns before removal if none is
connected and due. Its Sensor update does not independently gate on connection;
an individually disconnected due peer can advance/draw when another wakes the
family. Paused System updates can still process removal. Preserve Rust's existing
individual policy, with no individual-disconnect or immediate all-disconnected
removal parity claim. Reuse unchanged scalar/noise evidence
under exact source identities, rather than rerunning old campaigns.

One fresh, bounded original AirPressure System multiplicity campaign must cover
an all-connected pair and eight-sensor boundary with distinct topics, frames,
heights and rates, interleaved sensor WORLD poses, full canonical pressure and
variance values, time/frame/sequence headers and actual received wire bytes.
Remove one entity while a connected due peer continues; observe real publisher
retirement with the correct marked-removal/System/physical-removal lifecycle.
No arbitrary timeout relaxation or all-disconnected immediacy claim.

The original M15 AirPressure plugin/object is retained in
`artifacts/m15-m16-delivery/native-pressure-nzjy3tza`; inspect and pin its actual
lineage and source bytes before a producer release. Reuse it unmodified; no SDK,
plugin, Physics or provider rebuild. Retained M15 metadata records an actual
successful original producer, not M13's failure/continuation history; preserve its
real Root-attestation basis. Original AirPressure does not set an initial-height
reference: its Sensor keeps reference altitude zero and consumes absolute sensor
WORLD-Z. Do not copy Altimeter initial-height subtraction or invent point-velocity
inputs. Freshly bind the reviewed current SDK closure
and any exact separately justified link providers, preserving old manifests and
failures. Availability alone is not qualification. Keep accepted original-reference
source and the unaccepted Rust development parent distinct.

This is controlled original-System sensor input evidence, not a native Physics
trajectory or native/Rust RNG parity proof. Actual native and fixed-literal DATA
must pass before fixture insertion or observed consumers. Emit a named case type
from the outset. Preserve exact fixture bytes through `include!`; no placeholders,
Rust-generated oracle or per-package collector trials. Numeric comparisons keep
the existing strict pressure bound `<1e-9`; captured-value full header/wire encoding
is a separate byte-exact obligation. Missing packets expose no internal update,
draw or deadline state.

## Regression and finite-example obligations

Strengthen the exact former second-pressure refusal XML into real-motion and
healthy-reset positives, preserving neighboring refusals and every other retained
control. Add pair/eight admission, ninth/late-invalid refusal, force/torque-driven
world-Z acquisition at actual sample times, different rates/frames/topics,
independent deterministic and stochastic channels, demand/disconnect, pause,
healthy reset, one-owner/last-owner removal and terminal refusal controls.
Geometry comparisons must use fresh samples at their actual acquisition times;
held samples require exact equality. Keep noiseless/deterministic-mean physical
checks separate from stochastic same-seed controls.
Test normalized/remapped aliases and all late-set/prepared-map identity failures
before IO, including unchanged prior server state and a closed session witness.

A maintained C++ peer must receive actual upstream-Zenoh canonical FluidPressure
messages for the pair and eight boundary with per-topic stamp/frame/sequence,
pressure/variance, reset and retirement ownership. Numeric model replay and
captured-value encoding remain distinct from these live functional controls.
When strengthening an old control absent from inherited control pools, add an
explicit separately named existing-control obligation to both applicable Cargo/log
profiles. Check its exact approved whole-body/span projection once; do not change
historical pool counts or count it as a new test.

Add `examples/m50_multiple_air_pressures.sdf`: mass-two free body at world Z=3,
zero gravity, front `(1,0,.5)` at 100 Hz and rear `(-1,0,-.5)` at 50 Hz, distinct
pressure topics/frames, ideal front and deterministic position-independent pressure
mean on rear within the current noise domain. Append one finite Rapier command
after the exact existing 23-example prefix; preserve current asset and proof
requirements. It is a load/step control, not installation or browser acceptance.
No package/browser refresh is selected.

## Disjoint Source leases and cadence

Light owns Scene `src/air_pressure.rs`, existing private pressure tests, new
`tests/multiple_air_pressures.rs` and the later private observed-native consumer.
Messages owns Server `src/air_pressure.rs`, existing private pressure tests, new
`tests/multiple_air_pressures.rs`, original-reference helper and canonical C++ peer,
and the later private captured-value consumer. Root owns initial-query/export
seams, shared registration, peer and finite-example qualification scripts,
meaningful Python regression updates, fixture insertion, guides/planning/tracker.
SDF independently reviews complete contracts, implementation and all procedures.
Root integrates exact-parent leaves; no agent mutates a frozen qualification tree.

Finish implementation, meaningful regression Source and guides, batch all review
findings, then one consolidated formatter and complete unchanged normal hook,
strict coverage and independent retained local DATA. Root alone serially releases
heavy operations with fresh actual host floors and the unchanged full-normal
14 GiB initial SSD reserve. No per-edit checks or weakened assertions, tolerances,
lint or coverage. Preserve actual failures; batch fixes before affected reruns.
All hosted/official/publication/acceptance gates remain mandatory.
