## M51 lint diagnostic batching amendment

The first two complete normal attempts stopped at separate new-test Clippy
diagnostics before product tests or coverage. Their original failed trees,
logs and results remain retained. To collect available failures in batches,
both existing Clippy invocations now pass Cargo `--keep-going`. Cargo attempts
as many independent crates in the dependency graph as possible; failed
dependencies can still prevent downstream checks. Each lint profile keeps its
original targets, package/features selection, `--locked` and `-D warnings`.

A lint failure still returns nonzero and stops the normal hook before later
stages. This changes diagnostic scheduling only: no lint, test, coverage,
hosted or acceptance requirement is removed or weakened. The same Rust
1.97.1, two jobs, offline dependencies and resource guards remain. One complete
affected normal retry follows the reviewed loop correction and this amendment;
no separate target trial is introduced. Earlier checkpoints below are history.

See the [Cargo build options](https://doc.rust-lang.org/cargo/commands/cargo-check.html#miscellaneous-options)
and [Clippy usage](https://doc.rust-lang.org/clippy/usage.html) for the maintained
Cargo command behavior.

## Current M51 retained native evidence

One fresh original AirSpeed System pair/eight/active-peer-removal capture returned
actual Root zero, followed by independent retained native and fixed-literal DATA.
The immutable fixture preserves twelve configurations, all 331 explicit sensor
WORLD-pose, POINT-WORLD velocity and WORLD-wind inputs, 55 actual canonical
AirSpeed packets and 276 inputs without a matching packet. The 79 System calls
are summary and pinned-method attestations; missing packets reveal no internal
update, noise-draw or deadline state. Signed differential pressure is valid.

See `planning/reference/m51-multiple-air-speeds.json` for the actual procedures,
commands and provenance. No SDK/provider was rebuilt or capture repeated. The
new Scene exact-scalar/independent-geometry and Server exact-wire consumers are
being reviewed before one consolidated formatter and full normal/strict coverage
pass. No M51 Rust, Python, coverage, hosted or release qualification is claimed.
M1–M42 remain accepted; M40 remains installed Latest. All accepted-predecessor,
hosted, official and Main gates remain mandatory. Earlier checkpoints below are
historical records.

<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M51: independent loaded air-speed sensors

One bounded delivery, at most five working days including review and mandatory
checks, under D30–D32. This contract selects Source preparation only. Independent
complete contract review precedes implementation. No M51 execution, measurement,
qualification, publication or acceptance is claimed.

## Current integrated parent adoption

Later actual evidence supersedes the preparation-time pending M50 status below.
This integrated checkout starts at locally qualified, unaccepted M50
`623faca668151a283f3a6057a13e29d57fe3d9ff`, tree
`9f61769c30e0317d3c0d61e31077e89fba6bd4e3`. Its full normal returned actual
Root zero in 2096.578 seconds and independent local DATA passed: 1809 headless
and 1075 renderer passes with overlap, 403 aggregate Python tests, 24 finite
examples and every strict gate. Its first normal failure and both reader-method
failures remain preserved. The original wrench timeout cause is unknown.

Preparation used qualified M49 `4ccd4f06`; those worktrees remain retained.
All M50 pressure identities/controls, wrench diagnostic/readiness correction and
other Source are preserved. Root reapplies only additive air-speed identity
queries/controls, with independent overlap review before execution. This timing
adoption supplies no M51 result or accepted-predecessor/hosted/official/Main
transfer. Every native-input/procedure and completed Source/check gate below
remains mandatory. See the ignored delivery records for exact evidence.

## Exact dependency and parallel preparation boundary

Read-only feasibility used exact locally qualified but unaccepted M49
`4ccd4f067ecf3438cd9a3cc445cd45d682f6075d`, tree
`b15039730d9926ae49aa3233a0d1b06c4d09a4c2`. M50's separate corrected tree
`9f61769c30e0317d3c0d61e31077e89fba6bd4e3` is currently running its full normal
retry after a preserved wrench-test failure. Neither its outcome nor acceptance
is assumed. D31 permits disjoint Source preparation over the qualified M49 basis;
M51's final integrated parent remains unselected until M50's actual normal and
independent local DATA outcomes are available. Root must preserve M50's reviewed
wrench diagnostics/readiness correction and every other intervening change,
review any overlap/rebase, then bind the actual integrated tree and parent before
checks. No Source or qualification evidence transfers automatically.

M1–M42 remain accepted at Main `73e03b9f`; M40 remains installed Latest. M51
release still requires the actually accepted predecessor, exact hosted success,
independent official DATA and ordinary Main/readbacks. No CI retry/probe, snapshot
workaround, acceptance waiver, new eviction or package/browser refresh is released.
Root alone integrates shared files and serially releases/watches heavy work.

## Closed behavior

Admit zero through eight otherwise-supported loaded Rapier AirSpeed sensors.
Eight is a delivery resource bound, not an upstream limit. Keep the existing
loader domain: direct-link sensors in admitted top-level models, existing rates,
metadata, noise, default atmosphere, finite constant WORLD wind and existing
backend/nested/joint/physical-wind restrictions. No sensor editing, new wind or
atmosphere setters, topology or backend domain is added.

There are three Scene seams: recursive preflight currently admits only count one;
its return value must mean ANY fully validated AirSpeed consumer for counts one
through eight; spawn must enforce the same eight-owner cap. The shared wind guard
must continue to reject unused nonzero wind when no AirSpeed consumer exists.
Inspect every effective/typed consumer before Simulation creation so a ninth or
late-invalid sensor cannot leave partial state. Do not alter shared guard logic
when the existing consumer-presence boolean suffices.

All current sample, geometry, noise, scalar and error bodies stay exact. Use
actual link linear velocity plus angular velocity crossed with the rotated local
offset, the existing composite sensor-frame conversion exactly once, and the
separate WORLD-wind inverse rotation/subtraction in its current Hamilton order.
Preserve whole-zero wind normalization/bypass and mixed signed-zero behavior.
Use absolute WORLD Z. Preserve the existing atmosphere/density calculation,
signed-forward precursor, f32 assignment, final f32 multiplication by 100 and
positive temperature. Negative differential pressure for reverse flow is valid;
there is no AirPressure nonpositive-pressure rejection, variance/covariance or
Altimeter reference subtraction in this scope.

Noise retains its existing dt=0 application to the promoted hPa-scale float
precursor, cast back to f32 before final scaling. Preserve explicit NONE versus
enabled Gaussian/quantized behavior, parameter limits, seed/public-ID ownership,
ideal-failure no-draw, typed errors and schedule ordering. Preserve existing M41
exact captured scalar/wire evidence and its separate live-geometry bound of
1e-6; do not borrow another sensor family's tolerance or invent RNG parity.

Each owner retains its own schedules, noise draws, demand, samples, sequence and
publication lifetime. Same-seed comparison when adding a peer appends the peer
after the original hierarchy and asserts the original public ID is unchanged.
Compare motion geometry only at actual fresh acquisition stamps, and assert held
samples exactly. Keep zero/default single-owner behavior exact.

## Identity, transport and reset

Use distinct effective topics after normalization, partitioning and remapping.
Before ANY publisher lookup, reuse or declaration, validate the whole set:
count, metadata, unique nonzero public IDs, sensor kind, full immutable names,
blueprint/metadata correspondence, resolved endpoint uniqueness, occupied keys
and every prepared-reset binding. Derive the actual whole WORLD-name prefix,
which may contain `::`; do not assume one scope segment. Preserve the current
empty/manual-family behavior and old metadata-only query.

Root adds only a borrowed readonly
`initial_air_speed_publications_with_identity(&Simulation)` query returning
`Result<impl Iterator<Item=(u64, EntityKind, &str, &AirSpeedPublication)>, SceneResetError>`.
It borrows original blueprint identity/metadata including removed owners. Keep
the existing query intact and test missing/empty blueprint distinction plus
original metadata through removal/reset and replacement Bevy identities.

Preserve the old Prepared tuple's metadata/lease prefix `.0`/`.1` and append the
immutable public ID/full-name bindings. Keep existing scalar encoder, IO weak
ownership, acquisition, dedupe, publisher and cleanup bodies exact. Initial/reset
validation is atomic; healthy loaded reset restores original IDs, new entities,
seed and schedule state; terminal refusal leaves existing state unchanged.
One owner's removal preserves peers; final owner retirement releases family
callbacks/publishers under the existing policy.

## Fresh reference and its limits

M41's actual reference is SensorFactory component evidence only: 18 modes,
19 instances, 98 calls, 91 packets and seven suppressed calls. Reuse its unchanged
scalar/noise/wind fixtures; do not relabel them System multiplicity evidence.
Retained successful M26 provider provenance records an actual-zero original
AirSpeed System plugin `f5be1e7e5efaedf049bad7d9d41526337214c9d81aae1c8a0dcb74d4a2d4bf9d`
and object `b5342cbb81e6ec8723c3fe8476e089d3a3f9194735857098f134b2a42fd6a309`.
Preserve the separate failed M26 attempt. The historical actual-zero audit uses
a recorded Root attestation; do not invent an independent raw-wait receipt.
Historical availability is not a fresh current-byte or loader determination.

Before a new reference release, independently review exact provider paths,
current bytes/aliases, source/link provenance and a bounded input contract. The
current 349-node inventory has no air-speed Sensor component node: any exact
additional component and the existing two link-provider inputs remain explicit
separate roles with current guards, not a widened or weakened 349 claim. No SDK
or provider rebuild, old campaign replay or inventory sweep is selected.

Use one genuinely new noiseless original-System pair/eight/active-peer-removal
campaign after Source review. Native AirSpeed Load requires an ECM Wind entity
and Update dereferences its WorldLinearVelocitySeed even for zero wind: create
both explicitly. Record actual SENSOR WORLD pose and SENSOR POINT WORLD linear
velocity plus actual WORLD wind. Native Util rotates through the actual pose
hierarchy before SetVelocity; do not supply link-center or already rotated
velocity, double rotate, or copy pressure-only inputs. Retain every configuration,
actual input, received scalar, complete Header/AirSpeed packet and owner sequence.
Derive counts from the actual new helper; do not assume M50's 441/42 shape.

Native family-wide due+connected wakeup may update all owners, and its early return
can delay removal. This differs from Rust individual demand/no-draw/deadlines.
For removal keep a connected due peer and the marker/live entity through original
Pre/Post calls, physically Process/Clear afterward, then observe actual publisher
retirement under the unchanged bound. Do not claim individual-disconnect,
all-disconnected-removal, RNG-stream or native Physics equivalence.

Independent retained native DATA and fixed-literal DATA with actual Root zero
must precede immutable fixture insertion and observed consumers. Retain old
failures and original provenance; no placeholder, estimator or inferred packet.
Scene numerical and Server exact-wire obligations stay separate.

## Complete regression and qualification batch

Cover pair and eight owners with independent rates/heights/offsets/rotations
under real force AND torque, constant nonzero wind/head-tail/cross/reverse and
cancelling flow without physical wind force; fresh acquisition geometry versus
held samples; deterministic mean/quantization separately from stochastic seeded
owner isolation; pause/equal time, independent disconnect/reconnect, healthy/bare
reset, peer/last removal, terminal refusal, ninth/late-invalid/fullset alias,
occupied and prepared-owner failures before IO; exact original single/default
behavior with an appended peer. Preserve every old test and bound.

The exact old second-sensor XML slot inside Scene
`strict_noise_metadata_domain_nested_duplicate_and_backend_admission_are_atomic`
becomes a stronger same-input load/real-motion/healthy-reset positive. Preserve all
other negatives and require that existing control separately in BOTH actual
profiles outside inherited pools/new-control counts. The old Server second
metadata owner without EntityId must remain negative: it is invalid ownership,
not a legitimate second admitted sensor.

Add a meaningful finite 25th example/guide, a live upstream C++ canonical peer
`multiple_air_speeds`, complete registration and both verifier-set obligations.
Noisy-example counts and all finite examples remain fully checked. Finish
implementation, meaningful regression Source, documentation and independent whole
Source review; batch findings before one consolidated affected formatting/full
ordinary normal pass. Keep strict workspace/per-crate/actual-parent coverage,
offline/jobs2, actual host floors and unchanged 14GiB initial full-gate reserve.
Batch genuine failures before affected reruns; no per-edit trials or weakening.
Independent retained local DATA and all actual hosted/official/Main gates remain.

## Disjoint Source leases after contract clearance

- Scene owner: air_speed.rs, private/air_speed.rs, new multiple_air_speeds.rs and
  later a new private actual-observation consumer; existing math bodies frozen.
- Messages/Server owner: air_speed.rs, private/public tests, new canonical peer
  and original-System helper Source; held procedures use separately reviewed
  current inputs and actual outcomes, never agent execution.
- Root: initial identity query/export, shared registries/qualifier/example/guides,
  plan/tracker, integration and serial operation releases.
- SDF: independent contract and complete Source/procedure/data-method review.

Source leaf preparation may use qualified M49 only after this contract is clear.
Final integration waits for an explicitly selected actual parent preserving all
M50 corrections and requires overlap review; every execution gate remains held.
No new transport/runtime/packaging framework, dynamic plugins, custom solver,
body-velocity teleport, renderer/Fuel/Cargo/lock/schema changes or unrelated
sensor/backend work is in scope. No M40 Latest or accepted-state change.
