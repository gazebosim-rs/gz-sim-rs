## Current M52 native and fixed-literal evidence

The new original Magnetometer System campaign returned actual Root zero in
24.871 seconds, and its independent retained native DATA audit passed. All 34
child stages succeeded. It retained 12 configurations, 501 explicit SENSOR WORLD
pose and constant WORLD field inputs, and 42 received canonical packets across
339 mapped library paths. The 119 System calls are helper-summary and pinned
Source attestations, not invented individual call records. All 58 genuine
missing-spherical warnings and complete owned Console logs are retained. No SDK,
component or System provider was rebuilt; no native Physics or RNG/demand parity
is inferred. Current349, the two separate link providers, the exact magnetometer
component and original M17 System provider remain distinct guarded roles.

Actual fixed-literal generation and independent literal DATA also passed. The
immutable fixture copies all 501 inputs, 42 borrowed actual outputs, 459 inputs
without a matching packet and one active-peer removal event. None does not prove
an internal update or draw was skipped. The retained native component bound is
1e-15 Tesla, separate from live geometry's 1e-18 and exact full-wire obligations.
Observed numeric and canonical-wire consumer Source is independently reviewed;
no M52 Rust/Python/full normal/coverage/local-audit pass has happened yet.
Complete the consolidated Source review and formatter before one full normal
hook with every strict gate. Prior accepted phases, M40 Latest and all accepted-
predecessor, exact hosted/official and Main/readback boundaries remain unchanged.
Older checkpoint text below retains its original historical scope.

## Selected M52 source development and adopted integration parent

The independently reviewed M52 contract below is selected for implementation.
This new integration checkout uses locally qualified but unaccepted M51
`ea921f5886b02bbe576e328848899fd276696208`, tree
`4a29e5dfed7a62a3dfd3107da2e01d8173fb9dde`. Its complete normal hook and
independent retained local DATA passed. Its three original failed attempts,
source preparation trees and every earlier proof remain intact. These are
parent provenance, not M52 qualification or acceptance.

Disjoint Scene and Server/reference source preparation began over M50 623.
The four existing leased magnetometer leaves are unchanged between that base
and M51. Root's three shared query files are reapplied additively; exact reversal
recovers the M51 parent. Preserve all AirSpeed and pressure identity controls,
25 existing finite examples and both strict Clippy diagnostic-collection flags.
No M52 formatter, test, native/provider or metadata command has run. Complete
implementation, regression source, guides and independent review before checks.

The earlier proposal is retained below as its original review record; its
M51-running and future-parent statements describe that earlier checkpoint.

<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M52 proposal: independent loaded magnetometers

This is a contract proposal for independent Source review, not an implementation
or execution release. M51's complete normal gate is running with its candidate
frozen. M1–M42 remain accepted and M40 remains installed Latest. No M52 outcome,
qualification, source publication or acceptance is supplied.

The bounded increment admits zero through eight otherwise-supported loaded
Rapier magnetometers in the existing M17 ideal and M23 XYZ-noise domain. Eight
is an owned resource bound, not a claimed upstream limit. Budget at most five
working days, including independent review and mandatory verification.

## Dependencies and parallel work

M52 depends on M51 for eventual integration and acceptance. Source preparation
may use exact locally qualified but unaccepted M50
`623faca668151a283f3a6057a13e29d57fe3d9ff`, tree
`9f61769c30e0317d3c0d61e31077e89fba6bd4e3`, under D31/D32. This does not
transfer M50 results to M52 or claim M51 has passed. The future integrated parent
remains unselected until M51's actual full normal and independent local DATA
outcomes are known. Integrate in a new checkout, preserving every M51 repair,
air-speed identity/query/control, lint-diagnostic amendment and original failure.
Keep preparation and qualification worktrees and the Fuel owner's work intact.

After this complete contract is independently clear, Scene and Server/reference
Source may proceed in disjoint lanes. Root owns shared files, integration and
all serial heavy work. No M52 compiler, formatter, test, native capture, provider
diagnostic or Cargo metadata invocation follows from this proposal. Source review
precedes authorship; a completed implementation/regression/docs batch and
independent whole review precede one consolidated affected verification pass.

## Closed admitted behavior

Preserve the complete existing single-sensor domain: resolved model/link/sensor
frames, Rapier admission, metadata/topic/frame rules, finite rates and poses,
constant finite WORLD magnetic field, ideal and existing per-axis noise kinds.
Do not copy narrower AirSpeed restrictions into magnetometers. Existing nested
and resolved-frame behavior remains supported as before. Spherical/geographic
coordinates remain refused, including otherwise-default coordinate records.

Change the three existing Scene admission seams together:

1. Recursive preflight validates every sensor and refuses count greater than
   eight before Simulation/backend mutation.
2. Its final result means ANY fully validated consumer (`count > 0`). This is
   necessary for the producer guard to admit an otherwise-valid nondefault
   WORLD field with two or eight consumers. A later invalid owner must not
   authorize the field. Zero consumers still refuse an unused nondefault field.
3. Spawn admits another owner only while the live count is below eight.

Do not change `producer_guard`'s finite/default-field policy. The existing default
WORLD field is `[5.5645e-6, 22.8758e-6, -42.3884e-6]` Tesla. A finite zero field is
valid when otherwise admitted. Preserve typed refusals for metadata, rate,
noise, pose, backend and geodesy failures, and atomic rollback of all late-owner
failures. No Earth's-field model, geodesy, NED/Gauss configuration, runtime sensor
editing, new backend or new physical force is added.

## Arithmetic, sampling and resource ownership

Keep existing production arithmetic and noise bodies unchanged. Measurement is
the inverse actual sensor WORLD quaternion applied to the constant WORLD field,
in Tesla. Translation, linear velocity and angular velocity add no measurement
term. Physical torque may change orientation; pure translation must not change
the ideal field. Retain finite input/output checks and quaternion norm bound
`1e-12`. No pressure scaling, atmosphere, altitude reference, covariance or
body-velocity shortcut belongs in this increment.

Preserve noise after rotation, in X/Y/Z order, at existing zero elapsed noise
time. Keep NONE, Gaussian and Gaussian-quantized metadata admission, precision
thresholds, means, deviations, bias/dynamic settings, rounding and typed errors
exact. Keep the owner seed plus public EntityId derivation and reset behavior.
Appending a peer must preserve and explicitly assert the original public ID
before comparing its seeded sequence. Use stationary identical inputs for
stochastic owner-isolation comparisons; do not compare different trajectories
as if they were RNG evidence. No native/Rust RNG-stream equivalence is claimed.

An owner without interest takes no sample, deadline or draw. Preserve the exact
existing pause, equal-time, late acquisition, cache clearing, manual-family and
reset semantics. An absent family owns no callbacks, installed marker, interest
state or samples. A nonempty family shares the existing one PostStep/Reset pair;
retiring the last owner releases them, and loaded reset reconstructs original
owners outside stepping. Do not add a polling framework, worker or scheduler.

Numerical obligations remain distinct. Existing live ideal/mean geometry uses
its strict `1e-18` Tesla bound. Retained M17 direct-component comparisons use
their existing `1e-15` bound. Preserve all M23 deterministic bit assertions and
statistical bounds. Never import AirSpeed's `1e-6` pressure bound or infer byte
identity from a numerical tolerance. Server canonical Header/Vector3d/full-wire
comparisons remain exact.

## Identity, endpoints and reset leases

Root adds an immutable borrowed query analogous to the reviewed other families:

```rust
initial_magnetometer_publications_with_identity(&Simulation)
    -> Result<impl Iterator<Item = (u64, EntityKind, &str, &MagnetometerPublication)>, SceneResetError>
```

Return original public ID, sensor kind, full immutable WORLD-scoped node name
and borrowed original publication metadata. Removed initial owners remain in
the blueprint; healthy reset restores the same public IDs with new Bevy entity
handles. Keep the old metadata-only query and every old pressure/AirSpeed/
altimeter identity API unchanged. Add meaningful missing/empty-blueprint and
removal/reset/borrow-identity controls; no allocation or mutation in the query.

Server validates the WHOLE set before any output lookup, publisher reuse or
declaration: count, unique IDs, actual sensor kind, entity lookup, original full
name, blueprint/metadata correspondence, resolved/remapped endpoints, occupied
keys and complete prepared-owner bindings. A WORLD name may itself contain
`::`; do not assume a one-segment prefix. Preserve empty/manual-family paths.

Prepared is currently a BTreeMap keyed by resolved endpoint, whose values start
with `(MagnetometerPublication, Arc<MagnetometerOutput>)`. Preserve that `.0/.1`
prefix and append original ID/full name. Add full immutable name to Stream as
needed. Endpoint, metadata and publisher equality alone cannot identify the
owner after removal/replacement. Prepared drift retains `ResetMagnetometers`
error precedence. The old second-metadata-without-EntityId negative stays a
negative. Duplicate resolved keys and late alias/prepared mismatches must refuse
without any declaration or partial reuse. Keep packet, publish, refresh,
dedupe, weak-IO and retirement bodies unchanged.

## Regression batch

Cover pair and eight owners with distinct rotations, frames, rates and topics;
real force AND torque, fresh acquisition poses versus exact held samples,
pure-translation invariance, ideal and deterministic means separately from
stationary stochastic isolation, first-owner default/seed preservation,
independent disconnect/reconnect, pause/equal-time/late-first draw, healthy and
bare reset, removal of one then the final owner, loaded restoration, and terminal
refusal without mutation. Cover ninth and every available late invalid-owner
class, full-set alias/occupied/prepared failures before IO, and supported old
single-sensor behavior with an appended peer.

In Scene private
`admission_refuses_invalid_noise_metadata_second_sensor_and_other_engines`,
replace ONLY the exact existing second-sensor XML refusal with a stronger
positive using the SAME XML: load, real wrench/torque motion, both independently
fresh observations at the existing geometry bound, and healthy loaded reset
with stable IDs/metadata/initial pose and cleared samples. Preserve all nearby
invalid-noise, tiny-rate and TPE negatives. Bind this retained test separately
in BOTH profiles, outside new-test counts and inherited proof pools. Preserve
the entire M23 noise-control source and all old assertions and tolerances.

Keep tests meaningful and finite. Complete all new test Source and docs before
the consolidated checks. Do not add per-edit trials or suppress warnings.

## Original reference and provenance

Retained M17 success is historical provenance, not a new multiplicity result:
`native-magnetometer-joint-state-3e_f7yhi/result.json`, SHA256
`3be14c0414cf761f5ef6de0d2d643d4d070bcbe62acf53643d7c58ddc79d9b05`,
and independent audit `d47a6a159bc21d4c47be9a15f0e5945e1b26e9575d4f41bc0dc964c064c20140`.
It records actual zero and 29 successful stages, with a historical Root-zero
attestation; do not invent an independent raw-wait receipt or transfer its old
Physics/component modes. Preserve all earlier failed M17 attempts.

The retained original System DSO is
`libgz-sim-magnetometer-system.so.11.0.0~pre3`, SHA256
`cc8dfc2869ba3c1a6f16ea4a8e52e94faa13660b1313c57589bfaf63585beafc`;
object `Magnetometer.cc.o` is
`2145199493cb9dd8c6fee4268b826f127a6d2a1276cb02ae985b1a7cf03c7fd8`.
The historical Sensor component is
`libgz-sensors-magnetometer.so.11.0.0~pre2`, SHA256
`e65fcaa2474b03097558f093144b06b0db860bf084ea76cf30cd6644ca5e05ec`.
Current bytes, aliases and ELF metadata remain unverified. The current saved
349-node inventory has no magnetometer component node. Before any reference
release, independently review exact paths/provenance and one bounded component
input diagnostic; retain current349, the separate existing two link providers,
the exact component and original System as distinct roles. No inventory sweep,
SDK/provider rebuild, old campaign replay or automatic missing-provider fallback.

Source-only checks found native System/Sensor files matching M17 pre/post pins.
`8ddf4c17be8a434c51b51d5be7c866fb69aaa54f` is the gz-sensors Git revision,
not a file hash; MagnetometerSensor.cc is
`af554bf9c06ca9605f40b0ba9d44256fbd63eb049180fb243fabf07c2ac3016d`.
Current Source identity is not current binary equivalence or loader proof.

Use ONE fresh noiseless original-System pair/eight/active-peer-removal campaign
after complete procedure/helper/input review and Root release. Capture actual
sensor WORLD poses and the original constant WORLD field, each configuration,
actual XYZ values, timestamps/sequences, Header/full message bytes, owned Console
and loaded library maps. Derive counts and stage inventory from the actual new
helper; do not copy M50/M51 counts. No velocity input or native Physics trajectory.

Native-specific requirements:

- Create WORLD plus MagneticField before sensor creation. The original System
  snapshots that field when adding each sensor. Without admitted spherical
  coordinates, its geographic-update callback warns/returns before replacing
  that field, while the Sensor update still publishes. Keep the WORLD field
  constant; do not claim per-tick WORLD-field mutation reaches the native sensor.
- Configure dereferences its SDF argument. Supply a real non-null plugin SDF;
  never copy AirSpeed's null Configure argument. Do not enable geographic,
  Gauss or NED options to suppress the missing-spherical branch.
- Establish owned Console before loader/configuration. Retain genuine bounded
  warnings with exact pinned-Source provenance and reviewed log policy; no
  blanket stderr exemption, suppression or fabricated clean transcript.
- The family due/connected early return can defer removal. Keep a genuinely
  connected due peer and the marked live component through Pre/Post, then
  Process/Clear. Observe actual publisher retirement under the existing five-
  second bound. No individual-disconnect, all-disconnected removal, internal
  no-draw or native/Rust demand equivalence is inferred.

Actual Root zero plus independent retained native DATA precede fixed-literal
generation; actual generation and independent fixed-literal DATA precede any
observed fixture/consumer insertion. Preserve exact original evidence and the
separation of scalar/geometry comparison from exact packet encoding. No output
estimator, guessed author facts, placeholder observation or forged success.

## Leases and acceptance

Scene owns only `src/magnetometer.rs`, its private test leaf and new public
`tests/multiple_magnetometers.rs`. Server/reference owns only
`src/magnetometer.rs`, its private test leaf, new public multiplicity tests, and
new native reference/peer C++ leaves. Later observed private leaves require
actual native/literal DATA first. All paths are in the respective crates of new
preparation checkouts, never M51's frozen qualification tree.

Root owns initial/query/export seams, shared registration/verifier/peer scripts,
existing Python regression bodies, reference provenance, guides, planning and a
finite 26th example. Preserve the exact old 25-example prefix in the eventual
M51 integration parent and every old gate/alias/refusal. No Cargo/lock/schema,
workflow/snapshot, physics production, viewer, package, launcher or Fuel edits.
No new runtime/transport/packaging framework, dynamic Rust plugins, custom solver,
teleport, new sensor/backend domain or broad compatibility inventory.

Local qualification requires complete independent Source/regression/guide
review, fresh scoped native and fixed-literal DATA, real canonical peer controls,
finite example, the complete normal hook, strict workspace/per-crate/actual-parent
coverage and independent retained local DATA with raw comparisons last once.
Run heavy jobs serially under actual host floors, unchanged 14 GiB initial full-
gate reserve, jobs2/offline/Rust1.97.1 and existing bounded output/owned-process
guards. No new eviction, repeated accepted manual proof or duplicated job.

All results remain provisional until the actually accepted M51 predecessor,
exact hosted success, independent official DATA/durable originals and normal
Main/readbacks meet the existing release boundary. Current hosted recovery has
not been demonstrated; no CI retry/probe/mirror/pin/signature/workflow workaround
is released. Budget does not waive gates or make source readiness acceptance.
