<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M53: independent loaded logical cameras

## Current fixture-domain correction

The first complete normal run failed five new test cases in four headless targets.
Three fixtures used WORLD names rejected by the unchanged parser; two shared a
static target/behind fixture missing mandatory collisions. The corrected batch
keeps the former XML names as typed negative controls, uses admitted loaded-world
positives, and gives each static link one supported centered box collision.
Every original assertion, test identity, numerical bound and deadline remains.
No parser, loader, sampling, encoder, native helper or captured value changes.

The original selected contract hash `47203cd1` and failed tree `d8fc0cf9` are
preserved by the unqualified failure reference. Native/literal evidence was
captured against that historical selection and remains scoped to direct sensor
components, which do not exercise loaded WORLD-name or collision admission.
This clarification corrects the false XML-name premise; it expands no domain and
grants no new qualification. Independent Source review precedes one consolidated
correction format and full normal retry. Earlier selection text follows.

Root selected M53 Source preparation under D30–D32 after independent whole
scope and reference-amendment review. The exact selection and disjoint leases
are retained in `artifacts/m53-delivery/selection.json` (SHA
`1d472fcd84b65571fe12fb881287826fe1a6612e1e57cc681fcc7e5788f41630`).
This releases implementation/regression/documentation Source only. Formatter,
tests, lint, Cargo metadata/build, native helper compile/capture and publication
remain held until their complete independent Source/procedural prerequisites
and separate Root release. The exact integration parent is locally qualified
M52 commit `1ed59a7348a3dee94cdae8b7a4bbd847cbd9911b`, tree
`8b2073e0e601e6f11ce4ca61e6506b6525511bd0`; that parent remains unaccepted.
Accepted Main ancestor `73e03b9f741689b1538898bde96bf975a49f068e`, the 175 accepted
slices, 42 accepted phases, M40 installed Latest and all hosted/official/Main
holds remain separate. Preserve every M52 repair and failed-attempt record.

## Closed feature and unchanged mathematics

Admit zero through eight independent loaded ideal logical cameras in the existing
Rapier domain. Eight is a project resource cap, not an upstream limit. With zero
cameras preserve the existing early-return and absence behavior. With cameras,
retain zero through 128 total top-level model origins, including camera-owner
models, in one flat world; the eight-camera cap does not increase the model cap.
Require unique valid bare model names and exact world-scoped model names. Preserve
the existing parser name domain: SDF 1.12 WORLD names containing `::` are
refused as InvalidValue. Loaded positives use valid nondefault WORLD names and
derive names by stripping the ENTIRE actual WORLD prefix plus its separator.
The pure owner-identity validator also retains full multi-segment prefix guards;
that robustness does not admit such names through the XML loader. Existing
model and sensor name restrictions stay unchanged. Nested
models and joint-mounted cameras remain refused. Cameras remain direct children
of admitted top-level model links, with existing resolved attached poses.

Preserve the complete `gz-sensors::logical_camera` arithmetic and all original
Scene sampling bodies: six f64 frustum planes, corner and center operation order,
native normal-length `> 1e-6` normalization threshold, finite/nondegenerate derived
planes, and strict negative signed-distance exclusion with zero included. Add no
visibility epsilon. Select model origins, including the owner when visible;
geometry size, bounding boxes, occlusion and rendering do not affect membership.
Output lexical bare-model-name ordering, actual camera WORLD pose and each
selected model's full camera-relative translation and quaternion. Keep the
existing finite unit-quaternion squared-norm tolerance `1e-12`, existing pure
rotated-pose bounds `1e-14`, original native full-pose comparison bound `1e-12`,
all original nominal classification assertions, and exact canonical wire/header
comparisons. Do not substitute magnetometer or live-pressure bounds.

Keep the strict raw logical-camera configuration envelope: matching kind and
effective type, `SensorConfig::Unconfigured`, at most 65536 configuration bytes,
unnamespaced/no-attribute root, exactly four unique finite scalar children
`near`, `far`, `horizontal_fov`, `aspect_ratio`, `0 < near < far`,
`0 < horizontal_fov < pi`, and positive aspect ratio. Preserve finite derived
geometry checks and existing XML/comment/text rules. Noise, including a raw
`noise type='none'` child, unsupported metadata, plugins/extensions, metrics,
visualization, invalid rates, malformed topics/frames and configuration drift
remain refused. Add no backend, spherical/geodesy, dynamic configuration or
general geometry framework. Preserve the current surrounding loader domain.

## Minimal ownership and API changes

Scene has two singleton admission seams: `preflight`'s camera-count equality and
`spawn`'s no-existing-camera guard. Replace them with closed `<= 8` admission and
`< 8` spawn guards, retaining recursive counting so nested/joint candidates cannot
evade preflight. Existing install/nonempty and last-camera retirement predicates
already support collections; preserve them and all arithmetic, schedule, reset
and per-camera sample bodies. No noise stream or seed API is needed.

Root adds an additive borrowed initial-snapshot query returning
`Result<impl Iterator<Item = (u64, EntityKind, &str, &LogicalCameraPublication)>,
SceneResetError>`, following existing multiplicity queries. Preserve the old
metadata-only iterator signature. Identity/name/metadata come from the immutable
initial scene, including removed original owners. Do not make the metadata
struct's existing public literals require new fields.

Server must validate the complete candidate set before any endpoint lookup or
declaration: count, public-ID uniqueness and lookup, Sensor kind, exact WORLD
scoped name, immutable original metadata, live-owner uniqueness, both raw topics
and every normalized/remapped resolved endpoint. The two endpoints for every
owner and all `2N` endpoints across owners must be distinct, including image to
another owner's frustum collisions, and disjoint from all occupied publisher
types. Endpoint remapping does not change original owner metadata or identity.
Preserve reset error precedence and exact prepared-set equality; reject late
invalid candidates without partial endpoint acquisition.

Preserve the existing prepared value's three-field prefix
`(LogicalCameraPublication, String, Outputs)` and append the stable owner ID and
full scoped name, with an exact documented layout selected in final Source
review. Do not reorder or reinterpret those first three fields. Validate the
appended identity/name and original metadata before prepared-output reuse. Root
owns shared all-family reservation/reset registration; Server owns only its
logical-camera collection and peer leaves after an explicit lease.

## Acquisition, channels and lifecycle

One eligible acquisition produces one canonical image and one canonical frustum.
Either image OR frustum demand activates that owner's acquisition; both outputs
are generated. Each camera keeps its independent deadline, sequence and last
sample marker. At eight cameras there are at most sixteen logical-camera endpoint
leases. Do not promise atomic delivery of both remote messages: existing bounded
queue admission and error accounting remain independent per endpoint.

Preserve image stamp/frame/sequence and frustum stamp/frame with no sequence.
Empty images remain valid. Preserve actual sample-stamp geometry: on fresh ticks
compare against actual acquisition-time sensor/model WORLD poses; on held ticks
assert full held reading equality rather than comparing it to a moving current
pose. No demand leaves that owner's deadline untouched; setting interest removes
its disconnected samples immediately. Pause and equal-time polls do not acquire.
Preserve phase-aligned positive rates, existing nonpositive/very-fast rate
behavior, and at most one acquisition per owner per tick.

Invalid due identity, parent, metadata, pose or any model input invalidates the
whole owner's frame without a partial or stale successful pair. Preserve typed
schedule faults, stale-sample clearing and terminal reset nonmutation. This
proposal adds no cross-owner transactional sampling promise beyond existing
behavior. Removal must retire only the selected owner's callbacks/leases while
an active peer continues; model removal must remove that target from subsequent
fresh frames. Last-owner removal retires the sampler/reset systems and interest
resource. Zero-owner worlds own none of that acquisition state or IO.

Loaded reset restores immutable original owners/public IDs/configurations and
fresh entity handles, then restarts deadlines and sequence/epoch behavior.
Distinguish that from bare reset, which resets schedules/samples without restoring
world poses or removed owners. Preserve native-versus-Rust demand, rewind and
removal differences explicitly; claim no native immediate retirement, world
rewind, Physics, stochastic or family-demand equivalence.

## Required Source controls and old-refusal disposition

Keep all existing six Scene private controls, two Scene public controls, five
pure logical-camera controls, captured nominal fixture assertions and existing
Server/peer controls. Preserve exact original inputs and numerical assertions.
The existing private second-camera negative in
`strict_raw_configuration_and_metadata_refuse_before_creation` duplicates the
entire same-name sensor substring, including identical image/frustum topics,
through the existing XML replacement. It is a duplicate/collision refusal, not
a newly admitted distinct-owner input. Retain that exact negative and neighboring
negative rows; do not relabel it as a positive simply by lifting the count cap.
Add a separately identifiable stronger distinct-name/distinct-two-endpoint
physical-motion/reset positive, with its own Cargo/harness/log proof obligation
in both profiles. Final review must verify the parser/name refusal layer of the
retained duplicate input; no parser or duplicate-name domain relaxation follows.

New bounded controls must meaningfully cover absence, pair, eight, ninth refusal,
late-invalid candidate, independent rotated frusta and model selection/full
relative poses, visible-owner inclusion and valid empty frames. Use real queued
forces/torques with actual physical integration for new motion witnesses, not
velocity/pose replacement as a substitute. Exercise different rates and exact
fresh/held stamps, image-only and frustum-only demand, disconnected versus active
peer, paused/equal-time polls, reset and remove-one/remove-last behavior, stale
identity/metadata/parent/model input and terminal nonmutation. Cover immutable
initial query stability, all sixteen endpoint reservations, cross-pair image to
frustum collisions, remap collapse, occupied endpoints, late invalid metadata,
prepared identity/name drift and no-IO-on-invalid-set semantics. Preserve original
scalar/encoder bodies and actual prior test intent; no broad token stripping,
lint waiver or relaxed threshold is permitted in retained DATA projections.

## Fresh component prerequisite and proof scope

This successor preserves original unselected proposal `M53_SCOPE_PROPOSAL.md`
(`f116af4a3e196b9d39490b50d5efab83441e0bc798eabae5d64fbbf6a8197a80`)
and implements Root's before-selection reference amendment
`component-reference-scope-amendment.json`
(`d769ad816ca4086c7dde7abce64d69e9f4844e2fbe6385757563ea4c1583c2ed`).
It replaces the proposed original-System prerequisite with a genuinely fresh
SensorFactory COMPONENT numeric and dual-canonical-wire pair/eight reference.
No accepted M30 control or Rust criterion is removed. Source selection releases
no native operation and grants no qualification or acceptance.

The separately reviewed fixed System/object diagnostic actually failed before
children: result `6edbfc080f1e7d11253b23db5a362cab0dbc710ea0eb47bc87551033d11a6538`,
actual receipt `db568d0435d4035b253827ee4b59bcd29e694d474bb9aaa8b31e8abd3aab65f2`,
Root completion `bafcc1`, exit one. The exact planned System DSO and object were
absent. No provider body was hashed, readelf/build/load performed, or global
System absence proved. Keep this original failure and its method/plan/log
immutable. Neither component reference nor Source preparation repairs or
satisfies an original-System availability/equivalence claim. M30's successful
reference was itself SensorFactory component capture, not loaded Sim System
proof. Retain its actual historical success and earlier failures separately.

Current component identity remains unknown. The new, independently reviewed
fixed component-only input plan `inspect-logical-camera-component-plan.json`
(`ad1108649dbccc222d2478dbd334d5a53db372f7f961c58a1ad13b647b176e9c`)
proposes only one historical versioned component, its two exact alias candidates,
exact Source/installed/Factory headers and pkgconfig inputs, and two bounded
readelf metadata commands. The retained component existence/size observation is
stat-only, not a SHA, alias, ELF or historical-match proof. Its method requires
whole independent Source review and a separate Root release before invocation.
This component identity prerequisite must close before ANY native helper compile
or capture; it does not prevent later explicitly leased Source-only authorship
once the amended contract, parent and exact leases are reviewed and selected.
No body/alias identity or diagnostic success is assumed by this proposal.

Preserve raw RPATH/RUNPATH strings and the reviewed empty-preserving split policy:
empty, relative, variable and outside-approved-directory entries hold the input.
Resolve needed edges only through the exact saved current349 and existing two
link-provider metadata roles; reject unknown/ambiguous edges rather than expand
an implicit allowlist. Any required new exact input needs separate Source review.
Missing/changed/unresolved inputs stay held. Authorize no System/provider/SDK
build, search, fallback, library execution or runtime-loader equivalence claim.
Source, recipe and header identities do not prove Source-to-binary equivalence.

The sole future campaign is `DIRECT_SENSOR_FACTORY_POSE_AND_MODEL_MAP`, with
simultaneous distinct pair and eight modes ONLY. No native retirement mode,
original-System/ECM gathering, family demand, scheduling, clock/reset/rewind,
removal lifetime or Physics equivalence is claimed. Rust alone retains separate
actual physical-motion, demand, cadence, absence, reset/removal, terminal and
identity obligations. Do not repeat M30's original nineteen nominal-point or
twelve side-face-witness campaign, scalar/noise work or original-System replay.

Use exactly the accepted M30 public API path:
`factory.CreateSensor<gz::sensors::LogicalCameraSensor>(sdf::ElementPtr)`.
The derived component declares `Load(ElementPtr)` and hides the DOM overload;
do not substitute the DOM SensorFactory template path or infer its validity.
Parse a real sensor element with actual per-instance name/topic/frame/config,
record actual public getter values, and use public `SetPose` for the controlled
camera WORLD pose. Give EACH sensor its own full `std::map` copy before
`SetModelPoses(map&&)`; moving a shared map into the first owner must not empty
later owners' model inputs. Record exact input pose/model-map bits and source
configuration provenance. Keep finite unit poses, bounded flat model names/maps
and existing frustum semantics, with different camera poses/configs and a common
explicit model set for meaningful independent visibility/relative-pose witnesses.

Call and report the derived one-argument `LogicalCameraSensor::Update(now)` as
an explicitly requested component acquisition. It bypasses the base active/rate
scheduler; even the base overload is not proof of System connection gating.
No paused/no-demand/suppressed/deadline behavior may be inferred from these calls.
Require bounded readiness and actual typed BOTH image/frustum callbacks, complete
shared-time/per-owner sequence and frame contracts, model names/full poses and
full raw wire evidence for every intended acquisition. A returned true or ready
publisher is insufficient. Absence of a packet does not establish no internal
update. Preserve actual provider maps and owned console/log evidence. No output
estimator is allowed in exporter, native DATA reader or literal adapter.

Prospective schedule, counts, schema, build recipe and provider closure must
come from a frozen reviewed helper/producer; none are invented here. Keep any
accepted73e helper-only capture provenance separate from the proposed unaccepted
M52 Rust parent. Current349, existing exact two link inputs and the separately
reviewed exact logical-camera component remain distinct; no original System or
object is a selected capture input. Current component identity is an input pin,
not a claim of correspondence to current Source or inherited native outputs.

Fresh capture requires actual Root exit zero and independent retained native
DATA, audited literal export and independent fixed-grammar literal DATA before
fixture insertion or observed Rust consumer authorship. Then add meaningful
Scene numeric and separate Server wire consumers from actual captured inputs and
outputs, retaining existing native full-pose `1e-12` and exact header/wire duties.
Copy actual borrowed fixtures exactly. Add no placeholders, expected-output math,
Physics reconstruction, native demand/reset/removal credit or internal-update
inference. Future actual selectors stay held until their retained facts exist
and their bindings pass independent review.

## Disjoint Source leases and mandatory completion

Before any authorship, Root must enumerate exact final leases in the selected
contract. Proposed Scene ownership is `src/logical_camera.rs`, its existing
private test, a new `tests/multiple_logical_cameras.rs`, and a later observed
private consumer only after both DATA passes. Preserve existing public scalar
and single-camera motion tests. Proposed Server ownership is its logical-camera
leaf/private tests, new multiplicity public test and peer, a separately reviewed
native helper, and a later observed private consumer. Root owns additive
`initial.rs`/exports/shared reservation and reset hooks, scripts/registrations,
finite example, existing Python control adjustments, guides/contracts/planning,
Source state/control inventory and execution integration. Reference authorship
is artifact-only producer/plan, native DATA adapter, literal exporter/adapter and
their provenance/release proposals, assigned explicitly without overlapping
runtime leases. Independent reviewers must not approve their own authored
production semantics. No new framework or shared schema dependency is proposed.

Budget the bounded phase at no more than five working days including independent
whole Source review, one consolidated formatter after complete finding batches,
and mandatory checks. No per-target trial or alternate gate replaces the full
ordinary normal hook. Preserve both existing Clippy profiles, keep-going
diagnostic collection, denied warnings, tests, docs, Python, finite examples,
strict workspace/per-crate/actual-parent line and region coverage, resource
floors, initial reserve, owned waits, partial-output and finalization guards.
Require fresh exact integrated-parent overlap review, full normal actual zero,
independent retained local DATA with scoped Cargo/harness/log proof for all new
and retained obligations, and four fresh raw comparisons once LAST. Every
failure remains immutable and is corrected in one complete Source batch before
a separately reviewed full retry. Exact hosted CI, official DATA, accepted
predecessor, source/publication and Main gates remain mandatory and separately
held. No Source preparation or local success grants acceptance or transfers an
older phase's qualification.

## Exact initial Source ownership

Scene owns only `crates/gz-sim-scene/src/logical_camera.rs`,
`crates/gz-sim-scene/tests/private/logical_camera.rs` and the new
`crates/gz-sim-scene/tests/multiple_logical_cameras.rs` in its separate checkout.
Server/reference owns only its `src/logical_camera.rs`, existing private
`tests/private/logical_camera.rs`, new `tests/multiple_logical_cameras.rs`,
`tests/native_multiple_logical_cameras.cc` and
`tests/native_multiple_logical_cameras_peer.cc`, all under `crates/gz-sim-server`.
Root owns initial-query/exports/private-initial tests, shared Server integration,
scripts and Python registration controls, example, guides, planning and Source
inventories. Exact files/checkouts/branches are in the immutable selection.
SDF independently reviews authored semantics; cross-reviewers do not approve
their own production changes. No overlapping checkout edits are permitted.

The native fixture, provenance and two observed private consumer leaves remain
held until actual native and literal DATA. Root integrates completed lanes
before appending their observed module tails. The separate helper-only capture
checkout over accepted `73e03b9f` is not yet created and will contain only the
exact independently reviewed new helper. Frozen M52 and all prior original
success/failure evidence remain unchanged. Reference procedures are artifact-only
Source until their own complete reviews and serial Root release.
