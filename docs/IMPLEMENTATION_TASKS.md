# Current development: M48 IMU world-heading references

[M47](M47_DELIVERY.md) passed its complete provisional local normal hook in
2062.426 seconds and independent retained DATA at clean commit `f27a37fd`:
1757 headless and 1047 renderer profile passes with overlap, 403 aggregate
Python tests and 21 finite examples. Strict coverage passed; changed Scene
production lines were 152/155. All original failed attempts remain retained.
M47 is locally qualified, not accepted. M1–M42 remain accepted and M40 remains
the installed Latest. Hosted qualification and publication of M43–M47 are still
blocked; no retry, snapshot probe or workaround is released.

[M48](M48_DELIVERY.md) selects heading-aware ENU/NED/NWU and CUSTOM/world IMU
orientation references in a closed metadata domain. The new isolated checkout
starts at exact local M47 `f27a37fd`; every frozen predecessor remains intact.
Independent contract and complete implementation review passed. The fresh
original Sensors/Math capture completed in 32.537 seconds with all 89 stages
successful; independent native and fixed-grammar literal DATA passed for all
44 configurations and 264 packets (240 orientation-present, 24 absent).
Audited fixture consumers are integrated for final Source review. Rust tests,
lint, strict coverage and the complete normal hook remain pending. Preserve the
strict gates and actual
accepted-predecessor/hosted/official/Main requirements before publication.

The third normal attempt stopped after 4.166 seconds at the required formatting
check, before compilation or tests. Root had omitted formatting the completed
Server assertion correction. One affected Server formatter pass then completed
in 0.685 seconds, changing only that file's layout. Assertion values, inputs,
production code and fixtures remain unchanged. Preserve the failed `96426233`
source and its original method/log/result; all three actual failures stay failed.
The complete normal hook, strict coverage and independent local DATA still need
a successful actual result. No native reference or DATA reader is repeated.

The second complete normal attempt failed after 1600.379 seconds with two
headless test failures, after both Clippy runs and all 403 aggregate Python
cases passed. One new Scene test compared a held 100 Hz reading to the current
moving pose; its correction preserves all 21 strict geometry checks using the
actual acquisition pose, with exact held-sample and 1/10/20 ms cadence assertions.
One Server test expected a loader refusal for decimal `1e-310`, which the pinned
parser rejects first. That original input now requires exact parser InvalidValue
and conversion diagnostics; five loader-refusal cases remain, and normal degree
input `1e-306` adds the actual subnormal-radian loader boundary. Production code,
numerical bounds and native/literal fixtures are unchanged. Preserve the failed
`e6eeb0a4` source, all logs and partial evidence. No successful full normal,
renderer/strict coverage or local DATA result is claimed. No reference, reader
or formatter is repeated; a separately reviewed full retry remains required.

The first complete M48 local normal hook failed after 84.164 seconds at
required Clippy compilation, before tests or coverage. Rust E0502 identified
an immutable borrow of spherical metadata extending across later model mutation.
The correction stores one owned metadata snapshot during loading and borrows it
only for validation. Values, reference arithmetic, all assertions and every
native/literal fixture are unchanged. Original failed tree `47840d61`, logs and
packet remain retained. No native capture, data reader or formatter is repeated;
a separately reviewed complete normal retry remains required.

The earlier checkpoints below remain historical; their pending-result wording
does not override the actual M47 local outcome recorded above.

# Current development: M47 Gaussian odometry velocity noise

[M46](M46_DELIVERY.md) passed its full provisional local normal hook and
independent retained local audit at clean commit `cca6b07d`: 1735 headless and
1034 renderer profile passes with overlap, 402 aggregate Python tests, 20 finite
examples and all strict coverage gates. Both prior failures remain retained.
It is not accepted or publicly qualified. M1–M42 remain accepted and M40 remains
the installed Latest release; M43–M46 hosted/publication closeout remains blocked.

[M47](M47_DELIVERY.md) implements finite nonnegative Gaussian velocity noise
for the existing Two/Three OdometryPublisher, with native covariance and unchanged
physical pose. The complete Source, regressions and guides passed independent
review, and the single formatter passed. Fresh original-component capture and
independent retained native DATA passed: 16 modes, 18 configurations, 1418 inputs,
1316 triples and 3948 packets. The reviewed current-provider update preserves the
historical baseline; all 349 current nodes and 337 loaded providers were checked.
At this pre-verification source checkpoint, Rust tests, lint and the full normal
hook remain pending; scoped native DATA does not supply that qualification.
No CI retry/probe, acceptance waiver, package/browser refresh or Fuel work is
selected. Accepted predecessor and all normal hosted/official/Main gates remain
mandatory for publication and acceptance. See the M47 contract for both retained
pre-child failures and the precise current-provider baseline.

The first full normal hook returned actual exit one after 178.137 seconds at
required Clippy compilation, before tests or coverage. One retained M46 Server
replay initializer omitted the newly required `gaussian_noise` field. The complete
21-constructor/alias inventory found no further omission. Its correction adds
only `Default::default()` noise to that initializer, preserving every original
numeric and wire assertion. The failed source/tree/logs remain retained; no
native reference, audit or formatter is repeated. A separately reviewed complete
normal retry is still required; no Rust or coverage pass is claimed.

The older checkpoints below remain historical, with original evidence preserved.

<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

## Current bounded task: M46-ODOMETRY-ROTATION

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

M46 implementation and regression Sources have passed independent leaf review.
Fresh original-system capture and independent native/literal audits passed for
30 modes, 32 configurations, 1,020 calls and 858 triples (2,574 canonical messages).
The audited fixture and reviewed numeric/encoder consumers are integrated.
Complete integration review now precedes one full normal/strict local pass.
Rust checks and all hosted/publication prerequisites remain pending; reference
proofs do not supply Rust acceptance.

Read [M46 delivery](M46_DELIVERY.md). Develop finite local/global rotational
odometry offsets over exact locally qualified M45 `e66c21e7` in the new isolated
worktree. M45 full normal and independent local audit passed; neither that
provisional result nor the M46 selection supplies acceptance. M1–M42 remain
accepted at `73e03b9f`; M43–M45 hosted/publication closeout remains pending after
snapshot setup failure. No retry or probe is released.

Light owns the Scene production/private test leaf, new public regression leaf,
example and feature guide. Messages owns new Server regression and native-peer
leaves plus the bounded native case design. Root owns shared registration,
qualification, approved old-negative dispositions, planning/status and serial
execution. SDF independently reviews the whole Source. Preserve all old native
fixtures and every unrelated assertion. Complete implementation, meaningful
regression Source and docs, batch review fixes, then one consolidated pass.
The actual native and literal audits passed before fixture insertion. Rust
qualification remains pending; no package/browser refresh is selected.

## Historical M45 pre-qualification checkpoint: M45-IMU-CUSTOM-WORLD

Read [M45 delivery](M45_DELIVERY.md). This integrated candidate uses exact locally
qualified a08f0b50, preserving all containment repairs; accepted Main remains 73e03b9f.
The prior Source lane 9a is preserved. M43/M44 exact hosted/official and publication
are pending after the fresh snapshot TLS/select setup failure. No retry is released.
Reviewed timing amendment 5d823a75 permits one provisional full LOCAL qualification
only after complete Source/integration/tree/runner review and serial Root release.
Actual accepted predecessor, exact hosted/official and Main/readbacks still govern
publication and acceptance. No original proof is relabelled or transferred.

The standalone original-component capture and native/literal DATA passed for 14 new
configurations and 84 calls. The exact fixture/provenance copies are inserted. Light
and Messages completed the two new private native consumer leaves; SDF reviewed
their complete Source. Root owns append-only cfg hooks and shared Source/docs,
complete integration review and serial release. No M45 Rust check has run. Preserve
every old fixture/test body, strict bound and actual failure; no placeholders.

M45 admits effective CUSTOM angles with explicit world parent while preserving
legacy initial-relative bits and named frame arithmetic. Its new development
example becomes the nineteenth finite CLI qualification input; M44's eighteen
archived examples and package bytes remain unchanged. M45 selects no new
package or browser refresh. Keep actual physical-force, lifecycle, canonical
wire, strict coverage and exact hosted checks before acceptance.

## Historical source preparation: M43-IMU-REFERENCE and M44-INSTALL

Read [M43/M44 delivery](M43_M44_DELIVERY.md). M41/M42 are accepted at
Main `73e03b9f` after exact hosted CI and independent official DATA. This is the
actual parent for the continuing wave. Source preparation overlapped closeout;
native capture, tests, builds and installation still require complete independent
source review and Root's serial release.

Light owns the private Scene `imu_reference.rs`, its private and public
regression leaves, the M43 example and feature guide. Messages owns the Server
`imu_reference_frames.rs`, transport peer and separate component-reference C++
leaves. Captured-value fixture and encoder replay follow actual independent
native/literal DATA, without placeholder expected values. Root owns shared
Scene admission/reset-test disposition, peer registration and retained-profile
requirements, packaging and its regressions/guides, and canonical integration.
SDF reviews complete batches and test intent independently.

The finite native case table is agreed before oracle authorship: 17 fresh
configurations and six prospective forced updates each, with direct WORLD
inputs and actual native sensor-axis setter values recorded separately. Named
references reject spherical metadata; CUSTOM preserves legacy bits/admission.
Required controls cover all named frames, actual motion, disabled orientation,
noise/covariance preservation, demand/pause/rate, reset/removal and canonical
shutdown. M44 requires fresh eighteen-example, three-executable distribution,
clean installation, force/feedback and default robot browser evidence. Neither
scope is accepted by source completion or by an earlier phase's evidence.

## Historical bounded task: M38-ODOM3D

Accepted parent: `6d2d53209174e3398b79909553e4fc5fd7fe05e8`.
Implement the noiseless 3D ground-truth odometry contract in
[M38_DELIVERY.md](M38_DELIVERY.md). Light owns Scene leaves/tests/example/guide;
Messages owns the Server canonical peer and scoped original-system native
helper/cases/fixture; Root owns shared configuration and registration, CLI/
profile/retention requirements, integration and records. SDF independently
reviews semantics and test integrity. Agree exact arithmetic, type seams and
finite native cases before runtime edits. The implementation and actual native
fixture are now integrated; the guide names concrete controls. Original-system
capture and independent native/literal DATA passed. Rust tests/lint and full
qualification have not run. Finish whole Source review before the consolidated
mandatory verification.

The older current-task descriptions below are historical checkpoints.

# Implementation work packets

Each packet is a bounded assignment. Split broad packets by the named system or
test family before implementation; do not give an agent “port all sensors” with
no acceptance boundary. IDs below are stable coordination labels, not GitHub
issue numbers. No task is implemented merely because it appears in this plan.

Every packet follows AGENTS.md and produces an evidence record containing owned
files, upstream test IDs, dependency decisions, code revision, exact check commands,
results, coverage, relevant benchmark data, reviewer, and unresolved limits.
The task author cannot independently retire tests or approve coverage exclusions.

## Historical atomic M21–M25 task checkpoint (superseded)

[M21_M25_DELIVERY.md](M21_M25_DELIVERY.md) supersedes serial intermediate-parent
qualification with one new complete candidate over accepted M20. Every per-phase
functional/native criterion, mandatory normal/strict/exact-hosted check and fresh
M25 distribution/install/four-example/browser/shutdown proof remains required.
Two preserved hosted LLVM SIGBUS failures have unknown cause. Aligning debug info
and adding bounded resource diagnostics does not establish a cause fix.
Root owns integration/shared planning/CI/publication; existing source and procedure
owners keep their narrow leases. Preserve all old trees and evidence. Complete
independent Source review before the consolidated verification, serialize heavy
work, publish actual results to the tracker and continue after acceptance.

### Historical M10-CREDIT contract (closed; preserve evidence)

The initial M10 candidates passed required checks, but actual browser acceptance
failed. The qualified diagnostic viewer confirmed event-queue overflow. This
bounded correction uses the existing browser bridge, not a new transport or
runtime. Root owns `crates/gz-sim-cli/src/web/socket.rs`, the new credit contract
and real-socket regression sources, their parent test-module declaration and
M10 documentation. The viewer owner implements the matching maintained Session
and PrimitiveBackend contract; a separate reviewer checks both complete batches.

Only lifecycle clients opting into publication credit gain the one-flight
normal-publication handshake. Existing no-credit protocols, the 64-event queue,
64-event drain, world request ordering and exact committed replies remain intact.
Ready parsing must work while a world reply is pending; same-connection reset
must preserve an already validated transport token. No simulation wait belongs
in this network backpressure path. See [the exact contract](M10_WORLD_LIFECYCLE.md).

New regression families cover strict credit JSON and token exhaustion, paused
and advancing native worlds, command-before-Ready, delayed committed replies,
reset-before-old-Ready, query negatives and reconnect isolation. Keep all retained
assertions. Complete implementation, regression source, docs and independent
source review before one consolidated required pass per affected repository.
This historical packet required actual stable browser spawn/remove/state/reset
and reconnect observations. They passed at the revisions in M10_ACCEPTANCE.md;
source review or CI alone never constituted acceptance. Preserve all earlier
failures and frozen trees; this section does not authorize requalification.

## Foundation and reference tasks

| ID | Depends on | Assignment | Exit evidence |
| --- | --- | --- | --- |
| REF-01 | None | Verify all main-branch baseline SHAs, dependency compatibility, licenses, build flags, native versions; create immutable reference environment | Baseline lock and reproducible build/run instructions; clean source hashes |
| REF-02 | REF-01 | Enumerate runtime tests across CTest/gtest/Ruby/Python/GPU/backend combinations; reconcile static inventory | Complete ledger schema and expanded entries; baseline results and blocked cases |
| REF-03 | REF-01 | Capture balanced robotics benchmark suite and hardware/environment | Raw samples, variance, accuracy baselines, proposed budgets ratified |
| PHY-00 | REF-01 | Record Avian/Rapier evaluation; enforce full-f64 gate and probe prioritized Rapier | Reproducible precision/capability evidence, exact versions/features, selected backend ADR; no invented performance conclusion |
| API-01 | REF-02 | Freeze minimal IDs/time/frames/errors, SDF resolver, message/transport and Fuel integration contracts | Contract fixtures; independent repos compile without Bevy; no reverse dependency |
| QLT-01 | REF-02 | Implement staged-tree pre-commit, profile matrix, CI, coverage accounting, ledger checks | Known-bad fixtures fail; exactly 95% fails; missing profile fails; no false success from empty code |
| CORE-01 | API-01, QLT-01 | Create minimal Cargo workspace, f64 values, stable IDs, simulation epochs and clock | Unit/property tests >95%; no-render dependency closure; documented toolchain/Bevy pairing |
| EXT-01 | Superseded by D26 | Retired external system-loader task; implement ACT-01 instead | Record explicit dispositions for loader-only tests; retained system behavior remains required |

## Independent repository tasks

| ID | Depends on | Assignment | Exit evidence |
| --- | --- | --- | --- |
| MSG-01 | REF-01, API-01 | D29: preserve canonical proto definitions, imports/options, tags/defaults and notices; remove the repository runtime after consumer migration | Schema hashes/import closure, standard protoc validation and consumer-owned generated binding checks |
| MSG-02 | Superseded by D29 | Stop custom runtime/reflection/conversions/CLI/codec parity and optimization | Explicit dispositions; retain consumer schema/wire semantics; no library-completion claim |
| SDF-01 | REF-02, API-01 | XML/schema infrastructure and minimal validated semantic world/model/link/joint/sensor | Includes/frames/defaults/error cases and C++ differential tests |
| SDF-02 | SDF-01 | Complete version conversions, nested/merge/frame/inertia/import/export semantics | Full retained sdformat ledger, fuzzing and large-model benchmarks |
| NET-01 | Superseded by D28 | Stop gz-transport-rs and ZeroMQ protocol implementation | Preserve historical evidence; remove consumer dependency through ZEN-01 |
| NET-02 | Superseded by D28 | Stop standalone transport runtime, parity and CLI work | Retained simulator networking/logging requirements map to focused Zenoh consumer tasks |
| ZEN-01 | MSG-01, API-01 | In gz-sim-rs, use upstream Zenoh directly; replace legacy transport and packaged Messages dependencies | Pinned Zenoh/proto/codegen inputs; real C++ Zenoh pub/sub/query/reply; bounded queues, reset/removal/shutdown, schema fidelity, strict owned coverage and equal-work measurements |
| FUEL-01 | API-01; Fuel owner readiness | Consume existing Rust API, offline cache/resource identity, cancel/error contract | Local fixture integration; pinned owner-provided revision; no reimplementation |

SDF, Fuel and schema maintainers own their repository tasks. ZEN-01 belongs to
gz-sim-rs; it does not create a transport successor. gz-sim-rs owns acceptance at the
integration boundary. P0 contract coordination can proceed before any producer
is complete. QLT-01's policy must be applied in each independent repository.

## Runtime and physics tasks

| ID | Depends on | Assignment | Exit evidence |
| --- | --- | --- | --- |
| CORE-02 | CORE-01 | Bevy schedules, world ownership, spawn/remove transactions, pause/step/reset | Priority/load-order, deferred-command, timestamp, epoch, multiworld isolation tests |
| ACT-01 | CORE-02, LOAD-01 | Built-in system demand from loaded world/models; activate/retire schedules and owned resources on structural lifecycle transitions | Absent-system zero invocation/allocation/worker/endpoint controls; last-consumer teardown; dependency/shared-consumer behavior; reactivation/reset/multiworld and equal-work CPU/memory evidence |
| PHY-01 | CORE-01, PHY-00 | Capability API, normalized construction/command/state/query contracts | Tested unsupported-feature diagnostics; no template-framework port |
| PHY-02 | PHY-01 | TPE behavior port using reusable geometry | Upstream TPE motion/contact/lifecycle tests and benchmark |
| PHY-03 | PHY-01 | DART native adapter and safe ownership/threads, batched IO | Same-version upstream DART parity, FFI tests, >95% Rust and bridge evidence |
| PHY-04 | PHY-01 | MuJoCo adapter, SDF lowering, model/state mapping and topology edits | Actual supported features verified; state preservation/rollback; native lifetime tests |
| PHY-05 | PHY-00, PHY-01, CORE-02 | Built-in Rust engine adapter and explicit stepping/precision | Capability probes, accuracy tests, sync overhead and deterministic-profile evidence |
| SENSOR-01 | CORE-02, PHY-01 | Sensor scheduler, streams/seeds, shared models, IMU first | Rate/pause/reset/noise/frame tests and timestamp correctness |
| IO-01 | CORE-02, MSG-01, ZEN-01 | Clock/pose publication, commands/services and bounded transport integration | Actual C++ Zenoh client interoperability; ordering/backpressure/shutdown cases |
| LOAD-01 | SDF-01, FUEL-01, CORE-02 | Validated semantic scene to ECS/backend construction; local/offline resources | Original world fixtures, failure atomicity, frame and ID checks |
| SLICE-01 | LOAD-01, PHY-02, PHY-03 or PHY-05, SENSOR-01, IO-01 | First complete exact-step SDF world | P2 report with C++ comparison, >95% code coverage, performance data and remaining scope |

PHY-03 is required for final DART parity even if PHY-05 brings up the first dynamic
slice sooner. PHY-04 and PHY-05 are both required release backends alongside TPE.
Temporary SDF bridges require explicit removal tasks. Existing ZeroMQ/runtime
consumer pins remain historical transition inputs until ZEN-01 replaces them.

### Active bounded continuation

Root owns the LOAD-01/IO-01 world Light pose follow-up: Scene child construction,
Server hierarchy validation/publication, renderer refusal and lifecycle/real C++
peer tests. It depends on accepted SDF `13adde94`, existing pose ownership and
Zenoh codegen contracts. Native reference source, commands and library hashes are
checked in; [WORLD_LIGHT_POSES.md](WORLD_LIGHT_POSES.md) records prior-reviewed
assertion transitions and exact remaining qualification. This does not advance
B03, SDF radiometry, original-system completion or performance acceptance.

## Assets, rendering, and complete sensors

| ID | Depends on | Assignment | Exit evidence |
| --- | --- | --- | --- |
| ASSET-01 | LOAD-01 | URI policy, decoded cache, mesh/material/animation loaders | Collada/STL/OBJ/glTF and relative-resource fixtures; hashes/provenance |
| ASSET-02 | ASSET-01 | Terrain/DEM, heightmaps, geospatial/environmental data | CRS/origin/nodata/scale and sensor sampling fixtures |
| RENDER-01 | CORE-02, ASSET-01 | Bevy offscreen scene extraction, f64/frame/origin conversion | No-window GPU test; large-coordinate transforms; scene lifecycle and readback cancellation |
| RENDER-02 | RENDER-01, SENSOR-01 | RGB/depth/RGBD, calibration, distortion, wide-angle | Analytic projection/range tests, image fixtures, no hidden temporal postprocessing |
| RENDER-03 | RENDER-02 | Segmentation, bounding boxes, thermal and visual effects needed by retained features | Label/geometry/thermal checks and mapped upstream assertions |
| SENSOR-02 | SENSOR-01, PHY-03, PHY-04 | Force/torque, pressure/speed, altimeter, magnetic, NavSat/logical/contact | Model/noise/units/frame tests and backend capability errors |
| SENSOR-03 | RENDER-01, PHY-01 | CPU/GPU lidar and collision-versus-visual query semantics | Range/intensity/clipping fixtures and per-sample acquisition timestamps |
| SENSOR-04 | SENSOR-02, ASSET-02 | DVL, environmental and remaining sensor models | Complete sensor ledger; deterministic environmental sampling and error behavior |
| SENSOR-05 | RENDER-03, SENSOR-03, SENSOR-04 | Multi-sensor scheduling, overload, lockstep, cancellation | Exact sample counts, bounded buffers, reset epochs, CPU/GPU coverage and soak |

## Simulator system families

`planning/system-migration.json` assigns every one of the 78 inspected system
directories to a work family. The assignment is provisional organization; all
entries remain `not_migrated`. For each system, enumerate SDF parameters, default
behavior, topics/services, components/outputs, backend requirements, reset behavior,
fixtures, and tests before converting it.

| Family | Main dependencies | Acceptance focus |
| --- | --- | --- |
| SYS-CONTROL | SLICE-01, physics joints | Drive/controller commands, limits, odometry, actuation timing, reset |
| SYS-DYNAMICS | Physics forces/contacts, residual math | Aero/hydrodynamics, buoyancy, propulsion, slip, energy and battery behavior |
| SYS-SENSOR | Sensor packets | Sensor registration/configuration, topics, frames, labels and timing |
| SYS-WORLD | CORE-02, IO-01, ASSET-01 | Commands, scene/pose/state, attachment, lifecycle, labels and world services |
| SYS-COMMS | IO-01, ZEN-01 | Acoustic/RF/perfect communications, endpoint and triggered publishing semantics |
| SYS-MEDIA | RENDER-03, ZEN-01, ASSET-01 | Recording, replay, exporting, effects and scene assets; no GUI dependency |
| SYS-EXTENSION | API-01, ACT-01 | Retained external-language workflows and lifecycle mapping; no dynamic simulation system loader |

Family completion requires all its directory entries and associated tests, not
just one representative implementation. Split each family into individual system
or tightly coupled system packets for independent implementation/review.

## Non-system features and release tasks

| ID | Depends on | Assignment | Exit evidence |
| --- | --- | --- | --- |
| WORLD-01 | CORE-02, IO-01 | Server configuration, world services, entity factory, assets, multiworld lifecycle | Public API/service inventory mapped; concurrent lifecycle tests |
| WORLD-02 | WORLD-01, SYS-WORLD, ZEN-01 | Levels, performers, distributed stepping/state and resource behavior | Baseline distributed/levels tests; partition/disconnect/recovery and exact-step evidence |
| LOG-01 | ZEN-01, IO-01, SYS-MEDIA | State and transport recording/replay, format migration | Existing logs readable, ordering/time/seek/reset and output fidelity tests |
| SCRIPT-01 | API-01, WORLD-01 | Inventory Python workflows; choose focused bindings and migration path | Retained Python tests/workflows; no accidental GIL on numerical hot paths |
| CLI-01 | IO-01, MSG-01, ZEN-01, SDF-02 | CLI grammar, discovery/dispatch, version/help, package ownership | Retained simulator/SDF/Fuel command assertions ported; no standalone Messages/transport CLI |
| GUI-API-01 | WORLD-01, RENDER-01 | External GUI scene/control/asset/sensor protocol and version negotiation | Separate minimal protocol client tests; snapshot/delta resync; no GUI implementation |
| MATH-01 | REF-02; domain demand | Audit every math header; reuse or implement documented semantic gap | Complete reuse map, exact tests, no redundant generic math library |
| PERF-01 | REF-03, SLICE-01 onward | Profile and optimize measured bottlenecks continuously | Equal-work before/after evidence; representative release targets met |
| PKG-01 | CLI-01, all backends | Linux packages, native dependency discovery, feature bundles and install layout | Clean-machine install/run; no gz binary conflicts; reproducible versions |
| QUAL-01 | All retained scope | Full parity/coverage/performance/lifecycle/release matrix | P7 acceptance report; no unrun required cases or unexplained exclusions |

MATH-01 is incremental: do not build an unused port of the entire math library
before a simulator slice needs it. Requirements for public standalone math behavior
remain tracked in the ledger even when not used by the first slice.

## Assignment template

```text
Task ID and concrete behavior:
Prerequisites and current contract versions:
Owned files/modules and shared interfaces affected:
Upstream SHA, source, test IDs, and fixture hashes:
Reuse decision and any missing library semantics:
Implementation and failure/lifecycle cases:
Required configurations and exact check commands:
Performance probe and accuracy budget, if relevant:
Evidence artifacts and reviewer:
Remaining blockers and next bounded task:
```

Keep worktree/file ownership explicit when several contributors work at once.
Integrate shared interfaces before dependent implementations. An agent may not
change a producer contract solely to make its own consumer tests easier.

## Private Scene continuation: free-root serial three-body DART

Root owns a bounded LOAD-01/PHY-03 consumer follow-up in `joints.rs` and new
Scene tests. The accepted backend already supports a contact-free free root with
two serial revolute children. Admit only that topology, preserve the existing
world-rooted two-link case, construct constraints in parent-first order and keep
public joint declaration identities. Retain branch, cycle, world-rooted
three-link, larger-tree, limits and other-backend refusals. Tests must cover
reversed declarations, actual backend comparison, loaded reset and removal. No
old refusal assertion changes before independent disposition review. No new
backend contact admission, B03, multiple collisions or native SDF parity claim.
Implementation and test-code findings are batched in this candidate; final source
review and consolidated qualification are pending. See
[SCENE_FREE_THREE_BODY.md](SCENE_FREE_THREE_BODY.md).

### PHY-03 / SLICE-01: bounded DART compound collisions (in progress)

Root owns Scene loading and regression controls; the physics owner owns the
existing DART adapter and concrete shared physics contracts. Work is isolated
from accepted Main. Reuse pinned DART ShapeNodes on one BodyNode, existing shape
validation, resolved explicit inertia, transaction/reset machinery and numerical
bounds. Admit one to three primitive shapes with link-local transforms in DART;
keep the centered-COM restriction and all other backend refusals. Validate every
collision's geometry, surface and pose and retain each child's density metadata.
One body/ID represents a link; collision entities keep their declaration IDs.

Complete implementation, regression code, native comparison and independent
source review before one consolidated mandatory verification pass. Meaningful
controls cover final-shape refusal, local bounds, native body/shape semantics,
loaded reset/removal and unchanged other-engine refusals. B03 still needs offset
COM and full trajectory/equal-work evidence; this task does not accept it.

### PHY-03 / SLICE-01: explicit offset COM for DART compounds (source in progress)

This follow-up is isolated from the qualifying centered-COM feature. The physics
owner extends the existing explicit-inertia COM, origin-conditioning and velocity
semantics to every local primitive; Root owns Scene integration. Scene admits
resolved nonzero COM only for DART solid links, including the existing single
collider path. Other engines and planes keep their prior refusal. No new solver,
aggregate inertia, dependency, numerical threshold or runtime system is added.

Complete source, regression code, native observer source and independent review
before verification, and wait for the parent feature's acceptance. Native origin
velocity and COM velocity must be distinguished explicitly. Preserve old tests;
propose exact same-input stronger-positive transitions for independent review
before any fold. B03 loading, full native trajectories and equal-work evidence
remain separate acceptance requirements; do not infer them from this guard.

### PHY-03 / SLICE-01: atomic two-edge serial construction (source in progress)

The offset-COM prerequisite is accepted at `e55356c8`. This next bounded feature
adds one cold DART operation for three isolated dynamic free bodies and two
serial revolutes. Candidate preparation uses mature native skeleton cloning;
all final contact, velocity, grid and conditioning guards remain. Complete
readback/metadata precede the live swap and two-ID commit. Reset retains only
accepted pair grouping; partial removal must preserve the existing paths.
Root owns Scene integration and declaration identity. Independent source review
approved the exact B03 construction/reset transition, not its execution or
acceptance. Actual native B03 observations are available, while the new direct
wrench/contact/reset reference has one successful reviewed-source capture,
with independent raw/source/library audit passed. Finish all
source and data review before one consolidated mandatory gate; performance and
general topology/contact support remain separate.

### SDF-02 / PHY-03: adopt the checked radian constructor correction

Root owns the isolated `work/producer-euler-adoption` consumer batch. It retains
the independently reviewed child-frame serial construction and fresh-reference
changes from the failed 361-second qualification without editing that attempt's
tree or observations. The producer's native radian-Euler constructor difference
is confirmed by source; its role in B03 divergence remains unproven.

Owned integration paths are the dependency revision and lock source,
`sdfb246_admission.rs`, and adoption/handoff documentation. The seven supplemental
inputs retain their order and URI. Omitted density and an explicit identity
collision pose gain reviewed positive physical, metadata and lifecycle controls;
the other five producer negatives and manual-density policy remain unchanged.
No DTO, backend admission, system scheduling or transport rewrite is required.

The corrected producer `c94da539` passed its mandatory local gate, exact hosted
run `41417270384` and independent official audit before public Main publication.
Only then did this candidate change the manifest revision and lock source;
accepted simulator Main remains on `13adde94`. Complete independent review of
the combined consumer source before one consolidated mandatory verification pass. Retain all fresh B03 fields,
ticks and `1e-12` bounds, existing native and real C++ Zenoh controls, strict
coverage and exact hosted qualification. Neither producer acceptance nor this
source migration completes B03 or equal-work performance acceptance.

### SDF-02 / PHY-03: adopt native joint semantic-pose evaluation

Root owns `work/producer-joint-pose-adoption`, preserving the failed constructor
adoption source and observations. Owned additions are the minimal producer
revision/lock update to accepted `f0d05146`, the Scene child-frame query selection,
and adoption/handoff documentation. The producer passed exact hosted run
`41438358157` and independent official audit before publication. Complete source
review, then one consolidated mandatory simulator gate. Preserve every fresh B03
field, tick and `1e-12` bound, existing guards and strict coverage. Real C++ B03
interop follows accepted motion; performance remains a separate unmatched scope.

The first joint-pose consumer gate (`ad718fbf`) passed strict B03 motion but failed
at a later serial lifecycle identity helper after 367 seconds. Root's
`work/serial-public-identities` correction owns only that helper and factual docs,
retaining the failed source/evidence and every lifecycle assertion. Independently
review the complete correction before one affected mandatory rerun; no separate
preliminary test or lint is required.

The subsequent `f4db5841` gate passed serial lifecycle but failed at the two
analogous B03 world-light inventory loops. Root's `work/b03-public-identities`
correction batches both loops with the same strict public-identity helper and
source review of other new tests for this class. All retained assertions remain;
finish review before one affected mandatory pass, without preliminary checks.

### IO-01 / PHY-03: Gazebo C++ B03 interoperability

Root owns the next source slice on accepted Main `8ebb02db` (exact hosted
`41447340434`, independent official audit passed). Owned files are server
`tests/native.rs`, `tests/clock_client.cc`, `docs/B03_NATIVE_INTEROP.md` and these
handoff/task updates. The external Gazebo C++ peer must receive actual six-phase
clock/local-pose snapshots through control/reset, reconstruct body world poses
with maintained native math and compare the accepted fresh reference at unchanged
bounds. Retain all existing peer modes, assertions, new file-backed diagnostics,
strict coverage and pinned SDK verification. Upstream Rust Zenoh is used directly
and unmodified; this is interoperability testing, not a C++ transport port.
Complete final source review before one consolidated local gate, then exact
hosted qualification and independent official audit before acceptance.

### REF-03: configured physics-only B03 comparison

Root owns `work/b03-physics-benchmark` on accepted `43cf8e69`. The owned slice is
one Rust Scene example, one public native physics actor/reference, one concrete
benchmark driver and its protocol controls, and benchmark/handoff/task docs.
It excludes Server/transport/IO and keeps earlier contextual actors unchanged.
Explicit unlimited per-pair contacts match the Rust plain-ODE selection without
changing product defaults. Bind source-established settings honestly; require
actual common DART/ODE libraries, full physical ownership and all 16200 values
across 100 frames at `1e-12` before any timing. Report original-default compatibility
separately; full-Server equal work remains open. Complete Source review before
one consolidated build/protocol/correctness/timing qualification and required
normal hook/hosted checks. These checks and independent official audit passed;
baseline and CI correction `e829be51` are accepted after exact hosted `41481267132`.

The configured local batch and independent data audit passed full100 correctness;
its measured window shows Rust 2.043246 times slower than native. Actual paired
observations/build/maps/timing proof is retained in the compact reference archive
and measurement summary. Required local, hosted and independent official
qualification is complete. No additional timing or unchanged successful
protocol/native build replay is required.

### PERF-01: per-pass articulated contact factorization

Root owns integration in `work/contact-factorization`; the physics owner supplied
`joint_state.hh`, `bridge.cc`, appended static-contact regression controls and
[DART_CONTACT_FACTORIZATION.md](DART_CONTACT_FACTORIZATION.md). The candidate
reuses validation and LLT factorization only within one immutable contact query
pass, keyed by skeleton. No persistent state, numerical threshold, response
expression or supported topology changes. Existing test assertions, fixtures,
baseline measurement summary and observation archive remain unchanged.

Production and regression source review is complete. Root updated the benchmark's
source-established bridge binding and documented the historical baseline boundary;
these changes passed independent source review. The first candidate gate failed on the historical native header binding before
replay compilation; three new regressions passed, later checks and final coverage
remain unrun. The correction preserves the exact original oracle and header in
an explicit output-only replay projection, with source/dependency rejection
controls. Baseline and CI correction `e829be51` are accepted after exact hosted
run `41481267132` and independent official audit, and that parent is integrated.
Complete independent correction review before the affected required gate.
Only after correctness passes may a newly built, source-bound actor enter one
reviewed equal-work comparison. No measured hotspot or speedup is claimed yet.

The REF-03 configured baseline is now accepted at `e829be51`: complete local
checks, exact hosted `41481267132` and independent official audit passed. It
retains 924 headless, 546 renderer and 249 tooling checks. The accepted immutable archive reports the slower Rust result without relabeling
failed snapshot attempts. Broader P0 and full-Server performance remain open.

### SDF-02 / LOAD-01: accepted primitive inertia consumer adoption

Root owns `work/sdf-capsule-inertia-adoption`, based on reviewed simulator `f136e821`.
Owned files are Cargo.toml/Cargo.lock (exact accepted producer pin only), the new
Scene primitive-inertia regression and selected native fixture, this task/handoff
entry and [SDF_PRIMITIVE_INERTIA_ADOPTION.md](SDF_PRIMITIVE_INERTIA_ADOPTION.md).
Adopt only producer `a44bf8be`: centered cylinders/capsules and its earlier bounded
posed box/sphere support. Do not implicitly adopt the later translation candidate.
Retain all runtime/backend admission and density semantics; check inherited native
mass/tensor and four-backend force/reset/removal behavior without changing old tests.
Complete independent source review before one required gate. Parent `f136e821`
is accepted; coordinate actual disk usage with the running renderer gate. No compiler, tests, lint or native capture has run.

This integration also publishes the independently reviewed contact-factorization
measurement appendix (`reference/contact_physics_measurement.json`, its separate
141-member observation archive and `docs/CONTACT_PHYSICS_MEASUREMENT.md`). The
measured simulator/producer identities remain `f136e821`/`f0d05146`, distinct from
the new consumer pin. Original baseline evidence stays immutable. Both source
batches are complete before the same required consumer integration gate; no new
or repeated timing batch is implied.

### RENDER-02: bounded programmatic metric depth (qualification correction)

The renderer owner has completed calibrated 64x64 box depth readback using Bevy's
actual stored depth texture and projection inverse. Root integrates accepted
consumer parent `c5dd02c0` and coordinator documentation; executable/test payloads
remain the reviewed renderer files. The first full gate's 549 renderer passes
include all three new depth tests, but regions 1009/1066 failed the strict
per-crate threshold. A reviewed append-only lifecycle/admission/completion-error
test batch preserves production code, old assertions and numerical bounds.
Complete this final source batch before one affected normal gate, exact hosted
qualification and independent official audit. No corrected result is assumed.
Near-plane intersections and full loaded/native depth sensor behavior remain open.
