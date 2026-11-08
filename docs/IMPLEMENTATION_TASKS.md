# Implementation work packets

Each packet is a bounded assignment. Split broad packets by the named system or
test family before implementation; do not give an agent “port all sensors” with
no acceptance boundary. IDs below are stable coordination labels, not GitHub
issue numbers. No task is implemented merely because it appears in this plan.

Every packet follows AGENTS.md and produces an evidence record containing owned
files, upstream test IDs, dependency decisions, code revision, exact check commands,
results, coverage, relevant benchmark data, reviewer, and unresolved limits.
The task author cannot independently retire tests or approve coverage exclusions.

## Current delivery: M15 air pressure and M16 local OBJ

Owner D32 directs continuous delivery: select and start the next useful bounded
increment after acceptance while retained work remains. M1–M14 are accepted at
`2f32bc7e`; preserve all 154 reviewed slices and prior behavior/evidence.
[M15_M16_DELIVERY.md](M15_M16_DELIVERY.md) defines the current independent
five-day scopes, explicit file leases and required native/wire/browser proofs.
Root owns shared integration and one unchanged qualification cycle. No new
capability is accepted before actual proofs, all mandatory gates, independent
review/DATA and qualified Main promotion. Keep the web tracker current.

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
