# Implementation work packets

Each packet is a bounded assignment. Split broad packets by the named system or
test family before implementation; do not give an agent “port all sensors” with
no acceptance boundary. IDs below are stable coordination labels, not GitHub
issue numbers. No task is implemented merely because it appears in this plan.

Every packet follows AGENTS.md and produces an evidence record containing owned
files, upstream test IDs, dependency decisions, code revision, exact check commands,
results, coverage, relevant benchmark data, reviewer, and unresolved limits.
The task author cannot independently retire tests or approve coverage exclusions.

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
