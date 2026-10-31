# Implementation plan

## Outcome and scope

Build a Rust Gazebo simulator that preserves the supported behavior of the pinned
upstream main branches, uses Bevy ECS and rendering, and demonstrates measurable
performance improvement. Use existing Rust libraries wherever their semantics
fit. This is a redesign of the implementation, not a translation of C++ classes.

The required end state is four repositories in `gazebosim-rs`:

1. `gz-sim-rs`: simulator runtime, systems, sensors, physics adapters, residual
   robotics math, asset integration, CLI, and demand-driven built-in systems.
2. `sdformat-rs`: SDF syntax, schema conversion, semantic graphs, and model DOM.
3. `gz-msgs-rs`: canonical protobuf definitions and their import closure, schema
   identity and licensing. Each consumer generates its own language bindings.
4. `gz-fuel-tools-rs`: existing independently owned Fuel rewrite.

Use the upstream Zenoh Rust library directly. There is no gz-transport-rs release
target and no ZeroMQ compatibility requirement. D28/D29 supersede the earlier
transport and Messages runtime work; see ZENOH_SCHEMA_MIGRATION.md.

There are no replacement repositories for gz-cmake, gz-rendering, or gz-gui.
The proposed disposition of gz-tools, gz-utils, gz-plugin, and gz-common is in
`REPOSITORY_AUDIT.md`. Retiring a library does not retire useful behavior.

## Compatibility contract

The owner confirmed that new Rust APIs are allowed. Preserve observable behavior,
SDF and resource formats, message schemas, transport protocols, CLI workflows,
and existing test assertions for retained features. D28 explicitly removes
ZeroMQ protocol compatibility; D29 removes the custom Messages runtime/tooling.
Record those test dispositions separately from retained simulator and protobuf
behavior. Historical passes remain historical, not Zenoh acceptance. A C++ ABI or source-compatible
public facade is not a general release requirement. Fuel's independently agreed
C++ compatibility remains its owner's responsibility.

Existing tests remain the acceptance specification. Where their C++ APIs disappear,
port fixtures, inputs, expected values, failure cases, tolerances, and assertions
to Rust and cross-check against the original executable. Keep unchanged upstream
tests wherever a small harness can exercise the replacement. A source-code port
is not evidence until both behavior and assertion equivalence have been reviewed.

Tests solely about removed Bullet, Ogre, GUI, or CMake internals get explicit
scope dispositions. Tests covering retained rendering, sensor, CLI, or system
behavior are rehosted, even if the old implementation disappears. Tests solely
about the C++ ECS's storage mechanics need a reviewed mapping to Bevy-facing
contracts. Never claim every literal C++ test runs unchanged when it does not.

The first complete release supports Linux headless and Bevy rendering. A
headless server with no rendering sensors must not need a GPU or display. A
headless server with cameras may need an offscreen GPU. macOS/Windows follow
after platform-neutral boundaries and Linux acceptance are established.

## Built-in system activation

All simulation systems are integrated into gz-sim-rs. Loaded world/model needs
select their configuration, ECS state and schedule participation. Recognized
legacy SDF `<plugin>` records configure built-ins; they never load a system
library. D26 supersedes the earlier external system C ABI requirement. Native
DART/MuJoCo engine adapters remain ordinary backend integration boundaries.

An absent system has no scheduled callback, idle polling, system-owned heap state,
worker, timer, subscription or publisher. Shared simulator infrastructure and
compiled code are separate from per-system runtime state. Maintain demand on
load/spawn/remove/reset events, remove participation after the last consumer,
and reconstruct resources before the next step when demand returns. Activation
must preserve phase order, pause/reset behavior, multiworld isolation and failures.
Verify absence with invocation/resource counters and lifecycle checks, then measure
CPU and memory on equal-work reference workloads. Empty ECS queries alone do not
satisfy this requirement. See ACT-01 in IMPLEMENTATION_TASKS.md.

## Sequence and gates

Each phase exits on evidence, not elapsed time. No reliable calendar estimate
exists until P0 measures the suite, assets, backend capabilities, and available
CI hardware. Re-estimate after the first vertical slice.

| Phase | Work | Exit evidence |
| --- | --- | --- |
| P0: Freeze and measure | Pin upstream main SHAs, build the C++ reference, enumerate tests and system behavior, collect baseline workloads, evaluate Avian/Rapier, define demand-driven system activation | Reproducible reference image and commands; complete test ledger; benchmark raw data; backend decision; explicit environment failures |
| P1: Foundations and gates | Create Cargo workspace and independent repo contracts, pin versions/toolchain, implement required pre-commit/CI, time/frames/IDs, message and SDF minimums | Gate failure demonstrations; >95% coverage on initial production code; dependency graph checks; no window/GPU dependency in the core build |
| P2: End-to-end server | Load a simple SDF world, spawn entities, step TPE and one dynamic backend, publish clock/poses and IMU through Zenoh, pause/reset/remove | Original and Rust runs agree on selected assertions; C++ peer interoperates through the selected Zenoh contract; bounded queues; first measured performance report |
| P3: Physics and robotics | Complete capability-based DART, MuJoCo, TPE, and chosen Rust integration; controllers, joints, contacts, lifecycle | Per-backend conformance results and negative capability tests; DART same-engine parity; no silent fallback; articulation and dynamic topology probes |
| P4: Bevy rendering and sensors | Asset pipeline, offscreen rendering, camera family, lidar, scheduling/noise, environmental and non-rendered sensors | Full sensor inventory mapped; GPU fixture tests and analytic tolerances; deterministic timestamps; GPU/CPU overload and reset tests |
| P5: Remaining simulator behavior | All remaining 78-system inventory entries, world services, levels/distributed behavior, recording/replay, scripting contracts | Every retained behavior has an implementation and tests; no unexplained unmapped cases; legacy client and log compatibility evidence |
| P6: Integration and optimization | External GUI contract, mixed-version repos, packaging, profiling, bottleneck removal, lifecycle stress | Performance budgets met at equal fidelity; long-run tests; package installation and clean shutdown on supported Linux targets |
| P7: Release qualification | Full suite, release feature matrix, coverage, documentation, upgrade story, license provenance | All required jobs pass on release commit; >95% thresholds; no unresolved retained-scope test failures; reproducible release artifacts |

P3 and P4 work can overlap once their P2 contracts are stable. Independent repo
teams can develop against shared fixtures and versioned contracts during P0/P1.
Do not let provisional adapters turn into permanent reverse dependencies.

## First vertical slice

Use a small world with gravity, a ground plane, a freely moving body, an IMU,
and a simple joint variant. Resolve SDF frames before constructing physics.
Support loading local resources, start-paused, exact N-step execution, clock and
pose publication, velocity/force input, removal, reset, and shutdown.

Run it in Rust against a real DART or selected Rust backend and against TPE for
its supported motion semantics. Compare DART to upstream DART at the same
version, timestep, solver settings, shapes, and precision. Use separate analytic
expectations for TPE. The network test must involve an actual C++ peer using the selected Zenoh
contract; a historical ZeroMQ peer result does not qualify the migrated path.

P2 may use an explicitly temporary SDF bridge if the independent
Rust implementation is not ready. Keep it behind a feature, record the owner and
removal milestone, test the same contract, and exclude it from any “pure Rust
stack complete” claim. It must not become the default release dependency.

## Repository and release mechanics

- Use a Cargo workspace in gz-sim-rs. Separate crates by dependency boundaries and
  build profiles; use modules for cohesive features that do not need a crate.
- Keep one reviewed Bevy version across the workspace. Commit Cargo.lock for
  applications, the exact Rust toolchain, native engine versions, dependency
  feature selections, and reference image digests.
- Independent repositories publish semver contracts; a checked-in integration
  manifest pins a tested set of commits/releases. Do not rely on floating Git
  branches or developer-only path dependencies in a release.
- Use ordinary Cargo/build.rs and narrowly scoped native bridge build support.
  Dropping gz-cmake does not prohibit CMake used internally by DART, a reference
  harness, or another third-party native dependency.
- Initially install `gz-rs` to coexist with upstream `gz`. A distribution package
  may provide the `gz` name after ownership/conflict handling is defined. Exactly
  one package owns that executable. There is no standalone Messages or transport
  CLI release requirement.
- Commit this plan in public gz-sim-rs. Create the other public repos after first
  inspecting existing org repos. Do not create or
  push Fuel content over its owner's checkout. No empty implementation claim is
  justified by creating repository shells.

## Required handoff artifacts

Each completed task provides code, original test IDs and Rust mappings, capability
changes, commands/results, coverage JSON, performance data where applicable,
public contract updates, and known limitations. Each phase produces a short
acceptance report with links to immutable artifacts and the tested commit.

The release report distinguishes native backend code from rewritten Gazebo code,
CPU from GPU coverage, original versus ported tests, same-engine from cross-engine
benchmarks, and tested versus unsupported feature combinations.

## Highest-risk work

| Risk | Early probe | Required response |
| --- | --- | --- |
| New ECS changes update ordering | Priority, command visibility, pause, and sensor phase tests | Explicit schedule sets and barriers; preserve externally observable order |
| Rust backend lacks robotics fidelity | Motor limits, loops, contacts, articulation, force/torque fixtures | Capability report; retain DART/MuJoCo; choose backend on evidence |
| Rendering changes sensor physics | Known-depth, segmentation, distortion, thermal, and lidar scenes | Dedicated sensor pipelines and error budgets; no game-camera shortcut |
| SDF parser is treated as XML only | Nested includes, frame graphs, schema conversion, inertia fixtures | Independent semantic parser milestone; resolve before instantiation |
| Interop works only Rust-to-Rust | C++ discovery/service/pub-sub/log fixtures | Mixed-language matrix before P2 exit |
| Assets lose semantics | Collada/STL/OBJ/glTF, skinning, PBR, scale, heightmap corpus | Measured loaders or constrained native fallback, provenance preserved |
| >95% hides feature gaps | Disabled-backend and absent-file coverage checks | Required configuration matrix and per-crate denominators |
| Apparent speedup reduces fidelity | Same-engine controlled comparisons | Freeze accuracy settings; performance acceptance follows correctness |
| Independent repos become coupled | Compile SDF/Fuel independently; generate consumer bindings from schemas | One-way dependency rule and minimal data contracts |

The detailed work packets are in `IMPLEMENTATION_TASKS.md`. Open owner choices
are recorded in `DECISIONS.md`; they do not prevent unrelated P0 work.
