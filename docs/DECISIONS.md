# Decision register

Updated with the owner’s built-in system decision on 2026-10-30. `Confirmed` means the owner specified or answered it.
`Proposed` is an architecture recommendation, not permission to remove behavior.
`Open` blocks only the implementation that depends on that decision.

| ID | Status | Decision |
| --- | --- | --- |
| D01 | Amended by D28/D29 | Organization: gazebosim-rs. Independent repos: sdformat-rs, gz-fuel-tools-rs, and schema-only gz-msgs-rs. No Rust transport repository in the target architecture. |
| D02 | Confirmed | gz-sim-rs contains simulator, sensors, physics, and residual math responsibilities. |
| D03 | Confirmed | Use Bevy ECS and renderer; remove gz-cmake and gz-rendering; separate GUI is out of scope. |
| D04 | Confirmed | Support DART, MuJoCo, TPE, and a Bevy-integrated physics option; remove both Bullet backends. |
| D05 | Confirmed | Prefer stable Rust libraries to rewriting math, common utilities, or other generic functionality. |
| D06 | Confirmed | Preserve behavior, file formats, protocols, and existing tests; new Rust APIs are allowed. No general C++ ABI promise. |
| D07 | Confirmed | Linux first, headless and Bevy rendering; design for later macOS/Windows. |
| D08 | Confirmed | Require >95% coverage, pre-commit enforcement, concise human code/comments, and measured performance improvement. |
| D09 | Proposed | Retire gz-utils; retain CLI dispatch and required common behavior as focused workspace modules. No new standalone utils/tools/plugin/common repositories. |
| D10 | Proposed | Evaluate Avian and Rapier as requested. Recommend Rapier f64 after finding Avian 0.8's mixed-precision solver change; confirm with P0 probes. DART remains the parity reference where upstream uses DART. |
| D11 | Proposed | f64 canonical state; integer simulation time; Z-up physics; explicit render-frame conversion and origin rebasing. |
| D12 | Proposed | >95% line AND region coverage per production crate and workspace; branch coverage is additional, toolchain-dependent evidence. |
| D13 | Confirmed | Use Gazebo main branches. Pin the inspected local main SHAs in planning/baseline.json; verify and review updates in P0 before baseline execution. |
| D14 | Superseded by D26 | Earlier runtime-loaded system C ABI requirement is withdrawn. |
| D15 | Confirmed | New GitHub repositories are public. Authentication now works; existing Fuel repository remains independently owned. |
| D16 | Proposed | Linux x86_64 is the initial required execution target; add Linux aarch64 after backend/GPU availability review. Confirm supported distro and hardware during P0. |
| D17 | Confirmed | Faithful Rust test ports may preserve fixtures, assertions, edge cases, and tolerances, verified against C++ reference runs. Explicitly dropped internals receive recorded dispositions. |
| D18 | Proposed | Keep existing external Python workflows in scope through focused bindings; do not port the Python system loader unchanged. Inventory actual use before accepting a narrower release. |
| D19 | Proposed | Native DART/MuJoCo libraries are allowed behind Rust adapters; “rewrite in Rust” does not mean rewriting those third-party engines. |
| D20 | Proposed | Performance budgets in PERFORMANCE.md are initial targets to ratify after baseline measurement. |
| D21 | Confirmed | Balanced robotics is the primary performance workload: articulated robots, many entities, and camera/lidar sensors. Also benchmark batch/headless and large worlds. |
| D22 | Confirmed | The Rust physics backend must use a full-f64 solver. Prioritize Rapier f64 and validate its capabilities, integration, accuracy, and performance with probes. |
| D23 | Confirmed | Correct native TPE static-model motion and rotated-link world-velocity quirks. Keep static models immobile and honor world axes; preserve existing assertions and record native differences. Implementation and verification remain required. |
| D24 | Confirmed | Native MuJoCo 3.11.0 is loaded at run time and built from upstream gz-physics' vendored commit (b85fdca5) with its CMake options by scripts/bootstrap-physics.py; parity fixtures compare to 1e-14, so prebuilt releases are not substituted. |
| D25 | Confirmed | DART uses the packaged OSRF 6.13.2 build (assertions enabled). The adapter rejects inputs that would reach those assertions with explicit, documented admission limits rather than linking a different DART build; these limits may be stricter than Rapier and MuJoCo. |
| D26 | Confirmed | All simulation systems are built into gz-sim-rs and activated by loaded world/model requirements. No dynamically loaded system libraries, system plugin host or system C ABI. An absent system has no scheduled work or system-specific runtime allocations, workers or endpoints; teardown follows its last consumer. Native engine FFI remains allowed. |
| D27 | Confirmed | Upgrade gz-sim-rs to Rust 1.97.1 for Bevy 0.20 rendering. Independent repository toolchains stay unchanged. Requalify the existing headless, native interoperability, coverage and hosted CI gates; preserve historical compiler-specific evidence. Default headless builds remain free of rendering/GPU dependencies. |

| D28 | Confirmed | Use upstream Zenoh directly in simulator consumers. Drop gz-transport-rs from the target architecture and drop ZeroMQ compatibility. Match C++ Gazebo over Zenoh, including topic keys, discovery/type metadata and service contracts. Stop its implementation, legacy protocol parity and optimization work. Preserve historical repositories/results while consumers migrate; no replacement transport framework. |
| D29 | Confirmed | gz-msgs-rs contains canonical protobuf definitions, their imported options/import closure, schema identity and licensing. Consumers generate their own bindings with maintained tools. Stop custom message runtime, reflection, conversion framework, CLI and codec optimization work. Preserve protobuf schema semantics and consumer behavior; do not move the discarded framework into gz-sim-rs. |

## Outstanding questions and decision triggers

- Baseline: main is confirmed. Freeze one complete compatible set, not a
  different convenient release per repository. Changes need a reviewed diff.
- Bevy backend: prioritize Rapier f64 after the owner's precision decision.
  Validate articulation, contact, determinism, integration, and latency. Avian 0.8
  does not satisfy the full-f64 solver gate; record that result without treating
  it as a release candidate. Do not assume latest means most mature.
- System activation: D26 supersedes D14. Implement demand-driven Bevy schedule
  composition and resource lifetime; legacy SDF plugin records select recognized
  built-ins without loading libraries. Preserve independent SDF metadata.
- Test fidelity: assertion-preserving ports are confirmed. Do not build a universal
  C++ facade unless a later explicit use case justifies its cost.
- Performance: select representative worlds and hardware in P0. Numeric targets
  remain proposals until baseline variability and achievable budgets are known.
- External GUI: agree scene, asset, clock, entity, command, and streaming contracts
  before P4/P6. No implementation of the GUI is part of this migration.

## Decision change procedure

An architecture decision records context, alternatives, selected behavior,
affected tests, dependency direction, migration cost, and measured evidence.
Changing confirmed scope requires the owner's decision. Routine implementation
choices within scope do not require repeated approval.

Do not leave a blocking product choice hidden in an agent's local notes. Record
it here, continue independent work, and keep dependent tasks visibly blocked.
