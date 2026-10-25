# Decision register

Status as of 2026-10-24. `Confirmed` means the owner specified or answered it.
`Proposed` is an architecture recommendation, not permission to remove behavior.
`Open` blocks only the implementation that depends on that decision.

| ID | Status | Decision |
| --- | --- | --- |
| D01 | Confirmed | Organization: gazebosim-rs. Independent repos: sdformat-rs, gz-fuel-tools-rs, gz-transport-rs, gz-msgs-rs. |
| D02 | Confirmed | gz-sim-rs contains simulator, sensors, physics, and residual math responsibilities. |
| D03 | Confirmed | Use Bevy ECS and renderer; remove gz-cmake and gz-rendering; separate GUI is out of scope. |
| D04 | Confirmed | Support DART, MuJoCo, TPE, and a Bevy-integrated physics option; remove both Bullet backends. |
| D05 | Confirmed | Prefer stable Rust libraries to rewriting math, common utilities, or other generic functionality. |
| D06 | Confirmed | Preserve behavior, file formats, protocols, and existing tests; new Rust APIs are allowed. No general C++ ABI promise. |
| D07 | Confirmed | Linux first, headless and Bevy rendering; design for later macOS/Windows. |
| D08 | Confirmed | Require >95% coverage, pre-commit enforcement, concise human code/comments, and measured performance improvement. |
| D09 | Proposed | Retire gz-utils; retain CLI dispatch and runtime extension responsibilities as focused workspace crates. No new standalone utils/tools/plugin/common repositories. |
| D10 | Proposed | Evaluate Avian and Rapier as requested. Recommend Rapier f64 after finding Avian 0.8's mixed-precision solver change; confirm with P0 probes. DART remains the parity reference where upstream uses DART. |
| D11 | Proposed | f64 canonical state; integer simulation time; Z-up physics; explicit render-frame conversion and origin rebasing. |
| D12 | Proposed | >95% line AND region coverage per production crate and workspace; branch coverage is additional, toolchain-dependent evidence. |
| D13 | Confirmed | Use Gazebo main branches. Pin the inspected local main SHAs in planning/baseline.json; verify and review updates in P0 before baseline execution. |
| D14 | Confirmed | Third-party native systems must load without rebuilding the simulator, through a narrow versioned C ABI. Built-ins use source-built Bevy plugins. |
| D15 | Confirmed | New GitHub repositories are public. Authentication now works; existing Fuel repository remains independently owned. |
| D16 | Proposed | Linux x86_64 is the initial required execution target; add Linux aarch64 after backend/GPU availability review. Confirm supported distro and hardware during P0. |
| D17 | Confirmed | Faithful Rust test ports may preserve fixtures, assertions, edge cases, and tolerances, verified against C++ reference runs. Explicitly dropped internals receive recorded dispositions. |
| D18 | Proposed | Keep existing external Python workflows in scope through focused bindings; do not port the Python system loader unchanged. Inventory actual use before accepting a narrower release. |
| D19 | Proposed | Native DART/MuJoCo libraries are allowed behind Rust adapters; “rewrite in Rust” does not mean rewriting those third-party engines. |
| D20 | Proposed | Performance budgets in PERFORMANCE.md are initial targets to ratify after baseline measurement. |
| D21 | Confirmed | Balanced robotics is the primary performance workload: articulated robots, many entities, and camera/lidar sensors. Also benchmark batch/headless and large worlds. |
| D22 | Confirmed | The Rust physics backend must use a full-f64 solver. Prioritize Rapier f64 and validate its capabilities, integration, accuracy, and performance with probes. |
| D23 | Open | TPE static-model and rotated-link world-velocity quirks: reproduce pinned native behavior or correct it with documented differences. Owner asked; only dependent adapter behavior is blocked. |

## Outstanding questions and decision triggers

- Baseline: main is confirmed. Freeze one complete compatible set, not a
  different convenient release per repository. Changes need a reviewed diff.
- Bevy backend: prioritize Rapier f64 after the owner's precision decision.
  Validate articulation, contact, determinism, integration, and latency. Avian 0.8
  does not satisfy the full-f64 solver gate; record that result without treating
  it as a release candidate. Do not assume latest means most mature.
- Runtime plugins: the C ABI requirement is confirmed. Freeze the descriptor,
  ownership, callback, threading, and lifecycle contracts during P1.
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
