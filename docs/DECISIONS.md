# Decision register

Updated with the owner’s built-in system decision on 2026-10-30. `Confirmed` means the owner specified or answered it.
`Proposed` is an architecture recommendation, not permission to remove behavior.
`Open` blocks only the implementation that depends on that decision.

| ID | Status | Decision |
| --- | --- | --- |
| D01 | Amended by D28/D29 | Organization: gazebosim-rs. Independent repos: sdformat-rs, gz-fuel-tools-rs, and schema-only gz-msgs-rs. No Rust transport repository in the target architecture. |
| D02 | Confirmed | gz-sim-rs contains simulator, sensors, physics, and residual math responsibilities. |
| D03 | Confirmed | Use Bevy ECS and renderer; remove gz-cmake and gz-rendering; separate GUI is out of scope. |
| D04 | Superseded by D33 | Support DART, MuJoCo, TPE, and a Bevy-integrated physics option; remove both Bullet backends. |
| D05 | Confirmed | Prefer stable Rust libraries to rewriting math, common utilities, or other generic functionality. |
| D06 | Confirmed | Preserve behavior, file formats, protocols, and existing tests; new Rust APIs are allowed. No general C++ ABI promise. |
| D07 | Confirmed | Linux first, headless and Bevy rendering; design for later macOS/Windows. |
| D08 | Confirmed | Require >95% coverage, pre-commit enforcement, concise human code/comments, and measured performance improvement. |
| D09 | Proposed | Retire gz-utils; retain CLI dispatch and required common behavior as focused workspace modules. No new standalone utils/tools/plugin/common repositories. |
| D10 | Amended by D22/D33 | Rapier f64 is the selected Rust backend. DART product-backend parity is cancelled; existing upstream observations may remain reference fixtures for retained behavior. |
| D11 | Proposed | f64 canonical state; integer simulation time; Z-up physics; explicit render-frame conversion and origin rebasing. |
| D12 | Proposed | >95% line AND region coverage per production crate and workspace; branch coverage is additional, toolchain-dependent evidence. |
| D13 | Confirmed | Use Gazebo main branches. Pin the inspected local main SHAs in planning/baseline.json; verify and review updates in P0 before baseline execution. |
| D14 | Superseded by D26 | Earlier runtime-loaded system C ABI requirement is withdrawn. |
| D15 | Confirmed | New GitHub repositories are public. Authentication now works; existing Fuel repository remains independently owned. |
| D16 | Proposed | Linux x86_64 is the initial required execution target; add Linux aarch64 after backend/GPU availability review. Confirm supported distro and hardware during P0. |
| D17 | Confirmed | Faithful Rust test ports may preserve fixtures, assertions, edge cases, and tolerances, verified against C++ reference runs. Explicitly dropped internals receive recorded dispositions. |
| D18 | Proposed | Keep existing external Python workflows in scope through focused bindings; do not port the Python system loader unchanged. Inventory actual use before accepting a narrower release. |
| D19 | Superseded by D33 | Native DART/MuJoCo libraries are allowed behind Rust adapters; “rewrite in Rust” does not mean rewriting those third-party engines. |
| D20 | Proposed | Performance budgets in PERFORMANCE.md are initial targets to ratify after baseline measurement. |
| D21 | Confirmed | Balanced robotics is the primary performance workload: articulated robots, many entities, and camera/lidar sensors. Also benchmark batch/headless and large worlds. |
| D22 | Confirmed | The Rust physics backend must use a full-f64 solver. Prioritize Rapier f64 and validate its capabilities, integration, accuracy, and performance with probes. |
| D23 | Confirmed | Correct native TPE static-model motion and rotated-link world-velocity quirks. Keep static models immobile and honor world axes; preserve existing assertions and record native differences. Implementation and verification remain required. |
| D24 | Historical; superseded by D33 | Native MuJoCo 3.11.0 is loaded at run time and built from upstream gz-physics' vendored commit (b85fdca5) with its CMake options by scripts/bootstrap-physics.py; parity fixtures compare to 1e-14, so prebuilt releases are not substituted. |
| D25 | Historical; superseded by D33 | DART uses the packaged OSRF 6.13.2 build (assertions enabled). The adapter rejects inputs that would reach those assertions with explicit, documented admission limits rather than linking a different DART build; these limits may be stricter than Rapier and MuJoCo. |
| D26 | Confirmed | All simulation systems are built into gz-sim-rs and activated by loaded world/model requirements. No dynamically loaded system libraries, system plugin host or system C ABI. An absent system has no scheduled work or system-specific runtime allocations, workers or endpoints; teardown follows its last consumer. Native engine FFI remains allowed. |
| D27 | Confirmed | Upgrade gz-sim-rs to Rust 1.97.1 for Bevy 0.20 rendering. Independent repository toolchains stay unchanged. Requalify the existing headless, native interoperability, coverage and hosted CI gates; preserve historical compiler-specific evidence. Default headless builds remain free of rendering/GPU dependencies. |

| D28 | Confirmed | Use upstream Zenoh directly in simulator consumers. Drop gz-transport-rs from the target architecture and drop ZeroMQ compatibility. Match C++ Gazebo over Zenoh, including topic keys, discovery/type metadata and service contracts. Stop its implementation, legacy protocol parity and optimization work. Preserve historical repositories/results while consumers migrate; no replacement transport framework. |
| D29 | Confirmed | gz-msgs-rs contains canonical protobuf definitions, their imported options/import closure, schema identity and licensing. Consumers generate their own bindings with maintained tools. Stop custom message runtime, reflection, conversion framework, CLI and codec optimization work. Preserve protobuf schema semantics and consumer behavior; do not move the discarded framework into gz-sim-rs. |

| D30 | Confirmed (2026-11-02) | Deliver a runnable MVP first: Rapier f64 with simple primitive shapes, visualized live using gzweb-rs. Then mesh visuals, mesh collisions, individual sensor models and focused Gazebo systems in phases each budgeted at most five working days, including review and mandatory verification. The full rewrite becomes a deferred backlog, not a prerequisite for a usable release. Preserve existing backends, evidence, assertions and quality gates; do not count deferred work as complete. See DELIVERY_PLAN.md. This request authorizes planning/tracker changes; implementation agents and automation remain paused until explicit resume. |
| D31 | Confirmed execution plan (2026-11-07 owner continuation/parallel request) | Post-MVP independent bounded increments may run in parallel with explicit file leases and one integrator. M13 ideal altimeter and M14 local STL both depend on M12. Freeze implementation, regression source, documentation and independent Source review in one candidate before one unchanged full normal hook and exact hosted CI. Acceptance becomes effective only at qualified Main promotion after actual feature proofs and independent DATA; record generated run IDs/time/refs in the existing immutable release/promotion receipt and owned status. No invented future IDs, gate bypass, weakened thresholds or result-reuse framework. See POST_MVP_DELIVERY.md. |

| D32 | Confirmed (2026-11-07 owner instruction) | Always continue with the next useful bounded phase while retained work remains. Do not stop because the previous schedule is complete or its successor was unselected. Record each scope/dependencies/acceptance before implementation, allow independent lanes under D31, keep the web tracker current and preserve all required review/verification gates. Stop for an explicit owner pause, completion of retained scope or a genuine blocking owner decision. The next selected wave is M15 ideal air pressure and M16 local OBJ; see M15_M16_DELIVERY.md. |

| D33 | Confirmed (2026-11-17 owner instruction) | Drop DART and MuJoCo entirely as product backends. Cancel PHY-03/PHY-04 and all remaining adapter implementation, parity, optimization and distribution work; remove their workspace crates, consumer dependencies/features and dedicated build/test/provisioning jobs. Keep Rapier f64 and TPE. Preserve historical results, pending worktrees, original fixtures and attribution; reference observations still required to test retained Rapier/system behavior are not product backends or new DART/MuJoCo development. Remaining-engine assertions, coverage thresholds, Source review and mandatory final checks stay intact. See BACKEND_RETIREMENT.md. |

D33 supersedes D04, D19, D24, D25 and D30's DART/MuJoCo retention obligation.
Earlier decisions and results remain historical evidence. DART-specific future
parity in D10 is cancelled; existing upstream reference fixtures may still test
retained behavior. Other D30 compatibility scope remains deferred, not completed.
It supersedes the old P0–P7 execution order and the earlier whole-rewrite-first
forecast. gzweb-rs is the MVP viewer; Bevy server-side rendering remains the
sensor implementation for later phases. The observed viewer currently needs a
live scene/pose/control connection. No new transport framework or system plugin
architecture is authorized. M1 acceptance must demonstrate the real browser.

## M77 correction checkpoint: camera startup and input admission

This isolated candidate starts at frozen `353dc9ae86aac56c7653f36a933d688b43fa6fbf`.
That Source passed local normal/DATA, the fresh four-role build, package DATA and
clean installation/47 finite loads. The first installed camera run then failed
once (`8d2438` to `e5ef74`, 28.31 seconds), before any SDK peer, in
`ubuntu-install-um5894io`: the default web snapshot guard required all-visible
flags while the unchanged M76 fixture has opaque visuals with flags 4/1/2.
The selected product correction admits nonzero flags under the viewer's implicit
all-bits mask; zero visibility, transparency, unsupported materials and all
existing snapshot bounds remain refused. Camera masking, protocols, viewer/GPU
bytes and solvers do not change.

The candidate retains the independently Source-reviewed wrench-test preconditions:
finite Block configuration, fresh per-key presence generations, the original
five-second deadlines and bounded polling, with every real put and assertion.

The reviewed `95ef6ced` correction then ran one full normal attempt and failed
Root/Git 1 (`29733` to `d26f30`, 1367.114675317 seconds). All available headless
targets finished with one failure: the wrench presence query unwrapped
`Err(Disconnected)` at line 259. Renderer checks were not reached. The frozen
Source, full raw packet `95ef6ced...-92rp2mvd`, receipt and failure record remain
under primary `artifacts/m77-delivery`; no success audit or qualification follows
from that run. This failure is a demonstrated harness EOF-contract error, distinct
from the still UNKNOWN cause of original `353dc9ae` hosted attempt 1. The unchanged
original-head hosted attempt 2 succeeded, without accepting that old package or
qualifying this changed successor.

The current successor retains the exact reviewed web visibility correction and
all original semantic puts/assertions. Pinned Zenoh finite-query completion drops
the FIFO sender: drained/disconnected EOF is accepted only with a freshly open
observer. `Ok(None)` remains absolute receive-deadline failure. Only the exact
Source-defined `Timeout` payload with `Encoding::ZENOH_STRING` marks a 100 ms
query incomplete; all partial tokens from every key in that attempt are discarded.
Metadata queries may repeat only inside the same original five-second readiness
deadline. Unexpected errors, encodings, duplicate timeout or post-timeout samples
fail. No real command is replayed and no deadline is extended.

Four new no-network FIFO/error regressions bring the correction census to nine
Rust declarations (headless 9, renderer 2, overlap 2), zero new Python declarations,
with the original M77's 13 Python cases retained. Finite/CLI inventories stay
47/21. Current implementation/regression/docs require whole independent Source
review; no current formatter, checks, package, native operation or publication has
run. Its future qualified commit/tree/run remain unknown. Accepted M76 and public
Latest M70 are unchanged. Old passes and failures remain bound to their own Source.


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
- MVP viewer (D30): agree the minimal live gzweb-rs scene/pose/control contract
  in M1. Connecting the existing viewer is in scope; a general GUI rewrite is not.

## Decision change procedure

An architecture decision records context, alternatives, selected behavior,
affected tests, dependency direction, migration cost, and measured evidence.
Changing confirmed scope requires the owner's decision. Routine implementation
choices within scope do not require repeated approval.

Do not leave a blocking product choice hidden in an agent's local notes. Record
it here, continue independent work, and keep dependent tasks visibly blocked.
