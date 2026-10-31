<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Coordinator handoff

Updated 2026-10-31. Read this first when resuming coordination of the rewrite,
whichever agent you are. AGENTS.md has the standing rules; this file has the
current state and the next tasks. Update both when either changes.

## Current private Zenoh integration (2026-10-31)

The migration remains an unaccepted working tree at
`artifacts/direct-zenoh-consumer-worktree`. Public Main e945288 and hosted run
41206243415 describe the historical runtime. Root owns integration; never edit
Fuel. The Rust workspace no longer depends on gz-transport, the old Messages
runtime or ZeroMQ. Consumers use standard generated protobuf bindings.

Reviewed runtime work includes ServerOptions, IO-owned control/removal queries,
clock/pose/IMU publication, scalar rate/force ingress and bounded response
channels. `prepare_io` performs reset setup outside raw ticking; `update` combines
phases. Real controls cover rate retirement, canceled setup, queued shutdown and
admitted-response discard accounting. Complete ACT-01 absence and stopped-peer
pressure acceptance remain open. An earlier native rate no-reply failure has
unknown cause; its fixture now follows the actual persistent-querier readiness
contract before one request, without retries or relaxed deadlines.

The real source-built `gazebo-zenoh-client-v1` reference is qualified at
`artifacts/zenoh-native-reference`. Signed snapshot packages, source/tool identity,
installed closure and the locked ROS archive are checked. Identical archive bytes
were recovered over valid official mirror HTTPS after the canonical URL's TLS
hostname failure; no verification or pin was weakened. Keep SDK bootstrap inputs
unchanged while using this reference. Copied snapshots set
`GZ_ZENOH_REFERENCE_ROOT` to its qualified absolute root.

All four retained C++ fixtures compile through the selected-reference builder.
Real server, rate, force and CLI scenarios pass against this actual reference;
current camera worker/GPU replay is still required. Nine direct same-session
camera fault scenarios pass. The CLI's removed invalid-GZ_IP option has a reviewed
translation to malformed explicit TCP configuration, retaining exit/no-output
assertions.

A complete local headless execution now passes 783 tests, all three original
timing controls and real C++ peer checks. Five delegated child entrypoints are
verified through their harnesses, not counted as skipped domain cases. All 12
headless crates pass strict line and region coverage: workspace 9757/9931 lines
and 14931/15372 regions; server 2401/2491 and 3478/3655. Changed coverage passes:
proto 177/178, CLI 4/4 and server 1591/1654. Evidence is under
`/tmp/sdf-full-headless-8u55xnn1`; this is local qualification, not a staged,
hosted, renderer or comparative-performance pass.

Two findings were repaired without changing production assertions or thresholds.
A serial orchestration override changed libtest's child receipt formatting; it
was removed after a real one-seed comparison. Zenoh matching callbacks are
asynchronous: test clients now await actual IMU readiness outside stepping, and
last-drop retirement before the original idle steps. Frame deadlines, timestamps,
sequences, counters, noise oracles and advancing-step counts remain unchanged.
All 141 server controls and the five focused interest controls pass.

Coverage export exposed a shared unowned test module with three mappings. LLVM's
source filters also hid legitimate build-script aliases, so do not use them.
The reviewed `owned_coverage` helper selects whole JSON and LF LCOV records by
membership in the complete owned set. It never merges aliases or edits counters;
strict duplicate, missing-source, DA0 and >95% checks remain unchanged. The
original full JSON first passes the generated-binding guard, including rejection
of unexpected generated siblings. Both unfiltered artifacts are retained. All
57 executable owned JSON records and LCOV blocks match the original data exactly,
including 122 zero-hit entries; only the syntax-audited proto facade is absent.
New integrity controls and independent replay pass.

The complete renderer diagnostic also passes: 435 tests, all ten production
crates above the unchanged strict thresholds, 5058/5191 lines and 7523/7830
regions. Actual software Vulkan worker and C++ camera evidence are retained.
Two reviewed camera boundary tests raised camera regions from 555/586 to 557/586
with unchanged production code and prior tests. Their real Image wire, payload
budget, closed-publisher and constructor-cleanup assertions pass. The renderer
uses the frozen production snapshot plus those tests; it is not a combined-tree
staged or hosted result.

A separate reviewed follow-up releases the CPU IMU stream buffer after the last
consumer and on both committed-reset failures. Four strengthened existing
lifecycle controls each reproduced retained capacity four before the fix and now
pass with capacity zero, including independent retained-ELF replay. Original
Weak-lease, frame, epoch, ID restoration, sequence and physical assertions remain.
This is bounded allocation retirement; full ACT-01 and shutdown remain open.

The real `gazebo-zenoh-headless-v1` application build is qualified locally. Its
separate same-image client, rebuilt Sensors/Sim, unchanged Physics and stationary
Clock/Pose/IMU smoke passed. The immutable local image is
`sha256:bae6ddc3a806532805a8103967d7ab353fdaf14ab0fcb957343f8f32bece11da`;
fresh read-only validation passed. Proof is under
`artifacts/zenoh-headless-build/qualified-evidence`. The new overlay build paths
avoid a reproduced Docker lower-directory EXDEV error. A root-owned in-container
copy of six source caches resolved Git ownership rejection without a global
ownership bypass, host chown, package change or input waiver.

The untimed smoke proves a common stationary 1ms sample and bound native library
closure. It does not replace the unchanged moving benchmark's stronger metadata,
covariance, header, timing and motion predicates. No comparative performance or
native gtest execution credit comes from compiling the application targets.

Current lanes:
- SDF independently checks the native reference/image evidence after completing
  both profile diagnostics and preserving their source/object/raw bindings.
- The former Transport owner qualifies a disposable runtime container for the
  actual optimized Rust benchmark ELF. Only eight missing DART-family shared
  libraries may be bound read-only; existing common libraries must stay unchanged.
- The former Messages owner independently reviews the runtime mounts, exact ELF
  closure and source-bound release record. Root coordinates measurements and the
  combined-tree staged/hosted gates.

A release server test executable has built with the pinned Rust compiler,
optimization level three, no debug assertions and no instrumentation. A complete
source/Git-tree/Cargo-JSON/ELF record must match the final source before measurement.
Its DART bridge requires eight additional runtime files even for loading this
TPE-only test binary. Loader qualification must precede any Rust test main; do not
replace the immutable image's common libraries or relax the benchmark checks.
MuJoCo is lazy-loaded and is not a dependency of this TPE workload.

The CI workflow uses exact signed snapshot packages in private APT state and
verifies the ROS archive. Resolver and integrity controls pass; the complete local
tooling suite passes 217 tests. Current normal staged verification and hosted CI
are still required. Next: qualify the runtime overlay, run quiet equal-work
comparisons with the unchanged workload, finish pressure/absence work, and run the
normal staged hook and exact hosted gates. Publish only reviewed checked changes.
Never run comparative timings during native or Rust builds. Keep at least 1 GiB
host free space; reclaim only confirmed inactive compiler caches and preserve
source, ELF, dep-info, native objects, raw profiles and reports.

## Owner correction: Zenoh and schema-only Messages

D28/D29 supersede the transport and Messages runtime rewrite. Stop all old
ZeroMQ parity and message codec/runtime/CLI work. Use upstream Zenoh directly in
gz-sim-rs; gz-msgs-rs retains only canonical proto sources and required schema
metadata/licensing. Generate bindings in each consumer. See
ZENOH_SCHEMA_MIGRATION.md for the staged transition and acceptance criteria.
Existing checked-in pins and historical C++ peer results still describe the old
runtime; they do not qualify Zenoh. Preserve them until migration is verified.
SDF, Fuel, physics, rendering and built-in activation work continue.

## Owner correction: built-in systems

D26 supersedes the earlier system C ABI/host plan. All simulation systems live in
gz-sim-rs and participate only when loaded world/models need them. Keep legacy SDF
plugin metadata/configuration mapping; do not load system libraries. Native engine
FFI remains allowed. ACT-01 defines activation, last-consumer teardown and zero idle
system resource/work evidence.

The current implementation does not yet meet the full absence requirement:
empty force ingress is still polled, generic input/physics callbacks
remain installed, and force workers/templates survive last-consumer removal
for reset. Joint preflight and committed/reset sampling now follow loaded
consumer demand; see JOINT_SYSTEM_LIFECYCLE.md for last-consumer retirement,
loaded reset, cross-family observer handling and explicit failure scope.
Persistent effort application now follows configured or typed controller demand;
see JOINT_FORCE_SYSTEM_LIFECYCLE.md for zero-command and reset semantics.
The bounded IMU Scene lifecycle now installs sampling/reset callbacks only for
live sensors and retires them and interest state after the final consumer; see
IMU_SYSTEM_LIFECYCLE.md for reset, failure and remaining server-worker scope.
Audit all implemented families and fix these through reviewed lifecycle slices.
Do not claim the previous bounded-force acceptance proves zero-cost absence.

## History

- 2026-10-24 to 2026-10-27: a ChatGPT/Codex coordinator ran P0-P2 with one
  agent per repository. Its local evidence is in ignored `artifacts/` and its
  transcripts are under `~/.codex/sessions`; nothing there is required to
  continue.
- 2026-10-27 to 2026-10-29: a Claude coordinator took over. It landed the
  networked CLI, closed P2, added the shared physics backend contract and the
  native DART and MuJoCo adapters. The commits are listed below.

## Current P3 follow-up

Canonical Physics world metadata now maps to the existing built-in physics
system; see WORLD_SYSTEM_CONFIGURATION.md. Explicit backend selection is retained,
with no dynamic system loader or duplicate callback installation. A bounded canonical engine filename is accepted under that explicit override,
so native input fixtures now load without removing the record. Automatic engine
selection remains open. Bounded moving-parent support is described below;
the complete original ApplyJointForce fixture is still unqualified. Bounded static local +Z planes
now load through DART; see SCENE_PLANES.md for the default-inertia, centered
collision and backend restrictions and retained native finite-footprint behavior.

The joint-effort API adds `DynamicBackend::add_joint_effort`; its DART world-hinge
implementation was accepted at 1982aa0. The current extension supports MuJoCo's
world hinges through its native generalized-force array. Commands are deferred by joint ID, consumed only
by a successful advancing step, and retired with the joint/body or either reset.
DART checks its temporary world-wrench overlay before native mutation; MuJoCo
resolves current DOF addresses after model settlement. Public lifecycle/physical
controls and both C++ pulse oracles are documented in
[JOINT_EFFORT.md](JOINT_EFFORT.md). Rapier's extension covers world/fixed-parent
impulse and reduced hinges; its moving-parent effort remains open. DART's bounded
extension is recorded below. Finite limits remain open. Required staged/hosted evidence
and accepted status are recorded on the public dashboard.

Loaded scene input now exposes `queue_joint_effort` by public entity ID. It
checks the loaded joint's bindings and original backend owner before forwarding
to the existing deferred backend queue. No shadow queue or stepping system is
added. Scene controls exercise all three dynamic engines, pause/reset/retirement,
invalid admission and native pulse-rate parity. Core runtime faults are terminal;
there is no same-simulation retry contract. Programmatic `set_joint_force` now
stores a latest persistent value in a Bevy component and adds it once per
advancing step after preflight. Pure backend scalar validation preserves old
settings and pending inputs on admission failure. Both resets clear it. Recognized
ApplyJointForce model configuration now installs zero commands and captures only
immutable metadata for loaded reset. Native text probes require ASCII whitespace
rules and rejection of split lexical values; see APPLY_JOINT_FORCE_CONFIGURATION.md.
Bounded Double topic input now normalizes/remaps endpoints, fans out shared
subscriptions and updates these commands within a finite ingress budget. Prepared
workers survive loaded reset while old receiver queues and command values retire;
see JOINT_FORCE_TRANSPORT.md for wire epoch and failure limits. These primitives
do not justify complete original-system credit. Continue with the missing native
fixture capabilities and broader articulation/limit domains after qualification.

The DART runtime guard also checks active angular rows in contact-bearing groups
against the existing native-coordinate inverse-response ceiling. This addresses
implicit default-limit activation with tiny inertia; it does not enable finite
limits or change solver settings. See [DART_JOINT_BOUNDARY.md](DART_JOINT_BOUNDARY.md).

Finite position limits remain open. Preserve the existing strong first-crossing
controls rather than publishing a direct maintained `set_limits` wrapper:

- Rapier impulse angles lose winding; near-PI limits can miss the excluded arc.
  Reduced coordinates preserve winding, but Rapier 0.36.1 only enables its internal
  limit row once the coordinate is already outside the interval. At 12 rad/s and
  1 ms, an upper bound of 0.0106 rad is crossed to 0.012 rad on the first step,
  beyond the retained 1e-3 tolerance. Removing implicit damping and adding solver iterations did not
  resolve that predicate. Unbounded reduced construction is accepted at 50b28e5;
  keep existing scene solver selection unchanged.
- DART 6.13.2 tiny contacting bodies with finite limits can abort inside ODE.
  Private equilibration experiments address selected failures and have atomic
  failure/retry controls, but physical accuracy, native assertion domains and
  cost remain unqualified. Dantzig freezes friction bounds during its pivot
  sequence; a final-normal cone defect is not its frozen boxed-LCP residual.
  Do not invent a cutoff, change tolerances/settings, or claim broad safety
  from successful subprocess exits alone.

## Where things stand

| Commit | Change |
| --- | --- |
| `a549c52` | `gz-sim-rs world.sdf --server [--run]`: networked headless server, real C++ client tests |
| `1cc54cc` | World names sanitized like native `TopicUtils::AsValidTopic` |
| `d58223d` | P2 and SLICE-01 closed for the selected headless world ([acceptance/P2.md](acceptance/P2.md)) |
| `d35b3bf` | `gz_physics::DynamicBackend` contract, type-erased `PhysicsBackend`, `install` reporting via StepFault |
| `8e9cf52` | Rapier and the scene moved onto that contract |
| `074c29b` | Native MuJoCo 3.11.0 adapter ([MUJOCO.md](MUJOCO.md)), `--physics-engine mujoco` |
| `85dd4ab` | Native DART 6.13.2 adapter ([DART.md](DART.md)), `--physics-engine dart` |
| `ca37d5c` | Real C++ headless interoperability qualified on all four retained engines |
| `26ca44f` | Programmatic Rapier body-to-body revolutes |
| `9e8e056` | Initial two-link SDF articulation and endpoint/model lifecycle validation |
| `675c4e0` | Programmatic Rapier contact-exclusion groups and lifecycle evidence |

- The commits listed above passed the staged pre-commit gate and hosted CI. The [dashboard](https://gazebosim-rs.github.io/rewrite-progress/)
  shows P2 complete and P3 in progress.
- Engines: Rapier (full f64), DART, MuJoCo and TPE are selectable in the scene
  and CLI; the CLI also accepts the upstream `gz-physics-*-plugin` names.
  DART and MuJoCo match the native gz-physics plugins bit for bit or within
  1e-14 on their parity fixtures. Bullet stays rejected (decision D04).
- Rapier now has body-to-body revolutes and construction-time contact exclusion
  groups. Its scene loader supports top-level dynamic acyclic local-link revolute
  graphs, including world roots and disconnected components. See
  [ARTICULATED_SCENES.md](ARTICULATED_SCENES.md) for exact boundaries and evidence.
  DART's bounded world-rooted parent/child backend is accepted at `f60d3a3`;
  its scene integration at `506f94a` passed full local and hosted gates.
  Jacobian-cache repair `c5f73a1` is also qualified. MuJoCo body-joint
  construction remains unsupported.
- P3 remains open: native larger articulations, joint limits and damping,
  contact properties, meshes/planes/heightmaps, and conformance against
  gz-physics `test/common_test` for each backend. No general speedup is claimed.

## Owner decisions made during this handoff

Recorded here and, where they are product decisions, in DECISIONS.md.

- Process: keep the real gates and drop the earlier ceremony (see AGENTS.md,
  "Owner guidance (2026-10-27)").
- Autonomy: commit and push reviewed work to main, update the dashboard, and
  edit sdformat-rs, gz-msgs-rs and gz-transport-rs directly. Never edit
  gz-fuel-tools-rs. Show the owner a list before deleting non-regenerable files.
- P2 was closed for the selected world rather than waiting for every broad
  packet; broad packets keep their own exit criteria.
- D24: MuJoCo is built from upstream's vendored commit, not a prebuilt release.
- D25: DART uses the packaged OSRF build, whose assertions abort on
  ill-conditioned input. The adapter rejects such input with documented limits
  (minimum mass 1e-5 kg, anisotropy cap, conditioning rule) instead of linking
  a different DART. Where native DART itself diverges, the adapter returns a
  typed error; native Gazebo aborts.

Before changing the SDF consumer pin, audit newly admitted coupling metadata.
The current pin rejects mimic declarations before scene loading; a future
mimic-capable producer needs an explicit scene rejection until a solver implements
that coupling. Declared child frames must remain distinct from physical bodies.

## Local setup

- Native interop tests and the commit hook need a C++ transport reference with
  a sibling `environment.json`. This repo's `artifacts/native-reference/install`
  lacks it; use `GZ_REFERENCE_PREFIX=<gz-transport-rs>/artifacts/reference/install`
  or rebuild with `python3 scripts/bootstrap-native.py`.
- MuJoCo: `python3 scripts/bootstrap-physics.py`, then
  `MUJOCO_PREFIX=<repo>/artifacts/physics-reference/mujoco`.
- DART: `libdart6.13-dev libdart6.13-collision-ode-dev` from the OSRF apt
  repository (see `.github/workflows/ci.yml`). On this machine ROS Jazzy's
  vendored DART is on `LD_LIBRARY_PATH`; binaries are pinned with DT_RPATH and
  `DartBackend::new` refuses a different library, so that is handled.
- Commit with all three variables set; the hook runs the full gate (about 10
  minutes locally, 8 on hosted CI).
- Disk: on 2026-10-27, about 170 GB of regenerable build caches and old scratch
  were deleted with the owner's approval. About 190 old worktrees with
  uncommitted ChatGPT drafts remain under each repository's `artifacts/`, plus
  unpushed branches `work/reset-identities` (gz-sim-rs) and `socket2-fuel-compat`
  (gz-transport-rs). They are believed superseded but unaudited; archive before
  deleting, and only with the owner's approval.
- The DART and MuJoCo author branches (`worktree-agent-a5488f384295773d9`,
  `worktree-agent-a542e16c2bd136546`, worktrees under `.claude/worktrees/`) were
  squash-merged into `85dd4ab` and `074c29b`. They can be removed.

## Known tooling issues (2026-10-29)

- Shared owner guidance is installed in every coordinator-managed repository.
  SDF's existing hook still requires independent review for protected files;
  reviewers satisfy that check without adding frozen proposals or build archives.
- gz-transport-rs's hook fails in a shell with ROS Jazzy sourced: its native
    C++ test binaries in `gz-transport/build/bin` resolve `libgz-transport` and
    zenoh from `LD_LIBRARY_PATH` and fail to load, so the reference-ledger check
    reports every case missing. Its C++ peer build also needs
    `GZ_REFERENCE_PREFIX=<gz-transport-rs>/artifacts/reference/install`. Fix the
    environment (or the hook's library path handling) before the next
    transport commit.

## Next

1. DART server reset comparisons now cover both DART and Rapier, preserving
   TPE and all original assertions. The server pins the native library with
   DEP_DART_LIBDIR and tests it under a shadowing library path. Author and
   independent focused runs and the mandatory staged hook passed in published
   `c9449ac`; exact hosted run 40871693297 passed.
   Published `9cbb7cb` rejects MuJoCo hinge removal above mjMAXVAL and preserves
   projected velocity when removing an unstepped hinge. Both regressions were
   reproduced before the fix; independent review and the full staged gate passed.
   Exact hosted run 40873469093 passed. Published `0312c3c` also exercises real
   contact arena exhaustion through backend steps and reset; its exact hosted
   run 40875528990 passed. Unchanged real C++ CLI/server assertions are now
   qualified on DART and MuJoCo at `ca37d5c`; see NATIVE_BACKEND_INTEROP.md.
2. MuJoCo follow-ups (low, not reachable through the scene today): extreme
   inertial-centre offsets are admitted but fault the first step; the hinged load
   comparison against the native plugin is not like-for-like (the plugin adds
   actuators). Contact exhaustion now also has a real backend-step regression.
3. P3 capabilities, one reviewed slice at a time: general articulation (model
   joint trees, not only world revolutes), joint limits/damping/effort, contact
   surface properties, mesh/plane/heightmap shapes. Port the matching
   gz-physics `test/common_test` cases per backend; the upstream binaries are
   built in `gz-physics/build/bin/COMMON_TEST_*` and take a plugin path.
4. Continue P4 in parallel with P3, preserving the headless dependency boundary.

## Latest bounded qualifications

`f60d3a3` passed independent review, the unchanged full staged gate and exact
hosted run `41168474334`. Downloaded evidence matches all 60 source hashes and
25 build/check inputs against that commit, plus both local profile summaries.
This accepts a DART world hinge with one dynamic revolute child, including
coupled effort, reset and guarded topology changes. External articulation
contacts, active multibody limit rows and larger graphs remain unsupported.
The free-root follow-up is described below. The native oracle records its verified hybrid SDK provenance;
it is not a clean homogeneous reference build or performance qualification.

Scene commit `506f94a` preserves SDF declaration identities while constructing
the DART world hinge first, and removes physical leaves before parents. Its
four new analytical controls and existing body-joint assertions passed independent
review. Full local headless and renderer gates passed, including 60/60 changed
scene lines. Exact hosted run `41169945049` passed, with both downloaded profiles and
source hashes matching local evidence. The original
ApplyJointForce world has a free root and ground contact; adding a world hinge
would create a different fixture and earns no original-system completion.

The bounded free-root follow-up supports one dynamic parent and one revolute
child, with seven generalized coordinates. It preserves the extraction-cache
repair below and the existing guards on external contacts and active limits.
The exact previously rejected Scene XML now checks both endpoints, free-parent
translation and rotation under internal effort, sampling, loaded reset and
removal. Independent review regenerated all 34 native observations and approved
only three explicit rejection-to-positive transitions. Full unfiltered backend
and Scene body-joint tests pass. Ordinary strict staged and exact hosted evidence
must be checked before marking this slice accepted on the dashboard. See
[FREE_PARENT.md](../crates/gz-physics-dart/docs/FREE_PARENT.md). The original
ground-contact ApplyJointForce case is now covered by the bounded follow-up below.

The static-contact follow-up admits only a two-body free-root pair against static
geometry. It preserves world-root, dynamic external-contact and active-limit
refusals; the broader world-root experiment reproduced an upstream native abort.
Independent numerical/native review approved the conservative all-coordinate
response bound, valid anisotropy and condition-boundary controls, and exactly two
old static-contact assertion transitions. The unchanged original force-command
world passes fresh physical parity and a real C++ publisher to the Rust server.
Its single Linux original case is traced in the ledger with explicit API/component
translations and retained native baseline evidence. This does not complete the
system family or zero-cost absence. See
[APPLY_JOINT_FORCE_ORIGINAL.md](APPLY_JOINT_FORCE_ORIGINAL.md) and
[ARTICULATED_CONTACTS.md](../crates/gz-physics-dart/docs/ARTICULATED_CONTACTS.md).
Ordinary strict staged and exact hosted qualification remain mandatory.

Follow-up review found a DART extraction bug: rejected attachments and leaf
retirement could leave surviving native Jacobian caches zero while Rust state
appeared unchanged. Repair `c5f73a1` invalidates those caches at three structural
boundaries, preserving the admission domain and solver settings. Independent
clean-baseline regressions, 119 standard/native tests, native bridge coverage
498/521 lines and all nine changed executable lines passed. The full staged gate
and exact hosted run `41173269592` passed; downloaded Rust profiles and native
source hashes match the commit. See [TOPOLOGY_CACHE.md](../crates/gz-physics-dart/docs/TOPOLOGY_CACHE.md).

Canonical SDF camera snapshots are accepted at `5669d79`. An optional CPU-only
camera-output crate adds explicit acquisition and bounded publication through
the real Bevy worker, with actual C++ Image/CameraInfo delivery on software
Vulkan. Its private renderer profile and independent software-worker replay passed;
full Main headless/staged/hosted qualification remains pending. Keep real buffered-frame reset/removal/replacement controls and
independent protocol-error tests; those peers do not inject physical GPU faults.
Automatic cadence, native image parity, physical GPU qualification and full
sensor-family migration remain open. Read the tracker and retained evidence
before advancing these statuses.
