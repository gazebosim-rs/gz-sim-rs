<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Coordinator handoff

Updated 2026-10-29. Read this first when resuming coordination of the rewrite,
whichever agent you are. AGENTS.md has the standing rules; this file has the
current state and the next tasks. Update both when either changes.

## Owner correction: built-in systems

D26 supersedes the earlier system C ABI/host plan. All simulation systems live in
gz-sim-rs and participate only when loaded world/models need them. Keep legacy SDF
plugin metadata/configuration mapping; do not load system libraries. Native engine
FFI remains allowed. ACT-01 defines activation, last-consumer teardown and zero idle
system resource/work evidence.

The current implementation does not yet meet the full absence requirement:
joint callbacks remain installed unconditionally, empty force ingress is still
polled, and force workers/templates survive last-consumer removal for reset.
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
selection and moving-parent actuation remain open, so the complete original
ApplyJointForce fixture is still unqualified. Bounded static local +Z planes
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
impulse and reduced hinges. Moving-parent effort and finite limits remain open. Required staged/hosted evidence
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
  Native DART/MuJoCo body-joint adapters remain unsupported.
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
   run 40875528990 passed. The next slice extends unchanged real C++ CLI/server
   assertions to DART and MuJoCo; see NATIVE_BACKEND_INTEROP.md.
2. MuJoCo follow-ups (low, not reachable through the scene today): extreme
   inertial-centre offsets are admitted but fault the first step; the hinged load
   comparison against the native plugin is not like-for-like (the plugin adds
   actuators). Contact exhaustion now also has a real backend-step regression.
3. P3 capabilities, one reviewed slice at a time: general articulation (model
   joint trees, not only world revolutes), joint limits/damping/effort, contact
   surface properties, mesh/plane/heightmap shapes. Port the matching
   gz-physics `test/common_test` cases per backend; the upstream binaries are
   built in `gz-physics/build/bin/COMMON_TEST_*` and take a plugin path.
4. Then P4 (Bevy rendering and sensors) per PLAN.md.
