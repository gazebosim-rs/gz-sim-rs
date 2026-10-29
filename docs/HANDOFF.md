<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Coordinator handoff

Updated 2026-10-29. Read this first when resuming coordination of the rewrite,
whichever agent you are. AGENTS.md has the standing rules; this file has the
current state and the next tasks. Update both when either changes.

## History

- 2026-10-24 to 2026-10-27: a ChatGPT/Codex coordinator ran P0-P2 with one
  agent per repository. Its local evidence is in ignored `artifacts/` and its
  transcripts are under `~/.codex/sessions`; nothing there is required to
  continue.
- 2026-10-27 to 2026-10-29: a Claude coordinator took over. It landed the
  networked CLI, closed P2, added the shared physics backend contract and the
  native DART and MuJoCo adapters. The commits are listed below.

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

- All of them passed the staged pre-commit gate and hosted CI. `main` is clean
  and pushed in every repository. The [dashboard](https://gazebosim-rs.github.io/rewrite-progress/)
  shows P2 complete and P3 in progress.
- Engines: Rapier (full f64), DART, MuJoCo and TPE are selectable in the scene
  and CLI; the CLI also accepts the upstream `gz-physics-*-plugin` names.
  DART and MuJoCo match the native gz-physics plugins bit for bit or within
  1e-14 on their parity fixtures. Bullet stays rejected (decision D04).
- P3 remains open: articulation beyond world revolute joints, joint limits and
  damping, contact properties, meshes/planes/heightmaps, and conformance against
  gz-physics `test/common_test` for each backend.

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

- The owner guidance in AGENTS.md was also added to gz-msgs-rs and the tracker,
  but not to sdformat-rs or gz-transport-rs:
  - sdformat-rs's hook treats AGENTS.md as protected policy and requires an
    independent-review receipt (the hash-bound review step the owner asked to
    drop). Changing that hook is the owner's decision.
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
   independent focused runs pass; the staged hook and hosted CI remain required.
2. MuJoCo follow-ups (low, not reachable through the scene today): extreme
   inertial-centre offsets are admitted but fault the first step; `remove_joint`
   on a fast hinge can create a free velocity above mjMAXVAL; the hinged load
   comparison against the native plugin is not like-for-like (the plugin adds
   actuators); the real contact-overflow test bypasses `MujocoBackend::step`.
3. P3 capabilities, one reviewed slice at a time: general articulation (model
   joint trees, not only world revolutes), joint limits/damping/effort, contact
   surface properties, mesh/plane/heightmap shapes. Port the matching
   gz-physics `test/common_test` cases per backend; the upstream binaries are
   built in `gz-physics/build/bin/COMMON_TEST_*` and take a plugin path.
4. Then P4 (Bevy rendering and sensors) per PLAN.md.
