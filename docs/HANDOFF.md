<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Coordinator handoff

Updated 2026-10-28. Read this first when resuming coordination of the rewrite.

## Where things stand

- P2 is closed for the selected headless world ([acceptance/P2.md](acceptance/P2.md)).
  SLICE-01 is complete. P0, LOAD-01, PHY-02, PHY-05, SENSOR-01 and IO-01 stay in
  progress on their own exit criteria.
- Recent main commits: networked CLI server mode (`a549c52`), native-compatible
  world-name sanitization (`1cc54cc`), P2 closure (`d58223d`). All passed the
  pre-commit gate and hosted CI.
- The dashboard shows P2 complete, P3 in progress, PHY-03 active.
- Every independent repository's `main` is clean and pushed.

## Owner decisions for this coordinator (2026-10-27/28)

- Keep the real gates: tests, >95% line and region coverage per crate, the
  `scripts/verify.py --staged` pre-commit hook, hosted CI, real C++ interop
  tests, and preserved upstream test intent. Skip the earlier frozen-proposal,
  hash-bound review and resource-reservation ceremony.
- The coordinator may commit and push reviewed work to main, update the
  dashboard, and edit sdformat-rs, gz-msgs-rs and gz-transport-rs directly.
  Never edit gz-fuel-tools-rs.
- Ask before deleting anything outside regenerable build output.

## Local setup notes

- Native interop tests and the commit hook need a C++ reference install with a
  sibling `environment.json`. The `artifacts/native-reference/install` default in
  this repo lacks it. Use
  `GZ_REFERENCE_PREFIX=<gz-transport-rs>/artifacts/reference/install`, or rebuild
  with `python3 scripts/bootstrap-native.py` on a new machine.
- The full gate takes about 10 minutes locally; hosted CI about 8.

## Next

1. DART follow-ups: run the server reset native comparison
   (gz-sim-server/tests/private/reset.rs "dartsim" rows) against DART as well as
   Rapier; that needs the same DEP_DART_LIBDIR rpath build script as gz-sim-cli
   and gz-sim-scene. DART diverges natively for some fast spinning slender piles
   (documented in DART.md); revisit if upstream adds recovery.
2. MuJoCo follow-ups (low): bound or document admitted extreme inertial-centre
   offsets that fault the first step; `remove_joint` can create a free velocity
   above mjMAXVAL; like-for-like hinged load comparison; drive the real
   contact-overflow test through `MujocoBackend::step`.
3. Remaining P3: articulation beyond world revolutes, contacts/joint limits and
   damping, meshes/planes, and per-backend conformance against gz-physics
   test/common_test.
4. Run MuJoCo tests with `MUJOCO_PREFIX=<repo>/artifacts/physics-reference/mujoco`
   after `python3 scripts/bootstrap-physics.py`.
