<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M72: explicit pinhole camera intrinsics

Budget: at most five working days including review and mandatory qualification.
The owner continued development on November 19, 2026. D31/D32 permits this
bounded successor while exact hosted qualification is held. The base is clean,
locally qualified M71 `f31d7155c2569df875b393857a1339bb192caf2c`. M71 passed
its complete normal/strict gates and independent retained audit. M70's package,
installation, camera and browser proofs remain separate. Neither phase is accepted
or published; the retained snapshot HTTP 401 failure has no demonstrated recovery.
No CI retry or snapshot index probe is released here.

## Scope and dependencies

Support explicit SDF lens intrinsics for the existing calibrated 64 by 64 RGB and
metric-depth camera domain. Select K only when `lens.has_intrinsics` is true.
Require finite fx/fy/cx/cy and skew, positive fx/fy and exactly zero skew. Admit
independent focal lengths and off-axis principal points through the existing
calibration matrix, extraction, worker and canonical CameraInfo paths. With no
explicit intrinsics, preserve the current horizontal-FOV derivation exactly.
Retain the existing valid HFOV, near/far, size, format and sensor-domain limits.
Explicit projection remains unsupported even when its numbers match K.

Use the pinned native camera sources already recorded in CAMERA_MESSAGES.md:
explicit K configures the rendering matrix before an absent P is derived from it.
The earlier Ogre/Mesa native precedence helper failed initialization and produced
no runtime result. Preserve that failure. Existing CPU projection and native
CameraInfo captures retain only their stated math/wire scope; no old result
qualifies changed rendering here.

The selected domain publishes K and P=[K|0]; dormant projection defaults cannot
override the selected K. Do not claim native Ogre pixel equivalence. Existing
Calibration::new remains the final f32 projection representability gate, before
external IO. Scene metadata admission needs no new rendering dependency.

Production changes are limited to Scene camera metadata admission and render-scene
calibration selection. No parser, wire protocol, shader, rendering algorithm,
transport, dependency, SDK, solver, framework or camera-count change is selected.
The camera-output header comment must describe both selected-K and HFOV paths.
No calibration cache or ownership/lifecycle redesign is required.

For example, an otherwise supported camera may supply:

```xml
<lens><intrinsics>
  <fx>40</fx><fy>48</fy><cx>20</cx><cy>25</cy><s>0</s>
</intrinsics></lens>
```

`examples/m72_camera_intrinsics.sdf` gives the RGB and depth owners different K.
The existing `--camera-worker` and `--camera-mixed-pair` CLI options select the
same mixed family; this phase adds no new CLI flag or installed launcher option.

## Ownership and required evidence

Root integrates Scene admission/regressions, the finite/CLI registry, guides and
planning. The extraction lane owns render-scene calibration selection and focused
regression Source plus the camera-output comment. The camera-evidence lane owns
one explicit-intrinsics mixed fixture, real-worker/canonical consumer regression
Source and application lifecycle controls. Independent whole Source review closes
all findings before a single consolidated verification pass. File leases must be
explicit; no agent edits a frozen parent or another lane's files.

- Prove valid explicit K, its presence distinction, anisotropic focal lengths and
  off-axis principal points; absent K must keep exact HFOV-derived values.
- Reject nonfinite/nonpositive focal lengths, nonfinite principal points, nonzero
  skew and every explicit P. Retain all other admission and refusal scenarios.
  Replace only inherited assertions that explicitly describe the now-supported
  valid K domain; unsupported projection and malformed K refusals remain strict.
- Reject finite K that cannot become a valid f32 projection before external IO,
  without changing time, identities or retained calibration ownership.
- Use the existing software worker and canonical C++ consumer to observe RGB_INT8,
  R_FLOAT32, exact K/P, headers, first 1ms stamps and sequences. Distinct per-owner
  K must make silent HFOV fallback observable through analytic image/ray/depth
  checks, preserving positive-infinity depth background and numerical bounds.
- Info-only/no demand must remain worker-lazy. Pause, independent demand/rates,
  metadata drift, reset/removal and stale in-flight retirement retain current
  ownership semantics. Loaded reset restores original K/IDs/kinds and fresh
  sequences. Replacing metadata never silently recalibrates an in-flight frame.
- Keep all inherited tests except the narrow supported-domain transition above.
  Mutate invalid loaded metadata when the tested boundary is application admission
  rather than expecting a malformed SDF to pass the scene loader.

## Qualification and acceptance

Complete implementation, meaningful regression Source, fixture, documentation and
independent Source review before one consolidated full normal/strict local gate
and independent retained audit. No per-edit tests/lint or trial product runs.
Inspect all available real failures and batch corrections before affected reruns;
never weaken coverage, assertions, deadlines or numerical tolerances.

Local development/qualification can proceed on the genuinely locally qualified
parent. Final parent acceptance and reconciliation of any parent correction,
exact hosted success, durable official evidence and qualified Main promotion
remain required before M72 acceptance or product Source publication. Old parent
checks do not qualify this candidate. All prior failures and frozen worktrees,
Fuel ownership, Rapier/TPE and D33 retirement remain protected. M1–M69 and 202 slices
remain accepted; M65 remains Latest with 35 installed examples.

Exclude arbitrary P, skew, distortion, noise, lens-relative pose, RGBD registration,
synchronization, point clouds, greater sizes/counts, package distribution, viewer
changes, performance claims or new compatibility inventory. No old package,
installation or accepted browser/camera qualification is repeated.
