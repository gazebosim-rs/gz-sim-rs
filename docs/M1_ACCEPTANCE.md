<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M1: runnable primitive MVP

Accepted on 2026-11-03. Both implementations are published on Main:

- Simulator: [35100cf8](https://github.com/gazebosim-rs/gz-sim-rs/commit/35100cf8b487db787d38e0b1d6075278a82408d3), [exact successful CI](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41590174007).
- Viewer: [0db1f4e](https://github.com/gazebosim-rs/gzweb-rs/commit/0db1f4e46bae49fb5b187dd3286b2b28253e64a4), [all six exact CI jobs](https://github.com/gazebosim-rs/gzweb-rs/actions/runs/41590911006).

Use [MVP_BROWSER.md](MVP_BROWSER.md) for launch commands. The admitted world has
20 moving Rapier f64 boxes/spheres and static box ground. The server is headless;
the maintained Bevy browser viewer shows real poses, truthful visual rows and
committed pause/resume/reset. Reload obtains current state, and shutdown clears
stale geometry and disables controls. [Saved browser frames and bindings](evidence/m1-browser/README.md)
include the transient reconnect closure followed by automatic recovery.
The actual browser used WebGL2; no WebGPU runtime or timing claim is inferred.

The same patched native-free executable also launched in official Ubuntu 24.04
image `sha256:fe81560990fae86dc4743202b2cb4d3df98d0bcf949173cecef15ce1127ef852`.
Only the executable and exact fixture were mounted read-only. With no network,
GPU or SDK access, the actual server/listener completed two 5 ms Rapier steps
and exited successfully; dynamic linkage contained only system libraries.
This is a bounded clean-Ubuntu runtime observation, not a browser inside the
container or a claim of portability to every Linux distribution.

## Verification

Implementation, regression code, documentation and independent source review
preceded the consolidated required checks. Real failures were corrected in
reviewed batches; assertions, tolerances, policies and coverage gates were kept.
Both repositories pin upstream executor 1.14.1, replacing the yanked version.

The simulator's normal staged hook passed in 1,005.245 s. Exact hosted CI and
independent official artifact/source audits passed: 952 headless, 555 renderer,
276 tooling and ten native-off cases. The five/three ignored parent-harness
entries are explicitly delegated helpers, not skipped feature acceptance.

| Hosted owned profile | Lines | Regions |
| --- | --- | --- |
| Headless | 11026 / 11224 | 16744 / 17247 |
| Renderer | 6201 / 6327 | 9154 / 9506 |

Each required crate and profile independently exceeds 95% for both metrics;
profiles were not combined. The native-off proof additionally retains four
executables, actual disabled-backend line/region hits and an SDK-free closure.
The eight real C++ Zenoh peers and retained reference identities also pass.
Full-profile Rust executable records are metadata-only; raw/merged counters,
reports and native-peer artifacts remain retained. No independent re-export
from missing full-profile executables is claimed.

Viewer normal commit and pre-push gates passed. All six hosted jobs passed,
including advisories, WASM budgets, seven legacy protocol-parity worlds and
three separate cached Fuel cases. Official coverage is 54249 / 55767 lines
(97.277960%); the existing threshold and exclusions are unchanged. Both local
WASM flavors meet the 14,000,000-byte gzip budget. Independent review verified
official artifact digests, source blobs/modes, counters and browser bindings.

## Boundary and next delivery

M1 includes simple shapes, local visualization, controls and bounded lifecycle
behavior. It does not accept meshes, mesh collisions, joints, sensors, remote
downloads, full Gazebo service parity or a performance improvement over C++.
Historical full-rewrite inventories remain deferred.

M2 adds local glTF 2.0 visuals while retaining primitive collisions. Its existing
budget is at most three working days including review and required checks.
