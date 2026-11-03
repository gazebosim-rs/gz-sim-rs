<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M2: local mesh visualization

Accepted on 2026-11-03, published on both Main branches:

- Simulator [7b8a4e10](https://github.com/gazebosim-rs/gz-sim-rs/commit/7b8a4e10fb06ee4077e5092c588f309811a128b5), [exact successful CI](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41612752387).
- Viewer [5b67f45f](https://github.com/gazebosim-rs/gzweb-rs/commit/5b67f45fe0638197dc06ad2d2659a34460c7f396), [all six exact CI jobs](https://github.com/gazebosim-rs/gzweb-rs/actions/runs/41618298656).

[Launch and supported input contract](M2_LOCAL_GLTF.md). The actual WebGL2
browser renders the admitted local glTF pyramid, red sphere and box ground,
follows Rapier motion with primitive collisions, and acknowledges pause,
resume and reset. Reload recovers the current mesh scene. A separate corrupt-PNG
fixture produces one visible production error naming `pixel.png` and the PNG
decoder failure. Missing documents, malformed glTF JSON and missing external
buffers also fail through the actual CLI with named diagnostics before stepping.

[Ten actual screenshots and source/binary bindings](evidence/m2-browser/README.md)
record these observations. The initial/reset views are visually consistent;
quantitative transform and material provenance checks use the separate passing
actual imported `Mesh3d` regressions. A uniform green screenshot alone is not a
texture-sampling measurement. The corrupt-PNG error confirms the image dependency
path is used. Invalid dependency errors may leave partial geometry visible.

## Required checks

Implementation, regressions, documentation and independent source review
preceded the consolidated mandatory checks. Actual failures were corrected in
reviewed batches. No assertions, numerical tolerances, coverage exclusions or
quality gates were weakened. The simulator normal staged hook passed in
893.222 seconds; viewer normal commit and pre-push hooks passed. Both exact
hosted runs and independent official source/data audits passed.

| Simulator hosted owned profile | Lines | Regions |
| --- | --- | --- |
| Headless | 11393 / 11598 | 17340 / 17866 |
| Renderer | 6203 / 6327 | 9156 / 9506 |
| CLI, within headless | 994 / 1011 | 1535 / 1599 |

Every required simulator crate passes both strict metrics separately. Profiles
are not combined. The hosted full-source changed CLI line result is 935/945;
the local parent-relative result is separately 353/357. Actual logs show 975
headless, 555 renderer and 276 tooling passes. The five/three ignored parent
entries remain explicitly delegated helpers. Ten native-off cases pass with
four retained executables, actual disabled-arm hits and a system-library-only
closure. Eight existing real C++ Zenoh peer scenarios retain their source,
executable and selected SDK identities. Full-profile Rust executable records
remain metadata-only; no independent re-export from missing binaries is claimed.

Viewer official workspace line coverage is 54487/56020 (97.263477%) over 221
files, passing its unchanged policy. Logs contain 1074 native passes, zero
failures and five unchanged ignored entries; three separate cached Fuel cases
pass. Seven legacy protocol/asset parity worlds match under existing named
snapshot suppressions. Official viewer evidence retains source, LCOV, logs and
parity reports, not raw coverage profiles or native test executables.

Both current browser bundles pass the unchanged 14,000,000-byte gzip budget:
WebGPU 12,882,605 bytes; WebGL2 12,859,076 bytes. Their one build pass took
1674.421 seconds with no capacity stop. Browser observations reused the exact
qualified instrumented native-free simulator executable, avoiding another build.
This makes no performance claim. Actual captures show background 1 fps and
unavailable timing/physics statistics as dashes. Startup/reload initially showed
Waiting/CUA timeouts before the real scene appeared; no startup-time guarantee
or actual WebGPU browser acceptance is inferred.

## Boundary and next delivery

M2 admits bounded local glTF 2.0 visuals, embedded materials and declared local
resources. It retains primitive collisions. SDF mesh material overrides,
submesh selection/recentering, remote downloads, general formats, texture
conversion, mesh collisions and broad asset-format compatibility remain outside
this acceptance. M1 and historical evidence remain intact.

M3 now adds Rapier static triangle-mesh and dynamic convex-hull collisions,
with geometry/contact/reset checks and explicit unsupported nonconvex dynamic
refusal. Its scope budget remains at most five working days including review
and required checks.
