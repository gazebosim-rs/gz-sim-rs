<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Collision search cost while a robot holds cloth

In 1,305,600 constraint passes on a damped folded sheet, checking the cloth against itself took 2.687 seconds and checking rigid boundaries took 2.332 seconds. Stretch took 0.426 seconds and bending took 1.121 seconds. These timings came from a separate instrumented Rust build. In production the small-shell solver now reuses a conservative distance bound across passes. Larger meshes and any motion that exhausts the bound use the existing continuous collision search. The replay contains every original finger and cloth position. [Watch the pickup in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9991%2F) · [Individual runs and hashes](cloth-clearance-cost.json)

| Shared-host campaign | Fingertips | Median CPU seconds, original → new | Median p95 wall time per 1 ms command, original → new |
|---|---|---:|---:|
| CPU 8 | Damped | 11.41 → 8.86 | 4.80 → 3.75 ms |
| CPU 12 | Damped | 12.89 → 8.98 | 5.92 → 3.69 ms |
| CPU 8 | Spring only | 15.34 → 13.75 | 6.86 → 7.14 ms |
| CPU 12 | Spring only | 14.46 → 13.20 | 6.40 → 6.69 ms |

Each campaign ran four paired, order-balanced repetitions of the full 3.4 s grasp per fingertip type. Both used 16 native exchanges per 1 ms actuator period on an AMD Ryzen 9 9950X3D with Rust 1.97.1. The new search reduced CPU time in both grasps. The spring-only grasp's p95 rose by about 4–5%, motivating an additional limit on how often the solver rebuilds a short-lived bound. The table identifies the original implementation at f711cce and the first measured shortcut at 1b4d135.

All 32 runs reproduced the corresponding complete particle trajectory, boundary force recording, numerical diagnostics and Web replay byte for byte. Comparisons at 250 exchanges also reproduced both full grasps, including all 261 locally refined ticks of the spring-only fixture. The separate supported-shell regression reproduced the difficult 737th millisecond. The viewer independently checked 1,644 cloth positions across 137 frames and rewound to zero.

Each bound starts from the projected separation of every eligible pair of convex features. The solver subtracts a numerical margin and accounts for the relative displacement of both features over the sweep. Shared translation preserves the bound. A crossing, invalid projection or uncertain separation falls back to continuous collision checks. Raw query accounting remains intact. The native package passed 211 ordinary release tests, the opted-in supported-shell regression, and strict Clippy. LLVM covered 101/103 changed production lines and 182/184 regions.
