<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Local changes to Rapier 0.36.1

The reviewed import at `67b3830` contains the published `rapier3d-f64` 0.36.1 package. Its crate archive has SHA-256 `f4588855e9c5451702147b5c6ddff4d93887f1e59c9504b6ac3571818f6ec3d2`. The workspace supplies the Apache-2.0 license alongside the source.

Gazebo opts a bounded prismatic joint into a coupled solve for a single loaded, frictionless contact point. The joint can brace at either stop and release when the load reverses. The coverage report describes the eligibility checks, analytical comparisons and fallback cases. [Solver measurements and qualification](../../docs/measurements/local-manipulation/powered-slider-native.md)

A bounded reduced prismatic link can also carry a pad normal stiffness in N/m. The adapter keeps this authored value in the existing joint user-data field for reconstruction. Two coated links compress in series. Each native contact manifold shares its spring stiffness and viscous damping equally among its solver points. For a point with normal effective mass `m`, total pad stiffness `k`, damping ratio `ζ` and `n` points, the row uses stiffness `k/n` and damping `2ζ√(km)/n`. The normal row retains its implicit spring and damper through velocity relaxation. Translating flat faces with four equivalent rows thereby match the one-point transient under the same drive. A four-corner box face keeps four material sites as unloaded corners leave the predictive solver set. This preserves each bearing corner’s share while the jaw rocks onto an edge. For frictionless face–vertex pairs the solver places the normal reaction on the current vertex’s line of action and reports the same applied lever. A retained witness sliding across the opposite plane therefore cannot move the moment arm. These rules use the two actual collision shapes and exclude user-modified contact sets. [Tilted pad and hook controls](../../crates/gz-physics-rapier/tests/private/tilted_pad.rs). Uncoated pairs use the existing Rapier contact coefficients. The [pad regressions](../../crates/gz-physics-rapier/tests/slider_pad_stiffness.rs) measure point and face compression, transient response and free approach, and the [travel regressions](../../crates/gz-sim-scene/tests/reduced_slider_travel.rs) check first-impact impulses at a seated stop.

From the repository root, inspect the Rust patch with the following command.

```sh
git diff 67b3830 HEAD -- third_party/rapier3d-f64-0.36.1/src
```
