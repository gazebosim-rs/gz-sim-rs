<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Local changes to Rapier 0.36.1

The reviewed import at `67b3830` contains the published `rapier3d-f64` 0.36.1 package. Its crate archive has SHA-256 `f4588855e9c5451702147b5c6ddff4d93887f1e59c9504b6ac3571818f6ec3d2`. The workspace supplies the Apache-2.0 license alongside the source.

Gazebo opts a bounded prismatic joint into a coupled solve for a single loaded, frictionless contact point. The joint can brace at either stop and release when the load reverses. The coverage report describes the eligibility checks, analytical comparisons and fallback cases. [Solver measurements and qualification](../../docs/measurements/local-manipulation/powered-slider-native.md)

A bounded reduced prismatic link can also carry a pad normal stiffness in N/m. The adapter keeps this authored value in the existing joint user-data field for reconstruction. Two coated links compress in series. Each native contact manifold divides its stiffness equally among its solver points, and each point uses its normal effective mass and the native contact damping ratio. Uncoated pairs use the existing Rapier contact coefficients. The [pad regressions](../../crates/gz-physics-rapier/tests/slider_pad_stiffness.rs) measure equilibrium compression and force for point and face contacts, and the [travel regressions](../../crates/gz-sim-scene/tests/reduced_slider_travel.rs) check first-impact impulses at a seated stop.

From the repository root, inspect the Rust patch with the following command.

```sh
git diff 67b3830 HEAD -- third_party/rapier3d-f64-0.36.1/src
```
