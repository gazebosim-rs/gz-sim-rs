<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Local changes to Rapier 0.36.1

The reviewed import at `67b3830` contains the published `rapier3d-f64` 0.36.1 package. Its crate archive has SHA-256 `f4588855e9c5451702147b5c6ddff4d93887f1e59c9504b6ac3571818f6ec3d2`. The workspace supplies the Apache-2.0 license alongside the source.

Gazebo opts a bounded prismatic joint into a coupled solve for a single loaded, frictionless contact point. The joint can brace at either stop and release when the load reverses. The coverage report describes the eligibility checks, analytical comparisons and fallback cases. [Solver measurements and qualification](../../docs/measurements/local-manipulation/powered-slider-native.md)

From the repository root, inspect the Rust patch with the following command.

```sh
git diff 67b3830 HEAD -- third_party/rapier3d-f64-0.36.1/src
```
