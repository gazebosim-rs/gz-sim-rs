<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Render-frame conversion

`gz-render-frame` converts world-space f64 Z-up poses to f32 Y-up poses for a
future Bevy renderer. It maps Gazebo axes +X, +Y, +Z to Bevy +X, -Z, +Y. It
subtracts a caller-supplied f64 world origin before narrowing the position; the
same origin must be used for every camera and object in a rendered frame. The
rotation converts both the local and world bases. Inputs must be finite and
rotations unit length within the documented squared-norm tolerance; overflow
on narrowing is an error. A finite f32 position may still lose precision: origin
selection and its permissible error are the caller's responsibility.

A renderer must consume coherent snapshots of stable world-local entity IDs,
world poses and visuals, labeled with committed integer simulation time and
reset epoch. IDs must be paired with epoch across reset. Rendering remains
derived state and GPU work or waits must not occur on the physics critical step.
This crate is pure conversion and installs no simulation systems, renderer,
workers or device. It does not implement scene extraction, camera projection,
image rendering or readback.

The workspace baseline uses Rust 1.97.1, required by the selected Bevy and WESL
components. This environment has no `/dev/dri`; Vulkan enumeration found llvmpipe
(LLVM 20.1.2, device type CPU). Software Vulkan availability alone is not physical
GPU qualification. A separate renderer now exercises offscreen boxes and readback,
but simulator snapshot integration and RENDER-01 qualification remain open.

Analytic tests cover large-coordinate rebasing, all three rotated axes, invalid
inputs and f64/f32 overflow. They do not establish image or sensor correctness.
