<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Explicit directional illumination

The optional `gpu` API offers `set_directional_snapshot` and
`set_calibrated_directional_snapshot`. Each accepts the existing complete
`Snapshot` and exactly one `DirectionalWorldLight`. This is a programmatic
renderer input; Scene extraction continues to refuse world lighting.

`illuminance_lux` is nonnegative illuminance in lux, independent of SDF's relative
intensity. `color` is linear RGB in `[0, 1]`. The quaternion uses f64 Gazebo
world axes and maps local vectors to world vectors; `direction` is the local
propagation direction. Its magnitude has no effect. Direction is normalized and
rotated in f64 before the existing Gazebo-to-Bevy frame conversion and f32
rendering. A directional light has no position. Its nonzero ID must differ from
all visual IDs. Invalid orientations, directions, colors and illuminance are
rejected, including illuminance that overflows or underflows the f32 input.

A successful explicit lit snapshot owns one Bevy directional-light entity.
Updating its ID retires the old entity. Shadows and contact shadows are disabled.
Box materials use Bevy lighting with zero reflectance and maximum roughness;
the camera's ambient contribution is zero. Camera exposure retains Bevy 0.20's
default, EV100 9.7, with the existing disabled tonemapping and dithering. These
choices do not define an SDF intensity conversion or native pixel equivalence.

Zero lux keeps the lit-material path with no directional contribution. Calling
an existing unlit snapshot method removes the owned light and camera ambient
override, and restores the original unlit material settings. All light, camera
and visual validation precedes scene mutation and readback cancellation. The
existing bounded readback and snapshot-update admission rules apply. Lighting
adds no application callback, worker, endpoint, plugin or dependency.

The added tests cover f64 conversion, invalid inputs, replacement and removal,
atomic rejection, software face response, calibrated cameras and full-frame
unlit restoration. `cargo test --offline --locked -p gz-renderer --features gpu
-- --nocapture` passes all nine unit and three software tests, with no failures,
ignored tests or filtered tests. The Vulkan runner asserts a CPU adapter named
llvmpipe (Mesa 25.2.8, LLVM 20.1.2). This includes all retained renderer tests.
The build uses Rust 1.97.1, one build job, disabled incremental compilation and
zero debug information. Source-bound commands and logs are retained in ignored
`artifacts/explicit-lux/`.

Strict coverage, complete staged and exact hosted qualification remain pending.
Native radiometric or pixel parity, physical-GPU qualification, SDF-light
admission and original upstream case completion are not established.

A subsequent test-only lifecycle increment passes the complete package with nine
unit and four software tests on the same asserted llvmpipe runner. It submits a
real screenshot through an advanced Bevy update, replaces lighting while the
capture's delivery slot remains owned, then removes lighting during another
capture. Each cancelled slot remains busy until retirement and delivers no stale
frame. Successor stamps and full-frame calibrated unlit restoration are checked.
The device may already have completed when replacement arrives; this test does
not claim an unsignalled GPU operation at that instant. Separate source-bound
results are retained in `artifacts/explicit-lux-lifecycle/`; the earlier twelve-
test results remain unchanged. Complete integration qualification is still pending.

Snapshots that remain unlit skip lighting cleanup and retained-material lighting
writes. Only removal of an installed light restores those material defaults and
removes the ambient override. New unlit materials use the original default
construction. This avoids unnecessary admission work without a performance claim.
The unchanged thirteen-test package passes after this adjustment; separate logs
are retained in `artifacts/explicit-lux-transitions/`.
