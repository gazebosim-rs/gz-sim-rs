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

## Source-only calibrated metric depth

The new `set_calibrated_depth_snapshot`, `request_depth_readback` and
`take_depth_frame` API is an unqualified RENDER-02 candidate on accepted `c5dd02c0`.
After the initial attempt described below, the accepted primitive-inertia consumer
parent was fast-forwarded once. Its producer pin and publication changes touch no
renderer files. All four reviewed code/test payloads, including the appended
lifecycle correction controls, remained byte-identical across that integration;
only this factual README parent note then changed. The affected required gate
will qualify the corrected feature against the accepted consumer dependencies.
The first attempt compiled and passed the new depth controls but failed owned
region coverage. The amended controls and complete corrected qualification remain
unrun. The earlier light-only package logs above are historical evidence for
those changes, not depth qualification. No SDF sensor extraction, worker wire
format, scheduler, server, transport or intensity-to-lux policy changes.

This bounded programmatic API renders the existing opaque box snapshots through
the existing 64×64 calibrated camera. A `DepthFrame` contains epoch, simulation
time and 4096 row-major `f32` optical-forward axial distances in metres, starting
at the upper left. It is not Euclidean range. A zero reverse-Z depth sample maps
to positive infinity for background/far clipping. Nonfinite/out-of-range raw
samples or malformed delivery fail the capture rather than generating a frame.
Unprojection uses maintained glam with the inverse of the actual narrowed
projection matrix sent to Bevy; it does not ray-cast or synthesize a trajectory.
Authoritative camera/box transforms remain f64 before render conversion.

Native gz-sensors `8ddf4c17` integration `depth_camera.cc` requires centre-face
depth within `DEPTH_TOL=1e-4` and background/far positive infinity. Its near-range
negative-infinity behavior is outside this slice: a single raster depth texture
cannot distinguish near-clipped geometry from empty background. Every oriented
box must have all eight f64 camera-local corners strictly beyond the near plane.
Touching/intersecting/behind boxes are refused before snapshot mutation or
readback cancellation, even if outside the horizontal/vertical field of view.
Rotated cameras, world-origin rebasing and asymmetric pinhole intrinsics reuse
the existing frame/calibration API. No native original-case, native near-range,
physical GPU or whole depth/RGBD sensor completion is claimed.

Only explicit depth mode sets the camera to one sample and enables COPY_SRC on
its depth attachment. Bevy 0.20 `core_3d` uses Depth32Float, and the main opaque
and transparent nodes store that attachment. After an existing non-pipelined
update, a request copies that actual retained texture into one bounded staging
buffer; width64 makes its four-byte rows exactly 256-byte aligned. wgpu submits
the copy after rendering on the same queue. Its asynchronous map callback owns
the buffer until completion, copies the actual bytes, unmaps, and releases it.
Device polling is nonblocking and explicitly driven by `update`/delivery calls.
No render graph node, shader, plugin, scheduled callback, persistent worker,
background poller or dependency is added. No staging buffer exists without a
request. Empty optional depth state owns no heap resources.

Depth and RGB share the existing single-slot admission contract. Either pending
kind makes another request Busy. A new valid snapshot cancels delivery but leaves
the slot occupied until its actual callback retires. Epoch/time and projection
are captured at request, so a cancelled old frame cannot acquire successor
metadata. A poisoned completion lock retains slot ownership. Device loss is
terminal like the existing RGB path. Legacy snapshot methods restore the
original camera depth usages and default MSAA when leaving depth mode, and do no
depth-mode work when none was installed. `Snapshot`, RGBA `Frame`, all old test
assertions and protocol/client/worker contracts remain unchanged.

Source regression controls cover inverse-projection axial depth and invalid raw
samples, oriented f64 near-plane admission, actual GPU box-face/background/far
and empty output, asymmetric intrinsics/camera rotation/origin rebasing,
occlusion, invalid-admission atomicity during real readback, shared-slot
cancellation/retirement/successor stamps and full-frame calibrated RGBA restoration.
Their tolerances do not change any retained assertion. The mapped upstream
centre/background assertions are source references only until actual software
GPU execution and complete normal/hosted qualification pass.

The first completed-feature mandatory run compiled and passed all three new depth
controls (927 headless and 549 renderer tests passed), then stopped at the owned
renderer region threshold: 1009/1066 regions, below the unchanged >95% requirement.
It did not qualify the feature. Additional private controls now exercise missing
camera/attachment admission, terminal-device admission, overcapacity preserving a
pending capture, corrupt completion retirement and poisoned-slot ownership. These
amended controls are unrun; numeric bounds, production code and old assertions
remain unchanged pending the next consolidated required gate.
