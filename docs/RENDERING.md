<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Experimental offscreen renderer

The optional `gz-renderer/gpu` feature builds a bounded Bevy 0.20.0 Vulkan
renderer. A caller can supply a complete snapshot of up to 128 world-space
boxes, a rebasing origin, an epoch and committed simulation time. It derives
render transforms using `gz-render-frame`, updates or removes its private Bevy
entities, and renders an unlit 64×64 scene to an offscreen image. The camera is
fixed at render (0,0,5), looking at the origin. This is not a calibrated sensor
or a GUI. The optional `gz-render-scene` crate can explicitly extract a snapshot
from a simulation; it does not schedule rendering or start a worker.

`gz-render-scene::extract` reads live visual geometry and world poses from the
canonical ECS, validates owned public IDs, sorts them, and stamps the request
with committed integer epoch and simulation time. It accepts at most 128 boxes
with explicit opaque diffuse colors and default remaining material and visual
properties. Missing materials or unsupported geometry, material and visibility
settings return errors. Diffuse RGB is mapped to the renderer's unlit color;
this is a bounded visual approximation, not SDF material or shadow parity. The
caller chooses the origin and runs the renderer outside simulation steps. No
assets are loaded and no implicit renderer state or callbacks are installed.

One readback may be in flight. Replacing a snapshot or explicitly cancelling
suppresses delivery; the slot remains occupied until the GPU operation retires.
The caller drives updates, polling and teardown outside physics steps. Readback requests
are rejected until the target, mesh, material and render instances are prepared
and known pipelines have completed across consecutive updates. A snapshot is
rejected as busy until the preceding snapshot reaches an update, and updates
wait for the preceding GPU submission's completion callback before submitting
another. Pipelined rendering is disabled to keep that ordering. Off-screen
objects need no extracted material instance; visible objects do. Permanent shader
errors and Bevy device errors stop the renderer. This readiness approach and
its timestamp correspondence need independent review and further qualification.
Device loss can prevent a readback from retiring; the caller must drop the
failed renderer. A cancelled operation is not a guarantee that GPU work stopped.

Bevy's mesh instance and phase preprocessing use its CPU path. In the combined
physics and renderer graph, Rapier's deterministic scalar-math feature gives
Bevy's native mesh uniform a different size from its shader layout; the GPU
preprocessor's native-sized allocation then fails validation. CPU preprocessing
writes the shader layout; Vulkan still renders and reads back the image. This
configuration is not a hardware-performance qualification.

The default CLI does not depend on this crate or the GPU stack. Construction
requests a Vulkan adapter and device explicitly and reports their errors. The
adapter metadata identifies software devices. Process-local serialization of
this crate's Vulkan instance creation addresses a reproducible crash involving
concurrent wgpu initialization and an installed unusable NVIDIA ICD. It cannot
serialize unrelated same-process wgpu users; that coexistence remains open.
With multithreaded Bevy tasks, its process-global pools remain after an in-process
renderer is dropped. The experimental `gz-render-worker` isolates those pools in a
child process started by `gz-render-client`; its executable is selected only through
an explicit trusted operator configuration. The parent uses bounded request/reply
channels. Explicit `shutdown(timeout)` retains its supervisor handle on timeout.
On final `Drop`, the client requests shutdown and, if that bounded attempt times
out, forces stop and joins the retained supervisor. A kernel or driver stall may
therefore delay `Drop` indefinitely; callers must destroy it outside physics steps.
The worker currently uses Linux parent-death signaling and has not been qualified
on other platforms.
This lifecycle is not yet integrated with the simulator or accepted. The default Bevy panic hook,
Ctrl-C handler and log plugin are disabled in the renderer.

Local tests have exercised a red box, color update, removal, reset-epoch and
explicit cancellation, invalid snapshots, missing readiness resources and
shader/device error handling on llvmpipe, a CPU Vulkan implementation. An isolated
test destroys its actual wgpu device and observes a persistent renderer error;
this does not exercise a device failure in the worker process. The tests do
not qualify a physical GPU or establish camera, lidar, image timing, performance
or upstream rendering parity. The simulator integration, supported GPU profile,
feature-aware gate's full staged qualification, and real hardware runner remain incomplete.

## Related upstream assertions

The following bounded mapping uses the revisions in `planning/baseline.json`.
The current tests are new controls, not ports of these native tests; no upstream
test is credited as passed or retired by them. No native rendering comparison
has been run for this slice.

| Pinned upstream test and retained assertions | Related current control and remaining work |
| --- | --- |
| [`gz-rendering` `CameraTest.CopyIntoExternalBuffer`](https://github.com/gazebosim/gz-rendering/blob/fb8e38cee4dad491f95f205afe6697b6a5d0f5fa/test/integration/camera.cc#L821): a 64×48 RGB green-box capture equals the caller buffer byte-for-byte; undersized or wrong-dimension buffers are rejected, and the first undersized-buffer byte stays `0xFF`; Bayer conversion matches byte-for-byte. | [`software_box_snapshot_and_cancellation`](../crates/gz-renderer/tests/software.rs) checks a 64×64 RGBA frame length and exact red and green center pixels, but has no caller-buffer, RGB or Bayer API. The native assertions remain pending. |
| [`gz-rendering` `CameraTest.Visibility`](https://github.com/gazebosim/gz-rendering/blob/fb8e38cee4dad491f95f205afe6697b6a5d0f5fa/test/integration/camera.cc#L394): two cameras with visibility masks capture ten times; every pixel's intended green or red channel exceeds the other two, and summed absent channels equal zero while the intended sum is positive. | The software box test checks center-pixel changes on move, restoration and removal, plus projected extents for an asymmetric box and a quarter-turn. There are no camera masks or multi-camera visibility assertions. |
| [`gz-rendering` `CameraTest.ViewProjectionMatrix` and `RenderTexture`](https://github.com/gazebosim/gz-rendering/blob/fb8e38cee4dad491f95f205afe6697b6a5d0f5fa/test/common_test/Camera_TEST.cc#L36): camera pose, perspective/orthographic projection and projected pixel positions; HFOV and aspect checks use `1e-6`, and texture dimensions, format and memory size are checked. | [`gz-render-frame` conversion tests](../crates/gz-render-frame/tests/conversion.rs) check f64 origin subtraction and a proper basis change, including rotation-vector error below `1e-6`. They do not establish camera forward-axis semantics, calibration, projection, clipping, configurable format or aspect behavior. |
| [`gz-sensors` `CameraSensorTest.ImagesWithBuiltinSDF`](https://github.com/gazebosim/gz-sensors/blob/8ddf4c17be8a434c51b51d5be7c866fb69aaa54f/test/integration/camera.cc#L102): SDF sensor dimensions 256×257 and frame ID, image publication, active/inactive behavior and sensor removal. Its `CameraIntrinsics` and `CameraProjection` cases check published matrices, including focal length 866.23 with respective tolerances 0.3 and 1.0. | [`direct_worker_renders_a_stamped_snapshot_and_exits`](../crates/gz-render-worker/tests/direct_worker.rs) checks request ID, exact epoch/time and a red center pixel for one explicit snapshot. No SDF camera sensor, image or camera-info publication, sensor lifecycle, intrinsic or projection mapping is implemented. |
| [`gz-sim` `SensorsFixture.UpdateRate`](https://github.com/gazebosim/gz-sim/blob/71c60db31fddd5c98d2aa689e9a2cccb3548a0a3/test/integration/sensors_system_update_rate.cc#L79) checks exact sensor message counts and nonempty timestamp streams, with subsequent intervals compared to `1/rate` using `EXPECT_FLOAT_EQ`. [`ResetFixture.HandleReset`](https://github.com/gazebosim/gz-sim/blob/71c60db31fddd5c98d2aa689e9a2cccb3548a0a3/test/integration/reset_sensors.cc#L95) includes image, simulation and reset lifecycle assertions. | The software test checks epoch/time changes and cancellation of an obsolete readback. It does not schedule or publish cameras, reproduce rate/timestamp streams, or exercise simulator sensor reset. |

Depth, RGBD, segmentation, bounding-box, thermal, wide-angle, distortion and
lidar cases remain separate upstream sensor obligations; they are not covered by
the primitive box image or the frame-conversion controls.
