<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Explicit bounded camera output

The optional `gz-camera-output` crate provides one explicitly constructed
camera owner. A caller supplies the canonical camera ID, rebasing origin,
nonempty frame ID, topics and an absolute trusted worker path. Construction
checks camera extraction and worker-path form before advertising, then retains
the validated worker selection for restarts. Absolute-path validation does not
authenticate the executable or protect it from replacement. It installs
no system, schedule, callback or worker in the simulation. Calling code owns
all naming, sampling and sequence policy; this does not implement SDF sensor
scheduling or native default-topic selection.

`CameraOutputConfig::with_loaded_frame_id(&simulation, sensor_id)` explicitly
replaces only the configured frame ID from a Scene-loaded camera. It verifies
current public identity, sensor kind, camera metadata and a nonempty world-scoped
Scene Name before deriving anything. A nonempty canonical `frame_id` takes
precedence over a present legacy `gz_frame_id`, including an empty legacy value.
Otherwise it removes the world scope from the Scene name: for example,
`default::robot::body::camera` becomes `robot::body::camera`.

This follows pinned native SIM `71c60db` RenderUtil's `SetName` preprocessing
before gz-sensors `8ddf4c1` `CameraSensor::Load` and its common Sensor frame logic.
It differs from passing the original local sensor name directly to Sensor::Load.
The CPU characterization covers the three original `Sensor_TEST.FrameIdFromSdf`
inputs and seven empty/whitespace boundaries, plus actual parsed-DOM `SetName`
copies. It does not execute RenderUtil, a rendering scene or the complete sensor
Manager. A native programmatic Sensor without an XML Element has different
fallback behavior; this convenience method is only for Scene-loaded metadata.

The opt-in method performs no IO, advertisements, worker startup or schedule
registration. An empty native result stays empty and is refused by the existing
constructor; it is never replaced with a guessed name. Omit the method to keep
an explicit operator frame override. Default topic selection, calibration
admission, manual construction and publication behavior are unchanged. This
metadata follow-up passes all 39 CPU library tests (five new), all-target Clippy
and formatting. Independent review and the normal strict staged and hosted gates
remain pending; it adds no original-case or full-Manager credit. No new GPU or
software-worker execution is claimed for this metadata-only increment.

`request` samples the committed epoch/time and calibration once. It consults
cached publisher interest; info-only requests use CPU message construction and
never start the renderer. They still extract and allocate the CPU scene, and
therefore reject unsupported visuals even when only camera info is requested.
Image demand starts the selected worker outside the
physics step and admits at most one acquisition. `pump` advances bounded
nonblocking IPC submissions or replies and reports backpressure explicitly.
After readback, one frame is retained until the next pump rechecks the camera,
epoch and demand before publication; no additional allocation or frame queue
is created. Publication consequently requires one additional explicit pump
transition after `FrameReady` and may be delayed until the caller invokes it.
The image and info use the acquisition time, including when readback completes
after later simulation steps. Info is admitted before worker startup; a later
startup or image failure does not retract it. Publication success is Transport queue admission,
not a delivery acknowledgement.

Reset epoch, camera removal/replacement, simulation fault or loss of image
interest invalidates an in-flight sample before publishing. The pump reports
`RetireRequired`; the caller must call `retire` outside the simulation step.
The owner remains bound to its original ECS entity and shared sensor metadata,
even while idle. Removal, replacement, or a loaded reset that recreates that
entity invalidates the owner permanently. `retire` only releases the worker;
it retains the two publisher endpoints. The caller must drop an invalid owner
before constructing a replacement, and drop the last owner when the sensor
family is absent. Loss of subscriber interest alone does not remove the owner.
Client teardown joins its supervisor and may wait indefinitely for a hard OS or
kernel stall. There is no hard real-time shutdown guarantee. The owner itself
must also be dropped outside the critical step. Busy/error paths are observable;
this API does not queue multiple frames or block a physics tick.

Earlier pump development checks passed eighteen CPU tests, including four message
controls and fourteen validation, demand and ownership controls. A typed
protocol peer exercises nine failure sequences without fabricating frames or
GPU faults. Six same-graph worker tests passed together with the Mesa software
Vulkan ICD, including a native SDK Image/CameraInfo subscriber, submitted
capture and completed readback invalidation, worker retirement/restart and
oversized image rejection. The default CLI's normal dependency tree excludes
this crate and GPU/window packages. The private strict renderer profile passed:
camera output covered 287/300 lines and 376/395 regions, and the renderer
workspace covered 4678/4808 lines and 6893/7175 regions. Prior failing profile
results remain separate. Independent replay of all six same-graph worker controls also passed on software
Vulkan. Headless, full staged and hosted acceptance are tracked on the public
rewrite dashboard. No physical GPU, native
camera image, calibration parity, scheduled sensor, performance or original test
port is claimed by these controls.
