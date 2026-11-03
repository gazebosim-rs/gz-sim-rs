<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M5: bounded RGB camera delivery

Accepted at simulator Main `60a3a710`; see [M5_ACCEPTANCE.md](M5_ACCEPTANCE.md).
One configured SDF RGB camera uses existing offscreen Bevy rendering and bounded
canonical image publication. Network/GPU waits stay outside critical simulation
stepping; M1–M4 remain preserved.

## Application and retained boundaries

The accepted CameraServer already owns loaded topic/frame derivation, reset
preparation, identity/removal checks, output cancellation, a separate upstream
Zenoh session, and ScheduledCameraOutput over the existing bounded worker IPC.
Reuse it rather than adding a sensor runtime, scheduler, transport facade or
plugin loader. The current worker/image contract is 64×64 RGB_INT8, opaque box
visuals and no world lights. The known fixture remains within that domain;
broader geometry, formats, arrays and camera effects are not silently admitted.

CLI activation is explicit: build `gz-sim-cli` with `--no-default-features
--features rgb-camera`, then run the resulting `gz-sim-rs` with a local SDF,
`--server --run --camera-worker /absolute/path/to/gz-render-worker`. The worker
must be the existing GPU-enabled worker build; the CLI process retains CPU-only
render protocol/client/scene dependencies. Both sessions read the upstream
`ZENOH_CONFIG` file when provided, and canonical topics retain `GZ_PARTITION`.
The world Server keeps its configured listeners. The separate camera peer has
no listening endpoint and retains configured outgoing connect/scouting rules,
so it cannot consume a fixed world-server port. For isolated deployments,
configure both peers to connect to the observer/router as in the regression.
Direct incoming connections to the camera peer are not provided by this CLI.
Default-config/no-image admission is tested separately from remote image delivery.
This is application session configuration through the unmodified upstream API;
programmatic CameraServer construction and its error/unwind ordering are retained.
The admitted fixture is `crates/gz-camera-output/tests/fixtures/m5-rgb.sdf`.
The optional `--web-viewer 9007` serves the unchanged primitive viewer protocol;
RGB images are delivered through canonical Zenoh image topics.

Without the build feature the flag fails before world/resource access. Without
a configured camera, opting in leaves the ordinary headless Server and never
validates or starts the unused worker. With a configured camera but no image
interest, worker creation stays lazy. Default builds retain their existing native
backend selection; `--no-default-features` remains the explicit native-free mode.
Both strict coverage profiles include the CLI with RGB enabled; the retained
native-off qualification also executes feature-disabled refusal.

CameraServer.update may commit physics before output returns Busy. A caller
must preserve that committed UpdateInfo and report bounded backpressure without
repeating physics or acquisition. The source increment exposes existing tick
and capture operations separately; update still composes them with its retained
error semantics. No new scheduling or capture algorithm is introduced.

The focused real-worker/upstream-Zenoh regression source checks actual pixels,
axes, calibration, frame, dimensions, 10 Hz acquisition times, paused silence,
Busy without a repeated tick, retained capture timestamps, reset invalidation
and removal retirement. The worker Cargo test entry delegates to the owned
camera-output regression module so the real worker executable is available.
Existing assertions and deadlines remain intact.

CPU-only CLI controls cover admission, feature requirements, absent demand,
lazy worker creation, invalid camera/worker inputs, separate fixed listeners and
viewer setup failure/cleanup. Additional real subprocess controls exercise an
unresponsive operator worker and failed startup through actual Zenoh image
interest, asserting bounded backpressure and preserved advancing ticks without
claiming an image from that fixture. A failed worker spawn now closes request
admission after publishing its original IO error, so pre-Ready acquisition cannot
remain in NotReady indefinitely. The native-free RGB proof runs these in its own
fresh target and retains two Cargo-bound executables, raw counters, exports and
closure evidence separately from the unchanged original ten-case/four-ELF
feature-disabled proof. No profile union or new native-free whole-source coverage
claim is made. The actual renderer control launches the verifier-bound CLI and
worker, checks real canonical images and pause/resume, and requires exactly
10,000 advancing ticks before clean exit.

Independent source review, final required staged verification, exact hosted CI
and independent retained-data audits passed. Earlier failed attempts remain
failed evidence; their batched corrections preserve all quality gates. Preserve strict owned and changed-line
coverage gates and the accepted primitive/mesh/IMU demos. The paused depth
candidate remains deferred until M6 needs it.
