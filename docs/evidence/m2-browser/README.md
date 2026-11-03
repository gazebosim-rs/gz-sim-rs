<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Actual M2 browser observations

Captured from the current viewer `5b67f45f` and simulator `7b8a4e10`, using the
exact qualified native-free instrumented CLI and current WebGL2 WASM bundle.
`observations.json` binds source trees, executable, fixtures, bundles and each
original JPEG. Files are actual captures, without reconstruction or re-encoding.

- `01-initial` and `02-initial-angle`: paused initial green mesh, red sphere and ground; three visual rows.
- `03-motion`: current clock 10.855 s, changed physical placement.
- `04-pause`: current/acknowledged pause at epoch 0, 16.800 s.
- `05-reset`: acknowledged epoch 1, zero time, paused; initial relative placement restored.
- `06-resume`: current clock 15.790 s; the resume ACK records epoch 1 at 0.005 s.
- `07-reconnect`: browser reload recovered current mesh scene at 158.130 s.
- `08-reconnect-pause`: current/acknowledged epoch 1 pause at 162.125 s.
- `09-invalid-image-toast` and `10-invalid-image-console`: separate corrupt-PNG world reports one actual visible production UI error with the local glTF and `pixel.png` decoding failure.

Initial/reset pictures support visible consistency, not pixel-exact or numeric
transform claims. Passing imported-mesh regressions separately establish the
frame/scale/material bindings. A uniform green image alone does not measure
texture sampling; the corrupt-PNG decoding error proves that image dependency
is used. Partial geometry can remain visible beside that clear error.

The background browser shows 1 fps and unavailable RTF/physics values as dashes.
WASM startup/reload caused Waiting/CUA timeouts before recovery. This is actual
functional WebGL2 evidence, not timing, performance, WebGPU-runtime or universal
asset-format acceptance. See [M2 acceptance](../../M2_ACCEPTANCE.md).
