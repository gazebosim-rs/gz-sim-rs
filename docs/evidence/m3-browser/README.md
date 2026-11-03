<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Actual M3 browser observations

These are byte-exact copies of the original browser JPEG captures and their
source/binary binding manifest. The manifest preserves paths in the capture
environment; each repository image has the same recorded filename and hash.
The simulator is `ea8048f1`; the existing viewer is `5b67f45f`. Its accepted
WASM bundles were reused, with the exact instrumented native-free simulator
executable retained by the mandatory M3 gate. No extra build was needed.

| Capture | Actual visible observation |
| --- | --- |
| [Initial](01-initial.jpg) | Paused at 0 seconds after camera framing. |
| [Motion and contact](02-contact.jpg) | Current time 9.875 seconds; mesh and sphere moved onto the ground. Resume acknowledgement: epoch 0, 0.005 seconds, paused false. |
| [Pause](03-paused.jpg) | Current time and pause acknowledgement 19.765 seconds, epoch 0, paused true. |
| [Reset](04-reset.jpg) | Current time 0 seconds, reset acknowledgement epoch 1, paused true; original apparent placement returns at the same camera. |

The captures prove production UI observations and qualitative motion/rest/reset.
Quantitative geometry, static contact, convex-only admission, inertia, collision
frames and resource/reset ownership use the separate named regression tests.
Background 1 fps is visible; this is not performance acceptance. The renderer
backend was not separately inspected in this M3 capture. Startup initially showed
Waiting/Connecting before the scene appeared. One unsupported CLI setup argument
was corrected before startup and is not counted as acceptance.

The manifest's pending hosted-CI/publication statement records capture-time
status. It is preserved byte-exact; current acceptance is recorded in
[../../M3_ACCEPTANCE.md](../../M3_ACCEPTANCE.md).
