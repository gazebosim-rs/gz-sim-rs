<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M7: bounded planar CPU lidar

The complete Source batch is implemented and under final independent review;
required verification and runtime observations are pending. This document is not
acceptance evidence.
M1–M6 remain accepted unchanged. M7 is budgeted at most five working days,
including independent review and mandatory checks.

One loaded SDF `lidar` (`ray` alias) uses the existing scheduler and Rapier's
maintained shape ray casts. The admitted ideal domain is 2–128 inclusive,
strictly increasing horizontal angles spanning no more than one turn, horizontal
resolution 1, one zero-angle vertical layer, finite ordered ranges with
0 <= minimum < maximum, nonnegative finite range-resolution metadata, default
no-noise configuration and the full visibility mask. GPU, multilayer,
interpolated, noisy, filtered-visibility and multiple-lidar inputs are refused.
Other physics backends explicitly refuse this delivery's sensor admission.

Directions are local XY, zero along +X; angles are radians and distances metres.
Each query uses the actual committed sensor world pose, excludes only its parent
rigid body, and includes admitted primitive and mesh collision geometry.
Queries neither step physics nor update a collision cache. Returns remain
continuous; range resolution does not imply invented quantization. Hits below
minimum clip to minimum. No hit through maximum is positive infinity. A
canonical scan carries ideal zero intensities, without a reflectivity claim.

A loaded sensor installs its own PostStep/reset callbacks. An absent family owns
no lidar callbacks, interests or scan buffers. Explicit empty transport/viewer
interest performs no queries and clears stale samples. Standalone Simulation
callers can restore local acquisition with `clear_lidar_interest`. Acquisition
uses simulation cadence and time; paused ticks do not acquire. Reset restores
original configuration and identities, clears samples and restarts cadence in a
new local epoch; canonical schema fields do not invent an epoch field. Removing
the final configured sensor retires callbacks and interest storage.

Canonical LaserScan output and the CLI measured-scan bridge are connected in
the complete Source batch; see `LIDAR_SERVER_OUTPUT.md` for bounded IO ownership. The bridge admits exactly one opt-in query
`scan=1`, atomically captures each committed Scene with its bounded scan envelope,
sends that Scene before its scan, and
coalesces latest measured data at viewer cadence. Default clients receive no
scan messages and allocate no measured scan payload. Disconnect clears viewer
demand; canonical Zenoh demand remains independent. Acquisition timestamps and
poses remain those of the sample, never interpolated to the later Scene time.
World geometry and public IDs remain unchanged.

The existing fixture is a runnable static-wall demonstration after qualification:
`cargo run -p gz-sim-cli --no-default-features -- examples/m7_planar_lidar.sdf --server --run --web-viewer 9007`.
Connect the M7 viewer with `ws://127.0.0.1:9007/?scan=1`; it draws only real measured
returns. Near clipping, misses and unsupported domains remain explicit rather
than drawing fabricated maximum-range endpoints. Actual runtime/browser and
required coverage/CI evidence are still pending; this is not a shipped demo.
The viewer's separate `scan=1` channel preserves scene geometry and public IDs;
its exact bounded contract is documented in gzweb-rs `docs/M7_LIDAR_PREVIEW.md`.
No accepted M1–M6 manual qualification is repeated. No M7 tests, lint, builds or
runtime probes run until implementation, regression sources, documentation and
independent Source review are complete.

The Source admission update preserves every retained input and negative assertion.
Exactly three existing Rapier CPU cases change from rejection to explicit measured
positives: one 16-beam typed sensor and the two 2-beam lidar/ray aliases. A shared
same-input helper additionally proves source metadata, real geometric misses,
identity/frame/pose/cadence, no-demand, pause, reset and teardown. Seven
unsupported-engine rows retain their original exact `non-IMU sensors` diagnostic;
GPU aliases and wrong-wrapper/parser negatives remain unchanged. This scoped
transition was independently reviewed before application, with no test execution.
