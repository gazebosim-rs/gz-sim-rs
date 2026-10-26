# Runtime IMU rates

Copyright (C) 2026 Open Source Robotics Foundation
SPDX-License-Identifier: Apache-2.0

Loaded IMUs expose synchronous `gz_sim_scene::imu_rate` and `set_imu_rate`
functions using their public entity IDs. `Server` also advertises
`<sensor-topic>/set_rate`, with `gz.msgs.Double` requests and `gz.msgs.Empty`
response type. Gazebo treats Empty services as **one-way**. Transport handoff
success does not confirm application; no completion or rejection reply exists.
Observe acquisition or query local state when completion matters.

## Admission and phase

The original SDF rate sets the service ceiling. Native comparisons use the strict
predicate `a < b + 1e-6`; an SDF rate below `1e-6` disables the ceiling. With a
ceiling, near-zero and above-limit requests are ignored. Ignored requests preserve
all state. Negative requests clamp to positive zero; negative zero keeps its sign.

Changing a rate preserves the pending acquisition deadline, velocity history and
noise state. Full and time-only resets restore the loaded rate and restart the
sensor instance, phase and publication sequence. Both reset paths retire queued
controls already handed to the old service receivers. In-flight network arrivals
have no provider-generation fence in the native protocol and can reach a current
same-topic/type handler after reset.

For an uncapped sensor, native runtime control accepts NaN and positive infinity.
NaN and zero bypass the base sampling-period gate; positive infinity advances by
one nanosecond. The IMU system still checks whether **any connected IMU's raw
pending deadline** is due before updating sensors. Consequently, zero or NaN can
wait for an earlier future deadline, while another due connected IMU can admit
continuous acquisition. This coupling is preserved. Loader rejection of invalid
SDF rates remains unchanged.

Unrepresentable finite runtime periods return `ImuRateError::InvalidPeriod`
before mutation. Native stores those values, but its subsequent floating-to-clock
cast lies outside the safely qualified range. The one-way endpoint records a
local rejection; it cannot report a fabricated remote error response.

## Bounded service execution

The server polls a finite number of rate requests after each tick, including
paused ticks. `Counters` separates applied, ignored and rejected requests.
`transport_statistics()` aggregates world and rate-worker counters, including
queue drops. Limits remain explicit `gz_transport::Config` values.

Distinct sensors sharing a topic advertise distinct service providers. A request
changes one selected recipient, without fanout or a stable first-created choice.
A pool minimizes IO workers subject to per-Node capacity and duplicate-topic
constraints. Workers are created during setup and retained until shutdown;
reset and stepping replace service leases without creating or joining workers.
Scenes with many identical topics still require many workers with the current
Transport API; that scaling limit is not qualified as a performance improvement.

## Reproducing native references

Use the pinned repositories in `planning/baseline.json`, the qualified C++ build
and its dependency installation. Capture finite reset behavior with:

```sh
python3 scripts/probe-imu-rate.py --kind reset --engine tpe \
  --upstream-root /path/to/native/repos --sim-build /path/to/gz-sim-build \
  --dependency-prefix /path/to/install --work-dir /tmp/imu-rate-reset
```

Repeat with `--engine dartsim` for the dynamic reference. `--kind future` captures
zero/NaN admission with and without a second connected sensor. The qualified
future-deadline reference uses TPE; DART future-mode qualification is separate.
Optional `--headless-reference` and `--schedule-reference` select matching
qualification reports for another build. Their source and library identities
must match; a different compiler/image is not silently interchangeable.

The helper records raw packets, callback drain boundaries, actual iteration/time/
pause values, compiler and selected headers, source/cache identities and loaded
libraries. Failures retain workflow stage, partial inputs and raw diagnostics.
Its capture result requires independent interpretation; it does not declare Rust
parity. The standalone base-Sensor getter/phase/shared-provider reproducer is
`crates/gz-sim-scene/tests/native_imu_rate_bounds.cc`; compile against the pinned
`gz-sensors` pkg-config package. Base Update behavior alone does not qualify the
full IMU system's admission gate.

The real C++ rate client is built by `scripts/native-peer.py --client rate` and
exercised by `native_rates.rs` on TPE and Rapier. stdin/stdout synchronize the test
harness; a local processed-counter barrier avoids inventing an Empty response.
Exact native-derived sample times, sequences, physical readings and covariances
are checked. Delayed delivery, nonzero exit and Rust-unwind/hung-child cleanup are
separate controls. Existing clock/control/IMU clients remain mandatory.

This increment covers loaded IMU rates. Activation, triggers, metrics, runtime
sensor registration, other sensor families and full SENSOR-01/IO-01/P2 acceptance
remain separate work. Strict coverage, independent review, equal-work performance,
protected normal hooks and exact hosted CI are required before acceptance.
