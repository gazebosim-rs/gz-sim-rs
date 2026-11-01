# Runtime IMU rates

Copyright (C) 2026 Open Source Robotics Foundation
SPDX-License-Identifier: Apache-2.0

Loaded IMUs expose synchronous `gz_sim_scene::imu_rate` and `set_imu_rate`
functions using their public entity IDs. `Server` also advertises
`<sensor-topic>/set_rate`, with `gz.msgs.Double` requests and `gz.msgs.Empty`
response type. An Empty receipt acknowledges queue handoff, not the later
applied, ignored or rejected outcome. Observe acquisition, counters or local
state when application matters. Full ingress refuses handoff and cannot return
a successful Empty receipt.

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

When configured IMUs remain, the server polls a finite number of rate requests
after each tick, including paused ticks. `Counters` separates applied, ignored
and rejected requests. `ingress_statistics()` reports the shared world-lifetime
queue-drop counter, including actual rate ingress losses. Limits are explicit
`ServerOptions::capacity` and `max_message_bytes` values.

Distinct sensors sharing a topic advertise distinct service providers. The
selected direct remote Zenoh contract delivers a matching request to both
providers in the retained two-provider case, with an Empty handoff receipt from
each accepted queue. It does not promise the old ZeroMQ single-recipient choice.
CPU providers retain only scalar receivers and active gates. With no current
providers there is no retained Rates CPU owner, provider heap or receiver, and
no Rates poll or retirement call. Removing the last provider drops that CPU
owner and discards unread scalars. IO sender/callback state can retain the
disconnected channel allocation until its separate retirement completes.
A partial removal preserves the surviving providers and their
round-robin budget. Reset reconstructs demand from loaded Scene metadata rather
than retaining an empty provider template.

Direct Zenoh queryables, discovery tokens and Query values stay on existing
world IO owners. Construction and reset installation/acknowledgment occur outside
raw ticking. CPU retirement disables gates without waiting for callbacks; IO
retains callback/queued-query completion witnesses until native retirement ends.
Late Full accounting still uses the shared world metric. This CPU ownership
change does not establish whole ACT-01 absence, bounded network shutdown or a
performance improvement; other world work and pending native retirement remain
separate concerns.

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

The real C++ rate client is built through `scripts/native-peer.py` with explicit
`--client rate --profile gazebo-zenoh-client-v1 --json`, using the qualified
reference root. `native_rates.rs` exercises it on TPE, Rapier, DART and MuJoCo. stdin/stdout
synchronize the harness. Actual Empty queue-handoff
receipts and the later local applied/ignored/rejected outcomes are checked
separately; a receipt does not prove that a rate changed.
Exact native-derived sample times, sequences, physical readings and covariances
are checked. Delayed delivery, nonzero exit and Rust-unwind/hung-child cleanup are
separate controls. Existing clock/control/IMU clients remain mandatory.

This increment covers loaded IMU rates. Whole ACT-01, triggers, metrics, runtime
sensor registration, other sensor families and full SENSOR-01/IO-01 completion
remain separate work; selected P2 acceptance does not close those packets.
Strict coverage, independent review, equal-work performance,
protected normal hooks and exact hosted CI are required before acceptance.
