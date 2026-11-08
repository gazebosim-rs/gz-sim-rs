<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M21/M22: loaded altimeter and air-pressure noise

The functional scope below is retained. [M21_M25_DELIVERY.md](M21_M25_DELIVERY.md)
now supersedes its intermediate-parent sequencing: all reviewed M21–M25 source
qualifies in one new candidate over accepted M20, with new full normal/exact CI
and every separate native/functional and fresh M25 distribution criterion before
accepting these phases together. Earlier checkpoint trees and results remain
unchanged historical evidence, not current whole-gate success.

Selected under D30–D32. Each increment has a budget of at most five working
days including independent review and mandatory verification. M1–M20 are
accepted. The source parent is M20 review commit
`9a3e81cf8babbc87b305f09dd04458fddc70765f`: its full normal hook and independent
local audit passed. It is now accepted and published as `m20-preview`, after
exact hosted run 42479817732 attempt 1, official/package/install/browser DATA and
actual release 438259397 with seven checked asset digests. See
[M20 acceptance](M20_ACCEPTANCE.md). This wave authors Source in parallel and
inherits no feature acceptance. Parent corrections require rebasing and affected
checks; the parent acceptance prerequisite has been satisfied.

## Two independent increments

M21 adds Gaussian and Gaussian-quantized noise to the existing single loaded
Rapier altimeter's vertical position and velocity channels. Its ideal formula,
rotating-offset point velocity, noise-free load reference, topic, frame, sample
schedule, demand and backend restrictions stay intact.

M22 adds the same admitted noise kinds to the existing single loaded Rapier
air-pressure channel, after its unchanged absolute-world-height atmosphere model.
The default atmosphere and zero reference-altitude restriction stay intact.
Canonical pressure variance follows the pinned native sensor's
`sqrt(stddev)` calculation, including double precision; it is not IMU covariance.
Server validation admits finite nonnegative variance and preserves finite
positive pressure and all other packet/identity/epoch/lease/queue guards.

Both sensors' pinned native implementations call `Noise::Apply` without elapsed
time, whose default is zero. Apply noise with `dt=0`: configured dynamic bias
does not evolve with acquisition intervals. Reuse `GaussianNoise` and
`NoiseStream` unchanged. No shared noise framework or dependency is added.

## Admission and ownership

Admit finite Gaussian parameters, nonnegative standard deviations and precision,
and finite signed means/bias means. Finite correlation-time values have no
dynamic evolution at zero elapsed time. Precision uses the maintained native
threshold and rounding behavior for either Gaussian kind. Negative precision is
explicitly outside this bounded subset; native warning-only handling is not
claimed. `none` remains default-only. Preserve every existing sensor count,
metadata, atmosphere/reference, engine, plugin and lifecycle restriction.

Only an enabled Gaussian channel owns optional boxed noise state. Reuse the
existing load seed and stable public sensor ID for each sensor's stream. M21
applies position before velocity. Paused, uninterested and non-due acquisition
consume no random values. A valid ideal acquisition may consume noise state;
nonfinite noisy outputs, and nonpositive pressure, produce a typed invalid sample
and no packet, with no clamp or fabricated measurement. Distinguish ideal and
noise errors. Reconstruct seeded state on direct and loaded reset, retaining the
existing sample/deadline clearing and ownership teardown. This establishes Rust
deterministic replay, not native global-RNG/reset byte equality.

## File leases and parallel execution

| Owner | Worktree | Exclusive source responsibility |
| --- | --- | --- |
| Light | `gz-sim-m21-altimeter-noise`, branch `work/m21-altimeter-noise` | Scene `src/altimeter.rs`, `tests/private/altimeter.rs`, new `tests/altimeter_noise.rs` and `tests/native_altimeter_noise.cc`; Server `tests/private/altimeter.rs`, `altimeter_io.rs` and `altimeter_world_io.rs`; new `examples/m21_altimeter_noise.sdf`; `docs/M21_ALTIMETER_NOISE.md`. |
| Messages | `gz-sim-m22-air-pressure-noise`, branch `work/m22-air-pressure-noise` | Scene `src/air_pressure.rs`, `tests/private/air_pressure.rs`, new `tests/air_pressure_noise.rs` and `tests/native_air_pressure_noise.cc`; Server `src/air_pressure.rs` and `tests/private/air_pressure.rs`, `air_pressure_io.rs` and `air_pressure_world_io.rs`; new `examples/m22_air_pressure_noise.sdf`; `docs/M22_AIR_PRESSURE_NOISE.md`. |
| Root | `gz-sim-m21-m22`, branch `work/m21-m22-sensor-noise` | Shared exports/test attachments only when needed; planning, AGENTS/README/HANDOFF/status/tracker and Scene guide routing; integration, native procedure and every qualification/publication operation. |
| SDF | Read-only | Independent complete source/test-intent review and batched findings. |

The two leaf implementations must not edit Scene's shared `noise.rs`, the pure
sensor models/noise library, Cargo/dependencies, protocols, viewer or each other's
files. Coordinate a concrete additional lease before using another file. The
Server test lease is for actual existing wire-fixture regressions, not a new
transport abstraction. Shared native-client procedure changes belong to Root.
M20's frozen candidate, accepted previews, paused trees and Fuel are untouched.

Source lanes may overlap M20 hosted/artifact work. Authors run no tests, lint,
native captures or product commands. Root serializes local heavy qualification.
Complete both implementations, meaningful regressions and guides, batch the
independent review findings, then execute one full unchanged normal hook and
required native evidence. Batch actual failures before affected reruns; preserve
all thresholds, assertions, numerical tolerances and hosted jobs.

## Acceptance evidence

1. Retain all old ideal and refusal controls. Add exact mean/rounding cases,
   one-channel/default compatibility, unchanged physical reference/offset
   behavior, seeded repeatability and differing seeds, stable independent
   streams, pause/demand/non-due no-draw behavior, rate/reconnect behavior,
   direct/loaded reset replay, teardown and invalid configuration/output cases.
   Tests must distinguish zero-elapsed bias from IMU elapsed-time semantics.
2. Observe canonical actual Zenoh packets with time, frame, sequence, reference
   and pressure variance; preserve endpoint/weak-lease/reset/invalid-packet
   behavior. New noise faults cannot publish stale valid measurements.
3. Use one bounded, independently reviewed native capture batch against the
   pinned installed sensor provider. Set native altimeter physical values before
   each update because update mutates readings. Compare deterministic mean and
   quantization cases directly, native pressure variance exactly, and stochastic
   properties using retained upstream sample/tolerance policy. Do not compare
   C++ and ChaCha random sample bytes or infer trajectory parity.
4. Pass the mandatory full normal hook, strict workspace/per-crate/actual-parent
   coverage, exact hosted CI and independent retained audits. Promote only the
   qualified source after parent acceptance. Publish actual evidence, update the
   web tracker and continue the next useful bounded phase.

No additional sensors, atmosphere layers, runtime sensor spawning, reference/rate
services, native backend distributions, browser widgets, geographic behavior,
noise-library rewrite, new RNG model, general compatibility or performance claim
is included. A future refreshed binary preview is a separate explicitly scoped
artifact delivery; this wave does not silently relabel the M20 package.
