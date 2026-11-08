<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M22 loaded air-pressure noise

This increment adds one optional Gaussian channel to the existing single loaded
Rapier air-pressure sensor. The ideal model still uses absolute attached-sensor
world Z, default atmosphere and reference altitude zero. Its pure model API and
all topic, frame, sensor-count, metadata, backend and lifecycle restrictions are
unchanged. The example is `examples/m22_air_pressure_noise.sdf`; its initial
sensor world Z is 3.5 m, rate 100 Hz and topic `/m22/air_pressure`.

The selected scope and qualification boundary are in `M21_M22_DELIVERY.md`.
Source and tests are not passing evidence. Parent M20 acceptance, independent
review, the full normal hook, exact hosted CI, native observations and retained
DATA remain required before this increment is accepted. This feature does not
change the installed M20 package or claim a new binary preview.

## Native scalar behavior

`gaussian` and `gaussian_quantized` use the unchanged `GaussianNoise` and
`NoiseStream` implementations after a valid ideal pressure calculation. Both
kinds apply configured precision when greater than `1e-6`, with native
half-away-from-zero rounding. Parameters are SI pressure units except the
correlation-time field. Mean and bias mean may be signed. All parameters must
be finite; standard deviations and precision must be nonnegative. Negative
precision remains explicitly unsupported, rather than reproducing native's
warning-only admission. Type `none` remains default-only.

The pinned native `gz-sensors` revision is
`8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`. In
`src/AirPressureSensor.cc`, `Update` calls `Noise::Apply(pressure)` without an
elapsed-time argument. `include/gz/sensors/Noise.hh` defaults that argument to
zero. Accordingly this channel always passes `dt=0`: finite configured dynamic
bias parameters do not evolve the bias, even across long sampling or demand
gaps. This is different from IMU elapsed-acquisition-time noise.

Native publishes `sqrt(StdDev())` in the canonical `FluidPressure.variance`
field. That double-precision calculation is preserved exactly, including zero
as the schema's **unknown variance** value. It is not redefined as standard
deviation squared or rounded through IMU's float covariance path. The native
integration test `test/integration/air_pressure.cc` explicitly expects
`sqrt(0.2)` for standard deviation 0.2.

## Seed, acquisition, errors and reset

Only an enabled Gaussian channel owns boxed state. It uses the existing load
seed and the stable public sensor ID, independently of other sensors' streams.
Existing loaders use the all-zero seed; `load_world_seeded` permits explicit
Rust replay. Paused ticks, absent demand and non-due ticks consume no random
values. Invalid ideal atmosphere input also leaves noise state unchanged.

An actual valid ideal acquisition may consume random state even if applying
noise fails. `AirPressureSampleError` distinguishes the original ideal-domain
error, noise arithmetic error, and finite nonpositive noisy pressure. The new
sample replaces the old sample at its actual acquisition time. There is no
clamp, fabricated zero or reuse of a previous valid pressure. Server counts an
invalid sample once, sends no packet, and retains its sequence for the next
valid publication. Packet validation continues to reject nonfinite/nonpositive
pressure and nonfinite/negative variance.

Direct reset recreates the initial seeded noise state and clears the sample and
deadline. Loaded reset restores the same public identity and initial snapshot;
last-model removal drops state and sampling/reset callbacks, with restoration
outside stepping. This is deterministic Rust replay, not byte equality with
native's global RNG or a claim about native noise reset behavior. Demand-based
acquisition and fixed rate remain the existing pressure policy; this increment
adds no runtime reference or rate service.

## Regression and native evidence contract

The ideal tests remain, with their exact ideal error wrapped in the new typed
sample error. The old Gaussian admission negative is replaced by a negative
precision refusal because Gaussian is now admitted. Server's old positive
variance refusal becomes a positive double-preservation control; all NaN,
infinity, negative-variance and invalid-pressure refusals remain.

Scene tests cover exact default compatibility and both kinds' mean/rounding,
seed replay and seed divergence, independence from a simultaneously noisy IMU,
100-sample white moments, pause/demand/deadline no-draw behavior, native zero-dt
bias, direct and loaded resets, removal/restoration and fresh invalid samples.
Private controls preserve the original ideal error and prove an invalid ideal
acquisition consumes no noise. Real Zenoh controls check canonical bytes,
double variance, acquisition stamp/frame/sequence, reset replay, disconnect and
reconnect, weak-lease retirement, and a physical valid-to-nonpositive transition
that must not resend the previous valid packet. Existing bounded IO and endpoint
ownership tests remain unchanged.

`crates/gz-sim-scene/tests/native_air_pressure_noise.cc` is a focused direct
native `AirPressureSensor` capture. Its modes are `ideal`, `zero`, `mean`,
`gaussian-round`, `quantized`, `white` and `bias-zero-dt`; arguments are
`MODE MAPS_PATH`. Each deterministic mode emits five canonical messages over
known physical heights or fixed height with one-second time gaps. White mode
emits 100 messages and uses the retained upstream `NoiseTest` five-sigma mean
and unbiased-variance bounds. It re-supplies the physical pose before every
update and captures actual loaded libraries while the sensor remains alive.
Root owns the bounded native procedure and qualified compiler/SDK/provider
bindings. Native and Rust stochastic samples are compared by their contracts
and moments, never by cross-language RNG byte identity.

No second pressure sensor, other backend, atmosphere layer, custom/image noise,
runtime sensor spawning, noise/reference/rate service, browser widget, native
backend distribution, general compatibility or performance claim is included.
