<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M21: loaded Rapier altimeter channel noise

This source increment extends the existing single loaded altimeter with Gaussian
and Gaussian-quantized vertical-position and vertical-velocity noise. The scope
and required qualification are in M21_M22_DELIVERY.md. Authored regressions and
native-reference source are not an executed qualification or a refreshed binary
preview. M20 acceptance remains a prerequisite for accepting this increment.

## Measurement and admitted configuration

The ideal measurement remains current sensor world Z minus its fixed resolved
load reference, and world-Z velocity at the actual offset sensor point. Rotation
of the sensor does not rotate the measurement axis. Noise is applied to position
first, then velocity; the reference is never noised. The pure ideal model and
shared Gaussian implementation remain unchanged.

Each SDF channel admits `none` with all default parameters, or `gaussian` /
`gaussian_quantized` with finite parameters. Means and bias means may be signed.
Standard deviations (white, fixed bias and dynamic bias) and precision must be
nonnegative. Correlation time may be any finite value. Negative precision and
negative deviations are outside this bounded subset, even where native code
merely warns or disables a contribution. Nonfinite or unrepresentable initial
biases are refused. Positive precision above the maintained 1e-6 threshold uses
native half-away-from-zero rounding, for either Gaussian kind.

The pinned native AltimeterSensor calls `Noise::Apply(value)` without elapsed
time. Its default `dt=0` means dynamic bias does not evolve between acquisitions,
even at a slow or irregular acquisition interval. The implementation deliberately
does not copy IMU elapsed-time noise behavior. Fixed bias is drawn once at
construction, including native random sign; white noise is drawn per acquired
channel. No covariance field is invented for canonical Altimeter messages.

`examples/m21_altimeter_noise.sdf` uses the existing rotated/offset Rapier fixture
with both channels enabled, a 100 Hz rate, topic `/m21/altimeter`, and frame
`rig/body/altimeter`. Use the existing native-free CLI loading path and a canonical
subscriber in its partition. This phase adds no CLI seed setter, UI widget or
new transport endpoint.

## Stream ownership, demand and reset

An all-`none` sensor owns no noise box. An enabled channel uses the existing load
seed and stable public sensor EntityId to own a private NoiseStream. Within that
stream, construction and application order is position then velocity. Other
sensors do not consume that stream. `load_world_seeded` selects an explicit seed;
existing loader entry points use the all-zero seed. Reordering scene entities can
change public identities; identity-independent or native RNG-byte compatibility
is not claimed.

Paused, non-due and uninterested steps acquire nothing and consume no random
values. Restored interest resumes the existing stream without reseeding. Cadence
keeps the existing phase and actual acquisition timestamps. Direct reset reseeds
the stream and biases, clears the sample and resets the deadline; it does not
promise to restore an independently changed physical pose. Loaded reset restores
the original physical/configuration snapshot and public identity, using a fresh
Bevy entity, and reproduces initial seeded samples. Removing the last sensor
retires its existing callbacks, sample, interest and publication lease. Loaded
reconstruction reinstalls them under the established reset contract.

`AltimeterSampleError::Ideal` preserves the original ideal-input error;
`AltimeterSampleError::Noise` identifies nonfinite noisy output. A failed sample
replaces the previous valid sample and cannot publish stale valid data. There is
no clamp or fabricated measurement. Randomness already consumed on a failed
application is not rewound, matching the shared GaussianNoise policy. Schedule
faults retain their existing terminal behavior.

## Evidence and limits

Regressions retain ideal reference/offset-velocity assertions and the existing
sensor-count, metadata, engine, endpoint, lifetime and reset refusals. New controls
cover signed means/quantization, single-channel/default behavior, ordered seeded
draws, independent coexisting IMU streams, different seeds, rate/demand/pause,
loaded/direct reset, teardown, invalid parameters and noisy-output overflow.
Real canonical Zenoh controls inspect noisy values with time/frame/sequence and
noise-free reference, reconnect and loaded reset; noise faults cannot encode.
Existing weak-lease and IO retirement controls remain unchanged.

The bounded native helper `tests/native_altimeter_noise.cc` has `deterministic`,
`white` and `bias-zero-dt` modes, each requiring 100 actual canonical packets.
It refreshes native physical inputs before every update because native updates
mutate stored readings. Deterministic means/quantization and fixed-bias invariants
are checked directly. White-noise mean/variance use the retained upstream
100-sample/five-sigma policy. All modes check actual timestamps, frame, sequence,
reference, finite values and live provider maps. This is sensor-model evidence;
it does not compare C++/ChaCha random bytes, native reset streams or physical
trajectories. The accepted ideal M13 point-velocity evidence remains separate.

No multiple altimeters, runtime sensor spawning, reference/rate services,
additional backend, geographic behavior, noise-library rewrite, dependency,
protocol, browser surface, performance or broader compatibility is added.
