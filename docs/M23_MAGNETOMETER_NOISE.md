<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M23: loaded magnetometer channel noise

This source-feature increment adds bounded Gaussian and Gaussian-quantized noise
to the existing single loaded Rapier magnetometer. M23_M24_DELIVERY.md defines
its scope and required qualification. Authored tests and native-reference source
are not executed evidence; parent M21/M22 qualification and this wave's own
mandatory checks and independent DATA are required before acceptance. This does
not change or relabel the installed M20 preview.

## Measurement and admission

The ideal model remains the inverse resolved sensor-world rotation applied to a
finite constant world magnetic field, in Tesla. Apply noise to the resulting
sensor X/Y/Z components in that order. Adding noise in world axes and rotating it
would give a different measurement and is not this contract. Translation has no
magnetic-field effect. The pose/frame graph, spherical-coordinate refusal,
nondefault-world-field preflight, one-sensor/Rapier restrictions, topic/frame,
metadata and existing publication protocol remain unchanged.

Each channel admits default-only `none`, or `gaussian` / `gaussian_quantized` with
finite parameters. White/fixed-bias/dynamic-bias deviations and precision must be
nonnegative; means and bias means can be signed. Correlation time may be any
finite value. Precision above the maintained 1e-6 threshold enables half-away-
from-zero rounding for either Gaussian kind. Negative precision is outside this
bounded subset; native warning-only handling is not claimed. Hidden nondefault
parameters under `none` are refused. The maintained SDF parser already rejects
negative deviations; Scene retains explicit validation for typed configurations.

The pinned MagnetometerSensor invokes `Noise::Apply(value)` without elapsed time.
Its default `dt=0` means no dynamic-bias evolution between acquisitions, unlike
IMU's elapsed-time integration. Fixed bias, including native random sign, is
constructed once per enabled channel. No covariance or normalization of a noisy
magnetic vector is invented. Nonfinite initial derived bias is refused.

## Ownership, cadence and errors

An all-`none` magnetometer owns no noise box. Enabled channels share one private
sensor NoiseStream keyed by existing load seed and stable public sensor EntityId;
construction/application order is X/Y/Z. Other sensor acquisitions do not consume
this stream. Existing loaders use zero seed; `load_world_seeded` selects an
explicit seed. Scene reordering can change public IDs; identity-independent or
native global-RNG byte equality is not promised.

Pause, no demand and non-due steps consume no draws. Reconnected demand resumes
the stream and existing phase-aligned deadlines. Direct reset clears samples and
cadence and reconstructs seeded biases/stream; it does not promise to restore a
separately changed physical pose. Loaded reset reconstructs the original physical
snapshot and public ID using a fresh Bevy entity, and replays initial seeded
samples. Last-sensor removal retires existing callbacks, interest, samples and
publication ownership; loaded restoration reinstalls them.

`MagnetometerSampleError::Ideal` preserves the original ideal-input error.
`MagnetometerSampleError::Noise` marks an invalid noisy vector. A new invalid
acquisition replaces the preceding valid sample and cannot encode or republish
stale valid data. Values are not clamped or fabricated. Consumed randomness is
not rewound after failure, following the existing GaussianNoise contract.
Schedule errors retain their existing terminal behavior.

## Example and evidence

`examples/m23_magnetometer_noise.sdf` retains the rotated/offset M17 rig, constant
world field `(0.00002, -0.00003, 0.00004)` Tesla and 100 Hz acquisition. It adds
three distinct channel configurations at topic `/m23/magnetometer`, frame
`rig/body/magnetometer`. Use the existing CLI world-loading path and canonical
subscriber; no seed CLI option, sensor UI or new endpoint is added.

Regression source retains ideal model/pose/domain/count/backend/lifetime controls
and adds post-rotation asymmetric means, single-channel/default compatibility,
rounding, independent seeded XYZ draws, coexisting noisy IMU, rates/demand/pause,
reconnect, both resets, teardown and invalid configuration/output. Canonical
Zenoh controls inspect actual vector, time, frame and sequence; typed noise
faults cannot encode or publish a previous good measurement.

`tests/native_magnetometer_noise.cc` provides `ideal`, `zero`, `mean`, `quantized`,
`white` and `bias-zero-dt` SensorFactory modes. White noise requires 100 actual
canonical packets and the retained upstream five-sigma moment bounds; other
modes require six deterministic packets with asymmetric field/rotated poses.
All modes check frame/time/sequence, finite vector values and live provider maps.
They do not replay an accepted physical trajectory or establish native reset,
C++/ChaCha sample-byte, cross-engine, performance or geographic parity.

No new sensor family, multiple magnetometers, runtime sensor spawning, geographic
field model, rate/reference service, noise-library rewrite, dependency, physics,
wire protocol, UI surface or broader compatibility is added.
