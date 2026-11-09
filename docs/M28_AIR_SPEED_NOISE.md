<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M28: loaded AirSpeed pressure noise

The [source example](../examples/m28_air_speed_noise.sdf) extends the bounded
single direct-link Rapier AirSpeed sensor with Gaussian pressure noise. The
body stays at `(0,0,3)` in zero gravity. Its sensor remains rotated and offset;
the existing actual sensor-point velocity and composite-frame rotation apply.
The example adds mean `0.1`, standard deviation `0.02` and precision `0.001`.
This Source feature has its own qualification; it is not in an earlier preview.

## Native arithmetic and units

Pinned gz-sensors `8ddf4c17` AirSpeedSensor.cc applies noise to the **pre-100
float pressure**, promotes that value to the scalar noise model, assigns its
result back to float, then multiplies by `100.0f`. Temperature is untouched.
A mean of `0.1` therefore adds approximately **10 Pa** to the published number,
subject to those float rounding steps. Parameters are in native pre-100 units,
not the published Pa scale. The upstream protobuf hPa comment and the noise
constant's PASCALS name do not override the numeric implementation.

Noise is never added to an already published value or reconstructed by dividing
that rounded value. The shared pure numeric seam retains the original ideal
wrapper's exact outputs. A finite precursor whose unmodified final scale would
overflow can still be cancelled by finite noise before scaling. Negative ideal,
reverse-flow and noisy pressures remain valid; no positive clamp or pressure
variance field is invented.

## Supported configuration and lifecycle

Use `<air_speed><pressure><noise type="gaussian">...</noise></pressure></air_speed>`
or the existing `gaussian_quantized` type. Both kinds use the maintained scalar
Gaussian model. Mean, bias mean and dynamic-bias correlation time must be finite;
standard deviations and precision must additionally be nonnegative. `none`
requires every parameter to be exactly default. Gaussian with all-zero parameters
is enabled state, while default `none` owns no noise allocation.

Fixed bias gets the native random-sign initialization. Precision strictly above
`1e-6` activates half-away-from-zero rounding for either Gaussian kind; zero and
`1e-6` are inactive. Native `Apply` uses its default elapsed time **zero** here,
so dynamic bias does not evolve or consume dynamic draws. Sensor timestamps and
inter-acquisition gaps are not passed as noise elapsed time.

Each sensor owns its existing deterministic Rust stream, keyed by the world seed
and its actual spawned public identity. Acquisition consumes noise only when due
and demanded. Paused, non-due and disconnected steps consume no noise and do not
advance its deadline. Reconnection resumes the stream rather than accumulating
missed draws. Core reset recreates the seed, initial bias and deadline; loaded
reset reconstructs the same public identity with a fresh entity handle. Last
removal retires the family as before. There is no cross-language RNG bit promise.

Ideal input/domain errors retain `AirSpeedSampleError::Ideal`; nonfinite noise,
float conversion overflow and final float scaling overflow become
`AirSpeedSampleError::Noise`. A failed acquisition replaces the old sample with a
typed error and emits no previous valid wire packet. Schedule overflow remains a
separate terminal fault. Canonical type, frame, sequence, timestamp, temperature,
weak publisher leases and bounded reset-fenced IO are unchanged.

Zero wind, default atmosphere, one top-level direct-link sensor, Rapier, strict
metadata/rate bounds and all existing backend refusals remain. This adds no wind,
atmosphere, calibration, sensor family, parser, dependency or transport framework.

## Evidence boundary

Regression Source covers precursor/scale boundaries, quantization and units,
negative outputs, typed faults, seed/identity isolation, coexisting IMU draws,
pause/demand/cadence, reset replay and real canonical wire behavior. Original
ideal and ownership assertions remain. The native SensorFactory-only helper has
nine modes: `zero`, `mean`, `negative`, `gaussian-round`, `quantized`, `threshold`,
`active-boundary`, `bias-zero-dt` (five canonical rows each), and `white` (100 rows).
It checks native float outputs/temperature, actual timestamp/frame/sequence,
zero-elapsed fixed bias and predefined 100-sample five-sigma moment bounds.
No Physics or Sim plugin needs rebuilding for this focused noise comparison.

Source review is not execution evidence. Native execution, full mandatory normal
checks, strict workspace/per-crate/actual-parent coverage, exact hosted CI,
independent DATA and qualified Main acceptance remain required. Installed finite
example runs separately prove loading/stepping, not noise statistics. No native
RNG sequence, cross-engine trajectory or performance equivalence is claimed.
