<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M15 ideal air pressure

This increment admits one loaded-world, noise-free air-pressure sensor on Rapier.
It consumes the existing typed SDF configuration and publishes canonical
`gz.msgs.FluidPressure` through the existing upstream Zenoh IO owner. The
[delivery contract](M15_M16_DELIVERY.md) requires independent Source review,
actual native observations, the unchanged full normal hook, exact hosted CI,
independent DATA and qualified Main promotion. This document describes source;
it does not claim these checks or functional proofs have run.

## Measurement

Pressure is in pascals at the resolved **absolute sensor world Z**. A rotated
sensor offset changes its world height; sensor orientation does not rotate the
measurement axis. There is no subtraction of the sensor's initial height.
`variance=0` has the canonical message meaning **unknown variance**, not certainty.

The constants and geopotential formula preserve pinned `gz-sensors`
`8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`, `src/AirPressureSensor.cc`,
which attributes the model to RotorS's `rotors_gazebo_plugins` pressure plugin.
The original OSRF copyright and Apache-2.0 attribution are retained in source.
For height `z`, Earth radius `R=6356766 m`, sea-level temperature `T0=288.15 K`
and lapse `L=0.0065 K/m`, the calculation is:

```text
H = R*z/(R+z)
T = T0-L*H
P = 101325*exp((9.80665*28.9644/(8314.32*(-L)))*log(T0/T))
```

Only the implicit default atmosphere and reference altitude zero are admitted.
The native sensor's `Load` does not read SDF `reference_altitude`, and its system
never calls `SetReferenceAltitude`; M15 explicitly refuses nonzero SDF values
rather than silently ignoring them. There is no runtime reference setter.
Explicit world atmosphere, even with default values, remains refused. No layered
atmosphere, temperature/environment control, clamps or noise model is added.

Nonfinite height/intermediates, heights at/below the Earth-center singularity,
nonpositive temperature and nonfinite/nonpositive computed pressure return
`InvalidAirPressureInput`. Initial invalid domains are refused at load. A later
invalid acquisition retains its typed error with the acquisition epoch/time;
Server counts and suppresses it without fabricating a pressure value, matching
the established altimeter error policy. Sampling-time overflow is a separate
terminal typed system fault.

## Acquisition and lifetime

Fixed SDF `update_rate` uses the existing integer-nanosecond `SampleSchedule`.
Nonpositive rates sample each advancing step; finite positive rates must have a
representable period. There is no new rate service. Matched subscriber demand
selects acquisition. Scene manual acquisition and Server `InterestPolicy::Always`
retain their existing meanings. No subscriber means no acquisition or deadline
advancement; losing demand removes the stale sample. Paused zero-time ticks
acquire nothing. The canonical header carries acquisition time, `frame_id` and
attempt sequence; publication deduplicates epoch/time and fences old reset work.

Explicit topics use existing topic normalization and partition rules. The default
scoped topic ends in `/air_pressure`. Canonical `frame_id` wins over legacy
`gz_frame_id`, then the scoped entity name. Metadata/count/identity and every
reserved world/service/sensor/command key, including live and initial altimeter
and pressure metadata in both directions, must be checked before declarations.

An absent pressure family owns no sampling/reset callback, interest resource,
publication lease or endpoint. A world IO worker is reused, not added per sensor.
Removal retires its sample and last callbacks and drops CPU leases. Queued jobs
hold weak leases and cannot revive retired endpoints; an IO put already admitted
before CPU retirement may complete under the existing contract. Loaded reset
restores original public IDs/configuration/initial poses, clears samples and
cadence, reconstructs leases outside stepping and starts sequence zero in the
new epoch. This is loaded-world reconstruction, not native rewind parity.

## Runnable boundary and regression source

The retained fixture is
`crates/gz-sim-scene/tests/fixtures/m15_air_pressure.sdf`: centered inertia,
model Z=3 m and a rotated sensor at offset `[1,0,0.5]`, giving initial sensor
height 3.5 m. Use the existing Rapier world-loading CLI and subscribe to
`/m15/air_pressure` in the same partition. No viewer widget is added.

Source controls cover sea-level 101325 Pa and the upstream 3 m ideal reading
101288.9657925308 Pa, monotonic positive/negative heights, invalid domains,
actual offset rotation, cadence/demand/pause, omitted/default metadata, initial
refusals and later typed sample errors, removal/loaded reset, and Server canonical
wire/identity/epoch/lease/atomic endpoint contracts. Nondefault noise (including
nondefault parameters with type `none`), a second sensor, custom metadata,
visualization/metrics and other engines remain refused. Runtime pressure spawning
and rate/reference services are excluded; other sensor families stay unchanged.

`crates/gz-sim-scene/tests/native_air_pressure.cc` is the native reference source.
Its `model` mode compares five direct native absolute-height readings. `offset`
and `fall` modes use finite Physics+AirPressure worlds, exact name/type-presence
selection, fail-closed callback diagnostics, actual observed sensor world poses
and 11 canonical pressure samples over 100 advancing 1 ms steps. The caller
supplies the reviewed fixture; `fall` needs gravity `0 0 -9.81` while `offset`
uses zero gravity and the explicit initial linear/angular velocities. Live
library maps are captured before sensor/Server destruction. Actual qualified
compiler/plugin/library bindings and invocation are owned by Root's existing
reference recipe, not inferred from this source.

Pinned native `gz-sensors/test/integration/air_pressure.cc` covers creation,
setters, readings and topic handling. Ideal reading/default-topic assertions are
preserved by the narrow model/integration controls; Gaussian-noise and runtime
reference-setter behavior remain explicitly outside M15. Pinned
`gz-sim/test/integration/air_pressure_system.cc` uses no Physics plugin and
expects sea-level pressure despite its declared nonzero pose/reference. Its
zero-initialized `WorldPose` behavior is not evidence for physical-height parity.
The new native observer requires actual Physics-updated sensor world poses.
No DART/Rapier trajectory, performance or broad sensor/backend parity is claimed.
