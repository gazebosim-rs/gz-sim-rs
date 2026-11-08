<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M17 ideal constant-field magnetometer

This increment admits one loaded Rapier magnetometer. It expresses the finite,
spatially uniform SDF world magnetic vector in the resolved sensor frame, in
Tesla: `field_sensor = inverse(sensor_world_rotation) * field_world`. Both link
and sensor rotation contribute. Position, initial orientation and initial height
are not subtracted. Explicit nondefault and zero world vectors are supported.
The implicit SDF vector is `[5.5645e-6, 22.8758e-6, -42.3884e-6]` Tesla.

The pure `gz_sensors::magnetometer` model accepts `MagnetometerInput` containing
`world_orientation` in `[x,y,z,w]` order and `field_world`. It returns
`MagnetometerReading.field_tesla`. Inputs and derived values must be finite; the
quaternion squared norm must differ from one by at most `1e-12`. Invalid inputs
or rotation overflow return `InvalidMagnetometerInput`, without normalization,
clamping or fabricated readings.

Scene shares the complete sensor admission routine between pre-Simulation
preflight and spawning. Only one fully admitted typed consumer authorizes a
nondefault world field; removing the sensor declaration from such a world does
not make its otherwise inert field supported. Metadata, typed configuration,
engine, cadence, resolved pose and field-domain validation precede this
permission. Spherical coordinates, noise (including nondefault values in a
`none` record), visualization, metrics, sensor plugins and extensions are
refused. Unconfigured or mismatched sensor types, a second magnetometer,
unrepresentable rates and unsupported engines are refused atomically. Existing
world/model/backend capability restrictions still apply.

`MagnetometerSample` records acquisition time, reset epoch and a typed reading
result. A later invalid measurement remains an observable error sample and is
suppressed at the wire boundary; only a sampling-deadline failure faults stepping.
`MagnetometerPublication` supplies scoped entity identity, frame and requested
topic. Canonical frame metadata wins over legacy `gz_frame_id`, then the scoped
entity name is used. An omitted topic becomes
`/world/<world>/model/<model>/link/<link>/sensor/<sensor>/magnetometer`.
Server publishes canonical `gz.msgs.Magnetometer.field_tesla` with acquisition
stamp, per-stream sequence and frame metadata. Endpoint ownership and cross-family
collisions are validated before the first declaration.

A fixed positive SDF update rate uses the existing nanosecond `SampleSchedule`.
Zero or negative rates acquire on each advancing step. Pause does not acquire.
Explicit empty demand owns no sample and does not advance the deadline; reconnect
preserves its phase. Clearing transport demand selection restores local/manual
acquisition. Loaded reset clears samples, restores the constant field and initial
poses, resets schedule and wire sequence, and fences old queued work by epoch and
weak ownership. Removing the last sensor retires both sampling/reset callbacks
and its interest resource. An absent family owns none of those resources,
callbacks or endpoints. Runtime sensor spawning and rate/field services are not
part of M17.

## Source references and regressions

The measurement convention derives from OSRF's `gz-sensors` commit
`8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`,
`src/MagnetometerSensor.cc` and `test/integration/magnetometer.cc`
(`SensorReadings`, creation and topic controls). Its native class applies noise
after inverse rotation; this increment admits only ideal defaults. The new model
and reference controls preserve the original 2019 attribution.

Pinned `gz-sim` commit `71c60db31fddd5c98d2aa689e9a2cccb3548a0a3`,
`src/systems/magnetometer/Magnetometer.cc`, initializes the sensor with the world
field, updates sensor world pose, and retains that field when spherical-coordinate
lookup is unavailable. That path emits a geographic warning. The separate
geographic branch can replace the field using its Gauss/NED options; no geographic
lookup, that branch's units, or warning-free geographic operation is claimed.

Public model controls cover asymmetric/default/zero fields, independent scalar
rotation, quaternion sign equivalence, nonfinite/nonunit inputs and finite-input
output overflow. Private Scene controls exercise actual Rapier rotated-link plus
rotated/offset-sensor poses, translation invariance, angular motion, demand,
100 Hz cadence, pause, removal/loaded reset, absent-family ownership, metadata
fallback and complete preflight refusals. Server's separate wire/lease controls
cover canonical fields, collisions, invalid samples and stale work. The fixture
`tests/fixtures/m17_magnetometer.sdf` uses centered inertia, no ground plane, zero
gravity and asymmetric rotations, keeping the existing Rapier domain unchanged.

## Native reference procedure

`crates/gz-sim-scene/tests/native_magnetometer.cc` is reference source, with modes
`model`, `rotate` and `translate`. `model` uses the pinned public sensor factory
and actual canonical messages for six asymmetric/zero/default/translation cases.
The other modes read the fixture from standard input, start paused, resume via
canonical world control and observe 100 actual native poses during 100 ms. Each
requires exactly 11 canonical packets at 1, 10, ... 100 ms, exact sequences/frame,
finite observations and independent scalar inverse-rotation agreement within
`1e-15` Tesla. Initial composed orientation, nonzero sensor offset, actual link
translation, angular field change and translation-only field invariance are
explicit witnesses. These are same-native pose/output comparisons, without a
cross-engine trajectory or performance claim.

Use the already qualified Clang18 reference build at
`gz-sim-rs/artifacts/reference/gz-sim-clang`, existing `/home/nkoenig/local` SDK and
existing Zenoh/DART vendor closure. Root adapts the maintained M15 native recipe
(`gz-sim-rs/artifacts/m15-m16-delivery/native-reference.py`) to the exact configured
`src/systems/magnetometer/CMakeFiles/gz-sim-magnetometer-system.dir/flags.make`
and `link.txt`, with fresh owned helper/plugin outputs only. Do not reconfigure,
rebuild or modify the SDK. Record actual loaded providers and live maps while
the sensor/Server/plugin owners still exist. Callback errors retain field/entity/
type/timestamp diagnostics and fail after the finite run; no uncaught worker
exceptions or default-valued component lookup is used.

Implementation and regression source require independent review before any
checks. Native runs, full normal-hook/strict coverage, exact hosted CI and retained
DATA audits remain mandatory before qualified Main acceptance. M15/M16 evidence
is not an execution result for this new source. No magnetometer UI, geographic
model, noise, backend expansion or benchmark is added.
