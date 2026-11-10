<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Constant WORLD wind for AirSpeed

M41 extends the existing loaded AirSpeed sensor with a finite, constant WORLD
wind vector over accepted M39/M40 Main `470da6e6`. Its fresh scoped component
capture, independent native DATA and fixed-grammar literal DATA passed with
actual exits 0. The exact 91-reading scalar and 91-packet/header Rust controls
passed in the first executed feature headless batch. That batch failed elsewhere,
including the wind Server test's use of the narrow world-edit removal API for a
sensor-bearing model; it supplies no whole-run qualification. The source
correction uses the existing canonical removal service, with no production or
fixture change. Complete Rust/coverage/hosted/local/official qualification and
M41/M42 acceptance remain pending. M40 remains the installed Latest.

`examples/m41_air_speed_wind.sdf` retains the M26 free Rapier body, gravity zero,
1 ms step, sensor offset `(0, .5, .5)` and local yaw pi/4. Its WORLD wind is
`(1, -2, .5)`, topic `/m41/air_speed`, frame `rig/body/air_speed`, and rate 100 Hz.
The wind changes measured airflow, not body forces. A stationary sensor can
therefore report nonzero pressure. The existing native-free CLI can load the
example; two finite steps do not qualify its full sensor or canonical lifecycle.
There is no new viewer widget, launcher option or package refresh in this phase.

## Admission and immutable configuration

The complete AirSpeed preflight admits exactly one existing direct-link sensor
in a top-level Rapier model, with the existing metadata, rate, pose, atmosphere
and noise restrictions. Its boolean result allows the shared world guard to
admit finite WORLD wind only for that fully validated consumer. Nonzero wind
without that consumer remains refused. Nonfinite wind is always refused.
Model/link `enable_wind` remains unsupported: no aerodynamic force, WindEffects
system, runtime wind service, new physics backend or public wind setter exists.

The private AirSpeed component stores `wind_world: [f64; 3]`, copied from the
loaded world through SceneContext. InitialScene already clones and restores
that component. Healthy loaded reset reconstructs the original wind, schedule,
noise stream and public sensor ID; callers reacquire internal Bevy entities.
A faulted clock still refuses reset before mutation under the existing contract.
Sensor-bearing model templates remain unsupported; removal reconstructs the
loaded owner through healthy loaded reset rather than newly admitted cloning.
The Server lifecycle control removes the loaded owner through the existing
canonical `/world/default/remove/blocking` service: one typed Entity MODEL/id
query after actual Queryable matching, a true canonical Boolean response and
exactly one completed-removal counter increment. The paused clock/epoch stay
unchanged. The direct `remove_editable_model` API retains its narrower domain and
returns typed `WorldEditError::UnsupportedModel` for this sensor owner; the test
checks that refusal leaves entities, sample, counters, clock and pending steps
unchanged. All post-removal stream retirement, loaded-reset identity restoration
and owned-peer shutdown assertions remain required. No extra wind resource,
worker, polling callback or canonical field is added.

A wholly zero vector, including any signed-zero combination, becomes effective
`(+0,+0,+0)` and takes the exact old no-subtraction path. This preserves the old
M26/M28 numerical and noise behavior, including signed velocity zeros. Any
vector with a nonzero component keeps every requested component bit. Reference
records must distinguish requested and effective wind; legacy zero normalization
is a chosen compatibility policy, not a claim about native raw negative-zero
arithmetic.

## Frame and numerical policy

The existing point velocity is unchanged: link WORLD linear velocity plus
WORLD angular velocity crossed with the rotated local sensor offset. The
existing composite sensor-frame velocity is computed once by its existing
quaternion operation. For nonzero wind, only the new WORLD-wind term uses the
pinned native `Quaternion::RotateVectorReverse` arithmetic:

```
wind_sensor = Inverse(q) * (Quaternion(0, wind_world) * q)
air_velocity_sensor = existing_velocity_sensor - wind_sensor
```

Here `q` is the actual composite sensor WORLD rotation. Preserve Hamilton
scalar order and the inverse squared norm's `w,x,y,z` accumulation/divisions;
there is no normalization or optimized quaternion/vector replacement. Do not
rotate a pre-subtracted WORLD velocity. The unit-quaternion domain remains the
existing finite, squared-norm absolute error at most `1e-12`.

The pressure model remains unchanged: actual sensor WORLD Z, native float
atmosphere/density and signed sensor-X quadratic pressure, optional pressure
noise before float assignment, then final float multiplication by 100. Native
publishes a Pa-scale numeric value despite the canonical protobuf's hPa comment;
no new unit conversion is introduced. Temperature remains Kelvin. Default noise
and existing Gaussian/quantized behavior are unchanged; zero-stddev Gaussian
cases are deterministic, without an RNG-stream parity claim.

Invalid or overflowing ideal input yields the existing
`AirSpeedSampleError::Ideal(InvalidAirSpeedInput)` sample. The normal due
schedule advances before reading validation, the invalid ideal input draws no
noise, and Server publishes no stale valid frame. This is not a terminal clock
fault. Demand absence advances neither acquisition nor deadline; paused and
non-due updates preserve the established behavior. Reset/removal and weak IO
ownership keep their existing contracts.

## Source and reference boundary

The inspected original `gz-sim/src/systems/air_speed/AirSpeed.cc` at
`71c60db31fddd5c98d2aa689e9a2cccb3548a0a3` passes already sensor-relative
point velocity and the WORLD `WorldLinearVelocitySeed`. The original
`gz-sensors/src/AirSpeedSensor.cc` at
`8ddf4c17be8a434c51b51d5be7c866fb69aaa54f` rotates WORLD wind separately and
subtracts it before the existing pressure formula. Math's `Quaternion.hh`
implements reverse rotation as the two quaternion products above. These are
source traces; an original AirSpeed system DSO execution is not claimed and its
availability remains unknown.

The completed bounded reference uses the installed pinned SensorFactory AirSpeed
provider, direct WORLD sensor pose, already sensor-relative velocity, WORLD wind,
actual time/connection/update state and exact noise configuration. It applies no
M39 MODEL-to-WORLD conversion. Capture tree `3d3fe967` is the minimal accepted-parent
plus reference-helper tree, not the final M41/M42 runtime tree. The actual capture
contains 18 modes, 19 fresh configurations, 98 calls, 91 observed canonical messages
and seven suppressed calls. Reload creates a fresh default/no-noise instance and
then a fresh Gaussian mean 0.1/stddev 0 instance with separate topic/sequence state.
Native DATA `21d4718e` and literal DATA `bd0f5cb2` each passed with actual Root exit 0.
The immutable literal `m41_native_air_speed_wind.rs` is 125,341 bytes with SHA256
`2278d8043a0bf19edb47b7692d97abe4fa6474be7eaa004edd036af38edee668`.

The generating program is
`crates/gz-sim-server/tests/native_air_speed_wind_reference.cc` (SHA256 `dc4146e48b336e36d9a451193db7669b4a72982fb32d2a6a8424160d07a00565`).
[Checked-in generating provenance](../crates/gz-sim-scene/tests/fixtures/m41_native_air_speed_wind.provenance.json)
contains the verbatim actual compiler/link argv and cwd, all 18 helper run commands,
raw/maps/result hashes, component provider and critical mapped library/compiler
hashes, native/literal methods and actual-zero receipts. It records the component
pkg-config recipe and existing shared-library projection, without duplicating
all 349 historical SDK nodes or claiming every node was newly body-audited. No
SDK build, original AirSpeed system execution or native Physics run was added.
The original generated provenance and retained raw streams remain preserved.

`air_speed_wind.rs` adds the exact formerly refused M26 WORLD `(1,0,0)` positive
with real motion and healthy reset. It also checks head/tail/cross/cancelling
wind, tilt/WORLD-Z, the rotating sensor offset, exact unchanged physical motion,
legacy signed zeros, deterministic pressure noise ordering, demand/pause,
removal/reset/public-ID recovery and finite-overflow sample errors. Independent
point geometry keeps the established `1e-6` pressure comparison bound; this is
separate from exact native scalar/wire replay. Private controls retain
zero-path bits, mixed-zero configuration bits, source-level axis arithmetic and
no noise draw after invalid ideal input. The new private literal consumer uses
production `wind_to_sensor`, `relative_air_velocity`, ideal pressure and
`PressureNoise` paths. It compares all nonzero-wind rotation/subtraction bits and
all 91 actual pressure/temperature bit pairs; whole-zero configurations retain
the exact old velocity bypass without a false native-zero intermediate equality.
All 98 call/configuration/activity/deadline records are consumed as original
Sensors component information, not a mapping of Sensor Active to Scene pause or
demand. Exact canonical bytes are a separate Server consumer obligation. All
old M26/M28 fixtures and test bodies remain unchanged.

The obsolete finite-wind negative is replaced only under Root's separate exact
lease. Malformed/nonfinite wind, unused-world wind, physical wind modes and all
neighboring topology/backend/noise/atmosphere refusals remain meaningful negative
controls. The recorded native/literal and individual Rust replay results remain
scoped; the reviewed correction and complete unchanged normal/strict/hosted/local/
official gates, including the whole canonical Server/C++ lifecycle, remain
necessary before acceptance. No cross-engine trajectory, original system
execution, RNG, performance or legal parity is claimed.
