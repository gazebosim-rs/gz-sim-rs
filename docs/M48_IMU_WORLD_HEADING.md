<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# IMU orientation references with world heading

M48 extends the existing centered IMU reference selection to consume an immutable
world heading. Development starts at locally qualified M47 `f27a37fd`, not
accepted Main. M1–M42 remain accepted at `73e03b9f`, and M40 remains installed
Latest. Implementation and regression Source passed independent review. Fresh
original-component capture and independent native/literal DATA passed; Rust,
full normal/coverage, hosted qualification and acceptance remain pending.

The existing ENU, NED, NWU and effective CUSTOM with `parent_frame="world"`
references can use a typed spherical heading. Omitted localization remains
effective CUSTOM; no XML-presence flag or parser extension is added. The new
consuming domain requires Earth WGS84 and ENU, numerical-zero finite latitude,
longitude and elevation, and heading radians in `[-pi, pi]` that are normal or
zero. Signed zeros retain their bits. Values are not wrapped or clamped.
Subnormal, nonfinite and out-of-range headings are refused. This is heading-only
metadata consumption, with no geodesy, gravity conversion or world-coordinate
transformation.

Legacy CUSTOM with an empty parent and whole-zero custom RPY still returns the
exact original initial sensor quaternion before checking spherical metadata.
Its earlier metadata admission is preserved. Every no-spherical path selects its
old private Frame variant and performs the same Euler and Hamilton operations.
Named custom-RPY, non-world custom parents, unknown localization and translated
sensor positions remain outside the loader domain. The existing M45 finite,
nonsubnormal CUSTOM/world RPY domain is unchanged.

## Native arithmetic and immutable ownership

Pinned Sim `71c60db31fddd5c98d2aa689e9a2cccb3548a0a3`,
`src/systems/imu/Imu.cc`, seeds the initial sensor reference and then supplies
`Quaternion(0, 0, heading)` in ENU when a reference element exists. Pinned
Sensors `ImuSensor.cc` left-multiplies that quaternion by the selected named
ENU-source table entry or the CUSTOM/world RPY quaternion. The implementation
preserves native Euler scalar order and normalization, Hamilton product order
and quaternion sign. It does not normalize after that product. The existing
scalar IMU computes reference inverse times current sensor world orientation.
Motion setters stay in actual sensor axes; heading does not rotate their inputs.

The private `validate(config, Option<&SphericalCoordinates>, entity)` handoff
returns one sealed validated Frame. Four new variants store exact heading bits
and, for CUSTOM/world, the prior three custom-angle bit fields. The loader
owns one load-time snapshot of typed spherical metadata and borrows it only for
validation; the snapshot is not retained by the running IMU. No public heading
setter, resource, generic frame resolver or new wire field is introduced. InitialScene
already clones the constructed IMU reference for healthy loaded reconstruction.
Reset restores public IDs while Bevy handles may change. A latched terminal
fault still refuses reset before observers or scene mutation. Bare core reset
is not a promise to rebuild a loaded IMU.

## Example and controls

`examples/m48_imu_world_heading.sdf` retains M45's free mass-2 body with diagonal
inertia 1, body pose `(2,-1,3)` / RPY `(0.2,0.3,0.6)`, centered sensor rotation
`(0.1,-0.2,0.3)`, zero gravity, 1 ms step and 100 Hz acquisitions. It adds Earth
WGS84/ENU heading +45 degrees and selects NED. The topic is `/m48/imu` and frame
is `rig/body/imu`. The existing finite CLI accepts the ordinary development
example; it does not supply native component or canonical-message evidence.

New Scene Source controls cover all four consuming reference modes at positive,
negative, ±zero and ±180-degree boundaries. Real queued force and torque produce
physical acceleration and rotation; no velocity replacement supplies a false
acceleration premise. Independent axis-angle geometry checks orientation with
unchanged componentwise, sign-sensitive `<1e-9` bounds. Paired same-seed scenes
keep identical body pose, sensor-axis gyro/specific force, noise draws and
covariance across headings. Disabled orientation remains a publication choice;
the separate Scene reading is still computed.

The exact old named heading-zero spherical inputs and explicit/omitted
CUSTOM/world mixed-RPY heading-zero inputs now have stronger load, real-motion,
healthy-reset/public-ID and replay positives in their original tests. Their
remaining unrelated assertions are preserved, and nonzero latitude replaces
the old unsupported spherical premise. Private tests cover DTO-only nonfinite
and subnormal inputs, exact payload bits and legacy bypass. New public controls
cover rate, no demand, paused equal-time ticks, held samples, removal,
reconstruction and terminal-fault nonmutation.

The third normal attempt stopped after 4.166 seconds at the required formatting
check, before compilation or tests. Root had omitted formatting the completed
Server assertion correction. One affected Server formatter pass then completed
in 0.685 seconds, changing only that file's layout. Assertion values, inputs,
production code and fixtures remain unchanged. Preserve the failed `96426233`
source and its original method/log/result; all three actual failures stay failed.
The complete normal hook, strict coverage and independent local DATA still need
a successful actual result. No native reference or DATA reader is repeated.

The second complete normal attempt failed after 1600.379 seconds with two
headless test failures, after both Clippy runs and all 403 aggregate Python
cases passed. One new Scene test compared a held 100 Hz reading to the current
moving pose; its correction preserves all 21 strict geometry checks using the
actual acquisition pose, with exact held-sample and 1/10/20 ms cadence assertions.
One Server test expected a loader refusal for decimal `1e-310`, which the pinned
parser rejects first. That original input now requires exact parser InvalidValue
and conversion diagnostics; five loader-refusal cases remain, and normal degree
input `1e-306` adds the actual subnormal-radian loader boundary. Production code,
numerical bounds and native/literal fixtures are unchanged. Preserve the failed
`e6eeb0a4` source, all logs and partial evidence. No successful full normal,
renderer/strict coverage or local DATA result is claimed. No reference, reader
or formatter is repeated; a separately reviewed full retry remains required.

The first complete M48 local normal hook failed after 84.164 seconds at
required Clippy compilation, before tests or coverage. Rust E0502 identified
an immutable borrow of spherical metadata extending across later model mutation.
The correction stores one owned metadata snapshot during loading and borrows it
only for validation. Values, reference arithmetic, all assertions and every
native/literal fixture are unchanged. Original failed tree `47840d61`, logs and
packet remain retained. No native capture, data reader or formatter is repeated;
a separately reviewed complete normal retry remains required.

## Required qualification

The fresh bounded original Sensors/Math capture completed in 32.537 seconds
with all 89 stages successful. Independent retained native DATA checked all
44 configurations, 264 calls and 264 packets: 240 orientation-present, 24 absent,
no suppression, all 349 current SDK nodes and 51 actual mapped providers.
Matched baselines preserve motion channels and covariance; legacy no-reference
packets remain exact. Independent fixed-grammar DATA then confirmed the generated
Rust literals against those retained observations. No component or product was
replayed for either reader.

The observed fixture and exact generator provenance are retained under
`crates/gz-sim-scene/tests/fixtures/m48_native_imu_world_heading.*`.
[The generating recipe](../planning/reference/m48-imu-world-heading.json)
records the actual C++ source, compiler/run commands and library hashes, keeping
accepted capture parent `73e03b9f` distinct from unaccepted runtime parent
`f27a37fd`. New private consumers exercise strict sign-sensitive `<1e-9` numeric
comparisons and a separate exact 264-packet/header encoder replay. These Rust
checks have not run at this Source checkpoint. No M43/M45 fixture was modified
or recaptured; no native Physics or SDK build occurred.

Complete independent Source/integration review precedes one consolidated full
normal hook with strict workspace/per-crate/actual-parent coverage and retained
independent local DATA. Existing finite examples remain a prefix. Any eventual
local qualification is provisional; actual accepted predecessor, exact hosted
success, independent official DATA and ordinary Main/readbacks still precede
publication or acceptance. No package/browser refresh, CI workaround, native
RNG equivalence, performance or phase acceptance claim transfers from M47.
