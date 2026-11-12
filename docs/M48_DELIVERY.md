<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M48: IMU world-heading references

One bounded increment, budgeted at most five working days including independent
review and mandatory checks. Add world heading to the existing centered IMU's
ENU, NED, NWU and CUSTOM/world orientation references. No installed preview,
renderer, backend, dependency, schema or transport change is selected.

The complete contract and planning/tracker selection passed independent Source
review with no findings. The complete Scene and Server/reference Source batches
passed independent review. Fresh original Sensors/Math capture completed once
in 32.537 seconds with all 89 stages successful. Independent native and fixed-grammar
literal DATA passed for 44 configurations and 264 packets, including 240 with
orientation and 24 without. Both audited-fixture consumers are integrated for
final Source review. No Rust, lint, full-normal or coverage pass is claimed at
this Source checkpoint.

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

## Basis and release boundary

Development starts at clean local M47 commit
`f27a37fd94d8218a4c3c488a4e550e06f521c9c8`, tree
`67a967ba694e8a1efd3511214e65872ff6142698`. Its full provisional normal hook
passed in 2062.426 seconds and independent retained DATA passed: 1757 headless
and 1047 renderer profile passes with overlap, 403 aggregate Python tests and
21 finite examples. Every strict coverage gate passed, including changed Scene
production lines 152/155. Native component DATA passed separately. Preserve the
original failed attempts and all frozen M43–M47 trees and evidence.

M1–M42 remain accepted at Main `73e03b9f`; M40 remains installed Latest.
M43–M47 are not accepted. M48 development and provisional local qualification
may proceed under the existing D31/D32 timing policy, but publication and
acceptance require the actually accepted predecessor, exact hosted success,
independent official DATA and ordinary Main/readbacks. A changed predecessor
requires integration review and affected checks; no evidence transfers by label.
No CI retry, snapshot probe, alternate pin/mirror/workflow or eviction is released.

## Behavior and admission

Use typed SDFormat spherical `heading_radians`; do not inspect XML presence or
add parser state. The proposed closed new domain is Earth WGS84 / ENU metadata,
finite heading in [-pi, pi], zero or normal, and finite numerical-zero latitude,
longitude and elevation. This increment implements no geodesy, navigation,
gravity or physical coordinate conversion. Preserve the legacy initial-reference
branch's earlier metadata admission; these new restrictions apply only to the
newly admitted consuming references. The heading bound and private handoff were
included in the completed independent contract review.

Pinned Sim `src/systems/imu/Imu.cc` first seeds the initial sensor reference,
then supplies Quaternion(0, 0, heading) with ENU basis when a reference element
exists. Sensors `ImuSensor.cc` left-multiplies that quaternion by the existing
named table entry or CUSTOM/world RPY quaternion. Preserve native Euler scalar
order, normalization, Hamilton product order and quaternion sign. The unchanged
IMU core computes reference inverse times current sensor world orientation.

Legacy effective CUSTOM with empty parent and zero angles keeps its exact initial
sensor reference, including its existing spherical metadata behavior. Every old
no-spherical reference path keeps its arithmetic and bits. CUSTOM/world heading
changes orientation only: gyro, specific force, noise, covariance, cadence,
physical pose and all canonical field contracts stay unchanged. Other parents,
unknown localizations, translated sensors and invalid values stay typed refusals.
Healthy loaded reset reconstructs the configured reference through existing
scene state; terminal-fault refusal remains nonmutating. No public runtime setter
or generic frame resolver is added.

Use the private handoff `validate(&sdformat::Imu,
Option<&sdformat::SphericalCoordinates>, &str) -> Result<Frame, LoadError>`.
Root replaces the existing private SceneContext presence Boolean with an owned
load-time typed metadata snapshot. The validator borrows that snapshot only for
construction, so no original SDF-world borrow survives into model mutation. Preserve the five old sealed Frame variants and add four
heading-bearing equivalents containing exact heading bits and, for CUSTOM/world,
the existing RPY bits. No public type changes. No-spherical references select the
old variants; explicit signed-zero heading selects the native heading path.
Return the legacy CUSTOM branch before the new metadata validation. Do not
normalize the quaternion after the Hamilton product or canonicalize its sign.

## Ownership and evidence

Root owns the shared Scene context/loader seam, three old private native-call
argument migrations, registration, finite CLI inventory, qualification,
planning/tracker and serial heavy execution. Light owns Scene imu_reference.rs,
its private tests, new imu_world_heading.rs public tests, the two old public
reference tests' exact spherical-refusal dispositions, the new example and guide.
Messages owns new Server imu_world_heading.rs, original-component
native_imu_world_heading.cc and canonical native_imu_world_heading_peer.cc,
plus the held capture procedure. Observed fixture consumers follow actual native
DATA; no placeholders. SDF independently reviews the complete implementation,
regression source, docs and native procedures. No checks run after individual edits.

Required controls cover positive/negative/zero/boundary heading for named and
CUSTOM/world references; signed zero and exact old paths; real force/torque motion;
actual pose and non-orientation output invariance; noise/rate/demand/pause/equal
time; healthy reset/removal/terminal fault; invalid configuration and rollback;
and complete canonical IMU packets through the maintained upstream Zenoh peer.
Newly admitted old spherical-refusal inputs become stronger motion/reset positives,
with neighboring refusals retained. Do not delete or weaken old numeric assertions.

A fresh bounded component capture must use the existing original Sensors/Math
implementation with actual setters, configuration, provider identity and literal
canonical bytes. Reuse unchanged helper/process primitives and the reviewed current
349-node SDK manifest, with fresh checks; no SDK or native Physics build. Independent
native/literal DATA must precede committed observed-value fixtures. No generated
expected value, sign-insensitive comparison or Rust-derived oracle substitutes
for observation. Native component bounds stay below 1e-9; wire replay stays exact.

The bounded reference table has 44 configurations, each with six forced rate-zero
updates using the existing M45 WORLD/body/sensor-motion rows. For each of the four
consuming references: headings +0, -0, +45, -60, +180 and -180 degrees (24 cases),
one disabled-orientation +45 case (4), one unchanged mean-only/stddev-zero noise
case at +45 (4), and ideal plus matching mean-only no-spherical baselines (8).
Add omitted-localization CUSTOM/world at +45 and -180 (2), and the legacy
no-reference-element ideal pair with heading +45 versus no spherical metadata (2).
Expected shape is 264 actual packets, 240 with orientation and 24 without, no
suppression. These are required observations, not claimed results. Reset both
mutable motion setters before every component update. Record actual degree to
radian conversion and quaternion input bits; do not rotate motion inputs by heading.

Finish the entire implementation, meaningful regressions, one finite development
example and guides; batch independent review findings; then one consolidated full
normal hook, strict workspace/per-crate/actual-parent line and region coverage and
independent retained DATA. Preserve the old 21-example prefix. Source readiness
and local passes do not establish hosted, package or phase acceptance.
