## Current M51 retained native evidence

One fresh original AirSpeed System pair/eight/active-peer-removal capture returned
actual Root zero, followed by independent retained native and fixed-literal DATA.
The immutable fixture preserves twelve configurations, all 331 explicit sensor
WORLD-pose, POINT-WORLD velocity and WORLD-wind inputs, 55 actual canonical
AirSpeed packets and 276 inputs without a matching packet. The 79 System calls
are summary and pinned-method attestations; missing packets reveal no internal
update, noise-draw or deadline state. Signed differential pressure is valid.

See `planning/reference/m51-multiple-air-speeds.json` for the actual procedures,
commands and provenance. No SDK/provider was rebuilt or capture repeated. The
new Scene exact-scalar/independent-geometry and Server exact-wire consumers are
being reviewed before one consolidated formatter and full normal/strict coverage
pass. No M51 Rust, Python, coverage, hosted or release qualification is claimed.
M1–M42 remain accepted; M40 remains installed Latest. All accepted-predecessor,
hosted, official and Main gates remain mandatory. Earlier checkpoints below are
historical records.

<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Independent air-speed sensors in one loaded world

M51 extends the existing loaded Rapier air-speed domain to zero through eight
sensors. Eight is a delivery resource bound, not an upstream limit. This Source
uses exact locally qualified M50 `623faca6`; M51 has no measured qualification or
release credit yet. Read [the contract](M51_DELIVERY.md).

Each sensor retains its own rate, noise stream, demand, metadata and publication
lifetime. Topics must remain distinct after normalization, partitioning and
remapping. Whole-set validation checks original identity/name/kind/owner, immutable
metadata, occupied endpoints and prepared reset bindings before publisher lookup
or declaration. A ninth sensor, alias or late invalid owner refuses atomically.
An absent family remains inactive. Unused nonzero wind without a validated
AirSpeed consumer remains rejected.

The sample uses actual sensor-point velocity, including a moving link's angular
contribution at its offset, and the existing sensor rotation and WORLD-wind
conversion. Constant wind changes the measured relative flow; this preview does
not add physical wind forces. Signed negative differential pressure is valid for
reverse flow. Existing temperature, noise, rounding and atmospheric behavior is
unchanged; there is no pressure variance or new atmosphere/noise configuration.
Default and zero-wind behavior stay exact. Same-seed owner comparisons preserve
that owner's original public ID when another sensor is appended.

The development scene has two sensors on one free body at WORLD height three,
with gravity zero and finite constant WORLD wind `(1, -2, 0.5)`. Front and rear
have different offsets/orientations and rates of 100 and 50 Hz. The front sensor
is ideal. The rear uses existing Gaussian pressure noise with mean `0.25` and
standard deviation zero, a deterministic nondefault case. This uses the existing
noise configuration's pre-scaling units and rounding; it introduces no new units
or statistical convention. Each has its own explicit topic and frame ID.

The finite copied-example qualifier runs the maintained CLI with the exact form:

```sh
cargo run --locked -p gz-sim-cli -- examples/m51_multiple_air_speeds.sdf --physics-engine rapier --iterations 2 --step-ns 1000000
```

This command is documented Source and has not yet run for M51. It verifies the
copied scene's complete finite body output, not acquisition, live publication or
release acceptance. The full qualifier retains all earlier examples and adds
this as the twenty-fifth; no accepted manual/browser demonstration is replayed.
No packaged preview is refreshed, and M40 remains installed Latest.

Functional qualification separately requires real force/torque motion, fresh
acquisition versus held samples, independent schedules/noise/demand, canonical
C++ packets, pause/reset, owner removal and atomic refusal. Existing component
scalar/wind/noise evidence is retained within its actual scope. A new original
AirSpeed System multiplicity reference, independent native and fixed-literal DATA,
observed numeric/wire consumers and full unchanged normal/strict coverage/local
DATA remain pending. Native family-wide wakeup/removal differs from Rust's
per-owner demand: no individual-disconnect, RNG-stream, all-disconnected removal
or native Physics equivalence is claimed. Publication and acceptance still need
the actual accepted predecessor, exact hosted success, independent official DATA
and ordinary Main/readbacks. Source readiness alone is not release acceptance.
