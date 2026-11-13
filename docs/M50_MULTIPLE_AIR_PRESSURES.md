<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Independent air-pressure sensors in one loaded world

M50 extends the existing loaded Rapier pressure domain to zero through eight
sensors. Eight is a delivery resource bound, not an upstream limit. It uses the
exact locally qualified M49 parent `4ccd4f06`; M50 has no measured qualification
or release credit yet. See [the contract](M50_DELIVERY.md).

Each sensor keeps its own sample schedule, noise stream, demand, metadata and
publication lifetime. Effective endpoints must be distinct after normalization,
partitioning and remapping. Count, original identity/name/kind/owner, metadata,
occupied keys and prepared reset bindings are validated for the whole set before
publisher lookup or declaration. A ninth sensor, alias or late invalid binding
refuses atomically. Existing absent and single-sensor paths remain in scope.

Pressure uses absolute sensor WORLD-Z with reference altitude fixed at zero;
it does not subtract an initial sensor height. Sensor orientation does not turn
it into a local-axis measurement. Existing Gaussian/quantized pressure noise,
default zero elapsed noise application and the pinned `sqrt(stddev)` variance
convention remain unchanged. Rust retains its typed invalid/nonpositive sample
behavior, which is not a native rejection oracle. No atmosphere/reference setter,
geodesy, native/Rust RNG parity or individual-native-demand equivalence is added.

`examples/m50_multiple_air_pressures.sdf` keeps the mass-two free body at world
Z=3 with zero gravity. The front sensor at local `(1,0,.5)` publishes ideal
pressure at 100 Hz on `/m50/front_air_pressure`. The rear at `(-1,0,-.5)` publishes
at 50 Hz on `/m50/rear_air_pressure` with deterministic mean 25 Pa and zero
standard deviation. Frames are explicit and separate. The finite development
command is appended after the unchanged 23-example prefix:

```sh
cargo run --locked -p gz-sim-cli -- examples/m50_multiple_air_pressures.sdf --physics-engine rapier --iterations 2 --step-ns 1000000
```

This command has not been run for M50. It is a finite load/step obligation;
canonical acquisition, physical motion, noise/demand/reset/removal and installation
are separate proofs. No package or browser refresh is selected; M40 stays Latest.

The additive `initial_air_pressure_publications_with_identity` query borrows
original public IDs, kinds, world-scoped names and metadata from the immutable
loader blueprint, including removed original sensors. The old metadata-only
query remains available. The full original name, publication entity name and
frame ID are distinct. An actual world prefix can contain `::`; no first-segment
assumption is valid. The query performs no IO or mutation.

Healthy reset restores original identities, rates and streams. One-owner removal
retires only its leases; a surviving owner continues, and last-owner removal
releases the family. Terminal reset refusal preserves state. Pressure outputs
retain cached matching, weak leases, bounded queues and existing session-failure
semantics. An individually inactive Rust sensor consumes no draw or deadline
advance when a peer has demand.

Original AirPressure has family-wide due/connected wakeup and can advance an
individually disconnected due owner after another wakes the family. Its early
return can defer removal with no connected due owner. The new original-System
reference therefore uses connected intervals and an active due peer at removal,
with the live marker retained through System updates before physical removal.
Fresh native and fixed-literal DATA must pass before observed fixtures/consumers.
The pressure Sensor library needs its own exact input role outside M49's 349-node
baseline; provider identity/alias/dependency inspection and reviewed input closure
precede any capture release. Availability or Source readiness is not qualification.

Complete Source, regression and documentation review precedes one consolidated
formatter and unchanged normal/strict coverage pass. Independent retained local
DATA follows actual success. The strengthened former second-pressure refusal
keeps a separate existing-control obligation in both profiles, with its exact
approved body projection. Accepted predecessors, hosted success, official DATA
and Main readbacks still precede publication or phase acceptance.

The original-System capture and independent retained native/literal audits have
passed. The immutable fixture contains 441 sensor WORLD-pose inputs and 42
received packets, with reference altitude zero. `None` means no matching captured
packet, not an observation of internal update/draw/deadline state. The native
family-wide wakeup/removal policy and Rust per-sensor demand remain distinct.
See `planning/reference/m50-multiple-air-pressures.json` for exact provenance.
Observed numeric and exact-wire controls and complete Rust qualification remain
pending; this reference evidence does not change release acceptance or Latest.

The first full normal attempt stopped at an existing wrench test's generic
five-second admission deadline. Its actual failed wait and cause are unknown.
The bounded test-only phase diagnostics and three-publisher post-reset readiness
correction retain every outcome assertion and deadline. A fresh full normal
result is required; no native/literal result is repeated or relabelled.
