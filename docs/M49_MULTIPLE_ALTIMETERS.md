<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Independent altimeters in one loaded world

M49 is Source development over locally qualified M48 `a204d94b`; it has no
verification or release credit yet. See [the delivery contract](M49_DELIVERY.md)
for the closed domain, ownership and required evidence. M40 remains installed
Latest. This phase changes neither the installer nor the browser.

The selected domain is zero through eight otherwise supported loaded Rapier
altimeters, with different effective endpoints after topic normalization,
partitioning and remapping. Eight is an explicit resource bound for this
increment. A ninth sensor, aliases, duplicate identities, wrong ownership,
reserved-key collisions and invalid reset bindings must fail before publisher
lookup or declaration. Shared altimeter topics remain unsupported.

`examples/m49_multiple_altimeters.sdf` places two sensors on the existing
mass-two free body at world Z=3. The front sensor is at local `(1,0,.5)`,
uses `/m49/front_altimeter` at 100 Hz and is noiseless. The rear is at
`(-1,0,-.5)`, uses `/m49/rear_altimeter` at 50 Hz and requests deterministic
position mean .05 with zero standard deviation. Each keeps its own initial
world-Z reference and acquisition state. Sensor orientation does not turn
altitude into a local-axis measurement; the existing world-vertical and
offset-point velocity arithmetic remains unchanged.

The finite development command will be appended to the existing 22-example
prefix after full Source integration:

```sh
cargo run --locked -p gz-sim-cli -- examples/m49_multiple_altimeters.sdf --physics-engine rapier --iterations 2 --step-ns 1000000
```

It is a finite load/step control, not proof of canonical acquisition, installation
or browser behavior. No command in this guide has been run for M49. Ordinary
server endpoint options and upstream Zenoh remain unchanged. Actual per-topic
messages, rates, noise sequences, motion, reset and retirement need the required
functional and canonical controls.

Each sensor retains its own seeded stream and deadline. An individually
disconnected sensor loses its sample and consumes no draws or deadline advances;
a connected peer cannot wake it. This preserves Rust's current policy. Original
Sim uses family-wide connection wakeup, so it can advance an individually
disconnected due sensor while a peer has demand. Original removal also occurs
after that family wakeup return. Native comparisons therefore cover all-connected
intervals and removal while another connected peer is active, with no claim of
individual-disconnect or immediate all-disconnected removal equivalence.

Healthy loaded reset restores original public identities, initial references,
schedules and seeds, with full validation of the entire reset set before IO.
Internal Bevy entities can change and must be reacquired. Removing one loaded
model retires its own streams while an unaffected owner continues. Existing
terminal reset refusal and weak lease/queue/session ownership remain intact;
this phase adds no runtime setter, sensor-bearing model cloning or generic
registry.

The additive read-only `initial_altimeter_publications_with_identity` query
borrows initial IDs, kinds, world-scoped entity names and publication metadata
from the immutable loader blueprint, including removed original entities. The
old metadata-only query remains available. The original name includes the world
prefix; `AltimeterPublication.entity_name` omits it and `frame_id` is a separate
publication field. These values must not be compared as if they were identical.
The query performs no IO and changes no simulation state.

Fresh original-system multiplicity capture and independent native/literal DATA
have passed: twelve configurations, 441 input rows and 42 observed packets.
The exact audited fixture and provenance are retained under Scene `tests/fixtures`,
with the capture and actual Root receipts pinned in
`planning/reference/m49-multiple-altimeters.json`. Numeric model and exact-wire
consumers are being completed from these observations; complete Source review
and one consolidated normal/strict coverage pass remain required. Reuse unchanged scalar/noise evidence; do not infer native/Rust RNG
equivalence. Actual predecessor acceptance, exact hosted success, independent
official DATA and normal Main/readbacks still precede publication or acceptance.
