<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Static-vector PosePublisher

The built-in `gz::sim::systems::PosePublisher` accepts `static_publisher=true`
for one loaded flat top-level Rapier model. All direct links must be selected;
model pose is optional, and the total selection is bounded to 128 records.
The model must have a valid direct canonical link and joints must name direct
links without scopes (`world` is allowed as a parent). Existing joint/backend
restrictions apply. Ordinary M27 publication and defaults are unchanged.

Loaded Rapier Scene joint admission remains revolute-only; a physical fixed-joint
world is still refused. The Rust fixed-classification regression exercises only
publication metadata construction, without stepping an inadmissible world.
Actual all-static acquisition and removal use an admitted jointless model. The
native fixed-joint mode remains separate evidence of the native classification;
it does not qualify physical fixed-joint support in the Rust backend.

`examples/m31_pose_static_publisher.sdf` publishes the noncanonical moving tip
at `/m31/pose` (100 Hz), and the model plus canonical base at
`/m31/pose_static` (25 Hz). Native classification marks endpoints of non-fixed
joints dynamic except the canonical link. Fixed-only and jointless links, and
selected model poses, belong to the static channel. Static classification does
not imply an immobile world pose: each due update reads current local transforms.
No scene entity or physics feature is removed to satisfy this selection.

The static topic appends `_static` to the raw configured/default topic before
each endpoint is mapped independently. Both endpoints remain declared even if
a group is empty, but no empty Pose_V is sent. Both keys participate in live and
initial/reset reservations, including aliases and cross-family collisions.
The existing Pose IO worker, bounded queues, weak leases, epochs and counters
serve both channels; this adds no transport family or background worker.

`static_update_frequency` is allowed only with split mode and otherwise inherits
`update_frequency`. Both must be finite. Nonpositive values are unthrottled;
positive values use native `(1/f)*1e9` truncation, checked below signed-nanosecond
overflow, including a possible zero-ns period. Each channel begins with last=0,
keeps its own clock/revision, and skips only positive differences below its
period. Paused ticks suppress both channels. Equal-time unpaused and backward
updates remain eligible. No subscriber-interest gate or latching is added.
At coincident deadlines static is called/enqueued first; independent transport
callbacks have no cross-topic arrival-order guarantee. A static-only tick cannot
replay a held dynamic sample. A due invalid vector clears both stale samples and
reports a terminal fault before any partial due update is installed.

Each Pose has its acquisition stamp, ordered `frame_id` and `child_frame_id`
header entries, local parent-relative transform, child-frame name and ID zero.
The outer Pose_V header is absent; neither channel invents sequence metadata.
Rust orders frames lexically for deterministic output. Native uses an unordered
map, so interoperability comparisons key records by child-frame name.

Loaded reset restores fresh entity handles, both original zero clocks and both
endpoint leases atomically. Direct core reset retains both live last clocks and
clears samples; it is a different convenience boundary. Native world/time-only
reset reloads this plugin. Its control ACK only queues the command: the native
helper takes two finite paused updates and checks a real reset witness before
counting post-reset advancing updates. No reset-only iteration is counted as a
physics/publication tick.

Source regressions cover independent clocks, inheritance, checked rates,
canonical/fixed classification, actual motor-driven relative motion, moving
jointless models, no-interest acquisition, pause, reset, stale retirement,
two-endpoint preflight, canonical wire headers and weak/epoch cleanup. The native
helper `native_pose_static_publisher.cc` checks actual typed provider/consumer
readiness on both topics and actual callback completion before phase changes.
Its modes are `split`, `rate`, `inherit`, `paused`, `rewind`, `jointless`, and
`fixed`. Each received vector is compared with the corresponding current native
component transforms by key. These are required acceptance checks, not a claim
that authoring Source alone has executed them.

Pinned reference: Sim `71c60db31fddd5c98d2aa689e9a2cccb3548a0a3`,
`src/systems/pose_publisher/PosePublisher.cc` SHA-256
`e70f34969f0fa128da64e60803423d83327ab7a6e9ca9d1f6caf6761582e705b`.
Nested models, sensor/visual/collision poses, individual Pose messages, multiple
publishers, dynamic system plugins, native hash order and performance parity
remain outside this bounded feature. This is a Source feature increment, with
no new installer/browser proof or change to the accepted M29 installed preview.
