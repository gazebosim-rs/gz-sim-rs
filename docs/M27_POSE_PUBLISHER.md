<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M27: flat-model canonical PosePublisher

This bounded built-in recognizes the exact `gz::sim::systems::PosePublisher`
name and `gz-sim-pose-publisher-system` filename. It does not load a native
library into Rust. Use [the source example](../examples/m27_pose_publisher.sdf)
with the Rapier backend. The example retains its two links, articulated joints,
collision and visual geometry; selecting pose records does not filter entities.
This source feature is separate from the accepted M25 installed archive.

One top-level flat Rapier model may configure one publisher, with at most 128
selected records. Static models are not rejected merely for being static; the
separate `static_publisher` transport mode is unsupported. Set
`use_pose_vector_msg` to `true` explicitly. `publish_link_pose` defaults to
`true` and `publish_model_pose` defaults to `false`, matching pinned gz-sim
`71c60db3`. Either selection or both are supported, but an empty selection is
refused. The example selects both, on `/m27/pose`.

Without `<topic>`, the native default excludes the world:
`/model/rig/pose`. An explicit topic must be an admitted absolute local topic;
all remapped effective endpoints are reserved against existing world, sensor,
controller, drive and other publication families before declaration or reset.
Initial loaded endpoints remain reserved after removal, so a later loaded reset
cannot resurrect a conflicting endpoint. The shared WorldIO worker owns native
transport declarations; weak CPU leases, bounded queues and the global reset
fence prevent queued old-generation data from acquiring a new identity.

## Message and frame contract

The canonical message is the existing `gz.msgs.Pose_V`, not the viewer's world
pose stream. Its outer header is absent. Each contained `Pose` has:

- its own simulation timestamp;
- exactly two ordered header entries, `frame_id` then `child_frame_id`;
- `name` equal to `child_frame_id`, and wire `id` zero;
- the actual parent-relative local translation and quaternion.

For world `default` and model `rig`, the model record has parent frame `default`
and child/name `rig`. A link `base` has parent frame `rig` and child/name
`rig::base`. A moved or rotated model must not turn the link record into a world
transform. Public entity identities remain internal ownership guards, including
identities larger than uint32; no fabricated numeric wire ID or sequence is added.

Rust emits stable lexical child-frame order. Native `PosePublisher` iterates an
unordered map, so native order is not canonical. Native proof retains actual
wire record order and compares records by unique child-frame name, without
sorting or rewriting the captured message.

## Cadence, ownership and reset

`update_frequency` defaults to -1. Every finite nonpositive value publishes on
every unpaused tick. Positive rates use the native truncated signed-nanosecond
period, evaluating `(1.0 / frequency) * 1e9` in native order and rejecting an
unrepresentable reciprocal. A subnanosecond period becomes
zero, including rates above 1 GHz; this is not the sensor scheduler's 1 ns clamp.
The last publication starts at time zero. Skip only when the positive difference
from the last actual publication is smaller than the period. An equal-time
**unpaused** update and a backwards time jump are eligible; a paused update never
publishes. At 100 Hz and 1 ms steps, the first positive-time frame is at 10 ms.
Scheduling anchors to each actual publication, not a fixed phase grid.

Native does not gate acquisition on subscriber interest. Scene therefore samples
while the configured publisher exists, even without listeners. Server sends each
new epoch/revision once; it does not apply viewer wall-clock throttling. An absent
family owns no scheduled callbacks or family IO state, and last-model removal
retires its schedules and weak lease.

Direct Rust core reset clears the old sample and changes epoch while retaining
live cadence state. This convenience is distinct from native world/time-only
reset: native `SystemManager::Reset` reloads PosePublisher because it has no
`ISystemReset`. Rust loaded reset reconstructs initial metadata, last time zero,
revision zero and fresh Bevy handles, matching that native reconfiguration.
Neither path invents a paused-time publication. Immutable model/link identities,
kinds, parent relationships and metadata are checked before emission. An invalid
or missing local pose removes the previous sample and reports a terminal fault;
no partial vector or previous valid frame is substituted.

## Explicit limits and evidence

Nested models, nested/visual/collision/sensor pose publication, static publisher
or static frequency, individual-Pose mode, duplicate or unknown options,
unsupported XML, multiple configured publishers and runtime plugin spawning are
refused. Existing visual, collision and sensor entities may remain in the world;
they simply do not add records to this publisher. This adds no transform graph,
transport protocol, dynamic plugin host, viewer behavior or physics capability.

The regression source covers strict admission, local physical transforms, real
supported joint motor motion, cadence/paused/reset behavior, atomic bad-frame
refusal, absence/removal, canonical wire decoding, declaration lifetime and
bounded/fenced IO. The native helper's `default-links`, `both`, `model-only`,
`rate`, `paused` and `rewind` modes use the same loaded fixture, real DART component
poses and canonical messages. `rewind` witnesses actual native world/time-only
plugin reload, not a retained native timer. The control-service acknowledgement
only queues the request. One finite paused step consumes it after old-time
PostUpdate; the next finite paused step applies reset without PostUpdate. The
helper verifies the actual reset callback at time zero while paused before its
ten advancing measurement steps. An unpaused reset-only iteration otherwise
consumes an iteration without producing a measurement. The failed original
native attempt and its missing 10 ms post-reset frame remain retained; no timeout,
packet count or value assertion is relaxed. Direct zero-dt eligibility is tested
at the Rust publication boundary against the pinned native source rule.

Native execution, full normal/strict coverage, exact hosted qualification and
independent DATA are still required for acceptance. Source review is not a test
pass, and these cases establish no cross-engine trajectory, performance or full
PosePublisher compatibility claim. A future refreshed installed package needs
its own distribution qualification.
