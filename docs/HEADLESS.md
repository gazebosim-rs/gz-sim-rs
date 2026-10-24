# Headless integration

IO-01 and P2 remain in progress. The server joins an SDF/Bevy/Rapier world to
independent Rust message and transport libraries. It owns these endpoints:

| Endpoint under `/world/<name>` | Contract |
| --- | --- |
| `/clock` | gz.msgs.Clock with simulation, active real and system time |
| `/control` | gz.msgs.WorldControl to gz.msgs.Boolean; pause and exact stepping |
| `/pose/info` | gz.msgs.Pose_V with stable IDs and parent-relative poses |
| `/remove` | gz.msgs.Entity to gz.msgs.Boolean; deferred command acceptance |
| `/remove/blocking` | Same request; reply reports execution result |

Root `/clock` election, complete scene messages/services and full reset are open.
The coordinator owns this integration, native clients and reference build. Source
contracts come from pinned gz-sim SimulationRunner, UserCommands, Util and the
scene/sensor implementations. New conformance cases preserve the selected native
semantics; no complete original simulator case is retired.

## Clock, control and bounded IO

Each tick advances at most one physics step and publishes through bounded,
nonblocking queues. Active real time excludes pauses; system time is the actual
epoch. Queue acceptance does not prove subscriber delivery. Counters distinguish
queued publications, invalid samples/frames, rejected commands and send failures.
Transport also exposes queue and native error counters. Construction, host pacing,
filesystem access and worker joining stay outside simulation steps.

Control polling has a finite per-tick budget. Pause is always applied. Nonzero
multi_step overrides step; pending iterations are added only when pause=true.
They pause the server on completion; an intervening pause preserves the count.
Overflow is checked before changing state. Replies acknowledge command acceptance.
Seed changes, run_to_sim_time and nonempty reset requests currently return a
successful service response with Boolean.data=false. They are explicit unsupported
capabilities until their native contracts are implemented.

## Entities, poses and IMU

World, model, link, collision, visual and sensor ownership uses Bevy ChildOf and
Children. SDF frame resolution is separate. Each scene entity has an opaque,
world-local, nonzero public ID. New objects receive monotonically increasing IDs;
lifecycle hooks clean the lookup on removal. Moving an identity component cannot
redirect its original lookup. Direct ECS edits must preserve scene-owned resources
and backend/hierarchy invariants.

Dynamic model world poses follow their canonical link. Static nested models can
provide a dynamic parent's canonical link: canonical selection differs from SDF
frame attachment. Empty dynamic hierarchies without a resolvable canonical link
are unsupported. Link local poses are derived separately; attached visual,
collision and sensor world poses follow their link. Pose packets group models,
links and visuals using leaf names and stable IDs. Only the Pose_V header carries
the simulation stamp. Invalid records or IDs outside the legacy uint32 range
reject the whole frame; IDs are never truncated. The CLI lists physical links.

IMU packets contain actual noise-free samples, simulation stamp, frame_id and
decimal seq entries, orientation, angular velocity, specific force and three
present nine-element zero covariance arrays. Names and default frame IDs omit
world scope. Default topics use the structural world/model/link/sensor path;
explicit topics pass through the reviewed native transport name-repair helper.
Invalid topics fail setup. Sequence numbers count publication attempts and wrap
as unsigned native counters do. Paused ticks do not repeat stale samples; epochs
distinguish equal timestamps after the current core reset.

Sensor registration happens at construction; removed streams release their
publisher leases without socket waits. Runtime registration, explicit frame_id
XML, rates, metrics, noise and nonzero covariances remain open. Existing retained
streams keep their sequence across core time resets; this is not a claim about
full native world reset or sensor reconstruction.

## Removal and reset boundaries

The scene's remove_model API validates a world-direct model, every descendant
body and retained-body aliases before mutation. It then removes backend bodies,
the Bevy subtree and public-ID lookups. Missing, nested, wrong-kind and inconsistent
targets fail without partial deletion. Removed IDs do not resolve, IMU samples
disappear and subsequent steps have no ghost bodies.

Remote removal has a separate bounded batch of at most command_budget commands.
Polling alternates between the two removal endpoints and runs after publication.
The accepted batch executes at the next tick's final boundary, after that tick's
last old-scene pose and IMU packets have been queued. Removal also works while
paused, without advancing time. Already queued packets may arrive after a blocking
reply; receipt of that reply does not flush a subscriber's queue.

The ordinary endpoint replies true for a decoded, queued request even if its
target later proves invalid. The blocking endpoint replies with the execution
result. Malformed payloads reply false. Both use transport success=true for a
normal Boolean response. Client RPC timeout does not cancel an accepted command:
the wire request carries no cancellation/deadline. Reply failures are counted.

A nonzero ID overrides name and type. Otherwise MODEL type and a matching scoped
name are required. Scope matching follows the actual ownership ancestry. Ambiguous
model names are explicitly rejected; native unordered selection is not a stable
target contract. Only world-direct models are supported here; lights await scene
support. A whole batch resolves names before mutation, so duplicate requests for
the same valid model succeed together. Ingress stops at the batch limit; the
underlying finite service queues provide further backpressure.

Full reset must restore removed initial logical objects with their original IDs
and remove runtime additions. The current core reset only restores retained domain
state and time, keeping already removed models absent. Remote reset stays rejected
until initial-scene restoration, identity reservation and stream lifecycle are
implemented together.

## Reproduction and remaining work

Run `python3 scripts/bootstrap-native.py`, then `python3 scripts/verify.py`.
Native libraries are built from planning/transport-reference-lock.json. An existing
identical build may be selected with GZ_REFERENCE_PREFIX; source configuration,
library hashes, pkg-config prefix and dynamic linkage are checked.

Four real C++ client scenarios verify clock/exact-step control, actual IMU samples,
topic repair and removal. They wait for received clock/pose/sample packets rather
than assuming readiness. Removal checks ordinary acceptance, blocking failure,
ID precedence and disappearance of the ball while the ground keeps its IDs.
Rust tests also cover invalid/overflowing commands, duplicates, scoped-name and
identity checks, paused/running removal, batch limits, client timeouts, shutdown,
sequence wrapping, invalid samples and publication failures.

The current gate passes 66 Rust checks and nine gate tests: 1344/1355 production
lines and 1986/2026 regions. Every production crate exceeds 95% for both metrics;
117/118 changed executable lines are covered. Independent review and the exact
staged gate are required before publishing this increment.

Remaining P2 work includes full reset, TPE, local/Fuel resources, sensor scheduling
and the equal-work C++ comparison/performance report. Joints, forces, the complete
system/backend inventories, Bevy rendering and release qualification remain open.
