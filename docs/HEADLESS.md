# Headless integration

IO-01 and P2 remain in progress. The server joins an SDF/Bevy/Rapier world to
independent Rust message and transport libraries. It owns these endpoints:

| Endpoint under `/world/<name>` | Contract |
| --- | --- |
| `/clock` | gz.msgs.Clock with simulation, active real and system time |
| `/control` | gz.msgs.WorldControl to gz.msgs.Boolean; pause, exact stepping and loaded-scene reset |
| `/pose/info` | gz.msgs.Pose_V with stable IDs and parent-relative poses |
| `/remove` | gz.msgs.Entity to gz.msgs.Boolean; deferred command acceptance |
| `/remove/blocking` | Same request; reply reports execution result |

Root `/clock` election and complete scene messages/services remain open.
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
Seed changes and run_to_sim_time return a successful service response with
Boolean.data=false. Loaded-scene all/time_only reset is queued for the next tick;
model_only is ignored and acknowledged. See [SERVER_RESET.md](SERVER_RESET.md)
for control overwrite, pending iteration, stream and cancellation boundaries.

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
publisher leases without socket waits. Runtime registration, rates, metrics, noise and nonzero covariances remain open.
Frame metadata chooses a nonempty canonical frame_id, then a present legacy
gz_frame_id even when empty, then the effective worldless sensor name. Native XML
string normalization and the __default__ topic sentinel are handled by SDF before
transport name repair. Existing retained
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

The loaded-scene API now restores removed initial logical objects with their
original IDs and retires runtime additions after reset observation. It preserves
resources and registered Bevy infrastructure in the same World. The separate core
reset still restores only retained domain state and time. Remote reset composes
the loaded scene with command ordering and stream reconstruction. See
[SCENE_RESET.md](SCENE_RESET.md) and [SERVER_RESET.md](SERVER_RESET.md).

## Reproduction and remaining work

Run `python3 scripts/bootstrap-native.py`, then `python3 scripts/verify.py`.
Native libraries are built from planning/transport-reference-lock.json. An existing
identical build may be selected with GZ_REFERENCE_PREFIX; source configuration,
library hashes, pkg-config prefix and dynamic linkage are checked.

Six real C++ client scenarios verify clock/exact-step control, actual IMU samples,
topic repair, removal, two IMUs sharing one endpoint and remove/reset/resume. They wait for received clock/pose/sample packets rather
than assuming readiness. Removal checks ordinary acceptance, blocking failure,
ID precedence and disappearance of the ball while the ground keeps its IDs.
Rust tests also cover invalid/overflowing commands, duplicates, scoped-name and
identity checks, paused/running removal, batch limits, client timeouts, shutdown,
sequence wrapping, invalid samples and publication failures.

The current gate passes 115 Rust checks and nine gate tests: 1870/1880 production
lines and 2884/2932 regions. Every production crate exceeds 95% for both metrics;
The reported-fault increment covers all 57 changed executable lines. Independent review and the exact
staged gate are required before publishing this increment.

Remaining P2 work includes TPE, local/Fuel resources, sensor scheduling, the
simple-joint variant and velocity/force input required by PLAN.md, plus the
equal-work C++ comparison/performance report. The complete articulation, system
and backend inventories, Bevy rendering and release qualification remain open.


## Reviewed producer integration

The workspace pins SDF 80b6c74a, messages 195644d8 and transport c00de5e4.
SDF supplies canonical-link selection directly: the returned link is borrowed from
that model's subtree, whose links are all constructed before the model anchor.
The body lookup is consequently an internal construction invariant. No parallel
canonical-selection implementation remains in this workspace.

SDF also retains contact/friction metadata and uses its audited Linux GNU scalar
conversion boundary. The scene rejects every explicit surface, including an empty
one, until backend parameter mapping exists. It never silently accepts ignored
friction or collision filters. Parser conversion and resolver IO happen during
loading, outside stepping; Linux GNU qualification remains explicit. The reviewed
1.5 subset now passes the unchanged legacy-world physics/IMU assertions alongside
1.6 and 1.7. Any explicit fluid added-mass matrix, including empty or zero, is
rejected on static and dynamic links until a backend mapping exists. Absent
metadata retains current behavior; no fluid physics is silently ignored.

The transport pin rejects duplicate same-node publishers and orders withdrawal
before immediate recreation. Surviving stream leases are retained during
reset; recreation after removal has a native-tested producer contract.
Consumer tests cover frame-field precedence, empty legacy IDs, topic defaults and
XML whitespace, explicit unsupported surfaces and all seven real C++ scenarios.


World-bound identity reservations now permit explicitly restoring an original
logical ID after its owner is gone. They also bind the allocator instance, so
resource teardown cannot validate a stale reservation. Ordinary allocation remains
monotonic within the intact allocator; restoration does not rewind its counter.
See [IDENTITIES.md](IDENTITIES.md). Loaded-scene reset composes these reservations
with backend restoration and the remote stream/control integration.


Multiple IMUs may share a topic, including relative/absolute aliases under a
namespace. The server uses the transport qualifier as the cache key and shares
one publisher while retaining each source's metadata, samples and sequence.
The temporary cache holds only weak references; retiring one stream preserves
its peers, and retiring the last releases the endpoint. A fifth C++ scenario
receives both frame IDs and independent sequences through the aliased endpoint.
The world uses one logical publisher per shared topic; discovery publisher counts
differ from native per-sensor Nodes. Full discovery/inspection parity remains
separate NET work. No additional IO worker is created per sensor.


Prepared physics restoration now captures validated construction records and engine
settings, builds an independent solver before domain mutation, and commits only to
its owning backend. It revives initial body IDs, retires runtime bodies and retains
all later allocation history. See [PHYSICS_STATE.md](PHYSICS_STATE.md). This API and
the identity reservations are composed with the loaded-scene blueprint and reset-time
observations. The server composes command ordering and stream reconstruction.


Loaded-scene reset preflights identities (including resources), infrastructure
ownership and prepared backend state before mutation. New runtime handles retain
original logical IDs. Pending additions remain queryable during Reset and the
immutable observer, then retire. Global observers and registered systems survive;
targeted observers retain Bevy's runtime-target lifetime. Conflicting infrastructure
and initial-scene roles fail before reconstruction. The two original Name/reset
assertion scenarios each run nine fresh worlds to preserve static-isolation intent.


The dependency-free gz-physics workspace crate owns the shared f64 shape,
construction, inertial, rotation and state values. Rapier reexports preserve its
existing Rust paths. Handles, ownership, errors and schedules remain in the real
adapter. The values moved unchanged as the first TPE integration prerequisite;
this extraction adds no engine behavior or performance claim.


A separate TPE motion kernel now operates on validated shared f64 state. Its
native corpus, numeric contracts and narrow equal-work measurement are recorded
in [TPE_MOTION.md](TPE_MOTION.md). The server still installs Rapier; TPE world
integration and full headless acceptance remain open.


Canonical model anchoring and IMU sampling now read committed Bevy link
WorldPose/Velocity through stable public link IDs. EntityLookup is a read-only
system parameter over the existing identity index and validates the live owner.
Backend synchronization remains first in PhysicsCommit and Reset; observation
systems no longer fetch Rapier state themselves. Analytic tests without a physics
backend cover frame projection, pause/reset and replacement runtime link handles.
All six existing native client scenarios retain their assertions. This prepares
another backend; it does not install TPE or decide D23 behavior.


Reported system errors now use a typed first-fault channel that preserves Bevy
parallelism and stops later registered systems. Failed normal ticks retain last
committed timing, and server publication/control processing stops. Committed
reset faults clean pending nodes and streams without observing invalid state.
See [STEP_FAULTS.md](STEP_FAULTS.md) for terminal-run and side-effect boundaries.

## Selectable TPE kinematics

Use `--physics-engine tpe` or `load_world_with_engine(..., PhysicsEngine::Tpe)`
for the bounded kinematic TPE path described in [TPE_SCENE.md](TPE_SCENE.md).
Default Rapier behavior is retained. The seventh real C++ scenario verifies TPE
clock/pose/IMU/control/removal/reset interoperability; it does not close native
TPE contacts, commands, nested models or the complete P2 comparison.

The current SDF pin exposes reviewed pure model.config selection, merged-include
and terrain metadata subsets. Collision geometry capability guards remain in the
loader. Visual cast-shadows, transparency, visibility flags and optional laser
retro metadata are now owned by ECS components and the loaded-reset blueprint;
parsing them does not implement rendering or lidar effects. A removal/reset
regression preserves nondefault values and distinguishes Some(0) laser retro.
The config selector is preparation for a bounded filesystem/offline cache consumer;
selecting a filename is not a resource lookup or a promise that its SDF version
can be loaded. Local/offline resource resolution remains under implementation.
