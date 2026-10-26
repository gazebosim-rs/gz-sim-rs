# Initial headless scene contract

`gz_sim_scene::load_world(xml, source, world_name, step, resolver)` returns a new
`Simulation` after SDF validation and backend construction. A failure cannot
change another running simulation. Includes use the supplied SDF resource resolver;
`NoResources` supports self-contained files. No network or GPU is involved.

The current supported scene has unjointed models, nested models, static inheritance,
and one centered sphere, box, cylinder or capsule collision per link. Inertia
tensors use SDF's
xx,yy,zz,xy,xz,yz order; the inertial rotation is applied before Rapier's symmetric
eigendecomposition. Nonpositive/invalid inertia and derived overflow are rejected.
The scene requires a centered center of mass. The backend itself also represents
offset COM: `BodyState.position` is the body origin, while velocity is COM velocity.

Link entities expose `Name`, `WorldPose` and `Velocity`; Rapier links also carry a
read-only `BodyHandle`. Bevy hierarchy and stable EntityId values describe loaded
world/model ownership. Checked model-subtree removal and loaded reset preserve
original public IDs; general scene editing remains pending.
The scene owns body lifecycle: removing its backend or bodies while retaining
these systems violates the current constructor contract.

Sensor entities expose their scoped `Name` and, after acquisition, `ImuSample`.
Only centered IMUs, no noise, default CUSTOM orientation configuration, and a
native-phase sampling rate independent of the physics rate are supported. Rate0
samples each physics update; positive rates emit at most once per update and skip
missed periods without losing phase. See SENSOR_SCHEDULING.md for bounds and reset.
Unsupported configurations, jointed models, plugins, other shapes, offset or
multiple collisions fail with an entity-specific error. Rendering, subscriber
activation and metrics remain pending; HEADLESS.md describes server publication.

Relative poses within1e-12m/quaternion components are treated as centered/identity
to absorb semantic-transform roundoff; q and -q are equivalent. This tolerance is
tested on arbitrary rotated models. Sensor-local orientation may be arbitrary.
Its initial world orientation is retained as the measurement reference, matching
the original simulator's IMU system.

Each unpaused step runs physics, copies body state at PhysicsCommit, and acquires
IMU data at PostStep. Acceleration uses the step's velocity difference; sample time
is that step's integer end timestamp. No samples are synthesized for past instants.
Paused ticks keep the previous sample. Reset restores physics first, synchronizes
ECS state, clears samples/history, and starts a new clock epoch. The next sample
uses the restored initial orientation reference.

Tests cover falling-body acceleration/IMU values, static gravity, nested transforms,
relative orientation under body spin, pause/reset/epoch order, unsupported inputs,
and failure isolation. These are integration tests for this subset, not a completed
port of an original whole-world simulator test. Full performance and same-backend
C++ comparisons remain required before SLICE-01 acceptance.


Model anchors and IMUs retain public link IDs. Their scheduled systems resolve
those IDs through a read-only view of the existing core index and consume the
committed link WorldPose/Velocity. Backend synchronization runs first. This keeps
observations independent of solver handles and allows original logical links to
use replacement Bevy Entity handles after reconstruction. Sensor-relative frames,
velocity history and pause/reset rules are unchanged.
