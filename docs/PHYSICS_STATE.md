# Initial physics state

Every dynamic engine implements `gz_physics::DynamicBackend`; the scene owns one
type-erased `PhysicsBackend` resource and installs it with `gz_physics::install`,
which reports step and reset failures through StepFault. Engines return
`PhysicsError::Unsupported` for missing capabilities and `PhysicsError::Native`
for native failures rather than approximating behavior.

DynamicBackend::initial_state captures the remaining bodies' validated construction
records and current gravity/integration settings. It is intended to be captured
when scene construction completes. It preserves spawn velocities and rotations;
it does not snapshot a moving world's current solver state.

prepare_initial builds a complete replacement solver before domain mutation.
apply_initial commits that prepared state, reviving the original BodyIds and
retiring bodies added later. Both opaque values belong to their original backend
instance through an owner token; the payload type is private to each engine. A
foreign value returns ForeignInitialState without modifying the target.
Applying a prepared state preserves the allocation high-water mark, including
allocations made after preparation. IDs of removed runtime bodies are not reused.

A full scene reset must preflight both backend and public entity identity ownership,
then rebuild matching scene entities before invoking reset handlers. This backend
primitive does not update the ECS, clock, epoch or sensor streams. Runtime handles
must be paired with their owning world/reset epoch to distinguish revived initial
logical bodies from stale runtime references.

The existing retained-body reset remains separate: removed bodies stay removed.
The Rapier engine retains integration parameters as well as gravity. Both reset paths reuse
private validated construction records, including explicit inertia. Tests verify
removed-body revival, runtime retirement, sparse/empty scenes, repeated resets,
foreign ownership, velocities/orientation, gravity/settings and late allocations.
No native test is retired and no performance improvement is claimed by this API.
