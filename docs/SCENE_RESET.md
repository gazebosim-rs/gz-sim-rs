# Loaded-scene reset

reset_loaded_world restores the scene captured at load_world completion in the
same Bevy World. A typed immutable blueprint contains the loader's names, kinds,
initial poses, hierarchy, physics bindings and IMU configuration. It stores public
identity reservations rather than Bevy Entity handles. This is not a general
component snapshot; plugin configure/capture contracts remain separate work.

Reset first validates the epoch, original world/allocator identities, current
identity ownership and backend ownership. It constructs the replacement physics
state before modifying entities. Missing initial state/backend, foreign ownership,
moved identities, resources or Bevy infrastructure parented into scene ownership, initial scene
nodes carrying infrastructure roles, and unsupported runtime
physics bindings fail before mutation. Runtime model construction is not provided
by this loader; its future reset contract must extend the current explicit check.

The transaction detaches ownership edges, recreates original nodes with their
reserved IDs, reconnects initial and pending-runtime parents, and commits prepared
physics. Resources, registered systems and global observers survive. Observers bound to an
old runtime Entity follow Bevy target lifetime and need reattachment to a restored
logical ID when that runtime entity is reconstructed. Clearing/replacing World
would invalidate those systems; clear_entities/clear_resources are not used.
The allocation counters never rewind. Revived logical IDs use new runtime Entity
handles and a new simulation epoch.

Reset handlers run once without normal simulation phases. RestoredPhysics marks
the already prepared engine for that epoch, avoiding a second solver reconstruction;
other epochs retain ordinary backend reset behavior. The marker is removed after
handlers finish. SceneResetInfo records previous time, next epoch, restored IDs
and runtime public IDs awaiting removal. The previous time represents the native
rewind amount even though the new Rust UpdateInfo uses unsigned Duration.

The immutable observer runs after handlers while PendingResetRemoval entities
remain visible. Runtime Name/component observations are therefore available at
the same lifecycle boundary as native EachRemoved. Cleanup follows observation,
including when its returned value is an error. A panicking handler or observer
invalidates the run, as with other simulator systems; mutations are not rolled back.
The result is returned after cleanup. The caller supplies pacing and bounded IO.

Tests preserve the selected native ResetToWithDeletedEntity and
ResetToWithAddedEntity Name/new/removed observations using Bevy change tracking
and the explicit pending-removal view. They also cover original IDs/poses, nested
frames, removed physics and IMU recovery, custom resources, repeated paused/running
resets, unsupported bindings and preflight errors. Each original case repeats nine times to detect static-global leakage; each
Rust case likewise runs through nine independent Simulation instances. These are
repetitions, not alternative C++ storage implementations. Mappings require
independent review; no broad ECM parity is claimed.

SERVER_RESET.md describes the bounded remote composition: control ordering,
real-time clock reset, stream reconstruction and old admitted-request cancellation.
Already queued transport data cannot be recalled. TPE, offline resources and equal-work performance remain P2
requirements; no performance improvement is claimed by this reset increment.
