# Public identities and reset reservations

EntityId is a world-local, nonzero public identity. Bevy Entity values are runtime
handles and must not be serialized as Gazebo IDs. assign_id allocates monotonically;
removing a component or entity retires its lookup. Moving the component cannot
redirect that lookup. Within an intact allocator, ordinary allocation never
recycles an issued ID.

An explicit scene reset must revive initial logical objects with their original
public IDs. reserve_id captures an opaque ReservedId from a live, valid owner.
It binds the ID to both the Bevy WorldId and the current allocator instance.
Clearing/replacing the allocator invalidates its reservations, even in the same
Bevy world. Reinitializing a cleared allocator starts a new identity namespace
and may reissue numeric IDs. Direct resource destruction is simulator teardown,
not a supported scene reset. A scene reset must preserve installed resources;
Bevy 0.20 clear_entities also destroys resource entities and is unsuitable here.
ReservedId can be cloned for repeated resets; it does not copy any
domain components or authorize restoring into a different world.

restore_id restores a reservation to an unassigned entity after the previous
owner is gone. Repeating it for the same valid owner is harmless. Missing targets,
occupied IDs, conflicting target identities, foreign worlds and invalidated
allocators return typed errors before mutation. Restoring an earlier ID does not
rewind the allocation counter. The caller must restore the corresponding logical
object's domain data; assigning the token alone does not reconstruct that object.

A retained runtime handle must include its simulation epoch when identity can
survive reconstruction. A public ID refers to the logical object; a stale Bevy
Entity must not become a handle to its replacement. Full scene/backend restoration,
plugin handle validation, stream reconstruction and native reset-time added/removed
observations remain CORE-02 work. These primitives do not implement remote reset.

Tests cover repeated restoration, monotonic allocation, live-owner collisions,
missing or already identified targets, foreign-world components, allocator
replacement and lookup cleanup. Native reset reference behavior remains recorded
in planning/reference-results.json; no original case is retired by this primitive.
