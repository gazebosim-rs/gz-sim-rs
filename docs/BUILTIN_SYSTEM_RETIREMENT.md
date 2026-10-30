<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Built-in system retirement boundary

ACT-01 requires removing unused work, not running empty queries or a per-frame
condition for every possible system. The core retirement primitive uses the
pinned Bevy 0.20 scheduler directly. It adds no system loader or parallel registry.

`Simulation::remove_systems_in_set` removes the supplied Bevy set's systems from
the tick schedule, or the separate reset schedule when `Phase::Reset` is passed.
Other phase values select the tick schedule; they do not narrow the set's members
to that phase. Bevy's `RemoveSystemsOnly` policy retains ordering sets so later
reactivation preserves phase/dependency relationships. Empty shared schedule
metadata may remain. Collective or set-level conditions also belong to those
retained sets, so their Local allocations are not released by this primitive,
even after all member systems retire. Built-ins requiring D26 absence must not
put system-owned state in collective/set-level conditions. The immediate-release
promise applies to systems and their per-system conditions only.

Bevy keeps the old executable until the schedule is rebuilt. The primitive
initializes after removal so boxed systems, their query caches, Local allocations
and per-system conditions are released before successful return, even if no later
tick occurs. This work belongs at structural lifecycle boundaries outside an
executing schedule. Pending registrations can initialize there. Schedule errors
propagate; this API does not promise rollback of arbitrary user initializers or
repair a terminal simulation fault.

The alternative `RemoveSetAndSystems` policy panics on rebuilding the pinned
Bevy version in the independent two-system probe. Do not switch policy or vendor
a scheduler fork to erase retained ordering metadata. The selected policy is
covered by actual destruction, condition-invocation, reactivation and ordering
controls using owned Local buffers.

Nine core controls cover no activation before registration; immediate system and
condition destruction; no subsequent callbacks/condition checks; fresh Local
state on reactivation; preservation of unrelated Local state and phase order;
separate tick/reset retirement; unknown and already-empty sets; pre-tick
retirement and pending schedule errors without callbacks; multiworld
isolation; and preservation of terminal faults. These are new Rust lifecycle
contracts, with no upstream-test completion credit. A separate collective-condition
control preserves the retained-state limitation instead of claiming universal
zero-owned-state teardown.

This primitive does not yet wire Scene demand or retire transport workers.
Scene integration must preserve removal preflight/atomicity and loaded reset
reconstruction. Joint/IMU callbacks and server ingress require their own reviewed
consumer changes. Equal-work CPU/memory evidence and full zero-idle acceptance
remain open. Runtime checks must distinguish released owned state from shared
allocator/scheduler capacity rather than claiming process RSS becomes zero.
