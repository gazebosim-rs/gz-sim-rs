<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Contact exclusion groups

The candidate shared backend contract adds `set_self_collision_group(body,
group)`. Rapier excludes contact pairs whose bodies have the same assigned
backend-local group. `None` keeps ordinary contact behavior. All u64 IDs,
including zero and the maximum value, are supported without a layer-count cap.

Rapier stores the optional group in each body's existing construction record.
Its private collider `user_data` is reserved for the encoded group; zero means
unassigned and an assigned ID is stored as u128(ID) + 1. The contact hook reads
two collider values without a pair table. It retains the exclusion of two non-dynamic bodies and leaves existing collision, solver and joint
filters in place. Different groups still contact each other and external ground.

Edits are construction-only. A nonzero step closes construction; later edits
return explicit `Unsupported` without changing the body or group. Ordinary reset
retains the remaining bodies' current construction groups and reopens edits.
Prepared initial-state restoration restores the captured groups, including edits
made after preparation within the same epoch. Removing a body retires its group.

DART and MuJoCo explicitly reject this new capability. There is no silent native
fallback. This primitive does not expand SDF topology admission, implement model
collision policy, or change scene loading. Ray queries and sensor intersection
filtering are outside this contact-only contract. No numerical-parity or
performance improvement is claimed.

Tests cover same/different/unassigned groups, full-width IDs, clearing groups,
actual external fixed-ground contact, construction errors, both reset paths,
prepared same-epoch edits, removed IDs and real native capability rejection.
Equivalent enabled-contact paths are compared to an untouched Rapier reference;
the position comparison allows sixteen machine epsilons for final-bit rounding,
with velocity and orientation checks retained exactly. Existing tests are unchanged.

Independent review and the normal strict staged and hosted checks are required
before acceptance.
