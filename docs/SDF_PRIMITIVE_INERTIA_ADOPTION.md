<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Consumer adoption of primitive automatic inertia

Root owns this source-only follow-up in `work/sdf-capsule-inertia-adoption`.
It starts at accepted contact-factorization Main `f136e821`, qualified by exact
hosted run `41497378596` and independent official audit. The producer pin advances
from `f0d05146` to accepted SDFormat `a44bf8be`. Producer Cargo manifests and lock
content are identical between these revisions; only the consumer's exact Git
source identity changes. Fuel remains untouched.

The accepted producer adds combined-pose box/sphere inertia, single cylinders and
origin-centered capsules through maintained Parry mass properties and existing
frame/tensor validation. Scene must continue to consume resolved inertial values
without recalculating them or changing backend admission. Translation-only capsule
work beyond `a44bf8be` is not adopted implicitly.

The new consumer fixture selects the original-dimensions cylinder and capsule
inputs and native observations verbatim from the accepted producer. It retains
source paths, whole-file hashes, exact original XML and URI, native stdout/error
vectors, revision and Apache attribution. The regression checks resolved mass
and all six inertia components at the producer's existing four-ULP boundary,
then wraps the unchanged model in an explicitly zero-gravity 1 ms world.

Across TPE, Rapier, DART and MuJoCo it compares automatic and exact round-tripped
manual inertial inputs through existing loading, force/torque response, collision
density ownership, removal and loaded reset. Runtime identifiers and zero-velocity
initial state must be restored. TPE's existing unsupported-wrench result remains
explicit. No old test, fixture, backend restriction or tolerance is changed;
there is no new upstream-original credit.

Complete regression code, task/handoff documentation and independent source review
before a consolidated required gate. No compiler, tests, lint, native capture or
consumer qualification has run for this source follow-up.

The same publication carries the separately reviewed contact-factorization
measurement appendix and compact actual observations. That batch measured
`f136e821` with producer `f0d05146`; it does not measure this `a44bf8be` consumer
pin. The original baseline archive stays unchanged. Independent source/archive
review checked all 141 members against actual retained bytes. Completing this
appendix before the consumer's required gate avoids a separate verification round;
no timing workload is repeated. See
[CONTACT_PHYSICS_MEASUREMENT.md](CONTACT_PHYSICS_MEASUREMENT.md).
