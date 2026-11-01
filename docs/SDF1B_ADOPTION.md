<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# SDFormat manual inertia and legacy IMU adoption

The earlier adoption pinned SDFormat to
`1b57ab0ce1ce88dddcd0f36a3acd667cb6dff6da`, replacing `486b3e0`.
The later [automatic-inertia adoption](AUTO_INERTIA_ADOPTION.md) advances that pin.
The producer adds legacy IMU noise conversion, explicit manual-inertia
attributes, and standalone lidar aliases. Existing Scene admission and lowering
already handle these values; this update changes no simulation implementation.

For direct SDF 1.4/1.5 root or world models, and independently loaded resources,
legacy IMU rate and acceleration noise reaches all six channels. Supplemental
tests compare seeded readings, covariance, simulation timestamps and reset
against equivalent modern per-axis noise on TPE, Rapier, DART and MuJoCo.
They check resource provenance and refuse noise that the runtime cannot represent.

Manual inertia with omitted `auto`, `auto='false'` or `auto='0'` retains the same
mass, inertia, force response, torque response and reset behavior. TPE still
refuses external wrenches. Automatic inertia remains unsupported at this pin;
the separate automatic-box producer candidate is outside this adoption.

Standalone `lidar`, `ray`, `gpu_lidar` and `gpu_ray` roots expose metadata and
serialize as canonical `lidar`. They do not enable Scene lidar execution.
Sensor documents retain their existing lidar/ray payload grammar, and Scene
continues to reject these sensor kinds explicitly. No renderer, worker or
endpoint is activated by parsing standalone metadata.

`crates/gz-sim-scene/tests/sdf1b_admission.rs` contains the four supplemental
consumer tests. Existing noise and producer-admission tests remain unchanged.
These controls are not additional completed upstream tests. The producer's
original-test and native-oracle evidence remains in its own repository.

Acceptance requires independent review, the complete normal staged hook,
strict owned coverage, real native and rendering checks, and exact-commit
hosted CI. Focused readiness checks alone do not accept this consumer pin.
