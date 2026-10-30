<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Loaded-joint callback demand candidate

This ACT-01 candidate installs joint binding preflight and committed/reset
sampling only when the loaded scene contains joints. It has no per-tick
condition or polling to detect demand. Jointless worlds own none of these
three callbacks or their query state, and no installation marker.

Shared callbacks remain while any joint model is loaded. Removing the last
consumer validates pending schedules before final body validation/removal, then
retires the three callbacks after the removal commits. Schedule-built observers
therefore run before final validation or see committed body removal. A teardown
failure after commit reports the existing terminal scene fault rather than
pretending removal was rolled back. Loaded reset reinstalls callbacks after
restoring entities and backend bindings, before running reset sampling. Core
time reset does not recreate removed models. Retirement rechecks both joint
and IMU demand after schedule builds, because an observer may remove the other
family's last consumer. Installation markers only retire during this bounded
recheck; it never runs during stepping. Raw ECS consumer additions during
retirement are unsupported and produce a terminal committed fault if they
would leave live consumers without callbacks.

Joint preflight still precedes persistent effort application. Existing joint
samples, owner/binding validation, physical fixtures and tolerances are retained.
No loader, extension host, worker, endpoint or alternate ECS is introduced.

Eleven focused controls cover initial absence, idempotent installation, shared and
last-consumer removal, both reset paths, invalid schedules, schedule-built
observers, combined and cross-family teardown, post-reset cleanup and committed failure across the three dynamic engines.
Initial absence also covers TPE. Full scene regressions, independent review and
normal staged/hosted qualification remain required before acceptance.

Persistent effort application has a separate controller-demand lifecycle; see
JOINT_FORCE_SYSTEM_LIFECYCLE.md. Generic input callbacks, server workers and
physics activation remain unfinished demand work. Bevy may retain
empty set/ordering metadata after removing a system; this increment does not
claim whole-schedule zero memory or measured CPU/RSS improvement.
