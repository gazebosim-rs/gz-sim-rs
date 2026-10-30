<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Persistent joint-force callback demand

This ACT-01 follow-up installs persistent effort application only when
a loaded configuration or `set_joint_force` supplies a `JointForceCommand`.
Worlds without controllers have no effort-application callback or its query
state. Registration happens at lifecycle or typed input boundaries, never by a
per-tick condition. Joint preflight still precedes effort application.

A configured controller's initial zero and a programmatic zero retain their
controller state and callback. Zero stops forcing; it does not remove a loaded
controller. Core reset clears the latest command, while loaded reset removes
programmatic commands and restores configured commands to zero. Admission
failures preserve the prior command and activation state.

The last controller's model removal retires its callback even if other loaded
joints remain. Loaded reset reinstalls configured demand. Retirement uses the
existing lifecycle-boundary reconciliation, extended to three monotone markers,
so observer removal of another family's last consumer cannot strand callbacks.
Unsupported raw ECS additions during retirement produce a terminal committed
fault if they would leave consumers without callbacks.

Focused controls cover initial absence, typed activation and invalid input,
shared and last consumers, both resets, cross-family observer cleanup and raw
addition faults across the three dynamic engines. Prior physical assertions,
fixtures and numerical tolerances remain unchanged. Independent review and the
full normal staged and exact hosted checks are required before acceptance.

Generic input/reset callbacks, server ingress/workers and physics activation
remain unfinished demand work. No new worker, endpoint, dynamic loader or ECS
is introduced, and no CPU/RSS performance improvement is claimed.
