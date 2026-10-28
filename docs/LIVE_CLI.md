<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Headless transport server

Use `--server` (`-s`) to expose the existing world services and clock, pose and
IMU streams through Gazebo transport. The server starts paused. Add `--run`
(`-r`) to start advancing immediately:

```sh
gz-sim-rs world.sdf --server
gz-sim-rs world.sdf --server --run --physics-engine rapier --iterations 1000
```

Server iterations count advancing physics ticks across simulation resets. Zero
or omission means unlimited. Paused polls and reset ticks do not consume that
budget. Remote multi-step requests still obey the server's final iteration limit.
Without `--server`, the existing offline behavior remains: default 1000 exact
steps, explicit zero steps, sorted final link state, and no transport worker.

Transport configuration comes from the environment, including `GZ_PARTITION`.
Run preparation admits root-clock publication and refreshes membership before the
loop. It does not guarantee that a future peer is already connected. See
[the root-clock contract](ROOT_CLOCK.md).

The first SDF physics profile sets the target real-time factor; an explicit
`--step-ns` still takes precedence for the simulation step. Factor zero disables
pacing. Invalid or unrepresentable wall periods fail before worker creation.
The driver sleeps outside `Server::tick`; paused polls and paced waits check
termination at most every 10 ms. Preparation can take five seconds, and joining
workers is a separate shutdown lifetime. This is not a hard real-time guarantee.
The first SIGINT or SIGTERM requests shutdown, joins the workers, and prints
final state. A second termination signal forces immediate exit with status
`128 + signal`, including when final output is blocked; it can interrupt cleanup.
Queued publications do not guarantee final wire delivery.

A world name that is not a valid transport topic segment, such as one containing
a space, currently fails server startup with `InvalidName`. Native Gazebo
sanitizes such names instead; matching that is open IO-01 work.

## Focused evidence and remaining scope

Independent review verified one pacing unit test, all ten CLI integration tests,
a real-process finite-budget regression across pause, reset and multi-step
requests on TPE and Rapier, and consuming server shutdown. A real C++ client test exercises the actual
CLI executable on both engines: identical root/world Clock bytes at 0/1/2 ms,
remote stepping, unlimited paused lifetime, and clean SIGINT/SIGTERM exit.
A separate regression fills an unread final-output pipe and requires the second
SIGTERM to exit with status 143. The original draft ignored that signal; its
reproducing executable is preserved separately. These fifteen focused tests pass
after the fix. Child output is checked by assertions; a frame trace was not added.

These are supplementary contracts, not retirement of the original CLI suite.
The C++ peer uses the qualified `transport-client-v1` reference; this result does
not establish full SDK execution closure. Canonical coverage, ordinary mandatory
hooks and exact hosted CI remain required for this candidate. GUI launch, multiple
worlds, full CLI parity, backend extensions and broad P2 acceptance remain open.
See [the scoped record](../planning/reference/live-cli.json).
