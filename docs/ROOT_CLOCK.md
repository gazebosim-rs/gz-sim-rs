<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Root clock run preparation

The candidate server adds `/clock` alongside `/world/<name>/clock`. Call
`Server::prepare_run(timeout)` at a run boundary, before stepping. Discovery and
connection refresh share one deadline and stay outside `tick`.

Construction does not claim `/clock`. Any discovered publisher, including another
message type, prevents admission. An occupied topic can be checked again at a
later run boundary. Once admitted, the publisher survives reset, later competing
publishers and failed refresh attempts. The return value reports local ownership;
it does not promise peer readiness or atomic election across simultaneous servers.

Each tick constructs one Clock. The world endpoint always receives a publication
attempt. An admitted root endpoint receives the same encoded bytes when either
endpoint's cached connection snapshot reports interest. Snapshot reads do not wait
for the worker. Separate root queue-success and failure counters expose overload;
queue acceptance does not establish wire delivery.

## Current functional evidence

An author run and a fresh independent run passed both exact Rust integration
tests against the pinned native SDK. They retain the earlier seventeen C++
functional scenarios and add root/world Clock comparison on TPE and full-f64
Rapier. Each new case requires complete serialized message equality at paused
zero, one millisecond and two milliseconds. Three rate-client lifecycle controls
remain separate from the nineteen functional cases.

Each run retained sixteen clock-client compiler receipts and one rate-client
receipt. The shared root helper builds once for both engines, and the rate helper
builds once for its five process cases. Receipt counts alone do not prove test
execution; exact successful Rust test summaries and unchanged assertions bind the
case coverage. Per-frame raw captures were not added.

The source, executable, compiler, installed SDK libraries, archive entries and
owned container/process shutdown were independently checked. A failed independent
attempt stopped on a proc-stat observer race; it remains failed. The subsequent
fresh run used a separately reviewed disappearance fix without changing simulator
code, fixtures, assertions or tolerances.

See [the scoped record](../planning/reference/root-clock.json). The upstream
`SimulationRunnerTest.Time` case still has wider assertions, including mutable
step sizes and complete frame counts; these new tests do not retire it. Current
full workspace coverage, normal publication checks and performance evidence are
separate requirements. Earlier speedup reports do not qualify the added root
publication work; the refreshed benchmark must account for both endpoints.
