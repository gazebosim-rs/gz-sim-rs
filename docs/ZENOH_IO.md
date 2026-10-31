<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# World I/O ownership

This migration is not yet accepted. D28/D29 and
[the migration contract](ZENOH_SCHEMA_MIGRATION.md) define its scope.

The simulator uses upstream Zenoh directly. Private world-specific queues move
network work outside simulation ticking; they are not a transport API. One
world control owner retains admitted `Query` values, including reply and final
drop. CPU commands contain bounded payloads and a single-element response channel.
Admitted responses carry a plain discard counter guard, so a concurrent receiver
shutdown cannot lose failure accounting. Failed CPU admission is counted once by
the caller. Native replies and final query drops remain on IO. Rejected
queries finalize in the upstream callback, outside the simulation call graph.
Callback rejection under congestion still needs qualification.

Asynchronous and blocking removal routes use the same IO-owned query ledger.
CPU state resolves entities and preserves queue-acceptance versus deletion-result
semantics through plain Boolean reply slots. Native service declarations and final
query drops stay on the control owner. Bounded removal and shutdown ownership
have independent reviews; whole-profile acceptance remains pending.

A separate publication owner sends captured Clock, Pose_V and IMU bytes.
Congestion on publication must not prevent control processing. Queues have finite
capacity and admission failures are counted. Publication counters describe queue
admission, not remote delivery. Epoch checks cancel work that has not started
when reset commits; an operation already handed to Zenoh cannot be retracted.

IMU CPU leases contain no native declarations. Queued frames hold weak leases.
The publication owner retains publishers, matching listeners and liveliness
tokens; the final CPU lease sends only a retirement hint. A coalesced dirty flag
prevents lost hints and repeated full scans. The registry and retirement channel
are created with the first IMU requirement and released after the last one.
Rate commands similarly separate plain CPU queues from IO-owned queryables and
tokens. Force callbacks hand off only scalars; their subscribers, tokens and
callback-release witnesses also stay on the IO owner. Shutdown disables both
input families, undeclares their endpoints and retains their capture witnesses
until outstanding callbacks and queued rate requests release them. Canceled cold
setup receipts disable their declarations and follow the same retirement path. The caller
closes the session and joins both owners outside stepping. A rate-only owner also
reconciles retirement without requiring unrelated publication traffic. Bounded lifecycle changes have independent reviews; application pressure and
whole-profile qualification remain open.

## Supported TCP candidate

The application replaces upstream transport defaults with these finite settings.
Explicit settings must match this candidate; unsupported values fail before
opening a session. Endpoint, partition and namespace selection remain separate.

| Setting under `transport/link/tx` | Value |
| --- | ---: |
| `lease` | 1000 ms |
| `keep_alive` | 4 |
| `queue/congestion_control/block/wait_before_close` | 100000 µs |
| `queue/congestion_control/drop/wait_before_drop` | 0 µs |
| `queue/congestion_control/drop/max_wait_before_drop_fragments` | 1000 µs |

`.cargo/config.toml` forces `TCP_LINGER_TIMEOUT=0` when compiling
`zenoh-link-tcp`. A runtime environment value does not configure this option.
Coverage and CI evidence must bind the Cargo configuration and actual compiler
environment. Zero linger permits discarding unsent data during shutdown.

Independent stopped-peer probes demonstrated clean termination with this build
and configuration. Default close exceeded its advertised timeout; wrapping it in
another timeout returned with a live socket. The candidate is not a universal
cancellation guarantee, an equal-work performance result, or a substitute for
application shutdown and lifecycle tests.

## Remaining integration

`prepare_io` performs pending reset and declaration setup outside ticking.
`tick` refuses an unprepared reset without changing physics or publication
counters, then consumes a prepared reset snapshot without an extra advancing
step. `update` combines the phases for callers that allow preparation waits.
Application loops needing a critical ticking section call the phases separately.
This split still needs complete independent lifecycle and pressure qualification.

Complete application options/client overlays and native reference provisioning,
demand-driven schedule activation, pressure controls and strict whole-profile
coverage before removing historical runtime dependencies or declaring headless
acceptance.
