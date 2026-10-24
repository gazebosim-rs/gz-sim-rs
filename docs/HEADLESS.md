# Headless integration

IO-01 is in progress. The first bridge joins an SDF/Bevy/Rapier world to independent
Rust message and transport libraries. It owns `/world/<name>/clock` and
`/world/<name>/control`; root `/clock` publisher election remains open.

The coordinator owns the server crate, core preflight accessors, scene capability
checks, native client, reference build and integration CI. Source contracts come
from pinned gz-sim SimulationRunner.cc (UpdateCurrentInfo, PublishStats,
OnWorldControl and ProcessWorldControl). These are new conformance tests; no
original simulator case is retired.

Clock packets contain simulation time, measured active real time excluding pauses,
and the actual system epoch. Clock publication uses a bounded nonblocking queue.
The server counts queued clocks and publication failures; transport exposes its
own queue/native error counters. Queuing does not prove subscriber delivery.

Each tick advances at most one physics step, publishes, then polls a configured
finite number of control requests. Pause is always applied. A nonzero multi_step
overrides step; stepping adds pending iterations only when pause=true. Pending
iterations pause the server when they reach zero. An intervening pause preserves
pending iterations, matching the native control order. Requests are preflighted
before changing state. Replies acknowledge acceptance, not execution completion.

This initial control subset rejects seed changes, run_to_sim_time and nonempty
reset requests with a successful service reply containing Boolean.data=false.
Native support/ignored-field behavior for those requests remains a separate
compatibility task; the bridge does not silently reinterpret them. Clock range,
malformed messages, bounded polling, rejected requests and shutdown are tested.

The C++ client waits for an actual clock packet before requesting three steps,
checks the exact resulting timestamp and verifies unsupported-control rejection.
Its Rust host loads the real fall.sdf fixture and checks that the body moves.
Network setup, host pacing and worker joining happen outside simulation steps.
Native client libraries are built from planning/transport-reference-lock.json.
Run `python3 scripts/bootstrap-native.py`, then `python3 scripts/verify.py`.
An existing identical reference can be selected with GZ_REFERENCE_PREFIX; source
configuration, library hashes, pkg-config prefix and dynamic linkage are checked.

Remaining first-milestone work includes IMU metadata/publication, entity hierarchy
and pose publication, reset/removal, joints and forces, TPE, local/Fuel resources,
sensor scheduling/noise and the equal-work C++ comparison/performance report.
The full simulator system and backend plans remain unchanged.
