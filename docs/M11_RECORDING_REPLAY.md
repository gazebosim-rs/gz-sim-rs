<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M11: focused local recording and replay

Implementation is in progress; this document is the bounded integration contract,
not acceptance evidence. M10 runtime is published at simulator `d3967b61` and
viewer `952a5202`. Its metadata qualification runs separately. M11 remains at most
five working days including independent review and required checks.

## Admitted delivery

Record one accepted primitive/local-glTF scene, sampled poses and committed
browser world/lifecycle controls. Replay those scene states and control outcomes
in gzweb-rs. Retain spawn/remove identities and recorded reset timestamps. This
is visualization playback, not another physics simulation or deterministic
re-execution of historical mutations. Full sensor/canonical-transport recording,
historical Gazebo log compatibility, seeking, browser file import and broader
asset formats are deferred. Existing parity tooling is not production playback.

Root owns simulator capture, archive admission, playback cursor and CLI/bridge
integration. The viewer reuses its maintained primitive Session, renderer,
scoped local asset loading and existing widgets. Agree changes to this contract
before parallel implementation. All existing live behavior remains required.

## Recording and input bounds

Record only from world startup: the initial frame has recorded epoch 0 and time
0, captured before the first physics tick even with --run. Mid-run capture and
cut traces are deferred. Replay Reset therefore retains the existing strict
epoch+1/paused/time-zero ACK rule; no live Reset guard is relaxed.

Use a new local directory containing a version-1 manifest, newline-terminated
JSON records and captured declared asset files. Use maintained serde and standard
IO. Refuse an existing output directory. Only a cleanly finalized manifest is
playable; interrupted, truncated or failed recordings are not accepted prefixes.

The recording stream is at most 64 MiB, each line including its newline at most
256 KiB, and at most 4096 frame records plus one header and footer. Captured assets
reuse M2's admitted names and limits: 128 files, 16 MiB each, 64 MiB total. They
remain owned for the whole recording/replay, across runtime alias retirement.
Capture the maintained immutable asset snapshot once at startup: aliases reuse
those same scoped routes. Playback serves the admitted immutable bytes through
the existing HTTP implementation without constructing a native world.
Revalidate each referenced glTF document with the maintained parser and its
declared scoped buffer/image closure; image pixels remain the viewer loader's
responsibility. Reject escaping paths, remote resources, absent or changed captured files before
exposing a playback scene. Do not silently download missing assets.

Each frame has a global ordinal, the exact committed Scene and model catalog,
and at most eight bounded control outcomes. Preserve the original integer
epoch/seconds/nanoseconds and source sequence. Store admitted request and actual
result as provenance; never inject old request IDs, mutations or ACKs into a
playback connection. Capture committed changes before viewer Latest coalescing.
Redundant paused normal snapshots may be omitted; outcomes and lifecycle changes
may not be omitted merely because their simulation timestamps are equal.

Recording owns one bounded eight-entry writer queue only while enabled. File and
GPU waits do not enter the simulation's critical step. Busy/IO/size exhaustion is
an explicit recording failure, with no retry of a committed physics tick or
mutation and no finalized partial trace presented as complete. Disabled recording
owns no writer, file, queue, callback or recording buffer. Shutdown finalization
and resource copying happen outside stepping.

Read typed records through bounded buffers and validate the complete archive
before publishing it. Retain immutable admitted stream bytes (at most 64 MiB),
verified immutable asset bytes (at most 64 MiB), and a bounded index/current frame.
Do not reopen mutable files after validation; outside modifications cannot change
this admitted snapshot.
Use the already locked maintained SHA-256 implementation to bind stream and
asset bytes. Mandatory nullable provenance fields must remain explicit nulls.
Unknown versions/fields, duplicate fields, inconsistent counts or identities,
nonfinite/unrepresentable geometry, bad asset references and backwards time
within a recorded epoch are errors. World identity/name stay constant; recorded
epochs never decrease and source sequences strictly increase, with gaps permitted
when redundant snapshots were omitted. Retired runtime IDs cannot be recycled;
only startup identities may return on recorded reset. Pin model-to-visual ownership and static visual definitions for each identity.
Keep the accepted rendered-pose tolerance of 1e-6 and state-response rotation
tolerance of 1e-12, with the current viewer's admitted default-world domain.

## Playback clock, flow and controls

Playback starts paused and read-only. Play/Pause affect its cursor; Reset restarts
the recording. They acknowledge actual committed playback state, never canned
success. Recorded controls are annotations and outcomes. No native physics or
sensor runtime is constructed by playback. Finished playback remains paused; Play at EOF is refused until Reset.

The existing Scene carries a monotonic playback transport epoch/sequence and the
recorded integer seconds/nanoseconds. Advance the transport epoch on restart or
recorded epoch transition; never wrap counters. Metadata retains the original
recorded epoch and source sequence. Scene.paused describes the playback cursor,
while metadata separately describes the recorded world's paused state.

Pace forward by recorded simulation-time deltas within an epoch; equal-time
operations and recorded reset still preserve ordinal order. Consumer backpressure
may slow wall-clock presentation, while encoded recorded timestamps stay intact.
Do not coalesce away a lifecycle frame: advance only after the consumer accepts
that immutable playback generation/ordinal. An old Ready cannot advance a new
generation after restart. Bind each normal credit to its immutable playback
generation and ordinal: an old Ready retires its own flight but cannot advance
the restarted cursor. Keep the accepted one-flight credit and priority reply
rules; no queue enlargement or network wait in a physical step.

## Opt-in viewer wire extension

Playback clients explicitly request `replay=1`, `lifecycle=1` and the existing
publication credit. A replay endpoint refuses unlabelled clients; a live endpoint
refuses replay mode. Omit the existing automatic scan opt-in in replay mode and
refuse any explicit scan query there; full sensor recording is excluded. Default
live clients keep their scan/protocol behavior unchanged.

Send each normal tuple in this order: Scene, read-only catalog, replay_status,
publication_credit. The catalog retains authoritative model-to-visual IDs but has
no spawn templates or editing permissions. A control commit sends its coherent
Scene/catalog/status/ACK through the existing priority path without consuming an
older normal credit. Ready is released only after the validated tuple is flushed.

`replay_status` is strict version 1 with these fields:

| Fields | Meaning |
| --- | --- |
| `world_id`, `epoch`, `scene_sequence` | Must exactly match the committed transport Scene. |
| `frame`, `frames` | One-based current ordinal and total, each within 1–4096. |
| `recorded_epoch`, `recorded_sequence` | Original source identity, distinct from transport counters. |
| `recorded_seconds`, `recorded_nanoseconds` | Exact source time; nanoseconds below one billion. |
| `recorded_paused` | Source world's paused flag. |
| `playing`, `finished` | Cursor state; finished playback is paused. |
| `outcomes` | At most eight summaries: `action`, nullable `model_id`, and boolean `ok`. |

Action names are pause, resume, reset, spawn, remove, get_state and set_state.
World-action summaries require model_id null. Successful lifecycle summaries
require a nonzero model ID within the accepted public-ID range. Failed lifecycle
summaries may retain a requested nonzero u64 model ID, including an unknown one,
or null. Protocol-invalid requests, including zero lifecycle IDs, close before
commit and are not committed outcome records; archive admission retains the
existing request decoder's refusal. Removed IDs need not remain in the catalog.

Recorded seconds/nanoseconds must equal the current Scene time, playing must be
the inverse of Scene.paused, and finished implies not playing and frame==frames.
The total frames value is immutable within one connection. Paused playback, EOF
and long recorded gaps still publish coherent credit-paced heartbeats with fresh
transport sequences within the unchanged ten-second viewer idle guard. Use literal
kind=replay_status and version=1 with all unknown/duplicate fields refused.

Use integer counters/times and explicit booleans. No arbitrary display strings,
raw recorded requests or file parser enter the viewer. A compact visible status
identifies read-only playback, recorded epoch/time, frame count and outcomes;
existing Play/Pause/Reset operate the cursor. Mutation controls stay disabled.

## Required exit evidence

One real short session must show native movement, additions/removals and reset,
then reproduce those recorded timestamps and identities in the actual viewer.
Prove real playback controls, same-time event order, restart/epoch fences, asset
lifetime, malformed/incomplete input refusal and bounded overload/teardown.
Preserve all accepted live regressions and both viewer 14 MB bundle budgets.
Complete implementation, regression source, docs and independent source review
before the consolidated required checks. No checks accompany partial edits.

## Local CLI

Replay enables the pinned maintained JSON parser's `float_roundtrip` conversion
so decoded recorded f64 poses and geometry retain their exact binary values.
The actual first capture exposed a one-ULP height change with the default parser;
that failure remains retained. Regression checks compare float bits through
archive admission, publication, heartbeat and restart without a numeric tolerance.

Capture into a directory that does not already exist, for example:

```sh
gz-sim-rs examples/m10_world_lifecycle.sdf --server --web-viewer 9011 --web-lifecycle --record-scene /tmp/new-scene-recording
```

Use the existing browser lifecycle controls during capture; normal SIGINT or a
finite `--run --iterations N` shutdown finalizes outside stepping. An interrupted,
failed or overloaded recording has no playable complete manifest. The stream is
sampled at the existing bridge publication cadence, beginning with its startup
frame even when `--run` is requested. This does not claim every physics tick or
sensor acquisition was recorded.

Replay the finalized local directory with:

```sh
gz-sim-rs /tmp/new-scene-recording --server --web-viewer 9012 --replay
```

Select the viewer's explicit replay and lifecycle opt-ins, with its WebSocket URL
pointing to port 9012. Playback starts paused; disconnect pauses the cursor and
invalidates the prior consumer's acceptance. Reconnect obtains the current frame
and a fresh connection-local publication credit. The `--server` spelling retains
the existing local-viewer CLI requirement; replay returns before SDF, native
physics, Zenoh world-server or sensor construction. It does not publish recorded
mutations back onto a live world.
