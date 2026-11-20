<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Selected M77 input-admission and finite-query correction

This isolated candidate starts at frozen M77 Source
`353dc9ae86aac56c7653f36a933d688b43fa6fbf`. Root selected these already
Source-reviewed test preconditions together with the concrete web Scene
visibility correction after the installed camera startup failure. The earlier
contingency selection and Source-review receipt remain preserved under primary
`artifacts/m77-delivery`; selection does not prove either runtime cause or fix.
The candidate changes no production transport, deadline or command semantics.
No formatter, lint, build, test, native probe, push or CI request was run for
this candidate. The three original pure helper and four new FIFO/error cases are Source declarations only.

The first exact-head hosted attempt `44693213128/1` failed the existing
`real_canonical_inputs_preserve_pause_add_clear_validation_reset_and_shutdown`
case at post-reset persistent admission. Its original log remains under primary
`artifacts/m77-delivery/hosted-44693213128-attempt1-failure/verify-original.log`.
The observed state was epoch 1, elapsed 1 ms, 15 received, 8 rejected, 6 queued,
one clear, zero ingress drops and 61,609 Clock queues. That proves no final
persistent admission, but does not distinguish transport congestion from a
routing event. The older M50 disclosure and subsequent three-publisher matching
check remain history, not a proven diagnosis. Neither this candidate nor a
successful unchanged retry would establish the first failure's cause.

The fixture now declares the same three long-lived Rust publishers with explicit
`CongestionControl::Block`. Before opening their observer session it explicitly
sets `transport/link/tx/queue/congestion_control/block/wait_before_close` to the
existing five-second bound, in microseconds. Negative/unlimited values are not
used. The original fixed messages plus key and type attachment are bounded at
1,024 bytes, comfortably within the unchanged default 65,535-byte transport batch.
The actual admission phase deadline starts before the first of its original
puts; every put and the original final semantic queue assertion share it. In
particular, additive-persistent, invalid-input and ID/name batches do not acquire
another five seconds after publishing. Each real payload is sent exactly once.
Late publication/readiness fails; no command is replayed or assertion weakened.

Pinned upstream Zenoh 1.8 Source supplies the limits of this claim:
`src/api/builders/publisher.rs` synchronously calls `Session::resolve_put`, and
`src/api/session.rs` sends the push and returns without consumer acknowledgement.
`zenoh-config` defines finite Block wait in microseconds; `zenoh-transport` maps
negative values to unlimited duration and applies a finite queue deadline before
closing a blocked transport. There is no per-put absolute-deadline API here and
no hard OS scheduling guarantee. Block completion is not a command receipt.
The unchanged Server semantic counters, numeric motion, pause/reset and shutdown
assertions remain the delivery/behavior evidence.

Before initial sends and after reset, bounded existing liveliness queries must
observe exactly one valid MS presence token for each canonical input key/type.
Post-reset nodes must all be new UUIDs from the same producer process; old tokens,
missing channels, duplicate nodes, wrong types/keys and a changed process cannot
qualify. The existing production presence is declared after its subscriber and
contains the fresh per-declaration UUID twice. Boolean publisher matching is
retained but cannot identify a generation. Its check and fresh-token discovery
share one original five-second readiness deadline; there is no second budget. No false-to-true transition is required
because discovery notifications may coalesce. Presence is generation metadata,
not delivery or callback-quiescence evidence. These are metadata queries only;
there are no extra actuator commands or replacement publisher handles.

The full `95ef6ced` normal attempt failed at `recv_deadline(...).unwrap()` on
normal `Err(Disconnected)`. This is now a confirmed helper contract bug, not a
proven cause for the earlier hosted semantic failure. The complete headless
failure batch contained that one failure; Renderer never started. Preserve
`corrected-normal-attempt1-actual-exit.json`,
`corrected-normal-attempt1-failure.json`, the full raw packet and
`refs/qualification/m77-correction-attempt1-tree`.

Pinned `zenoh-1.8.0/src/api/handlers/fifo.rs` maps actual receive-deadline expiry
to `Ok(None)` and a dropped/drained sender to `Err(Disconnected)`.
`src/api/session.rs` removes the liveliness query at `DeclareFinal`, normally
dropping the FIFO sender. The maintained `tests/private/root_clock.rs` already
requires actual disconnection and an open observer for this normal completion.
The new helper adopts that distinction, also requiring the FIFO to be empty.
The observer-open closure runs at EOF, rather than using a stale pre-wait bool.
Absolute receive deadline remains fatal and no error is swallowed generically.

The same pinned session Source emits `ReplyError("Timeout", Encoding::ZENOH_STRING)`
when the intentional at-most-100 ms query expires. Only exactly that encoding and
seven payload bytes mark a query incomplete. The helper clears every previously
collected key and the current partial key for that attempt, drains to healthy EOF,
and retries metadata only within the unchanged shared five-second readiness
horizon. Unknown errors/encodings, duplicate timeout and samples after terminal
timeout are fatal. Incomplete tokens never qualify a generation. Real puts are
never retried. This is readiness metadata, not command delivery or quiescence.

Three local tests use the actual public FIFO/Callback API without a Session or
network: queued replies drain before healthy EOF (also empty EOF), an expired
receive deadline fails, and EOF with a freshly closed observer fails. A fourth
checks exact timeout discards all partial keys and that wrong payload/encoding
panics without clearing diagnostic inputs. All four are plain `#[test]` cases
with `catch_unwind` for expected failures, retaining ordinary harness evidence.

The common test poll helper sleeps at most one millisecond, capped by remaining
original deadline, after an update. This bounds the previous yield-only churn;
it does not alter paused simulation time or production Clock policy. Clock queue
counts establish work, not actual network delivery or observed congestion.
The original three pure helper regressions cover closed token parsing, all-three fresh-node
and process identity, and deadline-clamped backoff without transport or simulation.
If Root integrates the reviewed candidate, complete the required consolidated
qualification and exact hosted/official gates before acceptance. No standalone
trial or automatic retry is authorized by this document.
