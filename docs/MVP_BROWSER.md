<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Primitive browser connection (M1 source)

This bounded application connection supplies gzweb-rs with committed Rapier
visual poses. It is not a replacement transport library or evidence of C++ scene
service compatibility. Existing simulator pub/sub/services still use upstream
Zenoh directly. No worker, endpoint or snapshot allocation is created unless
`--server --web-viewer PORT` is explicitly selected.

The source is under development. Development CLI builds passed, including the
Rapier-only configuration. The maintained viewer now has fresh WebGL2 observations on the executor 1.14.1
graph: all 20 bodies, 21 truthful visual rows, real motion, committed
pause/reset/resume, fresh advancing reconnect and clean shutdown. See
[the saved frames and bindings](evidence/m1-browser/README.md). Reload briefly
showed a closed connection before automatic recovery; stale geometry was not
presented as connected. Both current WebGPU/GL bundles pass the unchanged payload
budgets. M1 is now accepted after current-graph staged qualification, exact hosted CI
and independent evidence review; see [M1_ACCEPTANCE.md](M1_ACCEPTANCE.md). Earlier bundles and prototype observations
remain historical. See DELIVERY_PLAN.md.

A portable record of the current actual browser frames is in
[evidence/m1-browser](evidence/m1-browser/README.md).

## Launch contract

Build the CLI's selected no-native profile (`cargo build -p gz-sim-cli
--no-default-features`) and start it with:

```sh
target/debug/gz-sim-rs examples/mvp_primitives.sdf --server --run --web-viewer 9002
```

Use the maintained `gazebosim-rs/gzweb-rs` checkout at `../gzweb-rs`, with its
explicit primitive backend (`?backend=primitive&ui=full` for full controls), and serve its built
WASM app on loopback HTTP port 8080 or 8081. The bridge socket is
`ws://127.0.0.1:9002/`. Browser graphics require an available supported backend;
WebGPU is preferred and WebGL2 support is being checked against the pinned
viewer dependencies. No renderer, GPU or native physics SDK is required by the
selected server runtime. Default full builds retain native backends and tests.

## Narrow JSON version 1

All frames have `version:1` and a `kind`. Scene frames are complete replacements,
not deltas: `kind:scene`, `world`, `world_id`, `epoch`, `seconds`, `nanoseconds`,
`sequence`, `paused`, and `visuals`. Each visual contains stable `id`, `name`,
world `translation:[x,y,z]`, quaternion `rotation:[x,y,z,w]`, linear `color:[r,g,b,a]`,
and `shape:{kind:box,size:[x,y,z]}` (full dimensions), or
`shape:{kind:sphere,radius:r}`. Authoritative values use f64 SI metres in Gazebo
Z-up; the viewer alone derives f32 Y-up transforms. Lighting is a viewer aid;
SDF light lowering, complex materials and physical image parity are not in M1.
Unsupported geometry or invalid visual metadata fails explicitly.

The server keeps one latest complete scene, at most 128 visuals/64 KiB encoded,
with names/world name at most 256 UTF-8 bytes. Publication is limited to 30 Hz,
except command completion. No snapshot work occurs without a connected viewer
once the initial frame has been admitted. A connection generation gates the first frame until the simulation thread has
captured a new committed scene after that connection. Reconnect receives a full snapshot;
the client clears disconnected state and rejects stale epochs/sequences.

Controls contain only `version:1`, `kind:control`, a nonzero strictly increasing
`request_id` per connection and `action:pause|resume|reset`. One outstanding
command is permitted. Reset is reset-all and paused; resume is separate. An
`ack` contains matching request ID, `ok`, resulting `epoch/seconds/nanoseconds`,
`paused`, and nullable `error`. It is emitted only after the application calls
Server control, `prepare_io` and `tick`, including actual reset reconstruction.
It does not change the existing Zenoh endpoint's acceptance semantics.

## Bounds and shutdown

The listener binds IPv4 loopback and serves one client. Browser Origin must be
loopback HTTP at `127.0.0.1` or `localhost`, on port 8080 or 8081; native local clients may omit
Origin. A second client is closed; clients can reconnect after disconnection.
Incoming frames/messages are limited to 4 KiB, command handoff to eight entries,
ack handoff to one, socket output buffering to 128 KiB. Invalid, reordered,
multiple-inflight, binary or overloaded input closes the connection. Valid
server-state refusal receives `ok:false`. A command lost during disconnection
has unknown delivery; reconnect and inspect authoritative state rather than
blindly retrying. Queued commands from already disconnected peers are discarded.

Handshake has a 250 ms absolute deadline using resumable nonblocking library
handshakes. Established reads wait at most 2 ms and writes 100 ms; slow clients
close without holding the physics tick. Shutdown stops/joins the network worker
outside stepping. No WebSocket codec, cryptography or networking framework is
implemented here; the maintained tungstenite library owns framing/handshakes.

## Real browser acceptance still required

Use the maintained viewer's explicit primitive backend. Preserve its default
Gazebo connection behavior and existing tests. Complete this checklist only
against the built application and actual running Rapier server:

1. Start the example paused (omit `--run`), then connect the browser. Confirm
   the static ground and all 20 bodies are visible, including non-cubic boxes
   with their admitted rotations and colors. No saved animation qualifies.
2. Play and observe actual changing simulation time and motion of all 20 bodies;
   the ground stays fixed. Pause and confirm both time and poses remain fixed.
   Controls reflect committed server acknowledgements, not a local prediction.
3. Reset and confirm a paused zero-time world, initial poses and stable IDs;
   play again and observe new Rapier motion. A failed/superseded control must
   report refusal rather than displaying success.
4. Disconnect/reconnect and confirm stale geometry is cleared while disconnected
   and the first newly admitted frame reflects current server state. Repeat
   after the server has advanced without a viewer; do not reuse an old snapshot.
5. Confirm unsupported MVP actions are visibly unavailable or explicitly refused.
   Unreported real-time factor, iteration count and step metrics show unavailable,
   while authoritative pause state and simulation time remain visible.
6. Stop the server cleanly, confirm visible disconnection and listener release,
   and follow the documented clean-start procedure again.

Retain the application revisions, build commands, browser/backend identity,
real observations and screenshots with the final evidence. A build success,
protocol-only test, old prototype probe or unavailable graphics adapter does not
satisfy this checklist. Run the required consolidated source/coverage/staged/CI
checks after the reviewed bounded feature is complete.
