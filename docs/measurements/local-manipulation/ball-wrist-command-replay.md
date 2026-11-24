<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Replay a loaded ball joint from its torque commands

The local CPU journal accepts a three-axis torque at a spherical joint in its parent joint axes. It also admits fixed clamps and the native joint-state publisher. Each recorded wrist reports its orientation, angular velocity and the sum of torques awaiting the next advancing step. A queued zero remains distinguishable from an empty queue. The replay reads all three values from the running solver. [Command journal](../../../examples/replay/ball_wrist.json) · [Loaded robot](../../../examples/worlds/gravity_wrist.sdf)

The fixture adds two commands while paused. The resulting 18, −24 and 12 N·m remained queued through another paused tick. On the first 1 ms step the commanded wrist reached angular velocity (0.095, −0.077, 0.081) rad/s and the torque was consumed. Its otherwise identical free twin reached (0.003, 0.008, −0.004) rad/s. A later reset discarded a pending zero and reproduced all 240 repeated steps, including both wrists, native contacts, pending inputs and public poses. The entire 752-record trace matched across independent processes using one and eight Bevy threads. [First run in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10023%2F) · [After reset](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10024%2F)

Before the fix, the first paused observation shifted an eccentric payload’s link origin by one floating-point increment when converting back from its center of mass. The replay now publishes its initial body, model, attached and joint poses using the ordinary Bevy publication systems before recording. The first pause preserves the observation exactly. Adding the encoder publisher to the ordinary serial transfer robot also exposed an admission regression in its replay profile. The authored robot now reopens with that publisher and its active controllers.

All four command-journal integration tests and 336 scene library tests passed. The browser checked 714 native poses across 51 frames before reset and 350 poses across 25 frames after reset. Scoped LLVM coverage reached 3/3 lines and 8/8 regions in Rapier, 3/3 and 5/5 in multifidelity, 20/20 and 21/21 in scene, and 27/28 and 59/61 in the CLI. The uncovered CLI branch reports an invalid solver binding. [Coverage and selected native readings](ball-wrist-command-replay.json) · [Browser capture](ball-wrist-command-replay.jpg)

```sh
export RUSTC_WRAPPER=
cargo build --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-local-replay
bin=target/release/gz-sim-local-replay
world=examples/worlds/gravity_wrist.sdf
journal=examples/replay/ball_wrist.json
"$bin" "$world" "$journal" --threads 1 --trace wrist.jsonl --record wrist-recording
"$bin" "$world" "$journal" --threads 8 --verify wrist.jsonl
cargo run --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  --server --replay --web-viewer 10023 wrist-recording/epoch-0
```
