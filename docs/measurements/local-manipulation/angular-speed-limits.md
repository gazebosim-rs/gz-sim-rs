<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Physical angular speed limits on a world-mounted joint

A spring raises a 1 kg nut while its SDF speed limit holds rotation to 2 rad/s. The screw still transmits gravity, spring torque and axial force. A universal wrist can use a different limit for each axis, including zero. The cap acts on passive motion, commanded torque and an external impulse. [Watch the speed-limited lift in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9807%2F)

The runtime uses two unilateral native angular rows per positive capped coordinate. Their kinematic reference frames rotate at the two signed boundary rates. A phase offset keeps each row available at every solver iteration. Zero positional correction makes the reaction depend on relative velocity alone. The second wrist axis follows the child. A zero speed keeps the original coordinate fixed. Reset rebuilds just the configured rows, and removing a joint or endpoint retires its private references.

| Test with a 1 ms simulation step | Measured result |
|---|---|
| Wrist under alternating +1.2/−1.5 N·m and −1.2/+1.5 N·m | Maximum absolute rates were 0.800000 and 1.300000 rad/s. Reversal released the opposing limit. Reset replay matched exactly. |
| One external 90/9 N·m pulse for 1 ms, 0.03 kg·m² isotropic inertia | The first axis reached 0.800000 rad/s at its limit. The uncapped expected speed on axis 2 was 0.3 rad/s, and it reached 0.300000. After 150 ms of coasting the two rates were 0.800000 and 0.299998. |
| Gravity-driven screw at 1.25 rad/s, followed by a 1.5 N·m reversal | Peak magnitude 1.250294 rad/s across both directions. Helical travel error stayed below 10 µm. A zero-speed screw stayed within 1.2 × 10⁻⁸ rad/s. |
| Five interleaved release runs, 5,000 ECS steps per run | Median of per-run p95 was 10.98 µs with the two angular rows and finer native solve, and 5.73 µs with the same screw spring alone. |

An initial stop recentered on its positional boundary allowed 0.0115 rad/s excess speed on reversal. The zero-position-bias row reduced the worst excess to 0.000294 rad/s across the loaded screw test. Explicit finite limits currently require a world-mounted screw or universal joint. A speed reference must turn at most π/4 per public step. An out-of-range timestep is rejected before it can consume a queued load. Moving-parent speed limits require transmitting the governor reaction back to the moving parent.

All 33 focused tests passed, along with strict release Clippy. Changed production coverage was 208/211 executable lines (98.6%) and 342/347 regions (98.6%). The browser verified 251 recorded frames and 1,004 native visual poses and shapes through playback and rewind.

## Reproduce

Use an unused output directory. The gz-web-rs frontend is served on port 8082.

```sh
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- record examples/worlds/local_robot_screw_speed_limit.sdf \
  5000 20 target/my-speed-limited-screw
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 9807 target/my-speed-limited-screw/replay
```

[Screw test](../../../crates/gz-sim-scene/tests/local_robot_screw.rs) · [Wrist tests](../../../crates/gz-sim-scene/tests/local_robot_universal.rs) · [Authored scene](../../../examples/worlds/local_robot_screw_speed_limit.sdf)
