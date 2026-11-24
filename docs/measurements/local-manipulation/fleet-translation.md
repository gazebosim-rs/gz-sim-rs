<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Publishing the poses of moving platforms

A model, its link and its visuals can all translate without rotating. The simulator now adds translations directly when composing two such frames and subtracts them when deriving a child’s local pose. Rotation continues through the ordinary quaternion path. This runs in the normal Bevy pose-publication systems for all physics backends.

| Moving decks | Dynamic payloads | Version | Real-time factor | Step p50 | Step p95 | Step p99 |
|---:|---:|---|---:|---:|---:|---:|
| 10,000 | 100 | Baseline | 7.10× | 1.581 ms | 1.927 ms | 2.192 ms |
| 10,000 | 100 | Translation shortcut | 7.62× | 1.375 ms | 1.629 ms | 1.663 ms |
| 100,000 | 1,000 | Baseline | 0.895× | 10.764 ms | 14.429 ms | 18.215 ms |
| 100,000 | 1,000 | Translation shortcut | 0.993× | 9.633 ms | 12.786 ms | 18.917 ms |

Each entry is the median of four interleaved runs on CPUs 24–31 of a shared workstation. Each ordinary SDF world ran 20 warmup ticks and 100 timed ticks at 10 ms. The largest run checked 404,001 public poses, 1,000 actual load contacts, the payload velocities and the IMU. Its maximum deck position error was 6.54 × 10⁻¹² m. The larger scene’s p95 improved by 11%, while its p99 rose by 4%. Absolute timings varied substantially under concurrent builds. A separate experiment combining the public-ID and pose reads did not give a reproducible benefit. The 100× full-observation target remains open. [Per-run timings, memory and hardware](fleet-translation.json) · [Native replay and independent pose checks](fleet-publication-view.md)

To repeat the mixed test from this workspace, run the command below. The benchmark also accepts 10000 for the smaller scene.

```sh
RUSTC_WRAPPER= cargo build --release --locked --no-default-features -p gz-sim-scene --example fleet_observation
taskset -c 24-31 target/release/examples/fleet_observation 100000 mixed-auto
```
