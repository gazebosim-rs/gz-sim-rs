<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# A powered finger travels to both stops after a strike

The motor starts at +2 N, holding the finger against its upper stop. The incoming sphere strikes at 67 µs. At 100 µs the command changes to −2 N and the finger travels the full 40 mm. At 65.1 ms the command returns to +2 N. The finger reaches the upper stop again and finishes at rest. In gz-web-rs the blue hub turns, the orange finger slides and the red sphere rebounds. Press Play or advance one recorded frame at a time. [Watch the stroke](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10107%2F) · [Watch the contact close up](powered-slider-view.md)

| From the same initial powered state | Ordinary joints | Coupled wrist and slider |
|---|---:|---:|
| First impulse on the finger (mN·s) | 58.5320 | 72.9932 |
| Slider speed at the first impact (m/s, positive toward upper stop) | +0.219294 | 0 |
| First sample within 0.1 µm of lower stop | 36.381 ms | 34.629 ms |
| First sample within 0.1 µm of returning upper stop | 102.874 ms | 101.739 ms |
| Largest sampled travel beyond either limit | 0.3304 µm | 0 |
| Median of five first-impact p95 wall times | 12.07 µs | 14.41 µs |

The impact calculation with the finger braced predicts 72.9932 mN·s. When the wrist's mass and inertia were reduced to one quarter, the same world-space strike turned the wrist far enough that the finger moved *away* from the upper stop at 0.113 m/s. The free-slider prediction was within 0.094 µN·s. Six seated cases, including a 100-times heavier wrist and a ten-times heavier parcel, reproduced their respective predicted impulses to floating-point precision. In a second test two separate robots were struck simultaneously. Enabling the correction on one left every body state and normal impulse of the other identical across 300 steps. [Mass sweep, travel and timing samples](powered-slider-cycle.json) · [Impact equations, resolution study and limits](powered-slider-impact.md)

Each public step advances 1 µs. The command journal now accepts `set_joint_force` and reports the held generalized effort per joint. It uses newtons on a slider and newton-metres on a hinge. A paused tick retains the drive, a new command replaces it and reset clears it. After resetting, the first 100 advancing steps reproduced every original pose, velocity, input and contact signature. Independent processes using one and eight compute threads matched all 130,210 trace records. The existing six-body gripper also reproduced its 18 s shelf transfer with all 451 recording frames byte-for-byte.

The impact timings use five batches of 201 trials per representation on core 2 of a Ryzen 9 9950X3D. Each pair alternates which world goes first. Only the actual Bevy step that reports the first strike is timed. The settling approach, input submission, reaction readout and reset are outside the timer. The two-row correction applies to a single loaded frictionless contact point with one bounded slider on the reduced tree. Frictional impacts, simultaneous manifolds and multi-stop trees continue through their native joint and contact rows.

## Reproduce the full stroke

From the simulator checkout, use a fresh output path.

```sh
export RUSTC_WRAPPER=
mkdir -p target/powered-slider
cargo build --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-local-replay --bin gz-sim-rs
world=examples/worlds/local_powered_slider_impact.sdf
journal=examples/replay/powered_slider_cycle.json
target/release/gz-sim-local-replay "$world" "$journal" --threads 1 \
  --trace target/powered-slider/trace.jsonl --record target/powered-slider/capture --record-every 2000
target/release/gz-sim-local-replay "$world" "$journal" --threads 8 --verify target/powered-slider/trace.jsonl
target/release/gz-sim-rs --server --replay --web-viewer 10177 target/powered-slider/capture/epoch-0
```

The first recording follows the full stroke through 130.1 ms. The second follows the repeated strike after reset. The full-stroke viewer samples every 2 ms and includes the last pose, while the trace records every physics tick. [Open your local recording](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10177%2F) · [Portable recording and world](powered-slider-cycle.tar.xz)
