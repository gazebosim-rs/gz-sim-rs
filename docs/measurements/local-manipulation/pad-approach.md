<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Timing the return of a rocking parcel

A block squeezed between two compliant fingers rocked back into both pads 137 µs early even when the impact itself used 1 µs physics. The settled runs differed by less than 0.000001° before the pulse. The first appreciable pitch error appeared during that single 200 µs torque pulse. A second error accumulated while the block turned on the loaded edges. We separated the two with independent runs starting from the same settled grasp. [Measured gz-web-rs comparison](pad-approach-view.md) · [Numerical results and costs](pad-approach.json)

## Where the clock drift starts

The pulse supplies 1.200 mN·m·s of angular impulse in every run. During the pulse the pads deliver a combined opposing impulse of 0.318090 mN·m·s with ordinary physics and 0.295453 mN·m·s with 1 µs physics. At 200.2 ms the ordinary run has pitched 0.268775° against 0.228027° in the fine run, yet its outgoing angular speed is already lower by 0.9202 rad/s. Its pitch crosses below the reference by 210 ms. Shortening the physics step for the pulse alone makes the impact 166 µs early. Refining the subsequent loaded motion alone puts it 26 µs late.

| Preload | Torque tick | Loaded approach | Left peak time | Left peak force |
| --- | --- | --- | ---: | ---: |
| Fine | Fine | Fine | 238.700 ms | 193.915 N |
| Ordinary | Fine | Fine | 238.700 ms | 193.914 N |
| Ordinary | Ordinary | Ordinary | 238.564 ms | 192.872 N |
| Ordinary | Fine | Ordinary | 238.534 ms | 192.560 N |
| Ordinary | Ordinary | Fine | 238.726 ms | 195.067 N |

All five diagnostic runs switch to an identical 1 µs grid at 238 ms. The shipped adaptive policy chooses the impact from geometry. Its previous peak was at 238.563 ms. The gripper commands still run every 200 µs in every case. [Matched-state regressions](../../../crates/gz-sim-scene/tests/support/pad_approach.rs)

## Selecting the physics step

The new world option `pad_rocking_steps` gives a moving, load-bearing face a separate step count. The engine uses the native torque queued on the enrolled parcel and the relative linear and angular velocities at its corners. While the parcel turns, it checks which corners of the face are compressed, separated, or about to lift inside the finger’s footprint. A queued nonzero torque on the loaded parcel or an imminent return gets the full impact budget. After the block settles the world returns to ordinary steps. Subdivision advances the local native world for the same duration as one public tick. The example uses frictionless contacts, zero gravity and eight solves per rocking tick. [Example SDF](../../../examples/worlds/local_rocking_pad_approach.sdf) · [Command journal](../../../examples/replay/refined_rocking_pad_grasp.json)

## Accuracy and work

The 200 g parcel returned at 238.678 ms against 238.700 ms continuously fine. The maximum pitch error fell from 0.21125° to 0.03250°. Left and right peak forces were 194.822 N against 193.915 N, a 0.47% difference. The previous peak-force error was 0.27%. Both normal impulses over 238–240 ms improved from 28.529 to 28.441 mN·s against 28.423 mN·s continuously fine. Angular impulse errors fell below 0.055% on both sides.

| Native solves per rocking tick | Old policy | 2 | 4 | 8 (example) | 16 | 32 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Peak timing error (µs early) | 137 | 86 | 43 | 22 | 10 | 5 |
| Native solves during 450 ms | 2,847 | 3,238 | 3,622 | 4,390 | 5,926 | 8,998 |

The example used eight solves on 192 turning ticks and 200 solves on four ticks at launch and impact. The remaining 2,054 ticks used a single solve. Continuous fine stepping used 450,000 solves. Waiting for a corner to be visibly separated missed the first moving tick after the pulse and left 46 µs of error at the eight-solve setting. Predicting the release of a compressed corner from its velocity reduced that error to 22 µs.

| Changed condition | Prior timing error | New timing error |
| --- | ---: | ---: |
| Reversed 6 N·m pulse | 137 µs | 22 µs |
| Weaker 4.7 N·m pulse | 70 µs | 12 µs |
| Stronger 7.4 N·m pulse | 874 µs | 132 µs |
| 120 g parcel, 4.8 N·m pulse | 108 µs | 15 µs |
| 320 g parcel, 7.0 N·m pulse | 137 µs | 24 µs |
| 2.5 N grip, 5.0 N·m pulse | 100 µs | 16 µs |

Each comparison uses its own continuous fine run with the same mass, inertia, drive, pulse and preload. A steady grasp selected no finer ticks. The stronger pulse tilted the parcel 27.5° and had the largest remaining timing error. The next test will put the gripper under gravity and friction to measure how the approach changes slip during a carry.

## Reproduce

```sh
export RUSTC_WRAPPER=
cargo test --locked --release --no-default-features -p gz-sim-scene --test two_finger_load \
  pad_edge_impact::approach -- --nocapture
cargo run --locked --release --no-default-features -p gz-sim-cli --bin gz-sim-local-replay -- \
  examples/worlds/local_rocking_pad_approach.sdf examples/replay/refined_rocking_pad_grasp.json
```

## Step cost and verification

Eight rotated release batches pinned to CPU 5 on a Ryzen 9 9950X3D measured each public Bevy step, including telemetry generated by the policy. The table gives the median of each batch statistic. The full trial covered 450 ms of simulated time.

| Wall time | Previous impact-only policy | Rocking and impact | Continuous fine |
| --- | ---: | ---: | ---: |
| 95th percentile per tick (µs) | 15.76 | 94.44 | 2,524.50 |
| 95th percentile after the pulse (µs) | 15.79 | 96.06 | 2,542.43 |
| Elapsed over the 450 ms trial (ms) | 38.66 | 56.85 | 5,212.44 |

The eight-solve rocking ticks had a p95 of 104.74 µs. For the four finest ticks per run, the median batch maximum was 2.31 ms. The full-run p95 rose because rocking occupied more than one in twenty ticks.

Six earlier replay fixtures reproduced all 6,702 observations. A separate off-parcel torque left the loaded gripper on ordinary steps, pausing preserved the parcel’s pending torque, and resetting during rocking reproduced the launch and its following steps. All 22 CLI replay tests and the numerical controls passed. LLVM covered 64/64 changed production lines and 125/125 regions. [Machine-readable measurements](pad-approach.json) · [Public replay regression](../../../crates/gz-sim-cli/tests/local_cpu_replay.rs)
