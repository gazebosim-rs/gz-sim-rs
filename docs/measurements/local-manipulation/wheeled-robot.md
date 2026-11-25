<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# A stock Gazebo robot drives, turns and stops

The Rust simulator and native Gazebo drove the same two-wheel vehicle through a 5.5-second course. The chassis stayed within 0.32 mm of native Gazebo’s position at all 551 sampled times. Both vehicles stopped 0.39 s after the brake command. Each wheel is driven by an effort-limited joint velocity constraint, and contact between the spherical wheels and the ground moves the vehicle. The third wheel rolls freely on a ball joint. [Watch the route and reproduce it in gz-web-rs](drive-view.md) · [Upstream Gazebo SDF](https://github.com/gazebosim/gz-sim/blob/9e3b78f5de297def8fe47fd189500fb5cd6538ef/test/worlds/diff_drive.sdf)

## Run the course

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --bin gz-sim-local-replay -- examples/worlds/upstream_diff_drive.sdf \
  examples/replay/upstream_diff_drive.json --sdf-default-friction --threads 1 --trace drive.jsonl
```

The input journal holds the vehicle at rest for 0.5 s, commands 0.5 m/s straight ahead for 2 s, requests 0.35 m/s with a left turn at 0.8 rad/s for 2 s, then brakes for 1 s. Linear and angular acceleration limits apply at the original 1 ms step. The local drive prepares each wheel target for the following solve as Gazebo does. The wheel readings come from the bearings and the trajectory comes from the chassis and wheel link poses. The live server also published wheel and caster coordinates in 16 consecutive tick-stamped Gazebo messages. [Command journal](../../../examples/replay/upstream_diff_drive.json) · [Native DART capture and metric definitions](../../../tools/physics-audit/drive-native/README.md)

## What changed the result

| Largest difference from native Gazebo | Original soft motor | Velocity constraint | Velocity constraint with SDF friction | Acceptance limit |
|---|---:|---:|---:|---:|
| Horizontal chassis position | 1,581 mm | 80.49 mm | 0.316 mm | 100 mm |
| Chassis heading | 1.4671 rad | 0.0880 rad | 0.000403 rad | 0.10 rad |
| Left wheel angle | 2.5792 rad | 0.000635 rad | 0.000647 rad | 0.15 rad |
| Right wheel angle | 6.7425 rad | 0.003442 rad | 0.003456 rad | 0.15 rad |
| Distance traveled after braking | 130.41 mm | 43.95 mm | 0.020 mm | 30 mm |
| Sideways speed at the drive axle | 0.143 mm/s | 43.47 mm/s | 0.100 mm/s | 50 mm/s |

The soft motor took too long to spin up. With the velocity constraint the wheels reached the commanded speed, but the chassis kept sliding after the wheels stopped. The stock SDF omits the contact surfaces, whose decoded friction coefficient is 1.0. The legacy local profile uses Rapier’s collider default of 0.5. Selecting `--sdf-default-friction` on the original SDF gave a braking distance of 61.414 mm against DART’s 61.395 mm. The journal records that profile for verification. [Three measured Rust runs, native traces, packet, checksums and standalone Rust comparator](drive-reference.tar.gz)

After the straight leg the chassis had moved 874.56 mm. Lowering friction to 0.001 reduced that to 10.34 mm. On a frictionless surface it moved 0.0052 mm. With the chassis on a stand both wheels spun at least 2.90 radians, kept at least 198 mm of clearance and delivered zero impulse to the floor. Reset during the turn reproduced all 5,504 observations, and one and eight Bevy threads matched all 5,505 journal records. [Control runs and browser replay](drive-view.md)

## Test coverage

Focused LLVM coverage of changed executable production lines and regions exceeded 95% in each affected crate. Scene reached 191/199 lines and 402/419 regions, server reached 31/31 and 40/40, and CLI reached 63/64 and 118/123. The combined export reported one function with mismatched profiling data. The focused tests exercised both motor profiles, stock wire publication, clone and reset, invalid commands, trace mismatch handling and browser archive playback. The existing whole-repository release gate remains open. [Release criteria and remaining coverage](../../PHYSICS_RELEASE_ROADMAP.md)
