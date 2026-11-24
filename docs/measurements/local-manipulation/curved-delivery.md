# Deliver a cylinder and capsule into a running robot world

The local Rust simulator can add a cylinder or capsule after the world starts, including its authored friction and restitution. In this gravity drop, the 250 g parts hit the slab 172 ms after delivery. An independently loaded world with the same parts present at startup reproduced every sampled position and quaternion and every normal impulse through 750 steps. The live viewer also spawned each shape after its source model had been removed, reused the original mesh, removed the copy and restored the originals on reset. [Measurements](curved-delivery.json)

| Local CPU solver at 1 ms | Cylinder | Capsule |
| --- | ---: | ---: |
| Authored restitution | 0.65 | 0.85 |
| First impulse | 0.76861 N·s | 0.77830 N·s |
| First impulse with zero restitution | 0.40710 N·s | 0.42183 N·s |
| First rebound center height above slab | 202.93 mm | 208.44 mm |
| First rebound height for an ideal axial impact | 166.26 mm | 209.76 mm |

The orange cylinder and green capsule drop from 250 mm. Both have a 105 mm center height when upright on the slab. The colored parts behind them are the original templates. Each run includes two deliveries, removal and reset. One and eight Bevy helper threads independently reproduced all 1,531 trace records. Each WebGPU replay checked 385 frames and 1,909 body poses and shapes. [Watch the bouncing parts](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10075%2F) · [Watch the zero-restitution control](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10076%2F)

## What the straight drop revealed

For a free body falling 145 mm under gravity, the ideal axial impulse is mass times approach speed times one plus the restitution coefficient. The capsule differs from that benchmark by 0.23% at 1 ms and under 0.01% at 50 µs. Its axis stays vertical. The cylinder’s square arrangement of four measured rim contacts delivered unequal impulses. The 1 ms result was 10.47% above the ideal impulse and gave the cylinder 6.72 rad/s of transverse spin. Between 1 ms and 5 µs the error ranged from 8.52% to 10.60%. Increasing the fixed-time solver passes from one to 32 did not improve that first reaction in either friction mode. These results describe the ordinary native solver at the recorded revision.

Run from the repository root. The trace header binds the selected friction law. Supplying a different law during verification reports a mismatch on the first line.

```sh
RUSTC_WRAPPER= cargo build --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-rs --bin gz-sim-local-replay
world=examples/worlds/local_curved_delivery.sdf
journal=examples/replay/curved_delivery.json
target/release/gz-sim-local-replay "$world" "$journal" --threads 1 --trace deliveries.jsonl --record delivery-view --record-every 2
target/release/gz-sim-local-replay "$world" "$journal" --threads 8 --verify deliveries.jsonl
target/release/gz-sim-rs --server --replay --web-viewer 10075 delivery-view/epoch-0
```

For live editing, run `target/release/gz-sim-rs --server --physics-engine multifidelity --web-viewer 10075 --web-lifecycle "$world"` on a free port and connect the live viewer. For contact diagnostics, run the native Rust example below. The portable bundle contains both native recordings and the independent startup world. [Portable controls](curved-delivery-replays.tar.xz) · [SDF](../../../examples/worlds/local_curved_delivery.sdf) · [Input journal](../../../examples/replay/curved_delivery.json)

```sh
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-scene --example cylinder_cap_rebound
```

The changed runtime code covered 49/49 lines and 41/41 regions in Rapier, 12/12 and 10/10 in the multi-fidelity backend, and 50/51 and 65/68 in the scene crate. The solver probe and its controls covered 123/125 lines and 195/203 regions. The friction-selection CLI and scene changes covered every executable line and region.
