<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Flat and rounded ends reach the ground at different times

Two tilted 250 g parts fall from the same height. Their overall dimensions match. The orange cylinder hits the table on its rim at 205 ms. The green capsule touches with its rounded end at 213 ms. Geometry and gravity alone predict 205.216 and 213.241 ms, respectively. Both predictions fall within one 1 ms solver tick of the first measured upward impulse. [Watch both parts land in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10034%2F)

Both centers start 320 mm above the floor. Each part measures 210 mm end to end and has a 45 mm radius with its axis tilted 30° from vertical. The capsule has a 120 mm straight barrel between its hemispheres. The SDF gives each part the inertia of its actual homogeneous solid. For half-length *a* and radius *r*, the cylinder's vertical extent below its center is *a* cos θ + *r* sin θ. For the capsule it is half its barrel length times cos θ plus *r*. [Authored solids and inertias](../../../examples/worlds/local_curved_drop.sdf)

| Native reading | Cylinder | Capsule |
|---|---:|---:|
| First upward impulse | 0.30604 N·s | 0.21978 N·s |
| Upward force over the final 1 ms | 2.45275 N | 2.45251 N |
| Weight | 2.45250 N | 2.45250 N |

The CPU command journal admits cylinder and capsule collision and visual geometry. Its offline recorder bundles the same generated meshes that the live viewer uses. Each replay epoch includes its own immutable asset declarations, sizes and hashes. The capsule mesh has a cylindrical barrel, 4,096 outward-facing triangles, analytical vertex normals and two hemispherical ends. Its identity display scale preserves the authored dimensions. The browser checked all 183 displayed poses and shapes across 61 frames, including rewind. [Portable native recording](curved-body-replay.tar.xz) · [Screenshot](curved-body-replay.jpg)

The 1,203 profile and observation records matched in separate one- and eight-thread processes. Reset reproduced the full 600-step physical trajectory, the contact forces and both mesh files. Six journal integration tests passed. Combined changed-source LLVM coverage reached 70/70 lines and 127/128 regions in assets, 97/97 and 180/186 in the CLI, and 4/4 and 5/5 in scene. The separate mesh qualification covered all of its changed lines and regions in both affected crates. [Numerical, browser and coverage receipts](curved-body-replay.json)

```sh
export RUSTC_WRAPPER=
cargo build --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-local-replay
target/release/gz-sim-local-replay examples/worlds/local_curved_drop.sdf \
  examples/replay/curved_drop.json --trace curved.jsonl --record curved-recording
cargo run --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  --server --replay --web-viewer 10034 curved-recording/epoch-0
```
