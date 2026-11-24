<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# A cylinder landing squarely on its flat end

The orange 250 g cylinder drops 145 mm onto a level slab. With the opt-in native contact correction it rebounds to a center height of **165.46 mm**, within 0.81 mm of the ideal bounce. In the ordinary run it reached 202.93 mm and started spinning at 6.72 rad/s. The corrected cylinder stays upright through ten successive landings and settles carrying 2.4525 N on the slab. The green capsule provides an independent shape control. [Watch the full drop and settling](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10078%2F) · [Original drop](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10075%2F)

| Native step | Ordinary first impulse (N·s) | Corrected first impulse (N·s) | Corrected error against 0.695757 N·s |
|---|---:|---:|---:|
| 1 ms | 0.768610 | 0.694425 | −0.191% |
| 500 µs | 0.755052 | 0.695222 | −0.077% |
| 250 µs | 0.768153 | 0.695621 | −0.020% |
| 100 µs | 0.846869 | 0.695455 | −0.043% |
| 50 µs | 0.768697 | 0.695737 | −0.003% |

The reference uses the known drop, gravity, mass and restitution. The runtime still solves the impact. On an isolated axial landing with a centered mass and a fully supported cap, the solver places one normal reaction through the cap center. At 1 ms, the ordinary four rim impulses produced 0.01132 N·m·s of angular impulse. The corrected solver recorded zero, an upward 0.694425 N·s on the cylinder, and the opposite impulse at its measured impact location on the slab. Six solver iteration budgets and both friction modes gave the same corrected impulse. At rest the solver resumes native support contacts. [Numerical receipt and per-side impulses](axial-cap.json)

The 1 ms step gave the same axial reaction after turning the cylinder upside down, rotating it about its axis, and tilting the entire drop and gravity together by 30°. Tilting only the body by 10° produced a genuine rim impact. Its impulse, every public observation and every recorded pose matched the ordinary solver. The off-edge, sideways and three low-bounce controls also matched in full. Every one of the 15 routes reproduced its trace in separate one- and eight-thread processes. A reset while delivered parcels were still present exercises removal of their cached contact entries. [Watch the tilted control](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10079%2F)

## Reproduce the drop

Run from the review checkout. The trace records the solver setting. These commands use one native CPU solver and a 1 ms world clock.

```sh
RUSTC_WRAPPER= cargo build --locked --release --no-default-features -p gz-sim-cli --bin gz-sim-rs --bin gz-sim-local-replay
world=examples/worlds/local_curved_delivery.sdf
journal=examples/replay/curved_delivery.json
target/release/gz-sim-local-replay "$world" "$journal" --rapier-axial-cap-impact --threads 1 --trace cap.jsonl --record cap-view --record-every 2
target/release/gz-sim-local-replay "$world" "$journal" --rapier-axial-cap-impact --threads 8 --verify cap.jsonl
target/release/gz-sim-rs --server --replay --web-viewer 10077 cap-view/epoch-0
```

The portable bundle includes the 15 authored world and command pairs and three complete native recordings with their meshes. To replay the full settling sequence after extracting it, pass `settle/epoch-0` to the replay server. WebGPU checked 385 frames in the standard drop, 1,508 during settling and 383 in the tilted control, including 11,338 visual poses and shapes. [Portable inputs and recordings](axial-cap-replays.tar.xz) · [Open the standard drop](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10077%2F)
