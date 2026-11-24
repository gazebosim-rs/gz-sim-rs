# Resolving the first GPU tool impact inside the actuator tick

At 64 ms the tool on the two-joint arm had 0.8381 mm of clearance. It struck the floor about 649 µs later. The GPU previously applied the impact at the end of that 1 ms tick, which left out the load that the floor carried during the rest of the tick. The solver now places one of its two internal boundaries at touchdown. The controller, Bevy clock and number of contact solves stay the same. [Numerical results](gpu-tool-impact.json) · [Watch the new gz-web-rs recording](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9939%2F)

| First loaded millisecond | Normal impulse (N·s) |
|---|---:|
| Previous GPU solve | 0.297710 |
| Updated GPU solve | 0.303190 |
| Independent Cartesian dynamics, 512 substeps from the same measured incoming state | 0.303219 |
| Rapier, 512 substeps from the same measured incoming state | 0.316147 |
| Continuous Rapier, 16 substeps from the start | 0.316379 |

The difference from the matched Cartesian result fell from 5.51 to 0.030 mN·s. The difference from the continuous 16-substep Rapier result fell from 5.90% to 4.17%. Rapier's matched 512-substep run applied its entire 0.316147 N·s on one internal contact step. The independent Cartesian solve applied 0.297294 N·s on impact and another 0.005926 N·s during the remainder. That difference between reference solvers remains. Across the full 600 ms run, the GPU's cumulative normal impulse changed from 4.11197 to 4.11787 N·s, versus 4.09326 N·s in Rapier. The maximum tool-path difference stayed at 0.840 mm, both shoulder stops still loaded at 211 and 510 ms, and all 600 GPU observations repeated byte for byte after reset.

The Rust kernel predicts the clearance at the start, midpoint and end of the public step. It brackets the crossing on the resulting quadratic and verifies the selected endpoint against the actual midpoint trajectory. Seven subdivisions on a half-millisecond interval give a 3.91 µs time bin. The solver applies contact only if the post-limit tool geometry reaches the floor. An already loaded tool retains the normal half-step schedule.

Five alternating release runs per version used an RTX 5090 with 2,048 arms. Of nine separately checked combinations of mass and friction, the worst first-impact difference against Cartesian dynamics fell from 0.929% to 0.094%, and the worst difference across the first ten loaded-window ticks fell from 1.525% to 0.183%. The native Rapier first-impact discrepancy across those nine cases increased from 7.73% to 8.01%. Every run reset and reproduced its complete device-state digest. Median p95 GPU time rose from 31.232 to 39.424 µs, and median p95 complete Bevy step time rose from 446.452 to 450.551 µs on the shared host. A first implementation that evaluated the complete geometry on every subdivision took 52.864 µs p95 on the GPU.

## Reproduce

Run the [server probe](gpu-joint-stops.md) to generate a fresh `trace.json`, then run the independent matched-state sweep with the following command.

```sh
RUSTC_WRAPPER= cargo run --release --locked -p gz-sim-scene --features gpu-batch \
    --example gpu_sdf_tool -- --landing artifacts/gpu-stop-run/trace.json artifacts/gpu-stop-run/landing.json
```

The trace supplies the published joint coordinates and velocities and the actual velocity motor demands. Each refinement starts once from rest and once from the measured 64 ms state. Both use the same public command period. The viewer checks archived transforms against positions reconstructed from the published joint angles and the authored visual offsets.
