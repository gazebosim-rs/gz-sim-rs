# Replaying adaptive impacts from commands

The journal now records an entire impact even when the last native solve carries zero force. An ordered command can enable or disable an authored contact policy while the simulation is paused. Three worlds reproduced all 20 records each across fresh processes using one and eight Bevy helper threads. From reset, all four advancing states repeated. [Machine-readable results](refined-input-replay.json)

| First impact | Refined impulse | Ordinary impulse | First positive solve | Native calls |
| --- | ---: | ---: | ---: | ---: |
| Sorting pad | 40.0001 mN·s | 0 | 10 ms | 100 |
| Touching right jaw | 45.2908 mN·s | 47.4339 mN·s | 250 µs | 4 |
| Right jaw across a 350 µm gap | 41.8275 mN·s | 36.7111 mN·s | 494 µs | 500 |

The two initial epochs used refinement. In the third, the command disabled refinement before the first step and restored it afterward. The trace records accumulated normal impulse per pair, signed world-axis impulse on each requested foot, friction where available, endpoint contact, solver settings, and the time of first delivery. It keeps the warm-start history separate. Missing tangent data are encoded as null. Two Chromium/WebGPU runs checked all five display frames and 100 body poses and shapes per replay. [Watch the refined jaw](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10071%2F) · [Watch the ordinary jaw](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10072%2F)

Run from the repository root, using a new trace and output directory. The bundle contains the two SDF jaw cases and the native touching-jaw recordings. [Portable recordings and worlds](refined-input-replay.tar.xz) · [Journal](../../../examples/replay/refined_contact.json)

```sh
RUSTC_WRAPPER= cargo build --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-local-replay
bin=target/release/gz-sim-local-replay
world=examples/worlds/local_sorter_crossing.sdf
journal=examples/replay/refined_contact.json
"$bin" "$world" "$journal" --threads 1 --trace sorter.jsonl --record sorter-view --record-every 1
"$bin" "$world" "$journal" --threads 8 --verify sorter.jsonl
```

The impact observation changes covered 32/32 CLI lines and 44/44 regions, and 14/14 scene lines and 21/21 regions. The enable command covered another 7/7 CLI lines and 14/14 regions, and 3/3 scene lines and 5/5 regions. Fourteen replay integration cases and three unit cases passed. Verification applies to the same executable and host profile. [Replay contract](local-cpu-input-replay.md)
