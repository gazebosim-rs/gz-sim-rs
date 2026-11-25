# Watching the stock Gazebo double pendulums

Both stands start at rest, five metres apart. Gravity swings their two hinged links through six seconds of simulation. The center of each lower rod starts 1.892 m above the plane, passes through 0.716 m at 0.80 s and reaches 1.624 m on the other side at 1.60 s. [Watch the six-second clip](pendulum-view.mp4) · [Open the interactive gz-web-rs replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=high&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11557%2F)

The interactive viewer initially fits the 100 m ground plane. Select the third visual row under **default** in the scene tree, the first stand’s **vis_pole**. Move the pointer into the viewport, press **1**, then **F**, and scroll down to fit the swing. Press **Play** in the top toolbar. The clip shows both stands from an oblique angle.

The local CPU solver loaded the original Gazebo SDF at its authored 1 ms step. The journal advances 6,000 steps with gravity and enables no drives. The run used the 120 Hz fixed-body contact setting. The archive samples every 40 ms and includes all 15 SDF visuals. The viewer supplies the scene lighting because its archive format does not carry Gazebo lights. The recorder’s explicit `--viewer-lighting` flag omits the sun from the display snapshot. Physics still loads the complete SDF. Live viewing and ordinary recordings retain strict light admission. [Stock SDF](../../../examples/worlds/upstream_double_pendulum.sdf) · [Step journal](../../../examples/replay/upstream_double_pendulum.json)

Chromium played all 151 frames in gz-web-rs with WebGPU, matched 2,265 visual poses and shapes and 151 model catalogs to the archive, and rewound to zero. An independent eight-thread execution reproduced all 6,002 trace records from the one-thread recording. [Download the replay archive](pendulum-view.tar.xz)

## Reproduce the recording

Run the following from the repository root with fresh output paths. Both commands use the same release executable.

```sh
cargo build --release --locked --no-default-features -p gz-sim-cli \
  --bin gz-sim-local-replay --bin gz-sim-rs
world=examples/worlds/upstream_double_pendulum.sdf
journal=examples/replay/upstream_double_pendulum.json
bin=target/release/gz-sim-local-replay
"$bin" "$world" "$journal" --threads 1 --stiff-static-contacts \
  --record target/pendulum-record --record-every 40 --viewer-lighting \
  --trace target/pendulum-trace.jsonl
"$bin" "$world" "$journal" --threads 8 --stiff-static-contacts \
  --verify target/pendulum-trace.jsonl
```

With the gz-web-rs frontend running on port 8082, start a replay endpoint on a free port. The command below uses port 11558. [Open this replay endpoint](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=high&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11558%2F)

```sh
target/release/gz-sim-rs --server --replay --web-viewer 11558 \
  target/pendulum-record/epoch-0
```

To use the downloaded archive directly, extract `pendulum-view.tar.xz` and pass its `pendulum-view` directory to the server. The archive’s SHA-256 is `47f15ccc755e3309ceeac85d61faea759ef4446dc0d4328d8ef60950d845572a`.
