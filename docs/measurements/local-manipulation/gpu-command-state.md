# Driving an SDF GPU arm over Gazebo transport

A Rust client commanded two joints on a separately launched Gazebo server backed by an RTX 5090. The arm pushed its round tool into the floor, held contact, and lifted away. Both the GPU and an independently integrated Rapier arm first loaded the floor at 65 ms and last loaded it at 311 ms. [Measured results](gpu-command-state.json) [Open the replay in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9863%2F)

| Quantity | GPU | Native Rapier |
|---|---:|---:|
| Normal impulse in first loaded 1 ms interval | 0.29771 N s | 0.31638 N s |
| Normal impulse over the run | 4.26510 N s | 4.27002 N s |
| Last interval with positive normal impulse | 311 ms | 311 ms |

The first landing impulse was 5.9% lower on the GPU. The tool positions stayed within 0.536 mm over the 600 ms run. Rapier used its own joint coordinates to calculate each motor's demand, and solved its floor contact and motor torques at 62.5 µs intervals. Each solver held its velocity demand over one public 1 ms actuator tick.

## Command and state interfaces

The SDF requests ordinary Gazebo joint position controllers in velocity-command mode. Each controller accepts a relative, continuous angle in radians on its own gz.msgs.Double topic. The shoulder and elbow use `/model/arm/joint/shoulder/0/cmd_pos` and `/model/arm/joint/elbow/0/cmd_pos`. A single gz.msgs.Model message on `/world/default/model/arm/joint_state` reports both measured angles and angular velocities. The GPU profile also accepts the standard joint velocity controller on a separately owned axis. [Commanded SDF world](../../../examples/gpu_commanded_tool_world.sdf)

The client advanced a neutral first tick, sent shoulder and elbow targets of −8 and 0.12 rad while paused, then changed them to +8 and 0.47 rad before tick 301. Each physical torque was bounded by its SDF effort limit. The shoulder and elbow reached 8.0 and 2.8 N m. The largest commanded motor speeds were 2.4 and 3.2 rad/s. Contact and coupled motion briefly drove the joints to 2.759 and 5.700 rad/s. This profile applies speed bounds to the actuator demand.

Every one of the 600 transported angle and velocity pairs matched a second hardware simulation driven through the public local command interface within 2.23 × 10⁻¹⁶ in SI units. That matched simulation supplied the 1 ms contact impulses, tool gap, motor demand and torque in the trace. The transported joint-state packet carries angles and velocities. A separate real server advanced 100 ms, accepted the opposite shoulder target while paused, and reset via the Gazebo world-control service. The clock returned to zero. Its next 600 joint-state pairs and matched physics observations reproduced the fresh run byte for byte. [Visualizer verification](gpu-sdf-playback.md)

The native viewer checked all 180 visual poses and shapes through the final 601 ms frame. The client subscribed through tick 600 and supplied a separately identified, matched hardware reading for the exit frame. The qualification ran eight solver tests, ten SDF integration tests, 32 server controller tests and 51 native scene controller/publication regressions. The implementation covered 266/272 changed lines (97.79%) and 493/513 regions (96.10%). Including the external qualification clients, coverage was 609/626 lines (97.28%) and 1,140/1,230 regions (92.68%). Unexercised client regions include external I/O error paths.

## Reproduce the transport run

Select your hardware f64 Vulkan adapter and choose a new output directory. The probe starts a separate ordinary Gazebo CLI process, discovers the standard subscribers, and advances it with the ordinary world-control service.

```sh
export RUSTC_WRAPPER=
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$PWD/target}"
cargo build --release --locked -p gz-sim-cli --features gpu-batch --bin gz-sim-rs --example gpu_joint_probe
"$CARGO_TARGET_DIR/release/examples/gpu_joint_probe" "$CARGO_TARGET_DIR/release/gz-sim-rs" \
  'NVIDIA GeForce RTX 5090' artifacts/gpu-command-run
cargo run --release --locked -p gz-sim-scene --features gpu-batch --example gpu_sdf_tool -- \
  --commands artifacts/gpu-command-run/trace.json artifacts/gpu-command-run/native.json
```

Append `--after-reset` to the probe command and choose a different new output directory to exercise the reset. To open a fresh run in the viewer, launch the Gazebo CLI with the recorded archive.

```sh
"$CARGO_TARGET_DIR/release/gz-sim-rs" --replay artifacts/gpu-command-run/replay --web-viewer 9871
```

With gz-web-rs running on port 8082, [open that local recording](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9871%2F).
