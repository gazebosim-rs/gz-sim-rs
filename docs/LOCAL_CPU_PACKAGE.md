<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Run the local CPU physics preview on Linux x86-64

Extract the archive in an empty directory. The simulation runs locally on the CPU. The source revision and build toolchain are recorded in BUILD-INFO.

```sh
sha256sum --quiet -c SHA256SUMS
./bin/gz-sim-rs share/gz-sim/upstream_joint_force.sdf --physics-engine multifidelity --iterations 250
./bin/gz-sim-local-replay share/gz-sim/upstream_joint_force.sdf share/gz-sim/upstream_joint_force.json --threads 1 --trace motor.jsonl
./bin/gz-sim-local-replay share/gz-sim/upstream_joint_force.sdf share/gz-sim/upstream_joint_force.json --threads 8 --verify motor.jsonl
```

To operate the rotor, start the server and leave it running. In a second terminal, check its current state and step once.

```sh
./bin/gz-sim-rs share/gz-sim/upstream_joint_force.sdf --physics-engine multifidelity --server --web-viewer 9003
```

```sh
./bin/gz-sim-control --port 9003 status
./bin/gz-sim-control --port 9003 step 0 0
```

If the connection drops before a reply, reconnect with `status`. A status at epoch 0 and 0 ms is the last observed state. An earlier request may still be in flight. If you want one step from that state, retry exactly `step 0 0`. The server refuses that same request once the clock advances. If it refuses, check status again. In this paused, single-operator, 1 ms world, epoch 0 at 1 ms confirms the interval happened. With other operators, the clock tells you the current state and cannot identify who advanced it. A reset changes the epoch. Inspect the robot before choosing a new command.

To watch the 200 g gripper lift, serve the packaged recording and open it in a running gz-web-rs frontend. Press Play and use Reset to rewind. [Open the recording on port 9002](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9002%2F)

```sh
./bin/gz-sim-rs share/gz-sim/gripper-replay --server --replay --web-viewer 9002
```

To run the stock double pendulum, use the qualified 120 Hz fixed-body contact setting. To watch it in gz-web-rs, run the second command and open the same port-9002 viewer link above. Stop any other replay server on that port first.

```sh
./bin/gz-sim-rs share/gz-sim/upstream_double_pendulum.sdf --physics-engine multifidelity --stiff-static-contacts --iterations 6000
./bin/gz-sim-rs share/gz-sim/pendulum-view --server --replay --web-viewer 9002
```

The pendulum is Gazebo’s Apache-2.0 `test/worlds/double_pendulum.sdf` at the same revision as the rotor. The bundled recording uses viewer lighting. [Angle, period and energy measurements](https://github.com/gazebosim-rs/gz-sim-rs/blob/review/local-cpu-mvp-20261124/docs/measurements/local-manipulation/double-pendulum.md)

The rotor is Gazebo’s Apache-2.0 `test/worlds/apply_joint_force.sdf` from commit `9e3b78f5de297def8fe47fd189500fb5cd6538ef`. The packaged gripper recording comes from the measured switched run on the simulator review branch. [Source, viewer setup and qualification](https://github.com/gazebosim-rs/gz-sim-rs/blob/review/local-cpu-mvp-20261124/docs/LOCAL_CPU_MVP.md)

Run the stock differential-drive robot through a straight line, a left turn and a stop. Compare the recorded trajectory with an eight-thread replay using the same friction setting.

```sh
./bin/gz-sim-local-replay share/gz-sim/upstream_diff_drive.sdf share/gz-sim/upstream_diff_drive.json --sdf-default-friction --threads 1 --trace drive.jsonl
./bin/gz-sim-local-replay share/gz-sim/upstream_diff_drive.sdf share/gz-sim/upstream_diff_drive.json --sdf-default-friction --threads 8 --verify drive.jsonl
./bin/gz-sim-rs share/gz-sim/drive-view --server --replay --web-viewer 11559
```

The browser needs support for named worlds. The viewer instructions include the tested source revision, its small compatibility patch and framing controls for the vehicle. [Open the drive replay and reproduce the viewer](https://github.com/gazebosim-rs/gz-sim-rs/blob/review/local-cpu-mvp-20261124/docs/measurements/local-manipulation/drive-view.md)

## Contact and collision course

Running each journal writes the measured link poses and contact impulses to a fresh file. The first run takes a foot past a finite edge, the second removes the right pad while paused, and the third sends three fast links past thin obstacles.

```sh
data=share/gz-sim
./bin/gz-sim-local-replay "$data/multifidelity_rocking_foot.sdf" "$data/course_foot_edge.json" --trace edge.jsonl
./bin/gz-sim-local-replay "$data/multifidelity_two_support_foot.sdf" "$data/course_paused_support.json" --trace paused.jsonl
./bin/gz-sim-local-replay "$data/local_fast_obstacle.sdf" "$data/local_fast_obstacle.json" --trace fast.jsonl
./bin/gz-sim-rs "$data/contact-course-view/paused" --server --replay --web-viewer 11561
```

Select `edge`, `paused` or `fast` under `share/gz-sim/contact-course-view` to serve that recording. [Watch the course in gz-web-rs](https://github.com/gazebosim-rs/gz-sim-rs/blob/review/local-cpu-mvp-20261124/docs/measurements/local-manipulation/contact-course-view.md)

## Controlled arm, gripper and joint checks

Run the Gazebo position-controller world through three angle targets. Run the gripper with a 1 kg parcel and repeat with a slippery 200 g parcel. Use fresh recording directories. Each gripper run writes a JSON outcome and records the motion for gz-web-rs.

```sh
data=share/gz-sim
./bin/gz-sim-local-replay "$data/upstream_joint_position.sdf" "$data/upstream_joint_position.json" --trace position.jsonl
./bin/serial_transfer clear-obstacle placed --parcel-kg 1 --grip-friction 0.85 > placed.json
./bin/serial_transfer clear-obstacle rejected --parcel-kg 0.2 --grip-friction 0 > rejected.json
./bin/loaded_joint_chain > joint-checks.json
./bin/gz-sim-rs "$data/controlled-arm-view/placed" --server --replay --web-viewer 11563
```

The joint check reports anchor drift and motion with a half-kilogram tool on a chain containing all six joint types. Each gripper recording can also be served directly by passing its output directory to the replay server. [Open the placement in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11563%2F)

## Automatic tier switching after placement

These two commands run the same 1 kg pick-and-place task. After delivery the empty pickup tray is removed and the arm returns to the cleared station. Once the unloaded arm settles, the adaptive run can park it until the next command. Each recording directory and output file should be fresh.

```sh
./bin/serial_transfer clear-obstacle dynamic --fidelity dynamic --parcel-kg 1 --grip-friction 0.85 \
  --threads 1 --handoff-trace dynamic.jsonl --timings dynamic-cost.json > dynamic-outcome.json
./bin/serial_transfer clear-obstacle adaptive --fidelity adaptive --parcel-kg 1 --grip-friction 0.85 \
  --threads 1 --handoff-reference dynamic.jsonl --timings adaptive-cost.json > adaptive-outcome.json
./bin/gz-sim-rs share/gz-sim/adaptive-arm-view/adaptive --server --replay --web-viewer 11566
```

To test loss of support, repeat both commands with `--remove-receiver-during-standby` and separate output paths for the recordings, logs and reference trace. This removes the shelf supporting the delivered parcel while the simulation clock is paused. To repeat the handoff stress test, run `./bin/serial_switch_soak soak.json --threads 1`. [Watch the adaptive run in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11566%2F)
