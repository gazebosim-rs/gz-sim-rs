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

If the connection drops before a reply, reconnect with `status`. For this paused, single-operator, 1 ms world, epoch 0 at 0 ms means the first step is still available. Epoch 0 at 1 ms means it already happened. With other operators, the clock tells you the current state and cannot identify who advanced it. A reset changes the epoch. Inspect the robot before choosing a new command.

To watch the 200 g gripper lift, serve the packaged recording and open it in a running gz-web-rs frontend. Press Play and use Reset to rewind. [Open the recording on port 9002](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9002%2F)

```sh
./bin/gz-sim-rs share/gz-sim/gripper-replay --server --replay --web-viewer 9002
```

The rotor is Gazebo’s Apache-2.0 `test/worlds/apply_joint_force.sdf` from commit `9e3b78f5de297def8fe47fd189500fb5cd6538ef`. The packaged gripper recording comes from the measured switched run on the simulator review branch. [Source, viewer setup and qualification](https://github.com/gazebosim-rs/gz-sim-rs/blob/review/local-cpu-mvp-20261124/docs/LOCAL_CPU_MVP.md)
