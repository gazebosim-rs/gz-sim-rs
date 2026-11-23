# Drive the local rail robot

The SDF model has a motorized carriage on a 0.8 m rail and a passive spherical wrist. A Rust controller reads `JointSample` and calls `set_joint_force` every millisecond to send the carriage out and back. Contact with the swinging tool moves a separate parcel. The parcel’s base and rotated cap share a rigid body with an offset center of mass. [SDF model](../examples/local_robot_parcel.sdf) · [Controller](../crates/gz-sim-cli/examples/local_robot_foundations/robot.rs)

From the simulator workspace, run

```sh
export RUSTC_WRAPPER=
cargo run --release --locked --offline --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- robot target/my-parcel-run --travel 0.30
```

The target is the outbound slider position in metres. The 0.30 m and 0.25 m commands have both pushed the parcel and returned home. Use a fresh output directory for each run. The example checks contact, at least 8 cm of parcel travel, wrist-anchor separation, rail travel, and a 6 mm home tolerance throughout the final quarter-second. An unsuccessful command returns an error. A successful run writes `task.json`, `trajectory.json`, `recording.json` and a `replay/` directory.

To watch your run, start the Rust replay server

```sh
cargo run --release --locked --offline --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 9734 target/my-parcel-run/replay
```

With the local gz-web-rs frontend running on 8082, open your recording, put the pointer over the viewport, press **3** for the side view and **F** to fit the scene, then press Play. A separate frontend installation can serve the same replay by changing the frontend port in the URL and keeping the WebSocket port at 9734. [Open your recording](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9734%2F) · [Watch the measured robot](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9724%2F)

## Run each physics phase

The recording example reads the SDF physics clock. For `record`, the two integers give the total number of steps and the number between saved frames. The stop-to-stop joint test also exercises motor commands, one-sided stops and wrist rotation about each axis.

```sh
cargo build --release --locked --offline --no-default-features -p gz-sim-cli --example local_robot_foundations
target/release/examples/local_robot_foundations record examples/local_robot_contact.sdf 400 8 target/my-contact-run
target/release/examples/local_robot_foundations inertia target/my-inertia-run
target/release/examples/local_robot_foundations record examples/local_collision_course.sdf 4 1 target/my-collision-run
cargo test --release --locked --offline --no-default-features -p gz-sim-scene --test local_robot_joints
```

The qualification records describe restitution and friction configuration, inertia checks, the selection of fast bodies for continuous collision checking, and the available slider and wrist controls. [Measurements and all phase replays](../artifacts/local-robot-foundations/README.md)

## Pick up and place a free parcel

The gripper uses four prismatic joints. Two bring the fingers together, one lifts the palm, and one carries it between the stands. The controller applies forces to those joints at 1 kHz. The parcel has its own rigid body and rests on the source stand before the fingers close. [Gripper SDF](../examples/local_robot_gripper.sdf) · [Rust controller and acceptance checks](../crates/gz-sim-cli/examples/local_robot_foundations/gripper.rs)

Run the example from this workspace with a fresh output directory.

```sh
RUSTC_WRAPPER= cargo run --release --locked --offline --no-default-features \
  -p gz-sim-cli --example local_robot_foundations -- gripper target/my-gripper-run
RUSTC_WRAPPER= cargo run --release --locked --offline --no-default-features \
  -p gz-sim-cli --bin gz-sim-rs -- --server --replay --web-viewer 9736 target/my-gripper-run/replay
```

Open the replay, press **3** over the viewport for the side view, press **F** to fit, and select Play. The blue stand is the source and the green stand is the destination. The output directory contains the parcel trajectory, individual contact impulses and the result of running the same lift and rail commands with the jaws held open. [Open your gripper recording](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9736%2F)

A second control closes the jaws with zero friction on the contact surfaces. The checks require both jaws to press on the parcel before the lift, and require the parcel to stay behind in this control. The loaded-world reset repeats the ordinary grasp and compares its measured results and trajectory exactly. [Recorded gripper run](../artifacts/local-robot-gripper/gripper/task.json) · [Loaded inertia sweep](../artifacts/local-robot-gripper/inertia/README.md)

## Compare grip feedback, obstacle clearance and solver switching

Build the Rust study and replay server once. Run each study with a fresh output path. The studies read the SDF fixtures from this checkout and return an error if their physical acceptance checks fail.

```sh
export RUSTC_WRAPPER=
cargo build --release --locked -p gz-sim-cli --no-default-features --example local_robot_foundations --bin gz-sim-rs
target/release/examples/local_robot_foundations grasp-envelope target/my-grasp-envelope
target/release/examples/local_robot_foundations arm target/my-arm
target/release/examples/local_robot_foundations arm-recovery target/my-arm-recovery
target/release/examples/local_robot_foundations arm-switch target/my-arm-switch
target/release/examples/local_robot_foundations arm-mid-swing target/my-arm-mid-swing
```

| Study | What to inspect | Recording directory |
| --- | --- | --- |
| Grasp feedback | Compare tightened and fixed jaw pressure for the same load. Check the independent slip measurement and where the parcel comes to rest. | `target/my-grasp-envelope/recovery/replay` |
| Feedback on the rotary arm | Inspect the recovered 450 g transfer, its fixed-pressure control, and the overloaded return to the source stand. | `target/my-arm-recovery/recovery/replay`, `fixed/replay` and `overload/replay` under the same output |
| Mid-swing slip | A 6 N downward pull lasts 65 ms after a 200 g parcel clears both stands. Compare the early return, the late set-down and the early fixed-pressure control. | `target/my-arm-mid-swing/source/replay`, `target/my-arm-mid-swing/target/replay`, `target/my-arm-mid-swing/fixed/replay` |
| Rotary arm | Compare the lifted path over the obstacle with the lower path that strikes it. | `target/my-arm/clear/replay` and `target/my-arm/blocked/replay` |
| Solver switching | Compare the arm that parks at rest with the always-dynamic arm, including both finger impulses and the parcel trajectory. | `target/my-arm-switch/switched/replay` and `target/my-arm-switch/reference/replay` |

If you need to start the local frontend, build the sibling gzweb-rs checkout and serve it in a separate terminal. The frontend and replay server use different ports.

```sh
cd ../gzweb-rs
RUSTC_WRAPPER= ./build.sh
python3 -m http.server 8082 --bind 127.0.0.1
```

To watch any recording, pass its directory to the replay server. With gz-web-rs running on port 8082, open the viewer and press Play. You can step through individual frames around pickup or release.

```sh
target/release/gz-sim-rs --server --replay --web-viewer 9744 target/my-arm/clear/replay
```

[Open the rotary arm in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9744%2F)

Measured results from the review run are available for the [grasp trials](measurements/local-manipulation/arm-grasp.json), [obstacle routes](measurements/local-manipulation/arm-routes.json), [inertia check](measurements/local-manipulation/arm-inertia.json), [parking comparison](measurements/local-manipulation/parking.json) and [coverage checks](measurements/local-manipulation/coverage.json).


The mid-swing command writes six trials to `comparison.json`. Each disturbance has a fixed-pressure control and an undisturbed carry from the same incoming body states. The reported drop is the parcel's downward travel relative to the palm during the pull and the next 100 ms. Only the three recorded trials have frame indices. The frame-index file identifies the alarm, the decision to turn toward the nearer stand and the first supported release. At the release frame the stand bears the load, and the next control interval starts opening the jaws.

With the recording served on port 9750, open the [source-return replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9750%2F). The local review also serves the [target set-down](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9751%2F) and the [fixed-pressure control](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9752%2F). To serve a freshly generated recording yourself, use a free port in the replay server command above and point it at the corresponding replay directory.

## Lift a flexible cable

The same four-axis arm pinches the middle of a free, 22 g cable, lifts it from the bench, holds it for 300 ms, then lowers and releases it. An open-jaw control follows the same lift command. Both replays show the eleven cable particles sampled from the solver. The orange spheres overlap to represent the cable thickness.

Run from the simulator workspace with a fresh output directory. The first command writes the measurements and replays. The second serves the closed-jaw replay on port 9760.

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- arm-cable target/my-arm-cable
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 9760 target/my-arm-cable/held/replay
```

With gz-web-rs running on port 8082, open your replay and press Play. For the control, point the server at `target/my-arm-cable/open/replay`. The local review serves both recordings. [Open your replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9760%2F) · [Recorded cable lift](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9754%2F) · [Open-jaw control](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9755%2F)

The SDF deformables plugin binds each centered jaw collision using `<box link="default::arm::left" coupling="articulated_wrench" normal_compliance="0.001"/>`, with a corresponding entry for the right jaw. Compliance gives the indentation in metres under one newton at a single particle contact. Zero selects hard contact. During each 1 ms arm tick, the material follows the measured jaw motion, then feeds each jaw’s force and torque into the next advancing rigid solve. The material clock can run faster while the actuator clock stays fixed.

The comparison reports average height gain of the cable’s center of mass, each tip’s drop below the middle particle, vertical support and squeezing force on each side, bench load before and after release, and step cost. It repeats the grasp at three material step sizes and reverses the jaw declaration order at the middle setting. [Measured trials](measurements/local-manipulation/arm-cable.json)
