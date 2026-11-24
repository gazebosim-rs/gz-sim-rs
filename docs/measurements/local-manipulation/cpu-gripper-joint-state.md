# Reading the CPU gripper while it carries a parcel

The local CPU robot publishes its measured rail and two jaw displacements in metres and their velocities in metres per second at 100 Hz on `/world/default/model/gripper/joint_state`. The SDF uses the standard Gazebo JointStatePublisher and names the three joints to observe. Its universal wrist and screw spindle continue driving the same transfer. The selected joint IDs and values matched the native physics state, including after reloading the original world. A real Zenoh subscriber received the standard `gz.msgs.Model` packet and checked the joint types, timestamp, positions and velocities. [Source workcell](../../../examples/worlds/local_robot_serial_transfer.sdf) · [Measured run](cpu-gripper-joint-state.json)

![The instrumented CPU gripper carrying the parcel](cpu-gripper-joint-state.jpg)

The robot lifted its 200 g parcel 126.07 mm, carried it 462.44 mm and placed it on the receiving shelf. The shelf then carried 1.962 N. All 451 visual frames were byte-identical to the earlier transfer without the publisher. gz-web-rs checked 9,020 displayed poses and shapes and rewound to the initial state. The release-mode CPU tests covered filtering, IDs, positive-rate publication, motion and loaded-world reset. The GPU multi-owner regression ran on the RTX 5090. LLVM recorded 21/21 changed production lines and 41/42 regions in the scene publisher. [Watch the transfer in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10003%2F)

To reproduce the transfer from the physics checkout, choose a fresh archive directory.

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli --example serial_transfer -- transfer artifacts/my-observed-transfer
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-rs -- --server --replay --web-viewer 10003 artifacts/my-observed-transfer
```

The live packet regression runs the actual server, subscribes to the robot topic, drives the rail and compares the resulting wire measurements with the independently accessible joint samples.

```sh
RUSTC_WRAPPER= cargo test --release --locked --no-default-features -p gz-sim-server --lib the_live_cpu_transfer_robot
```
