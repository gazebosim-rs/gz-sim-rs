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

## Watch the current local robot experiments

Watch one finger lose its grip during a lift, or compare a load-confirmed set-down with an attempted placement over the wrong receiver. [Twenty local robot phases in gz-web-rs](measurements/local-manipulation/regrasp-setdown-view.md) · [Measurements and run commands](measurements/local-manipulation/regrasp-setdown.md)

The moving gripper senses a parcel slipping, tightens both fingers, and can reduce lift effort when the fingers reach their force limit. Compare a catch driven by tactile contact with the same parcel sliding without feedback. [Watch all ten phases](measurements/local-manipulation/slip-catch-view.md) · [Run the experiment and compare with fine physics](measurements/local-manipulation/slip-catch.md)

A vertical gripper carries a parcel with its weight offset toward one finger. Compare the full-strength lift, the weakened grip and a shove that rocks the parcel during travel. [Open the carry recordings](measurements/local-manipulation/offset-carry-view.md) · [Grip threshold and measured slip](measurements/local-manipulation/offset-carry.md)

The rocking-parcel replay follows the parcel from the shove through the return strike. The plot shows how the fingers’ contact during that motion changes the time of impact. [Watch in gz-web-rs](measurements/local-manipulation/pad-approach-view.md) · [Timing, impulse and step cost](measurements/local-manipulation/pad-approach.md)

The returning-edge replay shows how three short bursts of fine stepping recover the force spike as the parcel rocks back into the gripper. The three traces compare ordinary, adaptive and continuously fine physics with the same gripper commands. [Replay and force curves](measurements/local-manipulation/pad-impact-refinement-view.md) · [Measured accuracy and cost](measurements/local-manipulation/pad-impact-refinement.md)

The gripper can rock a block onto two loaded corners on each pad and catch the opposite edge as it returns. The tall block shows how the solver keeps the grip steady as unloaded corners leave the collision prediction range. [Watch both in gz-web-rs](measurements/local-manipulation/pad-rocking-view.md) · [Force, torque and impact measurements](measurements/local-manipulation/pad-rocking.md)

The two-finger gripper accepts a pad stiffness in N/m for each slider. Under a 3 N squeeze, 30,000 N/m pads compressed by 0.100 mm each on three different finger masses. [SDF example](../examples/worlds/local_stiff_pad_grasp.sdf) · [Static qualification](measurements/local-manipulation/pad-stiffness.md) · [Loading and release qualification](measurements/local-manipulation/pad-transient.md) · [Watch the grip and release in gz-web-rs](measurements/local-manipulation/pad-transient-view.md)

The two-finger experiment shows a 200 g parcel holding, slipping as the motors weaken, and falling when the jaws open. A second view shows a sphere rolling along inclined jaws while both travel stops carry its weight. [Grasp and release](measurements/local-manipulation/two-finger-view.md) · [Double-stop loads and groove replay](measurements/local-manipulation/two-finger-load.md)

The powered finger can brace at the end of its travel, release under a strike from the opposite side, and reverse under a held motor force. Compare the first collision in slow motion or follow the full 40 mm stroke on the same mechanism. [Impact close-ups](measurements/local-manipulation/powered-slider-view.md) · [Full stroke, forces and commands](measurements/local-manipulation/powered-slider-cycle.md)

A separate passive wrist and sliding finger now share a strike on their first physics step. Both reflected strikes can be compared with the hand calculation through 200 ms. [Corrected wrist, timing and native replays](measurements/local-manipulation/reduced-wrist-slider.md)

The gripper's load-path recordings show how a parcel's first strike moves through the sliding finger into the palm and wrist. Three gauges follow the measured impulses through the first millisecond. [Signed measurements, native recordings and reproduction](measurements/local-manipulation/wrist-impact-loads.md)

The 50 µm jaw-strike recordings show individual hits inside a single 1 ms controller step. A separate 18 s recording follows the same gripper carrying its free 200 g parcel to a shelf. [Jaw impact, full pickup and replay instructions](measurements/local-manipulation/early-jaw.md) · [Watch the carry](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10090%2F)

The shelf-transfer robot grips a 200 g parcel, carries it past an obstacle and hands its weight to a shelf. The loaded wrist recording compares two motors responding to the same shove. The sheet recording shows a gripper losing its receiving table mid-release. Each link opens a measured run in gz-web-rs. Press **Play** to advance it. [Shelf-transfer instructions and native view](measurements/local-manipulation/serial-transfer.md) · [Loaded wrist](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10042%2F) · [Sheet landing](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10039%2F)

For the serial gripper under load, watch the fingers open at 140 ms and catch the parcel again at 178 ms. The same robot also has a jaw-impact recording with a 1 ms controller clock, and the folded-sheet arm shows a stable placement beside one that tips off the table. [Loaded regrip](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10069%2F) · [Fast jaw impact](measurements/local-manipulation/serial-touching-solver.md) · [Sheet placements](measurements/local-manipulation/sheet-table-edge.md)

For a fresh copy of the wrist replay, start at the root of this physics checkout. The archive contains the measured frames and can be opened without rerunning the experiment. Choose an unused output directory and WebSocket port.

```sh
mkdir -p target/wrist-view
tar -xJf docs/measurements/local-manipulation/loaded-wrist-recovery.tar.xz -C target/wrist-view
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 18142 target/wrist-view/replay
```

Keep that server running and open the viewer. The two wrists are offset along the Y axis, so use the perspective view to see them side by side. [Open your copy on port 18142](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A18142%2F)

The local qualification machine already serves the gz-web-rs frontend on 8082. To restart its existing frontend after a reboot, run the following in a second terminal from the same physics checkout. The paths select the WebAssembly bundle used in the recorded browser checks.

```sh
viewer=../gz-sim-rs-multifidelity/target/convoy-operator-ui/web
mime=$(realpath ../gz-sim-rs-multifidelity/target/local-robot-foundations/mime.conf)
busybox httpd -f -p 127.0.0.1:8082 -h "$viewer" -c "$mime"
```

On a separate machine, build and serve the replay-capable gzweb-rs checkout and use its frontend URL with the same replay parameters. To regenerate the physics recording with your own Rust build, run the loaded wrist example. [Build the viewer](../../gzweb-rs/README.md) · [Run the loaded wrist experiment](measurements/local-manipulation/loaded-wrist-recovery.md)

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

## Inspect a gripper landing

Follow a parcel across a 1.2 mm gap and step through the first jaw reaction. The two recorded approaches use the same rail setting with opposite incoming directions. A separate GPU replay shows an eccentric parcel rocking onto a second corner of a shelf. [Gripper and carriage rail](measurements/local-manipulation/mounted-rail.md) · [GPU shelf landing](measurements/local-manipulation/gpu-glancing-corner.md)

## Add parts while the robot world is running

Spawn a cylinder and capsule, watch them hit a slab, remove them and repeat from reset. Both parts use the same physical material and generated mesh as their source model. The command and live viewer paths accept the same local templates. [Run the curved deliveries and compare the two native replays](measurements/local-manipulation/curved-delivery.md) · [Watch the corrected cylinder bounce and settle](measurements/local-manipulation/axial-cap.md)

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

## Lift a folded sheet and vary its material

The arm closes around opposite panels of one unpinned triangular cloth, lifts both panels, holds them for 300 ms and returns the cloth to the bench. The orange and teal particle markers identify the panels. The open-jaw control uses the same lift command. The nominal sheet has a slight outward flare. A second fold starts with parallel panels and tests sustained sheet self-contact.

Run either study from the simulator workspace, choosing a fresh output directory each time.

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- arm-sheet target/my-folded-sheet
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- sheet-sweep target/my-sheet-sweep
```

The sweep varies payload mass and material stiffness independently at 0.5×, 1× and 2×. A 2× stiffness scales down all three stretch, shear and bend compliances by half. Pad compliance, friction, the authored fold and the 1 ms arm command clock stay fixed. Each cell in `sweep.json` includes its two- and four-substep outcomes. An additional eight-substep trial checks the heavy, soft sheet. A solver refusal records the first failed tick. A completed trial reports each jaw’s squeeze and vertical load, both panel heights, the bench reaction after release, and p95 step cost.

The energy measurements use the material's committed velocities and positions, gravity, rest lengths and signed dihedral angles. The estimated work supplied by each jaw pairs its signed reaction with its measured displacement and rotation over that tick. This resolves gains above estimated pad work during the lift. A complete robot energy balance would additionally need motor work, pad storage and every dissipative term.

To serve a recording yourself, start gz-web-rs on port 8082 and the Rust server on an unused port.

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 9770 target/my-folded-sheet/held/replay
```

Open the viewer and press Play. Change the replay directory to `open/replay` for the control, or to `target/my-sheet-sweep/heavy-soft/replay` for the heavier, more flexible specimen. [Open your replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9770%2F)

The four local recordings show the same arm command. Press Play to follow the pickup and release. [Nominal folded sheet](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9761%2F) · [Open-jaw control](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9762%2F) · [Heavier, softer sheet](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9763%2F) · [Parallel fold](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9765%2F)

Both panels rose about 15 cm for every weight and stiffness in the sweep. The open-jaw control stayed on the bench. Lift height repeated within 6.18 µm across the two sweep clocks, while the force measured inside the fold varied with the material step size. [Folded-sheet measurements](measurements/local-manipulation/arm-sheet.json) · [Mass and stiffness sweep](measurements/local-manipulation/sheet-sweep.json)

## Compare the force inside a loaded fold

The gripper picks up a sheet with parallel layers and holds it over the bench. At 1.650 seconds each trial starts from the same measured sheet positions, velocities, joint states and jaw reactions. The arm still advances every millisecond. Only the material substeps and constraint iterations change during the next 300 milliseconds. The orange and teal markers identify the two sides.

Run the study from the simulator workspace with a fresh output directory.

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- fold-hold target/my-fold-hold
```

The `comparison.json` file contains the mean, first resumed load and every millisecond of force measurements for seven resolutions. The force is the summed normal contact force between the two layers in newtons. Converting it to pressure would require a contact area. The separate mesh comparison changes the spacing between particles across the width and drives each specimen along the recorded reference jaw motion.

Watch the two local recordings and press Play. To serve your own result, use the replay server command in the preceding section with `target/my-fold-hold/coarse/replay` or `target/my-fold-hold/fine/replay`. [Eight material substeps](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9766%2F) · [Sixty-four material substeps](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9767%2F)

The force difference is too small to see from the marker positions alone. The Rust measurement checker verifies the common replay prefix and per-tick forces before plotting the trial and mesh comparisons. [Recorded force comparison](measurements/local-manipulation/fold-hold-forces.svg) · [Numerical results](measurements/local-manipulation/fold-hold.json)

```sh
cargo run --release --locked --manifest-path tools/physics-audit/numerics/Cargo.toml \
  --bin fold_hold_forces -- target/my-fold-hold target/my-fold-hold/forces.svg
```

## Compare jaw contact across the cloth surface

The `fold-patch` study holds the sheet for 300 ms starting from the loaded fold at 1.650 s. It runs particle contact and a prescribed surface patch on sheets with 3, 5 and 7 points across their width. Each pair follows the same measured jaw motion. Each patch uses eight equal-area samples per panel over the 85 mm jaw width and 43 mm panel height. The total flat-pad stiffness for uniform indentation matches the particle control. The mesh, cloth material, friction, 1 ms arm clock, 64 material substeps and 48 solver iterations stay fixed within each pair.

```sh
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- fold-patch target/my-fold-patch

RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 9781 target/my-fold-patch/patch-7/replay
```

Open the viewer and press Play. The colored markers show the material positions. The force chart resolves differences too small to see at the scale of the robot. Each run writes a `comparison.json` and six native replay directories. [Open your seven-point replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9781%2F)

To use material-point contact in an authored Bevy scene, put one or more `<pad_sample>` elements inside the deformables plugin's `<cloth>`. The three zero-based indices identify vertices of one cloth triangle. The weights are its barycentric coordinates and sum to one. `stiffness_weight` multiplies the reciprocal of the compliant cuboid's `normal_compliance`. A sample can share its normal and friction loads across the vertices. The normal forces from equal indentations add in proportion to the stiffness weights. For example, this sample gives half a unit of stiffness to a point inside one face of a cloth with two columns.

```xml
<pad_sample particles="0 1 3" weights="0.25 0.25 0.5" stiffness_weight="0.5"/>
```

Authored samples replace particle contacts for every compliant cuboid on that cloth. Other boundaries use their usual collision models. Set `pad_loads="true"` on the cloth to read each node's share of the jaw reaction.

The recorded coarse particle run showed sustained layer pressure; its surface-patch counterpart touched on just two ticks, both in the first 3 ms. Both seven-point runs held the sheet with a gap between the layers. [Coarse particle replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9769%2F) · [Coarse patch replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9772%2F) · [Seven-point patch replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9774%2F) · [Force and timing chart](measurements/local-manipulation/fold-patch-forces.svg) · [Measurements](measurements/local-manipulation/fold-patch.json)


Rebuild the chart directly from its measurements.

```sh
RUSTC_WRAPPER= cargo run --release --locked --manifest-path tools/physics-audit/numerics/Cargo.toml \
  --bin fold_patch_forces -- target/my-fold-patch/comparison.json target/my-fold-patch/forces.svg
```

## Measure when a live gripper presses the layers together

The `fold-closure` example starts the folded sheet at rest on the bench. An articulated arm picks it up and slowly closes its fingers. The jaw motors apply forces, and each measured cloth reaction feeds back into the following rigid-body step. The example records the actual opening between the inner jaw faces. Material width, jaw command and actual opening appear separately in the measurements.

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- fold-closure target/my-fold-closure
```

Both material meshes use the same mass, pad footprint and stiffness calibration as the preceding phase. To keep a cloth's material properties as its grid changes, its SDF can override the compliance of individual distance constraints or bending hinges. For example, in a two-column grid, particles 0 and 3 form the shared diagonal of the first cell.

```xml
<distance_compliance particles="0 3" value="0.00002"/>
<hinge_compliance particles="0 3" value="0.02"/>
```

Place these elements inside the deformables plugin's `<cloth>`. Distance compliance is in m/N and hinge compliance is in rad/(N·m). Unlisted edges and hinges use the cloth's `stretch`, `shear` and `bend` values.

A run writes a native replay, the exact authored `world.sdf`, and a per-tick `trace.json` for each tested case. The `mesh-5` and `mesh-7` directories contain the primary runs. The `-fine` directories use the finer material clock, and the `-fixed` directories hold the jaw target at the last level below the primary threshold. To view your own seven-point result with gz-web-rs on port 8082, start its replay server, then open the viewer and press Play.

```sh
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 9782 target/my-fold-closure/mesh-7/replay
```

[Open your replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9782%2F) · [Recorded seven-point replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9777%2F) · [Seven-point fixed-target control](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9784%2F) · [Force versus jaw opening](measurements/local-manipulation/fold-closure-forces.svg)

To redraw the comparison from a run, pass its measurements and a new SVG path to the Rust chart tool.

```sh
RUSTC_WRAPPER= cargo run --release --locked --manifest-path tools/physics-audit/numerics/Cargo.toml \
  --bin fold_closure_forces -- target/my-fold-closure/comparison.json target/my-fold-closure/forces.svg
```

## Pull a folded sheet sideways

In `fold-pull` the robot lifts a sheet and closes its fingers until the measured force between the folded layers settles near 2 mN. Each mesh faces the same 0.3 N sideways pull. A second run follows the same grasp with zero applied load. The traces report the sheet center and the same material locations under each finger, all measured relative to the moving jaws.

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- fold-pull target/my-fold-pull
```

Start the seven-point pull replay and press Play in gz-web-rs. The output directory also contains `mesh-7-sham`, `mesh-5-pull` and `mesh-5-sham`. Each includes the starting SDF, native replay, material observations and the full 1 ms force trace.

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 9782 target/my-fold-pull/mesh-7-pull/replay
```

[Open your replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9782%2F) · [Recorded seven-point pull](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9792%2F) · [Matched unloaded hold](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9793%2F)

The gripper motion is visible in the replay. Use the displacement chart for the much smaller motion of the sheet inside the grip. To regenerate the chart and independently compare the sheet's change in momentum with the impulses from the pull, jaws and bench, run the two Rust tools on the same study.

```sh
RUSTC_WRAPPER= cargo run --release --locked --manifest-path tools/physics-audit/numerics/Cargo.toml \
  --bin fold_pull_response -- target/my-fold-pull target/my-fold-pull/response.svg
RUSTC_WRAPPER= cargo run --release --locked --manifest-path tools/physics-audit/numerics/Cargo.toml \
  --bin fold_pull_balance -- target/my-fold-pull
```
