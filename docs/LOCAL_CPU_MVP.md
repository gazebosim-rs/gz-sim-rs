# Run a robot with the local Rust physics engine

Source candidate `2e7d155` includes published Gazebo port commit `dddf1f4` and runs rigid-body robots in a local Bevy world. The two starting points are an unchanged Gazebo motor on a free base and a screw-driven gripper that lifts a 200 g parcel. Start with the gripper to see a full task, or use the motor to test your own torque commands. The examples below use the locked Rust toolchain and an empty output directory.

## Get the simulator source

From the directory where you keep Gazebo projects, clone the physics review branch. The optional browser viewer builds from its own repository alongside the simulator. The recordings and physics tests live in the simulator checkout.

```sh
git clone --branch review/local-cpu-mvp-20261124 https://github.com/gazebosim-rs/gz-sim-rs.git
git clone --branch review/physics-worker-recovery-ui https://github.com/gazebosim-rs/gzweb-rs.git
cd gz-sim-rs
```

## Install the command-line tools

From a clean checkout, build all three tools into a directory you own. This uses the locked dependency graph and the local CPU feature set. The installation was exercised from a separate directory on Linux x86-64. A fresh process verified all 610 motor journal records using eight Bevy threads after recording them with one.

```sh
RUSTC_WRAPPER= cargo install --locked --path crates/gz-sim-cli --no-default-features \
  --bin gz-sim-rs --bin gz-sim-control --bin gz-sim-local-replay --root "$PWD/target/cpu-tools"
target/cpu-tools/bin/gz-sim-rs examples/worlds/upstream_joint_force.sdf \
  --physics-engine multifidelity --iterations 250
```

The locally packaged Linux preview, `gz-sim-local-cpu-preview.tar.xz`, includes these binaries, the unmodified source world, its command journal, the gripper recording, licenses and a checksum manifest. The archive built from `2e7d155` has SHA-256 `2983b9af0ad1164cf118f0c8b2deca79e0ed5dc65228292ef8d898423e0b7dee`. The review branch includes the source and portable recordings.

## Watch the recordings

Open the gripper replay and press Play. Both jaws touch at 1.274 s, lift the parcel 126.6 mm and release it onto the plate. Move the pointer into the viewport and press **3** for the side view. In the rotor replay, select the row ending in **rotor::visual**, move the pointer into the viewport and press **F**. The upstream ground is 100 m wide, so framing the rotor makes its rotation visible. [Gripper with tier switching](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11547%2F) · [Always-dynamic gripper](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11548%2F) · [Upstream motor](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11551%2F)

![Upstream rotor selected at 175 ms in gz-web-rs](measurements/local-manipulation/mvp-upstream-rotor.jpg)

The links use replay servers on the development machine. On another machine, build the CLI from this checkout, unpack the recordings, and start one replay server. The archive contains the original two gripper runs, both motor runs before and after reset, and the ground-plane mesh. Use a free socket port, and substitute it in the viewer URL. [Portable gz-web-rs recordings](measurements/local-manipulation/mvp-robot-replays.tar.xz)

```sh
export RUSTC_WRAPPER=
cargo build --release --locked --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs --bin gz-sim-local-replay --bin gz-sim-control
mkdir -p target/mvp-replays
tar -xJf docs/measurements/local-manipulation/mvp-robot-replays.tar.xz -C target/mvp-replays
(cd target/mvp-replays && sha256sum --quiet -c SHA256SUMS)
target/release/gz-sim-rs target/mvp-replays/switched --server --replay --web-viewer 9002
```

Serve a built sibling gzweb-rs checkout on HTTP port 8082, then open the viewer. The simulation and command executables are Rust. The static file server solely delivers the viewer's WebAssembly application to the browser. [Open replay on port 9002](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9002%2F)

```sh
(cd ../gzweb-rs && ./build.sh && python3 -m http.server 8082 --bind 127.0.0.1)
```

## Drive the upstream motor

A paused server for the upstream robot is also running on the development machine in partition `gz_local_cpu_mvp`. [Open the live motor in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&url=ws%3A%2F%2F127.0.0.1%3A11552%2F)

To launch your own, start the ordinary server paused. Its gz-web view supports play, pause and reset. Gazebo clients can publish a `gz.msgs.Double` torque in N·m on `/model/joint_force_test/joint/j1/cmd_force` and send `gz.msgs.WorldControl` to `/world/default/control` to step. A 0.012 N·m command held for 100 steps spins the rotor to 10 rad/s. [Open the live server](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&url=ws%3A%2F%2F127.0.0.1%3A9003%2F)

```sh
target/release/gz-sim-rs examples/worlds/upstream_joint_force.sdf \
  --physics-engine multifidelity --server --web-viewer 9003
```

The local control client talks to that same server. In a second terminal, start with a freshly launched paused world. A checked step requires its current epoch and tick. Reset restores the authored poses, clears held commands, pauses, and advances the epoch.

```sh
target/release/gz-sim-control --port 9003 status
target/release/gz-sim-control --port 9003 step 0 0
target/release/gz-sim-control --port 9003 play
target/release/gz-sim-control --port 9003 pause
target/release/gz-sim-control --port 9003 reset
target/release/gz-sim-control --port 9003 step 1 0
```

For repeatable controls on Linux x86-64, edit the ordered JSON journal. The shipped one applies torque for 100 ms, coasts for 200 ms, reverses torque for 200 ms, resets and repeats the first drive. The verification process checks the executable identity and every observation. The two epoch directories can each be served with the replay command above.

```sh
mkdir -p target/mvp-drive
world=examples/worlds/upstream_joint_force.sdf
journal=examples/replay/upstream_joint_force.json
target/release/gz-sim-local-replay "$world" "$journal" --threads 1 \
  --trace target/mvp-drive/trace.jsonl --record target/mvp-drive/recording --record-every 5
target/release/gz-sim-local-replay "$world" "$journal" --threads 8 \
  --verify target/mvp-drive/trace.jsonl
```

To run a different supported SDF, substitute its file and world name. To exercise the live transport check or regenerate the switched and always-dynamic gripper measurements, use the ordinary workspace commands below. The gripper example accepts a fresh output directory.

```sh
cargo test --release --locked --no-default-features -p gz-sim-cli --test native upstream_motor
cargo run --release --locked --no-default-features -p gz-sim-cli --example parked_spindle -- \
  target/mvp-gripper --capped --threads 1
```

## What this build covers

The CPU solver has physical regression tests for fixed, revolute, prismatic, ball, universal and screw joints. The independently authored model qualifies a free-base revolute joint, sustained effort, loaded inertia, plane contact, friction and reset. The full gripper task qualifies multiple joints, contact impulses and automatic tier changes against an always-dynamic run. The motor was checked against analytical acceleration and a native Gazebo DART run at eight matched times. The gripper's reference uses the same dynamic solver. [Motor measurements and source](measurements/local-manipulation/mvp-independent-robot.md) · [Gripper accuracy, p95 and real-time factor](measurements/local-manipulation/mvp-local-performance.md)

Ordinary SDF support is still selective. On this candidate three unmodified Gazebo test worlds failed at load time. The differential-drive world requests joint-state axes for a ball caster, the double pendulum specifies an offset mass center in an unsupported inertial form, and the position-controller world requests a world plugin outside the admitted set. The loader reports the first unsupported feature in each file.

The Linux package is a preview. On candidate `fc80ca5`, all 4,208 ordinary CPU tests passed and every local physics backend exceeded 95% line and region coverage. The full CPU gate still needs coverage in the CLI, scene and distributed crates, and a way to account for optional source files excluded from that build. The software renderer also missed its coverage targets. [Release qualification and next tests](measurements/local-manipulation/mvp-release-coverage.md)

## Build the installable preview

Run the following commands from a clean source checkout on Linux x86-64 with Bash, GNU tar, xz, the locked Rust toolchain, `protoc` 3.21.12 and its matching protobuf development includes. The build verifies `google/protobuf/any.proto` from those includes. The archive is created in a fresh staging directory and includes a checksum manifest, input journal, measured gripper replay and standalone instructions.

```bash
set -eu
mkdir -p target
stage=$(mktemp -d "$PWD/target/local-physics-preview.XXXXXXXX")
package="$stage/package"
mkdir -p "$package/share/gz-sim" "$stage/recordings"
RUSTC_WRAPPER= cargo install --locked --path crates/gz-sim-cli --no-default-features \
  --bin gz-sim-rs --bin gz-sim-control --bin gz-sim-local-replay --root "$stage/install"
cp -a "$stage/install/bin" "$package/bin"
cp LICENSE NOTICE "$package/"
cp docs/LOCAL_CPU_PACKAGE.md "$package/README.md"
cp examples/worlds/upstream_joint_force.sdf examples/replay/upstream_joint_force.json "$package/share/gz-sim/"
tar -xJf docs/measurements/local-manipulation/mvp-robot-replays.tar.xz -C "$stage/recordings"
(cd "$stage/recordings" && sha256sum --quiet -c SHA256SUMS)
cp -a "$stage/recordings/switched" "$package/share/gz-sim/gripper-replay"
git rev-parse HEAD > "$package/BUILD-INFO"
rustc -vV >> "$package/BUILD-INFO"
(cd "$package" && find . -type f ! -name SHA256SUMS -print0 | LC_ALL=C sort -z | xargs -0 sha256sum > SHA256SUMS)
tar -cJf "$stage/gz-sim-local-cpu-preview.tar.xz" -C "$package" .
sha256sum "$stage/gz-sim-local-cpu-preview.tar.xz"
```

After extracting, run the commands in the package README. To repeat the lost-request and lost-reply checks against those exact binaries, point the regression test at the extracted directory.

```sh
GZ_TEST_LOCAL_CPU_INSTALL=/absolute/path/to/extracted/package RUSTC_WRAPPER= \
  cargo test --release --locked --no-default-features -p gz-sim-cli --test native installed_rotor_reconciles
```
