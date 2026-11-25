# Native Gazebo double-pendulum reference

The Rust capture program calls the installed Gazebo Sim TestFixture binding through PyO3. It loads the exact upstream SDF, requests link velocities and both joint encoders on each model, and reads the committed data from Gazebo's post-update callback. For tick zero it reads the initial link inertials and assigns the release coordinates and speeds of zero specified by the SDF. The public binding uses the simulator's default server configuration. The program verifies the loaded DART plugin in the Linux process mappings. Gazebo's startup log names the selected engine.

The source world is `test/worlds/double_pendulum.sdf` from gz-sim commit `9e3b78f5de297def8fe47fd189500fb5cd6538ef`. Its SHA-256 is `24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa`. Each of the two free-standing assemblies has a 100 kg base, a 1 kg upper arm and a 1 kg lower arm. The native API reports gravity of (0, 0, −9.8) m/s². The output covers 6,000 unpaused 1 ms updates and the initial condition. It has 601 samples from each assembly at 10 ms intervals.

The CSV stores joint angle in rad, joint speed in rad/s, wrapped absolute world roll in rad, world COM coordinates in m and system energies in J. Potential energy sums −m **g** · **r** over the base and both arms. Native kinetic energy sums Gazebo's `Link::WorldKineticEnergy` for the same three bodies. This method includes translation at the body COM and rotation with the world inertia tensor. The first return interval uses the first two descending crossings of upper joint position −π + 1.5708 rad, with linear interpolation on the 10 ms observation grid. This measures one cycle of the nonlinear coupled motion.

## Reproduction

Run the commands from the gz-sim-rs workspace root. Select the CPython installation matching the installed Gazebo bindings. The paths shown are those used in the retained run. `source-and-libraries.sha256` in the evidence directory identifies the input SDF and the native libraries on that machine. Two independent native launches produced the same CSV bytes.

```bash
audit=tools/physics-audit/pendulum-native
py=/home/nkoenig/.local/share/uv/python/cpython-3.10.21-linux-x86_64-gnu
export PYTHONHOME="$py" PYO3_PYTHON="$py/bin/python3" PYTHONPATH=/home/nkoenig/local/lib/python
export LD_LIBRARY_PATH="$py/lib:/home/nkoenig/local/lib:${LD_LIBRARY_PATH:-}"
export RUSTC_WRAPPER=
cargo build --locked --release --manifest-path "$audit/Cargo.toml" --target-dir target/pendulum-native-tools
out=$(mktemp -d)
capture=target/pendulum-native-tools/release/pendulum-native-reference
GZ_PARTITION="pendulum_reference_$$" "$capture" examples/worlds/upstream_double_pendulum.sdf "$out/native.csv" > "$out/server.log" 2>&1
sha256sum "$out/native.csv"
# This host produced 5eb4b8f9885d9b34550ab1668e2294f3e25356e247593b86695393418d7a2b4d.
cargo run --locked --release -p gz-sim-scene --example upstream_double_pendulum -- "$out/rust"
target/pendulum-native-tools/release/compare "$out/rust/rust-stiff.csv" "$out/native.csv" "$out/comparison-stiff.json"
# Default contact settings exceed the stated energy threshold. This command records the failed comparison.
target/pendulum-native-tools/release/compare "$out/rust/rust.csv" "$out/native.csv" "$out/comparison.json"
```

The comparator requires both complete traces in the same order and verifies the finite energy components in every row. It writes the per-model maxima and crossing times, then returns a failure if any model exceeds 0.06 rad maximum absolute joint-position error, 1% first-return error or 2% peak absolute energy difference divided by initial total energy. Each maximum is taken over the matched samples. For a changed solver or precision configuration, pass its CSV in place of `rust.csv`.
