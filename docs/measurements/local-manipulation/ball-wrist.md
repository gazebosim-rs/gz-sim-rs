# A three-axis spherical wrist

The orange tool receives torque in the bearing's parent frame, which starts tilted and then turns with the teal carrier. The two free links counter-rotate about a common center. The largest measured total angular momentum over 2.4 s was 3.76 × 10⁻¹⁴ N·m·s. The native gz-web-rs replay contains 121 frames and all 363 displayed shapes and poses were checked through rewind. [Watch the wrist](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9916%2F)

The Rust scene entry point is `queue_ball_joint_effort(&mut sim, public_joint_id, [tx, ty, tz])`. Torques accumulate in N·m. The advancing solve transforms the sum from the parent joint frame and applies a balanced couple. A fixed world mount absorbs its reaction. Pausing retains the command, advancing consumes it, and resetting discards it. Overflow and invalid joint requests leave earlier inputs intact.

A 650-step test compared the change in tool angular momentum on every tick with the commanded impulse in the independently calculated moving frame. The sample's relative velocity matched the difference in link speeds and two passes across reset produced identical scalar bits for all joint samples. A world-mounted test checked accumulated input, invalid values, coasting and overflow. The six directly related Rust test targets passed 33 checks. Exact-build LLVM profiles covered 133/136 changed executable lines and 229/241 changed regions in the combined solver, adapter, scene and recorder. [Counts and native measurements](ball-wrist.json)

Run the example with a new recording directory, then open gz-web-rs against its replay socket.

```sh
export RUSTC_WRAPPER=
cargo run --release --locked --no-default-features -p gz-sim-cli --example ball_wrist -- target/my-ball-wrist
cargo run --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-rs -- --server --replay --web-viewer 9916 target/my-ball-wrist
```
