<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Upstream robot joint on the local CPU engine

Gazebo's SDF loads unchanged through the normal Rust CLI. It describes a free 120 kg base resting on a plane and a 0.6 kg rotor driven by an ordinary revolute joint. Gazebo uses the world to check delivery of a scalar force command. The Rust regression also measures the resulting motion, ground reaction and reset repeatability. The local solver now accepts effort on an ordinary impulse-solved revolute joint with a dynamic parent. It resolves the hinge axis in the current parent frame and applies equal and opposite torques.

## Reproduce

Run these from the workspace root. The headless run leaves the actuator at its default zero command. The second command drives it forward, lets it coast, reverses it and prints the physical measurements.

```sh
cargo run --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  --physics-engine multifidelity --world default --iterations 250 examples/worlds/upstream_joint_force.sdf
cargo test --release --locked --no-default-features -p gz-sim-scene --test upstream_joint_force -- --nocapture
```

The server accepts a scalar torque in N·m on `/model/joint_force_test/joint/j1/cmd_force`. The test uses the same persistent scene input on `default::joint_force_test::j1`. The zero-command CLI run ended at 250 ms with the base center 4.890 mm above the plane.

To record a driven run for gz-web-rs, choose fresh destinations under an existing directory. This journal drives, coasts and reverses the rotor, resets the world, then repeats the initial drive. It writes one native recording per epoch. The verification run checks every trace observation in a new process.

```sh
mkdir -p target
cargo run --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-local-replay -- \
  examples/worlds/upstream_joint_force.sdf examples/replay/upstream_joint_force.json \
  --trace target/upstream-rotor.jsonl --record target/upstream-rotor --record-every 10
cargo run --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-local-replay -- \
  examples/worlds/upstream_joint_force.sdf examples/replay/upstream_joint_force.json \
  --verify target/upstream-rotor.jsonl
```

## Measurements

The solver and controller used the SDF's default 1 ms step. The original file has an unrestricted hinge and default surface friction. Each variant below modifies a copy in memory.

| Case | Physical check | Measured result |
| --- | --- | --- |
| Original, supported | After settling 50 ms, apply 0.012 N·m for 100 ms. A rotor with axial inertia 0.00012 kg·m² should reach 10 rad/s. | 9.99999996 rad/s, 0.50125 rad of travel. The 100 ms normal impulse was 118.210 N·s against 118.188 N·s expected from total weight. |
| Original, command reversal | Coast for 50 ms, then apply −0.012 N·m for 200 ms. Reset and repeat. | Coast speed stayed within 0.01 rad/s and reversed speed within 0.02 rad/s of −10 rad/s. Sampled positions, speeds and summed support impulse were identical after reset. |
| Tilted, in zero gravity | Rotate the assembly by 0.6 rad and place it above the plane. Compare the original rotor to one with four times the axial inertia. | Rotor speed fell from 10 to 2.5 rad/s. Base speed was −0.000240 rad/s in both cases. Residual total angular momentum was below 5 × 10⁻¹⁵ N·m·s. |
| Rocking base | Start tipped above the plane and allow 140 ms for the base to rotate. Compare a one-tick torque pulse to an unforced twin. | The hinge axis moved 0.22745 rad. The pulse added 0.09996 rad/s along the moved axis, with a measured direction cosine of 0.999997. |
| Ground friction | Apply a horizontal gravity component of 1 m/s² for 100 ms. Repeat with the assembly elevated and clear of the plane. | Grounded lateral speed was 0.00000355 m/s, compared with 0.100000 m/s in free flight. |
| Added travel stops | Author ±0.25 rad stops. Hold torque into each stop for 250 ms. | Settled within 3 × 10⁻¹² rad of each stop, with speed below 1.1 × 10⁻⁸ rad/s. |

### Native Gazebo comparison

A separate headless run of Gazebo 11.0.0~pre3 with DART used the original SDF, the same 1 ms step and the same positive, zero and negative torque commands. Eight samples at 0, 1, 50, 100, 200, 300, 400 and 500 ms matched the committed simulation clocks. A second native run added only an encoder publisher. Its link and model poses matched the original native state service at every sampled time, to the precision of that service. The Rust poses used for comparison matched the packaged CLI recording within `1e-12`.

Both engines reached 10 rad/s after 100 ms and −10 rad/s after 500 ms. At 100 ms the native joint had turned 0.505000 radians and the Rust joint 0.501250 radians. Across the eight observations the largest angle discrepancy was 0.00375127 rad (0.215°), the largest speed discrepancy was 0.000002401 rad/s and the largest base height discrepancy was 0.1201 mm. These are sampled differences for this rotor and base. [Native and Rust samples](mvp-upstream-dart.json)

## Verification

The release build passed all 22 tests across `joint_effort`, `joint_motor`, `joint_force_feedback` and `upstream_joint_force`. These include the standalone engine's existing floating-parent refusal, its rotated fixed-parent response, and preservation of admitted effort after invalid, nonfinite or overflowing replacements. The upstream journal verified all 610 profile and state records in a second process and recorded 51 frames before reset and 11 afterward. The new scene regression passed strict Clippy.

On candidate `836bbcd`, relative to `1311332`, LLVM covered 16/16 changed executable Rapier lines (100%) and 23/24 changed regions (95.83%). The joint validator covered 6/6 lines and 11/12 regions. The backend entry points covered 10/10 lines and 12/12 regions. These are the four release test executables above, built with the repository's Rust coverage wrapper. Each executable was paired with its own LLVM profile. The counts take the union of mapped changed lines and distinct code-region coordinates starting on changed lines. The remaining region is the negative arm of the angular-motion check for an ordinary impulse hinge. [Full release coverage status](mvp-release-coverage.md)

The clean CPU CLI tests ran all 25 journal cases and 160 library cases. The upstream journal checked forward drive, coasting and reverse speed, support impulse, all 100 post-reset states, the generated plane in both archives, and rejection of an unsupported world plugin or a dynamic plane collider. The live upstream transport regression also passed 15 consecutive process launches. Focused LLVM measurement on the changed plane, default material and replay admission code covered 89/89 executable lines and 163/164 distinct regions starting on changed production lines. The local control client reached all five changed executable lines and all nine changed regions in its separate live-server test. Repository-wide qualification uses its own denominators. [Release gate](mvp-release-coverage.md)

## Provenance

The vendored file is the 3,003-byte original from `gazebosim/gz-sim` revision `9e3b78f5de297def8fe47fd189500fb5cd6538ef`, distributed under Apache-2.0. Its SHA-256 is `a25de92ae735a8b2b349a3e180928c1941153be7a787a1dcd54076c7d5e4d323`. [Upstream SDF](https://github.com/gazebosim/gz-sim/blob/9e3b78f5de297def8fe47fd189500fb5cd6538ef/test/worlds/apply_joint_force.sdf) · [Upstream command test](https://github.com/gazebosim/gz-sim/blob/9e3b78f5de297def8fe47fd189500fb5cd6538ef/test/integration/apply_joint_force_system.cc) · [Upstream license](https://github.com/gazebosim/gz-sim/blob/9e3b78f5de297def8fe47fd189500fb5cd6538ef/LICENSE)
