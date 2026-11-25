<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Two stock Gazebo double pendulums on a shared floor

The ordinary local CPU simulator loads the unchanged upstream Gazebo world. Each stand has a 100 kg base and two 1 kg rods. Gravity acts half a metre from each rod’s link origin. The SDF omits their inertia tensors, so the loader uses the SDF default unit diagonal tensor about each authored center of mass. The impulse hinge anchors stay in their link frames. A CLI regression gives identical states with the default tensor spelled out, and a different trajectory when just the centers of mass move to the link origins. [Upstream world](https://github.com/gazebosim/gz-sim/blob/9e3b78f5de297def8fe47fd189500fb5cd6538ef/test/worlds/double_pendulum.sdf) · [Rust fixture](../../../examples/worlds/upstream_double_pendulum.sdf)

From the repository root run the following command for six simulated seconds. The visual replay uses gz-web-rs lighting because its primitive scene format does not carry the authored sun. [Watch the pendulums](pendulum-view.mp4) · [Open in gz-web-rs and reproduce the recording](pendulum-view.md)

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  examples/worlds/upstream_double_pendulum.sdf --physics-engine multifidelity --stiff-static-contacts --iterations 6000
```

## Accuracy relative to DART

Both engines used the original file, the same gravity and a 1 ms timestep. Gazebo’s DART backend provided an independent angle, angular velocity, COM position and mechanical energy observation for both models every 10 ms. Two independent native launches reproduced all 1,202 CSV records byte for byte. The period measures the time between two successive descending crossings of the upper arm through vertical. The table uses the worst difference across both assemblies and all sampled times. The energy percentage uses the initial 39.12 J in each assembly with potential energy referenced to the authored floor. [Native capture method](../../../tools/physics-audit/pendulum-native/README.md) · [Raw traces, comparisons, library hashes and load sweep](pendulum-reference.tar.xz)

| Quantity | Default fixed-body contacts (60 Hz) | Stiffer fixed-body contacts (120 Hz) | Acceptance limit |
|---|---:|---:|---:|
| Largest joint angle difference | 0.04038 rad | 0.05557 rad | 0.06 rad |
| First return time, native 2.75577 s | 2.75212 s | 2.75103 s | 1% difference |
| Largest system energy difference | 1.22037 J (3.12%) | 0.44508 J (1.14%) | 2% of initial energy |

The default contact spring let the weighted base sink 1.22 mm. At the worst energy sample, its extra settlement relative to DART accounted for about 1.19 J. Setting geometric contact slop to 0.1 mm produced the same trajectory byte for byte. Doubling the fixed-body contact spring frequency limited maximum sinking to 0.43 mm while preserving the 1 ms simulation clock and contact damping. The stiffer profile brought energy within the acceptance limit and increased the peak angle difference by 0.0152 rad. The profile is explicit in the CLI and in the journal trace. Verification refuses a trace recorded with a different profile.

## Tested operating range

Each of the three lower-link masses (0.5, 1 and 2 kg) ran with each of the three isotropic principal inertias (0.5, 1 and 2 kg·m²). Across those nine combinations on each stand, the stiff profile’s worst energy drift was 1.46% or less, hinge-anchor separation stayed below 0.035 mm, and first return times ranged from 2.533 to 3.075 s. All use the stock upper link and base. This is a measured grid for this mechanism.

The stiff profile’s first return moved from 2.74989 to 2.75182 s as the timestep decreased from 2 to 0.125 ms. Successive angle differences shrank on each halving. The default profile showed nonmonotonic angle differences on the coarser grids. Reset reproduced all 601 time samples per assembly for both profiles. The gripper, rotor and other existing scenes continue to select their original response by default.

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-scene --example upstream_double_pendulum -- target/double-pendulum-survey
```

## Packaged and tested revision

A fresh GitHub checkout of `874572b80a0ed97ebafc7ecd920ba2e239ab9aac` produced the Linux x86-64 preview with SHA-256 `a2ad68a91b296aa5771dc6567f14571c3655f15d88c3057e56da61640a3aebf2`. A clean Ubuntu 24.04 container stepped both pendulums for six seconds, reproduced the packaged visual archive byte for byte and verified all 6,002 physics records across one and eight Bevy threads. The installed rotor also reproduced its 610 records. [Build the package](../../LOCAL_CPU_MVP.md#build-the-installable-preview)

The focused LLVM run covered all 57 changed executable production lines and all 76 mapped regions starting on those lines across the four affected crates. The combined export warned of seven functions with mismatched profiling data. The run included the native-reference samples, negative inertia, reset, checked controls, unsupported rendering, compound links and joint-controller regressions. Formatting and focused strict Clippy passed. The project-wide coverage and release checks remain open. [Hosted source check](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/45533925967) · [Repository release criteria](../../PHYSICS_RELEASE_ROADMAP.md)
