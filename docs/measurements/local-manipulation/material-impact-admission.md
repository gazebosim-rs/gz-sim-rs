<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Simultaneous gripper contact on cloth

Two fingers strike one cloth face at 100 ms in the authored local impact scene. The solver records neither pad at 99 ms. At 100 ms it resolves both native finger links in one contact batch, with both incoming normal speeds above 0.7 m/s and separating normal speeds above 0.4 m/s.

Fourteen malformed variants of the same SDF check which solver owns the instant of impact. The loader rejects separately scheduled pads at the same instant, mixes of timed and automatically detected impacts in either order, duplicate automatic owners, missing or conflicting clocks, invalid site lists, incompatible substep bounds and unsupported restitution. Running the existing material suite with and without this test on the same instrumented binary added 39 executed lines and 47 regions in the contact configuration loader. The five local tests and strict Clippy passed on Ubuntu.

```sh
RUSTC_WRAPPER= cargo test --locked -p gz-sim-scene --test articulated_material \
  the_two_loaded_pads_need_one_solver_batch_and_one_owner_of_the_approach -- --exact
```
