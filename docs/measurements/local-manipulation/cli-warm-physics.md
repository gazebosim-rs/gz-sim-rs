<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Warm physics checkpoints from the public CLI

A curved moving platform carries a free parcel. A second authored world rolls a sphere on a floor alongside a separate block. For each world, the Rust CLI saved the warm contact state at 400 ms and advanced uninterrupted to 600 ms. A new CLI process reopened the file at 400 ms and reached the same link positions. Another server reopened it while paused, accepted one real world-control request over local Zenoh, and saved a fresh checkpoint at 405 ms on the operator's SIGUSR1. Reopening that second checkpoint again reproduced every link position printed by the 600 ms uninterrupted run byte-for-byte. The test confirms the simulation clock at each comparison, refuses a second save over the operator's checkpoint, checks the original file contents, and rejects a truncated file.

The new integration test passed in 0.53 seconds against the frozen `8c57ff6` headless executable and worker. Adding its process counters to the original full headless report gained 81 CLI lines and 122 regions, 18 scene lines and 23 regions, four server lines and four regions, and two distributed regions. A separate Ubuntu build of the current source covered 28/28 lines and 31/32 regions in the CLI's operator checkpoint code. Both builds used the unchanged release thresholds.

```sh
RUSTC_WRAPPER= cargo build --locked -p gz-physics-distributed --bin physics_worker
GZ_MULTI_TEST_WORKER="$PWD/target/debug/physics_worker" RUSTC_WRAPPER= \
  cargo test --locked -p gz-sim-cli --test native -- \
  --ignored --exact warm_physics_cli::authored_worlds_resume_from_offline_and_operator_selected_warm_cuts
```
