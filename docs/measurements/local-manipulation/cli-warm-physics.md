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

## Restarting the 30-joint walking robot on the ordinary local backend

An independently stepped Bevy robot supplied a reference while its twin returned from three workers to the ordinary local solver at 3,597 ms. At the cut it carried a pending −0.125 N·m joint effort and a [4.75, −17.5, 1.25] N chest force. The release test now launches the actual stock `gz-sim-rs --returned-robot-session` command on a separate copy of the saved state. Four independent CLI processes reproduced the printed positions of all 32 links, including the floor, at 3,597, 3,598, 3,617 and 3,820 ms. The CLI refused the different authored force schedule, the wrong physics backend and a 2 ms step. Every invocation preserved the original checkpoint bytes. The optimized Ubuntu test ran in 22.67 seconds, including all 533 existing full-physics comparisons and the reset. The seven frozen CLI processes gained ten lines and thirteen regions in the original headless report.

The native replay shows all 32 visuals for 224 consecutive milliseconds from the restored local CLI. Its input was the acknowledged scene from a newly connected operator after every individual physical step. [Open the walking robot in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11347%2F)
