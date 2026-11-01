<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Explicit server shutdown and frame admission

`Server::shutdown` terminates frame admission on that server object, releases
CPU output leases and joins IO outside stepping. Subsequent `prepare_io`, `tick`
and `update` calls return `WorldClosed` before reset preparation, simulation,
control polling or publication. Repeated calls preserve the scene, pending
steps, epoch, samples and counters. `into_simulation` can still transfer the
preserved scene to a caller.

An unexpected underlying Zenoh session close retains the existing simulation
behavior: advancing frames can continue while IO failures are reported. The
explicit application flag distinguishes these cases without querying native
routing state or adding a worker, queue or shared lock.

The baseline demonstrated post-shutdown mutations in five states: unpaused,
paused, pending reset with IMU, pending reset without IMU and a prepared reset.
Each case ran both frame APIs three times. Calls could advance time and poses,
run paused callbacks, consume pending steps or commit a reset before reporting
an error. The new regressions require typed rejection and unchanged snapshots
for all thirty calls, including schedule witnesses and queued control state.

Two existing Rust assertions previously expected clock/root publication-failure
counters to increase after application shutdown. Prior independent review
approved replacing only those expectations with repeated typed rejection and
unchanged time, state, pending work and counters. A rejected frame never attempts
publication, so it does not manufacture a clock failure. Earlier assertions and
the direct-session-close advancement tests remain unchanged. No C++ fixture,
wire assertion or numerical tolerance is retired.

The isolated guard passed all 149 retained server tests, Clippy and formatting,
with independent source and evidence review. Integration with the actual SDF
Git upgrade also passes the seven terminal and six producer-admission controls.
Normal staged coverage and exact hosted/source qualification remain required
for the integrated revision.
