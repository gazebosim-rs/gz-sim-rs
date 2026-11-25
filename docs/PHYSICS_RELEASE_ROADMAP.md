<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Physics release roadmap

Start with the local Rust/Bevy simulator. A user can already apply torque to an unchanged Gazebo robot, step and reset it, or replay a gripper lifting a parcel. The next delivery puts those same workflows in a qualified Linux package. Subsequent iterations each add one supported robot or task to that package. The engine, SDF integration, command-line tools and regression scenes live in gz-sim-rs. The browser application lives in gzweb-rs. [Run the preview](LOCAL_CPU_MVP.md) · [Approved physics design](physics_tdd.md) · [Worklog](MULTIFIDELITY_WORKLOG.md)

## Starting point and first release

The integrated CPU candidate passed 4,208 ordinary tests. Each local physics backend exceeded 95% line and region coverage. The unchanged rotor stayed within 0.215° of native Gazebo at eight sampled times. The gripper completed ten simulated seconds in 5.746 seconds with tier switching and 6.324 seconds always dynamic, including recording. These figures are baselines for those particular workloads. [Independent Gazebo comparison](measurements/local-manipulation/mvp-independent-robot.md) · [Paired gripper benchmark](measurements/local-manipulation/mvp-local-performance.md)

**1. Install and operate a local robot.** Install the Linux package in a clean environment, run the stock rotor, disconnect the controller across a checked step, reconnect, inspect the actual tick, reset, and replay the lift. An accepted command must advance exactly once. A stale command must leave the robot and clock unchanged. Verify the 610-record journal across one and eight Bevy threads and the gripper recording from the installed binaries. Ship the commands, example inputs, recordings, checksums and supported-platform notes together. [Current commands and viewer](LOCAL_CPU_MVP.md)

The first installable increment passed on Ubuntu 24.04.5 and 26.04.1. The lost-request and lost-reply cases reproduced exactly across reset, and the bundled gripper replay completed in gz-web-rs. The next qualification work is the integrated coverage gate below. [Installed package results and reproduction](measurements/local-manipulation/installed-robot-control.md)

The repository-wide gate is still red. On the measured CPU build the CLI needs 250 more covered regions, scene needs 1,066, and distributed needs 5,931. The software-renderer profile also failed coverage. Exercise real control and contact/reset paths first. Map feature-gated sources to the build that executes them and verify complete ownership across profiles. Use the existing worker recovery scenarios for the distributed deficit. Run the canonical CPU, renderer and hosted checks against the exact candidate. The repository requires strictly above 95% line and region coverage for the workspace and each production crate, plus its changed-line checks. While that work proceeds, each local increment can ship as a clearly labeled review preview. Official releases require the repository gates and port integration to pass. [Counts and missing instrumentation](measurements/local-manipulation/mvp-release-coverage.md) · [Testing policy](TESTING.md)

## Local releases in order

| Iteration | What the user can run | Evidence required to ship it |
|---|---|---|
| **2. Double pendulum** | Load the unchanged upstream double-pendulum SDF, release it under gravity, and reset. Support its implicit inertia and offset center of mass through the ordinary loader. | Compare angle, period and energy against native Gazebo at matched times. Vary the load and timestep, publish the stable mass/inertia range, and exercise invalid inertia at load time. |
| **3. Wheeled robot** | Load the upstream differential-drive model, drive straight, turn and brake on a plane. Support its ball caster and joint-state requests. | Compare wheel readings, traveled distance, yaw, stopping distance and lateral slip against the native world. Check low friction, lifted-wheel motion and replay after reset. |
| **4. Contact and collision course** | Run the rocking foot across a finite support edge, remove a support while paused, and send a fast link past a thin obstacle in hit and clear configurations. | Compare each support’s impulse and the resumed trajectory with full dynamics from matched states. Retain impulses only for surviving contacts, check grazing hits and clear passes, and report penetration, first divergence and p95 step cost. |
| **5. Controlled arm and gripper** | Run the upstream position-controller world, then command a multi-joint arm to reach around an obstacle, lift a parcel and set it down. Admit the world plugin and controller behavior needed by the stock example. | Check all six supported joint types under load, including coupled universal axes and screw travel per turn. Report tracking error, effort and speed limits, anchor drift, actual payload range, and obstacle contacts. Show a successful placement and a rejected or failed grasp. |
| **6. Automatic switching on a working robot** | Use the same arm and command journal in adaptive and always-dynamic modes. Park while settled, wake before support loss, and resume the task. | Compare the first resumed impulse, trajectory, momentum and command count. Repeat after reset and across thread counts. Rerun 10,000 switches and publish paired accuracy, elapsed time and p95/p99 on the same machine. |
| **7. Balanced articulated robot** | Use the existing 30-axis humanoid to step across an uneven support and recover from a push. | Use the level-floor run as the control. Compare foot reactions, slip, joint-limit excursions and recovery with continuous dynamics. Publish the tested disturbance range and show what happens beyond it. |

Iteration 2 is runnable on the physics review branch. Both stock pendulums passed the native angle, period and energy checks with the 120 Hz fixed-body contact setting. The next robot is the stock differential-drive model, starting with its caster joint and joint-state publisher. Official release qualification remains open. [Double-pendulum results](measurements/local-manipulation/double-pendulum.md)

## After the local rigid-robot release

| Iteration | Runnable deliverable | Acceptance |
|---|---|---|
| **8. Cable handling** | Lift a cable, weigh it and place it on a moving support. | Establish sag and stretch with one-way contact, then measure supported weight and equal-and-opposite reactions with two-way coupling at 1, 2 and 4 soft substeps. |
| **9. Cloth handling** | Grasp, lift and release a garment onto a finite table. | Check self-contact, strain, penetration, grip retention, and linear and angular momentum across measured stiffness and load ranges. Replay both a release and a missed grasp. |
| **10. Single-machine fleet** | Run and observe a warehouse population with up to 100,000 moving entities, swept queries and lifecycle edits. | Benchmark fixed density, public timestep and sensor demand against the TDD’s 100× real-time target. Report full-observation and sparse costs, p95/p99, memory and replay across thread counts. The full-observation target remains open. |
| **11. Optional GPU batch** | Run 2,048 mixed robot environments on a named device with actions and observations enabled. | Compare pose and impulse with the CPU reference, verify reset and repeatability, then report end-to-end throughput against 10,000 environment-steps/s, transfer costs, memory and p99. Prior specialized batch measurements provide baselines. |
| **12. Optional distributed deployment** | When a measured scene exceeds single-machine capacity, run that same world on two physical hosts. | Compare boundary contact with a single node through migration, duplicate and late messages, connection loss and owner recovery. Verify one owner and exactly-once inputs before scaling further. |

## What ships each time

Each iteration starts from the last runnable candidate. Keep its examples working and add one documented command using the installed Rust CLI, the SDF and any input journal, and a gz-web-rs replay that can be served from a fresh checkout. Choose quantity-specific tolerances against an analytical, native Gazebo or continuous-fine reference before tuning. Record the physical result, an adverse case, the machine and timing in the worklog. Tag and publish the qualified revision and its downloadable package together after the exact-commit checks pass. Keep the previous package available so users can reproduce a result or roll back.
