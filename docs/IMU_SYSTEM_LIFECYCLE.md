<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Demand-driven IMU scene callbacks

The Scene loader installs IMU sampling and IMU reset callbacks only when the
loaded scene owns live private Imu state. Sensorless worlds register neither
callback and do not allocate ImuInterest. Setting empty transport interest on such
a world keeps that state absent. Shared pose synchronization and body-input reset
work are independent and remain installed.

Removing a model with sensors preserves the callbacks while any other live IMU
remains. If last-consumer retirement may be needed, pending schedules initialize
first, then the complete hierarchy/binding/body preflight repeats: Bevy's
ScheduleBuilt observers can synchronously mutate the World. No schedule build
intervenes between that final validation and body removal/despawn. Callback
retirement follows the commit, so its build observers see the removed model/body.
It releases per-system query/condition state, the private installation marker and
transport-interest buffer.

A schedule failure during initial preparation returns before internal model/body
mutation. If despawn observers delete the other sensor and expose a pending
schedule error only after commit, CommittedSchedule reports that removal already
happened and marks the run terminal. The API never reuses body snapshots across
schedule events or weakens its backend-removal invariant. Arbitrary user Local
initializers retain the core initialization contract.

Loaded reset reconstructs initial sensor state and reinstalls these two callbacks
before reset observation. Removing and restoring repeatedly must not accumulate
callback registrations or lose initial public IDs. The original world blueprint
remains shared reset data; this is not a claim that the original definition is
forgotten or process RSS falls to zero.

Inactive or unconnected live sensors still require the native continuous velocity
history, so they remain consumers. Rate, activation, interest, pause, noise,
reference orientation and sample timing semantics remain unchanged. The private
Imu component is constructed by the loader; this slice adds no public runtime
sensor-spawn or dynamic system API.

The core exposes pending schedule initialization and registered tick/reset counts.
Counts use Bevy's graph, including registrations added after a schedule previously
ran; the cached executable count can be stale until rebuilding. Six new Scene controls cover absent registration/interest, shared consumers and
three remove/reset cycles, continuous inactive history, schedule-error preservation,
committed-error reporting and no unneeded sensorless initialization across all four
engines. A seventh schedule-built observer/body mutation control runs on the three dynamic
engines with initially dirty and already-built schedules. One new core control
covers initialization without callbacks and pending-registration diagnostics.
Existing core, Scene, IMU and server/native client checks remain required.

This is a bounded ACT-01 consumer slice. It does not remove empty server ingress
or publication calls, retire transport worker Nodes after the last consumer, prune
joint/physics callbacks, or establish full zero-idle compliance. Collective
condition state retains the limitation in BUILTIN_SYSTEM_RETIREMENT.md. Equal-work
CPU/memory and native performance comparisons remain required; no speedup is
claimed from registration/resource absence alone.
