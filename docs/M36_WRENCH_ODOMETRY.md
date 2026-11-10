<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M36: wrench commands with actual ground-truth odometry feedback

This is the selected bounded integration slice in [M36_DELIVERY.md](M36_DELIVERY.md).
M36 is bound to accepted M34/M35 Main `1982ce86`, tree `a2302c57`.
Implementation, regression controls and documentation have completed independent
Source review. M36 still requires its own full normal hook, independent local
DATA, exact hosted CI and official audit before acceptance. No M36 checks,
hosted results or acceptance are claimed here. M32 remains the installed Latest. This phase adds a combined example and regression controls,
not another physics backend, publisher implementation or command API.

## Scene and canonical interface

`examples/m36_wrench_odometry.sdf` contains world `default`, model `rig`, link
`body`: a free 0.2 m box, mass 2, diagonal inertia 1, centered COM and link pose,
zero gravity, 1 ms steps, and model pose `(2,-1,3)` with yaw 0.6. It combines the
existing ApplyLinkWrench world plugin and noiseless dimensions=2 OdometryPublisher
model plugin. Both odometry offsets remain zero. The publisher period is 20 ms.
There are no joints or other models. Resolve actual public IDs from the loaded
`default::rig` and `default::rig::body`, rather than assuming allocator values.

| Purpose | Topic / frame | Canonical type |
| --- | --- | --- |
| One-shot command | `/world/default/wrench` | `gz.msgs.EntityWrench` |
| Add persistent command | `/world/default/wrench/persistent` | `gz.msgs.EntityWrench` |
| Clear persistent commands | `/world/default/wrench/clear` | `gz.msgs.Entity` |
| Odometry | `/m36/odometry` | `gz.msgs.Odometry` |
| Covariance | `/m36/odometry_covariance` | `gz.msgs.OdometryWithCovariance` |
| Transform | `/m36/transform` | `gz.msgs.Pose_V` |
| Parent / child frame | `m36/odom` / `m36/base` | Existing header fields |

Current ServerOptions partition/namespace/remap rules apply. No new telemetry,
wire epoch, message schema, viewer widget or launcher option is introduced.

## What the Scene controls establish

The new public Scene regression loads two identical combined worlds. One uses
`queue_apply_link_wrench` with `Once` and `Persistent`; the other receives the
same individual physical inputs in the same order on the same advancing ticks
through the existing `queue_link_world_wrench` seam. Every step compares actual
model WorldPose, authoritative link Velocity and the held/due odometry sample,
including stamp, revision, full quaternion and all numeric bits. This is an
integration comparison through existing inputs, not a second estimator or a
native Physics trajectory oracle. Both paths use the same existing backend and
publisher; the unchanged M34 literal-native controls separately bind estimator
arithmetic and canonical bytes to actual upstream observations.

The stationary warmup checks first-step initialization without publication and
the first 50 Hz frame at 20 ms. The nonzero initial pose retains its native
identity-history transient; the ten-entry mean has settled by that first frame.
Production input is already model WorldPose in world coordinates. Odometry
still executes native `globalOffset.Inverse() * worldPose * offset` scalar
operations even for zero offsets. Only native fixture MODEL-component inputs
receive the separate identity-parent conversion. Do not canonicalize signed
zeros or replace this computation with an initial-pose subtraction.

Four new Scene controls cover:

- Once force `[4,0,0]`, then additive persistent forces `[1,0,0]` and `[3,0,0]`,
  with no torque. One 1 ms application adds 0.002 to actual world x velocity;
  the one-shot does not recur. Each persistent step adds another 0.002.
- In a separate fresh world, once z torque 2 adds 0.002 rad/s; persistent z
  torques 2 and 4 add 0.006 rad/s per step. Linear and other angular axes remain
  zero. All analytic velocity comparisons use the existing absolute `1e-12`
  bound. No integrator-dependent position formula is asserted.
- Paused ticks freeze clock, physics and held sample while retaining one-shots.
  Clear-first maintenance removes persistent entries even while paused, including
  additions queued before that maintenance tick, but preserves queued one-shots.
  Loaded reset restores initial public identities, clears input and odometry
  history, advances epoch, and reproduces the finite input/history sequence.
  Removal with pending work retires both target owners and rejects the old ID;
  loaded reset reconstructs the original rig without stale commands.
- Invalid/nonfinite or wrong-target commands and full-buffer refusal preserve
  the held frame and prior valid one-shot. Exact zero commands fill the remaining
  admitted capacity; overflow cannot evict the physical command. The next step
  still matches the corresponding ordered direct-input control.

Filtered Twist is a ten-acquisition body-frame estimate. It is not instantaneous
world-axis Velocity. The tests compare the filter's actual history across matched
inputs and measure analytic physical velocity separately. The unchanged individual
M34/M35 bounds, intermediate-overflow ordering and endpoint/reset collision
matrices remain required; this slice does not duplicate or relax them.

## Server peer, lifecycle and limits

The separate Server/C++ regression lane must use real canonical inputs, actual
received **and** queued counter fences while paused, and bounded callback arrivals
on all three outputs keyed by acquisition stamp. It compares packets to the
actual Scene sample, including full orientation, shared headers and zero
covariance, without a second filter. Publish success and HasConnections alone
do not establish command consumption or output receipt.

Output demand loss must not restart odometry history. Reconnect requires fresh
due frames. Before loaded reset, drain the old phase's expected callbacks;
timestamps may repeat in a fresh epoch, but the wire has no epoch field. Phase
boundaries, actual Scene epoch and fresh owned leases distinguish those records.
Owned peer exit, endpoint retirement, removal and shutdown remain part of that
separate combined proof; this guide does not report their execution as passed.

The reconnect control uses the existing canonical `/world/default/clock` on the
same C++ Node/session as a bounded owned-queue fence. With ground-truth observers
still disconnected, advance one non-due step and require the actual 101 ms clock
receipt before reconnecting. This orders the owned IO work after the prior
100 ms ground-truth jobs; retain the strict zero-frame reconnect drain, then
advance 19 steps to the fresh 120 ms acquisition. This is a qualification of
that specific producer/queue/session path, not a general network guarantee of
cross-publisher receipt or permission to accept stale frames. No new observer
API, runtime telemetry, physics change or native replay is introduced.

The rig has a model plugin. Existing `prepare_model_template` refuses it, so this
combined example is reconstructed only by loaded reset. Unchanged earlier
plugin-free M35 Scene reactivation and inactive configured-M35 Server template
spawn refusal are distinct retained controls, not permission to clone this rig.

Qualification requires the accepted exact parent, complete independent Source
review, the full normal hook, strict workspace/per-crate/actual-parent coverage,
new real canonical peer evidence, exact hosted CI and independent local/official
DATA before normal qualified Main promotion. The existing 301-input/221-output
exact-native and 663-byte controls stay mandatory. No old whole-gate result is
credited to this new phase. Native component captures support only unchanged
Source-bound semantics; no native recapture, native Physics parity, performance,
installed package or browser claim is made. Fuel, paused work and frozen evidence
remain untouched. The selected budget is at most five working days including
review and all mandatory gates.
