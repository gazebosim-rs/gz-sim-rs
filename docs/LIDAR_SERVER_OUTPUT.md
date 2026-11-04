<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Bounded canonical planar lidar output

M7 source integrates one loaded Rapier lidar with the Server's existing world
publication IO owner. It introduces no transport facade, new worker or renderer.
Scene supplies validated `LidarPublication` metadata and each acquisition's
`LidarSample` time, epoch, world pose and continuous metric ranges.

The formatter emits the maintained canonical `gz.msgs.LaserScan` schema through
Prost: acquisition stamp, frame and sequence, acquisition world pose, horizontal
angles/count and range limits, one zero-elevation vertical row, ranges and one
zero intensity per ray. Positive infinity represents no return. It rejects
malformed metadata, mismatched counts, invalid poses, nonfinite/out-of-domain
ranges and unrepresentable timestamps. No epoch field or reflectance model is
invented. Message admission remains bounded by the existing payload/queue limits.

With ordinary subscriber policy, actual Zenoh matching and the explicit viewer
hint are unioned before ticking. No interest selects an explicitly empty Scene
interest set, so no ray acquisition runs. The explicit `Always` policy keeps
manual acquisition semantics. The live CLI supplies the viewer hint before every tick; standalone Simulation
callers clearing interest select intentional manual acquisition. Paused ticks produce no new scans.

CPU streams hold weakly queued IO leases, following the existing IMU lifecycle.
Publishers, matching listeners and discovery tokens are owned only by the world's
publication thread. Last-consumer removal releases the CPU lease; IO reconciles
and retires declarations, and queued work cannot extend that lifetime. Work
admitted before retirement may complete. Reset preflights initial endpoints,
verifies complete restored metadata/identity, resets sequences and uses the
existing epoch fence to discard old queued work. Changed reset metadata stops
the run. Shutdown retires streams and joins the existing IO workers.

New regression source covers canonical field/value/protobuf fidelity, strict
metadata/sample rejection, real upstream Zenoh scan cadence at 1/10/20 ms,
pause, reset and stable identity, removal/restoration, viewer/subscriber demand,
queue limits, sequence wrap, malformed ownership and terminal reset drift.
Separate IO controls cover real matching, last-lease retirement, stale queued
jobs, exact canonical attachments, closed-session errors and epoch cancellation.
All are unrun pending whole-feature independent review and consolidated required
verification. Existing IMU/RGB/depth assertions and quality gates are preserved.
