<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M13 ideal altimeter

This increment admits one loaded-world, noise-free altimeter on the Rapier
backend. It uses the existing typed SDF configuration and canonical
`gz.msgs.Altimeter` schema. No schema, dependency or viewer change is required.
The implementation and regression sources require the consolidated qualification
and actual native observations specified in [POST_MVP_DELIVERY.md](POST_MVP_DELIVERY.md)
before acceptance. Source tests are not a claim that qualification has run.

## Measurement and acquisition

The reference is the resolved sensor world-Z coordinate at load. Vertical
position is its current world-Z coordinate minus that fixed reference. Vertical
velocity is world-Z velocity at the sensor point: link-origin linear velocity
plus angular velocity crossed with the world-rotated local sensor offset.
Scene physics already translates center-of-mass velocity to the link origin.
Sensor orientation does not rotate the measurement axis; gravity direction does
not define that axis. Inputs and derived scalar readings must be finite.

The fixed SDF `update_rate` uses the existing nanosecond `SampleSchedule`.
Nonpositive rates mean every advancing step, as for the existing sensor cadence;
unrepresentable rates are rejected. There is no runtime rate setter or new
service. Demand follows a matching canonical Zenoh subscriber. Explicit local
`InterestPolicy::Always` or Scene's manual acquisition mode retains its existing
meaning. No subscriber means no acquisition or deadline advancement. Zero-time
paused polls acquire nothing; publication deduplicates the acquisition epoch and
time. The first advancing due sample is stamped with its actual acquisition time.

The default topic is the existing scoped world/model/link/sensor path ending in
`/altimeter`. Explicit `topic` uses the maintained topic normalization and partition
rules. `frame_id` takes precedence over legacy `gz_frame_id`, then the scoped
entity name. The canonical header contains acquisition time, `frame_id` and the
attempt sequence. The payload contains vertical position, velocity and reference.
Reserved clocks, poses, services, controls, IMU, lidar, drive and joint command
endpoints are checked before the first altimeter declaration.

## Lifetime

No altimeter means no altimeter sampling/reset callbacks, interest resource,
native declarations or transport workers. The existing world IO owner lazily
owns declarations only while CPU leases exist. A queued job has a weak lease;
an unsent retired job cannot keep a declaration alive or reach a replacement
endpoint. Epoch checks cancel old reset-generation work. A put already admitted
by IO before the last CPU drop may finish, matching the retained lease contract.

Removing its model removes the sensor and sample, retires the last acquisition
callbacks and drops the CPU publication lease. Loaded reset restores the original
public identity, configuration and reference, clears samples and cadence, and
starts sequence zero in the new epoch. This is the simulator's established loaded
reconstruction contract, not a native rewind-parity claim. A reset metadata mismatch
is terminal; it must not silently publish a changed stream.

## Admission and evidence

The retained fixture is `crates/gz-sim-scene/tests/fixtures/m13_altimeter.sdf`.
Use the existing CLI world-loading path with Rapier and a subscriber for
`/m13/altimeter` in the selected partition. The fixture deliberately includes a
rotated, offset sensor with the retained centered-inertial admission. Nondefault noise,
visualization/metrics/custom metadata, a second altimeter and other backends are
outside this increment. Runtime altimeter spawning is excluded; unrelated typed
sensor admission negatives remain in force.

Regressions cover the finite ideal model, actual offset-point velocity, default
names, fixed cadence and demand, paused polls, loaded reset/removal, wire fields,
invalid/stale/future sample suppression, bounded queue failure, IO epoch/lease
retirement and real Zenoh matching/publication. Native reference source preserves
the upstream `AltimeterSensor` position/reference/velocity conventions and compares
actual native sensor output. The native simulator reference must create the sensor
while paused before advancing so the load reference has an explicit basis.
Physical integration differences must be reported separately from the ideal
measurement calculation; no new DART/Rapier physics parity is implied.

Noise, multiple altimeters, runtime spawning, reference/rate services, UI widgets,
other sensor families and broad backend qualification remain out of scope.
