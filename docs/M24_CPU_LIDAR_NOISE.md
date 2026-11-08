<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M24 loaded planar CPU-lidar white noise

This increment admits finite Gaussian mean and finite nonnegative standard
deviation for the existing single loaded Rapier planar CPU lidar. All bias,
dynamic-bias and precision fields must stay default; `gaussian_quantized` is
refused. The shared Gaussian library, ideal model, physics and canonical
`LaserScan` protocol are unchanged. There is one optional private noise model
and stream per active sensor, not per beam or per consumer.
Lidar uses child elements, for example
`<noise><type>gaussian</type><mean>0.1</mean><stddev>0.2</stddev></noise>`;
its noise type is not an XML attribute. The maintained parser rejects hidden
noise fields before Scene admission; typed private controls also check the
Scene guard independently.

The selected scope and evidence boundary are in `M23_M24_DELIVERY.md`. Source,
regressions and native helper are not executed results. Acceptance requires the
finally qualified M21/M22 parent, independent review, the full normal hook,
strict coverage, exact hosted CI, focused native observations and independent
retained DATA. M20 remains the installed preview; no archive is relabeled.

## Deliberate minimum-range boundary

Every Gaussian declaration requires **range_min=0**, including an empty,
zero-valued or near-zero inactive declaration. The existing default `none`
configuration still admits nonzero minimum range and preserves M7's exact ideal
clipping behavior. Nondefault fields on `none` remain refused.

This boundary follows the pinned `gz-sensors` revision
`8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`:
`CpuLidarSensor::GenerateRays` starts rays at range_min, while
`SetRaycastResults` maps valid fractions in [0,1] to distances in [min,max].
It classifies out-of-range fractions as infinities before applying noise only
to finite entries. M7's accepted origin-based query and clipping for a hit
below a nonzero minimum is a different domain. Requiring zero minimum for the
new Gaussian mode makes its physical ray origin agree with native, without
changing old ideal behavior, allowing negative-infinite protocol values or
claiming parity for out-of-range native fractions.

## Activation, samples and failure

For the admitted mean/stddev subset, native activates noise only when
`abs(mean)>1e-6` or `stddev>1e-6`. Equality is inactive. Inactive declarations
allocate no noise state. Once active, both exact configured parameters are
preserved; the activation threshold is not separately applied to each draw.

Finite raw physical hits in [0,max] receive noise in beam order, with native
`dt=0`, before clipping to [0,max]. A missing return stays positive infinity
and consumes no draw. Nonfinite noisy output is rejected before clipping can
turn it into an apparently finite result. Range-resolution metadata does not
introduce quantization. All existing 2–128 beam, planar, resolution-one,
visibility-mask, metadata, rate, single-sensor and backend restrictions remain.

The load seed and stable public sensor ID own the independent stream. Paused,
unrequested and non-due acquisitions consume no draws. Healthy direct and
loaded resets reconstruct the initial seeded state and clear scan/deadline;
removal drops state and callbacks and loaded restoration recreates them.
Deterministic Rust replay is not equality with native's global RNG/reset bytes.

The existing terminal `LidarFault` policy remains. On any failed acquisition,
the previous scan is removed and no partial replacement is installed. The
simulation is terminal, so an old scan cannot be published after a failed tick;
subsequent steps still fail. There is no finite clamp of an overflowing noise
result, silent scan retention, retry, runtime noise setter or fault recovery
service.

## Source controls and native capture

The old ideal/refusal tests remain. The formerly rejected positive Gaussian
case becomes a still-unsupported quantized case, while explicit regressions
cover Gaussian plus nonzero minimum, hidden fields and activation boundaries.
Analytic Rapier walls verify signed means and lower/upper clips. A narrow wall
gives finite/missing beams to verify beam order and skipped random draws against
the existing deterministic scalar model. Other controls cover different seeds,
coexisting noisy IMU streams, demand/rate/pause, reset/removal and terminal
overflow with retirement of the last complete scan. Private failure injection
does not add a public runtime configuration API.

Actual Zenoh tests require `gz.msgs.LaserScan`, exact acquired range bytes,
world pose, count, frame/time/sequence and zero-min boundary values. Reset and
reconnect must continue the correct seeded acquisition sequence. Existing
bounded queues, retirement, endpoint ownership and invalid-packet controls are
unchanged; no new transport abstraction is added.

`crates/gz-sim-scene/tests/native_cpu_lidar_noise.cc` accepts `MODE MAPS_PATH`.
Modes are `none`, `mean`, `lower`, `upper`, `inactive`, `active-boundary` and
`white`. It uses the actual `CpuLidarSensor` provider, five generated min-zero
rays and bounded `SetRaycastResults` fractions. A middle miss remains infinity
(printed as a null range with an explicit miss flag). Deterministic modes emit
two scans; white emits 100, with the retained upstream five-sigma moment bounds
for each finite beam. Maps are captured once while the actual sensor is alive.
Root binds the compiler/provider/SDK and owns serial execution. No physics or
rendering plugin rebuild, cross-language RNG sample identity, trajectory or
performance claim is implied.

`examples/m24_cpu_lidar_noise.sdf` supplies a static scanner and narrow wall,
100 Hz, `/m24/scan`, `laser_frame`, mean 0.1 m and standard deviation 0.2 m.
No GPU or multilayer lidar, point cloud, intensity model, dynamic bias,
quantization, runtime noise/rate/reference service or new viewer widget is added.
