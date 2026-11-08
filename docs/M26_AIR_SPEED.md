<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Ideal loaded AirSpeed

M26 adds one bounded loaded Rapier air-speed sensor and canonical
`gz.msgs.AirSpeed` publication. This source feature does not change the accepted
M25 installation, archive, viewer or Latest download. Qualification and publication
require this increment's own normal/hosted/native and independent evidence.

`examples/m26_air_speed.sdf` is a self-contained zero-gravity `default` world.
Its sensor runs at 100 Hz, topic `/m26/air_speed`, frame `rig/body/air_speed`,
with an offset `(0,.5,.5)` and a 45-degree local yaw. The built-in AirSpeed
system is optional activation metadata, not a dynamic plugin host. Use a
compatible canonical consumer in the matching partition; the browser has no
numeric AirSpeed widget. No new CLI, reference/rate/wind service or worker is added.

The actual velocity is measured at the sensor point. Link-origin linear velocity
plus angular velocity crossed with the rotated local offset is inverse-rotated
once by the composite sensor-world orientation. It is neither link-centre
velocity nor an already-relative vector rotated a second time. Absolute world Z
sets native temperature/density; there is no initial-height reference subtraction.
The pure model accepts an already sensor-frame velocity vector.

The pinned native implementation deliberately uses float atmosphere evaluation:
`T=288.15f-.0065f*float(worldZ)`, density from
`1.225f/powf(288.15f/T,4.256f)`. Signed X velocity enters its mixed float/double
quadratic expression, followed by a float assignment and float multiplication
by 100 before promotion to protobuf doubles. The published number is Pa-scale
(about 61.25 at 10 m/s and sea level), although the upstream protobuf comment
says hPa. This implementation preserves actual native numbers and adds no
conversion. Temperature is Kelvin. Reverse flow has valid negative differential
pressure, stationary/lateral-only flow has zero. There is no variance field.
This reproduces a native troposphere formula, not a general atmospheric accuracy claim.

Only default no-noise settings are admitted: even a default-valued Gaussian kind
or hidden nondefault field on `none` is refused. World wind must be zero, model
and link wind disabled, and atmosphere metadata must remain the existing strict
default, including no explicit source/XML override. Non-Rapier, duplicate,
nested/joint-owned sensors, visualization, metrics, plugins/extensions and unsupported
metadata are refused before constructing the simulation. Finite body/frame inputs,
finite float casts/intermediates and positive temperature/density are required.
An invalid initial atmosphere refuses loading; an invalid runtime acquisition
becomes a typed ideal fault, replacing the prior valid sample and emitting no
stale packet. Nonfinite unused velocity channels are also invalid.

The existing native nanosecond sample scheduler is reused: finite nonpositive
rates are continuous; positive periods truncate to signed nanoseconds with a
one-tick minimum, and unrepresentable periods fail. No-demand, paused and non-due
steps acquire nothing; held samples retain acquisition time. Matching consumers
receive acquisition timestamp, frame and sequence under the current epoch.
Typed weak IO leases, queue limits, identity fences and global epoch cancellation
remain unchanged. Closing/removing the final owner retires that family's work.
Direct reset clears samples and resets cadence; loaded reset restores original
public IDs with fresh entity handles and publication state. Native rewind and
consumer-free removal timing are not claimed identical: upstream has a rewind
TODO and demand-dependent early returns.

The native helper source covers SensorFactory numeric/units/header cases and a
finite real Sim/DART rotated-offset witness, checking actual component poses and
point velocities against received canonical values. Independent scalar tests,
real Rapier motion, held-sample/cadence controls, typed faults, reset/removal,
metadata refusal and actual canonical wire/lease tests are separate evidence.
No C++/Rust trajectory, performance, RNG, optional-backend or geographic model
claim follows. Existing noise models and sensor families are unchanged.
