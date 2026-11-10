<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Named IMU orientation references

M43 adds the SDF localization names `ENU`, `NED` and `NWU` to the existing
centered-IMU domain. This is Source preparation over accepted Main
`73e03b9f741689b1538898bde96bf975a49f068e`. M41/M42 are accepted after their
exact hosted and independent official qualification. The scoped original IMU
component capture and independent native/literal DATA passed. Rust tests,
coverage, hosted and installation qualification remain pending. The complete
integration still requires independent review before consolidated checks.

`examples/m43_imu_reference_frames.sdf` contains a free centered box, mass 2 kg,
diagonal inertia 1 kg m², zero gravity and a 1 ms physics step. The body starts
at `(2,-1,3)` with roll/pitch/yaw `(0.2,0.3,0.6)`. Its centered IMU has local
rotation `(0.1,-0.2,0.3)`, publishes at 100 Hz on `/m43/imu`, and uses frame ID
`rig/body/imu`. The example selects NED. Loading or taking two quiet CLI steps
is finite-load evidence, not acquisition, native or trajectory qualification.

The selected reference changes orientation only. Angular velocity and specific
force remain in sensor axes; no world-axis conversion is applied to those
reported channels. Existing physics, acquisition ordering, activation, rates,
noise streams, covariance fields, frame IDs and canonical message schemas are
unchanged. Orientation-disabled metadata keeps its established encoder behavior:
the Scene still computes its reading, while the canonical orientation field is
omitted. There is no runtime reference setter or new public sensor API.

## Reference selection and compatibility

The private `imu_reference::validate` returns one sealed `Frame`. It accepts only
CUSTOM/ENU/NED/NWU, zero custom RPY and an empty custom parent frame. Every named
reference additionally requires the absence of world spherical-coordinate
metadata, even when that metadata has heading zero. Legacy CUSTOM world
admission is unchanged; the named-only restriction must not affect worlds with
no IMU. Nonzero custom RPY, parent frames, unknown names, offset IMU positions
and all neighboring existing domain refusals remain unsupported.

Root retains the loader's existing initial reference expression and evaluation
order: `DQuat::from_array(pose.rotation) * DQuat::from_array(local.rotation)`.
For CUSTOM, the helper returns that exact quaternion directly, including signed
zero and quaternion sign bits. It performs no extra multiplication or
normalization. InitialScene clones and reconstructs the existing private IMU
component and its selected reference on a healthy loaded reset. Public IDs are
restored; tests reacquire fresh internal entity handles. Terminal fault/reset
policy is unchanged.

The named table comes from the pinned original `gz-sensors/src/ImuSensor.cc`
`SetWorldFrameOrientation`: with source world frame ENU, the target entries are
Euler `(0,0,0)` for ENU, `(pi,0,pi/2)` for NED, and `(0,0,pi/2)` for NWU.
The private helper follows `gz-math/include/gz/math/Quaternion.hh`
`SetFromEuler` scalar construction/normalization and the explicit native
Hamilton product of zero-heading world orientation with the table entry.
Named reference selection is independent of the sensor's initial orientation.
The existing sensor model already computes `reference.inverse() * world` from
`ImuInput.reference_orientation`; that measurement model is reused.

The original Sim IMU system first calls `SetOrientationReference` with the
initial WORLD sensor orientation, then, for an explicit reference-frame element,
calls `SetWorldFrameOrientation` with ENU world heading. CUSTOM with empty
parent leaves the initial reference intact. Native unknown-name fallback is not
newly admitted by this bounded Scene implementation.

## Regression and independent proof boundaries

The new private controls cover validation, named-only spherical refusal,
CUSTOM signed initial quaternion bits, fixed table axes and measured quaternion
sign behavior. The six public controls cover all named frames with actual
angular motion and healthy reset/public identity recovery; default versus
explicit CUSTOM exact readings; identical sensor-axis motion, seeded noise and
covariances across references; orientation-disabled metadata; demand/pause/rate
and removal/reconstruction; and spherical/custom/unknown/offset refusals.
Root separately converts the exact former NED rejection input into a strong
positive without changing adjacent negative controls.

Scene geometry checks retain the established strict absolute `<1e-9` native IMU
bound. Exact equality is used for paired physical states, motion-channel bits,
seeded-noise outputs, reset replay and unchanged CUSTOM readings. These new
checks do not assert cross-library quaternion bit identity or native RNG-stream
identity. Old native IMU fixtures, comparison bounds and regression bodies are
preserved.

The scoped original Sensors capture completed in 20.024 seconds with all sixty
child stages successful: seventeen configurations, 102 forced rate-zero calls,
102 observations/canonical packets and no suppressed calls. It records direct
WORLD sensor pose, WORLD acceleration/angular velocity and gravity, together
with actual sensor-axis setters obtained by one native reverse rotation. Both
setters are reset before every update because the component mutates them.
The independent retained native audit and fixed-grammar literal audit passed;
the checked-in fixture is byte-identical to the audited extraction. Its
provenance records the helper, exact recipe, actual provider/maps and proof hashes.

These inputs qualify component readings and canonical messages, not native
Physics, the original Sim system or Scene velocity-history acceleration. The
finite component cases need not match this example's physical trajectory. The
zero-variance mean-noise cases do not establish random-stream parity. Disabled
native orientation is absent on wire; its separately recorded getter is not a
Rust computed-orientation oracle.

Pinned `ImuSensor_TEST` controls `OrientationReference`,
`CustomRpyParentFrame` and `NamedFrameOrientationReference` supply source context
for the existing API. This increment admits only the named zero-heading domain
above; native unknown-name fallback, custom parent/RPY and geodesy are excluded.
The component program is `tests/native_imu_reference_frames.cc` in the Server
crate. Original argument-validation, concurrency-guard and helper-link failures
remain retained. The link correction uses the existing public Console API for
an owned log directory; it adds no private dependency or SDK rebuild.

The first complete normal hook stopped at Clippy's large-enum fixture layout,
before tests or coverage. The corrected constant schema borrows immutable output
records instead of copying a large enum payload. Every captured bit and replay
assertion is preserved; no lint allowance or heap allocation was added. The
borrowed fixture passed its independent fixed-grammar audit. A complete reviewed
normal retry remains required. That retry stopped at compilation of the former
NED regression because `WorldPose` has no `PartialEq` implementation. Its reset
assertion now compares both translation and rotation arrays with exact equality;
production types and numerical bounds remain unchanged. The failed source and
normal packets are retained, and no tests or coverage ran in either attempt.

The third normal attempt reached the no-fail-fast headless tests and found one
failure in a paired-motion regression. Its velocity replacement deliberately
rebases IMU history, so the old acceleration-pulse premise was invalid. The
control now adds the same one-step world-frame force to each reference mode
through the existing wrench input. The original strict `abs > 1` acceleration
assertion, paired physical/channel/noise/covariance comparisons and reset checks
remain. No production behavior or numerical bound changed. Native-free, RGB
and eighteen finite example controls passed within that failed attempt; full
normal, strict coverage and hosted qualification remain pending.

The new scalar replay exercises the actual reference helper and all 102 captured
setters through existing scalar/noise paths with strict `<1e-9` bounds. The
separate captured-value encoder replay compares all 102 original header/packet
byte pairs. Live canonical controls compare actual current Scene values, with
owned peer shutdown and the existing bounded lifecycle controls. These Rust
controls are declared source until the complete normal and hosted gates run.

Full normal/strict coverage, exact hosted CI, independent local/official DATA
and the M44 fresh eighteen-example package/install/browser/publication proofs
remain mandatory. This Source batch grants no execution or phase acceptance.
