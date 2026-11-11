<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# CUSTOM IMU orientation relative to world

M45 was authored over provisional candidate 9a40768d and is now integrated over
locally qualified candidate a08f0b50 with its containment repair preserved. Accepted
Main remains 73e03b9f while M43/M44 hosted/official and publication closeout is
pending. This implementation has not run Rust or product qualification checks.
A separately reviewed original-component capture over accepted 73e and its native
and literal DATA audits have passed. Complete independent Source review and an
explicit reviewed integration over locally qualified a08 are required before Root
releases one provisional full LOCAL qualification. The original Source lane was
based9a; this integrated candidate preserves the a08 containment repair. An actual
accepted M43/M44 parent, exact hosted/official success and Main/readbacks remain
mandatory before M45 hosted publication or acceptance. This timing amendment under
D31/D32 grants no inherited parent proof or acceptance.
The slice retains the five-working-day scope including review and required gates.

The existing IMU configuration can select finite custom angles with an explicit
world parent:

```xml
<imu>
  <orientation_reference_frame>
    <localization>CUSTOM</localization>
    <custom_rpy parent_frame="world">0.3 -0.4 0.5</custom_rpy>
  </orientation_reference_frame>
</imu>
```

Omitting localization also selects CUSTOM through the existing DTO default.
There is no field-presence inspection or new parser API. Parent spelling is exact:
`world` is supported; empty or other parents with nonzero angles remain refused.
The new world-relative branch rejects any spherical-coordinate metadata, including
heading zero. IMU translation remains centered, with existing dynamic Rapier,
metadata, rate, noise and ownership bounds. No geodesy, other frame, runtime
reference setter, new gravity model or sensor-offset support is added.

The legacy empty-parent/all-zero CUSTOM path still returns the exact initial
body-times-local-sensor quaternion. It keeps its signed bits, quaternion sign,
spherical admission and existing defaults. Explicit CUSTOM/world zero instead
selects a world identity reference; on an initially tilted sensor this intentionally
differs from its legacy initial-relative reading. ENU/NED/NWU are unchanged and
cannot be combined with a world custom-parent override.

The implementation follows pinned original Sim Imu.cc: it first establishes the
initial reference, then the orientation-reference handoff calls Sensors
SetWorldFrameOrientation. Sensors CUSTOM/world replaces that reference with
worldRelativeOrientation multiplied by Quaternion(custom_rpy), in that order.
The initial sensor rotation is not multiplied into this new reference. The private
validated Frame stores angle bits immutably and uses the native Euler constructor's
scalar trigonometric order and normalization, followed by the existing explicit
Hamilton identity-world product. No generic sign or zero clamp is applied.

Only orientation reference changes. Actual WorldPose and sensor-axis acceleration,
angular velocity, gravity, noise draws and covariance behavior use the established
IMU paths. Orientation-disabled packets still omit orientation through the existing
encoder; a Scene reading is not a native disabled-orientation getter oracle.
No RNG-stream parity or original System/Physics trajectory equivalence is claimed.

The example `examples/m45_imu_custom_world.sdf` uses the M43 centered free body:
mass 2, diagonal inertia 1, zero gravity, 1 ms steps, body pose (2,-1,3) with
nonzero body and local sensor tilt. It publishes at 100 Hz on `/m45/imu`, frame
`rig/body/imu`, with custom angles (0.3,-0.4,0.5). It adds no resource asset,
framework, dependency, launcher or distribution requirement. It is not the native
component capture's direct-input trajectory.

The regression Source preserves old default/named tests and the unchanged M43
native fixture. The former private CUSTOM/world-zero refusal is a positive only
for effective CUSTOM; all named overrides and empty-parent nonzero negatives stay
closed. New public controls separately exercise explicit and omitted localization,
world zero, signed-zero angles, individual axes and mixed angles through actual
motion and healthy loaded reset. Independent axis-angle geometry checks use the
unchanged strict componentwise <1e-9 bound and do not stand in for a native capture.
Same-seed peers compare physical state, sensor-axis channels and covariances exactly.
A real one-step force supplies the nonzero acceleration stimulus: queued velocity
replacement deliberately rebases IMU history and is not an acceleration impulse.

Demand holds the last sample without consuming noise draws; pause/equal-time,
due cadence, removal and healthy reset use existing lifecycle seams. Reset restores
the configured reference, sample/noise/schedule state and public IDs while internal
ECS handles are reacquired. A terminal clock fault still refuses loaded reset before
observers or scene mutation. No template or spawn capability is expanded.

The fourteen new CUSTOM/world configurations have now been captured through
the original Sensors component, with six direct calls each: 84 actual observations
and canonical messages, 78 with orientation and six with orientation absent.
The reviewed capture records reference before/after handoff, RPY metadata, actual
WORLD and sensor-axis setters, readings, covariance and full wire. Independent
native DATA and fixed-grammar literal DATA audits both returned actual zero.
The exact fixture and provenance are copied under Scene tests/fixtures as
m45_native_imu_custom_world.rs and m45_native_imu_custom_world.provenance.json.
The fixture SHA256 is eb47abb6508d427a44f6129f1455fc6d15aac15a21df28ae44cbeba1c648e794.
These are actual original-component observations, acquired in a helper-only
checkout over accepted 73e; they are separate from the original 9a Rust Source-authorship
basis. Old M43 campaigns remain preserved. The new private Scene and Server
consumers are Source preparation only and grant no Rust pass or acceptance.

Acceptance still requires real native/literal DATA, production-path scalar replay
at unchanged sign-sensitive strict <1e-9, exact encoder/live canonical packets,
meaningful Scene and Server lifecycle controls, the full ordinary normal hook,
strict workspace/per-crate/actual-parent coverage, independent local and exact
hosted/official DATA, then normal Main/readback and accepted records. No new
package or browser refresh is selected. After its own acceptance M44 remains the
qualified download until a later explicitly selected distribution phase. AirPressure
reference altitude remains unselected, and joint-name filtering is already admitted;
neither is expanded by M45.
