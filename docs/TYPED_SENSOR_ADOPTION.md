# Typed SDF sensor metadata adoption

The workspace adopts the owner-recommended, hosted-green sdformat-rs revision
`575d4d9` (previously `4ed690b`). Transport `2a545f1`, Messages `2cea810e`
and Fuel `3747086e` remain pinned.
This imports reviewed sensor metadata and static-model graph fixes; it does not
implement their simulator measurements or rendering.

`Sensor.config` is authoritative. The scene accepts only `sensor.imu()` returning an
IMU configuration. Contact, force/torque, magnetic, altitude, pressure, NavSat,
airspeed, lidar and camera configurations fail a typed Unsupported boundary.
Future non-IMU variants are rejected by that same accessor. `Joint.sensors` is
checked before topology/backend lowering; a parsed force/torque record never
silently disappears or becomes a simulated wrench measurement.

The new tests first require valid parsing and nine distinct non-IMU configuration
variants, then require the exact capability error on both Rapier and TPE. A valid
joint sensor is rejected before either engine lowering. All existing IMU,
noise/rate/interest, metadata/reset/resource and C++ scenarios remain unchanged.
Malformed XML and unsupported simulator capability remain separate errors.

## Static model attachment identity

The producer now resolves static-model attachments to that model's implicit
`<scope>::__model__` sink, rather than collapsing them to world. That graph
identity does not choose physics static-body behavior. The consumer continues
to use the SDF static flag and owned resolved poses. A rotated static-model test
checks the sink, physical state and exact original identity/state after reset on
both engines. No TPE D23 quirk policy is changed.

The current producer's static_model_reference helper cannot replay its historical
reverse-parent assertion against later, legitimately edited semantics tests.
That failure is retained and reported to the producer owner. No guard is bypassed:
the exact historical `6ab311e` packet is exported and replayed instead, including all
61 native static-model observations and one unchanged original. Its corpus bytes
are identical in the adopted `575d4d9` revision. `planning/reference/sdf-adoption.json`
records source/library/binary/report hashes and qualifications. The existing SDK
and original binary remain qualified; no complete immutable P0 image is claimed.

Independent source/API/assertion review approved the bounded change after
private raw-export tests and two failing guard-removal counterexamples.
The receipt is `planning/reviews/typed-sdf-adoption.json`. The exact staged gate
passed 230 Rust tests, sixteen tooling checks and eleven C++ scenarios; every
production crate exceeds 95% lines/regions, workspace3513/3549 lines5366/5483
regions and8/8 changed scene lines. Normal protected hook, hosted verification
and publication remain separate requirements. This is bounded LOAD-01 integration; rendered
sensors, force/torque measurements, full resource/world loading and P2 acceptance
remain open. No complete original-case port or performance improvement is claimed.
