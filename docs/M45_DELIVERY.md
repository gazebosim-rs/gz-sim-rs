<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M45 CUSTOM world-relative IMU angles — provisional local development contract

> Root Source-authorship release: independent contract 2a64392c and native case
> proposal 3b78e3e7 were Source CLEAR before this isolated checkout was created.
> Leased Source preparation is now released; every execution/publication gate
> below stays held for Rust execution. Accepted-parent identity remains a future
> binding. Independent artifact-only capture and literal extraction were separately
> reviewed and released over accepted 73e; their retained DATA audits passed.
> Source-only extension 5872e794/releasec4826b3a now permits exact audited fixture
> insertion and the two private consumers, with no formatter/test/build release.


M45 is selected for bounded provisional local development under D31/D32. Root
created the original Source lane after independent contract review and separately
released the reviewed artifact-only native capture, literal extraction and Source
consumer extension. The new integrated lane preserves exact locally qualified a08.
This document itself releases no formatter, test, build, network, qualification or
publication action; complete Source/tree/runner review and serial Root release
remain required. M43/M44 hosted/publication closeout continues separately.

The original Source-authorship basis was provisional9a40768d/tree5872e52f.
The integrated local-development candidate now uses immutable locally qualified
a08f0b505bb775cbc02974e098084e763da62a82/tree5c22f0fdd2ddb9112036e77da5df2c7711cb0d43,
preserving its complete containment repair. Neither Source parent is accepted.
Accepted Main remains
73e03b9f741689b1538898bde96bf975a49f068e. M43/M44 are not accepted merely because
all local proofs passed; exact hosted/official and publication prerequisites remain.
Development may overlap that closeout. The reviewed timing amendment 5d823a75,
adopted as ab9586b5, permits one full provisional LOCAL normal hook and retained
independent DATA only after the complete Source batch, integration/rebase, actual
tree and runner receive independent review and separate serial Root release.
M45 hosted publication and acceptance still require actual accepted M43/M44, exact
hosted/official and Main/readbacks. Record provisional Source parent a08 and accepted
ancestor73e separately. If the accepted parent changes, no local result transfers
automatically; review the new integration and affected checks. Each phase is bounded to five working days including review and mandatory
gates, without an ETA or waived gate.

## Selected behavior and closed boundary

Admit finite, existing XML-representable custom roll/pitch/yaw when effective
localization is CUSTOM and `custom_rpy` has exact parent_frame `world`. Omitted
localization defaults to CUSTOM and is included. The DTO has no presence bit;
validation must not add parser presence fields or inspect raw XML to distinguish
literal localization omission. Existing explicit parent string already identifies
the selected branch.

Keep the centered sensor position domain, real rotating body/sensor orientation,
existing dynamic Rapier ownership and all prior sensor metadata/noise/rate bounds.
New CUSTOM/world admits no spherical-coordinate metadata, including heading zero.
No arbitrary parent frame, geodesy, translated sensor origin, new gravity model,
public runtime setter, reference reconfiguration or frame resolver is selected.

Preserve the legacy effective CUSTOM/empty-parent/all-zero branch exactly: return
its existing initial body-times-local-sensor quaternion directly, including signed
bits and quaternion sign. Its accepted spherical metadata behavior is unchanged.
Nonzero angles with empty parent and every other parent stay refused. ENU/NED/NWU
reference arithmetic and named-only spherical refusal remain unchanged; named
localizations with a world custom-parent override stay closed. Unknown names,
malformed/nonfinite/subnormal XML values and existing offset/noise/rate/ownership
refusals remain typed under their established admission paths.

Explicit CUSTOM/world zero is newly admitted and has WORLD reference semantics;
it must not be folded into legacy initial-reference semantics. Mixed signed zeros
in this new branch follow the actual native constructor/product operations, not a
new zero clamp. Preserve every old fixture, old numerical bound and production path.

## Private handoff and exact arithmetic

Keep the existing private APIs in Scene src/imu_reference.rs:
`validate(&sdformat::Imu, bool, &str) -> Result<Frame, LoadError>` and
`reference(Frame, DQuat) -> DQuat`. Extend only the sealed private validated Frame
with an immutable CUSTOM/world payload, preferably three u64 angle bits so its
Copy/Eq identity remains exact. No exported type or core ImuInput change is needed.
The validated payload must contain only finite accepted angles and cannot acquire
runtime setters. Use the existing shared SceneContext spherical-presence Boolean.

Original Sim Imu.cc first seeds SetOrientationReference with the initial sensor
world quaternion, then calls SetWorldFrameOrientation(identity ENU heading) when
the orientation-reference element exists. Sensors ImuSensor.cc CUSTOM/world
replaces reference with worldRelativeOrientation * customRpyQuaternion. It does
not multiply the initial sensor quaternion by the custom quaternion.

Preserve Quaternion::SetFromEuler's exact scalar order: half roll/pitch/yaw,
its w/x/y/z trigonometric products, native norm accumulation and per-component
normalization; then the explicit native Hamilton identity-world product. No glam
Euler shortcut, sign normalization, epsilon, generic zero canonicalization or
output estimator. Keep existing named/table and legacy default operation order
byte-exact where practical. The real current WorldPose still enters the unchanged
core IMU orientation calculation; sensor-axis acceleration/angular-velocity inputs
and gravity path are unaffected by reference choice.

Clone/reset already stores the resulting reference in the private Imu component
through InitialScene. Healthy loaded reset must reconstruct the configured reference,
clear acquisition sample/history/noise/schedule through existing seams, retain
public IDs/epoch behavior and reacquire internal ECS handles. Terminal fault reset
refusal remains exact and nonmutating. Scene spawn/template admission is unchanged;
no plugin or arbitrary reconstruction capability is added.

## Leases and hooks after separate Root authorship release

Light owns only these Scene/example/guide leaves in the new isolated lane:

- crates/gz-sim-scene/src/imu_reference.rs: private validated admission/reference.
- crates/gz-sim-scene/tests/private/imu_reference.rs: exact private old-input
  disposition and focused constructor/branch/bit/refusal controls.
- crates/gz-sim-scene/tests/imu_custom_world.rs: new public lifecycle/motion controls.
- examples/m45_imu_custom_world.sdf: existing centered dynamic body/sensor geometry,
  explicit world custom RPY, no asset or new plugin dependency.
- docs/M45_IMU_CUSTOM_WORLD.md: precise feature/native proof/reproducibility limits.

Actual native and independent literal DATA have now returned zero. Light owns
the new Scene tests/private/imu_custom_world_native.rs; Messages owns the new
Server tests/private/imu_custom_world_native.rs. Root has copied the exact audited
fixture and provenance and owns both append-only cfg(test) hooks. These are Source
preparation leases only. The generated types are actual observations; no numeric
placeholder or product execution is permitted.

Messages owns new Server public/private canonical controls and artifact-only native
case/reference/capture/extractor Source, with exact leaves assigned by Root before
authoring. This includes one reference helper and an existing maintained canonical
transport peer extension/new private test; no public Server API is expected.

Root owns shared Scene/Server registration, any exact former public refusal slot in
Scene tests/imu_reference_frames.rs, planning, integration, provenance insertion,
normal/CI/local/official qualification and publication. Shared lib.rs changes must
not be authored in feature lanes. SDF owns independent whole Source review and any
separately released retained DATA readers. Native and heavy jobs are serialized by
Root under existing floors/caps; parallel Source leases do not authorize execution.

No Cargo/lockfile/workflow, dependency, wire schema, process protocol, public API,
Fuel, viewer, package, launcher or generic framework change is leased.

## Meaningful regression obligations

Preserve the existing private nonzero-RPY/empty-parent negative. In the existing
private localization matrix, explicit parent-world zero changes to a positive only
for effective CUSTOM; every named-localization case stays refused. The matching
M43 public explicit CUSTOM/world zero refusal becomes a real load/motion/healthy
reset positive, not deletion of coverage. New omitted-localization/world positives
must exercise the DTO default independently from explicit CUSTOM spelling.

Public controls should cover explicit/omitted CUSTOM/world zero, signed-zero and
nonzero roll/pitch/yaw combinations; initial body and sensor tilt must be nonzero
so initial-relative substitution is observable. Compare named/legacy/world branches
under identical physical inputs: only orientation reference may differ. Existing
sensor-axis motion, six-channel noise and covariance semantics must remain identical
under a common seed. Use actual force integration when requiring nonzero measured
specific force; queued velocity configuration deliberately rebases IMU history.

Require orientation enabled and disabled behavior without inventing a disabled
native getter oracle. Keep noise means/quantization deterministic controls separate
from RNG stream parity. Preserve demand holding the last IMU sample, due cadence,
paused/equal-time behavior, disconnect/reconnect, healthy loaded reset/public-ID
recovery, removal and terminal refusal. Explicit world plus spherical heading-zero,
unknown parent, empty-parent nonzero, named custom override and nonfinite/malformed
input remain meaningful negative controls.

Example proposal is the existing M43 centered body layout at (2,-1,3), nonzero
body/local sensor rotations, zero gravity, 1 ms step, mass/inertia unchanged,
100 Hz /m45/imu with frame rig/body/imu and custom RPY (0.3,-0.4,0.5), parent world.
Exact example/topic agreement with Messages precedes its Source freeze. The example
is not a native component trajectory oracle or a new distribution requirement.

## Fresh component, functional and qualification proof

Use the already available original Sensors IMU component/provider and existing
M43 compile/load/SetOrientationReference/SetWorldFrameOrientation/Update recipe.
No original System/Physics execution or SDK build is needed or claimed. Messages
authored the helper and capture procedure; Root ran the one independently reviewed
fourteen-configuration/six-call capture over accepted 73e.
Independent native and fixed-grammar literal DATA audits both returned actual zero:
14 configurations, 84 observations and canonical packets, 78 orientation-present
packets and six absent orientations. The generated fixture is
crates/gz-sim-scene/tests/fixtures/m45_native_imu_custom_world.rs, SHA256
eb47abb6508d427a44f6129f1455fc6d15aac15a21df28ae44cbeba1c648e794;
its byte-exact provenance copy retains the separate capture and audit identities.
This is scoped original-component evidence, not a Rust result or phase acceptance.
No M43 case is replayed merely to manufacture fresh credit.

A reviewed new consolidated component capture must bind raw SDF metadata, effective
localization, explicit parent, custom RPY bits, initial reference, actual native
reference getter after handoff, WORLD ledger, actual sensor-axis setters, enabled
orientation/noise configuration, times/flags and acquired optional orientation,
other motion channels, covariance/header/full wire. Native reference must come from
the real component, never a copied math implementation. Preserve input/world-to-
setter handoff checks as a separate domain proof. Reuse bounded maintained capture,
retained native DATA, literal generation and fixed-grammar DATA mechanisms without
new protocols/frameworks or invented numeric values.

After actual native and literal DATA zero, exact immutable fixture insertion must
exercise production validate/reference and scalar/noise paths. Legacy CUSTOM initial
bits stay exact; all applicable component channels use the unchanged componentwise,
sign-sensitive strict <1e-9 contract. Exact captured-value encoder and live canonical
packet controls stay byte-exact. Real Server/C++ functional proof must compare actual
Scene reading/full Imu wire under matched physical inputs, noise/covariance settings,
reference variants, orientation enable/disable, demand/reset/removal and owned shutdown.
No hidden field, RNG parity, numerical trajectory, geodesy or native Physics claim.

Acceptance requires independent complete Source review, actual new native/literal
DATA, meaningful Scene and canonical functional controls, one full ordinary normal
hook including strict workspace/per-crate/actual-parent coverage, independent local
DATA, exact hosted CI and independent official DATA, then normal Main/readback and
accepted record publication. Every available failure is retained and diagnosed;
no partial component/source/finite-load result substitutes for a full gate.
M45 is not accepted by this preparation contract or by M43/M44 acceptance.

No new distribution/browser replay is selected for M45. After its own acceptance,
M44 remains the latest qualified download until some later explicitly selected
package phase; M45 adds Source/runtime compatibility, not new release assets.

## Explicit nonselections

AirPressure nondefault reference altitude remains unselected: the field exists in
SDF but its original component Load/System does not hand it to SetReferenceAltitude.
Do not introduce that absent SDF-setter behavior as a compatibility claim.
JointStatePublisher joint_name subsets and duplicate deduplication are already
admitted and tested, including public-ID-sorted canonical selection. No duplicate
refusal or new phase is invented. There is no selected M46 in this contract.

## Root shared Source integration

The former M43 explicit CUSTOM/world-zero refusal now loads that unchanged fixture,
checks actual orientation geometry during motion, preserves the public sensor ID
across healthy reset, requires a fresh entity and exactly repeats its sample.
All remaining named, spherical, unknown-parent and nonzero empty-parent refusals
remain covered. The new `imu_custom_world` canonical peer is explicitly selected
only by the existing Zenoh profile. Both normal and retained headless peer inventories
require its actual evidence; renderer selection remains unchanged. Dispatch and
missing-input regression controls include it.

The native-free finite CLI qualifier appends the new example after the unchanged
eighteen-example prefix, requiring its complete body output and retained invocation,
fixture bytes and modes. Its controls reject missing, duplicate, nonfinite, wrong
pose and substituted evidence. This nineteen-example development qualification does
not alter M44's eighteen-example archive or select a new distribution.

The shared and consumer Source batch requires complete independent review.
Actual native/literal DATA now permit the reviewed Source-only module insertion;
Local Rust execution still requires final Source/tree/runner review and separate
Root release under the provisional timing amendment; hosted publication and
acceptance require the actual accepted parent.
