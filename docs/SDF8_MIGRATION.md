## M75: per-camera visibility

The existing 64 by 64 RGB/depth paths select visuals by nonzero visibility-flags
and camera-mask intersection. Low 28 bits plus the existing u32::MAX default are
supported; other reserved-bit values are refused. All world visuals and the
128-visual limit are validated before filtering. The generic snapshot path keeps
its strict default-visibility and box-only contract. Empty selected images are
real existing-background RGB [43, 44, 47]/+INF depth acquisitions under demand,
not no-demand shortcuts. The background is pinned Bevy default Source, with actual
whole-frame bytes still requiring qualification.
Calibration, canonical packet fields and independent cadence are unchanged;
camera metadata drift retires stale ownership and loaded reset restores masks
and visual flags. Native references are pinned source/test mappings, not a new
native renderer capture. [M75_DELIVERY.md](M75_DELIVERY.md) defines the bounded
scope and required actual software-worker/canonical-consumer proofs. Current
source is unqualified until the completed batch passes independent review and
consolidated mandatory checks; no image-equivalence or release claim is made.

Earlier checkpoints below retain their historical scope.

## M74: signed intrinsic skew

Finite signed K skew is admitted for the existing 64 by 64 RGB/depth camera
domain when its projection and sheared corners are representable. The pinned
native convention gives clip coefficient +2*s/width and optical coordinates
u=fx*X/Z-s*Y/Z+cx, v=fy*Y/Z+cy. Absent K keeps HFOV; absent P renders with K.
Explicit zero-translation P overrides render skew to zero. Both selected
calibrations remain validated before IO. Stereo translations remain refused.

CameraInfo retains the native publisher's zero K[1]/P[1] entries despite rendered
skew. This omission is derived from pinned CameraSensor::PopulateInfo source;
the old native wire capture used zero skew. Do not treat it as a new actual
nonzero-skew native capture. The retained skew35 CPU utility matrix supports the
projection math only. Actual software-worker/canonical-consumer controls are
required for this change; no native Ogre image or complete sensor parity is
claimed. [M74_DELIVERY.md](M74_DELIVERY.md) defines scope and acceptance.

Earlier checkpoints below retain their original scope.

## M73: separate intrinsic and projection matrices

The admitted 64 by 64 monocular domain now accepts explicit SDF lens projection
with finite positive p_fx/p_fy, finite p_cx/p_cy and zero tx/ty. K is selected from
present intrinsics or the unchanged HFOV fallback. Present P overrides rendering;
CameraInfo keeps the separately selected K and P. Absent P derives P=[K|0] and
preserves the existing request/message bytes. Both calibrations pass the current
representability check before IO. All existing skew/noise/distortion/pose and
camera-count bounds remain. [M73_DELIVERY.md](M73_DELIVERY.md) defines evidence
and acceptance. No stereo, native Ogre image parity or new protocol is claimed.

Pinned native CameraSensor::UpdateLensIntrinsicsAndProjection selects K then P;
PopulateInfo publishes them independently. DepthCameraSensor uses both methods.
The native code ignores rendering tx/ty, so this bounded phase refuses nonzero
translation. The old failed Ogre/Mesa helper remains failed and supplies no
runtime precedence result. Earlier records below retain their historical scope.

## M72 explicit-intrinsics development

The isolated M72 candidate selects explicit zero-skew K for supported 64 by 64
RGB/depth cameras when the SDF presence flag is set. Absent K retains HFOV
calibration; absent P is derived from selected K. Every explicit P remains
unsupported. Existing calibration representability, demand, ownership and reset
checks remain authoritative. See [M72_DELIVERY.md](M72_DELIVERY.md) for the bounded
scope and required new numerical, canonical, worker and lifecycle evidence.

The pinned native sources establish the chosen precedence contract. The old
Ogre/Mesa precedence helper failed and produced no runtime result. Existing CPU
projection and CameraInfo captures retain only their stated math/wire scope;
no native pixel or sensor parity is claimed. The historical investigation below
retains its original versions and evidence scope. M72 is not yet qualified.

<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# SDFormat consumer migration boundary

The workspace pins the independent SDFormat producer to `8b17dfd7171688626bcf0f045ba3493cd262f8a2`.
This revision preserves whether parsed camera intrinsics and projection were
present, which is needed to distinguish an omitted lens from explicit defaults.
It also accepts and retains metadata that the previous simulator consumer did
not see. Parsing a value is not a promise to execute it.

Before constructing a simulation, the consumer rejects direct world lights,
world joints and opaque extensions; nonzero world wind, a nondefault world
magnetic field and explicit atmosphere; model wind, any retained namespace and
opaque extensions; link lights, emitters and projectors; explicit collision density;
joint physics metadata, axis mimic, and the newly parsed ball, gearbox and screw
joint kinds; empty collision geometry; and sensor plugins, opaque extensions,
visualization requests and non-IMU sensor execution. A limited 64×64 RGB
HFOV-only camera may be retained as data for explicit snapshot extraction; it
does not execute a sensor system. Explicit intrinsics and projection remain
rejected pending native qualification. The implicit native world
wind and magnetic defaults remain admitted. The namespace guard rejects every
`Some` value, including an owned `Some("")`; this producer normalizes an empty
parsed namespace to `None`. Native creates a namespace component only for a
nonempty resolved value. Explicit density is rejected even
with manually specified inertia because native retains the full collision
metadata and this consumer does not yet retain it. Existing supported model
plugin, physics, joint, geometry and IMU checks remain in force. PBR materials
remain loadable as metadata, but box-scene extraction reports unsupported
material instead of dropping PBR behavior. A supported revolute joint's parsed pitch is retained in `JointDefinition`
(including loaded reset); the current revolute solver does not use it. Fixed
joints remain unsupported by the current scene joint admission. Native SDFormat exposes that value and the native
DART and MuJoCo adapters use pitch in their screw branches. Screw joints remain
unsupported. The pinned previous parser rejected the `visualize` sensor child;
this migration rejects true rather than silently discard its GUI-only meaning.
An explicit false value is accepted as inert: the producer stores a boolean and
cannot distinguish that spelling from omission. This broadens the old parser's
rejection of the false child without activating visualization.

Merge provenance is not itself rejected. Parsed flattened children are checked
by the same rules as direct children. Native SDFormat's ordinary merge path
copies selected children and discards the included model's own namespace and
`enable_wind`; link-level wind remains on the copied link and is rejected by the
existing link-wind check. This conclusion is limited to the ordinary parser
path, not custom native parser/interface-model semantics.

Focused controls cover parser-fed rejection, default-world stepping and reset,
merged child validation, preserved typed-sensor/visual lifecycle behavior,
and PBR extraction. These are bounded admission checks, not claims that the
rejected features or original sensor tests are implemented. The full staged
headless and renderer profiles, independent review, and exact hosted checks are
required before accepting the pin change.
