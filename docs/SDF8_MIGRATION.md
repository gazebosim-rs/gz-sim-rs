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
visualization requests and non-IMU sensor execution. The implicit native world
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
