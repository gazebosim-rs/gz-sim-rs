<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Independent loaded logical cameras

M53 is in Source development. The fresh component capture and independent native
and fixed-literal DATA passed. The first full normal run found five invalid
fixture premises; a reviewed fixture-only correction is awaiting its full retry.
Strict qualification and release acceptance remain pending.
See [the delivery contract](M53_DELIVERY.md). Its locally qualified M52 parent is
unaccepted. Installed M40 Latest and all accepted capabilities are unchanged.

Admit zero through eight otherwise-supported ideal logical cameras in a loaded
Rapier world, with at most 128 unique flat top-level model origins total, including
camera-owner models. Eight is a resource cap, not an upstream limit. A world with zero
cameras owns no camera sampler, reset callback, interest resource or publisher leases.

Each camera has immutable public identity, full WORLD-scoped name, original
configuration and two distinct resolved endpoints. Loaded WORLD names obey the
existing parser restrictions: SDF 1.12 names containing `::` are refused. Preserve
the complete accepted WORLD prefix when resolving owner metadata. The pure
identity validator separately guards multi-segment prefixes without expanding
XML admission. All image and frustum keys
across all owners, remaps, prepared bindings and other occupied stream families
must be validated before any publisher lookup, declaration or reuse. At eight
owners there are at most sixteen logical-camera endpoint leases. Existing public
publication metadata literals and the prepared tuple's original three fields
remain compatible.

Visibility tests model origins against the existing six planes, using unchanged
corner/center operation order, the native `> 1e-6` normal-length threshold and
strict negative signed-distance exclusion. Zero lies inside. Geometry extents,
occlusion, rendering and collision shape do not affect membership. Output names
are lexical bare model names, including the owner when visible; poses contain
actual camera WORLD pose and each selected model's full camera-relative pose.
No visibility epsilon or alternative solver is added. Existing pure rotated-pose
`1e-14`, native full-pose `1e-12`, old nominal membership and exact canonical
Header/full-wire assertions remain separate obligations.

Image OR frustum demand activates one acquisition for that owner and generates
both outputs from that one sample. Owners retain independent deadlines and held
frames. Image Header has stamp, frame and sequence; frustum Header has stamp and
frame without sequence. Empty images are valid. Pause/equal-time polls take no
new sample. Disconnecting an owner clears its cached sample. Without demand its deadline
does not advance. Fresh geometry belongs to its acquisition stamp; held frames remain
exactly held. Existing bounded queues admit each endpoint independently; no atomic
remote delivery promise is supplied.

Loaded reset restores original owner/public IDs/configurations with new entity
handles, fresh deadlines and sequence/epoch behavior. Bare reset resets sampling
without restoring world state. Removing one camera or target leaves an active
peer valid; last-owner teardown retires the camera resources. Wrong identities,
metadata, parent bindings, malformed model input and terminal errors fail under
the existing strict rules, without partial endpoint acquisition or a stale
successful frame. Retain the old duplicate-name/same-topic-pair refusal and add
a separately observed distinct-owner physical-motion/reset positive.

The [Source example](../examples/m53_multiple_logical_cameras.sdf) has one physical
body and two distinct sensor poses, fields of view, frames, rates and endpoints.
The forward camera requests 100 Hz and image topic `/m53/forward/image`; the left
camera requests 50 Hz and `/m53/left/image`. Each frustum topic appends `/frustum`
to its image topic. Both can include their owner's model origin. The intended
27th finite CLI run proves loading and stepping only; it does not prove message
acquisition, demand, reset, installation or browser behavior. The earlier 26
finite examples remain unchanged.

Fresh native work is limited to simultaneous pair/eight original SensorFactory
instances with actual WORLD camera poses and copied full model maps, using the
same ElementPtr factory API as M30. The direct component `Update(now)` bypasses
base scheduling. Both actual typed image/frustum callbacks, input getters/bit
ledgers, provider maps and full wire bytes are required; returned true or a ready
publisher is insufficient. This proves no original Sim System/ECM gathering,
native demand, scheduling, reset/removal, rewind or Physics equivalence. Existing
M30 nineteen nominal and twelve side-face witnesses are retained, not recaptured.

The failed fixed System diagnostic remains failed. A separate fixed component
identity/alias/static-metadata check passed, with no provider build or DSO load.
Actual capture and both independent DATA audits passed. Ten configurations,
forty direct camera WORLD-pose/full-model-map inputs, forty paired observations
and eighty canonical messages are copied into an immutable fixture. The new
numeric consumer compares full poses within the retained 1e-12 native bound and
exact lexical membership. The Server consumer copies captured values through
the unchanged encoder and compares all eighty full messages and Headers exactly.
These are authored controls, not executed passes. The original precompile
controller failure and missing fixed System candidates remain preserved. See
[reference provenance](../planning/reference/m53-multiple-logical-cameras.json).
Full normal, strict workspace/per-crate and
actual-parent coverage and independent retained local DATA remain mandatory.
Actually accepted predecessor, exact hosted/official success and ordinary Main
readbacks precede publication or acceptance. No CI retry or workaround is released.
