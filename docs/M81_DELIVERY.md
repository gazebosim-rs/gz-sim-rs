<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M81: one posed capsule collision

M81 admits one translated or rotated capsule collision per Rapier f64 link.
The budget is at most five working days, including independent review and all
mandatory gates. Collision geometry moves relative to the link; the reported
link, visual, sensor and inertial frames keep their existing meaning.

## Basis and current state

Source preparation is selected over locally qualified M80
`ac75646aa5db9d1a1f983e47c0c86857415b9a2c`, tree `80c5de32`, true parent
`8ad64532`. Its complete normal/strict hook and independent retained DATA passed;
its review is published and exact hosted run 44831931346 attempt 1 is active at
selection. M80 is not accepted and no result qualifies changed M81 Source.
Accepted Main remains M76 `dddf1f42`: 76 phases and 202 slices are accepted,
catalog 81, Source finite/CLI inventories 47/21, Latest M70 original package 41.
M77 formal browser access and owner-approved shutdown still hold predecessor
acceptance. Its no-expiry manual demo session 86363 stays running.

Physics/Rapier and Scene Source is complete and integrated under disjoint leases.
Its 18 new Rust declarations run headless; nine also belong to the renderer
profile, with zero new Python declarations. Source contains 1781 files, 17 own
changes, five new files and five production leaves. These are Source obligations,
not passes. Whole independent integration review is pending; final formatting,
staged tree and qualification remain unbound. No M81 formatter, tests, lint or
native trial has run. Root alone releases the consolidated gates.

## Interface and admission

Add `PosedCapsuleBodySpec { radius, half_length, collision_position,
collision_orientation, position, velocity, mass }` and
`DynamicBackend::spawn_posed_capsule_link(spec, rotation, inertia)`. Use the
existing units and XYZW quaternion convention. `half_length` is half the straight
segment, so total axial half-extent is `half_length + radius`. The default trait
method returns explicit `Unsupported` before numeric admission or allocation.

Rapier uses the existing `capsule_z` geometry and caller collision pose. Unlike
cylinders, it needs no Y-to-Z correction. Compose local collision and world body
frames without moving the link origin or deriving its center of mass from the
collider. Validate finite positive representable dimensions, unit backend
quaternions, composed bounds and geometric mass properties before allocation.
Invalid inputs and identity exhaustion preserve IDs, state and pending commands.

Direct dynamic callers supply validated link-centered inertia; missing inertia
or a resolved nonzero center of mass remains unsupported. Scene passes existing
resolved SDFormat inertia, including defaults and centered automatic results;
there is no XML-presence parser or new automatic mass computation. Loaded XML
follows pinned SDFormat normalization; direct backend quaternion validation is
separate. Pinned SDFormat admits capsule automatic inertia only with identity
collision pose; nonidentity capsule auto remains refused upstream. M81 does not
expand that parser domain. Do not use normalized/defaulted XML as a negative
refusal oracle.

Scene routes one resolved nonidentity capsule collision through the new Rapier
operation. Identity capsule construction retains its old route. TPE posed
geometry, compounds, offset COM and other unsupported posed primitives stay
refused. One body/link, collision groups, original metadata and ordinary,
prepared and loaded reset/removal semantics remain intact. M79 may coexist on a
separate centered model; its posed-target refusal and exclusive velocity policy
are unchanged. No Server, transport, renderer, viewer, worker, package,
dependency, workflow or backend expansion is included; DART/MuJoCo remain retired.

## Ownership and retained assertions

Package owns Physics public API/default refusal and Rapier construction plus
private/public regressions. Scene owns admission, a test-owned SDF and integration
controls. Scope owns this contract and canonical planning; Root reviews docs and
integrates. Authors cross-review opposite code lanes; Scope independently reviews
the complete integration before any checks.

The only intended old-test transitions are the Capsule-refusal rows in
`crates/gz-sim-scene/tests/posed_box.rs` and `tests/posed_cylinder.rs`: replace
each with positive Rapier loading and independent geometry at its existing
tolerance. Preserve Sphere and all other negative rows and assertions. Review
these exact dispositions explicitly; do not weaken tolerances or blanket-update
unsupported lists. All 23 M80 case identities and assertion strength are retained except for the
two explicitly reviewed Capsule-refusal-row dispositions above. The cylinder
strict 3.6/1e-11 chord, all five ray-precision controls and the cylinder-only
query adapter remain byte unchanged.

## Acceptance

Require independent analytic axial, cylindrical-band, hemisphere, oblique,
tangent and miss rays under translated/noncommuting rotations; max-distance,
inside, exclusion, closest-hit and centered-identity controls. Check explicit
inertia and linear/angular impulses with link-COM countercontrols, stable contact
against a centered control, same-model exclusion and distinct-model contact.
Verify ordinary/prepared/loaded resets, removal, public identities and surviving
bodies, atomic malformed/overflow refusal and actual SDF default normalization.
Use existing supported visuals for runnable Scene fixtures; capsule visual or
camera expansion is excluded. Test-owned worlds do not increase 47 finite/21 CLI.

Retain `planning/reference/capsule.json`,
`crates/gz-sim-scene/tests/native_capsule.cc` and
`scripts/probe-capsule.py` with their original Source/command/library provenance.
Their historical TPE dimensions/contact observations are not fresh posed Rapier
solver parity. No native campaign is selected. A genuine gate-mandated evidence
dependency must be reviewed explicitly before any capture, never invented.

Finish implementation, regression Source and docs; independently review the whole
batch and correct all findings together. Then one final affected formatter and
complete ordinary staged hook/strict coverage, independent local DATA, exact own
hosted CI and official evidence are mandatory. Preserve retained tests and all
coverage bases. D31 permits qualification alongside predecessor closeout; Main
and acceptance still require accurate accepted ancestry and all predecessor/own
gates. No product pose teleport, repeated demo or acceptance bypass is selected.
