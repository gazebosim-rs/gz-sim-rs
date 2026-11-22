<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M100: translated COM for passive posed primitive singleton templates

Translated primitive mass frames and local joint frames are implemented and
independently reviewed under selection `8daad5d5`, over qualified but unaccepted
parent `1a4bdbb5`/`e8dd98dd`. Read [M100](M100_DELIVERY.md) and
[M101](M101_DELIVERY.md) in docs (from the repository root, use `docs/`).
The bounded pair retains at most five working days including review and required
checks. There are 23 new Rust declarations: 23 headless, 22 renderer, 22 shared;
18 public and five private. The native private case is headless only; the four
Scene private cases are shared. No new Python, finite or CLI cases are added.

The first full normal failed on tree `2af214b2`: 2046 headless cases passed, two
failed and two were ignored. Both failed cases are in M100's new public test
module; 21 of the 23 new controls passed headless, including all 14 M101 controls
and the native private control. Renderer and later coverage were not reached.
No commit or qualification resulted; the failed tree, packet, logs and partial
RAW evidence remain unchanged (`844527c5`, Root receipt `f460b14d`).

Independent Source review found two test-premise errors, with no production
finding. The corrective test source (`619aae73`, test `7195ce9b`) initializes
Bevy schedules without callbacks before the complete paused-state baseline.
It also witnesses the controller's committed zero on the first advancing step,
then keeps the original target witness on the second step after PostStep latch.
All inputs, strict comparisons, tolerances and eight case identities remain.
No production code changed in this correction. Corrected-source verification
is pending; the complete atomicity assertion is retained.

The integrated source has 1827 files, 23 owned paths (15 Rust and eight docs),
four new paths, 1804 protected parent paths and 124 cumulative changed paths.
Six implementation paths are separate from four comment-only spans in the
seventh production file. Corrected-source final review, formatting and one full
normal retry remain required, followed by independent local DATA, exact hosted
CI and its original-artifact audit. No new tree, commit or pass is presumed.

Parent M98/M99 passed its own normal, local DATA, exact hosted attempt 1 and
independent official audit (`6d3c3d07`, Root `1d6afe7d`). The official audit
compared four retained RAW groups once; it replayed no tests, builds or downloads.
Earlier watcher/API failures and both pre-I/O collector method failures remain
historical failures. No parent acceptance or qualification transfers to this pair.

The pending retry resource choice (`81b75411`) places only a fresh ordinary
Cargo target on executable SHM: initial 31 GiB SSD, 8.875 GiB SHM and 1.75 GiB
runtime. Mandatory 14 GiB initial SSD, 8/4 GiB SSD/SHM early stops, runtime
1.375 GiB early stop, all operating floors, jobs4 and Mem8 GiB remain. The
4.875 GiB sampled SHM net-free-decrease cap is not an allocation quota, continuous
peak or fit guarantee. All old targets, locators and evidence remain; no eviction
is authorized. Fresh Root admission and reviewed execution are still required.

M77 formal browser evidence and owner-confirmed shutdown still gate acceptance.
The no-expiry manual demo is protected; no stop, restart or expiry is authorized.

## Scope and exclusive ownership

The author owns Scene `crates/gz-sim-scene/src/world_spawn.rs`, the new
`crates/gz-sim-scene/tests/runtime_primitive_offset_com.rs`, and exactly four
native constructor files: `crates/gz-physics-rapier/src/posed_box.rs`,
`posed_sphere.rs`, `posed_cylinder.rs`, and `posed_capsule.rs` in that same
native source directory. The private native regression matrix belongs inside
posed_box.rs's existing cfg(test) module, with no external native campaign.
M101 separately owns its config file and new Scene test; no code file is shared.

Only an actual valid passive loaded or owned-runtime SINGLETON may supply the
new template route. The caller requests one existing Box, Sphere, Cylinder or
Capsule, with a nonidentity resolved collision pose and nonzero resolved
inertial center. Exclude actual loaded/runtime compounds, including a compound
source presented with singleton caller DOM. The existing variable named
`runtime_singleton` describes a runtime-COMPOUND source plus a one-collider
request; it is not proof of singleton source ownership.

Retain full model_state/source ownership validation before the new route.
Require resolved identity LINK, existing finite f64/f32 pose representation and
quaternion band `abs(norm_squared - 1) <= 1e-12`, positive mass/density and valid
explicit inertia/native admission. Actual velocity-controller ownership still
vetoes the new branch even if the caller removes a plugin. Copy requested
resolved parameters without normalizing or asserting they equal source physics.
Preserve four-node M/L/C/V identity, visual=model+3, the existing visual whitelist
and Arc alias, foreign-world and allocator rules, and source-first error order.

Compound COM, Mesh behavior, actual compound-to-singleton policies, mature
identity/tiny-COM paths, sensors/controllers/joints and articulation composition
stay closed or unchanged. This scope adds no new visuals, APIs, Server, world_edit,
initial, joints.rs, common backend lib.rs, body-state setter or solver changes.
TPE refusals remain exact first-reachable controls. Fuel, paused unrelated work,
DART/MuJoCo, live-demo lifecycle and all acceptance gates are excluded.

Root separately corrected four public-API documentation spans in
`crates/gz-physics/src/lib.rs` (`f7071e4d`). This is a Root-owned COMMENT-ONLY
Source path, with no signature, statement, common implementation or API extension.
It is separate from the five M100 implementation files.

## Bounded native prerequisite and numerical policy

Replace only the four nonzero-center Unsupported vetoes. Existing finite-center,
shape/rotational/mass/inertia checks remain authoritative. Additional preflight
runs only for nonzero c, before any allocator, native body, collider or world
mutation:

- c and `sum(c_i*c_i)` must be finite.
- w = BODY world pose * c and `sum(w_i*w_i)` must be finite. The existing local
  variable named world includes the COLLIDER pose and must not be used here.
- Compute the existing shape's local bounding sphere transformed by
  `shape.position()`; e = length(sphere.center - c) + sphere.radius must be finite.
  This matches Rapier's maximum-extent calculation about the local COM.

These are finite-domain guards, with no clamp, normalization or invented
numerical tolerance. They exclude rotated/composed COM and collider-relative
extent overflow even when individual inputs are finite. Centered constructor
behavior stays unchanged. Shared inertia_properties already installs the local
center and rotated tensor; insertion zeros collider density before adding the
explicit mass properties. Original inertia and collider construction records
are retained for reset. This bounded constructor admission does not promise
universal later-step, force, setter or extreme-trajectory safety.

## Meaningful controls and retained dispositions

Exercise all four primitive kinds and static/dynamic native construction with
nonzero COM, independently rotated model and collider frames. The actual new
regressions use explicit isotropic inertia with identity inertial rotation and
the existing Box visual. Existing inertia-frame and visual admission policies
are preserved; independently rotated inertial frames and every visual variant
are not claimed as new regression coverage. Native BodyState.position is the link/body
origin; its linear velocity is measured at COM. Verify actual world COM
p + R*c separately and the Scene conversion
`v_origin = v_COM - omega cross (R*c)` and its inverse. Do not label body-origin
position as the native COM position.

Compare actual native local/world mass properties, explicit tensor and zero
collider density, finite extent, native/Scene six-axis states and every attached
collision/visual frame. Use independent globally isolated curved-surface rays
with wide caps and retained strict1e-11, with exact binary endpoints kept in
separate identity fixtures. All bodies and children count in a global query.

Use explicit inertia/density twins, principal-axis or isotropic gyro-safe
force/torque controls at actual COM, pending-input and invalid-command atomicity,
source retirement/retemplate/removal, survivors, ordinary versus loaded reset,
public/native high-water and real camera128 capacity. Do not require ideal
multi-axis finite-step tau*dt/I or command-to-state servo convergence.

The private all-four native matrix must inspect real local/world COM, mass
properties, collider density and maximum extent; preserve pending inputs and
body/ID/native-world snapshots on nonfinite center, norm-squared overflow,
composed-world overflow, relative extent overflow, invalid inertia and allocator
exhaustion. Include repair, state edits, reset and removal using actual native
APIs. Preserve every existing exhaustion case body exactly. The private matrix
runs through gz-physics-rapier/src/lib.rs, target gz_physics_rapier, kind lib, as
`posed_box::tests::offset_posed_primitive_mass_frames_bounds_resets_and_pending_ids_are_native`.
The maintained renderer package list omits this native libtest; it remains a
headless obligation, not a promised renderer execution.

Inside the existing public Scene error/atomicity case, `d2c0ec9c` adds an all-four
constructor valid-control then finite c1e200 norm-squared-overflow refusal through
existing native APIs. Exact InvalidValue/state/no-ID-gap/zero-step and queued
force/torque repair twins exercise the native boundary in both profiles. No new
test declaration, package/profile selection, production change or native campaign
is introduced.

Root alone owns the same-input retained dispositions: existing native
`tests/posed_box.rs`, `posed_sphere.rs`, `posed_cylinder.rs`, `posed_capsule.rs`
under gz-physics-rapier, and the selected old tiny-COM row in
`crates/gz-sim-scene/tests/posed_primitive_runtime_spawn.rs`. The eight old native
inputs are four shapes times static/dynamic mass None/Some2 with c[-.25,0,0].
Turn them into meaningful native positives without changing inputs, case IDs,
neighbors or old invalid/exhaustion assertions. The selected Scene tiny-COM
input must gain an actual velocity witness; other first-error rows stay closed.
Prove exact whole-file outside-delta inverses to the parent. Do not promote
actual compound or Mesh COM refusals as part of these dispositions.

## Dependencies, budget and exit criteria

Budget at most five working days including review and mandatory qualification.
Parent1a4/e8dd has its own normal, local DATA, hosted and independent official
qualification. Official report `6d3c3d07` and Root receipt `1d6afe7d` are actual;
no result or acceptance transfers to this pair. Both pre-I/O collector failures
and watcher/API403 records remain historical. M77 browser and owner-confirmed
shutdown remain gates; demo86363 stays live without restart or expiry.

Finish implementation, regression Source and docs; independently review the
whole bounded batch and resolve all findings before Root releases one affected
formatter and consolidated staged hook/normal/strict coverage. Retain finite47,
CLI21, all inherited Source and native/reference obligations. Independent local
DATA, exact-head hosted CI and original artifact audits remain required before
qualification/acceptance. Resources and heavy procedures require separate Root
review/admission. This guide authorizes no tests, compiler/lint/build/import,
probe, archive replay, network, target, cleanup, commit or publication.

## Actual Source declaration inventory (not executed results)

- `offset_four_primitives_keep_model_child_frames_four_ids_and_visual_aliases` — headless and renderer, Scene integration
- `offset_geometry_keeps_binary_endpoints_and_global_nearest_exclusion` — headless and renderer, Scene integration
- `offset_six_axis_state_and_velocity_inputs_round_trip_origin_and_com_twists` — headless and renderer, Scene integration
- `offset_explicit_mass_inertia_and_com_wrenches_ignore_collision_density` — headless and renderer, Scene integration
- `offset_templates_reuse_passive_runtime_sources_retire_and_keep_reset_high_water` — headless and renderer, Scene integration
- `offset_request_errors_and_source_corruption_preserve_allocators_and_pending_work` — headless and renderer, Scene integration
- `offset_new_route_keeps_actual_compounds_controllers_and_mature_identity_distinct` — headless and renderer, Scene integration
- `offset_static_contacts_dynamic_fall_and_camera_capacity_keep_owned_inputs` — headless and renderer, Scene integration
- `posed_box::tests::offset_posed_primitive_mass_frames_bounds_resets_and_pending_ids_are_native` — headless only, Rapier-lib

## Pending corrective run resource choice

Choice `81b75411` uses one fresh ordinary Cargo target on existing executable
SHM; profiles, SDK, TMPDIR, RAWs and retained packets remain on SSD. Initial
admission is 31 GiB SSD, 8.875 GiB SHM and 1.75 GiB runtime. Mandatory 14 GiB
initial SSD, early 8/4 GiB SSD/SHM and 1.375 GiB runtime, all four operating
floors, Mem8 GiB and jobs4 are unchanged. The ordinary SHM mount's sampled
net-free-decrease limit is 4.875 GiB; it is not a quota, continuous peak or
future-fit guarantee. Root must recheck actual host space and exact mount/owned
target identity before one reviewed invocation. All old targets and originals
remain protected; no allocation or eviction is authorized by this guide.
The original runtime-target proposal/selection and failed execution remain
frozen as historical evidence.
