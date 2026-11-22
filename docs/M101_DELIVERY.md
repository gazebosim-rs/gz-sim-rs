<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M101: local joint frames for the existing JointController chain

The selected implementation and its regression Source are independently
reviewed under `8daad5d5`, over parent `1a4bdbb5`/`e8dd98dd`. Its original
completion `6ebb8c40` and first-error correction `5ddbc52f` remain preserved.
The loaded offset-COM fixture retains its exact input and expects Scene's
`Unsupported: offset center of mass` before native articulation admission.

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

## Closed production contract and leases

The author owns only `crates/gz-sim-scene/src/joint_controller/config.rs` and the
new `crates/gz-sim-scene/tests/joint_controller_joint_frames.rs`. Root owns the
integrated candidate, shared docs/planning and exact retained-row dispositions.
No fixture addition is selected; any necessary fixture must receive a concrete
lease before authoring. All retained case identities and assertions outside
reviewed same-input dispositions stay exact, with whole-file inverses to parent.

Replace only the identity local JOINT-pose restriction with the existing Scene
representability rule: translation and quaternion components are finite as f64
and after f32 conversion, and `abs(norm_squared - 1) <= 1e-12`. The caller DOM
must remain byte-semantic unchanged; do not repair or normalize it. Relative-to
is still empty or the exact child. Model rotations and local JOINT rotations
are distinct from LINK rotations and axis direction.

Retain two distinct links and two Revolute joints in the dynamic Rapier
world-rooted serial chain, the existing canonical-link relation, identity LINK
rotations and finite ordinary LINK translations. Preserve the arbitrary-axis
finite unit band, empty `expressed_in`, no `axis2`, and every current mode, topic,
gain, effort/velocity limit, binding and eight-owner boundary. Offset articulation
COM, extra links/joints/types, PID/actuator modes and new TPE articulation stay
closed. No Physics, Server, public API, `joints.rs`, solver or native change is
selected. Existing Scene pose/axis resolution and native anchor construction
are dependencies, not rewritten algorithms.

## Meaningful regression obligations

- Exercise translated and rotated local JOINT frames, noncommuting model/joint
  rotations and noncoaxial axes in both existing position and velocity modes.
  Include singleton and already-admitted compound endpoints without changing
  their geometry or controller ownership policy.
- Derive independent Rodrigues rotations and world-anchor trajectories from
  initial frames and measured committed joint coordinates. Check native body
  positions/orientations, all logical bases, angular velocities and the rigid
  cross-product origin velocities for BOTH LINKS. Check every attached CHILD's
  frame and pose; no child Velocity component assertion is claimed.
  Do not use an ideal commanded angle or exact servo convergence as an oracle.
- Compare opposite-axis/opposite-command physical twins, equivalent/default,
  clamped/latest, paused/zero and invalid commands. Keep soft-motor observations
  separate from predicted convergence, and use stable contact/quiet controls
  with a positive initial gap when proving physical response.
- Account for every body and child in global ray queries. Keep transformed curved
  surfaces isolated with wide caps and the retained strict `1e-11` comparison;
  exact binary endpoint, inside and exclusion checks belong to separate exactly
  representable identity fixtures.
- Cover ordinary reset versus loaded reset, source/lifecycle removal, survivor
  behavior, pending inputs, public/native identity high-water and eight-owner
  boundaries. Preserve healthy-before-step repair versus terminal fault latch;
  a reset must not be advertised as repairing a terminally faulted execution.
- Test direct DOM nonfinite, nonunit, overflow and two-sided near-unit quaternion
  inputs with complete caller immutability. Preserve actual parser-versus-config
  first-error ordering. Remaining LINK rotation, topology, joint kind, axis2,
  `expressed_in`, invalid mode/limit and TPE refusals remain typed controls.

These are acceptance obligations, not a forecast of declaration counts. Root
must identify each old refusal's first reachable guard before selecting a
same-input positive disposition: a nonidentity LINK pose is still refused and
must not be mistaken for the newly admitted JOINT pose. Preserve XMLs, case IDs,
all neighboring negatives and complete outside-delta byte inverses.

## Dependencies, integration and exit criteria

M101 uses the existing reduced-revolute anchor machinery. If whole Source
tracing shows another production seam is necessary, stop and ask Root to rescope;
do not widen the lease. Corrected selection `8daad5d5` adds a bounded four-constructor native COM
prerequisite to M100 only. Its initial Scene-only scope defect is preserved as a
pre-execution finding, not a failed test. That correction adds no native lease to
M101 and permits no unreviewed coupling or combined completion claim.

Finish implementation, regression Source and docs; batch all independent review
findings and fixes before Root releases the one affected formatter and full
mandatory staged hook. Preserve every retained profile, native/reference input,
strict coverage base, finite and CLI gate. Independently audit recorded local
results; publish and qualify the exact same review commit through hosted checks
and original artifact audits. Reuse only unchanged evidence where existing
checks explicitly permit it. The original selection's unknown parent hosted
status is preserved as a creation-time observation; genuine subsequent parent
hosted success does not transfer qualification or acceptance to this pair.

Fresh resource admission, the normal procedure and source custody remain Root
owned and unreleased here. No Cargo target, test/lint/build/probe, product import,
formatter, audit/RAW replay, network request, commit or publication is authorized
by this guide. Preserve all caches and evidence outside separately authorized
operations, the Fuel owner's work and all paused unrelated dependencies. DART
and MuJoCo remain retired. No new framework, protocol, transport or sensor scope
is included.

## Authored declaration inventory (not execution receipts)

The ten public declarations use the new Scene integration-test harness. The four
private declarations are implemented in config.rs but run through Scene lib.rs,
target gz_sim_scene, kind lib; they are not integration targets.

- `joint_frame_opposite_axes_and_commands_keep_measured_physical_equivalence`
- `joint_frame_coordinates_and_samples_remain_continuous_through_reversals`
- `joint_frame_noncoaxial_chains_move_native_bases_velocities_and_all_children`
- `joint_frame_commands_keep_clamp_latest_zero_pause_and_invalid_equivalence`
- `joint_frame_motor_effort_respects_explicit_isotropic_inertia_and_density_twins`
- `joint_frame_resets_preserve_geometry_targets_identities_and_survivors`
- `joint_frame_eight_owner_late_faults_keep_pending_atomicity_and_terminal_latch`
- `joint_frame_motor_contact_moves_an_independent_probe_with_quiet_clear_controls`
- `joint_frame_axis_admission_keeps_frames_topology_limits_and_other_domains_closed`
- `joint_frame_child_default_and_quaternion_signs_share_measured_physical_trajectories`
- `joint_controller::config::local_joint_frame_dom_tests::local_joint_dom_finite_translations_and_unit_frames_preserve_every_caller_bit`
- `joint_controller::config::local_joint_frame_dom_tests::local_joint_dom_nonfinite_overflow_and_nonunit_frames_are_never_repaired`
- `joint_controller::config::local_joint_frame_dom_tests::local_joint_dom_quaternion_unit_band_checks_both_sides_without_normalization`
- `joint_controller::config::local_joint_frame_dom_tests::local_joint_dom_frame_admission_does_not_authorize_other_axis_link_or_frame_domains`

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
