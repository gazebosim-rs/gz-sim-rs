# M99: Arbitrary finite unit JointController axes

## Current development: M98 runtime Mesh copies / M99 arbitrary joint axes

Read [M98](M98_DELIVERY.md) and [the plan](DELIVERY_PLAN.md) for the closed contracts, leases and acceptance criteria. Root selected
both D31 lanes in `c240ca62`, then dispatched their authors. The selection's
false dispatch field is its immutable creation-time observation. Each lane has
at most five working days including review and mandatory verification.

M98 admits a valid passive owned runtime compound as the source of one existing
supported Mesh collision template. Existing model_state and full immutable
compound_nodes/native-owner/member/ID/Arc/local-metadata validation run before
caller routing and asset I/O. Reuse the mature local-file decoder/native Mesh
constructor, four-node M/L/C/V identity, visual=model+3 and exactly centered
new-route inertia. Caller geometry/inertia are parameters, not original-SDF
equality. No new immutable singleton-child stamp is claimed. Preserve mature
loaded/singleton/posed/primitive/compound policies and legacy tiny-COM behavior.
No mesh visual/viewer/format/material/decomposition, public API, Physics, Server,
sensor, joint, controller or backend expansion is selected.

M99 changes only the existing two-Revolute JointController axis gate: finite
components, finite norm_squared and abs(norm_squared-1)<=1e-12, with no config
normalization or caller-DOM mutation. This explicitly selected unit-direction
rule is not an inherited parser contract. Cargo-pinned sdformat82c35f24 parse.rs
3640-3641 and Axis::try_set_xyz use the same normalizer: finite computed length
strictly greater than f64::from(1e-6_f32), then divide by length. XML scaled axes
normalize first; raw public xyz assignment bypasses that API. resolve_axis only
rotates; native joint normalization is a separate unchanged step. No exact norm
or normalization-fixed-point premise, looser glam::is_normalized band, lexical
magnitude restriction or silent config repair is permitted. Existing topology,
frames, expressed_in/axis2, gains, modes, limits, owners, DiffDrive and TPE stay.

Both implementations and regression Source are frozen. M98 `600d29fe` has eight
public cases; M99 `04aab3f2` has eleven public and four inline private cases.
The authored total is 23 unique H/R declarations (overlap23), public19/private4,
Python0 and no new finite/CLI cases. The private implementations are in config.rs
but execute in the lib.rs harness under
joint_controller::config::arbitrary_axis_dom_tests; use their full identities.
These are declarations, not executed passes. Server's whole M98 and Package's
whole M99 reviews are CLEAR. Scope independently closed Root's four retained
files `9a6ab8c5`: seven exact old XML axis rows become measured physical positives;
two invalid-Mesh rows retain every input, typed refusal and atomicity assertion,
with comments only. Old case identities, neighbors and whole-file inverses stay.

Observed code integration `f6e3b3d` has 1,821 physical Source files, eight owned
code paths, two new tests and two production paths; all 1,813 other parent bytes
and parent modes are exact. These docs add two guides and change six canonical
files. The intended full union is 1,823 files/16 owned/four new/two production/
120 cumulative versus accepted Main. Root must measure that entire physical
union after adopting the eight docs; those full-union numbers are provisional,
not observed final counts. Finite47/CLI21 remain unchanged. External final docs
review and that final census are pending at this author freeze. Formatter,
tests, lint, normal and audits for M98/M99 are unrun; future tree, commit and
results remain null, with no own failures. Batch findings before one consolidated
formatter and mandatory verification cycle; no per-edit trials or extra rounds.

The genuine locally qualified, unaccepted parent is a578c68a/treead639bd9,
parent ac4af1c7. Normal `2699554a` and independent retained DATA `694f4436` / report
`a483e19d` passed at the same head: H2002/R1538, two ignored each,494Python,
all18 own controls in both profiles, required248H/194R and15 strict Source bases.
Review publication `79df8bb6` preserved all106 old heads/Main and added only its
exact107th review ref. Automatic simulatorCI44970670576 attempt1 is running under
Root's sole watch54430; hosted/official outcomes are unknown. Preserve product
70d2f43e/6f23bf53, prehook4447773d and operational15996bc6 separately. Original
nine-file formatter8f18 supplies custody ONLYeight unchanged Rust copies; old
one-file0bd6 is historical and current one-file6c518 is distinct. M94/M95 official
method2 `1e419757` / report `e4797dd9` passed, RAW452/744/57/168 each once; first
before-RAW method1f39 remains. M94five, M92two+oldseven ONLYfive and M90three/one/
four histories remain separate. No qualification or acceptance is inherited.

Only Source work is released. Full-normal planning has a concrete capacity hold.
Proposal `1fbfd0df` and Root's conditional review `473453a7` plan initial
SSD32GiB/SHM12GiB, with mandatory initial SSD14GiB, early stops SSD8GiB/SHM4GiB,
maintained SSD/SHM1GiB, runtime1GiB+2MiB, MemAvailable8GiB and jobs4 unchanged.
Authoritative Root observation `87f800eb` has SSD31,443,066,880 bytes, short of
32GiB by2,916,671,488 bytes; SHM14,725,849,088 bytes meets12GiB by1,840,947,200
bytes. Observer-namespace values, including33GB SHM, are not Root admission.
Owner reply is pending on freeing4GiB or preparing an exact recovery proposal.
No new target, probe, cleanup, eviction, relocation, reuse or normal retry is
authorized. Resource samples and the complete concrete procedure must be reviewed
again before release; conditional planning is no universal peak/fit guarantee.
Source/docs review may continue while heavy execution is held. All old targets,
locators, caches, evidence and Fuel work stay intact; never inherit a parent's
run-specific reserve. Formatter, tests and current-wave outcomes remain unrun or
unknown at this freeze. Every output/deadline/ledger cap is preserved. All strict
workspace/per-crate/actual-parent/retained-ancestor line and region gates remain
strictly greater than95 percent, followed by independent local DATA, exact review/
hosted CI and independent official DATA. All predecessor and M77 browser/owner-
shutdown/release gates remain. Preserve accepted76/202, catalog99, Latest M70
original41, first97 phase objects/prior56 groups and old checkpoints losslessly.
Demo86363 stays live without expiry: no stop, restart or new expiry. DART/MuJoCo
remain retired; no unrelated framework/backend/native campaign is selected.

## Implementation and acceptance

Server owns only joint_controller/config.rs and new tests/
joint_controller_arbitrary_axes.rs. Four private cases remain in an inline
cfg(test) module in the same leased config file; no public seed/helper API.
Change only the six-axis gate to finite components, finite norm_squared and
abs(norm_squared-1)<=1e-12. Config does not normalize or mutate its DOM.

The selected numeric policy reuses retained ray direction admission explicitly;
it is new to controller admission. Actual pinned parser82c35f24/Axis::try_set_xyz
normalization accepts lexical scale above the existing length threshold. Raw
[0,3,4] or [2,0,0] assignment must refuse; explicit setter [0,.6,.8] is eligible,
as is the equivalent scaled XML. Setter failure preserves its prior direction.
Parsed zero/nonfinite inputs keep their earlier SDF failure; invalid raw DOM
keeps the config frame error. Preserve every caller field on both paths.
resolve_axis rotates only, with existing native normalization unchanged. No
lexical-unit check, exact norm/fixed-point premise, loose is_normalized rule,
new parser epsilon, config repair or solver/motor/sample algorithm is selected.
Keep two distinct links/two Revolute world-root serial chain, dynamic Rapier,
identity LINK rotations, current joint pose/relative_to, empty expressed_in,
no axis2 and all gains/modes/limits/bindings/owner/DiffDrive/TPE bounds.

Public controls include arbitrary rational/noncoaxial/signed/scaled directions,
setter/XML physical twins, singleton/2/4/16, both modes and nontrivial model frames.
Independent Rodrigues/pivot references use measured joint coordinates, never
motor targets or native body poses as expected geometry. Check native/logical
full bases and velocities, every child, opposite-axis/opposite-target physical
twins, explicit isotropic inertia/density, latest/clamp/zero/pause/invalid,
continuous samples, reset/removal/high-water/eight-owner and terminal faults.
Healthy pre-step repair is separate from Faulted latching. Contact uses axis
[0,.6,.8], tangent[0,.8,-.6], positive.006 gap and1500 steps against quiet/clear
controls. Generic advected geometry has wide caps and1e-11 tolerance; exact
inclusive endpoints belong only to an isolated identity binary Sphere.

Root alone dispositions precisely seven old XML rows: two in the principal-axis
case (0.5 0.5 0;1 1e-12 0), five in the signed-axis case (those two plus
-1 1e-12 0;0 -1 -1e-12;-1 -1 0). Keep their exact input/case identity and use
measured native/basis/query/command/quiet positives. Existing100 steps, no
immediate body change, stable IDs and all neighboring frame/axis2/type/topology/
mode/owner negatives/helpers stay. Full-file inverses restore the parent;
separate DiffDrive-Y and unrelated refusals are untouched.

## Authored public and private harness identities

Eleven public H/R integration cases in
crates/gz-sim-scene/tests/joint_controller_arbitrary_axes.rs:

- `arbitrary_opposite_axes_and_commands_keep_measured_physical_equivalence`
- `arbitrary_coordinates_and_samples_remain_continuous_through_reversals`
- `arbitrary_noncoaxial_chains_move_native_bases_velocities_and_all_children`
- `arbitrary_axis_binary_endpoints_remain_separate_from_advected_curved_queries`
- `arbitrary_commands_keep_clamp_latest_zero_pause_and_invalid_equivalence`
- `arbitrary_motor_effort_respects_explicit_isotropic_inertia_and_density_twins`
- `arbitrary_resets_preserve_geometry_targets_identities_and_survivors`
- `arbitrary_eight_owner_late_faults_keep_pending_atomicity_and_terminal_latch`
- `arbitrary_axis_motor_contact_moves_an_independent_probe_with_quiet_clear_controls`
- `arbitrary_axis_admission_keeps_frames_topology_limits_and_other_domains_closed`
- `arbitrary_scaled_parser_and_explicit_setter_directions_share_physical_trajectories`

Four private H/R cases are implemented in config.rs but execute with harness
source crates/gz-sim-scene/src/lib.rs, target gz_sim_scene, kind lib. Use their
full namespace, not an integration-test source or unqualified function name:

- `joint_controller::config::arbitrary_axis_dom_tests::arbitrary_direct_dom_unit_directions_preserve_the_entire_caller_model`
- `joint_controller::config::arbitrary_axis_dom_tests::arbitrary_direct_dom_nonunit_nonfinite_and_extreme_norms_are_not_repaired`
- `joint_controller::config::arbitrary_axis_dom_tests::arbitrary_direct_dom_near_unit_band_checks_both_sides_without_normalizing`
- `joint_controller::config::arbitrary_axis_dom_tests::arbitrary_setter_normalization_is_explicit_and_failures_preserve_its_prior_value`

Lane totals are15 unique H/R, overlap15, public11/private4/Python0. Completion
04aa/codejoinf6e3 record declarations, not execution. Mandatory gates still apply.
