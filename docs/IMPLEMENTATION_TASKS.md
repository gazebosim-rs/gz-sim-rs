## Current M100/M101 correction and qualification

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

---

The retained qualified-parent checkpoint follows as historical context; its
status statements describe their own recorded checkpoint.

## Current development: M98 runtime Mesh copies / M99 arbitrary joint axes

Read [M98](M98_DELIVERY.md) and [M99](M99_DELIVERY.md) for the closed contracts, leases and acceptance criteria. Root selected
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

## Historical parent and earlier Source observations

Everything below is preserved verbatim from a578c68a. Its then-current pending,
failed or unrun statements describe earlier checkpoints; current work is above.

## Current corrective checkpoint: M96/M97 typed error precedence

The completed headless run of the second product normal had one failed target
and case. Root session70400 (`6026bd` → `db6414`) returned Root/Git1 after
2039.9047407599865 seconds. Failure `6f23bf53` and actual receipt `fe3d2f32`
retain the failed `new_primitive_route_first_errors_and_late_native_refusals_preserve_pending_inputs`
case, whose assertion expected InvalidState. Seventeen of the eighteen new
controls passed in that failed headless profile; one failed. Renderer and strict
coverage qualification were not reached, and no new commit was created. These
partial passes belong only to frozen failed tree `d5d4b658`; they do not qualify
the corrected Source. Resource floors held, with no capacity, signal or
finalization fault. The failed log does not print a row or error variant: row9
is a Source inference, not a separately measured diagnostic.

Package and Server traced the complete seventeen early inputs and all three late
native-error inputs. The selected correction `6c2ecb26` changes only the two
incorrect expected error routes: rows9 and14 belong to InvalidScene; row8 stays
InvalidState. `prepare_model_template` resolves collision and inertial poses
through SDF resolve_pose, which validates finite translation and a unit
quaternion before later representability checks. Thus row9's NaN translation
and row14's zero quaternion fail resolution first. The exact groups are
InvalidScene [5,7,9,14,15], InvalidState [8], and UnsupportedModel
[0,1,2,3,4,6,10,11,12,13,16]. Row/error-only assertion diagnostics may make a
future failure visible. No production behavior, input, atomicity assertion,
geometry, tolerance, horizon, late-native case or case identity is changed.
The other seventeen new controls retain their complete Source. Corrected test `8956f183` and completion `b5208636` are complete; Scope
independently closed the complete file inverse to `a21575ac`, all eight case
identities and the resolver/error-order trace. Complete corrective code/docs
peer review remains pending at this author freeze. All earlier Source reviewers
missed the two error-precedence expectations; preserve their earlier CLEAR
checkpoints as observations, not successful verification.

Keep two product failures in order: `70d2f43e` was the earlier Clippy101
clone_on_copy failure before tests; `6f23bf53` is this later headless assertion
failure. The prior exact Copy-value repair remains in the test. Preserve
prehook method failure `4447773d` separately: Git, hook, checks, result and target
creation were absent. Preserve Root release-preparation failure `15996bc6`
separately as well. No reader/method failure is a product-normal pass or an
independent successful audit. Own retained-reader failure history stays empty.

The current correction root is distinct from both frozen earlier roots. Preserve
failed tree `d5d4b658`, review `7652fef1`, snapshot `1312c2aa`, preformat
`5817ef39`, one-file formatter `0bd6cbf7` and Root formatter receipt `8ff4ea94`
in their original locations. That prior one-file formatter is historical only;
the newly changed test needs one new affected-file formatter after complete
Source/docs and independent review. Original nine-file formatter `8f18f6f8`
continues to supply custody ONLY for the eight unchanged Rust copies, with
original review `3621997f` and snapshot `f7fe0526` preserved. Distinct roots and
receipts do not form a synthetic formatter union or inherit qualification.
Current formatter, tests, lint, normal retry and audits are unrun. Future final
tree, qualified commit and all corrected results remain null.

Source obligations remain 1,819 files /17 owned paths /four new files /two
production paths /116 cumulative paths versus accepted Main. Eight M96 and ten
M97 public controls give eighteen unique H/R declarations, overlap eighteen,
zero private or Python additions, and unchanged finite47/CLI21. Seventeen
headless passes from the failed tree are not corrected-profile results. The
selected features, disjoint production seams and seven retained same-input
dispositions remain the historical closed contracts below; this correction adds
no feature, declaration, API or backend scope.

The genuine locally qualified, unaccepted parent remains ac4af1c7/tree7d050c30,
parent cc83ca4a. Normal `ec381a89`, independent DATA `42221ccd` / report
`3d558421`, and review publication `ab18d1ca` passed. Exact hosted run
44950039686 attempt1 passed: receipt `602ca79a`, retired watcher74750, all
thirteen job steps successful. Original artifact collection and official DATA
remain pending. Parent reader3f67 and prelaunch60d29 remain separate from its
empty own normal-failure list. M92/M93 own official method2 `57c2fe5e` / report
`d92368a1` remains passed and unaccepted, with first methodb86 before-RAW
failure preserved. Parent M94five, M92two+oldseven ONLYfive, and M90three/one/
four formatter and failure histories remain distinct. No acceptance transfers.

The new conditional ordinary-RAM v3 proposal `df04f3fa` has complete Root
(`88631cc7`) and independent Package Source review. It selects only fresh
`/dev/shm/gz-m96-m97-ordinary-target-v3`, initial SSD30GiB/SHM14GiB, with
unchanged early SSD8GiB/SHM4GiB, mandatory initial SSD14GiB, SHM1GiB,
MemAvailable8GiB, runtime floors, jobs4, caps and full verification gates.
Its 499728384-byte historical margin above the prior complete-run SSD decrease
plus early stop is not a future peak or fit guarantee. Keep ordinary-only RAM;
profile/native-free/RGB targets, TMPDIR and evidence stay on SSD. Retain M96v1/v2,
M94v1 and every older target, locator, cache, scratch tree and proof. No eviction,
deletion, relocation or reuse is authorized. This is conditional Source approval,
not a concrete normal release or target-creation observation. Fresh floors and
mount/owned0700 identity/retained locator admission remain mandatory before
release; no new target or job is created here. All old31/22 contracts remain
immutable historical observations below.

Finish the complete code/docs and whole independent Source reviews, batch all
findings, then one affected-file formatter and one consolidated full mandatory
normal/staged hook. Keep every workspace/per-crate/actual-parent/retained-ancestor
line and region gate strictly greater than95 percent, independent local DATA,
ordinary review publication, exact hosted CI and independent official DATA.
All predecessor/M77 browser/owner-shutdown/release gates remain. No per-edit
trials or extra verification rounds. Accepted76/202, catalog97, Latest M70
original41 and live demo86363 remain fixed; no stop, restart or new expiry.
Preserve Fuel and retired DART/MuJoCo boundaries.

## Historical Copy-value correction author checkpoint

Everything below is preserved verbatim from the preceding document freeze.
Its then-current pending/unrun and single-failure statements describe that
observation time; the current failed normal and bounded repair are above.

## Current corrective checkpoint: M96/M97 Copy-value lint repair

The first actual product normal failed on one new test-source Clippy diagnostic,
not a test assertion. Root session38692 (`3795c3` → `bca7cb`) returned Root/Git1
after104.014645 seconds; Clippy returned101 under the unchanged workspace,
all-targets, keep-going and `-D warnings` invocation. Failure `70d2f43e` records
the sole `clippy::clone_on_copy` diagnostic at runtime_compound_to_primitive.rs:621.
Tests, renderer tests, coverage and a new commit were not reached. Resource
floors held; no capacity, signal or finalization fault caused this failure.
No current control has an executed pass or qualification from that run.

The earlier attempt `4447773d` is a separate prehook method failure: an inherited
helper read the wrong ambient release card. It returned Root1 with Git/result/
commit null before the hook, product checks or RAM target creation. Its corrected
method2 subsequently reached the real full hook and exposed the Copy-value lint.
Keep both original Sources, bindings, releases, logs and receipts. A failed
reader/method is not a product test result and never supplies a successful audit.

Root selected one isolated Source correction in `0d55971c`. Only
`v.state().clone()` becomes `*v.state()` in the TPE atomicity observation of the
new runtime-compound-to-primitive test. Motion::state returns &BodyState and
BodyState derives Copy; dereference preserves the same complete value. The full
file inverse restores frozen8c1e8d50 exactly. Corrected file `a21575ac` retains
all eight M96 case identities, all assertions, inputs, tolerances, geometry,
commands, horizons and first-error controls. All M97 tests and both production
files are unchanged. Original Root/Scope/Package/Server Source reviews missed
this lint; their earlier CLEAR observations are preserved, not counted as passes.
Root's code repair is complete and Scope's independent exact-delta/API review is
CLEAR. The complete corrected eight-doc/Source batch still awaits whole peer
review at this freeze; no current formatter or retry has run.

The new correction worktree is distinct from the frozen failed Root and frozen
docs-author lane. Preserve failed tree `dcc2c66f`, snapshot `f7fe0526`, review
`3621997f`, preformat `085fd14a` and original nine-file formatter `8f18f6f8`
in their original locations. That formatter is reused ONLY for the eight
byte/mode-unchanged Rust copies. The one affected new test requires its own
formatter after complete corrected Source/docs and independent review. These
custodies have separate roots and receipts; do not invent a combined formatter
or inherit qualification. Current final tree, commit, normal/audit/hosted results
remain null. Own product-normal history is [70d2f43e]; separate prehook-method
history is [4447773d]; own retained-reader failures remain empty.

Source inventory remains1,819 files /17 owned paths /four new files /two
production paths /116 cumulative paths versus accepted Main. M96eight plus
M97ten public controls give18 unique H/R declarations (overlap18), zero private
or Python controls and no new finite/CLI cases. Counts are authored obligations,
not passes. The selected features, disjoint production seams and seven retained
same-input dispositions remain exactly as the historical contracts below.

The genuine locally qualified, unaccepted parent remains ac4af1c7/tree7d050c30,
parent cc83ca4a. Normal `ec381a89`, independent method2 DATA `42221ccd` / report
`3d558421`, and review publication `ab18d1ca` passed. Its exact hosted run
44950039686 attempt1 now genuinely passed: receipt `602ca79a`, retired watcher
74750 (`ba3876`), all thirteen job steps successful and artifact14726704039
retained by the service. Original artifact collection and official DATA remain
pending; no new download or official success is inferred. Parent reader3f67 and
prelaunch60d29 remain separate from its empty own normal-failure list. M92/M93
own official method2 `57c2fe5e` / report `d92368a1` remains passed and unaccepted,
with first methodb86 before-RAW failure preserved. All inherited M92two+oldseven
ONLYfive and M90three/one/four formatter/failure histories remain distinct.

The independently Source-reviewed conditional v2 proposal `7d6e2d15` selects
only fresh `/dev/shm/gz-m96-m97-ordinary-target-v2` for ordinary Cargo output.
Keep failed v1 and
every older target, cache, scratch tree and evidence; no deletion, eviction,
relocation or reuse of v1. Only fresh ordinary Cargo output uses RAM; profiles,
native-free/RGB targets, TMPDIR and evidence stay on SSD. Keep selected initial
SSD31GiB/SHM22GiB, early stops SSD8GiB/SHM4GiB, mandatory initial SSD14GiB,
SHM1GiB, MemAvailable8GiB and existing runtime floors. A concrete v2 binding,
mount/ownership/locator contract and actual fresh floors require independent
Source admission before release. No v2 target or new executor is created here;
prior samples give no fit guarantee. Mandatory resource and output caps remain.

Finish whole corrected Source and eight-doc peer review, batch any final findings,
then one affected-file formatter and one consolidated full mandatory normal/
staged hook. Keep all strict workspace, per-crate, actual-parent and retained-
ancestor line/region coverage gates strictly greater than95 percent. Independent
local DATA, review publication, exact hosted CI and independent official DATA
remain mandatory, with every predecessor and original M77 browser/owner-shutdown/
release gate. No per-edit trials, extra verification rounds or old-packet success
audits. Accepted phases/slices remain76/202; catalog97, finite47/CLI21 and Latest
M70 original41 stay fixed. Preserve first95 phase objects/prior55 groups and all
old observations losslessly. Demo86363 stays live without expiry; no stop,
restart or new expiry. Preserve Fuel and retired DART/MuJoCo boundaries.

## Historical M96/M97 completed Source author observation

Everything below is preserved verbatim from the original pre-execution document
checkpoint. Its unrun/empty-history/CI-active statements describe that earlier
freeze; the current failure, repair, custody and parent disposition are above.

## Current development: M96 runtime compound to primitive / M97 signed principal axes

Read [M96](M96_DELIVERY.md) and [M97](M97_DELIVERY.md) for the
closed contracts, exact leases and meaningful acceptance. Root selected both
lanes under D31 in `ca34205a`, then dispatched implementation in `4e9d654b`.
The selection's dispatch-false field is its immutable creation-time observation.
Implementation and regression Source for both lanes are complete. All reciprocal,
Scope and Root whole-code reviews and the five retained-file reviews are CLEAR.
Joined inventory `91de9291` records 1,819 Source files, 17 owned paths, four new
files, two production paths and 116 cumulative paths versus accepted Main.
Eighteen unique new public Rust controls are declared in both profiles (overlap
18), with zero private or Python controls and no new finite/CLI cases. These are
Source declarations, not executed passes. External final eight-doc review is
pending at this freeze. Formatter, tests, lint, normal and audits are unrun;
final tree, qualified commit and all own results remain null, with no own normal
failure. Initial eight-doc checkpoint `3b1e254f` and all beforecopies remain
preserved as author-time observations.
Each lane has at most five working days including review and mandatory gates.

M96 admits a valid passive owned runtime compound as the source of a requested
single Box, Sphere, Cylinder or Capsule template, with identity or already
supported posed collision geometry. Always validate the actual model_state and
full immutable compound_nodes stamp/member/native/ID/Arc/local-pose domain before
route selection. The newly admitted branch retains identity-local LINK,
resolved exactly centered inertia and actual passive ownership, independent of
caller-stripped plugins. Caller-resolved parameters are copied, not asserted to
equal original geometry. Runtime-compound sources with Mesh, Plane, zero or more
than sixteen requested collisions stay refused; reject Mesh before decoding.
All mature loaded/singleton/posed/mesh/compound policies outside the new branch
remain exact, including tiny nonzero-COM identity singleton conventions.
Publication reuses four nodes M/L/C/V, visual=model+3, one native body and fixed
admitted_entities[4]. No new immutable singleton child-stamp guarantee is claimed.

M97 admits exactly six parsed-DOM signed Cartesian directions independently in
the existing two-link/two-Revolute JointController chain. SDF normalizes lexical
axis magnitudes; scaled signed directions are equivalence inputs, not lexical
unit-length refusals. Keep existing topology, identity LINK rotations, joint
poses, empty expressed_in, no axis2, dynamic Rapier, modes, finite limits and
eight-owner/sixteen-input bounds. Nonprincipal/near-principal directions remain
refused and zero/nonfinite inputs keep their earlier typed parser errors.
Use measured signed coordinates and independent Rodrigues/pivot/native-basis
references. Opposite-axis/opposite-target twins compare physical bases within
retained tolerances and signed coordinate/velocity reversal, not bitwise native
states or ideal motor speed. Explicitly exercise the antiparallel -X frame.

Package owns world_spawn.rs and new runtime_compound_to_primitive.rs. Server owns
joint_controller/config.rs and new joint_controller_signed_axes.rs. Root alone
owns the five retained test files and exact three runtime-singleton plus three
public signed-axis and one private -Z input dispositions. Preserve every case
identity and neighboring assertion. Independently reverse each complete retained
file to the exact qualified parent outside approved spans; production files
must likewise invert outside the reviewed change. Private DiffDrive -Y,
expressed_in, nonprincipal and other unselected refusals stay negative. All
whole-code reviews are complete; external final-doc review remains pending at
this freeze. Root's sole new signed-case oracle finding was closed in `eb72412f`
using a case-local measured-frame/wide-cap Sphere witness, leaving every mature
helper and neighboring body exact. No verification run found this Source issue.

The genuine locally qualified, unaccepted parent is
`ac4af1c738a9daaf3d9d899eca46c690d3aac22a`, tree
`7d050c307e07062487ef11a99c115e695cc26e05`, parent `cc83ca4a`.
Normal `ec381a89` and independent local DATA `42221ccd` / method2 report
`3d558421` passed; review publication `ab18d1ca` passed. Exact automatic CI
44950039686 attempt 1 is active under Root's sole watcher 74750; hosted and
own official outcomes remain unknown. Its failed retained reader `3f67e112` and
prelaunch operational failure `60d29f6d` remain distinct from own normal
failures, which are empty. Parent fresh five-file formatter custody stays
separate from M92/M93's two plus original seven reused only for five unchanged
Rust copies, its original `005bf028` failure, and M90's `f9d43298`/`fc5dce53`
with three/one/four custody. No result or formatter qualification is inherited.

M92/M93 completed own local, hosted and official method2 qualification at cc83:
actual `57c2fe5e` / report `d92368a1`, with RAW 450/742/57/168 read once and
actual zero CI base under new_branch_all_source. First official method failure
`b86f84d1` occurred before archive/RAW access and stays preserved. M92/M93 remains
unaccepted; none of its results transfers acceptance to this wave.

Bounded implementation/regression Source and all whole-code reviews are complete.
Finish external final-doc review and batch final findings before one affected
formatter and one consolidated full normal/staged-hook cycle. Retain
strict workspace, per-crate, actual-parent and every retained-ancestor line and
region gate, all production gates strictly greater than 95 percent, independent
local DATA, exact hosted CI and independent official DATA. All predecessor and
original M77 browser/owner-shutdown/release gates remain mandatory. No per-edit
trials, tests, lint, old qualification replay or fabricated pass counts.

The independently reviewed resource proposal `5a4ae1b3` is conditional Source
work only. Only the next fresh ordinary Cargo target may use executable host
RAM; profile/native-free/RGB targets, TMPDIR and evidence remain on SSD. Keep
selected initial SSD31 GiB and SHM22 GiB, early-stop SSD8 GiB and SHM4 GiB,
mandatory initial SSD14 GiB, SHM1 GiB, MemAvailable8 GiB and existing runtime
floors. The prior samples establish neither peak attribution nor future fit.
Before one full run, independently review the concrete binding and resample
actual floors/mount metadata; require a fresh absent nonsymlink owned0700 target,
retain UID/device/inode/mode and its durable locator on every outcome. No target
or heavy job is created/released here. Preserve the previous ordinary RAM target,
every old cache/target and evidence; no eviction, cleanup or cache relocation.
Preserve first 95 phase objects and prior 55 parallel groups exactly. Accepted
phases/slices remain 76/202; catalog becomes 97; finite examples 47, CLI cases 21
and Latest M70's original 41-example package stay unchanged. Demo 86363 remains
live without expiry: no stop, restart or new expiry. Keep Fuel ownership and
Rapier f64/TPE; DART/MuJoCo stay retired. No new backend, framework, Physics,
Server, visual/viewer, sensor, protocol, packaging or native-reference campaign.

## Historical M94/M95 document observation

All following retained content records its earlier author-time checkpoints.
The current M96/M97 contract and later parent disposition are stated above.

## Current development: M94 runtime compound retemplating / M95 principal-axis controllers

Read [M94](M94_DELIVERY.md) and [M95](M95_DELIVERY.md) for the selected closed contracts,
exact leases and acceptance oracles. Root dispatched the two Source lanes under
D31 after genuine parent local qualification. Each has at most five working days,
including independent review and mandatory verification. Implementation,
regression Source and reciprocal/joined whole-code reviews are complete and CLEAR.
Root's joined inventory `7dc0387b` records 1,815 Source files, 13 owned paths,
four new files, two production paths and 111 cumulative paths versus accepted
Main. These are Source counts. Final tree, qualified commit and all own execution
results remain unknown; no formatter, test, lint, audit or normal has run for
this wave. External final eight-doc review is pending at this freeze.

M94 lets an existing valid passive owned runtime compound seed another requested
2–16-member compound template. Keep full actual-source model_state/immutable
membership validation before route choice, actual VelocityControl-owner veto,
caller-resolved copied parameters, N+3 model/link/C0/visual/C1.. identities and
visual ID model+3. The actual-runtime-compound to singleton or mesh DOM route
stays refused. Existing singleton/posed/mesh/loaded-compound policies, fixed-four
singleton helpers, APIs and reset paths remain unchanged. No new provenance or
claim that caller parameters equal original geometry is selected.

M95 admits exactly parsed-DOM positive Cartesian +X/+Y/+Z directions independently
in the existing two-link/two-Revolute JointController chain. Pinned SDF parsing
normalizes lexical axis magnitudes: scaled positive X/Y are equivalent unit
directions, not new refusals. Keep identity LINK rotations,
existing joint frames, empty expressed_in, one world root, canonical topology,
centered inertia, existing position/velocity modes and eight-owner/sixteen-input
bounds. Negative/nonprincipal axes, axis2 and every other closed domain stay
refused. Reuse resolved native axes and scalar motor/state/reset paths; no solver,
gain, runtime-controller or public API change. Derive independent noncommuting
pivot/rotation witnesses from measured native coordinates, not ideal motor speed.

Package owns only world_spawn.rs and a new runtime-compound retemplate test file.
Server owns only joint_controller/config.rs and a new principal-axis test file.
Root alone owns one retained +X XML refusal row in compound_joint_controllers.rs;
its completed same-XML positive has real binding/native/frame/query/control
witnesses. Package, Server, Scope and Root whole-code reviews are CLEAR. Keep the
old runtime-compound singleton-DOM negative, private -Z/expressed_in negatives,
every neighboring assertion and case identity. Final doc review, one affected
formatter and one consolidated mandatory verification cycle remain next.

There are 17 unique new Rust controls declared in both profiles (overlap 17),
with zero Python controls: M94 has eight public cases plus the inline private
`world_spawn::runtime_compound_tests::runtime_compound_retemplate_requires_both_original_stamps_and_keeps_pending_input`;
M95 has eight public cases. Sixteen public plus one private controls are authored
obligations, not measured passes. No new finite examples or CLI cases are added.

The actual locally qualified, unaccepted parent is
`cc83ca4a62773edb80b05992fa67a03ff1dad134`, tree `1dc85433`, parent `e264c649`.
Normal `4a1c59a7` and independent local DATA `e33975e1` / report `59113b30`
returned zero: H1967/R1503 with two ignored each, 494 Python, all seventeen own
controls in both profiles and thirteen strict bases. Review publication
`6b16995f` passed on `review/m92-m93-posed-runtime-compound-wheels`.
Exact automatic CI 44939453527 attempt 1 is in progress under Root's sole watcher
12097 (`f59b22`); hosted outcome and official DATA remain unknown.

Preserve parent failure `005bf028` and its original seven-file formatter used
only for five unchanged Rust copies, with the separate current two-file formatter
`1ffb0cfd`. Keep M90/M91's inherited failures `f9d43298` / `fc5dce53` and distinct
three/one/four formatter custody. M90/M91 own qualification completed unaccepted;
none of these results or formatter records qualifies M94/M95 or transfers acceptance.

The independently reviewed resource proposal `cd0787da` is conditional Source
work. Only the next fresh ordinary Cargo target may use executable `/dev/shm`;
profile, runtime/native-free/RGB targets, TMPDIR and evidence packets stay on SSD.
The proposed run-specific initial SSD reserve is 31 GiB, with mandatory initial
14 GiB and early-stop 8 GiB unchanged. Add initial SHM 22 GiB and early-stop SHM
4 GiB, preserving mandatory SHM 1 GiB, MemAvailable 8 GiB and existing runtime
floors. The prior sampled SSD decrease and final ordinary allocation support
conditional arithmetic only, not peak attribution or a fit guarantee. Release
must resample all floors and mount flags, require a fresh absent nonsymlink owned
0700 target, retain its identity/locator on every outcome, and preserve every
normal/strict/native/profile/ledger guard. No directory, executor or release is
created here; no eviction, cache relocation or existing-evidence change is allowed.

Bounded code and meaningful regressions have completed whole Source review;
finish external final-doc review and batch every finding before one affected formatter and one consolidated
full normal/staged-hook/strict workspace, per-crate, actual-parent and retained-
ancestor line/region cycle. Keep every production gate strictly greater than
95 percent. Independent local DATA, ordinary review publication, exact hosted
CI and official DATA remain mandatory, with all predecessor and original M77
browser/owner-shutdown/release criteria. No per-edit tests, lint or trial runs.

Preserve all first 93 phase objects and prior 54 groups as exact historical
observations. Accepted phases/slices stay 76/202; catalog becomes 95, finite
examples 47, CLI cases 21 and Latest M70's original 41-example package unchanged.
Keep no-expiry demo 86363 live: no stop, restart or new expiry. Rapier f64 and TPE
remain; DART/MuJoCo stay retired. No unrelated backend, sensor, renderer/browser,
Physics, Server, protocol, framework, packaging or native-reference campaign.


## Historical M80 first-Source task

`LOAD-M80` extends the reviewed M78 single posed-collider path to a cylinder on
Rapier. Read [M80_DELIVERY.md](M80_DELIVERY.md) for the exact API, ownership,
regression obligations and exclusions. Physics/Rapier and Scene Source proceed
in parallel over locally qualified `8ad64532`; Root integrates shared contracts
and docs, and an independent reviewer checks the complete batch before tests.
The first final staged hook, strict coverage, exact hosted CI and independent
local/official DATA remain mandatory. No existing evidence qualifies new code.
M77 acceptance and Main remain held; its persistent demo stays running.

## Historical Source checkpoints

## Current development: M75 per-camera visibility masks

[M75](M75_DELIVERY.md) selects camera masks and visual flags for existing 64 by 64 RGB/depth
snapshots. Low 28 user bits and the original all-visible default are admitted;
reserved high bits remain unsupported. Exact nonzero bitwise intersection selects
visuals only after full world validation and the global 128-visual bound. Generic
snapshots remain box-only with strict default visibility. Empty selected images
still render under real demand; worker, packet and lifecycle paths stay unchanged.

The base is locally qualified M74 `487f0eab`, now on GitHub's owner-requested
`review/m74-signed-camera-skew` branch together with M70–M73. M74's first full
normal/strict gate, corrected retained audit and compact consistency passed;
the original reader failure remains preserved. Its automatic hosted run
44627469212 was in progress at M75 selection, with no success inferred. All five
phases remain unaccepted; final parent, exact hosted/official and Main gates are
mandatory. M1–M69/202 slices and M65 Latest35 remain unchanged.

Complete implementation, meaningful regressions, guides and independent whole
source review before one consolidated normal/strict cycle and retained audit.
No per-edit checks, old qualification replay, CI retry or snapshot probe is
selected. Preserve Rapier/TPE, D33 retirement, Fuel work and frozen evidence.

Earlier checkpoints below retain their historical scope.

## Current development: M74 signed pinhole camera skew

[M74](M74_DELIVERY.md) adds finite, representable signed intrinsic skew to the existing
64 by 64 RGB/depth paths. Projection, inverse rays and sheared frustum corners
must agree. Explicit P retains zero-skew rendering precedence. Native CameraInfo
keeps zero offdiagonals even when rendering uses skew; that limitation remains
explicit. Other camera bounds and packet formats stay unchanged.

Development is isolated over locally qualified M73 `03f3fc9d`. Its first full
normal/strict gate, independent retained audit and consistency review passed:
1,752 headless and 1,323 overlapping renderer tests, 475 aggregate Python checks,
44 finite source examples and 18 CLI cases. M70–M73 remain private and unaccepted
while final parent reconciliation and exact hosted/official/Main gates are held.
No CI retry, snapshot probe, old native capture or browser replay is selected.

Finish implementation, meaningful regressions, guides and whole independent
source review before one consolidated full qualification cycle. Keep Rapier/TPE,
D33 retirement, Fuel work, frozen parents and all older evidence intact.

Earlier checkpoints below retain their historical scope.

## Current development: M73 explicit monocular camera projection

[M73](M73_DELIVERY.md) admits explicit zero-translation SDF projection for the existing
64 by 64 RGB/depth camera domain. Select K from explicit intrinsics or the exact
HFOV fallback; explicit P selects rendering while CameraInfo publishes separate
K and P. Absent P preserves the M72 path. Stereo translation, skew, distortion,
noise, larger targets and package distribution remain excluded.

Development is isolated over locally qualified M72 `2f3e458f`. Its complete
normal/strict gate and independent retained audit passed: 1748 headless and
1309 overlapping renderer tests,473 aggregate Python checks,43 Source finite
examples and17 CLI cases. Both earlier failures remain preserved. M70–M72 remain
private and unaccepted while final accepted-parent reconciliation, exact hosted,
official and Main gates are held. No CI retry or snapshot probe is released.

Complete implementation, meaningful regressions, guides and whole independent
Source review before one consolidated full local qualification cycle. Do not
repeat completed parent proofs. Preserve Rapier/TPE, D33 retirement and Fuel work.

Earlier checkpoints below retain their historical scope.

## Current development: M72 explicit pinhole camera intrinsics

[M72](M72_DELIVERY.md) admits explicit finite zero-skew SDF focal lengths and principal
points for the existing 64 by 64 RGB/depth camera domain. It reuses the current
projection, worker and canonical message paths; absent intrinsics preserve HFOV
fallback. Explicit projection, skew, distortion and noise remain unsupported.

Development is isolated over locally qualified M71 `f31d7155`. M71's complete
normal/strict qualification and independent retained audit passed; its three
prior failures remain preserved. M70's package/install/camera/browser proofs stay
separate. Both phases remain private and unaccepted while exact hosted recovery
and final parent/official/Main qualification are held. M72 local qualification may
proceed after complete independent Source review, but inherits all final gates.
Do not replay standalone completed M70 demo or M71 qualification jobs. The new
M72 full gate retains all inherited regressions. Finish implementation, new
regressions and guides, then independently review before one consolidated cycle. Preserve Rapier/TPE, D33 retirement and Fuel work.

Earlier checkpoints below retain their historical scope.

## Current development: M71 independent RGB and depth pair

The owner requested the next development phase on 2026-11-19. M71 adds one
explicit RGB/depth pair with independent demand, rates and lifetime using the
existing camera and Zenoh paths. Read [M71_DELIVERY.md](M71_DELIVERY.md) for the
interfaces, ownership, fixture and acceptance criteria. Existing scalar and
homogeneous-pair defaults remain strict. RGBD registration, point clouds, greater
camera counts, renderer/physics changes and package distribution are excluded.

Development is isolated over locally qualified M70 `7094a31`. M70's local package,
Ubuntu installation, camera-component, RGB and actual browser proofs and their
independent audits passed. M70 remains private and unaccepted because exact hosted
qualification and official release evidence are pending after the retained
snapshot HTTP 401 failure. No CI retry or index probe is released. M71 Source
and local qualification may proceed after complete independent Source review.
Final parent acceptance and exact hosted/official/Main gates remain mandatory
before M71 acceptance or publication. Preserve the frozen parent,
all historical failures, Rapier/TPE and the Fuel owner's work.

Earlier checkpoints below retain their historical scope.

## Current development: M70 on the retained-backend Main

D33 is complete on Main `8f13dc7446be4fded424b1eba83df4c8253d0cef`.
DART and MuJoCo adapters, dependencies and dedicated jobs are retired; Rapier f64
and TPE remain. Cancelled backend work is not counted as completed.

M70 now carries the installed camera preview into an isolated successor on that
Main. Its four Python implementation/regression leaves are unchanged from frozen
`fcd5a30dc0a8431b250cf96720db27ae597a6585`; the new parent keeps the full D33
retirement. Four manifest-bound ELFs, 41 finite examples, optional camera output,
the native-free default robot and the six accepted viewer files remain the scope.
No Rust, dependency, renderer or protocol change is selected.

Complete independent Source review before one consolidated normal/strict coverage
cycle. Same-head hosted CI and independent audits precede source publication;
fresh source-bound release/package/install and actual robot/camera evidence
precede M70 acceptance. Old fcd5 results retain their historical scope and do not
qualify this successor. Browser access to `http://127.0.0.1:8082` still needs the
explicit owner approval required by automatic review; no browser retry is released.
M1–M69 and 202 slices remain accepted. M65 remains Latest with 35 examples.

Earlier checkpoints below retain their historical scope.

<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
## Current development: M69 two calibrated depth cameras

M1–M68 and 201 reviewed slices are accepted at Main
`24ea62519d0a671f789e6b3fc98bd2b72de18ae2`. M68 passed its full normal,
strict coverage, independent local DATA, [exact hosted CI](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/44077798140)
attempt 1, durable-original collection, official DATA and Main readback.
The [acceptance record](M68_ACCEPTANCE.md) keeps the repaired signed-input
lineage and historical failures distinct. Its nine-body metadata archive is
published; the fixed stager precedes unchanged signature, hash, size and package
record verification. M65 remains public Latest with 35 installed examples and
the unchanged viewer bytes and budgets.

[M69](M69_DELIVERY.md) adds exactly two independently demanded calibrated
depth cameras, with separate image/info demand and 10/20 Hz Source controls.
Existing scalar and RGB-pair APIs stay strict. The implementation, meaningful
regression Source and provisional lane reviews are complete in this successor
over the actual accepted M68 parent. The first normal on `733835b6` failed
at a stale registry count (40 versus 41); its strict correction remains.
The second normal on `b04a0c3b` failed after 2054.903 seconds at one new depth
Busy control: 498 Python and 2052 headless checks passed; renderer checks had
1304 passes and one failure. This is partial evidence, not qualification.
The test expected Busy after pumping a deliberately invalid worker, although
an asynchronous worker error can arrive first. The actual returned variant
was not printed and remains unknown. This isolated successor checks typed
Busy at admission before pumping, then requires the errored owner to leave
its peer's exact clock and publication intact, matching the retained RGB
control. Production code, fixtures, tolerances and accepted tests stay exact.
Preserve both failed trees and originals. No corrected check has run here.

The Source census carries 37 inherited Python declarations and two new M69
Python declarations. M69 authors 12 Rust controls (three headless, ten renderer,
one overlap), plus two separate renderer support instances. Its finite registry
has 41 Source entries and its auxiliary camera registry has 14; none of these
Source counts is an executed pass count or an installed-example count.

The complete correction has independent Source review. Its one selected Rust
leaf formatter passed with every source byte and mode unchanged. Close final
snapshot, census and procedure bindings, then run one consolidated full normal
and strict coverage retry. Reuse all formatting evidence; no per-edit checks.
Independent local DATA, exact hosted success, durable originals, official DATA
and Main promotion remain mandatory before M69 acceptance. Do not stage, commit
or publish an unreviewed candidate; no per-edit verification rounds. No mixed
RGB/depth pair, RGBD, noise, point cloud, physics, dependency, SDK, framework or
worker-distribution expansion is selected.

Preserve all frozen trees, historical evidence, current signed-input guards,
Fuel work and other owners' pending work. Root serializes heavy work. M70 remains
an unselected read-only installed-worker proposal.

Earlier checkpoints below retain their historical scope.

## Current M60: current installed two-robot preview

M60 is integrated on provisionally locally qualified, unaccepted M59
`7f849bd3e6b1d9842bdde415d9f2923e09ef0b0e`. M59 passed the full normal
(2,013 headless and 1,176 overlapping renderer tests, 449 Python checks and strict
coverage) and independent retained DATA, including four byte-exact RAW comparisons
and 33 finite examples. All six earlier failed normal runs remain preserved.
M60’s eight package, launcher, sender, regression and guide leaves have independent
Source review and are adopted without replacing a changed parent leaf. No M60
check, build, package, installation or browser result is claimed.

The package retains the default M19 robot and accepted viewer bytes, and copies
all 33 maintained finite examples with their existing relative assets. The
installed launcher gains a verified example selector; the finite drive helper
gains an optional concrete topic while preserving its existing defaults and
braking behavior. Physics, configuration math, dependencies and viewer rendering
are unchanged.

The exact M59 fixture and every parent repair are retained. Complete whole
integration Source review before one consolidated formatter and the unchanged
mandatory normal, strict coverage and independent retained audit. Do not run
checks between edits.

Then complete source-bound builds, notices and actual ELF dependency collection,
assembly and relocation audits, one supported clean Ubuntu installation, and a
fresh live browser demonstration. The installed M58 and M59 finite examples must
run; both M59 robots must receive independent installed-helper commands, produce
actual motion and canonical encoder odometry, and demonstrate Play/Pause/Reset
and owned shutdown. Proof-only host SDK tools are not installed dependencies.

Accepted predecessors, exact hosted CI, official audits and ordinary Main and
release readbacks remain required before acceptance or publication. M1–M42 and
175 slices remain accepted; M40 remains Latest. The required hosted snapshot
recovery is unconfirmed, and no CI retry or workaround is released.

The owner requested November 15 closeout. This is a calendar target, separate
from the five-working-day phase budget and retained quality gates. Root owns
integration and serial operations; independent Source preparation and review
continue in parallel. See [M60_DELIVERY.md](M60_DELIVERY.md).

Earlier checkpoints below retain their historical scope.


## Current M59: two independent differential-drive robots

M58 is provisionally locally qualified at `173b31d2`: its full normal passed
1,981 headless and 1,161 overlapping renderer tests, 448 aggregate Python checks,
strict coverage and 32 finite examples. Independent retained DATA passed with all
four byte-exact RAW comparisons. Seven normal failures and both failed readers
remain retained. This is local qualification, not hosted or release acceptance.

The sixth M59 full normal retired with actual exit 1 after 1,798.811 seconds.
It completed 449 aggregate Python checks and native-free/RGB qualification,
including 33 finite examples. Headless execution passed 2,011 tests with zero
failures and five existing ignored controls. Both corrected removal paths and
the three-second motion/wire observations passed. Renderer was not reached.

The strict changed-line gate stopped at Scene's 363 covered lines out of 391;
it requires strictly greater than 95 percent. All twelve headless crate-wide
coverage gates passed. Server's 700/721 changed-line result is derived only from
retained coverage, not an executed later gate or qualification. Full coverage,
normal success and independent retained DATA have not passed for M59.

All six failed originals remain retained. Attempts 1–3 stopped at lint/compile;
attempt 4 failed the one-second motion observations; attempt 5 passed the
three-second observations but used the unsupported local removal API. Attempt 6
passed the supported blocking-removal RPC and all headless runtime assertions.
None of these partial results is a full M59 pass or hosted acceptance.

Two additional Scene regression cases for the coverage gap are Source-complete
and independently reviewed.
Their current Source obligation is 30 feature headless/15 overlapping renderer
controls plus two separate headless support controls: 32/15 combined. These are
Source declarations, not new measured passes; run 6 used the earlier 30/13 set.
Complete final batch review and one affected formatter before another full
unchanged normal and independent retained DATA audit.
No gate is active. Existing resource floors, assertions and limits are unchanged;
no deletion, eviction, focused trial or native campaign replay is authorized.

M60's package, launcher, regression and guide Source is independently ready.
Its proof-only procedures remain held for genuine qualified inputs and results;
final integration waits for actual M59 normal and independent DATA success.

Complete Source, meaningful regressions and guides; batch review findings before
one affected formatter and the full unchanged normal/coverage gates, then an
independent retained DATA audit. See [M59_DELIVERY.md](M59_DELIVERY.md).
M1–M42/175 accepted slices and M40 Latest remain unchanged. Hosted Ubuntu snapshot
recovery, accepted predecessors, official audits and Main/readbacks still gate
acceptance/publication. No CI retry or workaround is authorized.

Earlier checkpoints below retain their historical scope.

## Current M55: complete correction batch after the second normal run

The second full normal returned actual1 in 1696.864 seconds: four cases across
three targets. The complete logs, partial packet and unqualified tree465548 are
retained at refs/qualification/m55-normal-attempt2. Coverage is incomplete; no
M55 local qualification is claimed. The first compile failure also stays failed.

The Source correction batch preserves the queued-wrench refusal on articulated
models, then uses the existing backend body-wrench API for the same physical
motion witnesses. Reset preparation preserves valid topic-remap compatibility
while validating original owner/channel identity before IO. The selected-joint
fault control observes the earlier global joint fault, full unchanged cache bits,
no new enqueue and terminal reset refusal; later sampling is not assumed to run.
All remaining assertions, native/literal bytes and strict gates remain unchanged.

Complete independent Source review and affected formatting before one full
normal/strict retry and independent retained local DATA. No successful native
campaign, fixture generator, literal audit or RAW comparison is replayed. M56
isolated Source work can proceed on qualified unaccepted M54; its final parent
waits for actual M55 normal/DATA and complete repair/overlap adoption. Accepted
M1–M42/175 slices and M40 Latest are unchanged; hosted/official/Main gates remain.

Earlier checkpoints below retain their original historical scope.

## Current M55 correction: observed Pose ID comparison

M55's complete implementation, meaningful regression Source and guides passed
independent review. The scoped reference capture, native DATA, immutable export
and literal DATA passed separately: 10 configurations, 40 inputs/calls, 65 vectors
and 95 Pose/Header records. The original DSO-input rejection remains preserved;
no SDK or provider was rebuilt. Read [the delivery contract](M55_DELIVERY.md) for scope.

The first full normal run failed during Clippy test compilation after 190.704
seconds, before Rust tests or coverage. Two diagnostics refer to one assertion:
the decoded protobuf ID is u32 and the retained fixture ID is u64. The reviewed
correction losslessly widens the actual ID for the same equality. Every wire,
value, numerical, cadence and lifecycle assertion remains unchanged. No runtime,
protobuf schema, fixture or native evidence changes. The complete failed tree,
logs and actual exit remain retained as unqualified evidence.

The single consolidated formatter passed before that run. Corrected Source,
documentation, provenance and the retained-failure retry procedure must pass
independent review before one affected full normal/strict coverage retry. No
Rust test, coverage or independent successful results audit is claimed for M55.
The original captured 95 inputs stay frozen; Root alone releases serial jobs.

M54 is locally qualified at d140e51e: 1,886 headless and 1,117 overlapping renderer
passes, 403 aggregate Python checks and 28 finite examples. M1–M42/175 accepted
slices and M40 Latest remain unchanged. M43–M54 are release-pending. Required
accepted predecessors, exact hosted success, independent official audits and
Main/readbacks still precede acceptance. Snapshot recovery is unconfirmed.

Earlier checkpoints below describe retained historical work.

## Current M54: batched corrections after the first full normal run

M54's first full normal run returned 1 after 1,680.7 seconds. Exactly two headless
cases failed: the new immutable Lidar-identity fixture changed an XML closing tag
while replacing its topic; the robot-sensor regression detected changed legacy
collision-error precedence. The original tree, logs, partial staged packet and
unqualified preservation ref remain intact. No M54 local qualification is claimed.

The XML correction targets only the complete topic element. The Server correction
preserves legacy constructor/reset typed errors while retaining full Lidar
identity, original/prepared binding and endpoint validation before publisher IO.
Finish the complete correction, regression Source and documentation with
independent review before one affected formatter and full normal/strict retry.
No test assertion, tolerance, reserved family or mandatory gate may be weakened.

Native component capture and independent native and fixed-literal DATA remain
valid and are not repeated: ten configurations, thirty captured scans and 3,096
present ranges. See [the reference](../planning/reference/m54-multiple-lidars.json)
and [delivery contract](M54_DELIVERY.md). M55 Source work remains in separate
checkouts; its integration waits for actual M54 normal and retained DATA success,
all repairs and overlap review. Root alone releases serial heavy work.
M1–M42, 175 accepted slices and M40 Latest remain unchanged. Hosted qualification,
official audit, accepted predecessors and Main/readbacks still precede acceptance.

Earlier checkpoints below describe retained historical work.

## Current M53 captured component and fixed-literal evidence

The fresh original SensorFactory component campaign and independent retained
native DATA passed. All 29 child stages completed successfully. Two modes retain
ten configurations, forty explicit camera WORLD-pose/full-model-map inputs,
forty received image/frustum pairs and eighty canonical messages across 51 mapped
libraries. The immutable fixture generation and independent fixed-literal DATA
also passed. See [the reference provenance](../planning/reference/m53-multiple-logical-cameras.json)
for the helper, exact commands, provider identities and original evidence.

The reference bypasses base scheduling and establishes no original Sim System,
ECM gathering, native demand, lifecycle or Physics equivalence. The empty map is
a direct component input, not a loaded ownerless world. Preserve the failed
fixed System diagnostic, the first precompile controller failure and all M30
original evidence. No SDK/provider was rebuilt or old campaign replayed.

The numeric and exact-wire Rust consumers are authored alongside the loaded-world
physical, independent-demand/cadence, reset/removal and endpoint controls. These
seventeen Source declarations are not measured passes. Complete independent
integration review and one consolidated formatter before the full normal hook,
strict coverage and independent retained local DATA. No M53 Rust/Python/coverage
or release result is claimed. M52 remains the latest locally qualified parent;
M1–M42 acceptance, all 175 accepted slices and M40 Latest remain unchanged.
Earlier checkpoints below retain their original historical scope.

## Current M53 independent logical-camera Source development

M53 is selected under [the delivery contract](M53_DELIVERY.md) and
[behavior guide](M53_MULTIPLE_LOGICAL_CAMERAS.md): zero through eight independent
otherwise-supported loaded Rapier logical cameras, preserving the existing
128-model-origin cap, frustum arithmetic, full relative poses, paired canonical
outputs, demand, scheduling, reset/removal and strict refusal domain. Scene,
Server/reference and Root work in disjoint checkouts; SDF independently reviews
completed Source batches. No M53 product check, formatter or native capture has
run. Complete bounded implementation, regressions and guides before the one
consolidated verification; all coverage and publication gates remain unchanged.

The exact local parent M52 `1ed59a73` passed full normal in 2046.562 seconds and
independent retained local DATA: 1848 headless and 1096 renderer tests with overlap,
403 aggregate Python tests and 26 finite examples. All strict coverage gates
passed; changed Scene lines are 18/18 in both profiles and Server is 116/118
headless, not 100 percent. Four raw comparisons ran once LAST. The first normal
compile failure and first reader path-binding failure remain preserved. This is
local qualification, not acceptance. M1–M42, all 175 accepted slices and installed
M40 Latest remain unchanged; hosted qualification is still externally held.

M53's fresh numeric and dual-wire reference will use the original SensorFactory
component directly, matching M30's accepted reference API. It grants no native
Sim System/ECM, demand/scheduling, reset/removal or Physics equivalence; actual
Rust physical/lifecycle controls remain mandatory. The fixed System/object input
diagnostic failed because its exact candidates were absent, without proving
global absence. A new fixed component-only identity/alias/static-metadata check
passed actual Root zero: 1401768-byte historical component match, ten needed names
and 35 saved reachable metadata nodes, no RPATH/RUNPATH. This is input evidence,
not a native sample, loader-equivalence proof or permission to build a provider.
Keep every original success/failure and frozen worktree. Earlier checkpoints
below are historical and do not describe current pending work.

## Current M52 native and fixed-literal evidence

The new original Magnetometer System campaign returned actual Root zero in
24.871 seconds, and its independent retained native DATA audit passed. All 34
child stages succeeded. It retained 12 configurations, 501 explicit SENSOR WORLD
pose and constant WORLD field inputs, and 42 received canonical packets across
339 mapped library paths. The 119 System calls are helper-summary and pinned
Source attestations, not invented individual call records. All 58 genuine
missing-spherical warnings and complete owned Console logs are retained. No SDK,
component or System provider was rebuilt; no native Physics or RNG/demand parity
is inferred. Current349, the two separate link providers, the exact magnetometer
component and original M17 System provider remain distinct guarded roles.

Actual fixed-literal generation and independent literal DATA also passed. The
immutable fixture copies all 501 inputs, 42 borrowed actual outputs, 459 inputs
without a matching packet and one active-peer removal event. None does not prove
an internal update or draw was skipped. The retained native component bound is
1e-15 Tesla, separate from live geometry's 1e-18 and exact full-wire obligations.
Observed numeric and canonical-wire consumer Source is independently reviewed;
no M52 Rust/Python/full normal/coverage/local-audit pass has happened yet.
Complete the consolidated Source review and formatter before one full normal
hook with every strict gate. Prior accepted phases, M40 Latest and all accepted-
predecessor, exact hosted/official and Main/readback boundaries remain unchanged.
Older checkpoint text below retains its original historical scope.

## Current delivery: M52 independent loaded magnetometers

M52 source development is active under [the reviewed contract](M52_DELIVERY.md)
and [behavior guide](M52_MULTIPLE_MAGNETOMETERS.md). Admit zero through eight
otherwise-supported loaded Rapier magnetometers with independent immutable
identity, metadata, endpoint, demand, samples and noise/reset ownership. Preserve
the existing constant WORLD field, inverse sensor WORLD rotation, Tesla units,
XYZ noise and strict bounds; do not add geodesy or another sensor/backend domain.

Source preparation began over qualified but unaccepted M50 623faca6. The new
integration checkout adopts locally qualified M51 ea921f58/tree4a29e5df after its
actual complete normal hook and independent retained DATA passed. M51 recorded
1828 headless and 1085 overlapping renderer profile passes, 403 aggregate Python
tests and 25 finite examples. All strict gates passed. These are parent evidence,
not M52 qualification. Preserve the three failed M51 attempts and every earlier
failure, original capture, pending worktree and Fuel owner change.

Root owns shared query/registration/guides and serial execution. Disjoint Scene
and Server/reference source work must finish implementation, regressions, docs
and independent whole review before consolidated verification. M52 current
provider inputs, fresh original-System native/literal DATA, observed controls,
full normal/strict coverage and independent local DATA remain pending. No M52
formatter, test, metadata, provider or reference operation has run. The finite
26th example appends to the exact old 25; it is not a browser/install proof.

M1–M42 and 175 slices remain accepted; M40 remains installed Latest. The actually
accepted predecessor, exact hosted success, independent official DATA and normal
Main/readbacks still govern publication and acceptance. Hosted snapshot recovery
has not been demonstrated; no retry, probe or workflow workaround is released.
Earlier checkpoint records below are retained in their original historical scope.

## Current M51 retained native evidence

One fresh original AirSpeed System pair/eight/active-peer-removal capture returned
actual Root zero, followed by independent retained native and fixed-literal DATA.
The immutable fixture preserves twelve configurations, all 331 explicit sensor
WORLD-pose, POINT-WORLD velocity and WORLD-wind inputs, 55 actual canonical
AirSpeed packets and 276 inputs without a matching packet. The 79 System calls
are summary and pinned-method attestations; missing packets reveal no internal
update, noise-draw or deadline state. Signed differential pressure is valid.

See `planning/reference/m51-multiple-air-speeds.json` for the actual procedures,
commands and provenance. No SDK/provider was rebuilt or capture repeated. The
new Scene exact-scalar/independent-geometry and Server exact-wire consumers are
being reviewed before one consolidated formatter and full normal/strict coverage
pass. No M51 Rust, Python, coverage, hosted or release qualification is claimed.
M1–M42 remain accepted; M40 remains installed Latest. All accepted-predecessor,
hosted, official and Main gates remain mandatory. Earlier checkpoints below are
historical records.

## Current M51 complete core Source checkpoint

M51 core Scene, Server/reference and Root identity/query/shared-registration/
example/guide Source passed independent whole review. Scene admission extends
zero through eight validated consumers while preserving the existing scalar,
wind, noise and lifecycle bodies. Server ownership/reset validation compares the
whole immutable blueprint and metadata before publisher IO. No M51 formatter,
test, lint, Cargo metadata, provider or native campaign has run.

This checkout uses exact locally qualified, unaccepted M50 623faca6/tree9f617.
Its full normal and independent local DATA passed: 1809 headless and 1075 renderer
profile passes with overlap, 403 aggregate Python tests and 24 finite examples,
with all strict gates. The first normal and two reader failures remain retained;
the original wrench deadline cause is unknown. Source preparation on M49 4ccd
is retained separately; the integrated candidate preserves all M50 changes.

Current original AirSpeed System/component inputs require a separately reviewed
bounded contract before execution. Historical successful M26 provider metadata
and M41 component91 evidence retain their distinct scopes. New original-System
multiplicity capture, independent native/literal DATA and observed consumers
remain pending. Complete the bounded Source/regression/docs review before one
consolidated qualification; Root alone releases heavy work. Accepted-predecessor,
exact hosted/official/Main/publication gates and the CI hold remain unchanged.
M1–M42 remain accepted and M40 remains installed Latest.
Earlier checkpoints below are historical.

## Current M50 normal failure and test-only correction

The first complete normal hook failed after 1622.437 seconds at an existing
Server wrench-admission test's five-second wait. Its generic diagnostic cannot
identify which of nine waits failed; the actual cause remains unknown. Both
lint stages and all 403 aggregate Python tests passed earlier. The native-free
and RGB reports, 24 finite examples and new headless pressure numeric/wire
controls are retained partial evidence, not full qualification. Headless coverage
finalization, renderer qualification and independent local DATA were not reached.

The reviewed test-only batch labels every wrench wait and prints existing
counters, ingress statistics, elapsed time and epoch on failure. The existing
post-reset readiness barrier now witnesses all three publishers before they are
used, under the same five-second limit. This strengthens readiness coverage; it
does not establish the earlier failure's cause. Production code, payloads,
numerical assertions, deadlines, test-thread settings and native/literal fixtures
are unchanged. Preserve failed tree `ff22f5c4` and every original result/log.
One complete affected normal retry requires final Source/tree/procedure review;
no separate target trial, native replay or accepted result is supplied.
Earlier checkpoints below are historical.

## Current M50 retained native and literal evidence

The fresh original AirPressure System capture and both independent retained
native and fixed-literal audits returned actual Root zero. The checked-in fixture
retains all 441 sensor WORLD-pose inputs, 42 received pressure/variance packets
and 399 inputs without a matching packet, across twelve configurations and
three modes. The 104 System calls are summary and pinned-method attestations;
missing packets do not expose internal update, noise-draw or deadline state.
Reference altitude remains zero; initial sensor height is not subtracted.

See `planning/reference/m50-multiple-air-pressures.json` for the exact program,
commands, provider inputs and provenance. No SDK/provider was rebuilt or capture
repeated. Scene numeric and Server canonical-wire consumers are being completed
before final independent Source review and one consolidated normal/strict coverage
pass. No M50 Rust, coverage, hosted or release result is claimed. M1–M42 remain
accepted, M40 remains installed Latest, and all release gates remain mandatory.
Earlier checkpoints below are historical.

## Current M50 original-System reference outcome

One reviewed fresh AirPressure System campaign returned actual Root zero in
23.920 seconds, with all 33 child stages successful. It retained three modes,
twelve configurations, 441 actual sensor WORLD-pose input rows and 42 received
canonical pressure packets. The 104 System calls are helper-summary and pinned
method attestations, not 104 fabricated individual raw observations. All current
349 inventory guards, the two link-provider inputs, exact pressure component and
retained original System provider remained bound. No SDK/provider was rebuilt.

Independent native and fixed-literal DATA, observed consumers and the full M50
Rust/normal/coverage checks remain pending. This is controlled original-System
input evidence, not native Physics, RNG/demand equivalence or release acceptance.
Every prior Source checkpoint below is retained as history.

## M50 current Source integration checkpoint

The reviewed contract is clear. Scene admission/lifecycle regression Source,
Server identity/reset/canonical regression Source, the original-reference helper
and live C++ peer, and Root's borrowed identity query are integrated after whole
independent Source review. Runtime model/noise/packet/publisher bodies and old
controls remain preserved within the exact approved change scope. The finite
24th-example and peer/verifier integration Source is authored for final review.
No M50 test, formatter, native capture or qualification has run.

Reference preparation found the pressure Sensor library outside M49's existing
349-node inventory. A separately reviewed bounded component identity/alias/static
dependency diagnostic must establish that additional exact input before any
reference release; all existing 349 and separate link-provider guards remain.
Actual native/literal DATA and later observed consumers remain held. M49 stays
frozen locally qualified, with release acceptance still requiring hosted/official
and Main evidence. Earlier Source-selection text below is historical.

# Current development: M50 independent loaded air-pressure sensors

M49 is locally qualified at `4ccd4f06` with full normal and independent retained
DATA passed: 1791 headless and 1066 renderer tests with overlap, 403 aggregate
Python tests, 23 finite examples and every strict coverage gate. Its original
failures remain preserved. M1–M42 remain accepted; M40 remains installed Latest.

M50 selects zero through eight independent air-pressure sensors in the existing
loaded Rapier domain. Read [M50_DELIVERY.md](M50_DELIVERY.md) for closed behavior,
exact dependencies, disjoint leases and acceptance criteria. Contract review
precedes authorship. No M50 implementation or execution is claimed. Root owns
shared seams and serial heavy work; Scene and Server/reference Source may proceed
in parallel after the contract is clear. Accepted-predecessor and all hosted,
official and Main gates remain mandatory; no CI retry/probe is released.

The earlier checkpoints below are retained historical records.

## Current M49 named-type fixture correction (2026-11-12)

The corrected fixture export and independent literal DATA both returned actual
Root zero. The exact named `NativeCase` alias replaces only the long `CASES`
type declaration; all five structs and the entire constant value body are
unchanged. All 441 inputs, 42 observed packets and 399 inputs without matching
packets remain bound to the same passed native DATA. No native capture, native
DATA audit, numerical assertion, packet byte, lint allowance or threshold changed.

The corrected fixture/provenance are integrated, with both old and new readers,
outputs and actual receipts retained. The first normal run remains failed at
Clippy; no M49 Rust test or coverage pass is claimed. Independent correction
review and the affected full normal retry remain required. Runtime consumers
and all formatted production Rust stay unchanged, so no formatter rerun is
selected. Earlier checkpoints below are historical records.

## Current M49 qualification failure and bounded correction (2026-11-12)

The first complete normal attempt stopped at Clippy after 182.222 seconds.
The generated reference fixture's five-part `CASES` tuple declaration triggered
`type_complexity`; product tests and coverage were not reached. Preserve failed
tree `b023aaa1`, its qualification ref, original method, logs, packet and actual
exit-one receipt. No native or literal DATA result is relabelled.

The available failure batch contains this one declaration issue. A reviewed
successor generator will give the same tuple a named `NativeCase` alias, and
the fixed-literal reader will admit exactly that declaration. All five record
structs, 441 inputs, 42 output values and canonical bytes remain unchanged.
This uses no lint allowance or threshold change. New export and literal DATA
outputs and actual exits are required before fixture replacement and one
affected full normal retry. No new native capture or native DATA audit is needed.

Earlier checkpoints below are retained historical records.

## Current original-system reference outcome (2026-11-12)

The corrected fresh capture returned actual Root zero in 24.204 seconds with
all 34 child stages successful. It recorded three modes, twelve configurations,
441 input rows and 42 received canonical packets, with 104 System calls attested
by the helper summary and pinned method. All 349 SDK identities and the two
separately pinned link providers stayed guarded. Independent retained native
DATA and fixed-literal DATA then passed with actual Root zero. The checked-in
fixture preserves all 441 inputs, 42 observed packets and 399 inputs without a
matching packet. The latter do not expose internal update, noise-draw or deadline
state. Scene numeric and Server exact-wire observed consumers are being completed
before one consolidated normal/strict coverage pass; no M49 Rust result is claimed.
See `planning/reference/m49-multiple-altimeters.json` for the pinned original
capture, generator, independent readers and actual Root receipts. Preserve
both earlier failed controllers and every original packet; none is relabelled
successful or merged into the fresh capture. The lifecycle correction retains
the removal marker through original System updates, then physically removes the
entity, with the unchanged five-second actual publisher-retirement assertion.

The earlier selection and failure checkpoints below are historical records.

# Current development: M49 independent loaded altimeters

Read [M49_DELIVERY.md](M49_DELIVERY.md) for the bounded scope and exact leases.
This checkout starts at clean local M48 a204d94b, whose full normal hook and
independent retained DATA passed: 1773 headless and 1057 renderer profile passes
with overlap, 403 aggregate Python tests and 22 finite examples. All strict
coverage gates passed; changed Scene lines are 57/58, not 100 percent.
M49 implementation, helper and peer Source passed independent review; no Rust,
full-normal or coverage result is claimed. Its first original-system reference
controller failed after a successful helper compile and pair child at a strict
mapped-library inventory check. All339 saved library paths were inspected: two
explicit link-recipe providers, libdw and libelf, were outside the existing349
SDK node set. The original failure is retained; exact supplemental inputs and
procedural review are required before a separately released retry.
M1–M42 remain accepted, M40 remains Latest, and M43–M48 stay unaccepted.
Actual accepted predecessor, hosted/official and normal Main/readbacks still
precede publication/acceptance. No CI retry/probe/workaround or eviction.
Preserve every frozen tree and original failed attempt.

The earlier delivery checkpoints below are historical records.

# Current development: M48 IMU world-heading references

[M47](M47_DELIVERY.md) passed its complete provisional local normal hook in
2062.426 seconds and independent retained DATA at clean commit `f27a37fd`:
1757 headless and 1047 renderer profile passes with overlap, 403 aggregate
Python tests and 21 finite examples. Strict coverage passed; changed Scene
production lines were 152/155. All original failed attempts remain retained.
M47 is locally qualified, not accepted. M1–M42 remain accepted and M40 remains
the installed Latest. Hosted qualification and publication of M43–M47 are still
blocked; no retry, snapshot probe or workaround is released.

[M48](M48_DELIVERY.md) selects heading-aware ENU/NED/NWU and CUSTOM/world IMU
orientation references in a closed metadata domain. The new isolated checkout
starts at exact local M47 `f27a37fd`; every frozen predecessor remains intact.
Independent contract and complete implementation review passed. The fresh
original Sensors/Math capture completed in 32.537 seconds with all 89 stages
successful; independent native and fixed-grammar literal DATA passed for all
44 configurations and 264 packets (240 orientation-present, 24 absent).
Audited fixture consumers are integrated for final Source review. Rust tests,
lint, strict coverage and the complete normal hook remain pending. Preserve the
strict gates and actual
accepted-predecessor/hosted/official/Main requirements before publication.

The third normal attempt stopped after 4.166 seconds at the required formatting
check, before compilation or tests. Root had omitted formatting the completed
Server assertion correction. One affected Server formatter pass then completed
in 0.685 seconds, changing only that file's layout. Assertion values, inputs,
production code and fixtures remain unchanged. Preserve the failed `96426233`
source and its original method/log/result; all three actual failures stay failed.
The complete normal hook, strict coverage and independent local DATA still need
a successful actual result. No native reference or DATA reader is repeated.

The second complete normal attempt failed after 1600.379 seconds with two
headless test failures, after both Clippy runs and all 403 aggregate Python
cases passed. One new Scene test compared a held 100 Hz reading to the current
moving pose; its correction preserves all 21 strict geometry checks using the
actual acquisition pose, with exact held-sample and 1/10/20 ms cadence assertions.
One Server test expected a loader refusal for decimal `1e-310`, which the pinned
parser rejects first. That original input now requires exact parser InvalidValue
and conversion diagnostics; five loader-refusal cases remain, and normal degree
input `1e-306` adds the actual subnormal-radian loader boundary. Production code,
numerical bounds and native/literal fixtures are unchanged. Preserve the failed
`e6eeb0a4` source, all logs and partial evidence. No successful full normal,
renderer/strict coverage or local DATA result is claimed. No reference, reader
or formatter is repeated; a separately reviewed full retry remains required.

The first complete M48 local normal hook failed after 84.164 seconds at
required Clippy compilation, before tests or coverage. Rust E0502 identified
an immutable borrow of spherical metadata extending across later model mutation.
The correction stores one owned metadata snapshot during loading and borrows it
only for validation. Values, reference arithmetic, all assertions and every
native/literal fixture are unchanged. Original failed tree `47840d61`, logs and
packet remain retained. No native capture, data reader or formatter is repeated;
a separately reviewed complete normal retry remains required.

The earlier checkpoints below remain historical; their pending-result wording
does not override the actual M47 local outcome recorded above.

# Current development: M47 Gaussian odometry velocity noise

[M46](M46_DELIVERY.md) passed its full provisional local normal hook and
independent retained local audit at clean commit `cca6b07d`: 1735 headless and
1034 renderer profile passes with overlap, 402 aggregate Python tests, 20 finite
examples and all strict coverage gates. Both prior failures remain retained.
It is not accepted or publicly qualified. M1–M42 remain accepted and M40 remains
the installed Latest release; M43–M46 hosted/publication closeout remains blocked.

[M47](M47_DELIVERY.md) implements finite nonnegative Gaussian velocity noise
for the existing Two/Three OdometryPublisher, with native covariance and unchanged
physical pose. The complete Source, regressions and guides passed independent
review, and the single formatter passed. Fresh original-component capture and
independent retained native DATA passed: 16 modes, 18 configurations, 1418 inputs,
1316 triples and 3948 packets. The reviewed current-provider update preserves the
historical baseline; all 349 current nodes and 337 loaded providers were checked.
At this pre-verification source checkpoint, Rust tests, lint and the full normal
hook remain pending; scoped native DATA does not supply that qualification.
No CI retry/probe, acceptance waiver, package/browser refresh or Fuel work is
selected. Accepted predecessor and all normal hosted/official/Main gates remain
mandatory for publication and acceptance. See the M47 contract for both retained
pre-child failures and the precise current-provider baseline.

The first full normal hook returned actual exit one after 178.137 seconds at
required Clippy compilation, before tests or coverage. One retained M46 Server
replay initializer omitted the newly required `gaussian_noise` field. The complete
21-constructor/alias inventory found no further omission. Its correction adds
only `Default::default()` noise to that initializer, preserving every original
numeric and wire assertion. The failed source/tree/logs remain retained; no
native reference, audit or formatter is repeated. A separately reviewed complete
normal retry is still required; no Rust or coverage pass is claimed.

The older checkpoints below remain historical, with original evidence preserved.

<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

## Current bounded task: M46-ODOMETRY-ROTATION

The second normal attempt passed both required Clippy runs, then ran 402 Python
cases with one failure: the mocked finite-example invocation assertion still
expected 19 rather than the selected 20. The correction keeps an exact 20-run
assertion, the original nineteen-source prefix and every per-source check.
Runtime profiles and feature numeric tests were not reached. Both failed
attempts remain retained; no native/literal evidence is repeated.

The first complete normal attempt stopped at required Clippy before tests or
coverage: `clippy::neg_multiply` rejected `value * -1.0`. The reviewed correction
uses unary `-value`, preserving every admitted finite value and signed-zero bit,
with all subsequent arithmetic and assertions unchanged. The original failed
source tree, packet and logs remain retained. No lint suppression or native/data
replay is selected. The corrected full normal pass is pending separate review.

M46 implementation and regression Sources have passed independent leaf review.
Fresh original-system capture and independent native/literal audits passed for
30 modes, 32 configurations, 1,020 calls and 858 triples (2,574 canonical messages).
The audited fixture and reviewed numeric/encoder consumers are integrated.
Complete integration review now precedes one full normal/strict local pass.
Rust checks and all hosted/publication prerequisites remain pending; reference
proofs do not supply Rust acceptance.

Read [M46 delivery](M46_DELIVERY.md). Develop finite local/global rotational
odometry offsets over exact locally qualified M45 `e66c21e7` in the new isolated
worktree. M45 full normal and independent local audit passed; neither that
provisional result nor the M46 selection supplies acceptance. M1–M42 remain
accepted at `73e03b9f`; M43–M45 hosted/publication closeout remains pending after
snapshot setup failure. No retry or probe is released.

Light owns the Scene production/private test leaf, new public regression leaf,
example and feature guide. Messages owns new Server regression and native-peer
leaves plus the bounded native case design. Root owns shared registration,
qualification, approved old-negative dispositions, planning/status and serial
execution. SDF independently reviews the whole Source. Preserve all old native
fixtures and every unrelated assertion. Complete implementation, meaningful
regression Source and docs, batch review fixes, then one consolidated pass.
The actual native and literal audits passed before fixture insertion. Rust
qualification remains pending; no package/browser refresh is selected.

## Historical M45 pre-qualification checkpoint: M45-IMU-CUSTOM-WORLD

Read [M45 delivery](M45_DELIVERY.md). This integrated candidate uses exact locally
qualified a08f0b50, preserving all containment repairs; accepted Main remains 73e03b9f.
The prior Source lane 9a is preserved. M43/M44 exact hosted/official and publication
are pending after the fresh snapshot TLS/select setup failure. No retry is released.
Reviewed timing amendment 5d823a75 permits one provisional full LOCAL qualification
only after complete Source/integration/tree/runner review and serial Root release.
Actual accepted predecessor, exact hosted/official and Main/readbacks still govern
publication and acceptance. No original proof is relabelled or transferred.

The standalone original-component capture and native/literal DATA passed for 14 new
configurations and 84 calls. The exact fixture/provenance copies are inserted. Light
and Messages completed the two new private native consumer leaves; SDF reviewed
their complete Source. Root owns append-only cfg hooks and shared Source/docs,
complete integration review and serial release. No M45 Rust check has run. Preserve
every old fixture/test body, strict bound and actual failure; no placeholders.

M45 admits effective CUSTOM angles with explicit world parent while preserving
legacy initial-relative bits and named frame arithmetic. Its new development
example becomes the nineteenth finite CLI qualification input; M44's eighteen
archived examples and package bytes remain unchanged. M45 selects no new
package or browser refresh. Keep actual physical-force, lifecycle, canonical
wire, strict coverage and exact hosted checks before acceptance.

## Historical source preparation: M43-IMU-REFERENCE and M44-INSTALL

Read [M43/M44 delivery](M43_M44_DELIVERY.md). M41/M42 are accepted at
Main `73e03b9f` after exact hosted CI and independent official DATA. This is the
actual parent for the continuing wave. Source preparation overlapped closeout;
native capture, tests, builds and installation still require complete independent
source review and Root's serial release.

Light owns the private Scene `imu_reference.rs`, its private and public
regression leaves, the M43 example and feature guide. Messages owns the Server
`imu_reference_frames.rs`, transport peer and separate component-reference C++
leaves. Captured-value fixture and encoder replay follow actual independent
native/literal DATA, without placeholder expected values. Root owns shared
Scene admission/reset-test disposition, peer registration and retained-profile
requirements, packaging and its regressions/guides, and canonical integration.
SDF reviews complete batches and test intent independently.

The finite native case table is agreed before oracle authorship: 17 fresh
configurations and six prospective forced updates each, with direct WORLD
inputs and actual native sensor-axis setter values recorded separately. Named
references reject spherical metadata; CUSTOM preserves legacy bits/admission.
Required controls cover all named frames, actual motion, disabled orientation,
noise/covariance preservation, demand/pause/rate, reset/removal and canonical
shutdown. M44 requires fresh eighteen-example, three-executable distribution,
clean installation, force/feedback and default robot browser evidence. Neither
scope is accepted by source completion or by an earlier phase's evidence.

## Historical bounded task: M38-ODOM3D

Accepted parent: `6d2d53209174e3398b79909553e4fc5fd7fe05e8`.
Implement the noiseless 3D ground-truth odometry contract in
[M38_DELIVERY.md](M38_DELIVERY.md). Light owns Scene leaves/tests/example/guide;
Messages owns the Server canonical peer and scoped original-system native
helper/cases/fixture; Root owns shared configuration and registration, CLI/
profile/retention requirements, integration and records. SDF independently
reviews semantics and test integrity. Agree exact arithmetic, type seams and
finite native cases before runtime edits. The implementation and actual native
fixture are now integrated; the guide names concrete controls. Original-system
capture and independent native/literal DATA passed. Rust tests/lint and full
qualification have not run. Finish whole Source review before the consolidated
mandatory verification.

The older current-task descriptions below are historical checkpoints.

# Implementation work packets

Each packet is a bounded assignment. Split broad packets by the named system or
test family before implementation; do not give an agent “port all sensors” with
no acceptance boundary. IDs below are stable coordination labels, not GitHub
issue numbers. No task is implemented merely because it appears in this plan.

Every packet follows AGENTS.md and produces an evidence record containing owned
files, upstream test IDs, dependency decisions, code revision, exact check commands,
results, coverage, relevant benchmark data, reviewer, and unresolved limits.
The task author cannot independently retire tests or approve coverage exclusions.

## Historical atomic M21–M25 task checkpoint (superseded)

[M21_M25_DELIVERY.md](M21_M25_DELIVERY.md) supersedes serial intermediate-parent
qualification with one new complete candidate over accepted M20. Every per-phase
functional/native criterion, mandatory normal/strict/exact-hosted check and fresh
M25 distribution/install/four-example/browser/shutdown proof remains required.
Two preserved hosted LLVM SIGBUS failures have unknown cause. Aligning debug info
and adding bounded resource diagnostics does not establish a cause fix.
Root owns integration/shared planning/CI/publication; existing source and procedure
owners keep their narrow leases. Preserve all old trees and evidence. Complete
independent Source review before the consolidated verification, serialize heavy
work, publish actual results to the tracker and continue after acceptance.

### Historical M10-CREDIT contract (closed; preserve evidence)

The initial M10 candidates passed required checks, but actual browser acceptance
failed. The qualified diagnostic viewer confirmed event-queue overflow. This
bounded correction uses the existing browser bridge, not a new transport or
runtime. Root owns `crates/gz-sim-cli/src/web/socket.rs`, the new credit contract
and real-socket regression sources, their parent test-module declaration and
M10 documentation. The viewer owner implements the matching maintained Session
and PrimitiveBackend contract; a separate reviewer checks both complete batches.

Only lifecycle clients opting into publication credit gain the one-flight
normal-publication handshake. Existing no-credit protocols, the 64-event queue,
64-event drain, world request ordering and exact committed replies remain intact.
Ready parsing must work while a world reply is pending; same-connection reset
must preserve an already validated transport token. No simulation wait belongs
in this network backpressure path. See [the exact contract](M10_WORLD_LIFECYCLE.md).

New regression families cover strict credit JSON and token exhaustion, paused
and advancing native worlds, command-before-Ready, delayed committed replies,
reset-before-old-Ready, query negatives and reconnect isolation. Keep all retained
assertions. Complete implementation, regression source, docs and independent
source review before one consolidated required pass per affected repository.
This historical packet required actual stable browser spawn/remove/state/reset
and reconnect observations. They passed at the revisions in M10_ACCEPTANCE.md;
source review or CI alone never constituted acceptance. Preserve all earlier
failures and frozen trees; this section does not authorize requalification.

## Foundation and reference tasks

| ID | Depends on | Assignment | Exit evidence |
| --- | --- | --- | --- |
| REF-01 | None | Verify all main-branch baseline SHAs, dependency compatibility, licenses, build flags, native versions; create immutable reference environment | Baseline lock and reproducible build/run instructions; clean source hashes |
| REF-02 | REF-01 | Enumerate runtime tests across CTest/gtest/Ruby/Python/GPU/backend combinations; reconcile static inventory | Complete ledger schema and expanded entries; baseline results and blocked cases |
| REF-03 | REF-01 | Capture balanced robotics benchmark suite and hardware/environment | Raw samples, variance, accuracy baselines, proposed budgets ratified |
| PHY-00 | REF-01 | Record Avian/Rapier evaluation; enforce full-f64 gate and probe prioritized Rapier | Reproducible precision/capability evidence, exact versions/features, selected backend ADR; no invented performance conclusion |
| API-01 | REF-02 | Freeze minimal IDs/time/frames/errors, SDF resolver, message/transport and Fuel integration contracts | Contract fixtures; independent repos compile without Bevy; no reverse dependency |
| QLT-01 | REF-02 | Implement staged-tree pre-commit, profile matrix, CI, coverage accounting, ledger checks | Known-bad fixtures fail; exactly 95% fails; missing profile fails; no false success from empty code |
| CORE-01 | API-01, QLT-01 | Create minimal Cargo workspace, f64 values, stable IDs, simulation epochs and clock | Unit/property tests >95%; no-render dependency closure; documented toolchain/Bevy pairing |
| EXT-01 | Superseded by D26 | Retired external system-loader task; implement ACT-01 instead | Record explicit dispositions for loader-only tests; retained system behavior remains required |

## Independent repository tasks

| ID | Depends on | Assignment | Exit evidence |
| --- | --- | --- | --- |
| MSG-01 | REF-01, API-01 | D29: preserve canonical proto definitions, imports/options, tags/defaults and notices; remove the repository runtime after consumer migration | Schema hashes/import closure, standard protoc validation and consumer-owned generated binding checks |
| MSG-02 | Superseded by D29 | Stop custom runtime/reflection/conversions/CLI/codec parity and optimization | Explicit dispositions; retain consumer schema/wire semantics; no library-completion claim |
| SDF-01 | REF-02, API-01 | XML/schema infrastructure and minimal validated semantic world/model/link/joint/sensor | Includes/frames/defaults/error cases and C++ differential tests |
| SDF-02 | SDF-01 | Complete version conversions, nested/merge/frame/inertia/import/export semantics | Full retained sdformat ledger, fuzzing and large-model benchmarks |
| NET-01 | Superseded by D28 | Stop gz-transport-rs and ZeroMQ protocol implementation | Preserve historical evidence; remove consumer dependency through ZEN-01 |
| NET-02 | Superseded by D28 | Stop standalone transport runtime, parity and CLI work | Retained simulator networking/logging requirements map to focused Zenoh consumer tasks |
| ZEN-01 | MSG-01, API-01 | In gz-sim-rs, use upstream Zenoh directly; replace legacy transport and packaged Messages dependencies | Pinned Zenoh/proto/codegen inputs; real C++ Zenoh pub/sub/query/reply; bounded queues, reset/removal/shutdown, schema fidelity, strict owned coverage and equal-work measurements |
| FUEL-01 | API-01; Fuel owner readiness | Consume existing Rust API, offline cache/resource identity, cancel/error contract | Local fixture integration; pinned owner-provided revision; no reimplementation |

SDF, Fuel and schema maintainers own their repository tasks. ZEN-01 belongs to
gz-sim-rs; it does not create a transport successor. gz-sim-rs owns acceptance at the
integration boundary. P0 contract coordination can proceed before any producer
is complete. QLT-01's policy must be applied in each independent repository.

## Runtime and physics tasks

| ID | Depends on | Assignment | Exit evidence |
| --- | --- | --- | --- |
| CORE-02 | CORE-01 | Bevy schedules, world ownership, spawn/remove transactions, pause/step/reset | Priority/load-order, deferred-command, timestamp, epoch, multiworld isolation tests |
| ACT-01 | CORE-02, LOAD-01 | Built-in system demand from loaded world/models; activate/retire schedules and owned resources on structural lifecycle transitions | Absent-system zero invocation/allocation/worker/endpoint controls; last-consumer teardown; dependency/shared-consumer behavior; reactivation/reset/multiworld and equal-work CPU/memory evidence |
| PHY-01 | CORE-01, PHY-00 | Capability API, normalized construction/command/state/query contracts | Tested unsupported-feature diagnostics; no template-framework port |
| PHY-02 | PHY-01 | TPE behavior port using reusable geometry | Upstream TPE motion/contact/lifecycle tests and benchmark |
| PHY-03 | — | Cancelled by owner D33: DART product backend | Historical evidence retained; no further implementation or parity requirement |
| PHY-04 | — | Cancelled by owner D33: MuJoCo product backend | Historical evidence retained; no further implementation or parity requirement |
| PHY-05 | PHY-00, PHY-01, CORE-02 | Built-in Rust engine adapter and explicit stepping/precision | Capability probes, accuracy tests, sync overhead and deterministic-profile evidence |
| SENSOR-01 | CORE-02, PHY-01 | Sensor scheduler, streams/seeds, shared models, IMU first | Rate/pause/reset/noise/frame tests and timestamp correctness |
| IO-01 | CORE-02, MSG-01, ZEN-01 | Clock/pose publication, commands/services and bounded transport integration | Actual C++ Zenoh client interoperability; ordering/backpressure/shutdown cases |
| LOAD-01 | SDF-01, FUEL-01, CORE-02 | Validated semantic scene to ECS/backend construction; local/offline resources | Original world fixtures, failure atomicity, frame and ID checks |
| SLICE-01 | LOAD-01, PHY-02, PHY-05, SENSOR-01, IO-01 | First complete exact-step SDF world | P2 report with C++ comparison, >95% code coverage, performance data and remaining scope |

D33 cancels PHY-03 and PHY-04. PHY-05 (Rapier f64) and PHY-02 (TPE) remain;
no active task depends on completing a retired adapter.
Temporary SDF bridges require explicit removal tasks. Existing ZeroMQ/runtime
consumer pins remain historical transition inputs until ZEN-01 replaces them.

### Active bounded continuation

Root owns the LOAD-01/IO-01 world Light pose follow-up: Scene child construction,
Server hierarchy validation/publication, renderer refusal and lifecycle/real C++
peer tests. It depends on accepted SDF `13adde94`, existing pose ownership and
Zenoh codegen contracts. Native reference source, commands and library hashes are
checked in; [WORLD_LIGHT_POSES.md](WORLD_LIGHT_POSES.md) records prior-reviewed
assertion transitions and exact remaining qualification. This does not advance
B03, SDF radiometry, original-system completion or performance acceptance.

## Assets, rendering, and complete sensors

| ID | Depends on | Assignment | Exit evidence |
| --- | --- | --- | --- |
| ASSET-01 | LOAD-01 | URI policy, decoded cache, mesh/material/animation loaders | Collada/STL/OBJ/glTF and relative-resource fixtures; hashes/provenance |
| ASSET-02 | ASSET-01 | Terrain/DEM, heightmaps, geospatial/environmental data | CRS/origin/nodata/scale and sensor sampling fixtures |
| RENDER-01 | CORE-02, ASSET-01 | Bevy offscreen scene extraction, f64/frame/origin conversion | No-window GPU test; large-coordinate transforms; scene lifecycle and readback cancellation |
| RENDER-02 | RENDER-01, SENSOR-01 | RGB/depth/RGBD, calibration, distortion, wide-angle | Analytic projection/range tests, image fixtures, no hidden temporal postprocessing |
| RENDER-03 | RENDER-02 | Segmentation, bounding boxes, thermal and visual effects needed by retained features | Label/geometry/thermal checks and mapped upstream assertions |
| SENSOR-02 | SENSOR-01 | Force/torque, pressure/speed, altimeter, magnetic, NavSat/logical/contact | Model/noise/units/frame tests and backend capability errors |
| SENSOR-03 | RENDER-01, PHY-01 | CPU/GPU lidar and collision-versus-visual query semantics | Range/intensity/clipping fixtures and per-sample acquisition timestamps |
| SENSOR-04 | SENSOR-02, ASSET-02 | DVL, environmental and remaining sensor models | Complete sensor ledger; deterministic environmental sampling and error behavior |
| SENSOR-05 | RENDER-03, SENSOR-03, SENSOR-04 | Multi-sensor scheduling, overload, lockstep, cancellation | Exact sample counts, bounded buffers, reset epochs, CPU/GPU coverage and soak |

## Simulator system families

`planning/system-migration.json` assigns every one of the 78 inspected system
directories to a work family. The assignment is provisional organization; all
entries remain `not_migrated`. For each system, enumerate SDF parameters, default
behavior, topics/services, components/outputs, backend requirements, reset behavior,
fixtures, and tests before converting it.

| Family | Main dependencies | Acceptance focus |
| --- | --- | --- |
| SYS-CONTROL | SLICE-01, physics joints | Drive/controller commands, limits, odometry, actuation timing, reset |
| SYS-DYNAMICS | Physics forces/contacts, residual math | Aero/hydrodynamics, buoyancy, propulsion, slip, energy and battery behavior |
| SYS-SENSOR | Sensor packets | Sensor registration/configuration, topics, frames, labels and timing |
| SYS-WORLD | CORE-02, IO-01, ASSET-01 | Commands, scene/pose/state, attachment, lifecycle, labels and world services |
| SYS-COMMS | IO-01, ZEN-01 | Acoustic/RF/perfect communications, endpoint and triggered publishing semantics |
| SYS-MEDIA | RENDER-03, ZEN-01, ASSET-01 | Recording, replay, exporting, effects and scene assets; no GUI dependency |
| SYS-EXTENSION | API-01, ACT-01 | Retained external-language workflows and lifecycle mapping; no dynamic simulation system loader |

Family completion requires all its directory entries and associated tests, not
just one representative implementation. Split each family into individual system
or tightly coupled system packets for independent implementation/review.

## Non-system features and release tasks

| ID | Depends on | Assignment | Exit evidence |
| --- | --- | --- | --- |
| WORLD-01 | CORE-02, IO-01 | Server configuration, world services, entity factory, assets, multiworld lifecycle | Public API/service inventory mapped; concurrent lifecycle tests |
| WORLD-02 | WORLD-01, SYS-WORLD, ZEN-01 | Levels, performers, distributed stepping/state and resource behavior | Baseline distributed/levels tests; partition/disconnect/recovery and exact-step evidence |
| LOG-01 | ZEN-01, IO-01, SYS-MEDIA | State and transport recording/replay, format migration | Existing logs readable, ordering/time/seek/reset and output fidelity tests |
| SCRIPT-01 | API-01, WORLD-01 | Inventory Python workflows; choose focused bindings and migration path | Retained Python tests/workflows; no accidental GIL on numerical hot paths |
| CLI-01 | IO-01, MSG-01, ZEN-01, SDF-02 | CLI grammar, discovery/dispatch, version/help, package ownership | Retained simulator/SDF/Fuel command assertions ported; no standalone Messages/transport CLI |
| GUI-API-01 | WORLD-01, RENDER-01 | External GUI scene/control/asset/sensor protocol and version negotiation | Separate minimal protocol client tests; snapshot/delta resync; no GUI implementation |
| MATH-01 | REF-02; domain demand | Audit every math header; reuse or implement documented semantic gap | Complete reuse map, exact tests, no redundant generic math library |
| PERF-01 | REF-03, SLICE-01 onward | Profile and optimize measured bottlenecks continuously | Equal-work before/after evidence; representative release targets met |
| PKG-01 | CLI-01, all backends | Linux packages, native dependency discovery, feature bundles and install layout | Clean-machine install/run; no gz binary conflicts; reproducible versions |
| QUAL-01 | All retained scope | Full parity/coverage/performance/lifecycle/release matrix | P7 acceptance report; no unrun required cases or unexplained exclusions |

MATH-01 is incremental: do not build an unused port of the entire math library
before a simulator slice needs it. Requirements for public standalone math behavior
remain tracked in the ledger even when not used by the first slice.

## Assignment template

```text
Task ID and concrete behavior:
Prerequisites and current contract versions:
Owned files/modules and shared interfaces affected:
Upstream SHA, source, test IDs, and fixture hashes:
Reuse decision and any missing library semantics:
Implementation and failure/lifecycle cases:
Required configurations and exact check commands:
Performance probe and accuracy budget, if relevant:
Evidence artifacts and reviewer:
Remaining blockers and next bounded task:
```

Keep worktree/file ownership explicit when several contributors work at once.
Integrate shared interfaces before dependent implementations. An agent may not
change a producer contract solely to make its own consumer tests easier.

## Historical cancelled continuation (D33): free-root serial three-body DART

Root owns a bounded LOAD-01/PHY-03 consumer follow-up in `joints.rs` and new
Scene tests. The accepted backend already supports a contact-free free root with
two serial revolute children. Admit only that topology, preserve the existing
world-rooted two-link case, construct constraints in parent-first order and keep
public joint declaration identities. Retain branch, cycle, world-rooted
three-link, larger-tree, limits and other-backend refusals. Tests must cover
reversed declarations, actual backend comparison, loaded reset and removal. No
old refusal assertion changes before independent disposition review. No new
backend contact admission, B03, multiple collisions or native SDF parity claim.
Implementation and test-code findings are batched in this candidate; final source
review and consolidated qualification are pending. See
[SCENE_FREE_THREE_BODY.md](SCENE_FREE_THREE_BODY.md).

### Cancelled by D33 — PHY-03 / SLICE-01: bounded DART compound collisions (historical)

Root owns Scene loading and regression controls; the physics owner owns the
existing DART adapter and concrete shared physics contracts. Work is isolated
from accepted Main. Reuse pinned DART ShapeNodes on one BodyNode, existing shape
validation, resolved explicit inertia, transaction/reset machinery and numerical
bounds. Admit one to three primitive shapes with link-local transforms in DART;
keep the centered-COM restriction and all other backend refusals. Validate every
collision's geometry, surface and pose and retain each child's density metadata.
One body/ID represents a link; collision entities keep their declaration IDs.

Complete implementation, regression code, native comparison and independent
source review before one consolidated mandatory verification pass. Meaningful
controls cover final-shape refusal, local bounds, native body/shape semantics,
loaded reset/removal and unchanged other-engine refusals. B03 still needs offset
COM and full trajectory/equal-work evidence; this task does not accept it.

### Cancelled by D33 — PHY-03 / SLICE-01: explicit offset COM for DART compounds (historical)

This follow-up is isolated from the qualifying centered-COM feature. The physics
owner extends the existing explicit-inertia COM, origin-conditioning and velocity
semantics to every local primitive; Root owns Scene integration. Scene admits
resolved nonzero COM only for DART solid links, including the existing single
collider path. Other engines and planes keep their prior refusal. No new solver,
aggregate inertia, dependency, numerical threshold or runtime system is added.

Complete source, regression code, native observer source and independent review
before verification, and wait for the parent feature's acceptance. Native origin
velocity and COM velocity must be distinguished explicitly. Preserve old tests;
propose exact same-input stronger-positive transitions for independent review
before any fold. B03 loading, full native trajectories and equal-work evidence
remain separate acceptance requirements; do not infer them from this guard.

### Cancelled by D33 — PHY-03 / SLICE-01: atomic two-edge serial construction (historical)

The offset-COM prerequisite is accepted at `e55356c8`. This next bounded feature
adds one cold DART operation for three isolated dynamic free bodies and two
serial revolutes. Candidate preparation uses mature native skeleton cloning;
all final contact, velocity, grid and conditioning guards remain. Complete
readback/metadata precede the live swap and two-ID commit. Reset retains only
accepted pair grouping; partial removal must preserve the existing paths.
Root owns Scene integration and declaration identity. Independent source review
approved the exact B03 construction/reset transition, not its execution or
acceptance. Actual native B03 observations are available, while the new direct
wrench/contact/reset reference has one successful reviewed-source capture,
with independent raw/source/library audit passed. Finish all
source and data review before one consolidated mandatory gate; performance and
general topology/contact support remain separate.

### Historical cancelled adapter continuation (D33) — SDF-02 / PHY-03: adopt the checked radian constructor correction

Root owns the isolated `work/producer-euler-adoption` consumer batch. It retains
the independently reviewed child-frame serial construction and fresh-reference
changes from the failed 361-second qualification without editing that attempt's
tree or observations. The producer's native radian-Euler constructor difference
is confirmed by source; its role in B03 divergence remains unproven.

Owned integration paths are the dependency revision and lock source,
`sdfb246_admission.rs`, and adoption/handoff documentation. The seven supplemental
inputs retain their order and URI. Omitted density and an explicit identity
collision pose gain reviewed positive physical, metadata and lifecycle controls;
the other five producer negatives and manual-density policy remain unchanged.
No DTO, backend admission, system scheduling or transport rewrite is required.

The corrected producer `c94da539` passed its mandatory local gate, exact hosted
run `41417270384` and independent official audit before public Main publication.
Only then did this candidate change the manifest revision and lock source;
accepted simulator Main remains on `13adde94`. Complete independent review of
the combined consumer source before one consolidated mandatory verification pass. Retain all fresh B03 fields,
ticks and `1e-12` bounds, existing native and real C++ Zenoh controls, strict
coverage and exact hosted qualification. Neither producer acceptance nor this
source migration completes B03 or equal-work performance acceptance.

### Historical cancelled adapter continuation (D33) — SDF-02 / PHY-03: adopt native joint semantic-pose evaluation

Root owns `work/producer-joint-pose-adoption`, preserving the failed constructor
adoption source and observations. Owned additions are the minimal producer
revision/lock update to accepted `f0d05146`, the Scene child-frame query selection,
and adoption/handoff documentation. The producer passed exact hosted run
`41438358157` and independent official audit before publication. Complete source
review, then one consolidated mandatory simulator gate. Preserve every fresh B03
field, tick and `1e-12` bound, existing guards and strict coverage. Real C++ B03
interop follows accepted motion; performance remains a separate unmatched scope.

The first joint-pose consumer gate (`ad718fbf`) passed strict B03 motion but failed
at a later serial lifecycle identity helper after 367 seconds. Root's
`work/serial-public-identities` correction owns only that helper and factual docs,
retaining the failed source/evidence and every lifecycle assertion. Independently
review the complete correction before one affected mandatory rerun; no separate
preliminary test or lint is required.

The subsequent `f4db5841` gate passed serial lifecycle but failed at the two
analogous B03 world-light inventory loops. Root's `work/b03-public-identities`
correction batches both loops with the same strict public-identity helper and
source review of other new tests for this class. All retained assertions remain;
finish review before one affected mandatory pass, without preliminary checks.

### IO-01 / PHY-03: Gazebo C++ B03 interoperability

Root owns the next source slice on accepted Main `8ebb02db` (exact hosted
`41447340434`, independent official audit passed). Owned files are server
`tests/native.rs`, `tests/clock_client.cc`, `docs/B03_NATIVE_INTEROP.md` and these
handoff/task updates. The external Gazebo C++ peer must receive actual six-phase
clock/local-pose snapshots through control/reset, reconstruct body world poses
with maintained native math and compare the accepted fresh reference at unchanged
bounds. Retain all existing peer modes, assertions, new file-backed diagnostics,
strict coverage and pinned SDK verification. Upstream Rust Zenoh is used directly
and unmodified; this is interoperability testing, not a C++ transport port.
Complete final source review before one consolidated local gate, then exact
hosted qualification and independent official audit before acceptance.

### REF-03: configured physics-only B03 comparison

Root owns `work/b03-physics-benchmark` on accepted `43cf8e69`. The owned slice is
one Rust Scene example, one public native physics actor/reference, one concrete
benchmark driver and its protocol controls, and benchmark/handoff/task docs.
It excludes Server/transport/IO and keeps earlier contextual actors unchanged.
Explicit unlimited per-pair contacts match the Rust plain-ODE selection without
changing product defaults. Bind source-established settings honestly; require
actual common DART/ODE libraries, full physical ownership and all 16200 values
across 100 frames at `1e-12` before any timing. Report original-default compatibility
separately; full-Server equal work remains open. Complete Source review before
one consolidated build/protocol/correctness/timing qualification and required
normal hook/hosted checks. These checks and independent official audit passed;
baseline and CI correction `e829be51` are accepted after exact hosted `41481267132`.

The configured local batch and independent data audit passed full100 correctness;
its measured window shows Rust 2.043246 times slower than native. Actual paired
observations/build/maps/timing proof is retained in the compact reference archive
and measurement summary. Required local, hosted and independent official
qualification is complete. No additional timing or unchanged successful
protocol/native build replay is required.

### Historical cancelled adapter optimization (D33) — PERF-01: per-pass articulated contact factorization

Root owns integration in `work/contact-factorization`; the physics owner supplied
`joint_state.hh`, `bridge.cc`, appended static-contact regression controls and
[DART_CONTACT_FACTORIZATION.md](DART_CONTACT_FACTORIZATION.md). The candidate
reuses validation and LLT factorization only within one immutable contact query
pass, keyed by skeleton. No persistent state, numerical threshold, response
expression or supported topology changes. Existing test assertions, fixtures,
baseline measurement summary and observation archive remain unchanged.

Production and regression source review is complete. Root updated the benchmark's
source-established bridge binding and documented the historical baseline boundary;
these changes passed independent source review. The first candidate gate failed on the historical native header binding before
replay compilation; three new regressions passed, later checks and final coverage
remain unrun. The correction preserves the exact original oracle and header in
an explicit output-only replay projection, with source/dependency rejection
controls. Baseline and CI correction `e829be51` are accepted after exact hosted
run `41481267132` and independent official audit, and that parent is integrated.
Complete independent correction review before the affected required gate.
Only after correctness passes may a newly built, source-bound actor enter one
reviewed equal-work comparison. No measured hotspot or speedup is claimed yet.

The REF-03 configured baseline is now accepted at `e829be51`: complete local
checks, exact hosted `41481267132` and independent official audit passed. It
retains 924 headless, 546 renderer and 249 tooling checks. The accepted immutable archive reports the slower Rust result without relabeling
failed snapshot attempts. Broader P0 and full-Server performance remain open.

### SDF-02 / LOAD-01: accepted primitive inertia consumer adoption

Root owns `work/sdf-capsule-inertia-adoption`, based on reviewed simulator `f136e821`.
Owned files are Cargo.toml/Cargo.lock (exact accepted producer pin only), the new
Scene primitive-inertia regression and selected native fixture, this task/handoff
entry and [SDF_PRIMITIVE_INERTIA_ADOPTION.md](SDF_PRIMITIVE_INERTIA_ADOPTION.md).
Adopt only producer `a44bf8be`: centered cylinders/capsules and its earlier bounded
posed box/sphere support. Do not implicitly adopt the later translation candidate.
Retain all runtime/backend admission and density semantics; check inherited native
mass/tensor and four-backend force/reset/removal behavior without changing old tests.
Complete independent source review before one required gate. Parent `f136e821`
is accepted; coordinate actual disk usage with the running renderer gate. No compiler, tests, lint or native capture has run.

This integration also publishes the independently reviewed contact-factorization
measurement appendix (`reference/contact_physics_measurement.json`, its separate
141-member observation archive and `docs/CONTACT_PHYSICS_MEASUREMENT.md`). The
measured simulator/producer identities remain `f136e821`/`f0d05146`, distinct from
the new consumer pin. Original baseline evidence stays immutable. Both source
batches are complete before the same required consumer integration gate; no new
or repeated timing batch is implied.

### RENDER-02: bounded programmatic metric depth (qualification correction)

The renderer owner has completed calibrated 64x64 box depth readback using Bevy's
actual stored depth texture and projection inverse. Root integrates accepted
consumer parent `c5dd02c0` and coordinator documentation; executable/test payloads
remain the reviewed renderer files. The first full gate's 549 renderer passes
include all three new depth tests, but regions 1009/1066 failed the strict
per-crate threshold. A reviewed append-only lifecycle/admission/completion-error
test batch preserves production code, old assertions and numerical bounds.
Complete this final source batch before one affected normal gate, exact hosted
qualification and independent official audit. No corrected result is assumed.
Near-plane intersections and full loaded/native depth sensor behavior remain open.
