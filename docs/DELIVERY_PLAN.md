## Current M115: complete the coverage regression batch

The second full hook on frozen `b52683da` exited 1 after 4,478.606 seconds.
Both Rust profiles had zero failed tests (2,195 headless and 1,694 renderer;
two ignored in each), and 498 Python tests passed. Server changed executable
line coverage was 195/208, below the required strictly-greater-than-95% gate.
No commit or capacity stop occurred. Keep this and the earlier three-match
compilation failure intact; test passes do not make the full hook a pass.

Work is isolated in `work/m115-lidar-rate-boundary-coverage` over `2419c923`.
Five new private boundary regressions address all ten reachable missed lines:
no-blueprint absence and restart, pure preflight refusal and repair, pending
original reservations, prepared-owner disappearance and fresh reconstruction,
and forged public metadata without a private sensor. Keep the three defensive
error arms that current getters cannot produce. No production behavior, old
assertions, coverage denominator or threshold changes are permitted.

Complete all regression Source, documentation and independent review, then one
affected formatter and one unchanged full mandatory retry. The 22 original
M115 declarations remain, plus five headless-only private declarations: 27
unique, 27 headless, 11 renderer, 11 shared, 10 public and 17 private. These new
controls have no executed result. Reuse only byte-identical formatting evidence.
See [the delivery contract](M115_DELIVERY.md).

Parent M114's exact hosted run 45412754337 attempt 1 passed. Its original
artifacts are retained; independent official audit remains a separate gate.
Root serializes local heavy work. Parent hosted/official plus own normal/local
DATA still gate review publication; every profile, all 20 coverage bases and
acceptance gates remain. Preserve Fuel, all frozen evidence and the live M77
no-expiry demo until explicit stop confirmation.

The checkpoints below are historical and retain their original observations.

---

## Current M114: organized CPU-lidar point clouds

M114 adds opt-in organized PointCloudPacked output to existing CPU lidars, using
one acquired sample for scan and cloud. Native five-field layout, sensor-local
coordinates, bounded payloads and separate cloud sequences preserve the existing
LaserScan behavior. See [M114_DELIVERY.md](M114_DELIVERY.md) for the closed scope,
regressions and native Source limitations. The budget is five working days,
including independent review and mandatory gates.

The isolated parent `02592f7a` passed its full normal hook, independent local DATA
and exact hosted CI 45297968661 attempt 1. Its original-artifact audit also passed.
M114 implementation and regression source are complete; its checks and acceptance have not occurred.
Finish code, regressions, guides and independent review before one consolidated
check cycle. No prior result transfers to this candidate. Accepted M1–M76 and
202 slices remain unchanged. Preserve the live no-expiry M77 demo; formal browser
and owner-confirmed shutdown evidence remain outstanding.

The following checkpoints are historical and retain their original evidence.

---

## Current M102–M113 renderer test correction batch

The frozen `7c8a5de4` full staged hook exited 1 after 4,440.450 seconds.
Both Clippy stages, documentation and the 498-test Python aggregate passed.
Headless completed 2,153 passed, zero failed and two ignored; its maintained
coverage gates passed, including Server 19,357/20,312 regions and 13,035/13,446
lines. Renderer completed 1,677 passed, four failed and two ignored. There was
no capacity stop or qualified commit. Those results belong to the frozen
candidate; this successor has no verification result.

The complete renderer failure batch has three reviewed test-premise or driver
classes. Preserve the original valid Capsule as a positive calibrated-depth
snapshot and lazy ownership control, with a same-target Plane refusal before
reservation. Advance every ready RGB camera once through ImagePublished before
receiving; preserve all clocks, deadlines and reset/removal controls. Assert the
established empty-scene RGBA [43, 44, 47, 255], including alpha, after complete
visual retirement. No production correction is established by these failures.

The isolated correction keeps all 122 Rust identities and harness mappings:
103 headless, 98 renderer, 79 shared, 70 public and 52 private; Python remains
498. The reviewed body/helper changes are explicit dispositions, not a claim
that every original body is unchanged. Complete the full failure batch and
independent source review before one affected formatter and one consolidated
full staged hook. Keep every workspace, crate and changed-base coverage gate,
independent local DATA, exact hosted CI and original-artifact audit. The genuine
qualified parent remains `08a0eeae`; no frozen failed candidate transfers proof.

Keep every previous failure and frozen tree. Accepted M1–M76 and 202 slices
remain unchanged. Leave the no-expiry M77 demo running until explicit owner
stop confirmation. Formal browser and shutdown evidence remain outstanding.
No demo action, cleanup, Fuel, backend or dependency change is part of this
correction. DART and MuJoCo remain retired.

---

The prior checkpoint below is preserved verbatim as historical context. Its
current-status wording describes its own observation time, not this wave.

## Current M102/M103 implementation

M102 adds translated explicit centers of mass to passive runtime primitive
compound copies. M103 admits representable rotated local LINK frames in the
existing two-Revolute JointController. Root selected disjoint lanes under D31;
each is bounded to five working days including review and mandatory checks.
Read the M102 and M103 delivery guides in `docs/` for the closed contracts.

The source parent is `08a0eeae97dede8daee6efe6cb2922bdb86ca722`, tree
`9e1f92e1586846bb86c253df1d519a928b949c8c`, true parent `1a4bdbb5`.
Its full normal passed after 4514.232 seconds and an independent retained audit
passed without replaying tests, builds or RAW merges. Actual headless results
were 2048 passed/two ignored; renderer 1583 passed/two ignored. All 23/22 new
headless/renderer controls passed; Python494, finite47 and auxiliary21 remain
separate aggregates. The original two-case failure and test-only corrections
remain preserved. The exact review ref is published; hosted outcome is unknown
at this checkpoint; exact automatic CI45074764720 attempt1 is in progress.

Both implementations, regression Source and retained same-input dispositions
are complete and independently reviewed. There are 26 new Rust declarations:
26 headless, 25 renderer, 25 shared; 20 public and six private (one native
headless-only and five Scene shared). No new Python, finite or CLI cases.
These are Source obligations, not current passes. The integrated Source has
1831 files, 16 owned paths (eight Rust/eight docs), four new paths,
1815 protected parent paths and 128 cumulative paths since accepted Main.
Only three production files change. Formatting, final tree/commit and
verification remain to be recorded. Finish code, regressions, documentation and
independent Source review; batch all findings before the one consolidated
final verification cycle. Keep the full staged hook, strict coverage, hosted
CI and independent original-artifact audit before release, Main promotion or acceptance.
No qualification or acceptance transfers from the parent.

M102 owns native `compound.rs` and Scene `world_spawn.rs`; M103 owns only
JointController `config.rs`. Their new regression leaves are disjoint. Root
owns retained same-input refusal-to-positive dispositions and shared docs.
Any additional production seam requires explicit concrete rescope before edits.

The persistent no-expiry M77 manual demo stays live until the owner confirms a
stop. Formal browser evidence and shutdown still gate acceptance. Accepted
M1–M76, 202 slices, Fuel, old targets, frozen worktrees and evidence stay intact.
DART and MuJoCo remain retired. No cleanup or new heavy resource allocation is
released by this document; the next final run requires a fresh reviewed budget.

---

The exact parent checkpoint follows as historical context. Its status statements
describe that earlier checkpoint, not the current pair.

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


## Historical M80 first-Source checkpoint

[M80](M80_DELIVERY.md) admits one translated/rotated cylinder collision per Rapier
link, preserving link-centered inertia and link/visual/sensor frames. Physics/Rapier
and Scene regression Source are integrated for independent whole Source review;
Root owns integration and canonical records. The budget is at most five working days including review
and mandatory gates. TPE posed geometry, compounds, offset COM, other new shapes,
Server/viewer/package/dependency changes and DART/MuJoCo remain excluded.

The development basis is locally qualified M78/M79 `8ad64532`, now published on
`review/m78-m79-posed-box-velocity-control`. Its full normal/strict hook and
independent DATA passed: 1,812 headless and 1,390 overlapping renderer passes,
494 aggregate Python checks, 47 finite examples and 21 CLI cases. Both profiles
retain two ignored controls; these are not passes. Exact hosted run 44806268854
attempt 1 is in progress at M80 selection; no hosted or official success is
inherited. All earlier product and reader failures remain retained.

M80 implementation, regression Source, docs and independent whole Source review
precede one consolidated verification pass. No per-edit tests, lint, formatting
or native trials. M1–M76 and 202 slices stay accepted; catalog grows to 80.
Accepted Main remains `dddf1f42`; Latest M70 keeps its original 41 examples.
M77 formal browser access and owner-approved shutdown still hold predecessor
acceptance. Keep its no-expiry demo running until explicit stop confirmation;
M80 does not stop it, modify frozen qualification trees or touch Fuel.

## Historical checkpoint: shared M78/M79 Source preparation

[M78](M78_DELIVERY.md) posed Rapier boxes and [M79](M79_DELIVERY.md)
VelocityControl share one D31 candidate over qualified but unaccepted M77
`57b44de1`; accepted Main remains M76 `dddf1f42`. The standalone M78 full hook
failed at its unchanged strict-headless timeout with Root/Git 1 and no commit.
The cause remains UNKNOWN; its frozen tree and all partial evidence are retained.

The merged Source obligations are 39 Rust declarations (39 headless, 21 also
renderer), with zero new Python declarations. They are not passes. Complete
merged Source review, fresh native component evidence/DATA and final adoption
before one consolidated normal/strict/hosted/official cycle for both scopes.
Neither phase has a standalone qualified successor or inherited acceptance;
acceptance/Main still require M77's accepted-predecessor closeout and all gates.

Accepted phases/slices stay 76/202, catalog 79, Source finite/CLI 47/21 and
Latest M70's original package 41 examples. M77's manual demo stays running
without expiry until owner confirms stop; it is not formal GUI acceptance.
Preserve frozen trees, all history and Fuel work.

The M77 sections below are historical Source checkpoints from the preparation
parent; current scope, leases and gates are in the linked contract.

## Current development: M77 installed camera preview

The selected [M77 contract](M77_DELIVERY.md) retains its 47-world/four-role scope.

## M77 correction checkpoint: camera startup and input admission

This isolated candidate starts at frozen `353dc9ae86aac56c7653f36a933d688b43fa6fbf`.
That Source passed local normal/DATA, the fresh four-role build, package DATA and
clean installation/47 finite loads. The first installed camera run then failed
once (`8d2438` to `e5ef74`, 28.31 seconds), before any SDK peer, in
`ubuntu-install-um5894io`: the default web snapshot guard required all-visible
flags while the unchanged M76 fixture has opaque visuals with flags 4/1/2.
The selected product correction admits nonzero flags under the viewer's implicit
all-bits mask; zero visibility, transparency, unsupported materials and all
existing snapshot bounds remain refused. Camera masking, protocols, viewer/GPU
bytes and solvers do not change.

The candidate retains the independently Source-reviewed wrench-test preconditions:
finite Block configuration, fresh per-key presence generations, the original
five-second deadlines and bounded polling, with every real put and assertion.

The reviewed `95ef6ced` correction then ran one full normal attempt and failed
Root/Git 1 (`29733` to `d26f30`, 1367.114675317 seconds). All available headless
targets finished with one failure: the wrench presence query unwrapped
`Err(Disconnected)` at line 259. Renderer checks were not reached. The frozen
Source, full raw packet `95ef6ced...-92rp2mvd`, receipt and failure record remain
under primary `artifacts/m77-delivery`; no success audit or qualification follows
from that run. This failure is a demonstrated harness EOF-contract error, distinct
from the still UNKNOWN cause of original `353dc9ae` hosted attempt 1. The unchanged
original-head hosted attempt 2 succeeded, without accepting that old package or
qualifying this changed successor.

The current successor retains the exact reviewed web visibility correction and
all original semantic puts/assertions. Pinned Zenoh finite-query completion drops
the FIFO sender: drained/disconnected EOF is accepted only with a freshly open
observer. `Ok(None)` remains absolute receive-deadline failure. Only the exact
Source-defined `Timeout` payload with `Encoding::ZENOH_STRING` marks a 100 ms
query incomplete; all partial tokens from every key in that attempt are discarded.
Metadata queries may repeat only inside the same original five-second readiness
deadline. Unexpected errors, encodings, duplicate timeout or post-timeout samples
fail. No real command is replayed and no deadline is extended.

Four new no-network FIFO/error regressions bring the correction census to nine
Rust declarations (headless 9, renderer 2, overlap 2), zero new Python declarations,
with the original M77's 13 Python cases retained. Finite/CLI inventories stay
47/21. Current implementation/regression/docs require whole independent Source
review; no current formatter, checks, package, native operation or publication has
run. Its future qualified commit/tree/run remain unknown. Accepted M76 and public
Latest M70 are unchanged. Old passes and failures remain bound to their own Source.


## Historical checkpoint: M75 per-camera visibility masks

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

## Current development: M70 WebSocket close handshake

M1–M69 and 202 slices remain accepted; D33 retirement is published at Main
`8f13dc7446be4fded424b1eba83df4c8253d0cef`. Keep Rapier f64 and TPE;
DART and MuJoCo remain retired. M65 remains Latest with 35 examples.

Frozen camera-connectivity candidate `2e8d9dc22ae267a7a108a76716b8aaefcabbddf2`
passed its full local normal/strict gates, independent audits, fresh package and
clean Ubuntu installation with all 41 finite examples. Installed camera evidence
passed the native cadence points and actual worker retirement, then failed the
Scene witness's client-initiated WebSocket close: actual code 1006 and
`wasClean=false`, after FINISH and EOF completed. These are provisional partial
proofs; reset, demand, RGB, full shutdown and browser acceptance remain pending.

This isolated successor fixes the concrete close path: the CLI currently drops
a received Close before flushing tungstenite's queued reply. Attempt one flush
under the existing socket write timeout, then retire the peer. Add a real-socket
regression requiring the echoed Close frame before EOF, unchanged CPU/world state
and a fresh reconnect. Preserve malformed-input refusals and all original tests.
No wire-format, transport, renderer, physics, dependency or timeout change is selected.

Finish implementation, regressions, guides and independent Source review before
one consolidated full normal/strict pass. No check has run on this successor.
Then derive fresh source-bound release/package/installation/runtime evidence using
the maintained procedures; reuse unchanged evidence only where their bindings
permit. Keep frozen candidates and every real failure intact.

Hosted qualification remains blocked: the older `597ac3dd` run failed before
product checks on three locked Ubuntu snapshot HTTP 401 responses. Its cause and
recovery remain unknown; current Ubuntu APT success is not snapshot recovery.
No additional CI retry or snapshot probe is released. Live browser access to
`http://127.0.0.1:8082` still requires the explicit owner approval requested by
automatic review. Exact hosted/official qualification and release readbacks remain
mandatory before M70 acceptance or publication.

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

## Current M57: two runtime-test premises under correction

The second complete normal retired with actual Root exit 1 after
1650.1202665108722 seconds. Strict Clippy, 403 aggregate Python tests and the
maintained native-free/RGB stage passed. Keep-going runtime execution found two
failed targets: Scene 276 passed/1 failed and Server 443 passed/1 failed. Full
normal/strict coverage and independent local DATA have not passed for M57.

The correction batch keeps production unchanged. Scene loads the supported
unbounded, zero-damping graph, checks captured finite metadata separately and
explicitly refuses unsupported limits/dynamics. Server distinguishes a live
binding fault from full entity disappearance and respects publication cadence;
terminal/cache/reset and selected/unselected endpoint retirement controls remain.
Complete independent Source review before one affected formatter and full retry.
The original failed trees, logs and partial passes retain their exact scope.

The earlier missing Name import is corrected and its affected formatter passed.
All four scoped native evidence stages remain actual Root 0; no capture, fixture
or old RAW replay is authorized. M56 remains locally qualified and unaccepted.
M1–M42/175 accepted slices and M40 Latest are unchanged. Hosted snapshot recovery,
accepted predecessors, official audits and ordinary Main/readbacks still gate
publication and acceptance. No new CI retry or workaround is released.

Read [the M57 contract](M57_DELIVERY.md) and retained evidence under
`artifacts/m57-delivery`.

Earlier checkpoints below retain their historical scope.


## Active M56: complete bounded correction batch

M56 implementation and observed-data consumers are source-reviewed on locally
qualified, unaccepted M55 `fcfa7176`. Fresh original-object capture, independent
native audit, typed fixture export and independent literal validation passed.
The rejected original DSO and both failed capture attempts remain preserved.

The first full normal returned actual1 after196.919272 seconds during strict
Clippy. Its complete keep-going output contains two test-code findings: a complex
boundary tuple return type and a collapsible nested if. Rust tests and coverage
were not reached. The failed tree831ccb, packet, logs and actual failure remain
retained and unqualified. The bounded corrections name the identical tuple and
use the same short-circuit pause guard; all assertions, values and test IDs
remain intact. Complete independent Source review and one affected formatter
before one full mandatory normal retry. Do not rerun the successful reference or
literal pipeline, relax lint rules or weaken coverage.

M1–M42 and 175 slices remain accepted; M40 remains the installed Latest.
M43–M55 are locally qualified and await dependency-ordered hosted qualification,
official audits and Main publication. Ubuntu snapshot recovery is unconfirmed;
no CI retry, provider rebuild or acceptance transfer is authorized.
See [the M56 delivery contract](M56_DELIVERY.md) for scope and remaining evidence.

Earlier checkpoints below retain their historical scope.

## Current M55: endpoint-bound output restoration

The complete four-case correction batch was Source-reviewed and formatted once.
Before normal3 was invoked, review found a separate concrete prepared-output swap
gap: distinct output handles could retain valid tuple identity while pointing to
each other's topics. The bounded correction stores the immutable setup endpoint
on the CPU output and checks it before reuse/IO, with distinct-swap and duplicate
refusals under the existing control ID. IO threading, Weak lifetime, queues,
packets and sampling remain unchanged. Five corrective Rust paths now include
this narrow IO metadata leaf. Review the complete Source and updated documents,
then one affected formatting pass/full normal; prior format/failures remain intact.
No heavy job or normal3 is active. M56 isolated Source preparation continues.

Earlier checkpoints below retain their original historical scope.

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

# Current development: M46 rotated odometry frames

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

M1–M42 remain accepted at Main `73e03b9f`; 46 phases are selected and M40
remains the installed Latest. M43–M45 are unaccepted. M43/M44 hosted run
43024343885 failed locked Ubuntu snapshot setup before product checks; recovery
is unverified and no retry or probe is released. Their completed local runtime,
package, installation and browser evidence remains retained under its actual
source/build lineage.

M45 clean local commit `e66c21e739089670b24e23b91fcd1363935b9a39` passed the
full normal hook and independent local audit: 1,717 headless and 1,022 renderer
passes with overlap, 401 aggregate Python tests, and all strict coverage gates.
This is provisional evidence; exact hosted/official qualification, accepted
predecessor and Main/readbacks remain required before publication/acceptance.

[M46](M46_DELIVERY.md) selects finite local/global rotational odometry offsets,
combined with existing translations, in both dimensions. Source development uses
that exact locally qualified M45 commit in a new isolated worktree. Independent
review, meaningful native and real-motion/canonical regressions, one consolidated
normal pass, strict coverage and every release gate remain mandatory. M46
tests have not run. The first Clippy failure and reference audits are recorded above. No package/browser
refresh is selected.

The earlier checkpoints below retain their historical scope and evidence.

# Current provisional local development: M45 CUSTOM world-relative IMU angles

M43/M44's original runtime/package proofs remain frozen at9a40768d/tree5872e52f.
Their containment repair a08f0b50/tree5c22 passed the complete normal hook and
independent local DATA over that Source parent. Accepted Main remains 73e03b9f.
The fresh hosted run 43024343885 failed signed snapshot TLS/select setup before
product checks; the previous9a run 42845720294 was cancelled. Both failures and
all original local/native/package/install/feedback/browser/visual proofs remain
retained in the primary artifacts/m43-m44-delivery. No retry/probe is released.
The reviewed one-package lineage amendment 0f2eea1d/adoption78d5cd56 retains exact
9a build/manifest/proof identities while separately qualifyinga08. No rebuilt
ELF/graph equality or CI credit is inferred. Hosted/official and all public
Main/tag/release/Latest readbacks still precede acceptance; M40 remains Latest.

M45 is integrated for PROVISIONAL local development over exact locally qualified
Source parent a08, under reviewed timing amendment 5d823a75/adoptionab9586b5. Its
original Source-authorship basis 9a and standalone component capture parent73e
remain separate. The component capture and native/literal DATA passed for 14 new
configurations and 84 observations; exact audited fixture copies and both new private
consumer leaves are now prepared. No M45 Rust pass is claimed.
Read [M45 contract](M45_DELIVERY.md). Complete Source/integration/tree/runner review
must precede one ordinary full LOCAL normal hook and independent retained DATA.
Strict coverage, all jobs, actual exits and resource floors are unchanged. Actual
accepted M43/M44, exact hosted/official and Main/readbacks still precede M45 hosted
publication and acceptance. If the eventual parent changes, local evidence does
not transfer automatically. No package/browser refresh or old replay is selected.
The five-working-day scope budget and all original exclusions remain.

The inherited M43/M44 closeout and older checkpoints below remain historical.

# Current delivery: M43 and M44

M1–M42 are accepted at qualified Main `73e03b9f741689b1538898bde96bf975a49f068e`.
M41 constant-wind AirSpeed and M42 joint-force feedback passed the complete
normal hook, strict coverage, exact hosted CI 42808865907 attempt 1 and
independent local/official DATA, followed by normal Main push and readback.
M40 release 440119258 (`m40-preview`) remains the latest installed preview.

M43 adds named IMU orientation references; M44 refreshes the preview with
eighteen examples and the installed joint-command helper. Source work proceeds
in parallel with explicit leases; Root owns shared integration and serial heavy
operations. Neither phase is accepted. Complete implementation, regression
source, guides and independent review before consolidated checks. Fresh native,
Rust, distribution, clean installation and browser proofs remain required. Each
phase is bounded at five working days including review and mandatory checks.

Read [M41/M42 acceptance](M41_M42_ACCEPTANCE.md) and the [M43/M44 contract](M43_M44_DELIVERY.md).

## Historical planning and closeout checkpoints (superseded)

# Current closeout and next source preparation

M1–M38 remain accepted; 42 phases are selected including the two next source-only lanes. M39/M40 complete normal, independent local DATA, exact review CI42778327635 and fresh package/install/browser DATA have passed. Independent official DATA and Main/tag/release/Latest readbacks remain pending; neither M39/M40 nor M41/M42 is accepted. M37 remains installed Latest.

The inherited earlier current-state blocks below are historical checkpoints; their outstanding gates remain requirements until actual completion. The new M41/M42 contract and source-only boundary above them govern this worktree.

## Next source preparation: M41 and M42

[The reviewed-before-code contract](M41_M42_DELIVERY.md) selects constant-wind
AirSpeed and a joint-force/JointState feedback example as disjoint source lanes.
M39/M40 closeout remains the priority; neither next phase inherits acceptance.
Each is at most five working days including review and all required gates.
Actual accepted-parent binding precedes next-wave native or consolidated checks.

<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

## Current delivery: M39 translation offsets and M40 preview

M1–M38 are accepted at qualified Main `00382fbc`; [M38 acceptance](M38_ACCEPTANCE.md)
records actual normal, exact hosted CI, independent DATA and Main readback.
M37 release 439888006 remains installed Latest. Forty phases are selected and
38 are accepted.

[Atomic M39/M40](M39_M40_DELIVERY.md) adds finite local/global XYZ translations
to existing noiseless odometry in both dimensional modes, and refreshes the
native-free Ubuntu preview with fifteen finite installed examples. Feature,
canonical/native and packaging Source lanes are independently reviewed and
integrated. Fresh scoped component capture and native/literal DATA passed;
new Rust, normal/strict/exact hosted and distribution qualification remain
required. Budget: at most five working days per phase including review and
mandatory gates. Root alone releases serial heavy checks. Neither phase is
accepted until every feature, package, install, browser/shutdown and publication
criterion passes. Continue the next useful bounded increment after acceptance.

The following earlier current-state statements are historical and superseded.

<!-- Copyright (C) 2026 Open Source Robotics Foundation -->

## Historical M36/M37 qualification checkpoint (superseded)

M1–M35 are accepted at qualified Main `1982ce86`; M32 remains installed Latest.
M34/M35's full normal, exact hosted CI42715967423 attempt1, independent
native/local/official DATA and exact Main promotion are recorded in
[M34_M35_ACCEPTANCE.md](M34_M35_ACCEPTANCE.md).

[M36/M37 are one atomic delivery](M36_M37_DELIVERY.md) over that accepted parent.
Both complete functional/package source batches, meaningful regressions and
guides have independent Source review. A new combined candidate must pass the
full normal hook, strict workspace/per-crate/actual-parent coverage, exact hosted
CI and independent local/official DATA. Fresh source-bound release inputs,
notices/ELF/archive/relocation DATA, clean Ubuntu nonroot installation with all
thirteen examples, actual default-robot browser controls and owned shutdown,
and Main/tag/assets/body/Latest readbacks remain required before either phase
is accepted. No M36 or M37 acceptance is claimed.

This supersedes only the intermediate M36-acceptance-before-M37 sequence. The
already-running M36-only normal gate may finish and is retained as provisional
historical evidence; it cannot qualify the combined candidate. Every individual
M36 canonical/physics-feedback/lifecycle criterion and M37 distribution criterion
is preserved. Root releases serial heavy operations after complete Source and
procedure review. No old gate, test, installation or browser success transfers.

<!-- SPDX-License-Identifier: Apache-2.0 -->


# Weekly delivery plan

## Historical M33 pending checkpoint (superseded)

Prior qualification: [M33 mobile-robot IMU/lidar coexistence](M33_DELIVERY.md),
locally qualified at `17dd62f2` with exact hosted/official/Main acceptance pending.
M1–M32 remain accepted, and M32 stays installed Latest.

[Selected M34/M35 contracts and leases](M34_M35_DELIVERY.md) open disjoint source
preparation during the remote qualification wait. Each scope is at most five
working days including review and mandatory checks. Detailed native/lifecycle
and typed-interface review precedes code release; accepted-parent binding
precedes native execution or full consolidated verification. No phase inherits
acceptance from another or changes M32's package.

> M1–M32 are accepted at qualified Main `679c9990`; see
> [M32 acceptance](M32_ACCEPTANCE.md) and the verified Latest
> [m32-preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m32-preview).
> Current work is [M33 mobile-robot IMU/lidar coexistence](M33_DELIVERY.md).
> New source, functional proofs and mandatory gates remain required.

## Retained delivery policy

Owner decision D30 (2026-11-02) replaces the previous full-rewrite-first sequence.
Ship a useful MVP first, then small runnable releases. The initial planning
update left implementation paused; the owner subsequently explicitly resumed
M1 implementation and its continuation automation.

## Schedule and acceptance

M1–M6 were accepted on 2026-11-03, M7–M9 on 2026-11-04 and M10–M11 on
2026-11-05 UTC; see [M1 evidence](M1_ACCEPTANCE.md),
[M2 evidence](M2_ACCEPTANCE.md), [M3 evidence](M3_ACCEPTANCE.md),
[M4 evidence](M4_ACCEPTANCE.md), [M5 evidence](M5_ACCEPTANCE.md) and
[M6 evidence](M6_ACCEPTANCE.md), [M7 evidence](M7_ACCEPTANCE.md) and
[M8 evidence](M8_ACCEPTANCE.md), [M9 evidence](M9_ACCEPTANCE.md) and
[M10 evidence](M10_ACCEPTANCE.md) and [M11 evidence](M11_ACCEPTANCE.md). M12 was accepted on 2026-11-07 UTC after final qualification, clean installation and actual preview publication; see [M12 evidence](M12_ACCEPTANCE.md). Its original **at most five working days** scope/budget is retained. Each later named increment is
also at most five working days, including source review, fixes, documentation,
mandatory staged checks and hosted CI. These are scope budgets and planning
targets, not guaranteed completion dates. No clock runs while paused. Re-estimate after a real integration checkpoint; report any risk immediately.
If work exceeds a budget, split unfinished optional scope into a new named
phase. Never label a broken mandatory exit criterion accepted to meet a date.
Do not carry a broad family indefinitely as one weekly phase.

Every phase ships a runnable increment of the previously accepted subset. Keep
retained assertions, numerical tolerances, per-crate/workspace >95% line and
region coverage, independent review, staged hooks and required CI. Complete the
bounded feature and source review first, then run one consolidated verification
pass. Reuse unchanged evidence where current checks permit it. New C++ parity
work is required when a changed retained contract needs it, not a new campaign
to complete every backend before a primitive MVP can ship. Rapier-versus-DART
results must never be described as same-engine performance improvement.

## Delivery sequence

| Phase | Budget | Runnable result | Acceptance boundary |
| --- | --- | --- | --- |
| M1: Runnable primitive MVP | 5 working days | One local SDF world, Rapier f64 box/sphere dynamics and static box ground; live gzweb-rs scene and poses; pause, resume and reset. | On a clean supported Linux setup, documented launch commands start the Rapier-only runtime and gzweb-rs. The real browser shows at least 20 moving primitive bodies and correct transforms; pause/resume/reset, reconnect and clean shutdown work. No C++ engine or GPU is required by the server. |
| M2: Local mesh visualization | 3 working days | Render local glTF 2.0 mesh visuals in gzweb-rs; keep primitive collisions. | A moving body with a local mesh visual has correct scale, orientation and resource paths in the actual browser; missing or invalid assets fail clearly. |
| M3: Mesh collisions | 5 working days | Rapier static triangle-mesh and dynamic convex-hull collision from the admitted local mesh input. | Static mesh contact and one dynamic convex hull pass geometry/contact and reset tests; unsupported nonconvex dynamic input is explicitly rejected. |
| M4: IMU sensor model | 3 working days | Wire the existing IMU model through loaded SDF demand and the live Rapier world; expose its canonical messages. | One configured IMU publishes correct units, frame, sample count and timestamps; pause/reset and last-consumer teardown pass. A no-IMU world owns no IMU callbacks or buffers. |
| M5: RGB camera model | 5 working days | Activate one SDF RGB camera using existing offscreen Bevy rendering and bounded image delivery. | A known primitive scene produces real timestamped images at the requested rate; frame, pause/reset, removal and overload controls pass. |
| M6: Depth camera model | 5 working days | Connect the existing metric-depth renderer to one loaded SDF camera and a bounded consumer stream. | A calibrated primitive scene yields metric depths and documented background/clipping behavior; reset epochs, removal and bounded delivery pass. |
| M7: CPU ray lidar model | 5 working days | One planar ray lidar using Rapier queries against admitted primitive and mesh collisions. | Analytic range fixtures, min/max range, frame, acquisition timestamps and pause/reset pass; visualization can inspect the scan. |
| M8: Differential-drive system | 5 working days | One two-wheel Rapier robot, velocity command and odometry, visible in gzweb-rs. | A simple robot drives and turns from real commands; wheel limits, odometry frames, pause/reset and absent-system teardown pass. |
| M9: Joint controller systems | 5 working days | One admitted Rapier revolute chain with position and velocity controllers. | Commands move the displayed joints with documented limits and stable IDs; bad commands, reset and removal pass. |
| M10: Interactive world lifecycle | 5 working days | Expose focused spawn/remove and entity-state operations to the live browser for the admitted MVP scene. | Spawn and remove primitives and one local mesh without stale IDs or visuals; rejected requests are atomic; reconnect obtains a current snapshot. |
| M11: Focused recording and replay | 5 working days | Record and replay the admitted scene, poses and controls for one local session. | A short recorded session reproduces its timestamps and entity lifecycle in gzweb-rs; malformed input and bounded buffering pass. |
| M12: Installable integrated preview | 5 working days | Package the accepted subset and document supported capabilities, setup and a runnable robot demo. | A clean-machine installation runs the integrated demo; required coverage, CI, licenses and retained regressions pass at the release commit, with a published limitations matrix. |
| M13: Ideal altimeter | 5 working days | One loaded noise-free Rapier altimeter with canonical publication. | Native/wire/offset-point velocity, cadence, absence, pause/reset/removal and stale-generation controls; all gates. |
| M14: Local STL meshes | 5 working days | Local ASCII/binary STL collision and immutable GLB visuals using the accepted viewer. | Bounded admission, geometry/scale/contact/lifecycle controls and actual browser evidence; all gates. |
| M15: Ideal air pressure | 5 working days | One loaded ideal Rapier pressure sensor with canonical FluidPressure publication. | Actual native geopotential model, absolute sensor height, wire/frame/time/cadence/demand/reset/removal and atomic-refusal controls; all gates. |
| M16: Local OBJ meshes | 5 working days | Bounded local triangle OBJ collision and immutable GLB visuals using the accepted viewer. | Maintained parser, bounded/index/resource admission, geometry/contact/lifecycle/immutable serving and actual browser proof; all gates. |
| M17: Ideal magnetometer | 5 working days | One loaded ideal Rapier magnetometer with a finite constant world field and canonical Tesla publication. | Native/model and resolved-pose vectors, frame/time/sequence/cadence/demand/pause/reset/removal, atomic refusals and canonical wire; all gates. |
| M18: Joint-state publication | 5 working days | Actual single-axis Rapier joint position/velocity and supported metadata in canonical Model packets. | Native packets and backend components, topic/filter/order, equal-time/pause/reset cadence, lifecycle/endpoint and moving-joint wire controls; all gates. |
| M19: Refreshed installed preview | 5 working days | Ubuntu 24.04 native-free archive through M18, copied local OBJ robot, unchanged viewer and a nonconflicting explicit loopback HTTP port. | Fresh release binaries and notices, actual manifest/archive/relocation, clean non-root supported installation and real installed robot/browser controls and shutdown; all gates. |
| M20: Explicit installed lifecycle and publication credit | 5 working days | Explicit existing lifecycle/credit opt-in with the complete M19 robot and unchanged defaults/viewer/wire. | Real socket full-world credit/catalog controls, guides, all mandatory gates and actual supported installed-browser controls/shutdown; see M20_DELIVERY.md. |
| M21: Loaded altimeter Gaussian noise | 5 working days | Existing single Rapier altimeter with seeded Gaussian/quantized position and velocity channels. | Native zero-elapsed noise application, noise-free reference, deterministic streams, demand/rate/pause/reset/fault and actual wire controls; focused native evidence and all gates. |
| M22: Loaded air-pressure Gaussian noise | 5 working days | Existing single Rapier pressure sensor with seeded Gaussian/quantized output and native double variance. | Native zero-elapsed application and sqrt(stddev) variance, positive-output/invalid suppression, deterministic streams, demand/rate/reset and actual wire controls; focused native evidence and all gates. |
| M23: Loaded magnetometer Gaussian noise | 5 working days | Existing single Rapier magnetometer with seeded sensor-axis X/Y/Z Gaussian/quantized noise. | Native post-rotation XYZ/zero-dt, demand/reset/fault/wire controls and all gates; see M23_M24_DELIVERY.md. |
| M24: Planar CPU-lidar white Gaussian noise | 5 working days | Existing single planar Rapier lidar with zero-minimum finite-hit mean/stddev noise. | Native activation, beam-order/skipped misses/post-noise clipping, stale-scan fault retirement, unchanged ideal behavior and all gates; see M23_M24_DELIVERY.md. |
| M25: Updated installed preview through M24 | 5 working days | Fresh native-free Ubuntu archive with the four parent noise examples and dedicated guides, preserving the default robot and viewer. | All gates, fresh binaries/notices/archive, clean non-root installation with four actual example loads, new installed browser controls/shutdown and actual publication readbacks; see M25_DELIVERY.md. |
| M26: Ideal loaded AirSpeed | 5 working days | One top-level Rapier sensor, zero wind/default atmosphere/no noise, sensor-point and frame-correct native numeric pressure and temperature. | Actual model/native/wire/demand/cadence/reset/removal/atomic refusal proofs and every mandatory gate; see M26_M27_DELIVERY.md. |
| M27: Flat-model PosePublisher | 5 working days | One built-in vector publisher for selected model/link parent-relative poses, exact headers and native unpaused cadence. | Native keyed frame/stamp records, real wire and lifecycle/atomic-collision controls and every mandatory gate; see M26_M27_DELIVERY.md. |
| M28: Loaded AirSpeed pressure noise | 5 working days | Existing AirSpeed with Gaussian/quantized noise on native pre-100 float pressure. | Numeric/seed/demand/reset/fault/wire controls, focused native evidence and every mandatory gate; see M28_M29_DELIVERY.md. |
| M29: Updated installed preview through M28 | 5 working days | Fresh native-free Ubuntu preview with all seven finite examples and unchanged robot/viewer. | Full gates, fresh package/closure/relocation, clean non-root install, seven actual example runs, new installed browser/shutdown and actual release readbacks; see M28_M29_DELIVERY.md. |
| M30: Ideal CPU LogicalCamera | 5 working days | One loaded Rapier origin-frustum camera with canonical image and frustum streams. | Native inclusive point predicate/full relative poses, both-stream demand/wire/lifecycle/refusal proofs and all gates; see M30_M31_DELIVERY.md. |
| M31: Static-vector PosePublisher | 5 working days | Existing flat-model publisher with native dynamic/static channels and independent cadence. | Native classification/refreshed poses/rates, both-topic reservations and real wire/reset/retirement proofs plus all gates; see M30_M31_DELIVERY.md. |

M1–M29 are accepted. [M28/M29 acceptance](M28_M29_ACCEPTANCE.md) records Main
`ab9660ee`, exact CI42586254208 attempt 1, all independent DATA and the fresh
seven-example installation, actual browser/shutdown and verified publication.
The M29 preview is checked Latest; M25 and every older release remain preserved.
There are 31 selected phases and 29 accepted. M30/M31 now continue under
[their explicit contracts and disjoint source leases](M30_M31_DELIVERY.md).
No new check, native proof or source-feature acceptance is inherited; local
heavy jobs remain serial and every required gate remains unchanged.

### Historical atomic M21–M25 integration checkpoint

The following original pending/Latest statements are superseded by the actual
M25 acceptance above; the old failures and capture identities remain protected.

M1–M20 are accepted and published; [M20 acceptance](M20_ACCEPTANCE.md) remains
the checked installed preview. [M21_M25_DELIVERY.md](M21_M25_DELIVERY.md) now
combines the complete reviewed M21–M25 source into one new candidate over M20,
avoiding three serial qualification cycles. Both frozen M21/M22 hosted attempts
failed at the same LLVM Scene-link SIGBUS of unknown cause; no third unchanged
retry is authorized. Old local/provisional passes and native captures remain
separate, correctly bound evidence.

The atomic candidate must pass new full normal/strict actual-M20-parent coverage,
exact hosted CI and independent Source/local/official DATA. All separate sensor
contracts and fresh M25 binaries/notices/archive/clean installation/four actual
installed examples/new browser/shutdown/publication proof remain required before
accepting M21–M25 together. The CI correction aligns debug information with the
passing local configuration and adds bounded diagnostics; it does not establish
the cause or weaken checks. All original starts/budgets and accepted history are
retained. There are still 25 selected phases and 20 accepted; M20 remains Latest.

D31 permits independent bounded increments to run in parallel with explicit file
ownership, one integrator and one complete release candidate. A dependency may be reused from existing
accepted code, but no phase inherits acceptance merely from an older test run.
Existing broad P0–P7 gates remain historical and long-term coverage inventories;
they are not this release schedule and do not block M1 as entire families.
The following M15–M19 preparation paragraphs are historical checkpoints; current
M25 acceptance and M26/M27 execution policy are recorded above.

`planning/delivery-phases.json` is the machine-readable schedule mirrored by the
tracker. M1–M16 are accepted under their bounded criteria, independently of total
rewrite effort. M15/M16 Main is `5f53c92e`; its public `m15-m16-features` receipt
records actual qualification. M12's installer remains unchanged. Current M17 ideal
magnetometer reuses accepted sensor infrastructure; M18 joint-state publication
reuses admitted M9 joints. See [M17_M18_DELIVERY.md](M17_M18_DELIVERY.md) for exact
contracts and parallel ownership. Neither inherits acceptance from an older run.

[M19_DELIVERY.md](M19_DELIVERY.md) selects the next useful bounded delivery:
refresh the installed preview, which otherwise remains the older M12 payload.
Its source, regressions and guides may proceed separately while the exact M17/M18
parent qualifies. After that parent passes its local normal hook, native proof
and independent local audits, reviewed M19 local checks may overlap parent
hosted CI. Local heavy work stays serial. Acceptance and publication still require
the finally qualified parent, every exact M19 gate and its own installed artifact
proof. A parent source correction requires rebasing and affected required checks.

Owner decision D32 directs automatic continuation: after accepting a phase,
select and start the next useful bounded delivery while retained work remains.
Do not stop merely because a successor was unselected. Each scope still needs
its dependencies, fixtures, unsupported cases and exit criteria recorded before
implementation, and every acceptance gate remains mandatory. Stop for an explicit
owner pause, completion of retained scope or a genuine blocking owner decision.



## M1: smallest real end-to-end product

Deliver one example world with a static box floor and at least 20 dynamic boxes
and spheres. Reuse the current Rapier f64 adapter, scene loading, IDs, clock and
upstream Rust Zenoh integration. Add a selectable Rapier-only runtime build/run
path that does not require native DART/MuJoCo libraries or the C++ reference SDK
on the user's machine. Preserve the existing backend implementations and gates.
The server remains usable without a window or GPU. The browser requires an available supported graphics backend; prefer WebGPU
and provide the maintained viewer's WebGL2 path where supported. Test an actual
browser; a successful WASM build alone is not visualization evidence.

The maintained viewer is `gazebosim-rs/gzweb-rs`, checked out alongside this
repository at `../gzweb-rs` (baseline `3c3ca32`, Bevy 0.20). It already has live
Gazebo WebSocket networking, scene rendering and controls. The initially
inspected `/home/nkoenig/work/gazebo-web/gzweb-rs` is an older untracked Bevy 0.18
prototype; its isolated work and probe evidence are preserved, but it is not the
M1 delivery target. Reuse the maintained viewer's `SimBackend`, scene and UI
interfaces. Add an explicit primitive-backend selection without changing its
default Gazebo protocol. Viewer and simulator remain separate processes.

Use the narrow versioned scene snapshot, committed clock and pause/resume/reset
contract in MVP_BROWSER.md for this bounded local bridge. Simulator networking
continues to use unmodified upstream Zenoh. Do not create a new transport
repository or general protocol framework. Preserve D28's selected Gazebo Zenoh
compatibility at the simulator boundary. This browser bridge is not proof of
C++ Gazebo scene-service compatibility.
The snapshot must include stable world/entity IDs, box dimensions or sphere
radius, simple color, explicit units and frame conversion. Pose updates carry
simulation timestamps and reset epochs. Reconnect replaces stale state from a
fresh snapshot. Bound update/command queues; show connection/error state rather
than freezing silently. Browser pause/resume/reset acknowledgements reflect the
actual server state. A hard-coded or replayed browser animation does not qualify.

Day 1: prove one live body in the actual browser and agree the minimal contract.
Day 2: complete the primitive scene, poses and coordinate conversion.
Day 3: connect controls and reconnect snapshots.
Day 4: complete reset/shutdown/error regressions, clean-start commands and docs.
Day 5: finish independent review, batch fixes, run required final checks and ship.
This is a work breakdown, not permission for daily broad test/lint reruns.

M1 excludes meshes, sensors, jointed robots, remote Fuel downloads, native-engine
feature extensions and full Gazebo service compatibility. They are deferred,
not retired or credited as complete. M2 visuals still use primitive collisions;
M3 adds only the named collision shapes. Each sensor phase activates its own
model on demand and preserves zero per-system runtime cost when absent.

## Later cycles and retained scope

After M12, choose bounded items by dependency: one additional sensor/noise model,
one Gazebo system, one asset format, one native backend capability, or one
packaging/performance improvement. Independent items may share a parallel source
wave under D31; shared runtime interfaces and heavy qualification have one owner. Before starting, name its fixtures, commands,
unsupported cases and exit evidence and budget no more than five working days.
D33 cancels DART/MuJoCo parity. Split retained TPE parity, remaining 78-system behavior, GPU lidar,
advanced cameras, distributed worlds, historical logs and Python workflows into
such increments. Keep the full legacy packet/test ledger for traceability.
Accepted phase counts do **not** set a whole-rewrite completion date. Continue
selecting bounded deliveries while retained work remains under D32.

M30 needs a narrow SDFormat correction over its consumed revision: admit the
matching LogicalCamera raw child without interpreting it. That dependency must
pass its own review, staged hook and hosted CI before the simulator pins it.
This does not resume the paused automatic-mass candidate.

Do not resume paused SDFormat mass-override or metric-depth-worker qualification
as a competing priority. Preserve their trees, evidence and accepted Main pins.
Adopt or finish them only when a weekly delivery actually depends on them. Never
edit the independent Fuel owner's work.
