<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M58: two independent loaded joint-controller chains

M58 is locally unqualified work on locally qualified, unaccepted M57
`9dfde487e7e377432573ebbcfd1e7a58901bc492`, tree
`5436a12578fca5cd745528913d6f4427bebe319b`. That parent passed its full normal
hook and independent retained DATA; its failed normal attempts, interrupted
controller, owned cleanup and failed readers remain preserved. Parent results do
not qualify M58. M1–M42 and 175 slices remain accepted; M40 remains Latest.

Scene, Server, shared identity/boundary integration, regression Source and the
canonical C++ sender have independent Source review, including the reset error
order and private binding-witness corrections. The Rust controls remain Source declarations, not measured passes. Fresh
reference capture, independent raw DATA, borrowed fixture generation and
independent literal DATA have passed. The consolidated formatter passed. The
first full normal failed during strict Clippy compilation after 186.102 seconds,
before tests, Python, native-free/RGB or coverage. All available diagnostics were
batched: missing `ChildOf`, two production physics paths using a dev-only crate,
and three exhaustive retained test-helper matches missing the controller family.
The correction imports the existing hierarchy type, moves the existing internal
physics dependency from test-only to normal Server dependencies, and excludes
controllers from the same seven-family helpers. No new package, feature, old
assertion or numeric behavior changes. Independent Source review and the one
affected formatter passed; a full normal retry remains required. The failed tree, log, partial
packet and actual failure remain retained at
`refs/source-snapshots/m58-normal-attempt1`; see
`artifacts/m58-delivery/m58-normal-attempt1-actual-failure.json`. M58 product tests,
strict coverage and independent product results DATA remain unpassed.
The second normal returned 1 after 196.146 seconds, also before tests or coverage.
Its complete available diagnostics are eight integration-test calls to private
`Server::set_paused`. The correction uses asserted public `apply_local_control`
with the same pause values, preserving the active clock, simulation pause state
and every old assertion. The first compile diagnostics are absent from this run.
All external API uses and the complete correction passed independent review,
and the affected formatter passed. A full retry is still required. Retain `refs/source-snapshots/m58-normal-attempt2` and
`m58-normal-attempt2-actual-failure.json`; neither failed run qualifies M58.
The third normal passed both strict Clippy profiles and the selected renderer
doc-test stage, then failed one of 404 aggregate Python checks: the copied-run
regression expected 31 executions although the finite registry now contains 32.
The correction changes only that literal expectation to 32; exact copied bytes,
commands, ordering, environment, bounds, actual-result and negative controls stay
unchanged. No Rust changed, so formatter evidence is reused. Preserve
`refs/source-snapshots/m58-normal-attempt3` and
`m58-normal-attempt3-actual-failure.json`. Full runtime tests, native-free/RGB
proof and strict coverage were not reached; a full retry and retained results
DATA are still required.
The fourth normal returned 1 after 984.376 seconds. Strict lint, all 404 Python
checks, both doc-test stages and native-free/RGB qualification completed. Before
full profiles started, selected-reference validation rejected the unchanged
Zenoh probe's dependency map. Retained metadata has three old libc/libm/libresolv
hashes matching the already reviewed libc transition; the failed fresh mapping
itself was not retained. Preserve `refs/source-snapshots/m58-normal-attempt4`
and `m58-normal-attempt4-actual-failure.json`, including the passed auxiliary
packet. No full runtime profile, strict coverage or local qualification passed.
The bounded repair adds an explicit separate same-probe runtime attestation and
dual provenance selection; it preserves every bootstrap self-input, old SDK,
manifest, probe and cache. See [the verification contract](TESTING.md#cached-zenoh-runtime-probe-after-the-recorded-libc-update).
The complete tooling/regression/docs batch passed independent Source review.
One fresh run of the unchanged original probe returned genuine external 0;
independent retained review confirmed all five zero/reaped stages, the full
current 24-library map, dynamic identity, ready line and operating floors.
The separately selected proof is retained in
`artifacts/m58-delivery/zenoh-runtime-probe-current-libc-attempt1` with actual
completion `actual-exit.json` and immutable `selection.json`. No SDK rebuild,
old evidence rewrite or Rust change occurred. The full normal retry, runtime
profiles, strict coverage and retained results DATA remain required.
The fifth full normal returned 1 after 1848.903438094072 seconds. Both strict
lint profiles, 440 aggregate Python checks, both doc-test stages, native-free/RGB
qualification and the selected current native-probe validation passed. Headless
keep-going found exactly two failed targets: the historical DART static-contact
replay's library guard and the old Server reset-error assertion. Scene/Server
and other partial passes do not establish complete qualification. Renderer and
complete coverage were not reached. Preserve the failed tree, 2,243 retained
artifacts, pure hook/controller logs and `m58-normal-attempt5-actual-failure.json`.

One bounded read-only inventory of the historical eighteen libraries found
fifteen unchanged hashes and exactly the known libc/libm/loader transition;
all three DART providers are unchanged. The fixture stays byte-exact. The explicit
`GZ_NATIVE_DEPENDENCY_SNAPSHOT` selection joins the reviewed current snapshot,
whole dependency inventory and genuine Root completion. Its in-memory projection
changes only those three historical SHA leaves, whose inverse recovers the full
old table. Unset keeps the old strict policy; an invalid explicit selection never
falls back. All eighteen live dependencies, compiler inputs, seven modes and
exact numeric observations must still pass the maintained full normal. Selection
and static inventory are not native execution or ABI qualification.

The reset correction adds a separate post-reconstruction preparation path.
It checks private/native binding against trusted original metadata, all structural
and original/current owner-key constraints and all peer reservations first.
Only then does otherwise valid public metadata drift return the retained
`ResetJointControllers` error. Constructor/live paths still require the full
public witness; no generic identity error is remapped. Two regression Source
controls cover reason isolation and state/lease purity; old assertions stay exact.
Complete the full Source/regression/docs batch and independent review, then one
affected formatter, final Source census and one full normal retry. Native capture,
literal DATA and the valid same-probe proof are reused without replay. M59 remains
disjoint Source preparation until M58 normal and independent DATA actually pass.

The sixth normal returned 1 after 206.12135092099197 seconds during strict Clippy
test compilation. Its complete keep-going diagnostics found two errors: the
retained private control still calls the original two-argument `candidates`, and
the new boundary-module purity control cannot access private Controller streams.
Python, runtime tests and coverage were not reached. Preserve failed tree
`fd4e9729`, its complete logs/partial packet and actual-failure receipt.

The batched correction keeps a real strict two-argument production wrapper.
It passes `None` to the internal implementation; trusted saved metadata is
passed only during restoration. The complete new
purity control moves into the existing Controller child module; its exact lease
pointer/count assertions remain there, and it reuses the same complete boundary
state snapshot via test-helper visibility only. No production field visibility,
old assertion or native/probe policy changes. The moved control's qualified
Source identity changes explicitly; its old failed Source remains preserved.
Complete independent whole review before one affected formatter, final Source
census and one full normal retry. Successful results DATA stays held.

The seventh normal returned 1 after 1758.7969630630687 seconds at strict
actual-parent changed-line coverage: Server 581/612 executable lines must exceed
95%. Both lint profiles, 448 aggregate quiet Python checks, docs, native-free/RGB
and all 1,980 headless Rust passes completed. Five ignored child harness entries
remain the existing explicitly executed child controls. The DART native replay,
old reset-error assertion and both corrective controls passed. All twelve
headless crates pass whole-source line and region coverage; Scene changed lines
pass at 183/188. Renderer and full normal/local DATA remain unpassed. Preserve
failed tree `158e8a7c`, its 2,255 retained partial artifacts and actual-failure receipt.

The coverage correction is test-only: genuine original IMU/rate/Lidar/Drive and
JointForce reservations survive model retirement; a partial actual lease handoff
is refused without state changes; and a controller-only legacy collision retains
constructor, direct reset-plan and real reset priority/purity. Existing cases
and assertions remain intact. Do not force unreachable defensive blueprint/getter
errors, refactor production for counts, change thresholds or substitute Source
counts for results. Complete the whole test/guide batch and independent review,
then one affected formatter, final census and full normal retry. Existing native
capture/literal DATA and same-probe evidence are reused without execution.

The first input retrieval returned Root exit 1: engine-info metadata completed 0,
then image inspection failed on the optional `.Config.Volumes` template lookup.
No container was created, no file copy ran, and all four inputs remained
unreached; all operating floors held. This method failure proves neither image
identity/availability nor input admission. Its exact receipt is retained as
`controller-reference-inputs-retrieval-attempt1-actual-exit.json` (`1f25671a`).
The separately reviewed correction ran once and recovered all four exact pinned
original inputs. Its retained result reports zero; the final external tool exit
was unavailable after output truncation and is explicitly not inferred. The
never-started owned container and all original streams remain retained. Read
`controller-reference-inputs-retrieval-attempt2-observed-result.json` (`ff9972a5`).
Independent retained-byte DATA subsequently passed. The one static inspection
completed all twenty child reads with exit0, but its controller exited1: every
read warned about the unsupported optional debug flag, and both original DSOs
have trailing-empty RUNPATH entries and remain refused. The two REL objects
also share a strong, non-COMDAT `GzPluginHook`; a single two-object executable
is not admitted. The reference design now uses one unchanged original object
per isolated controller-family helper, with two owner instances each. The completed isolated captures establish this bounded component path, while
the original DSOs remain refused. No static query was repeated or relabelled.

## Bounded change

Admit zero through two independent top-level dynamic Rapier models. Each retains
the accepted M9 two-link, two-+Z-revolute-joint world-rooted serial chain, with one
or two distinct configured joints. The world therefore has at most four
canonical `gz.msgs.Double` command inputs. Repeated local joint names across
models are valid; full loaded identity and original parent model distinguish
owners. A third configured owner or fifth input is refused.

[M9 controller behavior](M9_JOINT_CONTROLLERS.md) and
[M9 ingress behavior](M9_JOINT_CONTROLLER_IO.md) remain the numeric and lifecycle
basis. Parser fields, topology, reduced-coordinate construction, torque caps,
velocity clamping, continuous positions, damped position-servo constants and
finite arithmetic are unchanged. There is no new backend, solver, force-PID,
actuator message, output family, acknowledgement, worker or IO thread. Namespace,
nested-model, cross-owner joint, physical joint-stop and general articulation
support are not added. An unconfigured joint keeps the existing infinite
effort/velocity and zero-dynamics domain; simply removing its controller plugin
while leaving finite actuator limits does not make a supported fixture.

## Identity, atomicity and reset

Root's borrowed `InitialJointControllerIdentity` exposes original `id`, `kind`,
world-scoped `name`, stable parent model ID and configuration from immutable
`InitialScene`. The existing getter remains unchanged. Missing initial state is
distinct from a loaded empty controller set; removed originals remain reserved
for loaded reset. Mutable current metadata cannot authorize another owner.

Scene compares current ID/kind/name/model parent/configuration and actual native
joint/child bindings against that original set before target or motor mutation.
All requested motors are prepared and validated before any native request is
applied. Reset preserves the original per-row query order: check the admitted
slot bound, then validate that row's native joint before advancing. An earlier
missing-native-joint error therefore still wins over a later counterfeit extra
controller. The overflow diagnostic identifies the actual overflowing query
match, not an assumed spawn order. Every native clear and initial-target restore
follows the complete native and identity preflight.

Server validates complete original/current/retained/prepared sets and final
normalized/remapped keys before declaring inputs or committing a replacement
generation. Root's shared constructor/reset boundaries retain existing typed
collision precedence and all peer-family reservations. A retained live lease
must witness its stored **old** actual endpoint; each prepared lease must witness
its **new** remapped endpoint. Distinct same-channel Arc swaps and duplicate Arcs
are separate refusals. Pointer uniqueness alone is insufficient.

Reconciliation checks every surviving stream before any receiver is polled.
Live identity/configuration/parent/native drift remains a global terminal fault,
with the existing input-family retirement and later terminal reset behavior.
Actual owner disappearance or pending removal retires its streams; it is not a
fabricated binding fault. Coherent model removal permits peer motion, and last
removal retires scheduled callbacks and input resources. Loaded reset restores
original IDs and configured initial targets while retiring old generations,
queues and held callback captures. No peer rollback after a genuine later fault
is promised.

## Regression and evidence duties

The [two-chain fixture](../examples/m58_multiple_joint_controllers.sdf) duplicates
the genuine M9 chain as `arm_left` and `arm_right`, separated in space with zero
gravity. The authored Scene controls cover zero/single/pair and mixed one/two
configurations, owner limits and late invalid input, opposite continuous
position/velocity targets, real backend state and body geometry, pause, invalid
commands, reset and first/last removal. Same-ID reset controls preserve the old
counterfeit diagnostic and exercise a simultaneous earlier native fault. Existing
M9 tests, assertions, tolerances and neighboring refusals remain intact.

Server Source covers full-set admission before IO, exact original/current
identities, native child binding, old/new endpoint witnesses, distinct swaps and
duplicates, remap/reset/restoration, epochs, budgets, invalid-command target
preservation and terminal-versus-retirement premises. Its maintained C++ Double
sender supplies actual canonical wire and a separate Rust-owned physical motion
witness. Subscriber matching is discovery, not an actuation acknowledgement.
The fixture's finite CLI load/step obligation is separate from those controls.
No pose/body-velocity injection or inferred solver trajectory is evidence.

The exact four-original-file retrieval proposal and its bounded executor are
Source-reviewed (`e061d5ef` and `afbbde02` in the retained M58 procedure records).
The corrected attempt establishes retained file identities, not ELF/provider
admission. The completed retrieval used
one fixed local engine/image, four exact paths, an owned never-started container,
strict stream/tar/hash custody and no pull, build, mount, start, fallback or
provider load. Every nonzero copy is an unclassified hard stop. Independent retained DATA and static input admission require separate reviewed
Root bindings and release.

Read `reference-two-helper-isolation-source-decision.json` (`dbf18b24`) and
`controller-reference-static-query-actual-exit.json` (`df363f95`) for the exact
observations and revised Source-only boundary. Requested safety flags do not
prove that the rejected option actually disabled a capability.

The completed reference establishes both original controller families in
isolated helper executables, each linking exactly one unchanged original object. This earns per-family component evidence, not original
simultaneous cross-family execution or Rust-servo parity. Position requires genuine public `Configure`, then
`ConfigureParameters(ParametersRegistry&, EntityComponentManager&)`, then
`PreUpdate`, with reviewed registry ABI, link inputs and lifetimes. Configure plus
PreUpdate alone can leave the original position joint set unresolved. Velocity
component commands may remain raw where Rust clamps; original position ABS
lowering/order differs from the Rust damped servo. Compare only justified common
configuration/input semantics and preserve observed differences. Do not claim
original Server, solver, lifecycle or trajectory equivalence. Historical native
controller subrecords keep their original scope and failed campaign history.


The current reference pipeline completed all 49 child stages with actual zero
exits, followed by a separate successful independent raw audit. It retains 137
rows, 26 controlled input rounds, 52 PreUpdate calls and eight publications,
taps and effect fences. The generated fixture and its literal audit preserve all
records, optional presence, IEEE words, raw order and native wire bytes. The
Scene observed-input control uses captured command inputs through the existing
setter and checks target isolation without backend writes. Server uses the same
captured wires through real canonical ingress and checks selected queues,
application/rejection counters and unchanged physical state. Neither control has
run as a Rust product test yet.

A host libc6 upgrade changed four of the 349 saved library bodies. One complete
351-input inventory found the other 345 current349 nodes and both separate link
inputs unchanged. All four changed files match the installed package manifest;
their byte counts, complete dynamic texts and dependency edges are unchanged.
The new snapshot preserves the old manifests and full MySQL provenance. A fresh
capture and audit validated every current hash; earlier successful captures are
historical evidence, and the failed audit remains failed. The upgrade initiator
and that audit's unrecorded exact callsite remain unknown. See the exact records
in `planning/reference/m58-multiple-joint-controllers.json`.

## Completion and release

The increment has a maximum five-working-day scope budget, not an ETA or a pass
forecast. Reversible isolated Rust Source work may proceed while input retrieval
and admission close. No unavailable input may be consumed by native execution,
final verification or qualification. Stop for a concrete unresolved dependency
rather than broadening the physical domain or relabeling a failure.

Finish implementation, regressions, shared integration, guides, registrations
and native-procedure Source, then complete independent whole review and batch
all findings. Root releases one consolidated affected formatting/verification
sequence and the unchanged mandatory full normal hook, strict workspace,
per-crate and actual-parent coverage, followed by independent retained DATA.
Preserve all parent controls, cleanup/BOTH/wrench obligations and finite prefix.
The independent report keeps its 2 MiB bound and 64 KiB RAW reserve; four fresh
RAW comparisons run once last. Prior failed methods are never success evidence.

Local qualification is provisional. Accepted exact predecessors, actual hosted
CI, official audits and ordinary Main/readbacks remain mandatory before release
or acceptance. Ubuntu snapshot recovery is unconfirmed. This guide releases no
operation, CI retry, publication or acceptance.
