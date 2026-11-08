<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Coordinator handoff

> M1–M20 are accepted and published at qualified Main `9a3e81c`.
> M20's checked installed preview is published as `m20-preview`; its frozen
> source, artifacts and original failures remain preserved.
> Current M21 altimeter noise and M22 air-pressure noise use independent Source
> lanes under D30–D32. Neither inherits acceptance. One integrator serializes
> local heavy checks. Preserve accepted evidence, paused trees and Fuel work.


## Current work: M21/M22 regression corrections

[M20 acceptance](M20_ACCEPTANCE.md) records qualified Main `9a3e81c`, exact
hosted run 42479817732 attempt 1, independent local/official/package/install/browser
DATA and actual public release 438259397 at 2026-11-08T13:18:29Z. All seven
uploaded API sizes/digests, exact notes and intentional Latest designation were
read back. The complete robot moved forward and turned, acknowledged Play,
remained paused across two frames, reset to epoch 1/time zero and retired owned
processes/listeners. Original failures, the recovered idle warning and unknown
simulator-grandchild wait remain accurately disclosed. No M20 work is resumed.

[M21/M22 scope](M21_M22_DELIVERY.md) selects two independent bounded increments:
loaded altimeter and air-pressure Gaussian/quantized noise using the maintained
noise primitives. Native application uses zero elapsed time; pressure variance
is native double `sqrt(stddev)`. Both complete leaf batches passed independent
Source review, including the two-line pressure test Copy correction, and are
integrated in `gz-sim-m21-m22`. Root owns shared error exports, current docs and
planning. The reviewed native batch completed all 35 stages across ten modes
and 430 printed rows; independent retained native DATA passed. The first full
normal hook then failed two headless regressions: negative noise deviation was
already rejected by the SDF parser, and a pressure fixture assumed motion beyond
the backend velocity bound. Correct those fixtures together, preserve their
refusal and stale-packet assertions, and complete independent review before the
next required consolidated pass. The original failed tree and evidence remain
retained. No normal, hosted or phase acceptance is inferred. Local heavy work
stays serial; parent M20 acceptance does not qualify these new phases.

The sections below preserve historical preparation checkpoints; this current
state supersedes their pending statements. Do not replay old accepted work.

## Historical M19 preparation checkpoint

M19 is selected under D30–D32 in isolated `work/m19-refreshed-preview`.
[M19_DELIVERY.md](M19_DELIVERY.md) records its five-working-day scope and file
ownership. The assembler copies the coexistence robot and its one OBJ dependency;
dedicated installed guides and complete legacy-guide verification are retained.
Exactly loopback HTTP 8082 is added alongside 8080/8081, preserving defaults and
other origin/path refusals so protected old services remain untouched. Product
source, meaningful regressions and guides have independent review; shared planning
and the exact qualified-parent binding also require review before the one
consolidated normal pass. No M19 check, build, package or installed proof has run.

Source preparation adopted the exact M17/M18 parent tree `f74ed3e6` while its
final hook ran. The actual locally qualified parent is now `c7402b45`, with that
same tree: full normal hook 2020.358 seconds/exit zero, all 29 native stages in
59.336 seconds, native DATA `d47a6a15` and local DATA `46307263` clear with actual
audit exits zero. This branch was advanced to that exact parent while preserving
all reviewed product bytes, modes and index. Parent hosted CI, official audit and
Main acceptance remain required. Reviewed M19 local checks may now overlap that
hosted run, with local heavy work serial; neither parent nor child inherits
acceptance or may publish before every required gate and proof passes.

Build fresh release CLI/helper inputs, derive actual dependencies/notices and ELF
closure, assemble/relocate/audit the whole ordinary-file payload, and prove one
clean supported installation with non-root verification and actual installed
robot/browser forward, turn, Pause, Reset and owned shutdown. Reuse the existing
corrected procedures and accepted viewer bytes, not old M12 executables or fixed
file/package counts. Keep full hooks, strict coverage, exact hosted CI and final
audits before qualified Main promotion and a new preview release. Source target
completion flags take effect only at that boundary; actual IDs/times belong in
the release/promotion receipt and owned tracker status. Continue with the next
useful bounded phase after acceptance; preserve every old release and Fuel work.

## M17/M18 parent qualification checkpoint

The owner directs continuous delivery (D32). Work continues in isolated branch
`work/m17-m18-finite-native` over qualified Main `5f53c92e`.
[M17_M18_DELIVERY.md](M17_M18_DELIVERY.md) records the five-working-day scopes,
file ownership, native/wire proofs and exclusions. Implementation, regressions,
shared Scene/Server integration and documentation have independent Source review.
The actual completed hook and native observer failure are recorded below;
hosted CI and phase acceptance have not run.
No M17/M18 acceptance follows from source authoring or older qualification.

Candidate `6ab6010b` passed its full normal hook in 2026.380 seconds after the
batched compile and joint-motion test corrections. That source and its original
failures remain frozen. The native run compiled both plugins/observers and
completed all three magnetic modes, then the joint observer exceeded its finite
deadline: ordinary paused `Run` did not count iterations and produced duplicate
discovery callbacks. The corrected observer uses pinned native `RunOnce(true)`
for exactly one discovery and two final paused ticks. A second native attempt
completed the default mode's 102 observations, then the filtered observer found
missing joint-value arrays. Pinned source showed that the publisher requests
measurements only for its selected joints. The observer now explicitly requests
Physics-owned position/velocity measurements for both observed joints, without
changing the publication filter or fabricating values. Every packet, component,
count, cadence and motion assertion remains. Independently review the complete
helper/guide batch, run the corrected native check first, then one final full
normal hook on this maintained candidate. All native/local/hosted audits and
qualified Main promotion remain required; no earlier result accepts this phase.

M19 installer source preparation proceeds separately in `gz-sim-m19`, with
independently reviewed fixed robot/OBJ copies, dedicated guides, legacy guide
verification and one additional explicit loopback HTTP port 8082. No M19 checks,
release builds, package assembly or installed proof have run. Its local checks
may overlap M17/M18 hosted CI only after this exact parent's normal, native and
independent local proofs pass. Local heavy work stays serial; final distribution
and acceptance require the finally qualified parent and every M19 gate.

M15/M16 were accepted and published at Main
`5f53c92e04822ec25b00ca8483d2890bd1f70d40`, tree
`88a2e2ce03591d05510d41f881362f3d3fdf8693`, after the full normal hook
(1982.348 seconds; 1332 headless, 830 renderer and 334 tooling passes), exact
hosted `42446229290` attempt 1/all sixteen steps and independent local/official
DATA. Actual native pressure and OBJ robot/browser forward/turn/Pause/Reset/
clean-shutdown proofs also passed with independent DATA and image review.
The [public source-feature release](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m15-m16-features)
contains the actual receipt and eight JPEGs. Earlier browser timeouts, operator
input failures, attached-frame corrections and host-compiler preflight refusal
remain preserved; connection warnings remain disclosed. Local and official
coverage/profile counts stay separate. M12 installer and viewer `c40418cd` are
unchanged. Tracker reports 16 accepted deliveries and the next two in progress,
with all 154 older reviewed slices retained plus two new slices (156 total).

Complete the new bounded source, meaningful regressions, documentation and
independent review before the consolidated normal hook. Batch real failures
before affected reruns; preserve strict coverage and exact hosted CI. Reuse
unchanged evidence where permitted. After acceptance, continue directly to the
next useful named phase. Preserve pending/Fuel work and every frozen tree.

## Accepted M13/M14 baseline

Main `2f32bc7e906286ad16d9ec513c4bac659e21f996` passed the full normal hook
in 1967.681 seconds, exact hosted CI `42432620121` attempt 1/all sixteen steps,
independent local/official DATA, actual native altimeter comparison and one real
STL robot/browser forward/turn/Pause/Reset/clean-shutdown proof. The
[public feature release](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m13-m14-features)
contains the actual acceptance receipt and images. Local profiles passed
1289 headless and 815 renderer tests; the quiet tooling suite passed 334.
Local and hosted coverage remain separate. The browser needed one startup
reconnect; the warning and original evidence remain retained. This is source
feature acceptance, not an updated installer or a performance/parity claim.

The tracker recorded fourteen accepted deliveries and 154 reviewed slices,
preserving all 152 earlier slices. M12 distribution remains bound to source
`9147120a`, runtime `a542cb1b` and accepted viewer `c40418cd`. Its original
failures, payload and clean-install proof are preserved; do not replay them.
The previous M12 automation remains deleted. A new persistent continuation
heartbeat was rejected by automatic approval review; no automation was created
or workaround installed. Continue the authorized work in the current chat.

D31 still makes qualified Main promotion the acceptance boundary. Record actual
run IDs/times in the existing release/promotion receipt and owned status; do not
invent future IDs or run a second metadata-only gate to add generated values.

## Historical M12 closeout source record (superseded)

Updated 2026-11-07 UTC. Read DELIVERY_PLAN.md and M12_ACCEPTANCE.md first.

## Current closeout: M12 accepted; next work unselected

Root formally accepted M12 runtime/distribution at
`2026-11-07T19:34:44.558424+00:00` after actual qualification, clean installed
verification and normal publication/readback. Acceptance record
`m12-runtime-acceptance.json` has SHA `4af8e31e…`. Main is published at
`9147120aaccd9383d8e406c1b8ee82348153b734`; the public prerelease
[m12-preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m12-preview)
targets that exact commit. All three archive/checksum/provenance asset digests
were read back. Publication receipt is `e855b546…`; no future URL/ref is invented.

The final archive is 39,769,747 bytes, SHA `728bb722…`, manifest `53a77fa0…`.
Native executables remain `a542cb1b`; six viewer files remain `c40418cd`.
Separate generation/consumer-guide source is `9147120a`, tree `70084c19…`.
Normal hook passed in 1928.683120 seconds; local DATA `f07ac2c5…` is CLEAR.
Exact hosted CI 42402368577 attempt 1/all 16 steps and official DATA `14b87792…`
are CLEAR: 1248 headless/802 renderer/334 whole-suite Python passes. The 45 Python
methods are Source inventory, not individually logged quiet outcomes. The actual
hosted diff base was `a542cb1b`, with unchanged Rust 0/0; corrected reader semantics
preserve all strict whole/per-crate checks and the original failed method record.

Final package DATA `20bd1a84…` preserves original wrapper 1/UNKNOWN old child waits
and the completed no-repeat report-recovery basis. Continuation `c2fe1139…` has
actual extraction/relocated-verify 0. Final clean Ubuntu setup `df880f9b…` and
independent installed DATA `1044dc77…` are CLEAR: 25 CLI exits 0/seven guest exits 0/11 layer
snapshots, read-only payload unchanged, no compiler/SDK, owned idle container
stopped. Earlier installed robot/browser DATA `2c92603e…` is reused only through
exact 1560 non-guide/11 behavior byte-and-mode lineage. Actual helpers drove/turned,
Pause/Reset ACKs were observed, launcher/helper exits 0 and owned listeners retired;
simulator-grandchild return code remains UNKNOWN. Static HTTP service was restored;
the authorized old M9 live world ended and was not replayed.

M1–M12 are complete. No next scope or M13 is selected; full rewrite/backlog work
is not complete. Preserve every accepted contract, old evidence, Fuel and retained
failures. This nine-file canonical metadata needs its own independent Source
review, full normal gate, exact hosted CI and official DATA before publication.
Runtime/source/release qualification does not qualify this later metadata commit
or claim a future automatic Main workflow result. Tracker/status mirror updates
are separately reviewed and must preserve all 151 existing slices and other owners.

The native metadata launcher keeps 14 GiB initial scratch reservation and all four
operating floors. Its chosen SSD scratch is outside all Cargo ancestors; this
operational placement does not change verify.py or thresholds. Prior nested
workspace, wrapper, Source-reader, setup/startup and unknown storage-drift failures
remain recorded. See M12_ACCEPTANCE.md for exact hashes and evidence limits.

## Historical M11-to-M12 implementation handoff (superseded)

Updated 2026-11-05 UTC. Read DELIVERY_PLAN.md and M11_ACCEPTANCE.md first.

## Current delivery: M12 installable integrated preview

M11 runtime is accepted and published on simulator Main `182773f7` and viewer
Main `c40418cd`. Their exact review-ref runs `42103914738` and `41968630547`,
normal required checks, independent source/data/bundle audits and completed
actual browser image/data review passed. Direct owner approval preceded normal
fast-forward publication and exact remote readback. The viewer's publication
pre-push coverage passed in 151.069 seconds. Automatic viewer Main run `42130130009`, attempt 1, has separately completed
successfully with all six jobs at exact `c40418cd`, as authoritatively observed by
Root. Automatic simulator Main run `42130320961`, attempt 1, also succeeded at exact
`182773f7`; verify-core job `127604984873` completed on 2026-11-06 at
00:23:06 UTC, as authoritatively observed by Root. These separate successful
Main runs do not replace the audited review-ref proofs or qualify this metadata. The canonical closeout metadata itself
must pass its normal gate and exact hosted CI before publication.

The original 1,129-frame capture and unchanged exact comparator passed against
the corrected runtime. Browser evidence includes runtime box18 and mesh22,
reconnect at Frame530/source18.440 with both present, ordered removals, recorded
reset and EOF, and cursor Reset to Frame1/sourceepoch0/paused. Browser Play at
EOF was refused client-side; the already-passing production Player regression
proves API-level EOF Resume refusal. The unchanged external comparator proves
EOF completion, Reset and credit fences; it sends no Resume at EOF. The retained Scene trace contains hashes, not full pose payloads.
Saved `.png`-named browser images contain original JPEG bytes. Recovered idle
warnings and both storage-floor stops retain unknown causes; the original
one-ULP failure and all compile/lint/coverage failures remain separate records.
The qualified preview remains paused on9013. Preserve it and older demos; do not
repeat passed observers or accepted manual qualification.

Current M12 packages the accepted subset and documents supported capabilities,
setup and one runnable robot demo, at most five working days including review
and mandatory checks. Its exit is a clean supported Linux installation running
the integrated demo, with required coverage, CI, licenses, retained regressions
and a published limitations matrix at the release commit. Reuse existing build,
installation, CLI, examples and maintained viewer paths; inspect them and agree
the smallest packaging contract before implementation. No M12 implementation or
clean-machine acceptance is supplied by this closeout. Full rewrite completion
and unsupported platforms, backends and features remain excluded.

This current delivery and DELIVERY_PLAN.md supersede inherited older phase
banners. Preserve M1–M11 behavior, demand-driven absence, physics authority,
native-free/GPU-free headless defaults, retained assertions and quality gates.
Use upstream Zenoh and schema-only Messages without new runtime facades. Do not
resume retired or frozen qualifications, change Fuel, or delete evidence. Finish
bounded source, regression code, documentation and independent review before a
consolidated required pass. Batch real failures before affected reruns. Root's
native metadata-gate launcher requires its unchanged 14 GiB initial scratch
reservation and four operating floors; this is an operational launch guard, not
a new verify.py policy or a requirement for every viewer/build qualification.
This source-only preparation launches no gate. Runtime acceptance does
not qualify this new metadata commit. Full P0–P7 remains a separate deferred
inventory, not an automatic M12 assignment.

## Historical M10-to-M11 handoff (superseded)

Updated 2026-11-05 UTC. Read DELIVERY_PLAN.md and M10_ACCEPTANCE.md first.

### Previous delivery: M11 focused recording and replay

M10 runtime is accepted and published on simulator Main `d3967b61` and viewer Main
`952a5202`. Required normal local checks, exact hosted CI `41924068764` and
`41937814296`, independent source/local/official/bundle audits and reviewed actual
browser observations passed. The simulator keeps strict owned workspace/per-crate
line and region gates; the viewer keeps its existing workspace line-coverage gate.
Do not conflate these policies or relabel old ZeroMQ evidence as Zenoh acceptance.

The actual browser spawned/read/edited/moved/removed box18, spawned/removed
sphere22, spawned mesh26, reloaded to the current snapshot with mesh26 intact,
removed it, spawned fresh mesh30 and reset to epoch1/time0/paused with only
original IDs2,6,10,14. The browser and native world remain paused on9011. Two
initial/reload idle warnings recovered automatically; their cause is UNKNOWN.
The retained bottom log warning is historical alongside successful current ACKs.
Do not erase those warnings or claim the separate queued-ACK timer fix caused
recovery. See M10_ACCEPTANCE.md for exact receipts and evidence limitations.

M11 delivers one local recording of the admitted scene, poses and controls and
replays its timestamps and entity lifecycle in gzweb-rs, at most five working
days including independent review and mandatory checks. Root owns simulator
scene/recording/CLI integration. Inspect existing maintained recording/replay and
clock/resource APIs and agree the small admission/wire contract before parallel
viewer edits. Prove malformed-input refusal and bounded buffering. Historical
Gazebo log-format compatibility and full sensor recording remain excluded.

This M10 acceptance metadata change must pass its normal staged gate and
exact hosted CI before publication. Automatic Main workflows are separate from the already audited
review-ref successes; do not label pending runs passed. Metadata qualification
may finish while isolated M11 source progresses, with one local heavy job at a
time. Do not rerun accepted manual/browser qualification. Preserve all old demos,
raw failures, frozen candidates, pinned dependencies and independent Fuel work.
Use upstream Zenoh unmodified, schema-only Messages and demand-activated built-ins;
no dynamic system plugins or separate transport/runtime rewrite.

The historical handoffs below retain prior instructions and evidence. Their next
tasks and pending statements are superseded by this current delivery section.

Updated 2026-11-04. Read DELIVERY_PLAN.md and M9_ACCEPTANCE.md first.

## Historical M9-to-M10 handoff (superseded)

M9 is accepted and published on simulator Main `0f0f031e` after the normal
1411.351-second staged gate, exact CI `41817794153` (all 16 steps, attempt 1),
independent source/local/official audits and actual browser image review passed.
Canonical Double commands move the real Rapier chain. Continuous position,
velocity, torque/target limits, identity, pause/reset/removal and absence controls
pass. M9 uses maintained reduced-coordinate native motors; M8's compliant motor
policy and all old effort refusals remain unchanged. Position control is the
bounded Rust servo described in M9_JOINT_CONTROLLERS.md, not native ABS parity.
All original convergence assertions and earlier failures remain preserved.

Viewer Main `516a9a0e` and the accepted `19be842d` bundles are reused unchanged.
Five actual browser images show base/child motion and acknowledged resume, pause
and reset. The M9 preview remains paused on9009. Preserve the M8 preview on9008
and all older demos; no accepted manual qualification needs replay. The finite
M9 sender's full normal Cargo JSON and actual executable are retained.

Current M10 exposes focused spawn/remove and entity-state operations to the live
browser for the admitted MVP scene, at most five working days including review
and required checks. Spawn/remove primitives and one local mesh without stale
IDs or visuals, reject requests atomically, and obtain a current snapshot on
reconnect. Root owns Scene/Server/CLI integration in an isolated M10 tree; agree
the small admission/wire contract before parallel viewer edits. Reuse maintained
resource, physics, identity and lifecycle APIs. Preserve existing body state and
physics authority, native-free/GPU-free headless defaults and default viewer
protocols. Multiworld, levels and distributed stepping remain excluded.

M9 acceptance metadata is published on `main` at `aaf95d07` after its normal
1482.359-second staged gate, exact CI `41831849573` and independent local/official
audits passed. The M9 runtime and viewer were unchanged; no accepted manual
qualification was replayed. The M10 tree includes this reviewed ancestry.

The M10 implementation contract and commands are in
[M10_WORLD_LIFECYCLE.md](M10_WORLD_LIFECYCLE.md). The explicit `--web-lifecycle`
flag and viewer `lifecycle=1` enable startup-prepared templates and the separate
model-ID catalog. Scene/Server/CLI and viewer source plus regression code are
implemented and independently reviewed in bounded slices; consolidated
verification and actual browser acceptance are still pending. Camera-owned
mutable scenes remain excluded to preserve accepted renderer admission.

Finish complete implementation, regression source, documentation and
independent source review before a consolidated required pass. Batch actual
failures before affected reruns and never weaken gates or retained assertions.
Preserve frozen candidates and independent Fuel work. Full P0–P7 remains a
separate unscheduled inventory. SDFormat Main `1594ac36` and the simulator's
producer pin `a44bf8be` remain distinct; no implicit producer migration is made.

The sections below preserve previous continuation history. Their next-task
instructions are superseded by this section and D30.

Updated 2026-11-04. Read DELIVERY_PLAN.md and M8_ACCEPTANCE.md first.

## Previous M8-to-M9 handoff (historical)

M8 is accepted and published on simulator Main `fc967388` after the normal
1163.291-second staged gate, exact CI `41776261368` and independent source,
local/official data and actual browser image reviews passed. One two-wheel Rapier
robot drives and turns from real canonical Twist commands; measured encoder
Odometry, bounded native velocity motors, pause/reset/removal and absent-system
teardown pass. Target-speed bounds are not hard physical speed caps against
external impulses. The viewer's accepted `19be842d` bundles are reused unchanged;
viewer Main `516a9a0e` adds only closeout docs and a deterministic test clock.
M7 metadata Main `4a985bfb` is also fully checked, audited and published. Original
viewer `41757072753` Quiet-test failure and M6 `41740088744` attempt1 readback
failures remain historical evidence; successful corrections/retries do not erase
or diagnose their unobserved runtime interleavings.

Current M9 delivers one admitted Rapier revolute chain with position and velocity
controllers, at most five working days including review and required checks.
Prove real commands move displayed joints with documented limits and stable IDs;
bad commands, reset and removal must pass. Reuse existing maintained native
motors, joint state, input/lifecycle and bounded canonical IO. Keep physics
authoritative and absent families free of callbacks/buffers. General
articulations, mimic/transmission semantics and other controller families are
excluded. Source/regressions/docs and independent source review precede one
consolidated required pass; batch actual failures without weakening gates.

Root owns simulator integration in a new isolated M9 tree. M8 acceptance metadata
may finish normal gates/CI while M9 source progresses separately. Do not replay
accepted M1–M8 manual qualification. Preserve the paused M8 preview on9008 and all
older accepted demos, frozen candidates and Fuel work. The full P0–P7 inventory
remains deferred. SDFormat Main `1594ac36` and simulator producer pin `a44bf8be`
remain distinct; do not silently adopt a producer migration.

The sections below preserve previous continuation history. Their next-task
instructions are superseded by this section and D30.

## Current continuation

Qualified integration base `c5dd02c0700baec963464c9e0ce6a8dbc0dfae12` adopts accepted
SDFormat `a44bf8be` and publishes the immutable contact measurement appendix.
The complete 646.069-second local gate, exact hosted run `41505982041` and
independent official source/profile/native/archive audit passed before promotion:
928 headless, 547 renderer and 254 tooling checks. Every production crate and
workspace exceeds 95% lines and regions; all 39 inherited cold spans remain cold.
Centered cylinder/capsule native inputs and observations match the producer's
accepted source; four-backend force, density, removal and reset controls pass.
This pin does not implicitly adopt later producer translations or attributes.

Parent `f136e821` accepts per-pass articulated contact factorization with all
numerical guards and the historical native oracle retained. The separately
measured runtime is `f136e821` with producer `f0d05146`, not this later consumer
pin. One audited physics-only batch matched 129600 values at `1e-12`; native
median was 11.673473 ms and Rust 17.553577 ms. Rust elapsed time was 26.8443%
lower than the earlier batch, and still 1.503715 times its paired native time.
These single-host batches include Rust Scene synchronization and exclude Server,
network, sensors and rendering. Both original and new raw archives are preserved.
See [CONTACT_PHYSICS_MEASUREMENT.md](CONTACT_PHYSICS_MEASUREMENT.md).

Main retains B03 sampled fresh-native motion and actual Gazebo C++ clock, local
pose, control and reset interoperability. Upstream Rust Zenoh is used directly
and without modification; C++ is an external test/reference peer. The native
projection covers seven body world poses at snapshots 1/10/100 and post-reset
1 ms; broader local pose records use owned Scene/snapshot checks. No all-step
production-default or full-Server performance qualification is implied.

The renderer-only metric-depth candidate is based on accepted `c5dd02c0`.
Its first normal gate passed 927 headless and 549 renderer tests, including all
three new depth tests, but failed gz-renderer region coverage at 1009/1066.
The complete failed stage remains separate. Appended admission, cancellation and
completion-error controls passed independent source review; production code,
old assertions, precision bounds and strict coverage gates are unchanged. Complete
final parent/documentation review before the one affected normal gate, then
exact hosted and official qualification. No corrected execution or acceptance
is claimed yet. Near-plane intersections remain explicitly unsupported.

Root's next `work/metric-depth-worker` source slice extends the private worker,
client and bounded protocol to carry calibrated depth frames. Its source uses
internal protocol version 3 and explicit little-endian f32 bytes so +INF survives
Serde framing; captured kinds must remain stable across replacement/cancellation.
It is unqualified and must integrate the accepted metric-depth renderer parent
before checks. It changes no Zenoh or canonical message schema and claims no
loaded depth sensor or original native sensor parity. See
`docs/METRIC_DEPTH_WORKER.md` in that source worktree.

Broad physics, sensor families, assets, radiometry, original systems and full
release performance remain open. Later sections preserve historical evidence;
use this current continuation and the public dashboard for accepted scope.

## Accepted headless milestone (2026-11-01)

P2 was accepted at `c3f5a2f3c39f27d98314e032e78c39cb44d660a0`. P2 and the current
consumer migration in ZEN-01 are accepted. The complete local hook, independent
code review, exact hosted run `41270504128` and downloaded source/profile/native
audit passed before the fast-forward. The separate Main promotion run
`41272913232` also passed. Later integrations retain their own acceptance checks.

Hosted checks execute 800 headless and 462 renderer tests plus 217 tooling tests.
All required production crates exceed 95% lines and regions. The audit binds
70 production sources, 272 inputs, 1,781 retained files and all 1,711 retained raw profiles
remerged across the two profiles to the commit. Actual C++ Zenoh peers and software Vulkan pass. See
[the P2 report](acceptance/P2_ZENOH.md) for the fifteen-row matrix and limits.
The first matched performance report belongs to `5870373`; a new optimized c3
ELF completed one independently audited matched batch. The two-context tooling
and [C3 report](acceptance/C3_MATCHED_STREAMS.md) are qualified and published;
that small paced workload does not qualify balanced robotics or later Main
performance. Never measure during builds.

Terminal admission, optional CPU force ingress and application-owned camera
cadence are included. Fatal worker reply failures arm an eight-second backstop
before logging or process exit, preserving the existing ten-second assertions.
Normal EOF/shutdown still destroys the renderer normally. The original renderer
timeout cause remains unproven. Keep that failure, the diagnostic
non-reproductions and the first failed TCP fixture run separately; none is
relabeled as a pass. The reviewed private-network fixture preserves every
immediate single-bind assertion and rejects zero-test child execution.

The accepted baseline before this adoption is `0608577`. Its optional rate
owner and bounded cold-I/O admission fix passed the complete normal hook, exact
hosted run `41287528044` and downloaded source/profile/native audit. Cold setup
uses one existing five-second enqueue-plus-receipt budget; frame admission stays
nonblocking at unchanged capacity. The original `aacebc` hosted failure is retained;
a controlled Full-queue reproduction establishes the repaired race class without
claiming to identify that original interleaving.

Main `3d290ee` adopted accepted SDF `b246a80` and retains explicit collision
density; see [the adoption boundary](AUTO_INERTIA_ADOPTION.md). Its complete normal
hook, exact hosted run `41290666226` and downloaded source/profile/native audit
passed: 825 headless, 483 renderer and 223 tooling checks. All required production
crates exceed 95% lines and regions. Density metadata and presence survive reset;
metadata edits do not recalculate producer inertia.

Main `ce071305` subsequently published the separate [Zenoh/DART reference
recipe](ZENOH_DART_REFERENCE.md). The complete normal hook, exact hosted run
`41298713640` and independent official source/profile/native audit passed: 825
headless, 483 renderer and 237 tooling checks. The actual local unchanged B03
smoke passed as a reviewed continuation after two pre-probe layout failures.
This qualifies one native reference prerequisite; Rust B03 admission, a fresh
whole-bootstrap run, balanced accuracy/performance and broader P0 remain open.

This candidate adds the programmatic [single-camera application](AUTOMATIC_CAMERA.md)
on that baseline. Bare world Sensors intent and one supported loaded camera
activate its existing output owner; camera metadata alone does not. Default Server
refuses configured camera execution before advertising world IO. The headless
Server retains no rendering dependency. Camera preparation, demand observation,
publication and teardown stay outside raw ticking. Pending reset and terminal
shutdown retire old output before publication. Both world intent and collision
density remain in the immutable reset blueprint.

The camera source and composed CPU/software/C++ evidence passed independent
review. Its first integrated normal hook failed strict CameraOutput region
coverage at 1267/1365, below the required greater-than-95% gate. That failed stage
remains preserved and unpublished. Meaningful lifecycle and identity/hierarchy
controls were added and independently reviewed. The repaired camera candidate
then passed its complete normal hook, as recorded below. Exact hosted checks and downloaded source/profile/native auditing then passed;
Main `e0b1f6b` is accepted as recorded below. CLI ownership, multiple cameras, native Manager behavior, camera
set_rate, broader activation and physical-GPU parity remain open. Earlier failed
fixture/witness/interface runs remain recorded separately; no assertions,
deadlines or coverage gates were relaxed.

The camera parent candidate `e0b1f6b` subsequently passed its complete normal
hook after disk recovery, including strict per-crate coverage and real C++
Image/CameraInfo receipt. Its exact hosted run `41307067791` and independent downloaded audit also passed
before publication on Main: 832 headless, 509 renderer and 237 tooling checks. The original strict-coverage failure and
later capacity stop remain distinct; neither was relabeled as a pass.

This candidate additionally retains complete [world-light metadata](WORLD_LIGHTS.md)
and reset identity in the Core Scene. The producer's whole Light value and source
are preserved. Supplemental author checks pass 111 Scene, 16 renderer and five
Server controls plus Clippy; independent source/evidence review passed. Two
exact former light refusals have stronger, independently approved positive
replacements; unrelated assertions and fixtures remain unchanged. Integration
retains collision density, world Sensors intent and light metadata together in
the initial-scene blueprint. The additional shared-reset regression passed on the integrated source. Its
complete normal hook, exact hosted run `41313302279` and independent downloaded
source/profile/native audit subsequently passed before Main `db9d002` acceptance:
842 headless, 520 renderer and 237 tooling checks.

The subsequent candidate combines the reviewed producer `13adde94` consumer pin
with a separate explicit-lux directional renderer input. The focused Scene
target passes nine controls, including loaded include provenance, frame override
and reset identity across all four existing backends. The renderer package
passes nine unit and four software tests on asserted llvmpipe, including exact
full-frame unlit restoration. Complete combined strict staged and exact hosted
qualification remain pending. No SDF radiometric mapping, Scene light-render
extraction, Server light-pose wire behavior or B03 acceptance follows.

Renderer extraction and Server pose publication explicitly refuse unqualified
lighting. This slice does not fabricate the native GUI-only LightVisual child,
qualify full wire/render behavior or admit the unchanged B03 DART articulation.
No light schedule, worker, endpoint or renderer dependency enters headless Scene.

Next work: broader P3/P4 physics and sensors, full ACT-01 resource absence,
balanced-robotics reference qualification and remaining systems. Fuel is
independently owned and must not be edited.

## Published Zenoh migration baseline (2026-10-31)

Commit `5870373f0d8280601b051f2cbb1bb0aba264f70f` replaces the legacy Rust
Transport and Messages runtime dependencies with upstream Zenoh 1.8 and standard
consumer-generated protobuf bindings. Root owns simulator integration; never edit
Fuel. Exact hosted run `41248418524` and independent downloaded source/input,
coverage, native and software-renderer auditing passed. Schema-only Messages
`d030d06` also passed its own exact hosted/source/archive checks.

The complete normal staged hook passes on Git tree
`5cd4cf6c9b3d6c98300f5820c9e508e0bdc135f9`: 783 headless, 435 renderer and 217
tooling tests. Five headless and three renderer helper entrypoints are delegated
to their parent harnesses. Real C++ Zenoh peers and software Vulkan workers pass.
All required production crates exceed 95% lines and regions. Workspace headless
coverage is 9761/9935 lines and 14935/15376 regions; renderer coverage is 5058/5191
and 7523/7830. Independent review verifies all 1762 artifacts, 68 production
sources and 272 inputs against exact Git objects. Staged evidence is under
`artifacts/staged/5cd4cf6c9b3d6c98300f5820c9e508e0bdc135f9-zg8jjfj8` in the
integration worktree. Hosted profiles passed independent source/input and coverage comparison.

The selected `gazebo-zenoh-client-v1` reference is locally qualified at
`artifacts/zenoh-native-reference`. Copied staged snapshots set
`GZ_ZENOH_REFERENCE_ROOT` to this qualified root. Signed snapshot packages,
source/tool identities and the locked ROS archive are verified. Identical archive
bytes were recovered over valid official mirror HTTPS after the canonical URL's
TLS hostname failure; no verification or pin was weakened.

The source-built `gazebo-zenoh-headless-v1` application is independently qualified
in immutable image
`sha256:bae6ddc3a806532805a8103967d7ab353fdaf14ab0fcb957343f8f32bece11da`.
Its stationary smoke is infrastructure evidence, not moving-workload parity or
upstream gtest execution. The actual optimized Rust ELF and its runtime overlay
retain the image's common libraries; eight missing DART-family files are bound
read-only. Exact source, compiler, ELF and mapped library records accompany the
release evidence. Do not infer whole-engine qualification from this TPE case.

Seven alternating measured pairs of 1000 steps with the unchanged driver pass
all physical, schema, wire, count, cadence and native library checks. The matched
small TPE Clock/Pose/IMU workload measures native 63289.562ns and Rust 58563.369ns
median per step: about 7.47% lower Rust latency. Raw samples and independent
recomputation are under `artifacts/zenoh-equal-work-runs`. The historical direct
and unpaced Server cases retain unequal-work scope; their ratios are not matched
speedups or regressions. No balanced robotics performance claim follows.
Never run comparative timings during builds.

`prepare_io` reconstructs reset resources outside raw ticking; `update` combines
both phases. CPU endpoints own scalar data or leases, not native publishers,
queries or tokens. Tests cover actual rate retirement, canceled setup, bounded
response discard, queued shutdown and IMU buffer release after last removal and
both committed-reset failures. Preserve original physical, frame, epoch, identity,
sequence and Weak-lease assertions. Complete ACT-01 absence and stopped-peer
pressure remain open. Raw ticking still accesses shared Zenoh state, some absent
families still receive dispatch, partial declaration failure needs a capture
barrier probe. A bounded follow-up drops the CPU IMU streams before joining IO
on shutdown. Its new retained-Server test reproduces the old live leases and
passes after the fix, alongside all seven shutdown controls; ordinary strict
staged and hosted qualification remain required. This does not retire the
simulation or establish full absent-system behavior.
See `artifacts/act01-source-audit/review.md`; source findings are not reproduced
races or timing failures.

Coverage retains full JSON and LCOV before selecting complete records from the
owned source set. Run the generated-binding guard on full JSON first. Never use
LLVM source filtering to hide lexical build-script aliases, merge duplicate
owned records, edit counters or remove uncovered entries. The normal strict
missing/duplicate/changed-line and >95% checks stay mandatory.

The current next tasks and accepted revision are at the top of this file.
Historical evidence below retains its original revision and scope.

## Earlier acceptance publication

The CPU IMU shutdown-storage follow-up is published at `1480725`. Its normal
staged hook and independent audit passed 784 headless, 435 renderer and 217
tooling tests, strict per-crate coverage and the new changed executable line.
Its own exact hosted run `41252543726` failed the renderer worker exit bound.
Keep the first capacity-stopped
attempt separate from the complete retry; only inactive compiler caches were
reclaimed, preserving source, executables and raw evidence.

The new `docs/acceptance/P2_ZENOH.md`, fifteen-row matrix, dependency receipt and
matched-stream observations publish the selected Zenoh milestone's evidence.
Their implementation witness remains `5870373`; their own publication commit
must pass normal staged and exact hosted/source checks before Root advances P2.
Historical P2 records remain unchanged. Do not keep this selected milestone open
for unrelated broad ACT-01 or balanced robotics requirements, and do not extend
its acceptance to those packets.

The schema source policy retains original minimal-inventory provenance at
`a39a945` and identifies the equivalent public release at `d030d06`. Canonical
proto bytes are unchanged; new generated build evidence is required for the
updated policy. SDF `486b3e0` passed its independent hosted/source qualification,
and the reviewed integration now pins that actual Git revision. Six new Scene
controls verify legacy gravity/reset on all backends, magnetic-field refusal and
separate URDF boundaries; the existing 234-test Scene matrix passed in the
isolated producer-readiness check. No private Cargo path override is adopted.
The combined actual-Git integration passes those six controls and the seven
terminal-shutdown controls; c3 now passes the full integrated qualification.

The integrated terminal-frame guard is described in
`docs/TERMINAL_SHUTDOWN.md`. It preserves direct Session-close behavior while
rejecting frames after explicit application shutdown. Independent review
approved the exact implementation and two documented stronger assertion
transitions. Its combined normal staged and exact hosted gates passed at c3.

The earlier renderer timeout is retained, without a claimed root cause.
Diagnostic branches75b73d and b46c469 did not reproduce it. The reviewed c3
peer-loss bound passed controlled cleanup stalls and complete local/hosted gates;
see RENDERING.md. Full demand-only activation, general pressure, broader physics
and sensors remain open. Preserve direct Session-close behavior separately from
explicit application shutdown and do not compare timings during builds.

## Owner correction: Zenoh and schema-only Messages

D28/D29 supersede the transport and Messages runtime rewrite. Stop all old
ZeroMQ parity and message codec/runtime/CLI work. Use upstream Zenoh directly in
gz-sim-rs; gz-msgs-rs retains only canonical proto sources and required schema
metadata/licensing. Generate bindings in each consumer. See
ZENOH_SCHEMA_MIGRATION.md for the staged transition and acceptance criteria.
Historical ZeroMQ/runtime results retain their original scope; they do not
qualify Zenoh. Current consumers use the independently qualified Zenoh path.
SDF, Fuel, physics, rendering and built-in activation work continue.

## Owner correction: built-in systems

D26 supersedes the earlier system C ABI/host plan. All simulation systems live in
gz-sim-rs and participate only when loaded world/models need them. Keep legacy SDF
plugin metadata/configuration mapping; do not load system libraries. Native engine
FFI remains allowed. ACT-01 defines activation, last-consumer teardown and zero idle
system resource/work evidence.

The current implementation does not yet meet the full absence requirement:
generic input/physics dispatch and shared IO reconciliation still need work.
Optional CPU force ingress now retires after its final consumer and loaded reset
reconstructs it from Scene metadata; no old CPU worker/template is retained. Joint preflight and committed/reset sampling now follow loaded
consumer demand; see JOINT_SYSTEM_LIFECYCLE.md for last-consumer retirement,
loaded reset, cross-family observer handling and explicit failure scope.
Persistent effort application now follows configured or typed controller demand;
see JOINT_FORCE_SYSTEM_LIFECYCLE.md for zero-command and reset semantics.
The bounded IMU Scene lifecycle now installs sampling/reset callbacks only for
live sensors and retires them and interest state after the final consumer; see
IMU_SYSTEM_LIFECYCLE.md for reset, failure and remaining server-worker scope.
Audit all implemented families and fix these through reviewed lifecycle slices.
Do not claim the previous bounded-force acceptance proves zero-cost absence.

## History

- 2026-10-24 to 2026-10-27: a ChatGPT/Codex coordinator ran P0-P2 with one
  agent per repository. Its local evidence is in ignored `artifacts/` and its
  transcripts are under `~/.codex/sessions`; nothing there is required to
  continue.
- 2026-10-27 to 2026-10-29: a Claude coordinator took over. It landed the
  networked CLI, closed P2, added the shared physics backend contract and the
  native DART and MuJoCo adapters. The commits are listed below.

## Current P3 follow-up

Canonical Physics world metadata now maps to the existing built-in physics
system; see WORLD_SYSTEM_CONFIGURATION.md. Explicit backend selection is retained,
with no dynamic system loader or duplicate callback installation. A bounded canonical engine filename is accepted under that explicit override,
so native input fixtures now load without removing the record. Automatic engine
selection remains open. Bounded moving-parent support is described below;
the complete original ApplyJointForce fixture is still unqualified. Bounded static local +Z planes
now load through DART; see SCENE_PLANES.md for the default-inertia, centered
collision and backend restrictions and retained native finite-footprint behavior.

The joint-effort API adds `DynamicBackend::add_joint_effort`; its DART world-hinge
implementation was accepted at 1982aa0. The current extension supports MuJoCo's
world hinges through its native generalized-force array. Commands are deferred by joint ID, consumed only
by a successful advancing step, and retired with the joint/body or either reset.
DART checks its temporary world-wrench overlay before native mutation; MuJoCo
resolves current DOF addresses after model settlement. Public lifecycle/physical
controls and both C++ pulse oracles are documented in
[JOINT_EFFORT.md](JOINT_EFFORT.md). Rapier's extension covers world/fixed-parent
impulse and reduced hinges; its moving-parent effort remains open. DART's bounded
extension is recorded below. Finite limits remain open. Required staged/hosted evidence
and accepted status are recorded on the public dashboard.

Loaded scene input now exposes `queue_joint_effort` by public entity ID. It
checks the loaded joint's bindings and original backend owner before forwarding
to the existing deferred backend queue. No shadow queue or stepping system is
added. Scene controls exercise all three dynamic engines, pause/reset/retirement,
invalid admission and native pulse-rate parity. Core runtime faults are terminal;
there is no same-simulation retry contract. Programmatic `set_joint_force` now
stores a latest persistent value in a Bevy component and adds it once per
advancing step after preflight. Pure backend scalar validation preserves old
settings and pending inputs on admission failure. Both resets clear it. Recognized
ApplyJointForce model configuration now installs zero commands and captures only
immutable metadata for loaded reset. Native text probes require ASCII whitespace
rules and rejection of split lexical values; see APPLY_JOINT_FORCE_CONFIGURATION.md.
Bounded Double topic input now normalizes/remaps endpoints, fans out shared
subscriptions and updates these commands within a finite ingress budget. Prepared
workers survive loaded reset while old receiver queues and command values retire;
see JOINT_FORCE_TRANSPORT.md for wire epoch and failure limits. These primitives
do not justify complete original-system credit. Continue with the missing native
fixture capabilities and broader articulation/limit domains after qualification.

The DART runtime guard also checks active angular rows in contact-bearing groups
against the existing native-coordinate inverse-response ceiling. This addresses
implicit default-limit activation with tiny inertia; it does not enable finite
limits or change solver settings. See [DART_JOINT_BOUNDARY.md](DART_JOINT_BOUNDARY.md).

Finite position limits remain open. Preserve the existing strong first-crossing
controls rather than publishing a direct maintained `set_limits` wrapper:

- Rapier impulse angles lose winding; near-PI limits can miss the excluded arc.
  Reduced coordinates preserve winding, but Rapier 0.36.1 only enables its internal
  limit row once the coordinate is already outside the interval. At 12 rad/s and
  1 ms, an upper bound of 0.0106 rad is crossed to 0.012 rad on the first step,
  beyond the retained 1e-3 tolerance. Removing implicit damping and adding solver iterations did not
  resolve that predicate. Unbounded reduced construction is accepted at 50b28e5;
  keep existing scene solver selection unchanged.
- DART 6.13.2 tiny contacting bodies with finite limits can abort inside ODE.
  Private equilibration experiments address selected failures and have atomic
  failure/retry controls, but physical accuracy, native assertion domains and
  cost remain unqualified. Dantzig freezes friction bounds during its pivot
  sequence; a final-normal cone defect is not its frozen boxed-LCP residual.
  Do not invent a cutoff, change tolerances/settings, or claim broad safety
  from successful subprocess exits alone.

## Where things stand

| Commit | Change |
| --- | --- |
| `a549c52` | `gz-sim-rs world.sdf --server [--run]`: networked headless server, real C++ client tests |
| `1cc54cc` | World names sanitized like native `TopicUtils::AsValidTopic` |
| `d58223d` | P2 and SLICE-01 closed for the selected headless world ([acceptance/P2.md](acceptance/P2.md)) |
| `d35b3bf` | `gz_physics::DynamicBackend` contract, type-erased `PhysicsBackend`, `install` reporting via StepFault |
| `8e9cf52` | Rapier and the scene moved onto that contract |
| `074c29b` | Native MuJoCo 3.11.0 adapter ([MUJOCO.md](MUJOCO.md)), `--physics-engine mujoco` |
| `85dd4ab` | Native DART 6.13.2 adapter ([DART.md](DART.md)), `--physics-engine dart` |
| `ca37d5c` | Real C++ headless interoperability qualified on all four retained engines |
| `26ca44f` | Programmatic Rapier body-to-body revolutes |
| `9e8e056` | Initial two-link SDF articulation and endpoint/model lifecycle validation |
| `675c4e0` | Programmatic Rapier contact-exclusion groups and lifecycle evidence |

- The commits listed above passed the staged pre-commit gate and hosted CI. The [dashboard](https://gazebosim-rs.github.io/rewrite-progress/)
  shows P2 complete and P3 in progress.
- Engines: Rapier (full f64), DART, MuJoCo and TPE are selectable in the scene
  and CLI; the CLI also accepts the upstream `gz-physics-*-plugin` names.
  DART and MuJoCo match the native gz-physics plugins bit for bit or within
  1e-14 on their parity fixtures. Bullet stays rejected (decision D04).
- Rapier now has body-to-body revolutes and construction-time contact exclusion
  groups. Its scene loader supports top-level dynamic acyclic local-link revolute
  graphs, including world roots and disconnected components. See
  [ARTICULATED_SCENES.md](ARTICULATED_SCENES.md) for exact boundaries and evidence.
  DART's bounded world-rooted parent/child backend is accepted at `f60d3a3`;
  its scene integration at `506f94a` passed full local and hosted gates.
  Jacobian-cache repair `c5f73a1` is also qualified. MuJoCo body-joint
  construction remains unsupported.
- P3 remains open: native larger articulations, joint limits and damping,
  contact properties, meshes/planes/heightmaps, and conformance against
  gz-physics `test/common_test` for each backend. No general speedup is claimed.

## Owner decisions made during this handoff

Recorded here and, where they are product decisions, in DECISIONS.md.

- Process: keep the real gates and drop the earlier ceremony (see AGENTS.md,
  "Owner guidance (2026-10-27)").
- Autonomy: commit and push reviewed work to main, update the dashboard, and
  edit sdformat-rs, gz-msgs-rs and gz-transport-rs directly. Never edit
  gz-fuel-tools-rs. Show the owner a list before deleting non-regenerable files.
- P2 was closed for the selected world rather than waiting for every broad
  packet; broad packets keep their own exit criteria.
- D24: MuJoCo is built from upstream's vendored commit, not a prebuilt release.
- D25: DART uses the packaged OSRF build, whose assertions abort on
  ill-conditioned input. The adapter rejects such input with documented limits
  (minimum mass 1e-5 kg, anisotropy cap, conditioning rule) instead of linking
  a different DART. Where native DART itself diverges, the adapter returns a
  typed error; native Gazebo aborts.

Before changing the SDF consumer pin, audit newly admitted coupling metadata.
The current pin rejects mimic declarations before scene loading; a future
mimic-capable producer needs an explicit scene rejection until a solver implements
that coupling. Declared child frames must remain distinct from physical bodies.

## Local setup

- Native interop tests and the commit hook need a C++ transport reference with
  a sibling `environment.json`. This repo's `artifacts/native-reference/install`
  lacks it; use `GZ_REFERENCE_PREFIX=<gz-transport-rs>/artifacts/reference/install`
  or rebuild with `python3 scripts/bootstrap-native.py`.
- MuJoCo: `python3 scripts/bootstrap-physics.py`, then
  `MUJOCO_PREFIX=<repo>/artifacts/physics-reference/mujoco`.
- DART: `libdart6.13-dev libdart6.13-collision-ode-dev` from the OSRF apt
  repository (see `.github/workflows/ci.yml`). On this machine ROS Jazzy's
  vendored DART is on `LD_LIBRARY_PATH`; binaries are pinned with DT_RPATH and
  `DartBackend::new` refuses a different library, so that is handled.
- Commit with all three variables set; the hook runs the full gate (about 10
  minutes locally, 8 on hosted CI).
- Disk: on 2026-10-27, about 170 GB of regenerable build caches and old scratch
  were deleted with the owner's approval. About 190 old worktrees with
  uncommitted ChatGPT drafts remain under each repository's `artifacts/`, plus
  unpushed branches `work/reset-identities` (gz-sim-rs) and `socket2-fuel-compat`
  (gz-transport-rs). They are believed superseded but unaudited; archive before
  deleting, and only with the owner's approval.
- The DART and MuJoCo author branches (`worktree-agent-a5488f384295773d9`,
  `worktree-agent-a542e16c2bd136546`, worktrees under `.claude/worktrees/`) were
  squash-merged into `85dd4ab` and `074c29b`. They can be removed.

## Known tooling issues (2026-10-29)

- Shared owner guidance is installed in every coordinator-managed repository.
  SDF's existing hook still requires independent review for protected files;
  reviewers satisfy that check without adding frozen proposals or build archives.
- gz-transport-rs's hook fails in a shell with ROS Jazzy sourced: its native
    C++ test binaries in `gz-transport/build/bin` resolve `libgz-transport` and
    zenoh from `LD_LIBRARY_PATH` and fail to load, so the reference-ledger check
    reports every case missing. Its C++ peer build also needs
    `GZ_REFERENCE_PREFIX=<gz-transport-rs>/artifacts/reference/install`. Fix the
    environment (or the hook's library path handling) before the next
    transport commit.

## Next

1. DART server reset comparisons now cover both DART and Rapier, preserving
   TPE and all original assertions. The server pins the native library with
   DEP_DART_LIBDIR and tests it under a shadowing library path. Author and
   independent focused runs and the mandatory staged hook passed in published
   `c9449ac`; exact hosted run 40871693297 passed.
   Published `9cbb7cb` rejects MuJoCo hinge removal above mjMAXVAL and preserves
   projected velocity when removing an unstepped hinge. Both regressions were
   reproduced before the fix; independent review and the full staged gate passed.
   Exact hosted run 40873469093 passed. Published `0312c3c` also exercises real
   contact arena exhaustion through backend steps and reset; its exact hosted
   run 40875528990 passed. Unchanged real C++ CLI/server assertions are now
   qualified on DART and MuJoCo at `ca37d5c`; see NATIVE_BACKEND_INTEROP.md.
2. MuJoCo follow-ups (low, not reachable through the scene today): extreme
   inertial-centre offsets are admitted but fault the first step; the hinged load
   comparison against the native plugin is not like-for-like (the plugin adds
   actuators). Contact exhaustion now also has a real backend-step regression.
3. P3 capabilities, one reviewed slice at a time: general articulation (model
   joint trees, not only world revolutes), joint limits/damping/effort, contact
   surface properties, mesh/plane/heightmap shapes. Port the matching
   gz-physics `test/common_test` cases per backend; the upstream binaries are
   built in `gz-physics/build/bin/COMMON_TEST_*` and take a plugin path.
4. Continue P4 in parallel with P3, preserving the headless dependency boundary.

## Latest bounded qualifications

`f60d3a3` passed independent review, the unchanged full staged gate and exact
hosted run `41168474334`. Downloaded evidence matches all 60 source hashes and
25 build/check inputs against that commit, plus both local profile summaries.
This accepts a DART world hinge with one dynamic revolute child, including
coupled effort, reset and guarded topology changes. External articulation
contacts, active multibody limit rows and larger graphs remain unsupported.
The free-root follow-up is described below. The native oracle records its verified hybrid SDK provenance;
it is not a clean homogeneous reference build or performance qualification.

Scene commit `506f94a` preserves SDF declaration identities while constructing
the DART world hinge first, and removes physical leaves before parents. Its
four new analytical controls and existing body-joint assertions passed independent
review. Full local headless and renderer gates passed, including 60/60 changed
scene lines. Exact hosted run `41169945049` passed, with both downloaded profiles and
source hashes matching local evidence. The original
ApplyJointForce world has a free root and ground contact; adding a world hinge
would create a different fixture and earns no original-system completion.

Main `f19d195` accepts the backend extension from a free-root serial pair to
one isolated dynamic terminal child (eight generalized coordinates). Its complete
normal hook passed in 616 seconds after correcting the execution permissions;
the first sandboxed attempt failed unchanged TCP setup before DART tests ran.
Exact hosted run `41328864338` and independent official source/profile/native
audit passed: 854 headless, 520 renderer and 237 tooling checks. The earlier
151-test DART package/native evidence remains distinct from full qualification.
The stronger exact-input positive preserves all three old body snapshots,
identifiers and queued inputs; five other rejection rows remain unchanged.
Any contact involving a tree larger than two bodies is refused before contact
response. Existing pair/static-contact and active-limit mathematics remain
unchanged. No Scene, B03, multiple-collision or performance acceptance follows.
See [the backend record](../crates/gz-physics-dart/docs/FREE_PARENT.md).

The bounded free-root follow-up supports one dynamic parent and one revolute
child, with seven generalized coordinates. It preserves the extraction-cache
repair below and the existing guards on external contacts and active limits.
The exact previously rejected Scene XML now checks both endpoints, free-parent
translation and rotation under internal effort, sampling, loaded reset and
removal. Independent review regenerated all 34 native observations and approved
only three explicit rejection-to-positive transitions. Full unfiltered backend
and Scene body-joint tests pass. Ordinary strict staged and exact hosted evidence
must be checked before marking this slice accepted on the dashboard. See
[FREE_PARENT.md](../crates/gz-physics-dart/docs/FREE_PARENT.md). The original
ground-contact ApplyJointForce case is now covered by the bounded follow-up below.

The static-contact follow-up admits only a two-body free-root pair against static
geometry. It preserves world-root, dynamic external-contact and active-limit
refusals; the broader world-root experiment reproduced an upstream native abort.
Independent numerical/native review approved the conservative all-coordinate
response bound, valid anisotropy and condition-boundary controls, and exactly two
old static-contact assertion transitions. The unchanged original force-command
world passes fresh physical parity and a real C++ publisher to the Rust server.
Its single Linux original case is traced in the ledger with explicit API/component
translations and retained native baseline evidence. This does not complete the
system family or zero-cost absence. See
[APPLY_JOINT_FORCE_ORIGINAL.md](APPLY_JOINT_FORCE_ORIGINAL.md) and
[ARTICULATED_CONTACTS.md](../crates/gz-physics-dart/docs/ARTICULATED_CONTACTS.md).
Ordinary strict staged and exact hosted qualification remain mandatory.

Follow-up review found a DART extraction bug: rejected attachments and leaf
retirement could leave surviving native Jacobian caches zero while Rust state
appeared unchanged. Repair `c5f73a1` invalidates those caches at three structural
boundaries, preserving the admission domain and solver settings. Independent
clean-baseline regressions, 119 standard/native tests, native bridge coverage
498/521 lines and all nine changed executable lines passed. The full staged gate
and exact hosted run `41173269592` passed; downloaded Rust profiles and native
source hashes match the commit. See [TOPOLOGY_CACHE.md](../crates/gz-physics-dart/docs/TOPOLOGY_CACHE.md).

Canonical SDF camera snapshots are accepted at `5669d79`. An optional CPU-only
camera-output crate adds explicit acquisition and bounded publication through
the real Bevy worker, with actual C++ Image/CameraInfo delivery on software
Vulkan. Its private renderer profile and independent software-worker replay passed;
full headless/staged/exact-hosted qualification passed at c3. Keep real
buffered-frame reset/removal/replacement controls and
independent protocol-error tests; those peers do not inject physical GPU faults.
Automatic Server camera management, native image parity, physical GPU qualification and full
sensor-family migration remain open. Read the tracker and retained evidence
before advancing these statuses.

The accepted single-camera follow-up adds application-owned admission from loaded
metadata and the shared integer SensorSchedule. It gates advancing committed
frames, demand and the single acquisition slot, invalidates stale bindings and
requires explicit worker retirement outside stepping. Independent source and
retained-evidence review approved the slice and its observed-demand race fix.
It adds no automatic Server callbacks or complete sensor-manager claim. Full
integrated coverage, staged and exact hosted checks passed at c3; the original
worker timeout cause remains unproven. See [CAMERA_SCHEDULING.md](CAMERA_SCHEDULING.md).

## Accepted continuation and current serial assembly work (2026-11-02)

Simulator Main `e55356c8d6a80246d67b5bceb37a93faf1508e6b` accepts bounded
DART compound offset COM and Scene link-origin velocity conversion after the
mandatory local gate, exact hosted run `41383974766` and independent official
source/profile/native audit. This includes the earlier bounded compound
collisions and free-root serial chain paths. It does not accept B03 motion or
matched performance.

Producer Main `f0d05146300e31237d0cf0a6024fdfd9b0ef4226` now accepts scoped-root
joint semantic-pose evaluation after its mandatory local gate, exact hosted run
`41438358157` and independent official audit. It includes the accepted radian
constructor and bounded automatic-inertia corrections. Simulator Main still
pins `13adde94`; producer acceptance does not qualify a changed simulator.

Root's active consumer slice is `work/producer-joint-pose-adoption`. It preserves
the failed `05035e96` constructor-adoption source and references, updates only
the producer revision/lock source to `f0d05146`, and selects the existing
`Sdf::joint_pose(name, "")` API for serial child-frame input. No package versions,
dependency edges, numerical bounds, backend guards or world-frame metadata change.
See [NATIVE_JOINT_POSE_ADOPTION.md](NATIVE_JOINT_POSE_ADOPTION.md).

The prior constructor-adoption gate failed after 359 seconds at the strict
fresh-native B03 comparison. Two complete audited diagnostics reproduced their
respective native and Rust observations exactly. Initial rotations match
bitwise, but small translations and trajectories differ. The current Rust
comparison first exceeds `1e-12` at step 35 and reaches a transient difference
of about 0.02225 at step 94. Native joint semantic-pose translations retain two
scoped-root paths; the old common-ancestor query discarded that arithmetic.
This input difference motivates the correction without proving a dynamics cause.
Historical cached and fresh native references remain distinct and unchanged.
No consumer adoption, B03 motion or matched performance is accepted yet.

Finish implementation, regression code, documentation and independent source
review before consolidated verification. Batch corrections for actual failures;
do not repeat tests or lint after individual edits. Preserve Fuel and all
historical evidence.

### Serial lifecycle identity inventory correction

The `ad718fbf` normal gate failed after 367 seconds without a capacity stop.
The fresh-native B03 motion test passed all unchanged assertions. The later
serial lifecycle helper panicked while iterating internal Bevy entities as if
every entity had a public Scene name and ID. Its external-contact refusal case
passed; later suites and final coverage were not reached. This partial result
does not qualify the feature or the new C++ B03 interoperability case.

Root preserves the failed tree and corrects only that helper in
`work/serial-public-identities`. Public names and IDs must occur together, names
must remain unique, and entities with neither must have no body, joint or world
pose. All original ten-public-identity, ordering, lifecycle, motion, reset and
removal assertions remain. No production code or numerical bounds change.

### B03 world-light identity inventory correction

The `f4db5841` normal gate failed after 368 seconds without a capacity stop.
The serial lifecycle correction passed; the next newly reached B03 world-light
reset test contained the same invalid all-ECS-name assumption in both its initial
and reset inventories. Root preserves that failed tree and batches both loops
into a helper with the reviewed name/ID pairing, uniqueness and no-physical-state
checks. All original light, seven-body, four-joint, fifteen-collision, metadata,
identity equality and reset assertions remain unchanged. Source review also
checks the other new Scene/Server tests for the same assumption before the next
affected mandatory pass. Production code, fixtures and numerical bounds remain
unchanged; later suites and final coverage are still unqualified.
