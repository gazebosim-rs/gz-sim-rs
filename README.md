## Current development: M86/M87 isolated chord correction Source complete

Read [M86](docs/M86_DELIVERY.md) and [M87](docs/M87_DELIVERY.md) for the closed domains.

M86 compound state edits and M87 passive revolute composition share one bounded
D31 cycle. The second full normal failed Root/Git 1 after 1399.021763 seconds:
1918 headless passes, one failure and two ignored cases. Fourteen of 15 new
cases passed, including every other M86 control, both private controls and all
eight M87 cases. Renderer and strict qualification were not reached. Preserve
frozen tree `2d65c793`, failure `b1619de7` and its full packet. The first failure
`865ca8d0` at tree `ca8a0614` remains failed: 1915 headless passes, four failures,
two ignored and 11 of 15 new cases passing. Neither run created a qualified commit.

The remaining failure is a test-ray placement error. For four or sixteen children,
the old cylinder chord ray starts inside the adjacent box, so the compound's
correct nearest solid hit is zero. The isolated one-file correction casts along
world Z at radial Y offset 0.3, preserving the same cylinder, distance 3.6,
maximum 5 and tolerance 1e-11. It retains the old ray as an exact zero-hit
countercontrol for four/sixteen children and a 3.6 control for two. The complete
remaining case and later oracles were independently reviewed; Scope's exact
patch review reconstructs the full frozen file by inverse. No production code,
geometry, other case, tolerance or quality gate changes.

Confirmed Source census remains 1799 files, 14 owned paths, four new paths,
three production paths and 93 cumulative paths: 15 unique H/R Rust declarations,
all overlapping, zero Python, finite/CLI 47/21. These are Source obligations,
not passes. Implementation and the bounded independent Source review are complete;
Root reviews this eight-doc checkpoint before one affected single-file formatter
and one consolidated mandatory full retry. Reuse `b075001c` only for unchanged
`initial.rs`, and `78be4b89` only for its four unchanged Rust copies. Keep both
original formatter records; claim no combined formatter union. Current one-file
formatter, tests, lint and retry are UNRUN; final tree, commit and outcomes are NULL.

The genuine parent is M85 `22cffebd`, tree `bf830127`, parent M84 `ba2d85ee`,
own-qualified but unaccepted. Its corrected normal/DATA proved Scene 15/15 in
both profiles; the first 14/15 failure stays preserved. Own hosted run
44883777137 attempt 1 passed, original collection returned zero with 30
reaped-zero requests, and independent official DATA passed (`300b40da` /
`bab7fc31`). Four original RAW groups 442/734/57/168 were compared once; nine
strict bases and the actual zero CI base were qualified. M84 also completed its
own qualification. No result or acceptance transfers to this shared successor.

All original own, predecessor, M77 browser and owner-confirmed shutdown gates
remain mandatory. Keep no-expiry demo 86363 live; no stop or restart is selected.
Accepted phases/slices stay 76/202, catalog 87, finite/CLI 47/21 and Latest M70's
original 41-example package unchanged. Preserve Fuel, both failed candidates
and all historical evidence. DART/MuJoCo stay retired. Root alone releases serial
heavy operations. The proposed next development wave remains unselected.

## Current development: M77 installed camera preview

The selected [M77 contract](docs/M77_DELIVERY.md) retains its 47-world/four-role scope.

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

[M75](docs/M75_DELIVERY.md) selects camera masks and visual flags for existing 64 by 64 RGB/depth
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

[M74](docs/M74_DELIVERY.md) adds finite, representable signed intrinsic skew to the existing
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

[M73](docs/M73_DELIVERY.md) admits explicit zero-translation SDF projection for the existing
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

[M72](docs/M72_DELIVERY.md) admits explicit finite zero-skew SDF focal lengths and principal
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

## Current development: M70 camera connectivity correction

D33 retirement is published on Main `8f13dc7446be4fded424b1eba83df4c8253d0cef`.
Rapier f64 and TPE remain; DART and MuJoCo stay retired. M1–M69 and 202 slices
remain accepted. M65 remains Latest with 35 examples.

The frozen M70 candidate `597ac3ddccf150cc1c45a60c3c72b838f943093a` passed its
full local normal and strict coverage gates, source-bound release builds,
package audit, clean Ubuntu installation and 41 installed finite examples.
Its hosted run failed before product checks at three Ubuntu snapshot HTTP 401
responses. The first installed camera trial then failed before its first camera
packet barrier; its canonical Clock and Scene reached exactly 1 ms.

Source review found that the packaged camera peer clears its listener while its
configuration supplies no connection and disables multicast. This isolated
successor adds an explicit CLI path sharing the world's upstream Zenoh session.
Existing separate-session camera constructors keep their configuration semantics.
Camera reset, removal and teardown must release their publishers and workers
without closing the world session. No upstream Zenoh patch, transport framework,
protocol, renderer, physics, dependency or SDK change is selected.

The complete routing/lifecycle Source passed independent review. Its first normal
run failed after 541.339 seconds: Clippy and 466 Python checks passed, and all 15
CLI camera cases passed, but the auxiliary qualifier still registered 14 cases.
This isolated successor corrects that closed registry and its regression controls;
the failed tree and logs remain frozen. Batch the correction and review before one
full affected normal/strict retry. No test assertions or gates are relaxed.

The operational harness must also let the existing native 20-second deadline
finish instead of cancelling at 10 seconds. Keep the exact first 1 ms sample,
numerical assertions and all three depth trials plus separate RGB proof.
Reuse unchanged evidence only where its bindings permit; preserve all original
failures and frozen trees. Do not rerun completed installation or GPU provisioning
without an actual changed prerequisite.

Fresh source-bound package, installation and camera evidence remain required for
this changed candidate. Browser access to `http://127.0.0.1:8082` still needs the
explicit owner approval required by automatic review. Hosted CI, official audit
and release publication remain mandatory before M70 acceptance. No check has run
on this successor; preparation is not qualification.

Earlier checkpoints below retain their historical scope.

## Current owner priority: D33 backend retirement

The owner cancelled DART and MuJoCo on 2026-11-17. Retain Rapier f64 and TPE;
see [the retirement contract](docs/BACKEND_RETIREMENT.md). The isolated removal
is undergoing its own review and required checks; cancellation is not completion.
M1–M69 and 202 reviewed slices are accepted. M70 remains frozen and unaccepted,
with its separate browser-origin approval pending. No M70 package/runtime Python
changes are included here. Earlier development checkpoints below are historical.

<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
## Current development: M69 two calibrated depth cameras

M1–M68 and 201 reviewed slices are accepted at Main
`24ea62519d0a671f789e6b3fc98bd2b72de18ae2`. M68 passed its full normal,
strict coverage, independent local DATA, [exact hosted CI](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/44077798140)
attempt 1, durable-original collection, official DATA and Main readback.
The [acceptance record](docs/M68_ACCEPTANCE.md) keeps the repaired signed-input
lineage and historical failures distinct. Its nine-body metadata archive is
published; the fixed stager precedes unchanged signature, hash, size and package
record verification. M65 remains public Latest with 35 installed examples and
the unchanged viewer bytes and budgets.

[M69](docs/M69_DELIVERY.md) adds exactly two independently demanded calibrated
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
continue in parallel. See [M60_DELIVERY.md](docs/M60_DELIVERY.md).

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
independent retained DATA audit. See [M59_DELIVERY.md](docs/M59_DELIVERY.md).
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

Read [the M57 contract](docs/M57_DELIVERY.md) and retained evidence under
`artifacts/m57-delivery`.

Earlier checkpoints below retain their historical scope.


## Current development: independent odometry publishers

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
See [the M56 delivery contract](docs/M56_DELIVERY.md) for scope and remaining evidence.

Earlier checkpoints below retain their historical scope.

# Gazebo in Rust

This repository contains the implementation plan and the first Rust building
blocks for `gazebosim-rs/gz-sim-rs`: a Bevy ECS simulation clock and schedule,
a full-f64 Rapier backend, a noise-free IMU model, and an integrated SDF scene
constructor for a limited set of shapes and sensor configurations.
It is not yet a complete simulator. See [implementation status](docs/STATUS.md)
for tested behavior, coverage, reference evidence, and remaining work.

Copyright (C) 2026 Open Source Robotics Foundation. Licensed under
[Apache License 2.0](LICENSE); see [NOTICE](NOTICE). Imported material retains
its original license and attribution notices.

Run `python3 scripts/verify.py` for formatting, lint, tests, and strict per-crate
coverage checks. Install the commit hook with `git config core.hooksPath .githooks`;
it checks an isolated copy of the staged tree. Rust 1.97.1 is pinned.
The [scene contract](docs/SCENE.md) describes the working SDF-to-ECS path and its
explicit limitations. Run `cargo test --locked -p gz-sim-scene` to exercise it.

Run the headless scene example with:

```sh
cargo run --locked -p gz-sim-cli -- crates/gz-sim-scene/tests/fixtures/fall.sdf --iterations 100
```

The new `gz-sim-rs` command prints simulation time and sorted link positions.
It is limited to the scene contract above; legacy CLI compatibility remains open.
The [live progress dashboard](https://gazebosim-rs.github.io/rewrite-progress/)
tracks accepted increments, current development, remaining scope and evidence
across the four target repositories. Repository agents publish their own updates.

## Current delivery

M1–M42 are accepted at qualified Main `73e03b9f`; see
[M41/M42 acceptance](docs/M41_M42_ACCEPTANCE.md). The latest installed preview is
[m40-preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m40-preview),
with its matching [installation](docs/M40_INSTALL.md) and
[capability](docs/M40_CAPABILITIES.md) guides.

[M43/M44](docs/M43_M44_DELIVERY.md) are in development: named IMU orientation
references and an eighteen-example preview with the joint-command helper. Their
source and native component evidence do not qualify a new installed release.
Fresh Rust, hosted, package, installation and browser proofs remain required.

## Historical delivery notes

M1–M20 runtimes and the bounded distribution are accepted. Download the
[Ubuntu 24.04 x86_64 integrated preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m20-preview)
and its checksum, then follow the included installation guide. It packages the
native-free Rapier server, maintained browser and finite robot-command helper.
See [M20 evidence and limitations](docs/M20_ACCEPTANCE.md), the
[installation guide](docs/M20_INSTALL.md), [capability matrix](docs/M20_CAPABILITIES.md)
and [delivery plan](docs/DELIVERY_PLAN.md). This preview does not complete the
rewrite or qualify unsupported platforms/features. The accepted
[local recording/replay contract](docs/M11_RECORDING_REPLAY.md) remains retained.
M13 ideal altimeter and M14 local STL meshes are accepted at qualified Main
`2f32bc7e`; see the [feature release and actual receipt](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m13-m14-features).
M15 ideal air pressure and M16 local OBJ are accepted at qualified Main
`5f53c92e`; see the [feature release and actual receipt](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m15-m16-features).
M17 magnetometer and M18 joint-state publication are included in the accepted
[M19 installed preview](docs/M19_ACCEPTANCE.md), with copied local OBJ assets,
actual non-root installation/browser proof and explicit loopback HTTP 8082.
See its [installation guide](docs/M19_INSTALL.md) and
[limits](docs/M19_CAPABILITIES.md). Current [M20](docs/M20_DELIVERY.md) adds an
explicit existing lifecycle/publication-credit choice and is published as the
qualified M20 preview.
The next independent [M21/M22 Source wave](docs/M21_M22_DELIVERY.md) adds
loaded altimeter and air-pressure noise using the existing Gaussian library.
It passed local qualification at `36eb202c`; exact hosted qualification remains
pending. It is not included in the published M20 archive.
The following [M23/M24 source wave](docs/M23_M24_DELIVERY.md) adds
[magnetic noise](docs/M23_MAGNETOMETER_NOISE.md) and narrowly admitted
[CPU-lidar white noise](docs/M24_CPU_LIDAR_NOISE.md). Both source batches are
independently reviewed; their checks and acceptance remain pending.

[M21–M25](docs/M21_M25_DELIVERY.md) now qualify together in one new candidate,
including four direct finite noise examples and a fresh M25 installed preview.
All original checks and distribution criteria remain; source review is not a
qualified archive. M20 remains the checked download
until all parent, package, installation, browser and publication gates pass.

The design combines the responsibilities of `gz-sim`, `gz-sensors`, `gz-physics`,
and the necessary parts of `gz-math` in one Cargo workspace. Bevy supplies the ECS
and renderer. Existing Rust libraries replace general-purpose Gazebo utilities.
D33 retires DART and MuJoCo product backends. The simulator retains Rapier f64
and TPE. Bullet is removed. The replacement GUI is a separate application.

`sdformat-rs`, `gz-fuel-tools-rs`, and schema-only `gz-msgs-rs` remain
independent repositories in `gazebosim-rs`. Simulator consumers use upstream
Zenoh directly under D28/D29; the historical transport repository is retained,
not a target runtime. Fuel has another owner; this work does not modify it.

The owner has confirmed behavioral, file-format, protocol, and test compatibility
with new Rust APIs, and Linux first with headless and Bevy rendering support.
Simulation systems are built into gz-sim-rs and activated only by loaded world/model
requirements; unused systems have no scheduled work or owned runtime resources.
The Rust physics backend must use
a full-f64 solver; prioritize Rapier and validate it with probes. Balanced robotics
is the primary performance workload. Remaining proposals and validation choices are in
[the decision register](docs/DECISIONS.md). Review those before implementation.

Public repositories: [gz-sim-rs](https://github.com/gazebosim-rs/gz-sim-rs),
[sdformat-rs](https://github.com/gazebosim-rs/sdformat-rs),
[gz-msgs-rs](https://github.com/gazebosim-rs/gz-msgs-rs),
[gz-transport-rs](https://github.com/gazebosim-rs/gz-transport-rs), and the
independently owned [gz-fuel-tools-rs](https://github.com/gazebosim-rs/gz-fuel-tools-rs).

Start with [the implementation plan](docs/PLAN.md), then use these documents:

| Document | Purpose |
| --- | --- |
| [Agent instructions](AGENTS.md) | Mandatory implementation and writing rules |
| [Repository audit](docs/REPOSITORY_AUDIT.md) | What moves, what disappears, and why |
| [Architecture](docs/ARCHITECTURE.md) | ECS, time, physics, rendering, sensors, and built-in system activation |
| [Dependency decisions](docs/DEPENDENCIES.md) | Reuse candidates and semantic gaps |
| [Independent repositories](docs/EXTERNAL_REPOS.md) | Deliverables and integration contracts |
| [Test and coverage policy](docs/TESTING.md) | Existing tests, strict >95% coverage, pre-commit, and CI |
| [Performance plan](docs/PERFORMANCE.md) | Workloads, measurements, and proposed release budgets |
| [Implementation tasks](docs/IMPLEMENTATION_TASKS.md) | Dependency-ordered work packets and acceptance evidence |
| [Decisions](docs/DECISIONS.md) | Confirmed requirements, proposals, and open questions |
| [Baseline](planning/baseline.json) | Exact local upstream source commits |
| [Source inventory](planning/source-inventory.json) | Public headers, systems, and test-source candidates |
| [System assignments](planning/system-migration.json) | All 78 systems allocated to implementation families |

The owner selected upstream `main` branches. The baseline records local main
commits inspected on 2026-10-24 (America/Los_Angeles); P0 verifies their freshness
and compatibility before running the reference suite.
The source inventory is a starting list, not an executed test manifest.

The first implementation milestone establishes reproducible C++ reference runs,
dependency versions, test traceability, and enforced quality gates. A small
end-to-end simulation follows before broad feature migration.

### M71 development: an independent RGB/depth pair

The explicit mixed-pair path is under development; it is not an accepted release.
See [M71_DELIVERY.md](docs/M71_DELIVERY.md) for the source-only state and required
qualification. The existing scalar and homogeneous-pair paths remain unchanged.
The new fixture is intended for a renderer-enabled source build:

```sh
cargo build --locked -p gz-render-worker
cargo run --locked -p gz-sim-cli --features rgb-camera -- examples/m71_mixed_cameras.sdf --server --camera-worker "$PWD/target/debug/gz-render-worker" --camera-mixed-pair
```

The server starts paused. Consumers request the separate RGB and metric-depth
Image/CameraInfo topics; this mode provides no RGBD registration or synchronization.
No installed launcher or published archive gains this mode in M71.
