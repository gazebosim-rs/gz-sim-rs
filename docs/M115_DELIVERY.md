<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M115: runtime CPU-lidar update-rate control

M115 adds read/set access to existing CPU-lidar rates and the canonical
`gz.msgs.Double` to `gz.msgs.Empty` rate service. It reuses the existing IMU
policy, SampleSchedule and concrete Server IO/CPU handoff. The budget is at most
five working days, including independent review and every mandatory gate.

The source parent is `2419c92342dcd6f869c9832ba1d691b50a51a020`, tree
`c08baf47e466479d9eb359180280b809841d3490`. Its full normal and independent
local retained audit passed. Its exact review branch
`review/m114-organized-lidar-point-clouds` is published; hosted run
45412754337 attempt 1 passed; its independent original-artifact audit remains pending.
The parent is unaccepted. Root explicitly admitted isolated M115 Source work
during parent closeout under D31; this transfers no qualification. M115 has one successful final formatter after independent whole Source review:
1,874 Source files, 22 changed paths, six new files and ten Rust changes (four
production files). Its first full hook now has the preserved compilation failure described below.
There is no successful test, lint, build, coverage, CI, audit or commit result.
The corrected candidate will be sealed after complete Source review.

## Current coverage correction

The second full hook on frozen `b52683da` failed after 4,478.606 seconds at
headless Server changed-line coverage: 195/208, with a strict greater-than-95%
requirement. All 2,195 headless and 1,694 renderer tests passed, with two ignored
in each profile; 498 Python tests passed. All 22 original headless and 11 shared
renderer M115 controls were observed once, but the full normal failed and no
commit was made. There was no capacity stop. Preserve the complete partial
packet and both real failures.

The retained-data analysis covers every available crate/workspace line and
region metric and all 20 Source-base projections. It identifies this one gate
failure. Those analytical projections are not a new execution or qualification.
Thirteen executable changed lines were missed: ten are reachable; three generic
error alternatives cannot currently be returned by the MissingInitialState-only
Scene getters. Keep all three alternatives and the original denominator.

Five new headless private controls cover the complete reachable batch using
actual bare/loaded worlds, lawful ID repair, pure admission and exact typed
refusal, pending-original key retention, queued provider retirement and fresh
restart. Exact measured clocks, 4/cos(yaw) wall geometry and a healthy sample
continuation twin provide independent outcomes. Do not add production hooks or
mirror the implementation. No production logic or old test body changes.

The combined Source roster is 27 unique Rust declarations: 27 headless, 11
renderer/shared, 10 public and 17 private; Python method identities remain 498.
The additional five have not run. Finish the whole Source/docs/review batch
before one affected formatter and one unchanged full hook retry. Reuse prior
formatting only for physically byte-identical files; no failed or partial test
result supplies qualification. M114 hosted CI passed; its independent official
original-artifact audit and every own gate remain required before publication.

## First verification failure and bounded correction

The first complete hook on frozen `f1cceff6` exited 1 after 32.988 seconds.
All-target Clippy reported three E0004 errors in the retained seven-family
helpers `family_key`, `topic_mut` and `assert_family_refuses`: their exhaustive
matches did not name the new LidarRate variant. Tests and coverage were not
reached, no commit was made and no capacity guard fired. Keep all original logs,
packet and failure evidence. The isolated correction adds this variant to the
existing explicit outside-fixture panic alternatives; it does not expand or
weaken the old seven-family fixture, change assertions or alter production code.
The same 22 M115 regression identities remain. Complete the entire correction,
documentation and independent Source review before one affected formatter and
one unchanged full hook retry. The prior ten-file formatting evidence is reused
only for byte-identical files; it supplies no test or qualification result.

## Scene contract

`lidar_rate(&Simulation, id)` returns `LidarRateInfo { sdf_max_hz, rate_hz }`.
It requires the real world-local identity and private Lidar component; public
publication metadata alone is insufficient. `set_lidar_rate(&mut Simulation,
id, requested_hz)` returns the existing `RateChange::Applied` or `Ignored`, or
`LidarRateError`. Clock validation precedes identity lookup in the setter.
The getter is read-only. Missing sensors return `MissingLidar`; an
unrepresentable finite period returns `InvalidPeriod` before mutation.

The rate policy is exactly the retained IMU service policy. Negative requests
become zero. Native less-or-near comparison is the strict expression
`a < b + 1e-6`. If the authored maximum is not less-or-near zero, requests
less-or-near zero or not less-or-near that maximum are ignored. This is an
epsilon comparison, not a clamp to the authored maximum. NaN is not silently
clamped. Under the no-ceiling policy NaN selects continuous sampling; positive
infinity selects the existing one-nanosecond minimum tick. A finite period that
cannot be represented is refused on a schedule clone before any rate update.

Changing rate preserves the pending sampling deadline. It does not acquire a
scan, draw noise, alter the current sample, move simulation time or advance scan
or cloud sequences. Subsequent acquisition uses the existing phase and catch-up
rules. A schedule/time reset clears the next deadline and sample while retaining
the effective runtime rate, as for IMU. Loaded-world reconstruction recreates
the authored rate. Existing sample, noise and publication reset rules remain.

## Service, names and ownership

Construct the raw scan topic plus `/set_rate`, normalize that whole constructed
sibling with the existing TopicUtils function, then remap once and qualify.
For raw `/rate/scan0/`, the constructed string contains `//`; the retained
normalizer removes that exact pair, yielding `/rate/scan0set_rate`. It does not
collapse a filesystem path. The raw publication topic is never rewritten.
A scan-topic remap does not implicitly rename the rate sibling, and the mapped
destination is not remapped again. Scan and PointCloudPacked output are otherwise
unchanged. The new service is the intentional new observable endpoint.

At most eight lidar owners have independent rate providers. Their keys are
unique and disjoint from all current and loaded-original scan, point, IMU,
controller, sensor and world-service keys. Pure validation completes before
declaration. The existing IMU same-key multiple-provider policy remains intact.
Each new lidar provider retains the expected publication metadata and checks
the world-local ID, sensor kind and private sensor component. Late errors refuse
the complete new family before binding; metadata repair cannot resurrect an
already retired provider.

Lidar and IMU providers share the existing bounded, round-robin CPU command
budget and per-provider queues. Query payloads, replies and declarations remain
on the existing IO owner. A canonical Empty receipt means only that the request
or bounded malformed marker reached the CPU queue. It proves neither parsing
nor application. A full queue reports rejection instead of successful handoff;
oversized payloads are not retained in the IO queue and receive no successful
receipt. The separate `lidar_rates_applied`, `lidar_rates_ignored` and
`lidar_rates_rejected` counters describe the CPU outcome. IMU counters retain
their old meaning.

Requests can be handed off while paused or without lidar subscribers; they
cannot themselves cause acquisition or publication. With no requests, authored
cadence, default scan/cloud bytes, counters, demand and noise remain unchanged.
Worlds without lidars add no lidar providers or callbacks. Removal disables the
provider, and the IO owner completes Query/callback retirement. Reset discards
unread old-generation controls before rebind and preserves the existing terminal
failure rules. No new transport, scheduler, session or execution domain is added.

## Source evidence and regression obligations

The selected contract and admission are retained in `source-selection-proposal.json`
and `source-implementation-admission.json`. Scene author completion
`m115-lidar-rate-scene-source-completion.json` pins the APIs and regression
source. Server completion `m115-lidar-rate-server-source-completion.json` and
the independently closed `m115-normalized-sibling-server-source-completion.json`
pin the rate family and complete normalization correction. Original author
sources and their exact inverse proofs are preserved. Root owns shared exports,
setup/reset reservations, native-client registration and final integration.

There are 22 new Rust Source declarations: 22 headless, 11 renderer and 11 shared;
10 public and 12 private. Scene contributes seven public and four private cases
to both profiles. Server contributes three public and eight private cases to
headless only. Python remains 498 methods; existing method identities are
unchanged. These are obligations, not passes. The completed physical census records 1,874 Source files and 22 changed paths.

The meaningful controls cover the exact cap/no-ceiling/epsilon/NaN/infinity and
finite-period policy, clock-first errors and atomic refusal; independent analytic
timestamps and preserved sampling phase; paused and no-interest handoff; scan
and cloud from the same noisy acquisition; eight-owner fair queues and global
budgets; malformed/oversized payloads and pressure; complete live and removed
original key collisions with repairs; metadata/identity forgery and retirement;
old-generation cancellation; loaded reset and default admission. The retained
IMU tests keep their inputs, assertions and tolerances. Four private rate preparation calls and six retained WorldIo callers gain
only an empty lidar argument; the complete old bodies otherwise remain unchanged.

The separate `lidar_rate` C++ client uses the existing selected NativePeer SDK.
It discovers exactly one Double-to-Empty service at `/m115/scan/set_rate` and
observes `/m115/scan` plus `/m115/scan/points`. Its one-way `node.Request`
overload and `SENT` line prove local submission only, not a remote Empty receipt
or CPU queue admission. The Rust native test separately waits for CPU counters
and reads the effective rate; Rust real-query controls independently prove the
Empty-to-CPU-queue handoff.
An actual two-stream capture barrier precedes the controlled window. Each stream
retains at most 16 packets of at most 1024 bytes. There are at most 32 commands
of 128 bytes, seven-second discovery and five-second capture deadlines. Rust
captures at most two MiB stdout, 128 KiB per line and four KiB stderr, with
eight-second command and five-second exit deadlines plus owned kill/reap cleanup.
This is canonical transport interoperability, not a native sensor runtime oracle.

Native policy attribution is the retained clean `gz-sensors` commit
`8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`, `src/Sensor.cc` SHA-256
`5fb2ea0aaa27b960c463c82de3074ad0bb221341454fb999946eca9f5afff2a2`:
service advertisement at line 251 and capped service policy at line 410. The
uncapped public setter at line 476 is a different API. Native strict comparison
comes from `gz-math` commit `1316ed334f4d5bda7be59aae07cd6e8ff31cfe13`,
`include/gz/math/Helpers.hh` SHA-256
`24aeb75ca8b1ee502c654f045901d39befcbd84dfce4bd7fd93c8e2ca1e439f5`.
The unchanged parent Scene `rates.rs` and `schedule.rs` are the immediate policy
and cadence references pinned by the selection. Retained source inspection does
not establish installed native sensor binary equivalence.

## Required closure

Implementation, regression Source, documentation and independent Source review
are complete, with both finding classes corrected before one final formatter.
A reviewed D31 scheduling amendment permits the one unchanged local full staged
hook and local commit to overlap parent remote closeout, after fresh resource
admission. Root serializes all local heavy workloads, including the parent
original-artifact audit. Parent normal and independent local DATA are already
passed. M115 review publication and hosted execution remain held until parent
whole hosted and official audits and own normal and local DATA pass. No
qualification is inherited; an affecting parent failure freezes the descendant
for a complete batched correction and review. Keep strict workspace, per-crate and changed-base
coverage against all 20 bases: `2419c923` plus all 19 retained predecessor bases.
Both mandatory profiles, the maintained Python/native qualifiers, independent
local retained DATA, the exact automatic hosted attempt and independent original
artifact audit must pass for this candidate before release or acceptance.
Fresh reviewed resource admission is required; no future fit or cleanup is
authorized by this source plan.

Reflectance/intensity provenance, new rays/resolution/noise models, camera clouds,
additional sensor families, IMU policy changes, scheduler or protocol redesign,
new SDK/backend campaigns, DART/MuJoCo and Fuel changes are excluded. Accepted
M1–M76 and 202 slices remain unchanged. Preserve all frozen sources and evidence
and the live no-expiry M77 demo. Formal browser and owner-confirmed shutdown proof
remain release blockers; no browser, stop, restart or eviction is authorized.
