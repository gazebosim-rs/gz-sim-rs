# Implementation status

The compiler-diagnostic inventory now recognizes seven pinned physics/plugin
registrations without executing their wrappers. Author and independent listings
preserve all 1,175 identities, 579 result guards and 578 configurations/4,589
case expansions, with eleven gaps and the Zenoh empty-list exit1 retained.
Independent source/native/projection review and exact source checks passed
246 Rust/68 tooling/15 unchanged C++ scenarios with unchanged strict coverage.
Normal publication checks remain separate; no test was ported or retired by
recognition. See [the inventory report](NATIVE_TEST_INVENTORY.md).

The explicit clean SDK client profile is published and CI-qualified. A fresh exact-source
Rust native test passed its unchanged fifteen real C++ client scenarios in the
immutable fourteen-repository SDK image. Per-scenario executable/provenance
bindings and the original assertions are retained. The separate strict source
gate passed 246 Rust tests, 64 tooling tests and fifteen legacy-reference C++
scenarios, with every production crate above 95% line and region coverage.
Independent SDK replay passed with matching identities and empty stderr.
The normal protected gate and exact hosted run 40317804892 passed for public
commit b65bc63926c0398181a6924a418bbd7a7d10842a; see
[the scoped report](REFERENCE_IMAGE.md#explicit-sdk-client-profile). No P0/P2,
full-rewrite or performance completion is claimed.

The selected headless implementation is progressing; complete rewrite and P2
acceptance remain open. All 78 broad system migration packets remain open.
Scoped performance reports retain their explicit workloads and limits.

## Earlier configured native result guards

The earlier reviewed external collector listed 579 pinned result-file guards
separately from gtest cases and execution passes. All 578 native configuration/case
payloads and 4,589 case expansions were preserved. That observation retained
eighteen gaps and the empty Zenoh-gated binary's exit1; the original SDK and its
597-gap report are unchanged. Independent source/native review and exact source checks passed
246 Rust/57 tooling/15 real C++ scenarios with unchanged strict coverage.
See NATIVE_TEST_INVENTORY.md for identities and limits. Full REF02/P0 and P2
remain open; no upstream Rust port or new performance result is claimed.

## Clean native headless reference SDK

The fourteen-repository native SDK now builds from the pinned Ubuntu image and
package snapshot. Ten selected unchanged originals and the unchanged 100-step
Server/TPE/IMU fixture pass with empty stderr after providing private writable
Gazebo logs. The expanded inventory retains 597 explicit gaps; these results
do not complete REF-01, REF-02, P0 or P2. [REFERENCE_IMAGE.md](REFERENCE_IMAGE.md)
records the immutable local image, manifests, retained failures and remaining
profiles. Image registry publication and adoption into the native-client CI
contract remain separate work.

## Corrected TPE static and world-axis behavior

Confirmed D23 keeps static models immobile, including empty models without
contact bounds, and honors world-axis velocity commands on supported rotated
model/link frames. Pause/reset/public IDs, IMU frames and unsupported wrench
admission are covered. Independent source/native/assertion review and exact
staged246 Rust/41 tooling/15 unchanged C++ child clients passed; all crates
exceed95% lines and regions, workspace3548/3584 and5419/5538, changedTPE27/27
and scene11/11. [TPE_POLICY.md](TPE_POLICY.md) records retained native differences,
the reviewed guard correction and remaining frame scope. Normal protected commit
and exact hosted verification remain separate requirements. Full PHY02/P2 remain
open; no new performance result is claimed.

## Ownership and model routing

The coordinator owns this monorepo and integration. One agent owns each independent
repository. The sdformat-rs and Transport owners use gpt-6.1-sol with high
reasoning. The Msgs owner uses gpt-6-astra with high reasoning after repeated
capacity failures in both Sol models; work remains bounded and independently
reviewed. Agents work on bounded
packets and independently review one another's changes. Fuel remains with its
existing agent. Four concurrent slots cover the coordinator and three repo agents.

## Disabled IMU orientation

Independent source/native/assertion review approved the bounded loaded field-
presence policy: four actual native Server controls and all fifteen real C++
client scenarios pass. Exact staged checks pass with every crate above95% lines
and regions, workspace3516/3552lines5369/5486regions and changed scene2/2,server3/3.
See IMU_ORIENTATION.md and planning/reviews/imu-orientation.json. Runtime toggling
and full SENSOR01/P2 remain open.

## Cylinder geometry

Centered Z-axis cylinder loading, Rapier dynamics and native TPE AABB contacts
passed independent source/native/assertion review and exact staged checks:
238 Rust tests, 24 tooling checks and the unchanged fifteen C++ scenarios.
Workspace coverage is 3535/3572 lines and 5395/5514 regions; every production
crate exceeds 95%, with changed Rapier9/9 and scene12/12 covered.
[CYLINDERS.md](CYLINDERS.md) records the bounded contract and native controls;
planning/reviews/cylinder.json records the review and test dispositions. Normal
protected commit and exact hosted verification remain separate requirements.
Broader LOAD/PHY packets and P2 remain open.

## Reviewed Transport consumer integration

The reviewed pin c21ffcc adds separate-Node direct delivery and a bounded
service-response lane. The retained experimental2d gate failed a service timeout;
its consumed response hit the shared command queue. That failure is preserved.
The new first-frame/reset/origin test retains all existing payload and timing
assertions on Rapier and TPE. [LOCAL_CLIENTS.md](LOCAL_CLIENTS.md) records the
repair and native routing boundaries. Exact staged239 Rust tests,24 tooling
checks and all15 unchanged C++ scenarios pass; every crate exceeds95% lines and
regions with unchanged workspace coverage3535/3572 and5395/5514. Independent
source/API/assertion and private first-frame/service-response replay approved;
producer hosted40248292544 SUCCESS. Normal consumer hook/hosted remain required.

Seven within-Rust publication comparisons retain all 154 observations, including
regressions. The final paired ratio is 1.03598 with 95% interval [0.99243,1.07658]:
the point meets the proposed 5% target, while a 5% upper bound remains unproven.
The separate historical public05/native report applies only to its exact selected
TPE/IMU stream fixture. Neither report accepts balanced robotics performance,
ratified budgets or P2. See planning/reviews/transport-consumer.json.

## Implemented scope

| Crate | Behavior | Remaining boundary |
| --- | --- | --- |
| gz-assets | Bounded local files/configured directories, cache-only Fuel model includes, native lookup precedence, lossless source IDs and structured warnings | Downloads/cancellation, SDK/install/package policy and decoded resource caching |
| gz-sim-core | Bevy schedules, deferred commands, exact/pause stepping, integer time/epochs, atomic overflow checks, stable public IDs and cleanup, typed terminal fault diagnostics, world/allocator-bound reset reservations, isolated worlds | Persistence and complete lifecycle contract |
| gz-physics | Shared f64 body construction, inertial, rotation/state values and exclusive installation identity | Shared backend capability contract and TPE scene integration |
| gz-physics-tpe | Validated constant-velocity Body, Bevy-owned Motion, exclusive phase/reset installation, model overlap contacts and typed faults | Remaining geometry/commands and full backend conformance |
| gz-physics-rapier | Full-f64 sphere/box/cylinder dynamics, inertia tensors, static bodies, world impulses, gravity, world velocity/COM wrench inputs, world-revolute constraints, backend reset/removal, prepared initial-state restoration and numeric validation | General articulations, remaining force frames and common capabilities, DART/MuJoCo/TPE adapters |
| gz-sensors | IMU specific force/relative orientation, seeded Gaussian/bias/quantization and native covariance using f64 math | Activation, complete frame configuration and remaining sensor families |
| gz-sim-scene | Atomic SDF construction, Rapier/TPE construction, backend-independent ECS observations, Bevy ownership hierarchy, canonical model and local poses, phase-preserving IMU sampling, checked model-subtree removal and loaded-scene restoration | General joints, offset/multiple shapes, full resource policy and remaining sensor contracts |
| gz-sim-cli | Local SDF/configured world input and include roots, Rapier/TPE selection, exact integer steps, sorted physical-link state and typed load errors | Legacy grammar, transport serving, remote top-level resources, signal handling and packaging |
| gz-sim-server | Bounded clock/pose publication, shared-topic IMU streams, polled pause/step/reset controls, deferred remove services, real C++ clients | Root clock election, run-to controls, scene services and runtime sensor registration |

Rust 1.95.0, Bevy ECS 0.20.0, Rapier3d-f64 0.36.1 and glam 0.33.12 are pinned.
Authoritative state and solver use f64. The submillimeter motion probe at world
coordinate 10,000,000 m passes. The Linux normal dependency closure contains no
Bevy renderer/window or wgpu/winit dependency. Rendering profiles remain pending.

The backend is a capability probe, not the final physics interface. Invalid source
values, derived inertia/AABBs and impulse velocities fail before mutation. This
is not a solver-stability guarantee for arbitrary magnitudes or time steps. Direct
ECS access must preserve installed resources and scene ownership invariants;
system panics invalidate the run rather than rolling back state.

## Current evidence

Coordinated typed-SDF adoption passed independent source/API/assertion review
and exact staged checks:230 Rust tests, sixteen tooling checks and eleven C++
scenarios; coverage3513/3549 lines5366/5483 regions, every crate above95%,
changed scene8/8. [TYPED_SENSOR_ADOPTION.md](TYPED_SENSOR_ADOPTION.md) records
explicit unsupported sensor/joint boundaries and the qualified static-frame
native replay. Normal protected hook, exact hosted and publication remain
separate qualification requirements; broad LOAD-01/P2 remain open.

Subscriber-driven pose/IMU work passed independent native/source/assertion review
and the exact staged gate:227 Rust tests, sixteen tooling checks and eleven actual
C++ scenarios; coverage3505/3541 lines5353/5470 regions, every crate above95%,
changed scene/server33/33 each. [SUBSCRIBER_DEMAND.md](SUBSCRIBER_DEMAND.md)
records the global native rule, atomic-only transport pin and preserved workload
policies. Normal protected hook and exact hosted qualification are separate
requirements; broad IO-01/SENSOR-01/P2 remain open.

Gaussian IMU noise passed independent source/native/assertion review and the
exact staged gate: 221 Rust tests, sixteen tooling checks, eleven actual C++
scenarios; coverage3436/3472 production lines and5255/5371 regions, every crate
above95%. [IMU_NOISE.md](IMU_NOISE.md) records seeded/reset policy, native
controls, preserved fixtures and remaining scope. Normal protected hook and
exact hosted verification remain separate qualification requirements.

Native default pose cadence passed independent source/native/assertion review
and the exact staged gate: 208 Rust tests, sixteen tooling checks, nine actual
C++ scenarios, 3229/3264 production lines and 5007/5122 regions; every crate
exceeds 95%, with 43/43 changed server lines covered.
[POSE_CADENCE.md](POSE_CADENCE.md) records native controls, explicit old-policy
test dispositions and remaining IO-01 scope. Normal commit/publication and exact
hosted evidence remain separate requirements.

Published d4c73ee passed the normal hook and exact hosted CI 40219319485:
204 Rust tests, sixteen tooling checks and all nine actual C++ client scenarios.
Coverage is 3193/3228 production lines and 4964/5079 regions; every production
crate exceeds 95% on both metrics. The complete-stream observation increment is independently reviewed and hosted
green, with unchanged production coverage and all nine C++ scenarios. No existing test, tolerance,
exclusion or threshold was weakened. Full P2 acceptance remains open.
[HEADLESS.md](HEADLESS.md) records the behavior and remaining integration scope.

Published clock/control, IMU and topic-repair revisions passed clean GitHub CI.
The hierarchy/pose revision d3524f5 passed its exact hosted gate.
[CORE-01 is accepted](acceptance/CORE-01.md); API-01, QLT-01 and the broader phases
remain open. Local model removal is published at 5fd5e49 and remote removal at b39ea419.
The consumer now uses reviewed SDF frame/topic/canonical helpers and rejects
explicit Surface and fluid added-mass physics until implemented; transport includes native publisher
recreation ordering. Producer pins and tests are detailed in HEADLESS.md. The C++ client now
verifies seven scenarios using actual clock, local-pose and IMU samples, exact-step
controls, stable IDs, topic repair, deferred model removal and reset/resume, including explicit TPE kinematics. Its
native dependencies are rebuilt from the pinned reference lock in CI.

`python3 scripts/verify.py` runs formatting, Clippy with warnings denied, Rust and
gate tests, and LLVM line/region coverage. Every owned production source must be
instrumented. Per-crate and changed executable-line coverage must exceed 95%;
exactly 95%, empty denominators, missing/duplicate profiles and malformed mappings
fail. The staged hook verifies the exported Git index and checks it is unchanged.
Fresh counts, source hashes, toolchain and logs are written to artifacts/coverage.
Independent repositories retain their own coverage and test obligations.

Feature matrices, build-script input accounting, native/shader coverage, fuzzing,
branch protection and performance qualification remain open under QLT-01 and
TESTING.md. No test retirement or coverage exclusion is implied by these runs.

## Reference scope and next work

The local C++ math build passes 188 CTest entries, including wrappers and Python
tests. Seven original IMU unit and three integration cases pass. The Rust IMU
measurement port retains assertions/tolerances from SensorReadings without retiring
the full original case. An isolated Clang simulator build passes ten ServerConfig
and fourteen SimulationRunner cases; its GCC build encountered an upstream
compiler issue. Selected native case IDs and hashes are in
planning/reference-results.json. These installed dependencies are not yet a
complete reproducible reference environment.

The untouched native UserCommandsTest.Remove case and all 18 parameterized
ResetToWithAddedEntity/DeletedEntity cases pass. Their source/binary/output hashes
and partial or pending Rust mappings are recorded in planning/reference-results.json.
These use the same qualified installed dependency environment.

Additional native fall, nested/canonical-pose and remove/reset probes guide the
headless work. Loaded-scene reset now restores initial logical IDs, nodes and prepared physics,
with native reset-time removal observations and preserved Bevy infrastructure.
The separate core reset remains limited to retained domain state. Remote reset
now composes control ordering, real time and stream reconstruction. SCENE_RESET.md
and SERVER_RESET.md record supported boundaries, cancellation differences and ports.

Next: finish the selected-fixture assertions and equal-work reference comparison/
performance report for P2. The bounded simple-joint slice passes author/staged
checks and independent review; its normal hook and hosted qualification remain. Full downloads and sensor noise remain wider-plan work, rather
than additional prerequisites for this local-resource, noise-free IMU fixture.
Then complete the retained backend, sensor/system, rendering, plugin, packaging
and release matrices. CORE-02, full IO-01 and P2 are still open.

## Offline Fuel integration

The workspace consumes public Fuel 3747086 and Transport 7ce88e5 with its exact
Messages 2cea810 dependency; using one Messages identity preserves its TypeName
trait boundary. Transport's reviewed socket2 0.6.5 pin fixes the former Fuel/Tokio
resolution conflict. The actual root lock uses Tokio 1.54.1; hyper-util 0.1.15 keeps
its separate socket2 0.5.10 dependency. No constraints were relaxed or producer
source edited. Fuel cache lookup never invokes download/fetch/details, and misses
remain explicit. Both engines load through the cache, step and restore original
logical IDs/poses after the resolver and cached files are removed.

See LOCAL_ASSETS.md for limits, configuration and native-reference qualifications.
This is a model-include consumer, not completion of downloads, cancellation,
remote world/model-file URLs or the complete FUEL-01/P2 contract.

## Worktree hook isolation

The exported staged verifier removes official repository-local Git environment
keys only in its child process. The parent still selects and rechecks the intended
index. A new real-worktree/alternate-index regression checks child normal/bare
Git fixtures without redirecting them into the caller. Main HEAD ancestry, index
bytes, configuration, unstaged content and nonlocal settings are preserved. The
original wrapper fails the new test on leaked keys; the fix passes. This is a
verification-boundary correction; production Rust coverage and all seven native
client scenarios remain required. No root repository corruption is claimed.

## IMU acquisition rates

The scene now supports native-phase sampling rates independently of the physics
step, with checked time overflow, per-physics acceleration and pause/reset state.
SENSOR_SCHEDULING.md records native clock controls, actual C++ cadence/reset
observations and remaining scope. The former Rust-only 10 Hz rejection becomes a
stronger supported-input test; the error table retains a genuinely unrepresentable
1e-300 Hz period. That disposition requires independent assertion review; no
upstream test is retired or numerical tolerance relaxed.

## Exact staged source export

The verifier now exports through a private index and validates all raw Git blobs,
file modes, symlinks, directories and paths before running checks. The regression
that previously failed because export-ignore hid a staged file now passes. A
tracked changed-lines.json also survives unchanged; generated metadata stays
outside the exported source. Checkout conversion and deliberate snapshot mutations
fail closed. Simulator Rust and its native/coverage requirements are unchanged.


## World physics configuration

The CLI uses the first SDF physics profile's timestep when no explicit override
is supplied. Existing Duration APIs remain explicit overrides. Owned WorldPhysics
metadata survives loaded reset without IO. Three fresh native Server clock rows
match both backends; plugins were disabled, so this is clock/configuration
evidence only. Nondefault contact limits fail explicitly, and exact stepping
remains unpaced. Runtime profile changes and backend-specific settings remain
open. See HEADLESS.md and planning/reference/world-physics.json.


## Local physical input increment

A private pending Bevy component per target link provides world-velocity
replacement and one-step COM force/torque accumulation. The author gate passes
182 Rust checks, twelve tooling checks and the existing nine actual C++ scenarios;
2733/2767 production lines and 4288/4391 regions pass all per-crate gates.
Thirty fresh native world-input controls qualify supported velocity/angular
observations and explicit TPE gaps. Published 25c801d passed independent review, the normal staged hook and hosted
CI 40185967463. PHYSICS_INPUTS.md records the scope and
analytic force-position accuracy bound. The complete selected-fixture assertions/reference/performance report remains
P2 work; JOINTS.md records the bounded joint variant. These local APIs cover its bounded free-body input
capability when exercised by the selected fixture; native input transport systems
remain wider retained work unless that fixture requires them. Real C++ peer
interoperability remains required.


## Controlled producer metadata

SDF 706e8fde supplies native axis-default corrections, owned Material and link
flags. VisualMaterial and LinkProperties survive loaded reset; nondefault physical
flags are rejected until supported. Transport 7ce88e5 and matching Messages 2cea810
preserve one Messages identity and retain Fuel 374 unchanged. The full author
gate passed 184 Rust checks. Published 67c1af4 passed independent review, the
normal staged hook and hosted CI 40188844686. The joint slice now consumes
pre-Contact SDF 4ed690b (derived Clone only); later sensor enum changes remain
unadopted.
PRODUCER_METADATA.md records graph, scope and the unadopted Contact enum boundary.


## Native client request lifecycle

A strict shared-IMU gate exposed a pinned native blocking-request cleanup race.
The test client uses the public asynchronous API with the same bounded wait,
leaving request removal to the response thread. All original payloads, result
checks, timeouts and empty-stderr assertions remain unchanged. Independent native
replay and the full 194-Rust/nine-C++ staged gate pass with unchanged coverage.
NATIVE_CLIENT_REQUESTS.md records source/lifetime proof and the terminating-client
limitation. Normal-hook publication and hosted qualification remain required.
The shared-IMU packet was held until it adopted this fix and passed its checks.


## Shared-SDF sensor reference

The new headless IMU control runs identical unpaced SDF bytes through actual
native DART/TPE Server, Physics and Imu systems, and compares every acquired
reading/time to Rust over 100 pre-contact steps. Readiness and queued playback
replace startup sleeps. Independent source/native/Rust/mutation review and the
exact adopted 195-test/nine-C++ gate pass. Native installed-build provenance
remains qualified; HEADLESS_REFERENCE.md records scope and the earlier client
cleanup failure repaired by published 94814ee. Normal-hook publication/hosted
qualification and the full P2 assertion/performance report remain required.


## Bevy step executor control

The ordered headless pipeline defaults to Bevy's single-threaded executor;
set_step_executor accepts Bevy's parallel executor explicitly. Reset scheduling,
backend/query parallelism and phase/fault/deferred semantics are retained. The
same release binary improved this isolated Rust pipeline from 54.244 to 4.630 us
per step; independent CPU31 replay reproduced the result. Native full-server
numbers are context only because publication/bookkeeping paths differ. The
warmup command/IMU acceleration mismatch was repaired in published a867617,
with native force controls and hosted CI 40210013522 success.
HEADLESS_PERFORMANCE.md retains the historical direct-pipeline data;
FULL_SERVER_PERFORMANCE.md records the new per-step server comparison; this does not establish P2 or rewrite-wide performance acceptance.


## Velocity replacement and IMU history

A native-backed regression distinguishes velocity configuration from integrated
force. All 300 acquired readings/timestamps from native DART/TPE match the bounded
Rust checks; a real force remains visible. Multiple attached sensors, paused
commands, invalid input, failed execution and reset/removal lifetimes are checked.
INPUT_IMU.md records the changed benchmark warmup assertion and historical-report
limits. Independent native/Rust/mutation review approved the exact source and warmup
assertion correction. The strict staged gate passes201 Rust/14 tooling/nine actual
C++ scenarios:3182/3228 lines, 4941/5079 regions, all crates >95% and scene12/12
changed lines. Published a867617 passed its normal hook and exact hosted
CI 40210013522. P2 stays open.


## Full-server step-to-observation increment

The Rust harness now invokes actual Server::tick and receives/checks each IMU
before another step. The native control likewise invokes actual Server.Run for
one step and waits for complete receipt. Default bounded queues and all decoded
field, byte-count, timestamp and body-motion checks remain. Independent replay
reproduced Rust 27.962µs/native 25.098µs medians; author values are 28.403/24.177µs.
Native/Rust ratios remain contextual because unobserved publication and local
delivery paths differ. A separate same-harness Transport 7ce→48a replay reproduced
a large within-Rust improvement; it does not isolate command wakeup as the sole
cause. Missing-IMU mutation fails. FULL_SERVER_PERFORMANCE.md records limits and
the rejected overloaded batch; planning/reviews/full-server-pipeline.json binds
the approved source. The selected-fixture lifecycle matrix remains required.


## Durable reset/removal reference

Ten tracked native DART/TPE controls now reproduce selected queued-control,
reset callback, identity, model pose and reset-plus-step observations on the shared
SDF. Rust compares actual Reset observer counts and completed outcomes. Native
ordinary removal/reset does not expose a cancellation counter; the additional
Rust cancellation boundary is stated separately. Independent complete native
replay and Rust execution pass; counter/multistep/pose mutants fail as expected.
LIFECYCLE_REFERENCE.md and planning/reviews/lifecycle-controls.json record scope
and the declared two-step cross-engine position bound. This closes the missing
durable reference controls; the complete selected-fixture acceptance matrix is
still required for P2.


## Complete requested streams

Actual Rust/native servers now return a checked clock, four-role parent-relative
pose packet and IMU for each selected request. Prospective 20ms host gaps respect
native pose/info's hard 60 wall-Hz limit; measured call/receipt/check latency excludes
that declared pacing and reports wall time separately. An unpaced missing-pose
experiment remains a failed reference, and current Rust default pose-rate parity
is not claimed. Independent full 128-step/seven-pair replay and missing-pose/report
mutants validate the scoped protocol. STREAM_OBSERVATIONS.md records all fields,
opaque ID-byte accounting, large timing outliers and the remaining P2 matrix.


## Selected milestone and formal prerequisite audit

Independent review accepts the selected headless integration evidence across
fifteen qualified contract rows and 85 hashed source/reference files. It does not
accept formal P2 or SLICE-01: P0 image/inventory and named LOAD/PHY/SENSOR/IO
prerequisites remain open, including default pose cadence and noise/activation.
docs/acceptance/P2.md records the concrete gaps and unchanged task DAG.
Implementation continues through those requirements; phase/packet states stay
in progress. Publication and hosted checks for the audit metadata remain pending.
