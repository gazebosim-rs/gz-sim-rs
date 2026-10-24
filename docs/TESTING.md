# Test preservation and quality gates

## Acceptance rule

Every retained upstream behavior must pass its existing test semantics. The owner
allows faithful Rust ports of C++ tests while preserving fixtures, assertions,
edge cases, and tolerances, checked against C++ reference runs. New Rust APIs do
not justify reduced coverage of behavior.

The rewrite also requires **strictly greater than 95% production Rust line and
region coverage**, per production crate and for the workspace. The owner's >95%
requirement is confirmed; choosing both metrics and per-crate enforcement is the
proposed concrete policy. Exactly 95.00% fails. High coverage is necessary, not
sufficient: parity, negative cases, numerical accuracy, lifecycle safety, and
performance gates are separate requirements.

This repository currently contains the plan and inventory only. It has no
simulator implementation, executable migration test results, coverage report, or
installed quality hooks. Implement and validate these gates in P1 before the
first accepted production feature. Never present a plan-only CI result as simulator
acceptance or treat an empty workspace as 100% covered.

## Pinned reference and complete test ledger

P0 builds the selected upstream main commits in an isolated, reproducible
environment. Pin compiler, native engines, assets, environment, GPU/driver where
applicable, and build flags. Fetch repositories by immutable SHA. Preserve a
reference binary/image for differential runs without modifying the local originals.

Enumerate actual CTest targets and expanded gtest cases, not just TEST macros.
Include Ruby, Python, CLI, integration, regression, backend, GPU, performance,
and behavior-bearing example tests. Track disabled cases and platform/backend
conditions. Compare runtime enumeration against the static source inventory.

Each ledger entry needs:

| Field | Content |
| --- | --- |
| Stable ID | Repository, upstream SHA, suite, case, parameter/backend/configuration |
| Source | Exact path/line or symbol and fixture hashes |
| Contract | Behavior, assertions, units, tolerances, prerequisites |
| Baseline result | Pass/fail/disabled/unavailable plus exact command and artifact |
| Migration disposition | Unchanged harness, faithful port, explicit removed scope, or pending |
| Replacement | Rust test IDs or harness adapter and production area |
| Evidence | Rust result, differential result, configuration, commit/tree digest |
| Review | Assertion equivalence reviewer; approved scope decision if retired |
| Follow-up | Owner and task for unresolved issues; no silent exception expiry |

Use one entry for each expanded parameter/backend case or an explicit structured
expansion. CI rejects unmapped cases, deleted fixture hashes, duplicate IDs,
unexplained changed counts, and missing evidence for “passed” entries. Early
milestones can carry clearly pending future tasks; P7 cannot have retained-scope
pending, unavailable, or failing required cases.

Do not mark baseline failures as Rust passes. Investigate them, record any
upstream defect separately, and add corrected behavior only with a reviewed
specification. A new tolerance or corrected fixture needs an explanation and
regression test, not a blanket waiver. A flaky test needs repair or a documented
blocking issue; automatic retries do not convert it into reliable acceptance.

## Test migration routes

1. **Unchanged upstream harness:** compile the original test against a thin
   adapter where useful. Test the Rust implementation, not the old library
   accidentally left in the link path. Record linkage/provenance.
2. **Faithful port:** retain input/fixture and assertion semantics, including
   exception/error and lifecycle checks. Rust ownership differences can change
   expression syntax; they cannot remove behavioral cases. Review both sources.
3. **Differential runner:** run old and new systems on the same inputs and emit
   canonical observations. Compare values, protocol responses, timestamps,
   ordering where defined, and resource outcomes. Canonicalization must not erase
   meaningful differences.
4. **Explicit removed scope:** classify Bullet/Bullet Featherstone, GUI-only,
   Ogre-internal, CMake-only, and superseded C++ implementation-mechanism tests
   with rationale. Preserve their relevant user behavior through another route.

The release report publishes counts for all four routes and unresolved cases.
It must not say every upstream test ran unchanged unless that is literally true.

## Numerical and sensor validation

Use same-engine differential tests for DART and MuJoCo with pinned settings.
Cross-engine tests assert physical contracts and predeclared error budgets, not
identical trajectories. Compare transforms with consistent quaternion signs and
frame definitions without hiding orientation error. Specify absolute/relative
error and units, time horizon, transient behavior, and statistical confidence.

Add analytic checks: free fall, pendulum, conservation where applicable, inertia
transforms, contact normals/friction, joint motor/limit behavior, wheel odometry,
and IMU gravity conventions. Use property tests for transform composition,
serialization, resource resolution, and degenerate inputs. Fuzz SDF, transport,
image/mesh entry points, and native-plugin boundary validation.

GPU outputs need dedicated tests: known geometric depth/range; calibrated camera
projection; exact categorical IDs for segmentation; distortion/inverse mapping;
bounding boxes; thermal units; clipping and missing returns; sensor frame/timing.
Use tolerances for continuous outputs and approved raster-edge masks where needed.
Do not require bitwise RGB identity between Ogre and Bevy. Do not use an arbitrary
perceptual similarity score as the sole sensor-accuracy test.

Lockstep tests must verify complete observations for each requested step.
Throughput tests check loss policy, sequence gaps, bounded memory, and timestamp
truthfulness. Cover pause, exact N-step, reset, despawn during GPU work, sensor
rate changes, slow subscribers, and cancellation. Software rendering can support
some tests but does not substitute for required GPU qualification.

## Coverage accounting

Use pinned cargo-llvm-cov and matching LLVM tools to generate JSON with line and
region counts. Stable Rust coverage is the primary gate. Branch coverage is
currently an optional/unstable toolchain feature in cargo-llvm-cov; keep a pinned
additional diagnostic job rather than pretending it is an equally stable gate.
[cargo-llvm-cov documentation](https://github.com/taiki-e/cargo-llvm-cov)

Compare integer counts, not rounded percentages:

```text
count > 0 AND covered * 100 > count * 95
```

Apply that independently to lines and regions for each production crate and the
whole workspace. Add a changed-production-lines >95% gate to stop an already
large crate hiding new untested code. No crate can borrow another crate's coverage.
Every real file must appear in the report; missing instrumentation is a failure,
not zero executable lines or an implicit exclusion.

Use a checked-in coverage manifest listing production roots, crates, supported
feature profiles, required runners, and explicit exclusions. Allowed candidates
for exclusion: generated protobuf/bindgen output and unmodified third-party code.
Record provenance and generation hashes. Test code/bench harnesses are outside
production metrics. Include handwritten build logic, macro implementations,
adapters, error paths, and native-boundary Rust code when executable and owned.
Do not exclude entire FFI, IO, rendering, or “hard to test” modules.

Additional executable developer tooling gets its own tests and coverage policy;
it must not inflate product coverage. Handwritten C/C++ bridge coverage is
reported separately and should also exceed 95% line coverage where instrumentable.
Unmodified DART/MuJoCo/dependency internals do not belong in the rewrite denominator.
GPU shaders cannot be measured by Rust LLVM coverage; list every shader entry
point and required analytic/image tests separately. Be explicit about this limit.

Required coverage profiles include core/no renderer, every retained physics
adapter, rendered sensors, native extension host, and logging/transport bindings.
Keep independently useful configurations (e.g. headless TPE and headless DART)
tested rather than compiling only one all-features binary. Features may be mutually
exclusive; `--all-features` is not a universal verification strategy.

Merge raw profiles only for compatible instrumented builds and matching source
revisions. Different OS/toolchain/feature builds may need separate reports.
Calculate a source-level union carefully when valid, but also require each
supported configuration's behavior tests and declared coverage gates. Never average
percentages across reports. A missing backend/GPU profile blocks qualification.

## Pre-commit and CI

Implement one authoritative check runner in `xtask` (or a small equivalent) used
by both a checked-in `.pre-commit-config.yaml` and CI. Provide a bootstrap command
that installs pinned tools and hooks. Protect main with required checks; local
hooks alone can be bypassed and therefore are not the final enforcement boundary.

The intended interface below is a P1 deliverable, not an existing command:

```text
cargo xtask verify --staged
cargo xtask verify --profile headless-tpe
cargo xtask verify --profile headless-dart
cargo xtask verify --profile headless-mujoco
cargo xtask verify --profile headless-rust-physics
cargo xtask verify --profile rendered-sensors
cargo xtask verify --profile native-extensions
cargo xtask verify --release-matrix
```

| Gate | Pre-commit behavior | CI/release behavior |
| --- | --- | --- |
| Formatting | rustfmt and relevant native/shader formatting check | Same check on committed tree |
| Lints/build | Clippy with warnings denied on affected valid profiles; deny unsafe outside approved boundaries | All supported profiles; rustdoc/doc tests; dependency-direction checks |
| Unit/integration tests | Affected crates and transitive consumers; no tests skipped silently | Full Rust suites and required platform/backend matrix |
| Existing-test parity | Mappings plus applicable parity cases for affected behavior | Full retained baseline suite and ledger reconciliation |
| Coverage | >95% line/region for affected crates/profiles and required workspace evidence for exact staged tree | Full per-crate/workspace/profile thresholds and changed-line gate |
| Policy | Manifest, fixture, forbidden-dependency, and exclusion changes flagged for review | Required reviewer approval for policy exceptions and scope changes |
| Performance | Fast deterministic regression probes when hot paths change | Dedicated hardware comparison for relevant changes and full release workloads |
| Native/GPU | Required local runner or exact-tree evidence from designated runner | Mandatory real-backend/GPU jobs; unavailable is not success |

`--staged` must verify the staged snapshot in an isolated directory/index export,
not unstaged local edits. Reuse cached results only if keyed to the full source
tree, lockfiles, toolchain, features, native libraries, test data, and runner
identity. If required GPU/native coverage cannot run locally, the hook may submit
the staged snapshot to a designated runner and require authenticated evidence
for its digest. Until that mechanism exists, strict verification requires a
properly equipped machine. Do not silently downgrade it to CPU-only coverage.

Keep documentation-only checks proportionate: validate docs/manifests and reuse
unchanged-code evidence by verified code digest when implemented. No production
code exists today, so coverage is explicitly not applicable rather than passed.

CI must test the checks themselves during P1: exactly-95 and below-95 data fail;
missing crate/profile/instrumentation fails; stale evidence fails; a weakened
assertion/removed mapping is detected or review-blocked; backend failure fails;
partial staging cannot validate different source. No `continue-on-error` on
required jobs, unpinned “latest” tools, or ignored exit codes.

## Review and release evidence

Automation checks syntax, counts, test results, and forbidden dependencies. A
reviewer checks human-readable code/comments, semantic equivalence, meaningful
assertions, appropriate abstraction, and unjustified custom algorithms. No
automatic “human style” score can replace that review.

Archive machine-readable test results, coverage JSON, missing-line reports,
feature/runner metadata, differential outputs, and exact commands against the
release commit. Record failures and unrun work plainly. Only a complete required
matrix supports a release acceptance claim.
