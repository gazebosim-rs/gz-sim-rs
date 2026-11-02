<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Per-pass articulated contact factorization

This PERF-01 candidate now starts at accepted `e829be51`, which
includes the `dff3bc7` baseline and its signed-snapshot acquisition correction.
Exact hosted run `41481267132` and independent official audit passed before Main
promotion. The CI-only parent integration changes no candidate runtime source.
The first required correctness hook ran for 316.033 seconds and failed before
native replay compilation: the historical oracle bound the original
`joint_state.hh`, while the replay script read the changed candidate header.
All three appended contact regressions passed before that failure. The complete
gate, final coverage and candidate performance remain unqualified; no candidate
timing has run. The failed stage and original oracle remain unchanged.

The audited configured physics-only B03 baseline measured a median 11.743475 ms
native window and 23.994811 ms Rust window: Rust was 2.043246 times slower.
That window contains stepping plus 700 complete body and 400 joint observations;
Rust includes Scene synchronization. It is not a full Server or rewrite ratio.
Source inspection found repeated skeleton mass validation/factorization for
contact points within the same admission pass. This motivates a reuse hypothesis;
it does not establish a measured hotspot or a speedup.

`World::contactsWithin` now owns one private `ArticulatedContactPass` on its stack.
The default vector owns no heap storage. Only a mobile multi-body contact that
passes the existing topology restrictions and has a free root creates an entry,
keyed by the actual skeleton pointer. Single bodies, static bodies, an empty
contact list and earlier topology refusals do not allocate factorization entries.
Entries and their Eigen storage are destroyed before the method returns. No
World field, callback, persistent map, public API or FFI contract is added.

Mass and inverse mass depend on configuration and inertial properties. The
contact loop only queries these properties, world transforms and Jacobians:
it neither steps the world nor changes joints, body properties, contacts, solver
settings or collision ordering. DART's lazy query caches can be refreshed, but
those queries do not change the physical configuration or inertia. The same
owned world keeps every referenced skeleton alive throughout this synchronous
loop. The Rust boundary does not permit concurrent world mutation. Thus a
factorization can be reused for that skeleton during this invocation only.
A later step, edit, rollback, clone, reset or removal necessarily enters a new
pass with no previous entry. Invalid factorizations retain a refusal for this
pass instead of being retried for later points.

All mass/inverse symmetry and finiteness tests, positive diagonal test,
normalization, eigenvalue positivity and `1e8` conditioning bound, LLT success,
error norm and strict `error < 1` test remain. LLT still constructs directly
from the same normalized inverse. It remains an LLT object, and every point
still evaluates `jacobian * inverseScale.asDiagonal() * factor.matrixL()` in the
original order. The world-to-local point, dependent Jacobian expansion, squared
norm, division by `1-error` and finite final bound remain per point. No dense
factor conversion, explicit inverse, reordered response or new numeric cutoff
is introduced. Every point is still checked for finiteness. The standalone
`articulatedContactResponse(body, point)` entry remains available and constructs
its own fresh factorization. A private owner check rejects accidental reuse with
a different skeleton.

The benchmark contract now binds the candidate `bridge.cc` bytes for its
source-established settings; the detector, solver, limits and comparison contract
are unchanged. The earlier measurement summary and observation archive retain
their original source hashes and baseline timings. They are historical baseline
evidence, not measurements of this candidate. Any candidate observation must
come from a newly built, source-bound actor after correctness qualification.

Only two production files change: `joint_state.hh` and `bridge.cc`. The existing
static-contact test file retains its complete original prefix and assertions;
new isolated-child controls append native sphere/four-corner footprint checks
across prepared resets and real queued velocity/wrench/effort edits, partial
removal/rebuild and destruction. A second control uses separate articulated
owners with different masses, footprints and anchors in one world, comparing
all physical states against their isolated worlds before and after owner
removal/reset. A third control repeats the actual native above-condition
refusal across prepared reset and floor-removal recovery, checking complete
state/rate atomicity after repeated refusals without changing the `1e8` bound.
These compare public physical observations and the unchanged
actual native fixture, rather than comparing the new helper against itself.
Existing static-contact condition, bounds, B03 and all-100 benchmark assertions
and reference data remain unchanged.

Independent review of the completed correction must precede the next required
gate. The baseline prerequisite is accepted; candidate correctness qualification
remains incomplete after the source-binding failure. Any later timing requires
Root's explicit release after actual correctness qualification; no extra timing
batch is implied by this source work.
The baseline capture and SDK remain untouched. Root owns integration and any
reviewed update of the benchmark's production-source binding.

## Historical native replay

The numeric fixture, original C++ probe and identity helper retain their original
bytes and recorded hashes. The original `joint_state.hh` bytes from accepted
`e829be51d42fed27268606528ba758a93efc37a2` are retained, including Apache notices,
at `tests/fixtures/three_body_static_contacts_joint_state.hh` (SHA-256
`7e5de3ff53d2e6b4a16aa55d176564942cca05ae5611c875951d1c3041a7ec5d`).
The replay validates all three logical historical source hashes before compiling.
It copies only those three inputs into its output's `historical-sources/` tree,
preserving the original `crates/gz-physics-dart/tests/fixtures/` and `src/` layout
so the unchanged relative includes resolve to the historical header.

`native-result.json` records each logical path, actual retained input path,
projected compiler path and content hash, the header's historical revision,
the fixture hash and actual compiler dependency hashes. The dependency check
requires all three projected owned inputs with their exact hashes and the
unchanged qualified system closure. Compiler, DART version, loaded-library guards,
all seven replay modes, exact selected rows and final source/library hash checks
remain in place. A replay output cannot silently reuse an existing projection.
This is explicitly a historical native oracle; the candidate runtime is exercised
by the Rust adapter controls against that immutable oracle.

New Python protocol controls cover the retained header identity and notices,
relative include layout and input provenance, rejection of changed or missing
inputs and redirected source sets, and rejection of missing, modified or extra
compiler dependencies (including accidental selection of the current header).
This correction batch has not run tests, lint, compilation or native replay.
