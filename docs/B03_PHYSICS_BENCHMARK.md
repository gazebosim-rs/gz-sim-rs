<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Configured physics-only B03 workload

This contract was prepared on accepted simulator `43cf8e69` and independently
reviewed before consolidated qualification. Actual outcomes are recorded by the
benchmark driver; the contract definition alone is not passing evidence.
The earlier contextual Server/Scene actors and historical observations stay
unchanged in their separate worktree. Their network and observation differences
still prevent a full Server or rewrite speedup claim.

The two new actors use the exact imported `b03_double_pendulum.sdf` bytes (SHA
`24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa`),
declared input tag `b03.sdf`, default world name, 7 bodies, 4 joints, 15 colliders,
1 ms increments and zero commands. The native public gz-physics plugin and Rust
Scene have no Server, network, sensor or rendering runtime. The sun remains in
the input; the common physical identity inventory has 30 entries: one World,
three Models, seven Bodies, four Joints and fifteen Shapes. Native public physics
has no Light identity, so none is invented. Rust filters its broader metadata to
these physical families. IDs are opaque per role; full names and parent relations
must agree. This contract includes Rust Scene stepping and ECS synchronization;
it is not an assertion that each role executes the same internal instructions.

## Explicit configuration and evidence

Native construction reuses the maintained `ConstructSdfWorld` API and selected
qualified `cb921` SDK/plugin. Bare construction ignores Physics DOM settings,
so the actor explicitly requests gravity `(0,0,-9.8)`, ODE, Dantzig and unlimited
per-collision-pair contacts. Gravity, detector, solver and pair limit are read
back through public APIs before the window. Pair infinity is the actual
`size_t::max()` readback, serialized as a decimal string, on the qualified x86-64
SDK. It bypasses the GzOde copy/clear/re-add pair limiter through its existing
public implementation. It does not patch the SDK or change any product default.

The selected native plugin sets **global** maximum contacts to 10,000 in
`ConstructEmptyWorld`. The accepted Rust adapter explicitly sets the same global
value and uses plain ODE without a per-pair limiter. These selections are
**source-established**, bound to the implementation sources and selected library
hashes in `reference/b03_physics.json` and the actual per-role build/DSO records.
There is no public global-cap getter. Global 10,000 is not the native default
per-pair 20. Matching this new explicit setting does not qualify original-default
B03 dynamics, all private solver options or full Server equal work.

The native Dantzig iterations getter warns and returns unsupported sentinel
zero. The actor deliberately does not call it; both roles record `value: null`,
`evidence: unsupported`, rather than zero solver iterations. The 1 ms native dt
is the actual submitted `ForwardStep::Input` duration; timestamps count submitted
steps. No public native clock/dt getter exists in this feature set. Rust can
read its Scene step duration; its gravity value is the parsed SDF input,
explicitly labeled `sdf-input`. Scene construction passes this value to
`DartBackend::new(world.gravity)`, bound to the checked Scene source. No public
Scene/backend gravity getter exists. Detector/solver/contact selections remain
source-established. Different capabilities keep different evidence labels.
No unsupported readback is fabricated to make the records look identical.

## Window and correctness

Before the window, each actor loads the input, constructs the world, resolves
and stores all body/joint handles and names, verifies physical ownership, and
allocates 100 complete frame buffers. The native buffers are fixed arrays. The
Rust actor fills preallocated slots. During the window each performs exactly:

- 100 unpaused steps of requested 1 ms;
- 700 successful fresh body rows, each with the existing 22 fields: world origin
  pose, origin linear velocity, declared-COM-point linear velocity, angular
  velocity, declared COM world position and declared local COM;
- 400 successful joint rows, each position and velocity.

Native body rows use two public point `Resolve` calls, one at identity and one
at the declared local COM. Joint rows call both public scalar getters. Logical
row counts do not assert equal internal API call counts. Counters increment only
after each successful step/complete row. No setup, handle lookup, output formatting,
file writes, maps collection or registration runs in the window. Engine internals
can still allocate; there is no global no-allocation claim. All 100 frames are
retained and checked for finite/full values after the window. Full paired output
contains 16,200 scalar comparisons at unchanged absolute `1e-12`.

`steady_clock` / `Instant` record wall nanoseconds around only this work. GNU
`time` separately records whole-process user/system CPU, elapsed time and peak
RSS, including load, output and destruction. Its decimal resolution is retained;
these are not fabricated high-resolution window CPU or memory measurements.
Setup/query/output/destruction scopes must not be mixed in a reported ratio.

## One bounded batch

The driver `scripts/benchmark-b03-physics.py` runs one excluded correctness/warmup
pair, then seven measured pairs with alternating order on one explicit allowed
CPU. Pair zero must first prove common settings, every one of the 100 paired
frames, physical ownership, exact input and actual DSO selection. Any failure
stops the batch without retry or timing eligibility. Later processes must also
match the role's excluded capture and the other role at `1e-12`.

The accepted original-default fresh native fixture remains unchanged. Pair zero
reports its selected 1/10/100 compatibility separately. Only if the new configured
native actually matches those 486 values does the driver require Rust to match
them as well. A failed compatibility report is not relabeled a default pass;
the new full-100 configured comparison always remains mandatory.

Both actors use optimized builds with assertions retained. The native record
must contain actual `-O3 -UNDEBUG` compiler argv; the Rust record must contain
actual rustc `-C opt-level=3` argv with debug assertions and overflow checks
enabled. Rustc boolean spellings `yes`, `on`, `true` and `y` are accepted;
disabled, unknown, missing or repeated settings fail closed. This includes
Cargo’s canonical `on` spelling without changing the required semantics.
Neither permits fast-math. The small per-role build record contains `role`,
`executable_sha256`, `compile_argv` and `inputs_sha256` for actual canonical
compiler/source/build inputs. Native implementation setting sources and the
qualified SDK manifest are retained alongside compiler inputs, labeled as source
evidence rather than falsely described as observer compilation dependencies.
Use the established pinned SDK compiler/link/RPATH/loader recipe with this
observer and optimized flags; retain actual expanded commands, dependencies,
compiler, source and selected-library identities. Do not rebuild or swap the SDK.

An explicit known-role closure contains `role`, `executable_sha256` and `files`
(canonical DSO path to SHA-256). After the window, while the world remains alive,
the actor copies `/proc/self/maps`. The driver hashes the actual executable and
every file-backed ELF, refusing missing, extra, deleted, wrong-build or changed
selections. Anonymous/kernel mappings remain in the raw file and are not claimed
as hashed DSOs. Both roles must contain the qualified DART/ODE libraries and the
native role must contain the exact selected plugin. Build/source/closure hashes
are bracketed across the batch. Actual maps and per-role process output are kept
on every completed actor; timeout and failure output stays file-backed.

A future reviewed invocation supplies `--native`, `--rust`, `--plugin`,
`--native-build-record`, `--rust-build-record`, `--native-closure`, `--rust-closure`,
`--cpu` and a new `--output`. Each actor accepts XML, new output prefix and mode
`correctness|measure`; native additionally takes the plugin path first. Output
suffixes are `.input.sdf`, `.state.json`, `.metrics.json`, `.maps`, `.stdout`,
`.stderr` and `.process-time`. No old capture may be overwritten.

The synthetic tests in `tests/test_b03_physics_benchmark.py` exercise complete
100-frame schedule/ownership/width/finite validation, unsampled-step divergence,
configuration capability labels, actual logical work counts and missing/extra/
changed runtime selections. Synthetic zeros/ELF markers are metadata fixtures,
never physics goldens or valid executable proofs. Their actual execution outcome is
recorded below. Review and actual paired correctness/configuration evidence must
precede timing eligibility. Even a successful physics-only window ratio is not
a full Server, balanced robotics or overall rewrite acceptance ratio.

The initial consolidated qualification built both optimized actors and passed
all twelve protocol controls. The driver then refused the Rust build record
before any actor execution because Cargo elided the explicit overflow-checks
argument when debug assertions already enabled it. That failure is retained.
The affected continuation adds an explicit enabled overflow argument, preserves
the strict record guard, and reuses unchanged native/protocol evidence. No paired
correctness or timing result follows from the initial build-record refusal.

## First independently audited measurement

The completed local configured batch passed all eight paired correctness checks:
129600 scalar comparisons, maximum absolute difference `4.440892098500626e-16`,
none outside `1e-12`. Configured native output also matched the original-default
486 selected values exactly; Rust's maximum selected difference was
`2.220446049250313e-16`. This does not extend default compatibility beyond those
selected snapshots or change the explicit unlimited-pair setting.

After the excluded correctness/warmup pair, seven alternating measured pairs
produced these window-wall medians:

| Actor | Median window wall time |
| --- | ---: |
| Native public physics plugin | 11.743475 ms |
| Rust Scene step, synchronization and queries | 23.994811 ms |

Rust is **2.043246 times slower** in this bounded window. This baseline identifies
performance work; it is not a rewrite speedup. Whole-process peak RSS medians were
37552 KiB native and 23012 KiB Rust, including different setup/output costs; they
must not be combined with window timing into a single performance claim.

`reference/b03_physics_measurement.json` records the summaries, actual machine
context, source/binary/build/configuration bindings and archive member hashes.
The compact `reference/b03_physics_observations.tar.gz` retains all actual actor
outputs, timing records, maps, successful build records and independent audit.
No binaries, libraries or failed build caches are bundled.

The first affected continuation satisfied the explicit Rust overflow-argument
requirement. It then stopped before any actor because the selected SDK lacked
`/usr/bin/time`. A read-only GNU time executable mount supplied that standard
wrapper, whose hash was recorded before and after. The same unmodified driver
then executed the one successful batch. Native compilation and the twelve
passing protocol controls were not repeated. Independent data/SDK/timing audit
passed; normal staged-hook and exact hosted qualification remain separate
acceptance requirements. No further timing run is needed for this source batch.
