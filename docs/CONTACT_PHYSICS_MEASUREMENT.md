<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Contact-factorization physics-only measurement

One audited batch measured simulator
`f136e8211559dfdd831d39ce73e8132e413404a4`, tree
`cdb53ed377800fe5dee6c349601b0dfd4efacd6b`, with producer
`f0d05146300e31237d0cf0a6024fdfd9b0ef4226`. That runtime revision is accepted.
This appendix is being published alongside a later producer adoption; neither
that later pin nor the publication revision was measured by this batch.
Qualification of the combined publication remains separate.

The measured workload is the existing
[configured physics-only B03 contract](B03_PHYSICS_BENCHMARK.md). It excludes
Server, network, IO, sensors and rendering; Rust includes Scene synchronization.
The exact B03 input, physical ownership, ODE/Dantzig selection, explicit unlimited
native pair limit, source-established global limit 10000, requested 1 ms steps
and comparison bound `1e-12` remain unchanged. This is not a product-default
change or a full-Server comparison. Unsupported solver iteration counts remain
null; source-established settings are not relabeled as public readbacks.

## Observed batch

Both roles preallocate storage outside the window. Each window contains 100
steps, 700 complete fresh body rows of 22 fields and 400 two-scalar joint rows.
Setup, formatting, files, maps and destruction are outside the elapsed window.
One excluded correctness/warmup pair precedes seven alternating measured pairs
on CPU 0, an AMD Ryzen 9 5950X, in the same pinned `cb921` SDK image.

| Median elapsed window | Earlier accepted batch | Contact-factorization batch |
| --- | ---: | ---: |
| Native public physics | 11.743475 ms | 11.673473 ms |
| Rust Scene | 23.994811 ms | 17.553577 ms |
| Rust/native ratio | 2.043246 | 1.503715 |

The Rust elapsed median was **26.8443% lower** than the earlier recorded Rust
median, calculated as `(23.994811 - 17.553577) / 23.994811`. Rust remained
**1.503715 times slower** than its paired native actor. These are two batches on
one host, without confidence intervals or cross-machine evidence; they do not
establish general rewrite or full-Server speedup. GNU time CPU/RSS observations
retain their whole-process scope and recorded precision, separate from the
elapsed step/query window.

All eight pairs matched all 100 frames: 129600 scalar comparisons, maximum
absolute difference `4.440892098500626e-16`, none outside `1e-12`. Per-role repeat
checks, settings, 30 physical identities/parents, actual successful work counters
and all 16 post-window library maps passed. The separately checked default
reference contains only the 1/10/100 snapshots, 486 values: configured native
matched with zero difference and Rust with maximum `2.220446049250313e-16`.
That selected compatibility check is not all-step default-Server qualification.

## Exact records and build view

[contact_physics_measurement.json](../reference/contact_physics_measurement.json)
contains the statistics, measured source identities, hardware, proof hashes and
complete member hashes/sizes for
[contact_physics_observations.tar.gz](../reference/contact_physics_observations.tar.gz).
The 1,853,490-byte archive has 141 safe relative regular members: all 113 actual
paired files plus 28 compact context, commands, build/closure, recipe, audit and
failure records. Members preserve actual bytes. It contains no executables,
libraries or caches. Its SHA-256 is
`199f209dcde417e68995b1bdc58a24c5134909361146a506699084390359a33c`.

The native O3/assertion-enabled executable and its original build inputs/closure
were reused unchanged. Only the Rust actor was rebuilt with opt-level 3,
assertions and explicit overflow checking. The build reused baseline Cargo paths
through three declared read-only file projections: candidate `bridge.cc`,
`joint_state.hh` and `reference/b03_physics.json` over their original build-root
paths. The archived context records both physical and virtual paths and hashes;
tracked source/index cleanliness was checked before and after. These are actual
build-view substitutions, not source-URI equivalence or a claim that baseline
host files changed. The measured producer remains `f0d05146`.

The Rust build's input table includes a cache superset of release `.rlib` files.
The data audit separately identifies selected actor/DART `--extern` inputs,
direct C++ source/header hashes and generated native objects/archive. It does
not invent complete C++ compiler argv or a transitive C++ header trace. Three
DART `.rmeta` externs were not directly recorded; their exact bytes were proven
as the `.rmeta` sections of `lib.rmeta` ELF members in matching, recorded,
measurement-bracket-hashed `.rlib` archives. Maintained `ar`/`objcopy` tool
identities and archive/member/section hashes are retained. This is explicit
archive-section provenance, not a separately recorded historical `.rmeta` hash.
Historical baseline Rust cache files were not rehashed as unchanged originals.

## Qualification and retained failures

The corrected normal hook passed in 644.069 seconds: 927 headless, 546 renderer
and 254 tooling checks, with strict per-crate/workspace coverage and all 39
inherited cold spans preserved. Its retained audit is `ebe93937…`.
[Exact hosted run 41497378596](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41497378596)
and official source/profile/native audit `a5aa91a5…` also passed for `f136e821`.
The separate candidate data/SDK audit `c62aa191…` checked the actual batch;
hosted CI did not replay timings. Complete hashes and reports are in the archive
and metadata.

The initial 316.033-second hook failure stopped at the historical native-header
binding before replay compilation. The original oracle and exact old header
were preserved, with explicit historical source projection; the candidate Rust
controls still compare against the unchanged numeric oracle. The implemented
successful exporter retains source-bound passing logs, not that standalone
seven-mode replay's temporary ELF/dependency/maps directory. No unavailable
remote raw or live-library rehash is claimed.

The first read-only measurement audit then refused the three `.rmeta` paths;
its source, stderr and result remain distinct from the corrected section-binding
audit. No workload ran again for that correction. Earlier baseline pre-actor
overflow/GNU-time stops and snapshot HTTP prerequisite failures remain historical.
The original 119-member baseline archive is unchanged at SHA-256
`a941e96b430504abd6229c3ac5e2a83f53397dd7603502f568307829ea586cb5`.
No new trajectory golden, tolerance change or second timing batch accompanies
this appendix.
