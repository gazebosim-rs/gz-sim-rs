<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M10 acceptance: interactive world lifecycle

M10 runtime is **accepted under D30 on 2026-11-05 UTC**. Both public `main`
branches now contain the exact reviewed and qualified commits below, promoted
without force after normal hooks, exact review-ref CI, independent source/data
and release-bundle audits, and actual matching browser evidence review.

| Repository | Published commit | Qualified tree | Exact successful review-ref CI |
| --- | --- | --- | --- |
| Simulator | [`d3967b6178e8cba2056ba744f38499d03833edd0`](https://github.com/gazebosim-rs/gz-sim-rs/commit/d3967b6178e8cba2056ba744f38499d03833edd0) | `e919a00f995025ce64e03eefba9ffa5e42fc0fe1` | [41924068764](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41924068764), attempt 1, all 16 steps |
| Viewer | [`952a520265bd349c9ca015f1a2007ab835e313ca`](https://github.com/gazebosim-rs/gzweb-rs/commit/952a520265bd349c9ca015f1a2007ab835e313ca) | `212a87b21e132be2b9b3b91c7092956627473073` | [41937814296](https://github.com/gazebosim-rs/gzweb-rs/actions/runs/41937814296), attempt 1, all six jobs |

The simulator normal local hook passed in 1431.357 seconds. The final viewer
normal commit and pre-push hooks passed separately, as detailed below. The
retained `artifacts/m10-delivery/main-promotion.json` binds the actual API
fast-forward responses and independently read-back public refs; its SHA-256 is
`6dc4e59fdb3a48cde5e07aaaabf359d660caa797e78314cd0f0ef19810fb453b`.

This document records accepted runtime evidence. It does not supply its own
canonical metadata commit's normal-gate or exact-hosted result; those results
are recorded independently by the required qualification pipeline. Automatic
Main workflows are separate from the successful review-ref runs above; no
passing result for those separate workflows is inferred here. See the
[bounded contract and launch commands](M10_WORLD_LIFECYCLE.md) and
[delivery plan](DELIVERY_PLAN.md).

## Runnable scope and limits

The explicit `--web-lifecycle` server option and `lifecycle=1` primitive viewer
expose one loaded Rapier world's declared primitive/local-glTF templates. The
browser supplies no XML, paths, URLs or downloads. Editing admits standalone
single-link/collision/visual models with identity local link frames; models
owned by joints, sensors or controllers remain read-only. Camera-owned mutable
scenes are refused. Existing IMU/RGB/depth/lidar/controller operation and
native-free/GPU-free defaults remain part of the retained regression suite.

Spawn reuses maintained native body/shape construction and reserves four public
IDs before commit. Local mesh visual bytes and aliases are captured at startup;
runtime mutation performs no resource download or mesh decoding. Remove retires
the native body, owned entities and visual aliases. High-water IDs are never
recycled. Full Scene and catalog bounds are checked before ordinary mutation,
and rejected requests leave unrelated native bodies and public identities alone.
Corrupted exclusive ownership is a terminal invariant error, not an ordinary
atomic-refusal result.

Read/apply state uses the model/canonical-link origin in world Z-up metres,
xyzw unit quaternions, world origin velocity in m/s and angular velocity in
rad/s. Maintained native COM state supplies reads and writes with the existing
COM-to-origin conversion. Explicit entity-state editing is not an actuator or
custom physics solver. Fixed bodies require zero velocity. Reset restores loaded
state and retires runtime additions and aliases without recycling IDs.

The optional catalog owns the model-to-visual map. Requests carry world/epoch
fences and monotonic request IDs; controls and edits share one pending slot and
the existing eight-slot ingress. UI changes follow server Scene/catalog/ACKs,
not optimistic local insertion or mutation retries. Committed replies retain
the exact Scene sequence, state, catalog and optional measured scan even when
Latest advances. Reconnect obtains a current full snapshot and clears stale
client selection/pending outcomes.

## Publication backpressure correction

The original qualified candidate failed actual browser acceptance: the separately
qualified diagnostic viewer recorded event queue full, requested close 4002 and
a close event of 1006. It did not qualify stable editing or reconnect. That failure
and all original sources, bundles and observations remain preserved.

Only clients additionally negotiating `lifecycle_credit=1` use publication
credit. One coherent normal Scene/catalog/optional-scan tuple is followed by a
strict version-1 `publication_credit` token. The exact `publication_ready` echo
allows the next normal tuple. Tokens are checked nonzero, monotonic and local
to a connection. The producer retains bounded Latest while waiting; physics and
command handling do not wait for the browser or GPU.

Credit is independent of simulation request IDs and the pending-command queue.
Ready is parsed while control/lifecycle replies are pending. Priority committed
replies and control ACKs neither consume nor replace an outstanding normal token.
An already validated Ready survives same-connection reset clearing and is sent
once after successful inbox flush. Malformed/stale/duplicate credit or bounded
send failure closes the connection without mutation retry. Existing no-credit
protocols, the 64-event queue/drain, frame bounds and all retained assertions are
unchanged.

## Simulator local verification

The complete local pass has 1,204 headless, 760 renderer and 289 tooling passes.
Eight new credit functions run once in each profile. Controls include strict
JSON/token domains, checked exhaustion, query/default negatives, real paused
and advancing native worlds, pending-command ordering, exact delayed replies,
reset before old Ready, and a real bounded 64-event TCP consumer that withholds
application polling while the native world continues updating.

| Profile | Local lines | Local regions | Hosted lines | Hosted regions |
| --- | --- | --- | --- | --- |
| Headless workspace | 17613/18013 | 26102/27002 | 17611/18013 | 26100/27002 |
| Renderer workspace | 10985/11255 | 16080/16763 | 10985/11255 | 16080/16763 |
| CLI, both profiles | 2038/2097 | 2975/3124 | 2038/2097 | 2975/3124 |

Each owned crate and separate workspace profile passes the unchanged strict
line/region gates; changed CLI lines pass separately at 76/78. No hit-set union,
threshold reduction or numerical tolerance change is used.

Independent local audit checked 2,390 artifact hashes and 625 Git bindings,
including 101 production files and 272 profile inputs. All 1,320 protected parent
blobs/modes are unchanged. Four raw merges from 1762/363/14/33 inputs are byte-exact.
The 19 auxiliary cases, six actual auxiliary ELF copies, nine C++ peer builds
and 24 local SDK DSO files retain their stated provenance. The 296 normal-profile
ELF records are metadata-only; 29 unchanged helper source paths/55 records are
Git/Cargo-bound outside explicit input maps, while 241 records have those maps.
RGB/depth/support controls remain separate; no new C++ parity or speedup is claimed.

The complete 403,780,338-byte RAM packet remains intact. A durable verified
24,459,152-byte tar.xz copy preserves every 2,424 entry name/type/mode/UID/GID,
file byte and link relationship, plus all 2,390 artifact bindings. Local audit
SHA is `006aa1f3fa5137456c1186cb832491d12f1847f1c6b306bc2691c95462d065a0`;
archive SHA is `b8db613be1a08ef07b83a20e8806c3dcf6b7837454848412a0f8353b7f7dda33`.

The earlier unchanged-tree run stopped at the retained scratch floor during
renderer build preparation. Its 1,204 headless passes and partial evidence remain
separate; no renderer/full pass is inferred from it. Capacity was repaired by
byte-preserving relocation of regenerable third-party cache files, with original
logical paths retained. The physical SSD/scratch/memory floors and required
checks were unchanged.

## Simulator official verification

The official audit binds the exact commit/tree, 1,326 source blobs/modes and 625
source/input bindings. It confirms 1,204 headless and 760 renderer passes (five
and four unchanged ignored/delegated entries respectively), 289 tooling passes,
all eight credit controls in each profile, separate strict whole/changed gates
and four byte-exact raw-profile merges (1762/367/14/33 inputs). Local and hosted
counters remain separate; their two headless line/region hits differ without a
hit-set union or an equality requirement.

Official audit: `hosted-41924068764/audit-summary.json`, SHA-256
`4c949070438855163d66e0ab9550e12ef4c0abe5ff8f14191f85f53b9ebd2da3`.
The original three ZIP byte streams total 107,969,117 bytes and are durably
retained inside a verified 87,719,584-byte lossless tar.xz container; individual
ZIPs also remain in RAM, not as individual SSD ZIP files. The audit records the
container identity and restore command. The full-profile Rust ELFs remain
metadata-only; six auxiliary ELFs and existing C++ peer binaries are actual
retained bytes. Twenty-four remote native DSO identities are provenance records,
not archived remote DSO bytes. These boundaries are not claims of new C++
lifecycle parity, universal GPU support or performance.

## Final viewer qualification and release bundles

The accepted viewer includes the four-path timeout-diagnostics/queued-ACK correction on
qualified publication-credit parent `138f5759`. Its normal commit hook passed
in 81.022 seconds and ordinary pre-push coverage passed in 152.054 seconds,
without a capacity stop. Independent local audit SHA-256:
`091391f29d6a32f1c6a3ae921787d549b7df165a4e9c8b6e2b881e3e25e1777a`.

Exact hosted [41937814296](https://github.com/gazebosim-rs/gzweb-rs/actions/runs/41937814296),
attempt 1, has six successful jobs, 66 successful steps and one unchanged optional
Fuel-cache skip. Official evidence records 1,109 native tests passed, zero failed,
five unchanged ignored tests, plus three Fuel and one Node startup control
separately. Seven protocol/snapshot parity pairs retain the 25 existing allowlist
suppressions without new unsuppressed differences. Official audit SHA-256:
`8d24af29235cd18a2d4caf1b629f2498749cd81a300ec6a9db241a0280bd298c`.

Viewer local LLVM line coverage is **55343/57129**; official is **55347/57129**
over 225 source records. Both pass the unchanged viewer **workspace line coverage
>=95%** policy. This is not the simulator's separate per-crate line/region policy.
Unique physical DA counts remain separate from LLVM LF/LH; neither denominators
nor local/official hit sets are substituted or combined. Successful local hook
output does not retain individual Cargo test totals; official totals do not fill
that retention gap. Official artifacts retain LCOV/parity reports and debug
Wasm compilation evidence, not raw profiles, coverage ELFs or release bundles.

One completed current-source both-flavor release build passed in **1121.776
seconds**, with unchanged **14,000,000-byte gzip budget per flavor**. Retained
stdout reports WebGPU 12.92 MB gzipped and WebGL2 12.90 MB gzipped; these are
rounded stdout values, not newly measured exact gzip sizes. Raw Wasm identities:

| Flavor | Raw bytes | SHA-256 |
| --- | ---: | --- |
| WebGPU | 47937450 | `726957911d18f302081075560cb8b260e6ad7767c7f44c4018b73f680291a3de` |
| WebGL2 | 47842883 | `27d23a1b8512941121374e6cfdaec03594b11aceb07af76e4db7fedd34993752` |

Manifest `viewer-timeout-diagnostics-wasm-build/preview-manifest.json` SHA-256:
`201fdf60c17723bd9a5ab193d217b6253d5e4c7f03225ffc4b3ba67d9fa39ae3`.
Independent bundle audit SHA-256:
`2a4c02ae90d0f95dfa945f6c23a9b8d6743ca0623bc8ed97b5b637d53496e188`.
It verifies all six produced/manifest/RAM/served-filesystem/HTTP-200 byte streams,
23 source/lock bindings and pre/post clean Git identities, and all 42 preserved
parent files/link bindings. M7, e276, 744 and 138 previews remain unchanged.

The new `m10-timeout-preview` uses tiny SSD index/startup copies and an absolute
target symlink into owned RAM. Its six-file RAM preview is ephemeral; it is not a
claim of durable release-bundle storage. The actual browser used this WebGL2
bundle against unchanged qualified native d396 on port 9011. No WebGPU runtime,
new performance rate or manual M1–M9 qualification replay is claimed.

## Actual matching browser operations

`timeout-browser-observations.json` SHA-256
`1bd68d1ef0f3e9ff366062b78403dca82914af3f0edcaff1a921d3a11bcf59c9`
binds the exact simulator/viewer, launch/fixture and release manifest. Thirteen
saved PNGs and two retained console records received independent saved-evidence
review CLEAR in `browser-audit.json`, SHA-256
`2db58056f9b63e37898bc8e90517fdd3e808f95c08f434a44d8cf2b3fea5c9e6`.
This review performed no browser replay. Root's recorded operations are:

| Operation | Actual receipt / visible result |
| --- | --- |
| Box spawn 18 | Spawn #1, epoch 0, time 0, sequence 99717; runtime_18 and blue box visible. |
| Read box 18 | GetState #2, epoch 0, time 0, sequence 101134; authoritative z=3 and Apply enabled. |
| Apply box 18 | SetState #3, epoch 0, time 0, sequence 103098; x=4,z=3,vx=1 edits and visible relocation. |
| Resume / pause | Actual motion and advancing clock, then committed epoch 0, 21.745 s, paused=true. |
| Remove box 18 | Remove #6, epoch 0, 21.745 s, sequence 106777; added row/visual retired, original four retained. |
| Sphere spawn/remove 22 | Spawn #7 sequence 110233 and Remove #8 sequence 111203 at epoch 0, 21.745 s; orange sphere/row appeared then retired. |
| Local mesh spawn 26 | Spawn #9, epoch 0, 21.745 s, sequence 112869; green runtime mesh visible. |
| Reload / reconnect | Fresh snapshot at 21.745 s retains IDs 2,6,10,14,26; no mutation resent or duplicate addition. |
| Remove retained mesh 26 | Fresh-connection Remove #1, epoch 0, 21.745 s, sequence 116075; row/mesh retired. |
| Fresh mesh 30 | Spawn #2, epoch 0, 21.745 s, sequence 117766; IDs 18/22/26 not reused. |
| Reset with mesh 30 alive | Committed epoch 1, time 0, paused=true; runtime mesh retired and only original IDs 2,6,10,14/placement restored. |

The genuine initial/reload idle warnings at 00:25:54.886Z and 00:41:11.882Z
reported 11.954 and 14.959 seconds without a server message, no control pending,
last_credit=Some(1), pending_ready=None. Both connections automatically recovered.
Their cause remains **UNKNOWN**; a locally submitted Ready is not proof of native
receipt. The historical bottom warning did not describe the later successful
panel receipts. The document was visible; 1-fps display is not evidence of a
background-tab cause, a GPU cause or a performance rate. Neither these warnings
nor the earlier confirmed queue overflow is erased by later success.

Images/captions establish visible geometry, rows, controls and displayed receipts.
The separate console, qualified CLI/fixture and release-byte bindings establish
provenance. Numerical state/frame tolerances, COM conversion, rejection atomicity,
exact packet ordering, bounds and failure ownership are established by retained
regressions, not pixel measurements or screenshot-derived wire counts.

## Preserved failures and operational/reviewer limits

The retained simulator history includes early compile/lint corrections (private
queue reachability, explicit test import, fixed-array cases), the first functional
pass that failed CLI region coverage at 2842/3024, and its meaningful-regression
completion. Those are real failed/unfinished gates; they do not inherit the final
pass. Hosted original attempts failed signed Ubuntu snapshot HTTP503 and selected
Zenoh reference setup before code qualification; unchanged retries and their
specific completed results remain separate. The publication-credit same-tree
1188.285-second capacity stop during renderer preparation is a partial result,
not a renderer failure or full pass.

Viewer history remains separately retained:

- First normal hooks failed at 38.010 seconds (qualification/last-use clone/marker
  Default), 25.005 seconds (formatting and test initializer lint), and 46.010
  seconds (two unreachable-public items). Source-observed same-class fixes are
  distinguished from emitted diagnostics; old assertions and gates were retained.
- 936ab148's first pre-push failed at 96.023 seconds on the new asset-retirement
  assertion (393 other gz-bevy tests passed). A normal next frame observes Bevy's
  maintained PreUpdate asset cleanup; exact counts and retired/current IDs remain
  asserted. e276's corrected normal hooks, official run and release build passed.
- The e276/744 actual browser failure was confirmed as event queue full, requested
  close 4002, close code 1006. 744 added bounded observability, not queue capacity.
  Its successful qualification/build is historical and does not prove stable editing.
- Credit parent 709's first pre-push failed at 95.023 seconds on a new reset fixture
  expecting UI admission before the fresh epoch Scene (400 other gz-bevy tests
  passed). The test-only correction retained all prior assertions and explicitly
  exercised the actual reset fence. Corrected 138 hooks/CI/build passed. Its watcher
  HTTP504 was a polling failure, not a CI failure or reason to replay that successful run.
- 952a separates opening/initial-Scene/idle diagnostics from actual pending-command
  unknown-outcome errors and fixes the independent queued-ACK deadline race. It
  processes accepted/refused replies before retiring their old timer; new commands
  get a fresh timer. Existing 10-second connection/idle and 7-second ACK thresholds,
  credit, queue and protocol bounds remain unchanged. This does not diagnose the
  observed idle silence or establish why automatic recovery occurred.

Resource stops and repairs remain operational evidence. Approved lossless
archives and reversible third-party cache relocations preserved original bytes,
logical paths and restore information; source, proofs, ELFs and served bundles
were preserved, and resource floors were not lowered. Pre-execution launcher
review tightened Git identity, compiler inventory, explicit 14M budget and
post-build binding. Those were review findings, not emitted product failures;
an output-path precondition refusal performed no build workload.

The final viewer audit retains an unsupported reviewer separate-SF assertion and a
sandbox namespace invocation refusal (host RAM was invisible). They were reviewer
adapter/invocation errors, not product or CI failures; the corrected data-only audit
ran in the host namespace. LF/LH and physical DA remain explicitly distinct. No
passing claim is inferred from an interrupted audit or from a missing artifact.

## Acceptance boundary and metadata closeout

Independent saved-browser review is CLEAR and both exact runtime commits are
published on `main`. This fulfills the bounded M10 runtime delivery, including
primitive and one local mesh spawn/remove, authoritative state read/apply,
reconnect retention and reset retirement. It does not complete broad P0–P7
compatibility or turn ten named deliveries into an effort-completion percentage.

Canonical acceptance metadata and tracker publication follow their normal
required checks and exact CI. This runtime record supplies no passing result for
that later metadata commit or for automatically triggered Main workflows. No new
runtime build or manual browser qualification is inferred from a documentation
change, and no later-phase implementation is included in this acceptance.

Mesh format expansion, arbitrary imports, articulated editing, renderer-scene
mutation, multiworld, levels and distributed stepping remain deferred. Zenoh is
upstream Rust and unmodified; canonical Messages remain schema-only.
