<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M7 accepted: bounded CPU lidar and measured scan visualization

Accepted and published on 2026-11-04 at simulator Main
`9a652063d85938143ac298f428b55dd68a7f22da` (tree
`29bd53a7673131ceb5e4be1e9504417638c4e391`) and viewer Main
`19be842d3c42c2f7b2c827e8fdd54c8603114acc` (tree
`3945879f83d79c55b89d22690448b502157cda62`). Required local gates,
[exact simulator CI41747544467](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41747544467),
[exact viewer CI41748284641](https://github.com/gazebosim-rs/gzweb-rs/actions/runs/41748284641),
and independent source, local/official data and actual browser image reviews
passed before nonforced Main promotion. M8 is now the current five-working-day
phase. This record's metadata follows the required normal hook/CI separately;
it does not replay accepted manual qualification.

## Delivered source and admitted domain

One loaded ideal planar CPU lidar uses maintained Rapier/Parry read-only shape
casts against admitted primitives and meshes. Directions are sensor-local XY,
zero along +X; authoritative pose and distance are f64 SI values. Actual current
body/local collider transforms are used without hidden stepping or cache/contact
mutation, and only the parent rigid body is excluded. Other backends explicitly
refuse sensor admission. No separate transport or message runtime is introduced.

The domain is 2–128 inclusive, strictly increasing horizontal beams spanning no
more than one turn; one zero-elevation layer, horizontal resolution 1, finite
ordered nonnegative ranges, finite nonnegative range-resolution metadata, default
no noise and the full visibility mask. Hits below minimum clip; a miss through
maximum is +infinity. Resolution does not invent quantization. Canonical
LaserScan carries acquisition time/frame/pose, fixed one-layer metadata and ideal
zero intensities, without an intensity or reflectance-model claim.

Loaded lidar configuration installs callbacks; demand controls acquisition and
sample buffers, and last-sensor removal retires callbacks. Absent families own
no lidar callbacks or scan buffers; explicitly empty interest performs no queries and
clears stale samples. Actual upstream Zenoh matching and opt-in viewer interest
are unioned before stepping. IO uses the existing bounded publication owner with
weak leases and epoch cancellation, keeping network waits outside stepping.
Pause, reset, removal and restoration preserve the documented acquisition rules.

The additive `scan=1` WebSocket channel atomically captures a Scene/scan pair,
sends Scene first, carries bounded latest-only measured data and maps no return
to JSON null. Default clients receive no scan frames. The viewer uses maintained
Bevy gizmos in the actual acquisition frame through the existing Z-up root once;
no scan scene nodes, visual assets or fabricated Core IDs are created.

## Independent source review and retained assertions

Physics/model/Scene/Server/viewer/CLI source and batched findings were independently
reviewed before consolidated checks. Exactly three prior Rapier CPU rejection
rows became stronger same-input positives: one 16-beam typed sensor and the two
2-beam lidar/ray aliases. XML, URIs, parsed assertions, case counts/order, all GPU
and wrong-wrapper negatives, and seven unsupported-engine exact diagnostics
remain. The shared positive helper adds real range/cadence/frame/pose/identity,
no-demand, pause, reset and teardown checks. Retained IMU/RGB/depth assertions,
numerical tolerances, feature matrices and quality gates are unchanged.

The first simulator gate failed after 24.006s on a new test's nonexistent Server
API. The next failed after 30.008s on a suspicious map/count helper. Both packets
remain; only the reported test setup/counting logic was corrected, independently
reviewed, and the mandatory affected gate rerun. The corrected fold validates
every actual reply. Neither failure is relabeled as passing.

## Local mandatory evidence — independently audited

Simulator final normal staged gate: 1180.316472s, exit 0, no capacity guard stop.
Independent retained-data audit verified all 2366 artifact hashes and 579 distinct
Git-bound source/input paths, complete ownership/projection and all four raw
profile merges. Separate whole-production line/region totals:

| Profile | Passing tests | Lines | Regions |
|---|---:|---:|---:|
| Headless | 1059 | 13269/13536 | 19982/20644 |
| Renderer | 670 | 7984/8154 | 11743/12231 |

Every production crate and workspace line/region result is strictly >95%, as are
parent-changed line gates independently in each profile. Tooling: 289 passing.
Five/four delegated helper tests remain explicitly ignored; they are not added
to passing counts. Changed headless: physics12/12, Rapier55/56, sensors169/169,
Scene270/279, CLI192/199, Server415/427; renderer Scene270/279 and CLI192/199.
Actual new analytic/mesh/read-only/reset ray controls, loaded sensor demand and
cadence, canonical output/lifecycle/queue/epoch and real opt-in socket controls
passed. Socket execution is distinct from the manual browser observation.

Original native-free10 and separate camera-feature9 controls passed with six
retained auxiliary ELFs. Required system-only/native-free/GPU-free headless
default closure controls passed; the separate offscreen worker intentionally
uses rendering and is not mislabeled GPU-free. Raw profile merges
were independently byte-identical (1747/354 full inputs and14/33 auxiliary inputs).
Nine existing C++ peers and24 actual local DSO files remain provenance-bound;
this is retained compatibility evidence, not new C++ lidar parity. Full-profile
286 ELF metadata records (142 headless/144 renderer) have absent original paths;
only six auxiliary executable copies are retained. No metadata-only binary claim
is made. The qualified native-free CLI actually used by the browser is retained
and rehashed, SHA256 `9f75e3fd293987396d33d07bbd73b4812b13b0084be55a7b043ce412204efc80`.

Viewer normal precommit87.019694s and normal prepush coverage/review push185.060307s
passed without guard stops. LCOV54686/56247=97.224740875%, independently recomputed
from223 unique source records, SHA256
`0c1772e2bb6935f088a3a77f6dbbd141038e666fca446c710656600e3d417606`.
Successful hook output does not retain an exact Cargo test total; none is invented.
The one maintained two-flavor build passed1710.407249s. Recomputed gzip -9 payloads
are WebGPU12,892,886 bytes and WebGL2 12,868,306 bytes, each below the unchanged
14,000,000-byte budget. Retained copied WASM/JS and source/build bindings were
independently checked; old accepted M2 served bundles remain byte-identical.

## Actual M7 browser evidence

The qualified native-free CLI ran the committed `examples/m7_planar_lidar.sdf`
on new local port9007 and a unique partition; no rebuild or M1–M6 manual replay.
The new-only m7-preview distribution uses the existing8081 server. Actual WebGL2
console identified ANGLE/NVIDIA RTX2070; no universal driver claim is made.

Actual UI screenshots show exactly two visual entities and three measured ray
segments/markers on the wall. Pause acknowledged epoch0 at46.361s and retained
the last scan. Reset acknowledged epoch1 at0, paused, and removed all measured
rays/markers while retaining geometry. Resume restored advancing time and the
fan; the saved resumed image is38.743s (an earlier thread-only observation was
20.791s). The review view is paused epoch1 at42.539s with the fan visible.
Image/CLI/fixture/served GL WASM hashes and exact captions are in
`browser-observations.json`; `browser-console.json` retains the console record; independent image/data/source review is CLEAR,
including all five PNGs, captions, actual CLI, fixture and served GL byte bindings.

The first non-PTY launch exited cleanly before attachment after390576 ticks;
relaunching the same ELF under a persistent PTY enabled the actual observation.
No product/startup fault or repeated test qualification is inferred from that
operational event. Browser images demonstrate actual visualization and control
lifecycle; metric range/cadence/protocol assertions are separate passing controls.

## Official evidence and publication

Simulator exactCI41747544467 attempt1 passed all16 steps. Independent official
audit verified all1267 Git blobs/modes and579 distinct source/input bindings,
including84 production sources and272 profile inputs; workflow/gates and the
35 feature-path delta match the qualified commit. Actual1059 headless,670 renderer
and289 tooling passes retain five/four explicit delegated ignored helpers.
Official headless coverage is13271/13536 lines and19985/20644 regions, two/three
more hits than local; renderer7984/8154 and11743/12231 matches local. CLI is
1267/1296 lines and1909/2000 regions in each profile. Every strict per-crate,
workspace and parent-changed gate passes separately; official changed Server
is417/427 versus local415/427, with other changed counts equal. No profile union
or fabricated equality is used. All four raw merges are byte-identical from
1747/358/14/33 official inputs; local renderer354 remains distinct.

Official CLI/worker Cargo handoffs/features/source and pre/post-test hashes are
bound; full286 Rust ELF records remain metadata only and six auxiliary copies
are actual retained executables. Nine existing C++ peer source/binary/provenance
bindings pass. The24 hosted SDK DSO paths are manifest-bound; their remote bytes
are not archived or locally rehashed. These are preserved reference controls,
not new C++ lidar parity. All earlier depth/RGB/support controls remain passing.

The official simulator artifact ZIP SHA256 is
`91b677f5ea0dbc4a6f84a2fee699ad22796d815d3a7c4a15cab0c1f12b418cdc`;
source ZIP is`bd76e86a6168b2c371248d145cdd701f4d96842a9f18b94740372f1ec6155497`;
independent audit summary is
`4ecbd8d48161a0dfb0c3c2ea89823df95292906472fd8db287a52ae2e4dbb0bc`.
The local stage-result SHA256 is
`784db80502b001a74fc1a4d6820ce4b2316b6dd28f9d9473222c77bb9a35434e`.

Viewer exactCI41748284641 attempt1 passed all six jobs and66 steps, with one
unchanged optional Fuel cache-save skipped. Actual1083 native tests pass with
five unchanged ignored helpers; separate Fuel3 and startup1 pass. Official
LCOV223 records give54685/56247=97.222963%, one fewer hit than local54686;
these remain separate evidence. Seven protocol/snapshot pairs pass with25
existing allowlisted differences. The allowlist and gates are unchanged.
All692 source Git blobs/modes and10 local build bindings were independently
verified. Hosted Wasm is debug compilation; no hosted release-bundle/raw-profile/
ELF retention claim is made. The independent viewer audit summary SHA256 is
`d4f39e5a1a81ee5010202773eb82033f8440d8199d3c2168d91ffcb3c7b84321`;
official LCOV/parity artifact ZIPs are
`a5bd4365a7729df2d8e849521b3c6a11e94f7b22eac0089dfee58d9b06a8e4be`
and`f43bf6e86b6f4bd13ab2f53c0022fe39fab49518f3c98cc31fceedc63728027b`.

The published parent M6 metadata0b9541dc also passed its normal1127.283s gate
and independently audited exact CI41740088744 attempt2/all16 steps. Attempt1
remains a preserved real failure: three original20-second camera readback
timeouts while another actual GPU/canonical delivery passed in the same run.
Runtime/test/input bytes were identical to the accepted M6 feature. The one
bounded same-SHA failed-job rerun preserved every deadline, assertion and job;
all three cases and the full profile then passed. Cause remains undetermined;
a successful retry neither diagnoses nor erases the failures. No retry-until-
pass loop, timeout increase or accepted manual replay occurred. Attempt2's
separate artifact ZIP SHA256 is
`960a9b1fd14294f02190863a338ec3a1a1a13d07136db814294c4ceaab374baf`.

Simulator Main advanced7fcc→0b954→9a652063 without force only after the required
audits. Viewer Main advanced5b67→19be842d after its exact normal pre-push coverage,
bundle, official and browser qualification. The identical checked commit's
proof was reused during ref promotion; no passed workload was replayed by the
coordinator. Normal automatic Main CI remains governed by the unchanged workflow.

## Scope limits

No GPU lidar, noise parity, multilayer scans, point clouds, intensity model,
additional sensor family, new geometry formats, C++ lidar parity or performance
speedup is claimed. Later deliveries remain named bounded phases under D30.
