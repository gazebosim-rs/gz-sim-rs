<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M11 acceptance: focused local recording and replay

M11 runtime is **accepted under D30 and published on 2026-11-05 UTC** after
normal required checks, exact review-ref CI, independent source/data/bundle
reviews, completed actual browser evidence review and direct owner approval.
Both Main refs were fast-forwarded without force and independently read back.

| Repository | Published commit | Qualified tree | Exact successful review-ref CI |
| --- | --- | --- | --- |
| Simulator | [`182773f7a31922eb69eaa3d3fcbbce4f5af3714b`](https://github.com/gazebosim-rs/gz-sim-rs/commit/182773f7a31922eb69eaa3d3fcbbce4f5af3714b) | `855081a75c59f04e3d35398c8daeb87af5d8d58f` | [42103914738](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/42103914738), attempt 1, all 16 steps |
| Viewer | [`c40418cd85e21db2bd0271919c90eda0c1184ece`](https://github.com/gazebosim-rs/gzweb-rs/commit/c40418cd85e21db2bd0271919c90eda0c1184ece) | `c043069149de1a35d1fe1cb28241a4dcb56f81a8` | [41968630547](https://github.com/gazebosim-rs/gzweb-rs/actions/runs/41968630547), attempt 1, all six jobs |

Publication records are `artifacts/m11-delivery/native-main-promotion.json`
and `viewer-main-prepush-gate/qualified-result.json`; the viewer's additional
ordinary Main pre-push passed in 151.069 seconds with exact remote readback.
The direct owner approval is retained separately. Formal runtime acceptance is
recorded at `2026-11-05T23:42:38.917778+00:00` in
`artifacts/m11-delivery/runtime-acceptance-decision.json`; the earlier native
promotion request timestamp is not treated as completion. Automatic viewer Main workflow `42130130009`, attempt 1, separately completed
successfully with all six jobs at exact `c40418cd`, as authoritatively observed by
Root. Automatic simulator Main workflow `42130320961`, attempt 1, also succeeded at
exact `182773f7`; verify-core job `127604984873` completed on 2026-11-06 at
00:23:06 UTC. Root retained the authoritative result in
`native-main-ci-42130320961-state.json`. These separate successful Main runs do
not replace the audited review-ref proofs or qualify this metadata.
This document does not supply its own metadata commit's normal-gate or exact-CI
result; the required qualification pipeline records those independently before
metadata publication. See [the bounded contract and commands](M11_RECORDING_REPLAY.md)
and [delivery plan](DELIVERY_PLAN.md). The contract's earlier implementation-status
intro is historical; this acceptance record states the qualified runtime result.

## Runnable scope

An explicit `--record-scene` directory captures one admitted primitive/local-glTF
scene, sampled poses and committed controls from startup. `--replay`
serves that finalized local recording read-only to the opt-in maintained viewer,
including its captured immutable assets. Playback owns no native physics world
or sensor runtime. Play/Pause control the cursor and Reset restarts the archive;
recorded requests remain provenance, not injected historical mutations.

The admitted stream is at most 64 MiB, 4,096 frames and 256 KiB per line, with an
eight-entry writer queue; immutable asset limits are 128 files, 16 MiB each and
64 MiB total. Malformed, incomplete, escaping or changed resources are refused.
Consumer credit preserves ordinal/lifecycle order with bounded state; slower
wall-clock presentation does not change recorded integer timestamps. No network,
filesystem or GPU wait is added to a physical step. Disabled recording owns no
recording writer/queue/buffer. This bounded result does not claim full sensor or
historical Gazebo-format recording, arbitrary seeking or deterministic physics
re-execution. Existing live behavior and default protocols remain retained.

## Qualified source and retained evidence

The corrected native runtime is `182773f7a31922eb69eaa3d3fcbbce4f5af3714b`, tree `855081a75c59f04e3d35398c8daeb87af5d8d58f`, parent `5c2a36dd8a755b4cea592b8d6d4a39fd219be2f3`. Its normal consolidated commit hook passed in 1487.403540 seconds with no capacity stop. Local retained-data audit `local-roundtrip-audit.json` has SHA `553e6fc2c26d79be37a31ee8022f37ee99091881bbfb1ccd34d1f21b16b8f878`; exact official run [42103914738](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/42103914738) passed, and `hosted-42103914738/audit-summary.json` is independently CLEAR (SHA `80b3d6bf78a8f042dee08131f3523b432ac21ac83539c0d9aed7ddbf6d14f392`). Both strict headless and renderer profiles retain their separate denominators and policies; no cross-profile coverage union is asserted. All 39 prior M11 controls and three added bit-preservation controls were checked per profile.

Actual simulator totals are **1,246 headless, 802 renderer and 289 tooling
passes**, with five/four unchanged delegated entries in the two profiles. All
42 new M11 controls ran once per selected profile. All owned crates and separate
workspace profiles pass unchanged strict line/region and parent-changed gates.

| Profile | Local lines | Local regions | Hosted lines | Hosted regions |
| --- | --- | --- | --- | --- |
| Headless workspace | 20490/21018 | 31455/32594 | 20488/21018 | 31452/32594 |
| Renderer workspace | 13862/14260 | 21432/22355 | 13862/14260 | 21432/22355 |
| CLI, both profiles | 4915/5102 | 8327/8716 | 4915/5102 | 8327/8716 |

Parent-relative changed lines pass separately at 109/109 in each profile. The
hosted headless difference is two line and three region hits in Server error
paths, not a lowered threshold or union of runs. The audit binds five correction
paths against qualified parent `5c2a36dd` and 35 cumulative M11 paths against
accepted metadata `135128e4`; Cargo.lock is unchanged by the correction. Actual
pinned `serde_json` selection includes `float_roundtrip` in both full profiles
and both CLI-specific auxiliary configurations, rather than inferring it from
unrelated package features.

Independent local audit binds 2,390 artifact hashes and 645 Git/source-input
bindings; official evidence binds 1,348 source blobs/modes and the same 645
bindings, with 2,395 artifact files. Four local raw-profile remerges are byte-exact
from 1762/363/14/33 inputs; official counts are 1762/367/14/33 and all four
remerges are separately byte-exact. The original 10 auxiliary cases/four copied
ELFs and separate RGB/depth 9 cases/two copied ELFs retain actual bytes and closure
proofs. The 296 normal-profile executable records are metadata-only, not retained
Rust ELFs. Twenty-nine inherited helper-source paths/55 object records are
Git/Cargo-bound outside explicit source maps; 241 object records have those maps.
Nine C++ peer binaries and local library identities are retained separately;
24 remote SDK DSO identities and 36 manifest matches are provenance, not archived
remote DSO bytes. No new C++ recording parity or performance result is claimed.

The actual retained native-free CLI ELF has SHA `30578f6def651d611549d47831164a1a96663012e3ac74f1bb00762eef3790bd`; its maintained `serde_json 1.0.149` Cargo event includes `float_roundtrip`. The official original ZIP bytes are retained inside the verified lossless container `hosted-42103914738/original-zip-files.tar.xz`, SHA `4a734e1be8d7b0640d31aff313309d5f13395002fb2c18089a47dcf5a3b4cf83`: 90,176,852 container bytes, three members, 110,605,425 original ZIP bytes verified. Metadata-only normal-profile executable paths are not substituted for the actual retained native-free ELF.

The viewer remains unchanged at `c40418cd85e21db2bd0271919c90eda0c1184ece`, tree `c043069149de1a35d1fe1cb28241a4dcb56f81a8`, parent `952a520265bd349c9ca015f1a2007ab835e313ca`. Normal commit and pre-push hooks passed. [Official run 41968630547](https://github.com/gazebosim-rs/gzweb-rs/actions/runs/41968630547) passed all six jobs; independent official audit SHA `88f503c12fb5c5f46cb77099d36dc8ec83c551e3bac58cafd790ffdc6b90ce7f` binds 705 Git blobs/modes, nine new cases, 1118 passing native tests, zero failures, five unchanged ignored tests, and separate Fuel 3 / Node 1 controls. Local LCOV is 55,568/57,351; official LCOV is 55,571/57,351. The viewer policy is unchanged workspace LLVM line coverage >=95%; this is not the simulator's per-crate line/region/changed-line policy. Physical DA counters remain a separate view, not an alternate denominator.

The single current production both-flavor build passed in 1124.9224455 seconds. Actual rounded gzip stdout reports WebGPU **12.93 MB**, WebGL2 **12.91 MB**, each under the unchanged 14,000,000-byte gate; no exact gzip count is inferred from rounded text. Independent bundle audit SHA `c1121a2fb0d9690821177f37cc0b44e71991bad20cb1a6f64b5ddb9dacbdd62a` binds all six outputs/served bytes, 27 source/lock bindings, and all 54 preserved parent preview files. The WebGL2 WASM SHA is `3d550b658a1c305c07d7e3abeff121b4beb16ecd4e2f1d15b1eea85fbdfe00bc`. The preview is RAM-backed; no historical preview was overwritten. Production bundle qualification, debug CI Wasm, and actual browser observations remain distinct evidence.

## Original capture and exact same-input comparison

The original capture at `/dev/shm/gz-sim-m11-actual-qja3b8iu/recording` completed cleanly: 1129 frames, nine actual control outcomes, 3,328,970 stream bytes. Stream SHA is `2436c361b76f62cc4962bf3767e2ef591797548af6174007d8bc9b752ae39159`; manifest SHA is `8f2faf5a90ca73e9a7c8e52a4776d2eb1126422ab8e5031625e428613a00a1db`. Independent capture audit `actual-capture-audit.json` SHA `8dbf54c3a66146a4d2985b2f017188f4531e137de7c9bd9e44d44a4c31c22728` binds source, assets, raw outcomes, screenshots and retained ELF.

Capture included box 18 and mesh 22 spawn, positive advancing clock/motion, both explicit removals, and epoch-1 reset to zero/paused/original placement. Both runtime models were removed before capture reset, so this recording does not prove reset retiring a still-live runtime model. Original asset server port 46815 stopped. Playback must serve the captured three immutable resources—`pyramid.gltf` (1753 bytes), `pyramid.bin` (384), `pixel.png` (68)—through the new proxy, without relying on that stopped original server.

The corrected comparison used the **same original archive and unchanged comparator** SHA `7a194a2627a9c0526c2f317b8758635de3459e90af1ca84c86b4959e3aafc5d3`. It passed once: 1129 distinct source frames, 1132 observations, recorded elapsed 49.974 seconds. Independent source/method/data audit `roundtrip-observer-audit.json` SHA `34b1f3b8d95608e7f5501fc43c95118b8b3773baa4d69c537309d5e5854bc61b` binds the success, original asset bytes, source clocks, catalog/status/outcomes and credit/ACK observations. The maintained comparator's `deepStrictEqual` remains unchanged. Permitted normalization is limited to transport epoch/sequence/paused and the same scoped loopback asset origin.

The trace retains Scene hashes, ordinals and clocks, not full Scene payloads or independent per-message wall timestamps. Exact live pose/geometry equality is supported by the unchanged source-reviewed comparator's successful assertions; it cannot be independently re-decoded from hash-only trace. Elapsed time is not a throughput, physics-determinism or rendering-performance result.

## Completed bounded browser evidence

Prior partial saved-image audit `corrected-replay-images-audit.json` SHA `3f4814dafb8f478f87628dc4e2ec320ab316ba124f101a77df713e5572398415` clears only startup, observed play/motion and pause: Frame 1/1129 at source zero/paused, then Frame 102 at source 3.505 seconds/recorded-running with the cursor paused. It does not prove reconnect, reset or EOF. Exact model count and numeric geometry are data/source bindings, not screenshot measurements.

Root's completed observation binder `browser-observations.json` remains byte-exact SHA `4b2edc769bc449e053d7a98b1808145f3e3c5b7637c20cf5d5d0408bd3454aff`. Independent `browser-image-audit.json` SHA `77a93afd376def47da9eef8c880a8955eba195347085189f736c682fc0396d2c` clears all ten viewed selected images and source/runtime/bundle bindings; all 68 retirement images are hash/signature-bound, with the crucial preceding frame 058 also viewed.

At Frame 530/source 18.440 seconds, the recording is running while the cursor is paused, and the scene tree shows all six names including runtime 18 and runtime 22. After Root's actual reload, the saved tree again shows those six names and the metadata again shows Frame 530/source 18.440/cursor paused. The reload action is Root's observation; identical screenshots alone do not independently establish that event.

At source 39.535 seconds, original frame 058 shows six rows; frame 059 shows five, retaining runtime 22 after runtime 18 removal; frame 060 shows only the four originals after runtime 22 removal. Frame 061 shows the originals at source time zero following the recorded reset. EOF metadata reads Frame 1129/1129, recorded epoch 1/time zero/paused, cursor finished, recorded reset accepted. GUI Play at EOF was **client-refused** with “waiting for a scene or committed ack”; this UI result is not evidence of a backend refusal. The already-passing production `Player::control` regression
`real_cursor_controls_preserve_source_time_and_credit_fences` proves API-level
EOF Resume refusal. The unchanged external comparator proves EOF completion,
Reset and credit fences; it sends no Resume at EOF, so it supplies no wire-level
EOF-refusal ACK.

Actual cursor Reset then displays transport-epoch-2/time-zero/paused acknowledgement and Frame 1/1129, recorded epoch 0/time zero/paused. The final saved image includes the full original green mesh, box and sphere. Root left the demo paused. Source/manifest/three captured resource bytes and current viewer/native ELF bindings are independently checked; exact numerical geometry remains a Source/comparator claim, not a pixel measurement.

All ten selected and all 68 retirement images are **unmodified JPEG/JFIF browser images (`image/jpeg`) with original `.png` filenames**. Every original byte/hash/path is preserved; no rename or re-encoding occurred. They must not be described as RAW PNG. Runtime publication is complete at the exact refs above. Canonical closeout metadata still requires its own normal gate and exact hosted CI; the image review does not supply those results.

## Preserved failures and limits

The first actual replay comparison genuinely failed on one ULP: source translation z `2.9998468750000002` versus replay `2.999846875` at source 0.005 seconds / sequence 5000 (visual IDs 9, 13, 21). It remains retained, alongside its original source/archive/comparator. The product correction enables maintained decimal-to-f64 roundtrip parsing and adds exact bit controls; no tolerance or equality assertion was weakened. All earlier native/viewer compile, lint and coverage failures and corrected/unrun candidates remain separate retained records.

The actual earlier normal-gate failures remain distinct: the 28.023-second
static-kind type mismatch, 30.021-second batch of six Clippy findings, and
1019.281-second headless CLI region failure (7931/8361) are not successful full
qualifications. The latter had 1,241 headless tests and 37 new controls pass but
never reached renderer coverage. The viewer's first 51.027-second test-unwrap
Clippy failure is also retained. Later corrections and appended meaningful
regressions do not erase these failed candidates or weaken their assertions.

Predecessor hosted run `41967679689`, attempt 1, failed with HTTP 503 in selected
SDK preparation before product tests/coverage; the failed suite was not logged
and remains unknown. The upload failure was secondary to missing results.
After bounded read-only locked-URL health checks, a single unchanged-source
failed-job retry succeeded as attempt 2. This ancestor history is separate from
the new corrected run `42103914738`, attempt 1. No suite identity or service root
cause is inferred, and the failed run is not retroactively relabelled passed.

Visible recovered idle warnings of approximately 17.96 seconds (capture), 17.959 seconds (partial replay) and 39.962 and 25.956 seconds (completed browser timeline) remain genuine, with **UNKNOWN causes**. They are not erased by later successful operations. No timer-fix causality is asserted. Screenshots showing 1 fps provide no performance claim.

Physical storage-floor guard stops are retained as operational failures, not product passes. In the partial browser replay, child exit was 0 while the guard controller's effective exit was 1; reconnect/reset/EOF were not completed there. The cause of the capacity change remains unknown. Safe reversible forwarding of exactly 12 old compiler-cache files is a separate resource operation: recovery receipt SHA `67cc9d05fc277a851c11823bcdf262d8ebc3493eeafbd29081feb52045f4f334`, 169,500,268 bytes, all metadata/bytes and logical paths independently checked. It is not a product fix, cache-loss eviction or new qualification result.

The earlier replay launch was refused before runtime when physical SSD free
space was 1,008,402,432 bytes, below 1 GiB. The roughly 95 MB drift remains
unattributed; bounded audit-derived writes did not establish its cause. A separate
reviewed eight-file, 187,212,076-byte old-cache forwarding relocation restored
space. Direct owner approval also authorized exactly 353 old rebuildable Wasm
compiler files to be losslessly archived: 217,374,988 archive bytes preserved all
353 members and recovered 1,267,228,672 bytes, with independent all-member audit
`cd3f6970` clear. Restore those compiler files before reusing their old release
target. This was not deletion of source, raw proof, qualified ELF or served
bundles. All exact recovery scopes, byte/metadata checks and restore maps remain
recorded; runtime acceptance authorizes no additional eviction. The later
partial-browser guard stop at 1,073,684,480 SSD bytes and the exact 12-file repair
above remain separate. Physical floors were never lowered. The 14 GiB initial scratch reservation
belongs to Root's native metadata/staged-gate operational launcher; it is not
the operating scratch floor, a new maintained verify.py policy, or a universal
viewer/build qualification requirement.

Scope is bounded local scene/pose/control recording and read-only replay, with captured immutable local resources. Historical recording-format compatibility, full sensor recording, arbitrary seeking, file-import UI, general asset handling, WebGPU runtime parity and performance claims are excluded. Manual runtime profiles are not merged into qualified coverage. Runtime acceptance does not qualify this new metadata commit or implement M12. Independent source review and ordinary required metadata verification precede its publication; no accepted manual qualification is repeated for this closeout.
