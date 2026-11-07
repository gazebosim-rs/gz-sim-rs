<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M12 acceptance: installable integrated preview

M12 runtime/distribution is **accepted and published**. Root's formal decision
is `2026-11-07T19:34:44.558424+00:00`, retained in
`artifacts/m12-delivery/m12-runtime-acceptance.json`, SHA
`4af8e31ea2786011bfbef8056d755141bd3e7a95630badc0989efc49cc121f80`.
All required source, local/hosted/official, final package/install and retained
robot/browser evidence passed. The normal Main push/readback and public
prerelease target are exact `9147120aaccd9383d8e406c1b8ee82348153b734`.
M1–M11 remain accepted. No next implementation or M13 is selected.

This canonical metadata is separate from runtime/distribution acceptance. Its
required Source review, normal gate, exact hosted CI and official audit must
complete before its own publication; the qualification pipeline records those
actual results separately. No own future hash/CI or automatic Main result is
invented here.

## Concrete final package

The retained final archive is
`artifacts/m12-delivery/final-package-2hq8jloh/gz-preview-m12-linux-x86_64.tar.gz`:
**39,769,747 bytes**, SHA-256
`728bb722c63e17a336fb9f9c7a70b9d9b3e9db122ca5f0dfe23e0dc4ce8e7247`.
Extraction yields `gz-preview-m12-linux-x86_64/`. Its manifest SHA is
`53a77fa0f7e30948a0d323db65db40a99c41abd78b16a8e0090b752a51dba7c9`.
The [public prerelease](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m12-preview)
contains the [archive](https://github.com/gazebosim-rs/gz-sim-rs/releases/download/m12-preview/gz-preview-m12-linux-x86_64.tar.gz),
[checksum](https://github.com/gazebosim-rs/gz-sim-rs/releases/download/m12-preview/gz-preview-m12-linux-x86_64.tar.gz.sha256)
and [release provenance](https://github.com/gazebosim-rs/gz-sim-rs/releases/download/m12-preview/m12-release-provenance.json).
Actual publication/tag and all three asset digest readbacks are recorded in
`m12-release-publication.json`, SHA
`e855b5460dbe45623b83b6c08032a3165ac5b56ebd134fae56afe8755cf77e04`.
The public provenance digest is
`4a827695858c4f049882ac95f931ab84bfedc95df19cb62ad26aa524ef5ebf68`;
it is separate from the continuation's internal generation provenance.
External checksum and generation provenance are retained separately in
`final-package-continuation-u0qb7vbg/`; provenance SHA is
`f3f57db44da0e9063f3e4f92f023c8694293ffd373bb9ff48fc0a0ded95cb019`.

The archive contains 1,562 manifest payload files, the manifest, and exactly
2,393 tar members including directories. Both assembled and relocated trees,
all tar-member bytes/types/modes and the complete notice graph passed retained
DATA checks. Exactly two payload entries differ from the earlier provisional
archive: `M12_INSTALL.md` and `M12_CAPABILITIES.md`. All **1,560 non-guide files**
and the **11 behavior inputs** retain exact predecessor bytes, sizes and modes.
The original 39,789,016-byte provisional archive `69a4aea1…` and manifest
`04471766…` remain preserved; no canonical wording retroactively changes them.

Original assembly wrapper session 56927 exited 1 after its completed assembly and
assembled-prefix verification guards. A filesystem-wide free-space drift was
incorrectly charged as owned packet growth; the activity causing that drift is
UNKNOWN. The failure/result writer hit the same work guard, leaving no completed
result. Old child raw wait codes are UNKNOWN, not fabricated zeroes. The reviewed
retained-output continuation ran once, without reassembly or assembled-verify
replay: actual extraction and relocated verification each exited 0. Its result
SHA is `c2fe1139b148d0d04bcf2a84a2f12f06450a77bacc9d7e1325f4625640fcabfb`.

Independent package DATA is `final-package-audit.json`, SHA
`20bd1a847411080f4cc4d17ecccc7b8085cfbf5397903a3c231c808c4da7efe0`.
The first audit invocation omitted its required argument and checked nothing.
The corrected invocation completed all payload/tar/739-package checks, then
failed only constructing its report because its launcher supplied a relative
`__file__`. A separately Source-reviewed remaining-only finalizer used the exact
reviewed source, retained failure trace and completed guards; it repeated no body
checks. Both failed method invocations remain failed. Unpersisted mode histograms
and body-local statistics remain UNKNOWN rather than reconstructed as observations.

## Separate source and qualification identities

| Input | Exact source or identity | Evidence boundary |
| --- | --- | --- |
| Native simulator and finite sender | `a542cb1b2c84280180e6acc73821a225557f6f44`, tree `1bef5e54ce67f62dc5a9825d1671bcf5052592ef` | Actual pinned Rust 1.97.1 release Cargo transcripts; native-free/no `rgb-camera` |
| Maintained six-file viewer | `c40418cd85e21db2bd0271919c90eda0c1184ece`, tree `c043069149de1a35d1fe1cb28241a4dcb56f81a8` | Inherited accepted production build/CI and copied actual bytes; no new viewer build |
| Unshipped generator and final consumer guides | `9147120aaccd9383d8e406c1b8ee82348153b734`, tree `70084c19e76ea2149894d1e1c9ed1d87855f98e2`, parent `a542cb1b` | Five notice/guide paths; runtime Rust, hooks, workflow and lock unchanged |

The actual simulator ELF SHA is
`b240813c10703a60fed45c55243803def131b8957bae5e8c78f4dbd5672cc19f`;
the finite sender is
`943e885bb7ba0e68f06e0bdbe85de902fb7da94bed2ab9c5097d1f22d4b17e95`.
The unchanged M8 robot, loopback configuration and viewer resources are ordinary
copied files, not development symlinks. The assembler measured gzip sizes
**12,901,614 bytes WebGPU** and **12,879,428 bytes WebGL2**, each below the
unchanged 14,000,000-byte budget. These measured copied inputs are separate from
historical rounded production-build stdout and debug Wasm CI.

Runtime `a542cb1b` passed its full normal gate and exact hosted
[42228908156](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/42228908156),
attempt 3/all 16 steps. Independent official audit `8b5f0f36…` binds actual
1,248 headless, 802 renderer and 320 tooling passes, separate strict profiles,
source/input/Cargo/auxiliary proofs and durable original ZIP bytes. Attempts 1/2
failed during snapshot bootstrap before product checks; their records are not
relabelled passing.

Generator/guide `9147120a` passed a full normal hook in **1928.683120 seconds**,
with no floor stop. Local DATA report `f07ac2c5…` binds 1,358 Git blobs/modes,
1,353 unchanged parent files, exact five changed paths, 645 source/input bindings,
and actual **1,248 headless, 802 renderer and 334 tooling passes**. The 45 authored
Python methods are Source inventory, not individually logged quiet outcomes.
The 42 inherited M11 controls and two outgoing-only helper controls retain actual
selected-profile observations. No Rust/runtime change is hidden in this child.

| Final profile | Local lines | Local regions | Hosted lines | Hosted regions |
| --- | --- | --- | --- | --- |
| Headless workspace | 20488/21018 | 31452/32594 | 20490/21018 | 31454/32594 |
| Renderer workspace | 13862/14260 | 21432/22355 | 13864/14260 | 21434/22355 |
| CLI, each profile | 4915/5102 | 8327/8716 | 4915/5102 | 8327/8716 |

Every owned crate and each workspace separately passes unchanged strict >95%
line/region gates; actual-parent Rust changed coverage is0/0 for this five-file
child. Four retained raw comparisons are byte-exact once, from
1762/363/14/33 inputs. Original 10 and separate RGB/depth 9 auxiliary cases retain
six actual copied ELFs. The 296 normal-profile executable records are metadata,
not retained original Rust ELF bytes. Actual renderer RGB 4/depth 5/support 34
controls remain separate from installed native-free operation. Nine retained
C++ peer binaries and source/provenance do not supply new C++ package/demo parity;
24 remote DSO identities/36 SDK matches are provenance, not archived remote DSOs.

Exact new hosted
[42402368577](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/42402368577)
attempt 1/all 16 steps succeeded at `9147120a`. Independent official report
`final-release-hosted-42402368577/audit-parent-base-summary.json`, SHA
`14b877922530cd2869e3e7c324e7a85f53189d89f1e16b8ee84cecf3d526c4e3`,
is CLEAR. It independently binds 1,358 source blobs/modes, 1,353 protected parent
files, 645 source/input bindings and 2,396 artifact files. Actual totals are
1248 headless/802 renderer/334 whole-suite Python passes. All original ZIP bytes
are durably retained individually: three streams totaling 110,700,227 bytes.
Official raw counts 1762/368/14/33 were byte-exactly recombined once, separately
from local counts. Separate-run hit differences above are not equality gates.

The original reader failed before raw comparisons because it assumed full/base-zero
changed coverage. Retained workflow/job evidence instead declares actual push
base `a542cb1b` (`event.before`). The independently Source-reviewed correction
uses that base and independently recomputes actual-parent diffs: Rust changed 0/0
in both profiles, while strict whole/per-crate coverage remains enforced. No
product checks, source, denominator, tolerance or completed raw comparison were
repeated or weakened to correct this evidence-reader assumption. Original failed
reader Source/logs remain retained.

## Notices and installed profile

The 739 selected package records preserve raw license expressions and all
required alternatives. Native selections follow actual Cargo artifact IDs;
viewer graphs are conservative root-reachable notice envelopes, not historical
compilation proof. There are 1,546 notice files and 233 explicit extras. Nine
disclosed missing-author-body cases use identified full license definitions with
separate unmodified author records; these definitions are not invented author
grants or a legal completeness verdict. The actual empty `sha2-const-stable`
source file is separate zero-byte evidence and is never counted as license text.
No metadata or OR branch was dropped. Rust standard-library `COPYRIGHT-library.html`
is preserved as `Rust-COPYRIGHT.html`, alongside the applicable project/font/vendor
and package materials.

The supported initial target is **Ubuntu 24.04 x86_64**. The final archive's clean
Ubuntu setup ran once as Root session 1264/exit 0, with successful ordinary Ubuntu
APT and installed verification as UID/GID 65534. Setup result SHA is `df880f9b…`.
Independent retained installed DATA `final-clean-install-retained-audit.json`,
SHA `1044dc7711ee769ef0427aae490ad4b9964d30a037f5ba11d366f15744d7e462`,
verifies **25 CLI exits 0, seven guest exits 0 and 11 layer snapshots**, the read-only
package mount, no compiler/native SDK, unchanged prefix bytes and stopped owned
container. The largest recorded writable layer is38,653,094 bytes. Idle sleep
PID 1 exit 137 is container cleanup, not an application exit.

Actual installed packages include Python3 `3.12.3-0ubuntu2.1`,
`libgcc-s1:amd64` `14.2.0-4ubuntu2~24.04.1`, `libc6:amd64` `2.39-0ubuntu8.9`
and Ubuntu keyring `2023.11.28.1`. Both actual ELF `ldd` outputs resolve
`libgcc_s.so.1`, `libm.so.6`, `libc.so.6` and the x86-64 loader. This final setup
records paths and installed versions; it does not add a new per-file ownership
query. Earlier demo ownership evidence mapped the same admitted closure with
usrmerge/diversion handling. The package is not fully static and does not qualify
other distributions, architectures or graphics drivers.

## Retained real robot/browser demonstration

The independently reviewed installed demo remains
`packaged-demo-attempt4-audit.json`, SHA
`2c92603ee366861a44e85335489810cfa2853f60a757a8a844224e540678d417`.
Its provisional behavior bytes are exactly the final archive's 1,560 nonguide
files and 11 behavior inputs. This lineage is explicit; no new browser run is
invented for the final guide changes. The real package ran as UID/GID 65534 with
HTTP 127.0.0.1:8081, WebSocket 9014 and loopback Zenoh 7449. Packaged forward
`0.4,0,3` and turn `0,0.8,3` helpers each exited 0, including successful zero-target
braking. Real WebGL2 observations show translation against the fixed grid and
changed orientation. Pause was acknowledged at epoch 0/time 92.754; Reset at
epoch 1/time 0 restored the initial pose.

Three checkpoints bind six unique images, with one additional saved image.
All seven retain original JPEG/JFIF bytes despite `.png` names; no rename or
re-encoding is claimed. Images and raw command/control evidence prove visible
behavior, not numerical odometry, image-depth accuracy or performance.
Actor/guest/launcher/helpers exited 0 and simulator identity/listeners disappeared.
The unchanged launcher's simulator-grandchild return code is UNKNOWN. The
borrowed static HTTP service was restored; the ended M9 live simulation was not
replayed. Protected port owners and prior accepted evidence were preserved.

## Preserved failures and release limits

Earlier setup migration-comment refusal, label mismatch and loader ownership/
diversion admission errors remain preserved. License syntax, missing text,
empty-placeholder handling, mode assumptions and audit wrapper/parser failures
were corrected without replacing retained originals or dropping checks. The earlier
240-second browser-startup wait failed; the later warm-tab demo does not qualify
cold startup or establish the timeout's cause.

The 82329 full gate failed in a tiny no-dependency Cargo test fixture nested inside
an enclosing workspace: 334 Python tests included one error, before coverage.
The original generate-lockfile stderr is UNKNOWN; a separate single read-only
locate-project diagnostic reproduced that workspace context. The successful
full gate used fresh SSD scratch outside all Cargo ancestors, preserving its
14 GiB initial reservation and four operating floors. This operational launcher
choice does not alter verify.py, assertions or thresholds. Owner capacity
recovery, old cache/evidence preservation and the later unknown global disk
drift remain distinct facts; no outage or filesystem cause is inferred.

The initial binary distribution excludes RGB/depth workers and native backend/
SDK profiles. Accepted source for those configurations remains retained. The
robot demo does not newly qualify all mesh, sensor, lifecycle, controller or
record/replay interfaces merely because their accepted source ships. A capable
graphics browser is required; the headless server is GPU-free. Full Gazebo
compatibility, other platforms and performance claims remain outside this preview.
Finishing twelve named milestones would not complete the rewrite or its backlog.

## Published result and canonical qualification

The release tag/prerelease `m12-preview` targets exact 9147120a, with no force or
invented future ref. `final-release-main-publication.json` records actual normal
Main push 0 and exact remote readback. Root's formal acceptance decision/time
follows those publication facts, final official DATA, installed verification and
retained demonstration review. All twelve delivery statuses are complete; the
current-phase field stays M12 and execution_state remains active under the existing
schema. Later scope is unselected; no owner pause, M13 or full-rewrite completion
is inferred.

This canonical metadata still follows independent Source review and its full
normal hook/exact hosted CI/official DATA before publication. Its own actual
qualification is recorded by that pipeline, not preclaimed by this record.
Automatic Main CI remains separately observed, never borrowed from review CI.
Tracker/status mirroring is a subsequent reviewed task preserving 151 existing
slices, concurrent owners, legacy fields, history and Fuel.
