<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M20 acceptance: installed lifecycle and publication credit

M20 was accepted and published on 2026-11-08 UTC as
[m20-preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m20-preview),
release **438259397**, published at **13:18:29Z**. Qualified Main is
`9a3e81cf8babbc87b305f09dd04458fddc70765f`, tree
`3431cf6a2f95fb73aed57b585e211864c0415860`, parent
`e807a97da402308629af30156f845e9cf1ace02d`. Annotated tag object
`ce0fa3e84ae4049ffc596808fee8856a82b4bb99` peels to that exact commit.
Normal nonforced Main/tag pushes and exact remote readbacks passed.

## Delivered contract

Installed `gz-preview run --lifecycle` pairs native `--web-lifecycle` with the
printed viewer URL's outer `lifecycle=1` query. Default run behavior, existing
protocol, queue/deadline bounds and all six accepted c404 viewer files remain
unchanged. The complete M19 robot and copied OBJ remain visible and drivable;
unsupported articulated-robot remove/state edits are refused atomically. Existing
local lifecycle controls are disclosed in the shipped guides.

The native-free Ubuntu 24.04 x86_64 preview keeps authoritative Rapier f64,
ordinary copied assets, a finite command helper and dynamically linked system
libraries. It requires Python 3.11 or newer and browser graphics support. Optional
RGB/depth workers and native SDK distributions remain excluded. See the shipped
[installation guide](https://github.com/gazebosim-rs/gz-sim-rs/releases/download/m20-preview/INSTALL.md)
and [capabilities](https://github.com/gazebosim-rs/gz-sim-rs/releases/download/m20-preview/CAPABILITIES.md).

## Qualification and actual distribution

The full normal hook passed in **2002.361 seconds**, followed by independent
local DATA. Exact [hosted run 42479817732](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/42479817732)
attempt 1 passed all 16 required steps; independent official DATA is
`912ddc35e54b785fd5ced32fa5efefe8bb9f604acf60d0faee13c71f5863ac56`.
Actual profile passes are **1398 headless / 855 renderer**; profiles overlap.
The **344** quiet Python passes are an aggregate, not individually observed cases.
Both new full-world credit/catalog controls passed in each applicable profile.

| Profile | Local lines | Local regions | Hosted lines | Hosted regions |
| --- | --- | --- | --- | --- |
| Headless | 24181/24810 | 36814/38201 | 24184/24810 | 36817/38201 |
| Renderer | 15154/15570 | 23210/24167 | 15154/15570 | 23210/24167 |

All strict workspace/per-crate/actual-parent gates passed. M20 changes tests,
Python launcher/assembler configuration and guides; zero changed Rust executable
lines are not new production-coverage credit. Four raw comparisons were byte-exact
once: local **1773/369/14/33**, hosted **1773/373/14/33**. Full-profile executable
records are metadata; six auxiliary ELF copies are retained. Relevant unchanged
native behavior reuses the source-bound accepted parent proof, not a new native
campaign. Three original hosted ZIP streams, **118,704,200 bytes** total, remain
in RAM and as individual durable copies.

Fresh source-bound release builds, the direct 25-child dependency/ELF collector
and independent input DATA passed. All **740** selected notice identities and
branches retain actual source/provenance and canonical-definition disclosures;
technical selection consistency is not a legal verdict. Both original and
relocated prefixes and the actual archive passed verification and independent
package DATA. Archive and internal root are `gz-preview-m20-linux-x86_64`.
The archive is **40,161,015 bytes**, SHA-256
`61ad2a82312dd1b9778023671bf7301c015421e43e11572baaab6f4024f09b23`.
Manifest SHA-256 is
`249c74afa95a1f2b0ea4394ae2dba2056d89d7037b5dc77edc72060c2df91f55`.
The 1560 manifest entries and unchanged viewer gzip budgets were independently
checked. No renderer rebuild occurred.

A fresh supported Ubuntu installation passed **27 actual CLI / 8 guest exits**,
including installed verification as UID/GID 65534, actual system-library ownership
and no compiler/native SDK requirement. The actual opted-in browser session
showed the full scene, acknowledged Play and advancing clocks, finite forward and
turn, stable Pause, Reset to epoch 1/time zero and restored initial arrangement.
Both helpers and the launcher/host actor/guest waited zero; simulator PID and
owned listeners retired before stopping the container. Six durable journals and
all eight consumed images passed independent retained and visual review.

All seven uploaded API asset sizes/digests and the exact release body matched
the reviewed files. Intentional Latest changed from M19 release 438180834 to
M20 release 438259397; older releases remain intact. The public
[proof manifest](https://github.com/gazebosim-rs/gz-sim-rs/releases/download/m20-preview/m20-acceptance.json)
is SHA-256 `67467b3c1768a683412ecfe12cc32dc9f98de4bdffb9fb3d1c634f69613245de`.

## Preserved failures and limits

The original new-test expectation failure, relative-path controller invocation,
missing AX-field reader failure and interrupted local CI watcher remain retained.
The corrected retained reader bound already captured AX bytes without replaying
the product or browser or modifying consumed checkpoints. The watcher exit was
not a hosted CI result. Separate automatic Main/tag CI was not queried for credit.

The initialized browser recovered from an idle-timeout warning reporting
**29.973 seconds** without a server message. This is a warning interval, not
measured initialization delay; its cause remains **UNKNOWN**. No cold-start,
reliability, cause-fix, numeric trajectory or performance claim follows. The
simulator grandchild wait remains unknown because the unchanged launcher does
not expose it; idle container exit 137 is not application shutdown proof.

[M21/M22](M21_M22_DELIVERY.md) continues with independent sensor-noise Source.
Those features are not included in the M20 archive and inherit no acceptance.
