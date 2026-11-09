<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M28/M29 acceptance

M28 loaded AirSpeed noise and M29 installed preview are accepted at Main
`ab9660ee8ed7d7de1727f51d5ca7300ecb19a865`, tree
`39c2b75f2e9c9bae5a92f7f24eb8face6f7bdcf7`, over accepted M26/M27
`0f525592`. Acceptance was recorded on 2026-11-09 UTC after all actual proofs
and publication readbacks. [m29-preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m29-preview)
is verified Latest (release 438932998); all seven API asset digests/sizes and the
exact reviewed notes match. Annotated tag `9bc25c962e68e03707e48f9fb5c02ba4ededa716`
peels to that Main commit. M25's body and seven assets remain unchanged.
The public `m29-acceptance.json` asset records sanitized hashes and limitations.

## Qualification

The actual complete normal hook passed in 1930.3456199720968 seconds, with
observed controller and child zero, no sidecar/signal/floor stop. Native,
independent local DATA `c7d63ffeead75871da82c37e4970f8e6b0781fcf73b482bea0342659edac93bb`,
exact [CI 42586254208 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/42586254208)
(all 16 steps), and official DATA
`84a4c6fd97ee10a96038712ff38ccc34b62b6909638ea35dd11ebf95a7b71335`
passed. Later automatic Main CI is separate and is not inferred here.

Actual Rust passes are 1518 headless and 922 renderer; profiles overlap.
The 358 quiet Python results are an aggregate, not individually observed quiet
case outcomes. All twelve new headless and nine new renderer controls appear.
Every strict crate/workspace and actual-parent line/region gate passed. Local
headless is 26428/27110 lines and 39913/41418 regions; official headless is
26430/27110 and 39915/41418. Renderer is 16338/16784 and 24746/25774 in both.
Runs are neither unioned nor forced equal. Each independent audit made four
byte-exact raw comparisons once, from 1781/376/21/33 inputs.

Audits bind 1498 Git entries, 26 changed paths, 1472 protected parent entries,
721 source inputs, six actual auxiliary ELFs, seven actual finite copied-CLI
controls and 328 full-profile executable metadata records. Three original CI
ZIP streams total 123657438 bytes, retained in RAM and individual verified durable
SSD copies. Remote SDK DSO provenance is not archived remote library bytes.
The nine-mode native AirSpeed noise comparison has 140 printed rows. Its original
capture tree `a35652b9` remains identified separately from current `39c2b75f`;
only two exact reviewed test-file differences are permitted, with the other
1496 entries unchanged. No native RNG-sequence or performance claim follows.

## Fresh installed distribution

Fresh native-free CLI and drive helper builds passed. Independent notice-input
DATA bound all 740 selected identities; package DATA
`58b3d6e6a6bd6bccfdb99f656729cda89afe0486662f2ee8493b082a4161a6ab`
checked both layouts, all archive members/modes and notices. The archive is
40285104 bytes, SHA-256
`cde4fe323292b2ba7df4a028b2052cd36d237065b6d1c027ee7d2a26a865e1f1`,
with 1568 payload files plus manifest. Both unchanged viewer gzip sizes,
12901614 and 12879428 bytes, stay below their separate 14000000-byte budgets.

A fresh Ubuntu 24.04 x86_64 installation passed all 28 Docker CLI and nine guest
checks, including installed verification and seven actual finite examples as
UID/GID 65534, without a compiler or SDK. Installation DATA
`37349681c35317a66bff71045f0f892b2019d805e71bb6ed86a773e3cc7a44a0`
passed. Two one-ms steps per example prove finite loading/stepping, not canonical
acquisition, noise statistics or a 100 Hz PosePublisher's first 10-ms frame.

The new installed robot/browser session passed actual Play, finite forward and
turn helpers, stable Pause and epoch1/time-zero paused Reset. Eight original
JPEGs and eight canvas AX captures have independent image review. The original
controller, host actor, guest, helpers and launcher have actual zero waits;
simulator PID and listeners retired before owned container stop. Six immutable
journals bind those observations. Retained method2 DATA
`f42b93c3140dc63db6db5d13a6ac303380392a8f6e18bec9c701250de2088bd7`
passed without UI/product replay. The unchanged launcher does not surface its
simulator grandchild return code; that code remains unknown. Idle container exit
137 is not application shutdown proof.

## Preserved failures and limits

Original syntax, formatting and clock-fixture port failures remain failed.
The original normal controller disappeared with watch143, unknown cause and no
producer final result; the later full run independently passed. Initial CI watch
143 is separately retained; a subsequent read-only watch observed the same
attempt1 succeed, with no CI retry. The first browser attempt missed Root's
240-second checkpoint before any drive command; its late checkpoint was never
consumed. The original retained reader failed at an early helper kernel-argv
assumption. Isolated method2 is bound only to the successful current capture:
forward's stable kernel argv was NOT_OBSERVED, while exact source-bound Popen
arguments and actual wait0 are separately retained. Other identities and all
shutdown/journal/UI assertions remain strict.

The recovered 15.957-second idle warning has no asserted cause fix. Actual
screenshots support bounded visible behavior, not pixel-derived numeric poses,
cold-start reliability, comparative performance or universal GPU support.
Canonical notice definitions and author disclosures are technical provenance,
not invented author grants or a legal verdict. Optional native engines,
RGB/depth workers and full compatibility remain outside this distribution.
Root's immutable receipt is `artifacts/m28-m29-delivery/acceptance-main-promotion.json`;
all originals, private audit packets, frozen trees and Fuel work remain intact.
M30/M31 continue independently under [their bounded scope](M30_M31_DELIVERY.md).
