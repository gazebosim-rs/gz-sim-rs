<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M20: explicit installed lifecycle and publication credit

M1–M19 are accepted. The parent is qualified Main
`e807a97da402308629af30156f845e9cf1ace02d`, published as
[m19-preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m19-preview)
(release 438180834). M19's full normal hook, exact hosted run 42462737108
attempt 1, independent local/official/package/install/browser evidence and
published asset digests passed. Its original failures and unknown causes remain
retained. M20 is selected under D30–D32, with at most five working days including
independent review and every mandatory check.

## Runnable increment

Add an explicit `gz-preview run --lifecycle` choice that pairs the existing native
`--web-lifecycle` flag with the existing maintained viewer's outer
`lifecycle=1` query. This enables the already accepted lifecycle and publication
credit contract. It also exposes its existing admitted local spawn/remove/state
controls; guides must disclose that surface rather than presenting credit as an
invisible transport change. Ordinary `run` defaults remain unchanged.

Reuse the complete M19 coexistence robot and OBJ bytes, authoritative Rapier f64,
upstream Zenoh and all six accepted c404 viewer files. Source inspection indicates
that the articulated robot remains in the catalog and publication while its
unsupported lifecycle edits are refused. That is not an executed proof: tests and
the actual installed session must establish it. Do not remove the robot's joints,
sensors or built-in systems to make the new mode pass.

This phase does not change wire schemas, renderer code, queue capacity, deadlines,
physics, sensor models, transport libraries or the default publication mode. It
does not promise to repair the unknown cause of M19's initial consumer stall.
No new transport, observer, packaging or runtime framework is authorized.

## Parallel ownership

Root integrates in `gz-sim-m20` on `work/m20-credited-preview`. Each source lane
uses its own worktree over the same accepted parent. Authors do not run tests,
lint or independent qualification before the complete integrated source batch
and independent review are ready.

| Owner | Worktree | Files and responsibility |
| --- | --- | --- |
| Light | `gz-sim-m20-launcher` | `scripts/preview_runtime.py`, `tests/test_preview_runtime.py`, `scripts/package-preview.py`, `tests/test_preview_package.py`, `docs/M20_INSTALL.md`, `docs/M20_CAPABILITIES.md`: explicit flag/argv/URL pairing, meaningful default/opt-in/negative controls, complete new guide pair and unchanged legacy verification. |
| Messages | `gz-sim-m20-credit-tests` | Existing `crates/gz-sim-cli/tests/private/web_publication_credit.rs`, reusing its real-socket fixtures and preserving the separate credit unit controls: inspect retained controls first, then add only missing real socket credit/catalog/coexistence regressions. Viewer and production protocol files remain read-only. |
| Root | `gz-sim-m20` | Shared planning, AGENTS/README/HANDOFF and M19 acceptance records, fixture and file-lease coordination, integration, normal staged hook, exact hosted CI, package/install/browser proof, publication and tracker. |
| SDF | Read-only review | Independent complete source and regression integrity review; batch findings before the one consolidated pass. |

If a lane needs another file, coordinate the narrow lease with Root first. Keep
all paused worktrees and Fuel ownership intact. Do not add extra frozen proposal
files, private review machinery or per-edit test rounds. Reuse unchanged evidence
where existing checks permit it; preserve the mandatory staged hook, strict
workspace/per-crate coverage and exact hosted CI before acceptance/publication.

## Acceptance

1. Default run behavior remains unchanged. Explicit opt-in supplies both native
   and viewer flags; neither a URL-only nor native-only pairing may silently
   masquerade as a successful opted-in launcher contract. Existing loopback
   origins, port ownership refusal, finite helper and cleanup remain unchanged.
2. Real socket controls establish the admitted existing one-flight/Ready contract
   for the full copied world, including delayed Ready and relevant reset or
   reconnect behavior not already covered by retained tests. Every visual remains
   represented; unsupported articulated robot edit flags remain false. Existing
   sensor/joint systems and publication contracts are retained.
3. New installed guides explain default versus explicit mode, enabled lifecycle
   controls and unsupported edits. Preserve complete legacy M12/M19 guide-pair
   verification. Final archive and internal root are both
   `gz-preview-m20-linux-x86_64`; the external checksum uses that exact basename.
4. Finish implementation, meaningful regression code and guides; independently
   review the complete integrated batch and fix findings before one full normal
   hook. Inspect actual failures together before affected reruns. All strict
   coverage/parent checks and exact hosted CI must pass without weakening.
5. Assemble, relocate and independently audit the selected native-free payload
   with actual dependency/notices and unchanged viewer budgets. Complete supported
   non-root Ubuntu installation and a real opted-in browser session showing the
   full robot/OBJ scene, finite forward/turn, Pause/Reset and actual owned clean
   shutdown. Use existing maintained proof tools, preserve actual waited exits,
   and do not replay old accepted manual qualification as a substitute.
6. Promote only qualified source, publish actual checked assets, and verify their
   digests. Update canonical records and the web tracker, then select the next
   useful bounded phase while work remains. No acceptance follows from source
   authoring, older gates or a proposed proof.
