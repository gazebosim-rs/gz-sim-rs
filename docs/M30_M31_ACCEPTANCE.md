<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M30/M31 accepted source delivery

M30 ideal LogicalCamera and M31 static-vector PosePublisher were accepted at
[Main `e53bf432`](https://github.com/gazebosim-rs/gz-sim-rs/commit/e53bf432010f1d42752ab126a72d1e0266a2c661)
on 2026-11-09T14:00:38.918228+00:00. The exact tree is `e82d1168106c3b229d60e8e91180b4ebd0498f4e`,
with accepted parent `ab9660ee`. The immutable generated receipt is
`artifacts/m30-m31-delivery/acceptance-main-promotion.json`.

The complete reviewed source passed its full normal hook in 1,948.363 seconds,
independent local DATA, exact [hosted run 42623164901](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/42623164901)
attempt 1 (all sixteen steps), independent official DATA and normal nonforced
Main promotion with exact readback. Local and hosted profiles each passed
1,559 headless and 940 renderer Rust tests; these profiles overlap. The 358 quiet
Python passes are aggregate, not individually observed case outcomes. All
required strict workspace/per-crate and actual-parent gates passed. Local and
hosted raw comparisons each ran once over 1784/378/21/33 separate inputs.

The independent local report is `d9b41f1f20ec7954d6af8c4f33b8bdf2134033574f781c692448e33ca9c2edad`;
the official report is `c3e0cae76423225d99dbcfa44afac84efe067cf86b6c83f3e3006264e3381a6e`.
Original source, logs and coverage archives are retained durably. Native proofs
remain the original 36 LogicalCamera image/frustum pairs and seven static-pose
modes/461 vectors, with independent DATA; no native capture was repeated.

M30 is bounded model-origin visibility in a flat Rapier world, not AABB,
occlusion, pixels or general LogicalCamera compatibility. M31 preserves the
current physical joint domain. Loaded reset uses fresh handles/zero clocks;
direct-core reset retains live clocks. Weak-lease cleanup is a Rust lifetime
guarantee, not an invented native reset result. See the respective feature guides
for all admitted and refused cases.

All original native compile/boundary, lint, parser/fixture, outer-143 and
reader/collector method failures remain distinct and retained. In particular,
the original controller exit/minimum observations remain unknown across its gap;
no successful successor relabels that run. The corrected SDF dependency passed
its own normal, exact hosted and independent local/official qualification before
consumption.

This is Main-only source delivery, following M26/M27. No feature tag, archive,
installation or browser acceptance was added. Release438932998 `m29-preview`
remains installed Latest and its identity was read back after promotion. The
separate automatic Main CI42630305638 was observed in progress, not inferred to
have passed. M32 prepares the next refreshed Ubuntu preview with fresh required
package/install/browser proof and every mandatory source gate.
