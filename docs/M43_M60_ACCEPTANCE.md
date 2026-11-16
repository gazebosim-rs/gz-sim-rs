<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M43–M60 accepted

The eighteen phases below are accepted in dependency order on 2026-11-15.
Each of the seventeen exact candidate heads passed its own mandatory normal,
strict workspace/per-crate/actual-parent coverage, independent local DATA,
exact hosted CI and independent official DATA. The official audits retained
all original ZIPs and completed four byte-exact RAW comparisons per head (68).
Local and hosted coverage remain separate; profile test counts overlap.

| Phase | Qualified revision | Successful exact hosted run |
| --- | --- | --- |
| M43/M44 | [`a08f0b50`](https://github.com/gazebosim-rs/gz-sim-rs/commit/a08f0b505bb775cbc02974e098084e763da62a82) | [43024343885 attempt 2](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43024343885) |
| M45 | [`e66c21e7`](https://github.com/gazebosim-rs/gz-sim-rs/commit/e66c21e739089670b24e23b91fcd1363935b9a39) | [43724814232 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724814232) |
| M46 | [`cca6b07d`](https://github.com/gazebosim-rs/gz-sim-rs/commit/cca6b07d9dd0f8f85c98e0de9b9488780bfa7d83) | [43724825734 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724825734) |
| M47 | [`f27a37fd`](https://github.com/gazebosim-rs/gz-sim-rs/commit/f27a37fd94d8218a4c3c488a4e550e06f521c9c8) | [43724837902 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724837902) |
| M48 | [`a204d94b`](https://github.com/gazebosim-rs/gz-sim-rs/commit/a204d94bc377caa8fedb4d89a1114c3795de1b05) | [43724850058 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724850058) |
| M49 | [`4ccd4f06`](https://github.com/gazebosim-rs/gz-sim-rs/commit/4ccd4f067ecf3438cd9a3cc445cd45d682f6075d) | [43724861747 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724861747) |
| M50 | [`623faca6`](https://github.com/gazebosim-rs/gz-sim-rs/commit/623faca668151a283f3a6057a13e29d57fe3d9ff) | [43724874449 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724874449) |
| M51 | [`ea921f58`](https://github.com/gazebosim-rs/gz-sim-rs/commit/ea921f5886b02bbe576e328848899fd276696208) | [43724887248 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724887248) |
| M52 | [`1ed59a73`](https://github.com/gazebosim-rs/gz-sim-rs/commit/1ed59a7348a3dee94cdae8b7a4bbd847cbd9911b) | [43724899846 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724899846) |
| M53 | [`9bf3ada4`](https://github.com/gazebosim-rs/gz-sim-rs/commit/9bf3ada4675022b45e39d5f089e1bb3aaeaa2c04) | [43724911598 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724911598) |
| M54 | [`d140e51e`](https://github.com/gazebosim-rs/gz-sim-rs/commit/d140e51e56b8c83d3857ad16977472412a9629f8) | [43724922969 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724922969) |
| M55 | [`fcfa7176`](https://github.com/gazebosim-rs/gz-sim-rs/commit/fcfa717630d74f8e0f3b283083c48f0ff408e08e) | [43724935254 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724935254) |
| M56 | [`d635b119`](https://github.com/gazebosim-rs/gz-sim-rs/commit/d635b119ca8d9e3f338fb0cf8af7df88b71a7f34) | [43724950682 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724950682) |
| M57 | [`9dfde487`](https://github.com/gazebosim-rs/gz-sim-rs/commit/9dfde487e7e377432573ebbcfd1e7a58901bc492) | [43724961266 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724961266) |
| M58 | [`173b31d2`](https://github.com/gazebosim-rs/gz-sim-rs/commit/173b31d2d13889f851c58dfff1ea81a34680ede8) | [43724974801 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724974801) |
| M59 | [`7f849bd3`](https://github.com/gazebosim-rs/gz-sim-rs/commit/7f849bd3e6b1d9842bdde415d9f2923e09ef0b0e) | [43724986854 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724986854) |
| M60 | [`a1a01192`](https://github.com/gazebosim-rs/gz-sim-rs/commit/a1a01192661281fc4691fe5c048a65e1986aefa4) | [43724997382 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/43724997382) |

All candidates are unchanged ancestors of actual Main
`a1a01192661281fc4691fe5c048a65e1986aefa4` (tree
`351670d1d490e0e6b36d190eb3bcc4b79ee4799a`). The reviewed sequencing amendment
used one ordinary nonforced final Main fast-forward after every candidate's own
gates passed. It does not claim intermediate Main ref states or transfer results.
M43/M44 were accepted atomically, then M45–M60 in predecessor order.
Automatic Main and tag workflows are separate from these qualified review runs.

[M44](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m44-preview)
(release446213571) explicitly distinguishes qualified source `a08f0b50` from
its unchanged payload actually built at `9a40768d`. Both identities remain in
its manifest and proof; binary/source equivalence is not inferred.
[M60](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m60-preview)
(release446217017) is Latest. Its source and payload are exact `a1a0119`.
Both releases passed actual draft download verification of all seven assets,
public/Latest readbacks and prior-release metadata, tag and asset preservation.
M40 and M44 remain available unchanged.

M60's clean Ubuntu installation ran all 33 installed finite examples without a
compiler or physics SDK. Its actual maintained browser demonstration verified
independent two-robot commands/motion, canonical odometry, Play/Pause/Reset and
owned shutdown, with independent retained RAW and ten-image visual DATA.
The launcher exited zero and owned processes/listeners retired; the simulator
grandchild exit code remains unknown. No native Physics/trajectory/performance,
legal-completeness or broad compatibility claim is made.

The exact acceptance chain and actual execution records remain under
`artifacts/m60-delivery/ordered-acceptance-records/`. Old snapshot failures,
quota stops, native-input/reader failures, failed demo attempts and all original
qualification evidence are preserved. The successful snapshot diagnostic checked
three pinned signed releases and six exact indexes; no mirror/date/signature or
workflow workaround was used. New work is [M61](M61_DELIVERY.md), not accepted.
