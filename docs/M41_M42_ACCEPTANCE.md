<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M41/M42 accepted: wind-relative AirSpeed and joint-force feedback

M41 and M42 are accepted at Main
`73e03b9f741689b1538898bde96bf975a49f068e`, tree
`0facf42d43766009165bea2e6e649ca8996605e7`, over accepted M40 `470da6e6`.
Normal nonforced promotion and exact Main/review-ref readbacks completed after
the complete normal hook, independent local DATA,
[hosted CI 42808865907 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/42808865907)
and independent official DATA passed. All sixteen hosted steps succeeded.

The normal hook passed in 1964.372 seconds. Rust profiles report 1,682 headless
and 1,003 renderer passes with overlap; Python 387 is an aggregate quiet-suite
result. Hosted headless coverage is 32,313/33,154 lines and 48,916/50,767 regions;
renderer is 19,271/19,784 lines and 29,435/30,638 regions. Local headless
32,307/33,154 lines and 48,911/50,767 regions remains a separate measurement.
All unchanged strict workspace, production-crate and actual-parent gates passed.
Each local and official set's four raw comparisons completed once, with
1,799/385/31/33 profiles. Original hosted ZIPs total 135,245,699 bytes and are
retained individually in durable storage as well as RAM.

M41 admits finite immutable WORLD wind for the existing single AirSpeed sensor.
It changes measurement-relative airflow without applying physical wind forces.
The scoped original component capture, independent native/literal audits and
91-reading/91-packet replay passed. Zero paths, point-velocity arithmetic,
noise, demand, pause, rate, reset, removal and actual canonical delivery remain
bounded by the feature guide. This is not a native Physics or RNG parity claim.

M42 combines existing ApplyJointForce and JointStatePublisher on the admitted
WORLD hinge. Actual canonical commands, complete Model feedback, zero coasting,
opposite effort, reset/removal and owned peer shutdown passed. The six retained
native velocity bounds remain unchanged; position and velocity also match a
paired direct-input Rapier scene. Native DART position trajectories are not a
Rapier contract. Axis.force remains zero and unmeasured. No solver or mechanical
domain was added.

The original Clippy and retained-caller failures, monitor exit 143 and later
three-target headless failure remain recorded. The original nonparent Git wait,
monitor-loss cause and resource history remain unknown. Corrected tests preserve
native velocity assertions and use paired actual Rapier position feedback and
the supported canonical removal service. No production, C++ helper, native
fixture or numerical tolerance was changed by that correction. Unused completion
routes were not executed or credited.

The durable acceptance record is
`artifacts/m41-m42-delivery/acceptance-main-promotion.json`; local DATA
`151f33cc` and official DATA `976f126c` bind the exact source and retained proofs.
Automatic Main CI is separate and has not been queried or credited. These phases
do not refresh the installed package or repeat browser qualification:
[m40-preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m40-preview),
release 440119258, remains the most recent accepted installed preview.

Continue directly with M43 named IMU references and M44's eighteen-example,
three-executable preview. Their source preparation overlapped closeout; their
native, Rust, fresh package/install/browser and publication gates remain required.
