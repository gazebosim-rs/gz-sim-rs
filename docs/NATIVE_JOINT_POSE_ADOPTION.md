<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Native joint semantic-pose evaluation

This source batch is unqualified and isolated from the failed constructor
adoption tree `05035e96`. It now pins published producer `f0d05146`, whose joint-pose correction passed
the mandatory local gate, exact hosted run `41438358157` and independent official
audit. Accepted
simulator Main remains `e55356c8`, with its original producer pin.

One complete native diagnostic used the selected immutable SDK and public DOM
construction, frame, shape, joint and contact APIs. All 16,362 body and joint
values across 101 frames matched the independently audited fresh Server
observations exactly. A separate Rust diagnostic reproduced all 486 retained
values from the failed current consumer exactly. Both captures passed source,
library and retained-data audit; neither qualifies the simulator candidate.

Initial body and shape rotations match bitwise. Small initial body and joint
translations differ. The actual native joint `SemanticPose().Resolve` results
have identity rotation and translation `(0, -4.440892098500626e-16,
-2.220446049250313e-16)` for all four B03 joints. These are observations, not
constants to insert into production code. The previous producer's common-ancestor
pose query returns the mathematical local identity while native named-joint
evaluation retains two scoped-root paths and the final inverse/product.

Scene now selects the existing `Sdf::joint_pose(name, "")` API for the child-frame
serial input. Empty target retains the declared-child semantic contract. Public
world-pose metadata, axis resolution, exact identity-orientation routing and the
world-frame fallback remain unchanged. This one-line selection uses the separately qualified producer implementation;
it does not manufacture the
observed residual or change backend guards.

The complete diagnostic comparison first exceeds `1e-12` at step 35 and reaches
a transient maximum difference of about 0.02225 at step 94. Joint differences
and contact-force differences remain. Thus neither the radian constructor fix
nor the newly observed semantic input establishes a passing trajectory or its
cause. Keep every old assertion, field, tick, reference and numerical bound.

The dependency update changes only the revision and lock source, with no package
versions or edges changed. Complete independent review of the combined consumer
batch, then run one consolidated mandatory verification.
The first combined mandatory gate failed after 367 seconds on the later serial
lifecycle identity helper; strict fresh-native B03 motion passed unchanged.
The separate correction preserves that failed tree and fixes only public identity
inventory in the test helper. Later suites and final coverage remain unqualified.
No additional native capture occurred. All diagnostic observations remain unchanged.

The subsequent `f4db5841` gate passed the serial lifecycle correction and failed
at the same all-ECS-name assumption in the B03 world-light test. Both duplicate
inventories are corrected together without changing their identity equality,
physics, metadata or reset assertions. Other new tests receive source review
for the same assumption before the next mandatory pass. No final acceptance
is inferred from these partial results.
