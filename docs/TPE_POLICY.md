# TPE static and world-axis policy

Confirmed D23 corrects two native quirks. Loaded static models hold their pose
and clear stored linear/angular velocity on the next physics phase and reset.
This includes empty models without contact bounds. `Motion::new_fixed` carries
that immutable policy in the existing Bevy component and reset blueprint;
ordinary `Motion::new` with static ModelBounds also freezes. The standalone Body
kernel and ordinary unbounded/dynamic Motion retain their native arithmetic.

Supported single-link velocity commands use world axes, including rotated model
and link frames. Fixed-body commands reject before mutation. TPE wrenches still
reject; there is no force implementation or solver fallback. The current adapter
reports model-origin world velocity with zero independent link-local motion.
It does not add angular cross link-offset velocity or claim general rigid-link
origin semantics. New analytic rotation controls use zero link translation.

## Native observations

`planning/reference/tpe-policy.json` retains six fresh direct native TPE controls
and a fresh replay of all thirty existing Server/Physics input controls. Source,
probe, runner, binary, cache and loaded-library hashes identify both runs.
Library names replace private host paths in this projection; raw report hashes
bind the retained local artifacts. Installed dependencies and existing builds
remain qualified; this is not an immutable reference-image claim.

The direct model controls use a 10 ms step, initial position [0,0,5], identity
orientation, linear velocity [1,0,0] and angular velocity [0,0,1]. Native static
and dynamic models move and rotate identically for three steps. Rust dynamic
motion matches all position/quaternion/velocity fields at 1e-12; Rust fixed
motion holds the initial pose and reports zero velocities. The unchanged native
Model.BasicAPI still passes. That original case tests static flag setters,
without asserting static immobility under stored velocity; its wider API remains
partial. This correction does not retire the whole original case.

The existing thirty-row input fixture is unchanged. Its twenty previously
supported DART/TPE controls retain their comparisons and tolerances. The ten
rotated TPE rows remain explicit native observations. For requested world +X
velocity, the native model90 control moves +X but reports -X velocity; link90
moves/reports -Y. Rust moves/reports +X in both frames. No native values were
rewritten to claim matching behavior.

## Assertion dispositions and verification

Two Rust-only capability guards previously expected rotated TPE velocity
admission to fail. Their Unsupported/no-pending assertions now exercise the
still-unsupported TPE wrench on the same frames. The original velocity payloads
are covered positively by the corrected world-axis tests. This is an explicit
capability change under D23 and received independent assertion review.

New controls cover finite extreme stored velocities, paused phases, static
geometry and empty models, noncommuting model/link rotations, world velocities,
analytic poses, IMU frames, fault absence, core/loaded reset, cancelled pending
commands and stable public IDs. A draft assertion incorrectly required internal
Bevy Entity reuse after reconstruction; its failed log is retained. The test now
checks the documented stable public ID and resolves the current Bevy handle.
No upstream assertion or numerical tolerance changed.

Independent review found the first runner could accept skipped/error XML despite
the actual native result being clean. The runner now requires one completed
Model.BasicAPI and zero error/skip/failure counters; six tooling regressions pass
and the exact old predicate fails them. Fresh native replay preserves all six
observations. The first passing gate was not treated as source approval; the
corrected source was replayed and checked again.

Independent source/API/assertion/native/projection review and the corrected full
staged gate passed: 246 Rust tests, 41 tooling checks and fifteen unchanged real
C++ child-client scenarios. Every production crate exceeds 95% lines and regions;
workspace coverage is 3548/3584 lines and 5419/5538 regions, with changed TPE27/27
and scene11/11. The receipt is planning/reviews/tpe-policy.json. Final metadata
binding, normal protected commit and exact hosted verification remain separate
requirements. Full PHY02 and P2 remain open. No new performance claim is made;
prior reports remain tied to their recorded source revisions.
