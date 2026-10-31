<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Surviving Jacobians after topology changes

This repair starts at `506f94a` and changes no admitted topology or solver setting.
It owns `src/bridge.cc`, `tests/attachment_rollback.rs` and this document.

DART 6.13.2 `Skeleton::extractBodyNodeTree` reinitializes the bodies that remain
in the source skeleton (`Skeleton.cpp:2761`). `BodyNode::init` clears its cached
body/world Jacobians and their derivatives, then calls `dirtyTransform`
(`BodyNode.cpp:1437–1442`). This does not invalidate an already-clean Jacobian.
A later query can therefore return zero columns for a surviving parent.

Two public failures reproduce on unmodified `506f94a`:

- After two contact-refused attachments, the next successful attachment ignores
  a moving world's hinge contribution when projecting the child rate. The
  regression observes child velocity -5.4 m/s and angular velocity 4.4 rad/s,
  instead of the clean world's -3.9 m/s and 2.9 rad/s. Public caches remained
  unchanged during refusal and concealed the stale native Jacobian.
- After either leaf-joint or leaf-body retirement, an angular impulse on the
  surviving world-hinged parent produces zero rate instead of `0.1 / 1.004`
  rad/s. A projected velocity edit also needs the surviving Jacobian.

The bridge calls DART's `dirtyJacobian` and `dirtyJacobianDeriv` on survivors
after extracting a child: attachment rollback, leaf-body retirement and
leaf-joint detachment. The detachment call precedes its contact admission check,
so a refused detachment and subsequent reattachment also retain valid ancestor
caches. Other moves extract an isolated body, leaving no survivor; their joint
frame/position setters already invalidate the moved body's caches. No custom
Jacobian, solver, state reset or tolerance change is introduced.

The new tests retain 1e-12 assertions. Repeated refusal verifies unchanged
endpoint caches and unconsumed joint identity, then compares complete native
states with a clean world before and after three steps carrying pending wrench
and effort. Separate analytical tests cover parent impulses after each retirement
path, plus root velocity projection. All three initial regressions failed on
clean `506f94a` and passed with the repair. Independent review reproduced both
failure families before applying the fix.

Author commands use `CARGO_NET_OFFLINE=true CARGO_BUILD_JOBS=2
CARGO_PROFILE_DEV_DEBUG=0 CARGO_PROFILE_TEST_DEBUG=0` and a private target:

```sh
cargo test -p gz-physics-dart --test attachment_rollback
cargo test -p gz-physics-dart --all-targets
cargo clippy -p gz-physics-dart --all-targets -- -D warnings
```

The standard and GCC 11 gcov-instrumented full packages each pass all 119 tests,
with none ignored or failed. Clippy passes with warnings denied. Native unique
line coverage is 498/521 (95.585%) in bridge.cc, 23/23 in joint_state.hh and 7/7
in bridge.h. All nine added executable lines are covered. Raw gcov JSON, source
hashes and logs are retained in the private `artifacts/rollback-evidence`; raw
.gcda/.gcno files are in `/tmp/gz-dart-rollback-native-target`. Reproduce native
instrumentation with `CXX=g++-11 CXXFLAGS=--coverage` and
`RUSTFLAGS='-C link-arg=/usr/lib/gcc/x86_64-linux-gnu/11/libgcov.a'`, then use
`gcov-11 --json-format --stdout` and aggregate template instances by source line.
No production Rust changed. Independent source and regression review passed,
including independently reproduced failures on clean `506f94a` and a repaired
full package run. Mandatory staged/hosted gates remain required. Free-root construction remains
unsupported in this repair; its separate private investigation exposed this bug.
