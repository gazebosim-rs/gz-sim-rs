<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# PHY-03 moving-parent revolute slice

This private slice starts at `5669d79` and owns only
`crates/gz-physics-dart/{src,include,tests,docs}`. Independent source review has passed; strict staged coverage, full mandatory
gates and hosted checks remain required.
Scene admission and shared contracts belong to the coordinator. No performance
or general-articulation claim is made.

## Admission and lifecycle

The construction graph is exactly a dynamic world-hinged parent and one
initially isolated dynamic child. The second revolute uses the existing
`DynamicBackend::add_revolute` contract. Free-root construction, fixed parents,
cycles, repeated incoming joints, branches and longer chains remain unsupported.
Missing body IDs, invalid anchors/axes, out-of-bound projected relative rates
and unsupported graphs leave endpoints and ID allocation unchanged. Body IDs
are backend-local values under the existing contract; a raw integer carries no
foreign-world token. The caller must pair IDs with their owning backend.

Attachment takes the child-relative rate that best fits its requested world COM
twist after subtracting the ancestor contribution. Parent rates stay unchanged.
DART owns joint frames and dynamics. All body/joint caches refresh after coupled
velocity edits and impulses. Body-joint effort goes directly to the incoming
native joint's generalized force; its axis therefore follows its moving parent,
and the solver supplies the parent reaction. Existing world-hinge effort keeps
its validated world-wrench overlay.

Stable native BodyNodePtr slots follow the bodies across skeleton moves. An
emptied source skeleton is removed only after successful admission. A refused
contact check restores the child to its original skeleton without changing world
registration order. Joint removal splits a leaf into a new free skeleton;
root-hinge removal refuses atomically while that body still has a jointed child.
Detaching/removing the child first permits root-hinge removal. Parent body
removal releases the child, while child removal keeps the parent's skeleton live.
Survivors remain usable and reconstructible through either reset. Incident
joints and their pending efforts retire with a removed endpoint; surviving IDs are unchanged.

Actual contacts involving a multi-body skeleton are refused at attachment and
before native constraint solving during a step. The pair's internal collisions
are disabled, matching the upstream model default. Single-body contact behavior
is unchanged. Any active native JointConstraint on a multi-body skeleton is
also refused before Dantzig, including predictive activation of SDF's retained
±1e16 default bounds. This slice does not qualify multi-body contact response or
finite/default-limit solving. A refused step restores poses, joint coordinates,
velocities and time, clears temporary native forces, and retains deferred input.

## Native source and reference

Pinned gz-physics `b44cd0d0ea6606b6c98320c0c87a5670d4fc6be6`,
`dartsim/src/SDFFeatures.cc:1119`, saves parent/child world transforms, calls
`BodyNode::moveTo<RevoluteJoint>`, then sets each local joint frame. Installed
DART 6.13.2 `detail/BodyNode.hpp` shows `moveTo(nullptr)` keeps the current
skeleton, while `split<FreeJoint>` creates a new one. Existing native controls
establish BodyNodePtr ownership and dependent Jacobian index behavior separately.

`tests/fixtures/moving_parent.cc` uses the actual pinned Gazebo DART plugin.
Aligned and rotated two-link models include nonzero initial rates; the rotated
case has offset COMs and differently oriented axes. Both use ODE collision,
Dantzig with its existing fallback, SDF defaults, gravity -9.81 m/s² and 1 ms
steps. The child receives positive, negative and zero SetForce pulses. Thirty-four
rows record both joints and both complete body states through 1000 steps.
The adapter matches all samples at the retained 1e-12 absolute assertion.

Reproduce from the repository root:

```sh
python3 crates/gz-physics-dart/tests/fixtures/probe-moving-parent.py \
  --physics-source /home/nkoenig/work/gazebo/rust/gz-physics \
  --build /home/nkoenig/work/gazebo/rust/gz-physics/build \
  --prefix /home/nkoenig/local \
  --work-dir artifacts/moving-parent-reference \
  --report artifacts/moving-parent-reference/report.json
```

The helper verifies the pinned source and CMake source/install paths, rebuilds
the plugin target, compiles the client and checks actual mapped libraries.
It compares against the checked-in fixture unless `--write-fixture` is given.
The reference build is explicitly hybrid: its CMake configuration and target
compile/link commands select DART headers and libraries under
`/opt/ros/jazzy/opt/gz_dartsim_vendor`, while the replay loads the OSRF system
DART/ODE libraries listed below. The helper records those actual target commands
and paths. All 105 installed gz-physics core headers and seven SDF feature
headers match the pinned source. All 326 common DART headers other than
config.hpp match byte-for-byte, including dynamics and constraints. The vendor
tree additionally contains the recorded lodepng header.

The two config.hpp files differ only in HAVE_NLOPT and HAVE_IPOPT (vendor 0,
system 1) and the four root/data path macros DART_ROOT_PATH, DART_DATA_PATH,
DART_DATA_LOCAL_PATH and DART_DATA_GLOBAL_PATH. Their SHA-256 hashes are
`859af8bb60e81f95bd12061b3c4dc38fcd331eaf62ace9ff207a5459966be20b` (vendor) and
`3a7b692406e7433e08fef91619b25d93fa0b6fcdcfb08d8752a0872bddd73309` (system).
Both macro maps and complete header-tree hashes are recorded in the report.
Unexpected header or configuration differences fail the helper. This qualifies
the bounded observations under that disclosed build/runtime combination, not a
homogeneous or clean SDK profile. A clean immutable reference profile remains
part of the broader P0 qualification work.

The client source hash is recorded in the fixture. The author run used GCC
11.5.0 and these loaded-library SHA-256 hashes:

| Loaded library | SHA-256 |
| --- | --- |
| libdart.so.6.13.2 | f74f4ec9820aee785ef94fb7bedafa7e4c832f209675a8ca1d35019c72b48e06 |
| libdart-collision-ode.so.6.13.2 | 2940e0bd6b056f186d3596e051a26ec25f1c27add88d870cdfe673785dad4b44 |
| libdart-external-odelcpsolver.so.6.13.2 | b890ed6936ea7cf486303fc5101ee5ff68356fc89596273ebaef83671aa4cb16 |
| libode.so.8.0.2 | c511d66333708785c1a22f6cd572c4c84495a535566c9c80eb37d8e0f67bc469 |
| libgz-physics-dartsim-plugin.so.10.0.0~pre2 | 7a28d3cd63634dc3dd3d42d423b35b0c6949963572ded7f8cb8f3eb02caf7dfe |

## Author checks

`cargo test --offline -p gz-physics-dart --all-targets` passed all 115 package
tests under both Rust LLVM instrumentation and native GCC 11 gcov
instrumentation, including unchanged existing assertions, topology probes and
fuzz controls. Twelve public articulation controls and a private native
ownership/preflight control cover this slice. Clippy with `--all-targets -- -D
warnings` passes. Private targets used jobs=2 and debug=0.

`articulation_bounds` runs 48 isolated native processes: radii 1e-29, 1e-10 and
0.1 m; mass pairs (1,1), (1,1e60), (1e60,1), (1e-3,1e5) kg; shared/separate
anchors; parallel/nonparallel axes. The cases check admission rejection or real
native effort/step, impulse, atomic root-removal refusal and child retirement
without abort. Selected runtime refusals are valid negative results, not
physical-accuracy claims. This finite probe set is not proof over every admitted
numeric value.

The unchanged crate/changed-line coverage checker passed with 1044/1062
production Rust lines, 1564/1616 regions, and 150/151 changed production lines.
The denominator includes build.rs. Raw instrumented package output and LLVM
profiles are in the private `artifacts/dart-coverage/profile` directory. This
focused crate report does not replace mandatory workspace/profile evidence.

Native line coverage is 489/512 for bridge.cc, 23/23 for joint_state.hh, and
7/7 for bridge.h's exception boundary. Counts aggregate template instantiations
by source line. Generated CXX code and upstream engine/header code are not
original bridge production. The raw .gcda/.gcno files and gcov JSON remain under
`artifacts/dart-native-coverage`. Reproduce native instrumentation with:

```sh
CXX=g++-11 CXXFLAGS=--coverage \
RUSTFLAGS='-C link-arg=/usr/lib/gcc/x86_64-linux-gnu/11/libgcov.a' \
CARGO_TARGET_DIR=artifacts/dart-native-coverage/target CARGO_BUILD_JOBS=2 \
CARGO_PROFILE_DEV_DEBUG=0 CARGO_PROFILE_TEST_DEBUG=0 \
cargo test --offline -p gz-physics-dart --all-targets
```

Use matching `gcov-11 --json-format --stdout` on the target's bridge and generated
FFI .gcda files, then aggregate owned files by source line. The generated FFI
object carries bridge.h's instantiated exception handler.

The original free-pair rejection test is unchanged. No upstream test is retired,
no tolerance or solver setting is changed, and no coverage exclusion is added.
Independent review passed the 115-test package suite, clippy, the fresh final
reference helper replay, coverage/source checks and additional isolated numeric
probes. Its hybrid-provenance finding was resolved by the disclosure and
fail-closed header checks above. Mandatory staged/hosted qualification remains
outstanding.
