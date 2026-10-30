<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Native DART adapter (PHY-03, first slice)

`crates/gz-physics-dart` implements `gz_physics::DynamicBackend` on the system
DART 6.13.2, the release the upstream gz-physics dartsim plugin links. A small
cxx bridge (`src/ffi.rs`, `src/bridge.cc`, `include/bridge.h`) owns one DART
world; Rust validates every input before a native call. The coordinator owns
scene/CLI selection through `PhysicsEngine::Dart` and `--physics-engine dart`.

## Supported scope

The shared solid-body scope covers free and static bodies with sphere, box,
cylinder or capsule collision; mass with shape-derived or explicit inertia
(offset center, rotated inertial frame); gravity; contacts; world velocity,
one-step COM force/world torque and impulses; one unbounded world revolute per
body with continuous position/velocity readback; removal; retained reset and
owner-bound prepared initial-state reset. Each body is one DART skeleton.

DART also supports fixed local +Z planes through the native shifted-cube
construction; see [DART_PLANES.md](DART_PLANES.md). The bounded scene loader
accepts centered planes with default inertia on effectively static links;
see [SCENE_PLANES.md](SCENE_PLANES.md). Other engines remain unsupported.

## Settings matched to upstream

Each item mirrors gz-physics `dartsim/src` at the checked-out source:

- `ConstructEmptyWorld`: ODE collision detection (`GzOdeCollisionDetector` adds
  only a per-pair contact cap, which defaults to unlimited), 10000 contacts, and
  a body-node collision filter (`BitmaskContactFilter` with default masks).
  DART's default boxed LCP solver (Dantzig, PGS fallback) is kept, as upstream.
- `ConstructSdfModel/Link/Collision`: skeleton added to the world before its
  link, `setMobile(!static)`, no self collision, free root joint posed with
  `setTransform`, inertia from mass, COM and the moment rotated into link axes
  (`Inertial::Moi`), shape nodes with collision and dynamics aspects, SDF default
  friction 1 and DART defaults for the remaining surface values. DART box sizes
  are full extents; cylinder and capsule lengths are the cylindrical section.
- `ConstructSdfJoint` revolute to `world`: the child moves to a `RevoluteJoint`
  whose frame keeps the child orientation, with SDF default limits (±1e16,
  enforced), infinite effort/velocity limits and zero damping, friction and
  spring stiffness. Construction pose is angle zero.
- `AddLinkExternalForceInWorld/TorqueInWorld` for wrenches (force at the local
  COM), and `World::step` with command reset so wrenches last one step. The
  bridge resolves every wrench target before applying any, so a failed step
  applies none and the retained pending wrenches apply exactly once later.

Deliberate differences: the step sets the world time step to the requested dt
exactly (upstream skips changes under 1e-6 s). A step whose result leaves the
bounds below restores the previous positions, velocities and time and returns
`PhysicsError::Native` (upstream restores the last good positions, zeroes
velocities and continues).
Velocities are center-of-mass velocities like Rapier's. For a hinge body, a
velocity request takes the joint rate whose COM twist best fits it in the least
squares sense, and an impulse changes the rate by M⁻¹Jᵀ·impulse. Adding a hinge
projects the body's current twist the same way, because DART's `moveTo` starts a
new joint at rest; reset and prepared reset replay that, so a body spawned
swinging keeps its hinge rate (checked against Rapier in `tests/joints.rs`).

## Runtime library

`build.rs` accepts only a dart.pc for exactly 6.13.2 whose `includedir` and
`libdir` share its `prefix`, whose `dart/config.hpp` declares 6.13.2, and whose
`libdir` holds `libdart.so.6.13.2` and `libdart-collision-ode.so.6.13.2`; a ROS
dart.pc on `PKG_CONFIG_PATH` therefore uses ROS headers and libraries together or
fails with a message naming the mismatch. That prefix's include root precedes
dependency include paths (`/usr/include` is left to the compiler's default).

Every link output of this crate gets `-Wl,--disable-new-dtags -Wl,-rpath,<libdir>`.
The resulting DT_RPATH is searched before `LD_LIBRARY_PATH`, so the DART loaded at
run time is the one compiled against, even with ROS Jazzy's vendored DART 6.13
first on `LD_LIBRARY_PATH` (as in the author's shell). `tests/build_script.rs`
asserts from `/proc/self/maps` that `libdart` and `libdart-collision-ode` are
mapped only from the pkg-config `libdir`; it fails when another build is
preloaded. Downstream binaries do not inherit link arguments. Their build
scripts should add the same flags from `DEP_DART_LIBDIR` (exported via `links =
"dart"`); either way `DartBackend::new` uses `dladdr` on the vtables of the
world and its ODE collision detector and on an LCP and an ODE function, and
returns `PhysicsError::Native` if libdart, libdart-collision-ode or
libdart-external-odelcpsolver was loaded from outside dart.pc's libdir, or
libode from outside ode.pc's. A test preloads a copy of
the pinned `libdart` from another directory in a child process and checks that
refusal.

## Numeric bounds

The packaged OSRF DART 6.13.2 keeps assertions enabled. It aborts the process on
a NaN in its dynamics (`updateTransmittedForceFD`, `updateBiasForce`,
`GenericJoint` accelerations), on a contact LCP matrix that is not symmetric
within an absolute 1e-6 (`BoxedLcpConstraintSolver.cpp:229`), and on a zero
pivot in its Dantzig LDLT (`_dLDLTRemove`); ODE aborts when a collision bound
leaves its hash grid (`aabbBound`). None is a C++ exception. The owner chose to
keep this package, so the adapter checks input and runtime numerical domains
before constrained solves. Admission rejects with `InvalidValue` (a hinge with
`InvalidJoint`), before any change:

| Quantity | Admitted range |
| --- | --- |
| Mass and each principal moment | [1e-60, 1e60]; with the conditioning rule, mass ≥ 1e-5 kg for dynamic bodies |
| Anisotropy of the input principal moments, before rotation | λ_max/λ_min ≤ 1e8 |
| Each shape dimension | [1e-60, 1e60]; collision bounds limit it to 5e8 m |
| Gravity, COM offset, spawn and commanded linear/angular velocity, accumulated force and torque, impulse results | Euclidean norm ≤ 1e60 |
| Free body origin velocity at spawn (\|v\| + \|ω\|·\|c\|) and every generalized velocity after a command | ≤ 1e60 |
| Every world coordinate of a collision bound at spawn, and hinge anchor coordinates | ≤ 5e8 m |
| Dynamic body response: 1/m + Σ_i (Σ_{j≠i} e_j²)/λ_i | ≤ K = 1e5 /kg |
| Dynamic body origin-inertia conditioning: m·\|c\|²/λ_min | ≤ K |
| World revolute: ρ_a²/I_axis and m·ρ_a²/I_axis | ≤ K |
| Summed inverse effective mass of the two bodies at every contact | ≤ 4K = 4e5 /kg |
| Active world-hinge angular row in a contact-bearing group | Finite positive 1/I_axis ≤ 4K / (kg m²) |

Here λ_i are the principal moments with axes u_i, e_j = \|c·u_j\| plus the
shape's half-width along u_j is the shape's extent from the COM along that axis,
c is the COM offset from the body origin, ρ_a the anchor's distance to the
farthest shape point, and I_axis the moment about the hinge line (axis moment
plus m·d²). A unit impulse in direction d at point p (r = p − COM) changes the
contact velocity by 1/m + Σ_i ((r×d)·u_i)²/λ_i ≤ 1/m + Σ_i \|u_i×r\|²/λ_i on a
free body, and \|u_i×r\|² = Σ_{j≠i} (r·u_j)² ≤ Σ_{j≠i} e_j² for any shape point,
so the response rule bounds every diagonal entry of DART's contact matrix at
the body's own surface (off-diagonals follow by Cauchy-Schwarz). It is exact
enough for slender bodies: a pencil (18 cm × 7 mm, 6 g) scores about 2e3, a
cable link (10 cm × 4 mm, 2 g) and a rod (1 m × 1 cm, 10 g) are also admitted,
whereas the earlier ρ²/λ_min rule rejected all three. The per-body rule is
evaluated in the principal frame of the validated moments; the anisotropy cap
keeps that frame accurate. A symmetric eigen-solve is accurate to about
eps·λ_max, so within 1e8 the rotated λ_min has a relative error below 2e-8;
without the cap a rotated 1e60:1 tensor's λ_min was rounding noise, and such a
body (mass 1e30, moments (1e60, 1, 1e60), COM offset ~1 m) resting on the floor
aborted DART. DART stores inertia about the body origin, so m·\|c\|²/λ_min is
bounded too: otherwise the COM moments are lost in I + m\|c\|² cancellation.

A body spawned into, or stepped deep into, another body meets contact points
far outside its shape. DART's own solver is replaced by a subclass of its
default boxed-LCP solver (same Dantzig/PGS solvers and options) that, before
solving, checks the contacts its collision pass just produced against the
stored principal moments and axes of each body (a hinged body by \|p −
anchor\|²/I_axis, zero at the anchor itself; a negative or nonfinite bound,
as from a non-positive inertia, counts as a violation), and checks that
every generalized velocity after force integration is finite and at most 1e60.
On a violation it skips the solve and the step is refused and restored. This
replaces a second collision pass. Spawning a body and removing a hinge run one
collision pass and refuse the change if a contact would exceed 4K; adding a
hinge only constrains a body, which cannot raise the inverse effective mass.

A contact-bearing group can also contain active joint-limit rows from the SDF
default ±1e16 bounds. Their angular unit-impulse response is 1/I_axis, which the
contact lever-arm bound does not constrain for tiny geometry. Before native LCP
assembly, the bridge checks these rows against the existing numerical ceiling
4K, in angular SI coordinates. It checks the actual active group and each row's
own one-DOF skeleton; isolated scalar solves and unrelated islands are unchanged.
This is an additional numerical-domain bound, not a unit-invariant condition
number proof or finite-limit support. [DART_JOINT_BOUNDARY.md](DART_JOINT_BOUNDARY.md)
records the native failure, rollback/command controls and remaining scope.

The 1e60 bound keeps products of up to five such factors (I·ω², m·c·ω², F/m,
τ/λ_min, m·g, a hinge's m·r·ω² and r×(m·r·ω²)) below 1e300. The 5e8 m bound
comes from ODE: it grids coordinates in cells of at least 2^-3 m and asserts
beyond 2^32 cells; measured aborts were at 4.3e9 m for a 0.5 m sphere and
1e9 m for a 0.01 m sphere, and 5e8 m < 2^29 m is safe for all sizes.

K was first chosen from measurements with no rule: bodies with 1/m and
ρ²/λ_min both at κ, rotated, anisotropic and offset, resting on the floor
aborted at one sphere 1e10, a 0.2 m cube by mass alone 3e9, stacked spheres and
cubes 1e10, a hinged sphere in contact 1e10, three touching capsules 1e8, and a
pile of six mixed shapes 5e7 (zero LDLT pivot). With the current rules at other
values of K, the fuzz search below found no abort in 10000 trials at K = 1e7
or 1e8 and 12 aborts in 10000 at K = 1e9 (zero LDLT pivots), and none in 30000
at K = 1e5. K = 1e5 therefore sits at least 1000x below the smallest K that
aborted.

After every step the bridge also requires every generalized velocity to be
finite and at most 1e60 and every collision bound to stay within 5e8 m;
otherwise it restores the pre-step positions, velocities and time and returns
`PhysicsError::Native` naming the reason. A throwing `World::step` is restored
the same way. Velocity commands and impulses that would leave a generalized
velocity (a free body's origin velocity or a hinge's projected rate) above 1e60
are refused unchanged, and so is a hinge whose projected rate would.

`tests/bounds.rs` tests each row just inside, with bodies resting on the floor
and on each other, rotated and anisotropic, a three-body stack, a six-body pile,
a heavy body on a light one, hinged bodies in contact, piles of pencils, cable
links and rods, and the anisotropy cap on both sides, and just outside, plus
every input reviewers reproduced as an abort; those scenes must step with no
refusal. `tests/fuzz.rs` runs 500 seeded trials of one to four rotated,
anisotropic (ratios 1 to 1e60, both sides of the cap), offset spheres, boxes,
cylinders and capsules (a quarter with shape-derived inertia, slender ones
included) from these extremes, on a floor, stacked or apart, optionally hinged,
with 40 rounds of random wrenches, velocity commands, impulses and steps from
1 ms to 2^64 s; each trial runs in its own child process so an abort fails the
test with its seed. It asserts that contact steps with bodies on the floor
succeed, and a second test steps 40 ordinary resting scenes (0.1-10 kg, shape
inertia) for 1000 steps each with no refusal. The same generator ran 30000
trials with no abort, and the reviewers' search (30000 seeds) and adversarial
search (10000 trials) also found none.

This is stricter than the Rapier and MuJoCo adapters, which do not abort on
such inputs: bodies lighter than 1e-5 kg, strongly anisotropic or far-offset
bodies, and contacts deep inside other bodies are rejected or refused for DART
only. Native Gazebo with the same package is exposed to the same aborts:
`tests/fixtures/upstream_light_cube.cc`, run against the installed dartsim
plugin, survives a 1e-8 kg 0.2 m cube resting on a floor and aborts with the
same `isSymmetric` assertion at 1e-10 and 1e-12 kg. Build and run it with:

```sh
PKG_CONFIG_PATH=/home/nkoenig/local/lib/pkgconfig \
  g++ -std=c++17 -O2 crates/gz-physics-dart/tests/fixtures/upstream_light_cube.cc \
  $(PKG_CONFIG_PATH=/home/nkoenig/local/lib/pkgconfig pkg-config --cflags --libs \
    gz-physics gz-physics-sdf gz-plugin-loader sdformat gz-math-eigen3) \
  -Wl,-rpath,/home/nkoenig/local/lib -o /tmp/upstream_light_cube
LD_LIBRARY_PATH=/home/nkoenig/local/lib /tmp/upstream_light_cube \
  /home/nkoenig/local/lib/libgz-physics-dartsim-plugin.so 1e-10   # exits 134
```

Remaining process aborts would be engine defects reachable from admitted input;
none is known.

## Divergence in ordinary scenes

Some ordinary scenes diverge: in the reviewer's search of 300 scenes (2-26
bodies of 0.1-10 kg, up to 50 m/s and 20 rad/s, 3000 steps), 43 grew a body's
spin exponentially (about doubling every 130 steps) until the step was refused
for its velocity or for leaving the collision grid. This is DART's behaviour,
not the adapter's: `tests/fixtures/native_scene.cc` runs the same bodies,
inertia, contact settings and 1 ms steps through the installed dartsim plugin,
and for six such scenes (seeds 2, 4, 68, 77, 93 with cylinders, 215 without)
its largest body spin and first body position equal the adapter's at every
recorded sample to the printed precision. Native Gazebo then aborts the process
in ODE (`aabbBound`) where the adapter refuses the step. `tests/divergence.rs`
checks seeds 215 and 2 against recorded native trajectories. Slender bodies dropped at speed while spinning diverge in DART fairly often:
in a reviewer's run, 17 of 40 piles of five pencils dropped at 12-38 m/s did,
identically with the adapter's checks disabled and with DART's default solver.
Through `gz_physics::install`, the resulting velocity or off-grid refusal ends
the simulation. Decision: the adapter keeps this typed refusal where native DART
aborts the process in ODE; upstream has no recovery path for this case.
Reproduce:

```sh
PKG_CONFIG_PATH=/home/nkoenig/local/lib/pkgconfig \
  g++ -std=c++17 -O2 crates/gz-physics-dart/tests/fixtures/native_scene.cc \
  $(PKG_CONFIG_PATH=/home/nkoenig/local/lib/pkgconfig pkg-config --cflags --libs \
    gz-physics gz-physics-sdf gz-plugin-loader sdformat gz-math-eigen3) \
  -Wl,-rpath,/home/nkoenig/local/lib -o /tmp/native_scene
LD_LIBRARY_PATH=/home/nkoenig/local/lib /tmp/native_scene \
  /home/nkoenig/local/lib/libgz-physics-dartsim-plugin.so \
  crates/gz-physics-dart/tests/fixtures/divergent_scene_215.txt | grep step
```

## Performance

Resting scenes (spheres, boxes and capsules of 0.1-10 kg on a floor, 1 ms
steps, mean of 1000 steps after 200, release builds): one body 8.6 µs per step,
50 bodies 560 µs, 200 bodies 2320 µs. The same adapter without the contact
check takes 8.6, 552 and 2285 µs (the solver-time check costs about 1.5%; the
earlier second collision pass cost 10-16%). The native dartsim plugin on the same
scenes takes 7.8, 525 and 2150 µs.

## Same-engine parity

`tests/fixtures/reference.cc` loads the installed plugin through gz-plugin,
builds SDF worlds with `ConstructSdfWorld`, steps with 1 ms `ForwardStep`, and
records link frame data and joint state (steps 0-10, then every 10th step).
Native origin velocities are converted to COM velocities (v + ω × R·c) to match
this API; scenarios with a centered COM are unchanged by that conversion.
Regenerate `tests/fixtures/reference.json` with:

```sh
crates/gz-physics-dart/tests/fixtures/generate-reference.sh /home/nkoenig/local /tmp/dart-reference
```

(`g++` 11.5 with `-O2`; plugin and SDK from `/home/nkoenig/local`; the script's
`LD_LIBRARY_PATH` resolves the system `libdart.so.6.13.2`, sha256 `f74f4ec9…`.)
The fixture (sha256 `50dda937…`) regenerates byte-identically; its first three
scenarios are unchanged from the previous fixture, which was also byte-identical
when the plugin loaded ROS Jazzy's vendored DART 6.13.2. The Rust tests now
always load the system library (see Runtime library). Results:

| Scenario | Steps | Worst absolute difference | Asserted |
| --- | --- | --- | --- |
| Free-falling sphere | 1000 | 0 (bit-identical) | `== 0` |
| Sphere landing and resting on a static box | 2000 | 0 (bit-identical) | `== 0` |
| World-revolute pendulum | 4000 | 1.1e-16 | `< 1e-14` |
| Tilted hinge, offset COM, rotated inertial frame, 0.7 rad/s start | 2000 | 7.8e-16 | `< 1e-14` |

The pendulum fixture specifies the hinge 1 m along the child Z axis, which DART
composes with the child pose; this API takes the world anchor, so the joint frames
differ only by rounding. The tilted hinge takes the plugin's world-frame
construction inputs (recorded in the fixture) and differs by rounding paths only:
nalgebra rather than gz-math rotates the inertia, the world anchor/axis replace
the child-relative SDF joint pose, and the initial rate comes from projecting a
spawn twist (including an angular component perpendicular to the axis, which the
hinge discards) rather than `SetVelocity`. The existing native gz-sim pendulum fixture
(`crates/gz-physics-rapier/tests/fixtures/world_pendulum.json`, also DART
6.13.2) now matches joint position/velocity within 1e-12 and body x/z within
2e-6 because of gz-sim's 1e-6 changed-pose cache.

## Threads and ownership

Every bridge function is declared fallible, and `bridge.h` defines cxx's
`rust::behavior::trycatch` with a `catch (...)`: std and non-std C++ exceptions
both return as `PhysicsError::Native` (cxx's default would terminate on a
non-std exception). A world constructor or `add_body` that throws after touching
ODE releases what it created under the ODE lock, leaving no partial skeleton.

`World` is `Send` through one documented `unsafe impl`: the `UniquePtr` owner is
unique, no DART pointer escapes, and the bridge serializes ODE. Running two DART
worlds' collision at once, or creating/destroying ODE geometry while another
world collides, aborted inside ODE's hash space (`aabbBound` assertion) in
`tests/threads.rs`. Every bridge call that creates, changes, steps or destroys
ODE geometry therefore holds one process-wide mutex, and per-thread ODE data is
allocated before each step. Distinct DART worlds in one process never step in
parallel. The backend reaches `Sync` through a never-locked `Mutex` used only via
`&mut self`; queries read Rust-side state refreshed after each mutation.

## Limitations

- One world joint per body (DART trees); a second returns `Unsupported`.
- Mesh, plane, heightmap and other shapes, surface parameters, joint limits,
  damping, articulations, and the remaining dartsim features are not exposed.
- Step parallelism across DART worlds is serialized, as above.
- The numeric bounds above are a DART backend limitation, stricter than Rapier
  and MuJoCo: dynamic bodies lighter than 1e-5 kg (the 1/m term alone reaches
  K), with a response bound or m·|c|²/λ_min above 1e5, with input moments more
  than 1e8 apart, hinges with ρ_a²/I_axis or m·ρ_a²/I_axis above 1e5, larger
  velocities, forces, torques or gravity, collision bounds or hinge anchors
  beyond 5e8 m, and masses, moments or dimensions outside [1e-60, 1e60] are
  rejected; steps that would solve a contact above 4e5 /kg are refused, and so
  is removing a hinge that would. Rapier tests that used f64::MAX or 1e308
  magnitudes are ported at these bounds, and a capsule segment shorter than
  1e-60 m is rejected.
- Ordinary scenes can diverge in DART and are then refused (see Divergence).
- `build.rs` `main` (probe, bridge compile, printing directives) runs only in
  Cargo's build phase and is uncovered; its probe, install checks and directives
  are tested. docs/TESTING.md allows no exclusion for handwritten build logic,
  so `build.rs` stays in the denominator.

## Evidence

Author run after the fourth review: `cargo test -p gz-physics-dart` passes 75
tests (the fuzz test spawns 500 child trials); clippy with `-D warnings` is
clean. Measured crate coverage: 815/832 lines, 1278/1326 regions; uncovered
Rust is build-script `main`, a few error closures and native-error propagation
that valid records cannot reach. Re-review of these fixes is pending.

## Server reset integration

The server replays the five `dartsim` rows in
`planning/reference/lifecycle-controls.json` on both DART and Rapier, and the
five TPE rows on TPE. The fifteen executions retain the original control
precedence, removal, identity restoration, reset-callback, timestamp and pose
assertions. DART uses the existing 1e-12 pose tolerance; only Rapier retains its
previous step-reset integration allowance. These are comparisons to recorded
native DART observations, not a claim of identical native library builds.

Server binaries pin DART with the same DT_RPATH directives as the CLI and scene.
Tests run the build-script entry point in a child and construct DART with a decoy
`libdart.so.6.13` first on `LD_LIBRARY_PATH`. Author and independent focused runs
pass the reset comparison and all four runtime tests; the ignored build-script
helper is executed explicitly by its parent test.

## Native dependencies

Build and test need DART 6.13.2 with its ODE collision component and pkg-config
files: on Ubuntu 24.04 from packages.osrfoundation.org, `libdart6.13-dev`,
`libdart6.13-collision-ode-dev` (pulls `libode-dev`), `pkg-config` and a C++17
compiler. Regenerating the reference additionally needs the gz-physics dartsim
plugin, gz-plugin, sdformat and gz-math development files.

## Generalized joint effort

World hinges accept deferred additive Nm commands through
`DynamicBackend::add_joint_effort`. Preparation, native refusal, retirement and
both reset paths preserve the command lifecycle described in
[JOINT_EFFORT.md](JOINT_EFFORT.md). The six-row C++ plugin pulse oracle uses actual
`SetForce`; rotated offset-hinge controls additionally check analytic inertia,
world pose and COM twist. Finite limit editing remains unsupported.
