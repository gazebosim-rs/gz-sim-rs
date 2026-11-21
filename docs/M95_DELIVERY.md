<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M95: positive principal-axis JointController chains

The existing two-link/two-Revolute Rapier JointController chain admits positive
Cartesian +X and +Y axes in addition to retained +Z, independently at each joint.
This bounded SYS-CONTROL/PHY-01 composition shares one D31 candidate with
[M94](M94_DELIVERY.md). No arbitrary-axis or controller algorithm rewrite is selected.

## Basis and current checkpoint

Root selected both lanes on 2026-11-21 in source-selection `9e68c1b5`. The
selection's `implementation_dispatched:false` is its immutable creation-time
observation; Root subsequently dispatched both authors. Both implementations,
regression Source and all reciprocal/joined whole-code reviews are now complete
and CLEAR. The joined inventory `7dc0387b` is 1,815 Source files / 13 owned paths /
four new files / two production paths / 111 cumulative paths versus accepted
Main. Seventeen unique new Rust controls are declared in both profiles (overlap
17), with zero Python controls. None is an executed pass. External final eight-doc
review is pending at this freeze; formatter, checks and normal remain unrun,
with final tree, commit and all own results null.

The genuine locally qualified, unaccepted parent is
`cc83ca4a62773edb80b05992fa67a03ff1dad134`, tree
`1dc854336947b3cdd136fa9094dff9e11b9b3c01`, parent `e264c649`.
Normal `4a1c59a7` and independent retained local DATA `e33975e1` / report
`59113b30` passed: H1967/R1503, two ignored each, 494 Python, seventeen own
controls in both profiles and thirteen strict coverage bases. Review publication
`6b16995f` passed on `review/m92-m93-posed-runtime-compound-wheels`.
Exact automatic CI 44939453527 attempt 1 is in progress; Root alone owns watcher
12097, launch `f59b22`. Its hosted outcome and official DATA are still unknown.

Preserve failure `005bf028` and the parent's separate current two-file formatter
`1ffb0cfd` plus original seven-file formatter `12ccb626` reused only for five
unchanged Rust copies. The inherited M90/M91 failures `f9d43298` / `fc5dce53` and
three/one/four formatter custody remain separate. Parent local success does not
qualify this wave, promote Main, change acceptance or establish new native parity.
Each selected phase has at most five working days including review and gates.

## Closed behavior

The parent config chain gate required axis.xyz == [0,0,1]. Existing joints.rs
already resolves axis frames and constructs native world/reduced revolute joints;
existing controller/model.rs lowers scalar measured coordinates to bounded motor
commands without an axis-specific algorithm. The completed config admission
change accepts exactly parsed-DOM [1,0,0], [0,1,0], [0,0,1]. Pinned SDF parse.rs
3639–3641 normalizes lexical axis magnitudes before this gate: XML `2 0 0` resolves
to +X, and scaled positive X/Y have the same policy as retained scaled +Z.
Negative axes, zero/nonprincipal directions, axis2 and non-Revolute kinds remain
refused; no lexical-magnitude restriction is added.

Keep empty expressed_in, identity LINK rotations, existing joint pose/relative_to,
one world root, canonical two-link chain, top-level dynamic Rapier, centered
resolved inertia, existing position/velocity modes, finite force/speed limits,
eight owners and sixteen inputs. Keep all other frame/topology/controller and
engine gates. Passive generic-joint admission is not controller admission.
Reuse existing native scalar motor/servo/continuous JointState, ownership and
initial blueprint/reset paths. No joints.rs, controller runtime, Physics, Server,
protocol, public API, motor gain or solver edit is expected. Do not promise a new
immutable collision-metadata guard or infer native speed from requested targets.

## Meaningful acceptance and exact retained disposition

- Cover +X/+Y/+Z and noncoaxial X/Y chains under the existing position and velocity
  modes, singleton and 2/4/16 primitive compound endpoints, finite centered
  explicit inertia and contact-isolated gravity-free controls.
- Derive independent noncommuting Rodrigues/pivot transforms from measured native
  joint coordinates and the actual model frame. Compare real link/child WorldPose
  and global geometry, retaining `1e-11` generic ray checks and existing frame
  bounds. Account for every body/child; use wide finite transformed query caps,
  with binary endpoints separate. No ideal speed-convergence or no-slip premise.
- Use same-input quiet/equivalent-command/native-state twins, finite bounded motor
  response, explicit-inertia/density controls and no immediate body teleport.
  Preserve pause/latest/invalid-command behavior. Any contact witness needs its
  own computed positive-gap fixture and established horizon, not a universal
  Cylinder skid-turn assumption or threshold adjustment after a run.
- Preserve ordinary/loaded reset, bare prepared-backend geometry/ID distinction,
  retired/surviving owners and native/logical high-water IDs. Late failures stay
  atomic across existing pending inputs. A faulting step is terminal; a healthy
  before-step repair is a separate control, not reset recovery after fault.
- Root alone changes the exact existing global +Z-to-+X XML refusal at
  `tests/compound_joint_controllers.rs:579` into a same-XML positive with actual
  binding, native coordinate, frame/query and meaningful command witnesses.
  Keep its case identity, every neighboring negative and strict numerical bound.
  Private -Z at joint_controller_config.rs:403 and expressed_in at :404 stay
  negative, along with passive axis2/type/topology controls.

M95 declares eight public cases in both profiles; completion `0a6505b7` pins
config `570c62d5` and new regression `a7688253`. Whole Source reviews are CLEAR.
Root's exact retained-row disposition `63d7f82d` is independently CLEAR, with its
same XML and case identity preserved. These counts are declarations, not passes.

Server owns exactly `crates/gz-sim-scene/src/joint_controller/config.rs` and new
`crates/gz-sim-scene/tests/joint_controller_principal_axes.rs`. No fixture is
planned. Root alone owns `crates/gz-sim-scene/tests/compound_joint_controllers.rs`
for the single specified row; neither author changes common lib/joints/initial,
controller runtime or the other lane's product/test files.

## Leases and mandatory gates

The two production seams and new regression files are disjoint. Root owns the
single retained +X row disposition, integration, canonical documents and serial
heavy-job release. Scope owns this guide, the other new guide, AGENTS.md,
README.md, DELIVERY_PLAN.md, HANDOFF.md, IMPLEMENTATION_TASKS.md and planning JSON.
Reciprocal whole-lane and Scope/Root joined code reviews are complete and CLEAR.
Independent final authored-doc review remains pending at this Source freeze.
Batch findings and fixes before one affected formatter
and one full consolidated normal/staged hook, strict workspace/per-crate/actual-
parent/all retained-ancestor line-and-region coverage cycle. Every production
gate stays strictly greater than 95 percent. Then independent retained local DATA,
ordinary review publication, exact hosted CI and independent official DATA, plus
every predecessor and M77 browser/owner-shutdown/release gate remain mandatory.
No per-edit tests, lint, trial runs, old qualification replay or invented receipts.

The independently reviewed resource proposal `cd0787da` is conditional Source
work. Only the next fresh ordinary Cargo target may use executable `/dev/shm`;
profile, runtime/native-free/RGB targets, TMPDIR and evidence packets stay on SSD.
The proposed run-specific initial SSD reserve is 31 GiB, with mandatory initial
14 GiB and early-stop 8 GiB unchanged. Add initial SHM 22 GiB and early-stop SHM
4 GiB, preserving mandatory SHM 1 GiB, MemAvailable 8 GiB and existing runtime
floors. The prior sampled SSD decrease and final ordinary allocation support
conditional arithmetic only, not peak attribution or a fit guarantee. Release
must resample all floors and mount flags, require a fresh absent nonsymlink owned
0700 target, retain its identity/locator on every outcome, and preserve every
normal/strict/native/profile/ledger guard. No directory, executor or release is
created here; no eviction, cache relocation or existing-evidence change is allowed.

All first 93 phase objects and prior 54 groups stay exact. Accepted phases/slices
remain 76/202, catalog 95, finite examples 47, CLI cases 21 and Latest M70 original
41-example package. Keep no-expiry demo 86363 live; no stop, restart or expiry.
Rapier f64 and TPE remain, DART/MuJoCo stay retired. No new backend, framework,
native parity campaign, visual/renderer/browser/package, sensor, Physics, Server
or public API expansion. Unexpected shared production work requires explicit
rescoping before edits; do not force parallelism through a shared seam.
