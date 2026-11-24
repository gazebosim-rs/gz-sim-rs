# Local compliant foot development

An opt-in component of the ordinary Bevy scene now owns a compliant foot and one or more prescribed supports. The existing 3D landing reference, the analytical drop, grid refinement and mechanical balances supply different checks. The desktop replays use the measured results.

| Replay | What to look for |
|---|---|
| [Fixture 80 on the live backend](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9691%2F) | Force, heat and rebound across the three damping settings. |
| [Tiled support experiments](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9692%2F) | Blue shows the coupled 3D solver, orange shows the analytical solution. Each chapter has 13 frames and a different layout or drop. The small spheres show normal force. Replay time is a presentation clock. Each chapter's physical times are in cases.json. |
| [Two moving force plates](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9693%2F) | One panel moves away and comes back under the foot. The other continues supporting it. |
| [Fast elastic tier and 3D solver](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9694%2F) | Five pairs of identical worlds cover bouncing, standing, an elevator, a shove and a damped sole. Each green sphere shows the current contact force. |

## Measurements

Fixture 80 used three damping settings and a public 0.5 ms clock over 13.5 ms. At 128 fixed substeps the worst center-of-mass discrepancy at shared clocks was under 0.071 µm. Adaptive refinement also met the impulse checks but cost more per landing. [Fixed measurements](../artifacts/multifidelity-live-sole/comparison.json) · [Adaptive measurements](../artifacts/multifidelity-live-sole/adaptive/comparison.json)

For 100 combinations of support layout and incoming conditions, the largest error relative to the relevant physical scale was 0.00624%. The study checks position, velocity, normal force, individual support impulse, heat, support work and spurious rotation. Its reference assumes centered normal motion, fixed effective area, and zero relative external acceleration. The inclined configuration rotates the complete experiment. The translating belt starts with zero tangential relative speed. [All cases, measurements and frame numbers](../artifacts/multifidelity-panel-landings/cases.json)

An independent five-world experiment kept gravity on and compared the optional analytical elastic tier with a 128-substep 3D solve. At 1 ms per public tick the exact solver was eligible through all 600 ticks of the bouncing, standing and rising-deck runs. A shove at tick 151 switched the foot back to 3D on that tick. The damped comparison used 3D throughout. The sphere heights and poses in the replay come from both live runs. [Five-world measurements and timings](../artifacts/multifidelity-flat-sole-tier/measurements.json)

| Further replay | Experiment |
|---|---|
| [Spinning and braking the turntable](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9695%2F) | Compare the fixed, adaptive and fine solves as the plate spins up the foot, stops and receives a separate torque pulse. |
| [Adaptive fixture 80](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9696%2F) | The adaptive solver chooses the contact resolution for each tick. |
| [Adaptive moving seam](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9697%2F) | Both panels keep their own error budget through release and reacquisition. |
| [Damped normal-motion tier](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9699%2F) | Same five scenes with the optional uniform damped solver enabled. Each force marker comes from its own solver. |

## Numbered worklog

The sequence below counts local engineering and qualification milestones. Related milestones share a gz-web-rs replay. The separate 100 combinations in the tiled-drop study contribute one benchmark campaign to this count. Timing and accuracy here apply to the named fixtures. Every material measurement here is synthetic.

### 1–10. A compliant rectangular sole in 3D

1. Clip the moving lower face against the four sides of a finite deck. Overhanging sole area stops carrying pressure.
2. Clip by compression depth inside the remaining footprint. A pitching foot can bear weight on just a corner or edge.
3. Integrate the two layers’ recoverable strain over the compressed patch. The independent energy audit includes the unloaded part of a decompressing sole.
4. Clip the local unloading pressure at zero across a rotating face. A separating strip can stop pushing while its neighbor still carries load.
5. Integrate normal damping loss across both portions of that unloading face. The force law and energy loss remain consistent through the clamp.
6. Apply regularized sliding friction using velocity at each material point. A twisting foot produces a moment as well as a resultant force.
7. Return the normal center of pressure and the loaded perimeter in world coordinates. The fixture replay draws both directly from the contact solve.
8. Integrate energy entering or leaving at moving finite edges. The tilted-rim control compares the flux with the derivative of stored energy.
9. Advance triaxial inertia using world angular momentum and an offset center of mass. The torque control checks angular momentum under an independent applied wrench.
10. Solve relative to a translating, oriented deck and transform the result back to world coordinates. Rotating and boosting the entire experiment preserves its relative solution.

[Contact implementation](../crates/gz-physics/src/sole_contact.rs) · [Mechanical controls](../crates/gz-physics/src/sole_contact/tests.rs) · [Fixture visual](../artifacts/multifidelity-live-sole/README.md)

### 11–20. Contact resolution and cost

11. Bypass surface integration in certified free flight using ballistic propagation over the public tick.
12. Exclude diagonally separated footprints using projected support axes, including trajectories tangent to the top plane.
13. Detect a narrow support crossed entirely between unloaded sample times and rerun the interval at higher temporal resolution.
14. Refine time independently when adjacent grids disagree on motion, impulse or heat.
15. Refine surface quadrature independently when a twisting patch has unresolved traction.
16. Reuse identical trial grids within one adaptive decision. Each trial restarts from the incoming state and the accepted trial contributes the only committed impulse.
17. Surface a finite-budget failure without committing a partial trajectory. Invalid inputs and penetration past the coating also preserve the incoming state.
18. Integrate uniform-velocity patches without subdividing every triangle. An independent rotating-quadrature limit checks the tilted patch wrench.
19. Benchmark the hot query at three spatial depths for flat, inclined and twisting footprints. The original depth-2 flat query took 6,081 ns at p95 and the optimized query took 225 ns in the paired measurements.
20. Compare the fixed and adaptive live scene from the archived incoming state. Publish the actual grid, public-tick time and end-to-end p95 separately from the microbenchmark.

[Grid implementation](../crates/gz-physics/src/sole_contact/adaptive.rs) · [Query timings](../artifacts/multifidelity-local-sole-program/checks/kernel-contact-cost.txt) · [Live adaptive measurements and visual](../artifacts/multifidelity-live-sole/README.md)

### 21–30. Gazebo and Bevy integration

21. Enroll the foot and deck by scoped link name through an optional SDF world plugin on the local multifidelity backend.
22. Resolve the box and offset center of mass through semantic frames. Unsupported collision offsets and incompatible inertia frames produce a load error.
23. Assign the coated pair to one physics step owner. An otherwise identical world without the plugin retains ordinary native collision handling.
24. Apply a queued one-tick foot wrench while preserving a prescribed deck’s velocity at the same public boundary.
25. Keep the last committed reaction visible while paused. Advancing exactly one tick consumes the pending inputs exactly once.
26. Preflight the native endpoint identities and states before publishing any of them. A bad final panel leaves the first panel and the queued wrench intact.
27. Retire the missing panel’s reaction on removal, including a removal made while paused. Surviving bodies and their commands remain usable.
28. Reconstruct the registered pair on loaded-world reset. Public identities are restored and the first repeated impulse matches.
29. Allow static collisionless observation models in the same world. A mounted CPU lidar sees the moving foot without changing its trajectory.
30. Qualify the complete fixture-80 SDF load, compound mass representation and public observation path against the independent continuous reference for each of three materials. Chromium checked all 84 force and 84 heat readings.

[Authoring](../crates/gz-sim-scene/src/damped_sole/config.rs) · [Lifecycle tests](../crates/gz-sim-scene/tests/damped_sole_lifecycle.rs) · [Measured replay and independent controls](../artifacts/multifidelity-live-sole/README.md)

### 31–40. Multiple supports acting on the same foot

31. Sum disjoint deck reactions at the same body midpoint, advancing the foot once even when several panels carry weight.
32. Return one impulse, angular impulse, peak load and contact patch per stable panel ID. Enrollment order leaves the result unchanged.
33. Check moving panel ownership across the entire tick using swept separating axes. Abutting edges are allowed and passing through another panel is refused.
34. Qualify split versus unsplit support during a dynamic drop. Two equal coplanar panels reproduce the single plate’s motion and summed impulse.
35. Accept independent deck-layer stiffness on each panel. Tripling one deck layer’s stiffness moves its load share to about 60% in the symmetric test.
36. Support separate grip, slip regularization and damping per panel. A slippery undamped panel can sit beside a panel that delivers both traction and heat.
37. Compute each panel’s actuator work from its own velocity and impulse. Opposing moving supports retain their separate work and shear.
38. Compare adaptive errors per panel before acceptance. Opposite impulse errors cannot cancel merely because the foot’s total impulse agrees.
39. Detect a second short support crossing while the first panel already carries weight.
40. Bound a receding edge relative to the moving sole plane. This fixes the actual adaptive rejection at 109.5 ms after the other end of the foot remained loaded.

[Coupled integrator](../crates/gz-physics/src/sole_supports.rs) · [Adaptive regressions](../crates/gz-physics/src/sole_supports/adaptive/tests.rs) · [Moving-seam visual](../artifacts/multifidelity-moving-sole-seam/README.md)

### 41–50. Measuring load and contact history

41. Transform action/reaction forces and moments to the local axes and origin of a force plate. The tests reconstruct the original wrench at the foot COM.
42. Derive the zero-moment point in the sensor plane and retain the residual twisting moment. Small normal readings have an explicit validity threshold.
43. Build the convex support footprint from the polygons actually carrying load, including across a gap between panels.
44. Measure signed distance to the most limiting support edge. Removing one side moves the test foot from a 0.1 m margin to the edge.
45. Project an arbitrary load line onto the support plane and compute a force-weighted pressure center across panels.
46. Detect load and unload events with separate force thresholds so small force fluctuations do not split a stance.
47. Identify a transient strike when the peak and impulse are positive but the foot is already unloaded at the public endpoint.
48. Accumulate episode impulse, heat, friction work, support work and peak force through the releasing tick.
49. Handle repeated observations, paused removals, epoch resets and gaps in a recording without duplicating a measured impulse.
50. Record live force plates throughout the lowering, opening and return of a support. The moving panel was unloaded for 30 consecutive hold ticks and returned at the same 104 ms boundary as the finer reference.

[Force plate and footprint](../crates/gz-physics/src/sole_balance.rs) · [Episode recorder](../crates/gz-sim-scene/src/sole_episodes.rs) · [Measured seam and force traces](../artifacts/multifidelity-moving-sole-seam/README.md)

### 51–60. Independent landing reference and calibration

51. Derive a phase-space solution of the flat damped drop independent of the 3D midpoint solver. The undamped limit reproduces a half sine.
52. Integrate physical elapsed time in the regularized phase coordinate through maximum compression and lift-off.
53. Query arbitrary times and distinguish the loading and unloading crossings of the same measured depth.
54. Locate peak force independently of maximum compression. For a damped drop the force peak comes earlier.
55. Infer effective stiffness from measured indentation and the incoming/rebound speed pair.
56. Infer stiffness from contact duration when indentation is unavailable.
57. Infer stiffness from the peak recorded by a force transducer and cross-check the three instruments on the same synthetic drop.
58. Recover separate layer properties from static strain measurements, a known deck stiffness or an independently measured foot share. The Rust calibration example emits the matching SDF fields.
59. Intersect admissible damping intervals across repeated drops with bounded speed measurement errors. Mutually incompatible readings are detected.
60. Fit one synthetic training drop per support layout, then predict 40 held-out drops with different speeds, masses and deck motions. The held-out normal-force traces use the fitted parameters.

[Independent drop solution](../crates/gz-physics/src/flat_landing.rs) · [Material fitting](../crates/gz-physics/src/sole_calibration.rs) · [Measured and predicted force](../artifacts/multifidelity-panel-landings/held-out-force.svg) · [Holdout results](../artifacts/multifidelity-panel-landings/holdouts.json)

### 61–70. Closed-form elastic contact under gravity

61. Admit centered, fully covered normal motion to an optional analytical tier. Off-center footprints, spin and incompatible forces remain with the general solver.
62. Compute time of touchdown analytically across clear flight under gravity, including a foot initially moving away from the deck.
63. Solve the compressed spring around its gravitational equilibrium, preserving static sag without drift.
64. Locate lift-off inside a public tick and carry the remaining time into free flight.
65. Skip complete elastic bounce cycles while accumulating their exact contact impulse. Long intervals agree with thousands of shorter steps.
66. Handle a grazing minimum that touches zero pressure and immediately reloads.
67. Record the interior maximum force even when both public endpoints are unloaded.
68. Follow an elevator in the deck frame while retaining the exact world pose and velocity.
69. Apply a sideways shove and torque to the live foot. At tick 151 the exact path gives the tick back to the general 3D solve.
70. Time both complete Bevy worlds on the same 1 ms clock and audit vertical momentum and mechanical energy over 600 ticks. The worst foot-position difference across the original five pairs was under 3 nm.

[Elastic tier and tests](../crates/gz-physics/src/sole_contact/flat_tier.rs) · [Five live comparisons](../artifacts/multifidelity-flat-sole-tier/measurements.json) · [Run the comparison](../crates/gz-sim-cli/examples/flat_sole_tier.rs)

### 71–80. Rotating prescribed support

71. Advance a finite turntable with a prescribed world angular velocity about its translating center.
72. Compute the local surface velocity at each pressure point. A co-rotating tilted sole has zero relative slip and normal heating.
73. Transfer turning friction to the rigid foot as angular impulse. The independent spin-up case checks angular momentum.
74. Separate rotational actuator work from translational support work and include both in the energy balance.
75. Carry strain across the moving rim of a rotating finite panel. Its instantaneous flux agrees with independently differenced stored energy.
76. Detect a rotating narrow rim crossing between coarse sample stages and refine through it.
77. Control turntable time and surface resolution using per-tick errors in work as well as force and motion.
78. Apply an SDF turntable twist and stop it at the next advancing public tick. Its orientation and the angular velocity are visible through the ordinary scene.
79. Qualify braking with an independent torque pulse. After the stop, rotational actuator work was zero for 80 ticks while the foot kept spinning.
80. Compare fixed, adaptive and fine turntable trajectories at all 230 public clocks. Browser verification covered 294 measured attitudes and 147 actuator-work indicators.

[Rotating support](../crates/gz-physics/src/sole_turntable.rs) · [Turntable tests](../crates/gz-physics/src/sole_turntable/tests.rs) · [Turntable measurements and replay](../artifacts/multifidelity-live-sole-turntable/README.md)

### 81–90. Fast damped motion and numerical robustness

81. Reduce the 3D material calculation to a single pressure value per midpoint stage when a damped sole is centered, fully supported and moving normally.
82. Carry normal heat, impulse, recoverable strain and the final 3D pressure polygon through the reduced step. The unloading and elevator tests compare these observables directly with the full spatial solver.
83. Give a requested adaptive error budget precedence over the fixed-grid damping shortcut. The loaded scene continues to publish the actual temporal and spatial estimates.
84. Restore eligibility for the reduced damping solver after a loaded-world reset. A lateral command while paused takes the next advancing step back to 3D.
85. Propagate certified free flight directly in world coordinates in all three solver branches. This preserves a small motion in the presence of a very large deck offset or common velocity.
86. Compute the elastic natural frequency for stiffness-to-mass ratios whose quotient overflows or underflows before its square root. Representable frequencies pass the extreme-ratio controls.
87. Guard against false panel separation when an overlap is shorter than one representable arrival-time increment. Large opposite-sign world coordinates also retain a valid swept ownership decision.
88. Reject nonfinite transformed footprints rather than certifying an invalid airborne state. The incoming state stays available for diagnosis.
89. Qualify 100 physically distinct layout/drop combinations against independently integrated normal mechanics. Every case records both foot trajectories, each panel’s load-cell wrench, impulse, work, pressure center, and support margin where measurable.
90. Compare the reduced damping solver with the fixed 128-substep 3D baseline under gravity for 600 public ticks. Center separation stayed below 10⁻¹⁶ m in this symmetric test, heat agreed within the test tolerance, and p95 was 4.53 µs versus 76.82 µs in the recorded run.

[Damped tier](../crates/gz-physics/src/sole_contact/damped_tier.rs) · [Live command and reset controls](../artifacts/multifidelity-local-sole-program/checks/flat-live-tests.log) · [Damped-tier measurements](../artifacts/multifidelity-flat-damped-tier/measurements.json) · [100-case Gazebo archive](../artifacts/multifidelity-panel-landings/replay/manifest.json)

### 91–100. Authored fixed joints and the next contact discrepancy

91. Construct a six-axis weld from an authored SDF fixed joint on the selected local multifidelity backend. The two links retain separate Gazebo identities and collision shapes.
92. Initialize a free welded tree at a single assembly twist while accounting for each link’s COM offset and rotation.
93. Accept a wrench applied to either welded link and include its moment arm relative to the total COM. A successful step consumes the applied load once.
94. Hold a world-anchored welded tree fixed under gravity and a force on the upper link.
95. Keep a mixed free hinge or disconnected model from being reclassified as a rigid welded assembly. Other backend selectors preserve their existing admission.
96. Expose the welded assembly’s mass, world COM, full inertia tensor, momenta and held wrench through either link. An independently rotated parallel-axis calculation checks the tensor.
97. Restore the assembly and its published joint and link states on reset. A nonzero initial twist reproduces the trajectory over 300 steps.
98. Compare a true two-link weld with a consolidated rigid body after a shove. The measured link positions differed by at most 1.37 µm in flight. The visualizer used both links’ actual Gazebo poses.
99. Repeat the rigid-floor impact at 500, 250, 125 and 62.5 µs. The first load stayed in the same 84–84.5 ms observation window, but the peak COM discrepancy did not decrease monotonically.
100. Isolate the first differing ground response. At 84 ms the incoming COM and both momenta agreed. At the next 0.5 ms boundary the first normal impulse was 1.077 versus 1.405 N·s and the angular impulses differed too. The retained before/after states, native impulses and verified replay provide the next solver regression.

[Weld controls](../crates/gz-sim-scene/tests/local_weld.rs) · [First divergent contact](../artifacts/multifidelity-native-welded-foot/checks.json) · [Ground convergence and gz-web-rs replay](../artifacts/multifidelity-native-welded-foot/README.md)

## Reproduction

From the multifidelity worktree, generate into fresh output directories. The existing frontend listens on port 8082. Each replay server prints its WebSocket address.

```sh
export RUSTC_WRAPPER=
cargo build --release --locked --offline --no-default-features -p gz-sim-cli \
  --example panel_landings --example sole_material_holdout --example flat_sole_tier --bin gz-sim-rs
target/release/examples/panel_landings target/reproduce-panel-landings
target/release/examples/sole_material_holdout target/reproduce-panel-landings/cases.json target/reproduce-panel-fit
target/release/examples/flat_sole_tier target/reproduce-flat-tiers
target/release/gz-sim-rs --server --replay --web-viewer 9701 target/reproduce-panel-landings/replay
```

Open [the reproduction viewer](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9701%2F) and press Play. To review the flat-tier run instead, serve its replay directory. The material study’s independent comparison figure and measurements are generated alongside its fit report. The fixture, two-panel, spinning deck and welded foot have their own reproduction commands in the linked study guides.

## Verification

The confirmed locked release run passed 1,572 tests across the five physics and scene crates, plus two additional CLI scene tests for tier selection. Strict Clippy passed for the new production kernel, material fitting and replay examples. Chromium WebGPU checked 1,823 recorded frames and 24,766 visual poses and shapes across nine replays. The 1,300-frame panel archive was verified on a second loopback server while the desktop displayed the same recording on port 9692. [Full test log](../artifacts/multifidelity-local-sole-program/checks/confirm-tests.log) · [CLI regressions](../artifacts/multifidelity-local-sole-program/checks/flat-live-tests.log) · [Browser coverage](../artifacts/multifidelity-local-sole-program/checks/browser-coverage.json)

## Open work

The matched native floor strike and two-link compliant landing are qualified. The coated lower face now accepts a free box, a true two-box compound, or a floating two-link fixed assembly with diagonal aggregate inertia in the sole frame. An explicitly authored frictionless corner spring can now act on one finite panel side while the lower coating remains loaded. The coupled oblique strike converged against independently integrated rigid-body momentum, and a laterally displaced ledge produced no upper impulse. Unresolved upper contact still stops the tick before publication and preserves pending loads. Open work includes rigid frictional multi-point upper contact, free joint coordinates and contacts outside the enrolled panels. [Upper strike and miss replays](measurements/local-manipulation/compliant-upper.md) Multiple translating panels require parallel normals and disjoint projected interiors throughout the tick. A single support may rotate. [Welded compliant foot measurements and replays](measurements/local-manipulation/compliant-weld.md)

Wall-clock timings describe the development host and workload recorded in each report. The final code rerun took 4.03 µs at p95 for the centered damped foot versus 71.8 µs for the fixed 3D solver. The largest center-coordinate difference from the archived five-world replay after the free-flight precision fix was under 6 × 10⁻¹⁵ m. [Final measurements](../artifacts/multifidelity-local-sole-program/checks/flat-final-measurements.json)

The flat-tier examples use 600 public ticks with the first five excluded from timing and alternate which comparison lane steps first. The damped and elastic replays preserve the two successive benchmark generations.
