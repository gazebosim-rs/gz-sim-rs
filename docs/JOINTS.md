# World-anchored revolute slice

PHY-05 / LOAD-01 adds one dynamic top-level model with one link and one
world-parented revolute joint on Rapier. TPE joints, other topologies/types,
finite or inverted limits, nondefault stop parameters, effort/velocity limits,
and nonzero damping, friction or springs return typed unsupported errors.
This is a bounded P2 variant, not full articulation or backend acceptance.

The SDF semantic graph resolves the joint anchor and axis into world coordinates.
Rapier supplies the f64 constraint solver. A colliderless fixed world body owns
constraints and retires when the last constraint disappears. Jointed bodies
cannot sleep: the default sleeping policy stopped the low-amplitude pendulum
at 501 ms. Removing the last constraint restores the normal activation policy.

The initial body pose defines joint angle zero. Matching world/local constraint
frames preserve that reference through reset. Backend-local JointIds retain a
high-water mark across reset; child removal retires attached constraints.
Construction closes on the first advancing step and reopens after reset.
Retained-body reset preserves only remaining constraints; loaded reset restores
original bodies, constraints, public IDs and owned SDF metadata in the same world.

Bevy owns joint entities under their model; physical child endpoints are public
link IDs. JointDefinition owns a cloned SDF joint. Editing it does not reconfigure
the solver. JointSample contains continuous radians, axis rad/s, integer simulation
time and reset epoch. Paused ticks preserve physical state. Joint binding/owner
corruption faults before advancing time or publishing state; removal validates
bindings before mutation. Free-body queued inputs reject jointed bodies.

Rapier reports a wrapped angle. The backend unwraps successive observations.
Its private world retains Rapier's default fast-rotation-disabled policy:
`staged_island_solver/worker.rs` caps full-step rotation at pi/4. No API exposes
that flag, so a complete turn cannot disappear between observations. This
inherited angular-speed cap is a backend limitation at large timestep/speed;
these tests do not establish arbitrary-speed fidelity.

## Evidence and limits

The actual native DART Server probe records a 4,000-step pendulum at 1 ms,
then records selected post-reset observations at iterations 1 and 10. The two
Run calls advance 11 total iterations after reset processing. Cross-engine bounds were declared before
comparison: joint position/body x,z within 1e-4 rad/m and axis velocity within
3e-4 rad/s. Separate analytic checks bound anchor drift below 1e-5 m, pendulum
small-angle error below 1e-4 rad, and energy drift below 2%. Neither different
engine trajectories nor their speed ratio constitutes same-engine parity.

`planning/reference/world-joints.json` binds source/compiler/binary/fixture,
loaded simulator, Physics system and DART hashes, plus three native controls.
The two extra controls spin at +/-4 rad/s through multiple full turns and reset;
native continuous position matches omega*time within 1e-9 rad. Installed native
dependencies and existing build provenance remain qualified P0 work. The native
reset-request poll emits a duplicate old endpoint, not a reset-time-zero sample.
Native changed-world-pose caching can lag joint feedback near turning points;
body angular velocity is not asserted identical to the joint velocity.

Rust tests retain the original assertions and tolerances. New tests cover native
observations, full turns in both directions, invalid construction atomicity,
ID exhaustion, sleep/anchor ownership, prepared reset, frames/axis resolution,
loaded metadata reset, removal and stale/missing/duplicate/foreign bindings.
The new spin test initially assumed exact omega*dt integration at 1e-10 rad.
Its failure is retained; Rapier uses normalized quaternion Euler substeps.
The corrected independent discrete expectation is 2*n*atan(omega*dt/(2*n)) per
step, with the same 1e-10 tolerance. Independent review explicitly approved this correction after checking Rapier
source and three additional frame/axis/rotation-cap controls.
No existing test, fixture or numerical tolerance was changed.

Reproduce native controls with:

```sh
python3 scripts/probe-joints.py --sim-source /path/to/gz-sim \
  --physics-source /path/to/gz-physics --sim-build /path/to/native-build \
  --dependency-prefix /path/to/native-prefix --work-dir /tmp/joint-probe \
  --output artifacts/joint-proof.json
```

Author working checks pass 194 Rust tests, 12 tooling checks and all nine existing
actual C++ peer scenarios. Aggregate coverage is 3162/3208 lines and 4914/5049
regions; every production crate exceeds 95% on both metrics. Changed lines are
197/197 in the backend and 211/216 in the scene. The exact staged gate also
passed. Independent review reproduced 139 checks,
all nine C++ scenarios, three native controls and three extra reviewer controls.
The receipt is planning/reviews/world-joints.json. Normal-hook publication and
hosted CI remain required before acceptance. The selected end-to-end fixture and equal-work performance report still block P2 acceptance.
