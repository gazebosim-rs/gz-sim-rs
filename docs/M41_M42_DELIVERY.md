<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M41 constant-wind AirSpeed and M42 joint-force feedback

These are the next bounded source lanes under D30–D32. Each has a maximum of
five working days including review and mandatory checks. Source preparation may
overlap the frozen M39/M40 hosted and publication closeout. This worktree starts
at now-accepted Main `470da6e699ecf9c442a7f1daa4cb3c36146de8f8`; actual M39/M40
release acceptance is recorded in M39_M40_ACCEPTANCE.md. This parent is now bound;
complete independent source review still precedes native execution, consolidated
verification or publication. A parent correction requires
affected rebasing and review. M40 release440119258 is installed Latest; this source wave includes no distribution refresh.

Root continues M39/M40 closeout first, integrates shared files and serializes
heavy work. No completed test, native capture, installation or browser proof is
repeated for planning. No implementation starts until this contract and its
interfaces have independent source review. Each feature retains its own proof;
one combined candidate must pass the full normal hook, unchanged strict coverage,
exact hosted CI and independent local/official audits before acceptance.

## M41: finite constant WORLD wind for the existing AirSpeed sensor

Admit finite SDF WORLD wind only when the complete existing typed, single
AirSpeed preflight succeeds. Keep no-consumer and other-only worlds with nonzero
wind refused; retain model/link wind-mode refusals, default atmosphere, Rapier,
topology and sensor-count restrictions. This changes sensor-relative airflow,
not physical forces. There is no WindEffects system, wind service, runtime wind
mutation, new public sensor API, transport or backend.

The private AirSpeed component gains immutable `wind_world: [f64; 3]`, copied
from a private SceneContext field. InitialScene's existing component clone and
restore carry that value. `air_speed::preflight` returns `Result<bool, LoadError>`:
true only after a complete admitted AirSpeed visit, false when absent. Root
passes that result into producer_guard alongside the existing magnetic-consumer
flag. Wind must always be finite. Model/template sensor spawning restrictions
remain unchanged.

Preserve the existing WORLD point velocity and existing sensor-frame velocity
calculation. For nonzero wind, compute the native reverse rotation as
`Inverse(q) * (Quaternion(0, wind_world) * q)`, preserving the pinned upstream
Hamilton scalar evaluation order, then subtract that separately rotated wind
from the existing sensor-relative velocity. Do not rotate a pre-subtracted WORLD
vector, normalize, substitute a shortcut or change pressure/noise/f32 arithmetic.
An entirely zero vector, including signed zeros, takes the exact old bypass;
retain requested and effective configuration bits in reference evidence. Mixed
signed zeros in a nonzero vector remain unchanged. Preserve existing scheduling
and error history: invalid or overflowing ideal input produces the existing
`AirSpeedSample.reading = Err(AirSpeedSampleError::Ideal(InvalidAirSpeedInput))`.
It produces no valid canonical publication or noise draw. Do not turn this into
a terminal clock fault or change the existing schedule-before-reading contract.

Use the existing pinned original Sensors AirSpeed component/provider and trace
the original Sim handoff in source. Retained M26 zero-wind maps show its historical original AirSpeed system DSO
was loaded. Current system-DSO availability and any new nonzero-wind system
execution are not established; do not inherit that execution or build an SDK. The
fresh component reference uses direct WORLD sensor pose, already sensor-relative
velocity, WORLD wind, actual time and noise configuration. It does not reuse the
M39 MODEL-to-WORLD transform. Preserve real provider/source/recipe identity,
actual pressure/temperature bits and canonical bytes, requested/effective wind,
suppressed updates and connection/update conditions. No RNG stream parity claim.

Acceptance includes head/tail/cross/cancelling wind, tilted/world-Z and offset
rotating sensor cases, mixed-zero and f32 boundary cases; zero/no-noise and
existing zero-stddev Gaussian behavior; real Scene motion and canonical Server
packets; demand absence, pause/due/reset/public-ID recovery, disconnect/removal,
and finite/overflow negatives. A formerly refused finite wind input becomes a
strong load/motion/pressure/healthy-reset positive. Replace only that obsolete
negative with an appropriate malformed/nonfinite case and preserve its neighbors.
Physical body motion must match the zero-wind case. Retain all M26/M28 controls.

## M42: joint-force command and canonical JointState feedback

Deliver a runnable world-hinge Rapier example combining existing ApplyJointForce
and JointStatePublisher, plus a minimal `force` mode in the existing bounded
joint_command helper. Keep its position and velocity modes unchanged. Force
publishes canonical gz.msgs.Double at `/model/rotor/joint/hinge/cmd_force`, uses
finite Nm and the existing bounded repeat argument shape, then explicitly sends
zero effort. Zero effort coasts; it is not a braking or stop command.

The example has gravity zero, a 1 ms step, a sphere of radius 0.2 and mass 1,
diagonal inertia 0.016, COM zero, and a WORLD revolute hinge about (0,1,0).
JointStatePublisher emits at 100 Hz. Preserve the existing world/fixed-parent
effort domain and explicit moving-parent Rapier refusal. The retained rotating-
parent physical failure is not repaired by lifting that guard or applying a
world-space couple. No new mechanical domain, solver, effort sensor or core
production behavior is introduced.

The real C++ canonical peer publishes commands and receives complete gz.msgs.Model
feedback. Validate the model identity/public ID/pose, sorted joint identity,
parent/child/construction pose, axis/limits/damping and actual position/velocity,
and every existing absent/default field. Preserve valid infinite joint limits.
The axis force field is zero and unmeasured, not a force echo. Admission counters
and actual JointSample/physics changes must agree. Commands admitted at the end
of a tick affect the next advancing step, not the current physics/publication.

Use the unchanged six native effort pulses [1.5,0,-3,0,2,0] at 1 ms with coast
through 10 ms, plus a bounded positive/zero/negative workflow. The retained
cross-backend native contract compares joint velocity at its unchanged absolute
1e-12 bound. It does not qualify DART position as a Rapier trajectory reference.
Compare actual Rapier position and velocity with a paired simulation using the
same physical SDF, the force plugin omitted and the existing direct joint-effort
queue, while retaining complete canonical
feedback comparisons to actual JointSample values. The Server counterpart uses
the same ten zero-effort warmup steps. Do not change the native fixture, inertia,
backend or tolerance, or claim a new native Physics campaign. Compare sample
epoch/revision before and after
every Server update; drain actual packets only when changed and preserve FIFO
duplicates at equal wire timestamps. JSP can publish at paused/equal time; do not
invent wire epochs, assume one packet per timestamp, or add a general barrier.

Acceptance includes real canonical input/output, force/coast/opposite-force
motion, reset clearing admitted commands/history, new internal handles with
stable public IDs, fresh paused publication after reset, removal, disconnect and
owned peer shutdown. Sensor and joint models use the existing canonical blocking
MODEL removal service: require one actual query, a true Boolean response and one
completed-removal increment before retirement assertions. Preserve the bounded
model-edit API's refusal and world state; do not widen that production domain. Existing JSP acquisitions remain unconditional, including
disconnected and paused operation; disconnect closes peer/IO delivery, not sample
acquisition. Add no subscriber-demand or pause gate. Preserve existing unread-queue/callback retirement controls;
do not claim cancellation of remote in-flight packets. Keep old native controls,
ApplyJointForce and JSP source/contracts unchanged.

## Disjoint ownership and integration

Light owns `crates/gz-sim-scene/src/air_speed.rs`, new M41 Scene test/fixture
leaves, `examples/m41_air_speed_wind.sdf` and `docs/M41_AIR_SPEED_WIND.md`.
Messages owns new M41 component-reference/Server/canonical leaves and new M42
Scene/Server/C++ test leaves, `examples/m42_joint_force_feedback.sdf` and
`docs/M42_JOINT_FORCE_FEEDBACK.md`. Agree actual filenames with Root before edits.
SDF independently reviews the interfaces, native source and the complete batch.
Root owns shared lib/context/admission/initial/scheduling/IO/registry/CLI/retention
seams, `joint_command.rs`, planning and acceptance records. Agents do not edit
each other's files or shared seams. Source disagreements return in messages.

Complete implementation, regression source and guides, batch all independent
review findings, then one consolidated qualification. Root alone releases any
fresh native component capture after its complete source review and accepted-
parent binding. Generated literals require retained independent comparison;
no product test is run to trial a fixture. No dependency/framework/schema,
renderer, package refresh, optional sensor/backend or unrelated work is included.

### M42 selected peer priming seam

The Server protocol validates a paused-zero Model separately, then advances ten
zero-effort 1 ms steps and awaits the sole actual due Model at10ms on that same
topic before arming strict recording. ACK/drain waits perform no Server update.
After arming, every changed sample epoch/revision requires its exact FIFO packet;
any unexpected late priming packet fails rather than being cleared or ignored.
Server effort pulses therefore run10→20ms; the Scene native six-pulse comparison
retains its exact0 start. Reset/reconnect use a fresh subscriber lease and an
unambiguous same-Model due cutoff. A Clock on the same Node is not a cross-topic
callback-completion fence, and no such guarantee or new general barrier is used.
