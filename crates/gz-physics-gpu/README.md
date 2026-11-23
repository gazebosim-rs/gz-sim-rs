# Gazebo GPU physics batches

The optional Rust crate runs independent f64 fixtures on a hardware Vulkan adapter. The Bevy simulation APIs start paused, advance on the Gazebo clock in 1 ms ticks, and restore the construction states on reset. The vertical cable/receiver profile uses `simulation::batch_simulation`. The 3D solid-sphere profile uses `simulation::sphere_plane_simulation`. Both use a Rust-authored Naga compute graph and publish a complete host snapshot after the fenced device submission succeeds.

## Oblique sphere and stationary plane

Each environment contains one homogeneous solid sphere and a fixed infinite plane. The outward world unit normal **n** and offset **d** define the admissible region **n** · **x** ≥ **d**. Supply SI position, velocity, angular velocity, vector gravity, radius, inverse mass, Coulomb friction coefficient and an xyzw unit quaternion. An initial gap or exact surface contact is supported. Initial overlap and invalid unit vectors, inertias, output fields or nonfinite inputs are rejected. Outputs give the impulse on the sphere over the last 1 ms tick, separated into a nonnegative scalar normal impulse and a world-frame tangential vector.

Each tick applies gravity, sweeps the center to the plane surface, closes normal velocity with zero restitution and solves point friction from the full surface slip. With mass **m** and radius **r**, the solid-sphere inertia is 2**m****r**²/5. A sliding contact opposes slip at the Coulomb limit. A sticking contact removes the tangential speed at the point of contact. Torque acts at the sphere surface. Translation and normalized orientation are integrated on both sides of impact. A stationary plane can receive support reactions, and spin around its normal remains torque-free.

Run the release qualification with the exact hardware name reported by the local Vulkan driver. The optional recording directory must be new.

```sh
RUSTC_WRAPPER= cargo run --offline --locked --release -p gz-physics-gpu \
  --example oblique_sphere_plane -- --adapter 'NVIDIA GeForce RTX 5090' \
  --capture target/oblique-viewer/my-replay
GZ_GPU_ADAPTER='NVIDIA GeForce RTX 5090' RUSTC_WRAPPER= \
  cargo test --offline --locked -p gz-physics-gpu --lib --tests -- --include-ignored --test-threads 1
```

The qualification compares all 2,048 lanes for 600 ticks with a Rust CPU contact solve that derives tangential mobility from the point moment arm and inertia. It independently checks linear and angular impulse balance, contact energy dissipation, friction-cone membership, sticking slip, normal spin preservation and penetration. An independently configured native f64 Rapier world runs eight finer steps on each of 64 lanes per public tick. Instantaneous impact velocity can differ until both trajectories have crossed the impact. The report includes full, unfiltered maxima, impact timing, post-impact differences after four public ticks, and accumulated impulse. Timing starts after these checks and includes upload, submission, fence, dense readback, decode and host consumption.

The local RTX 5090 replay consists of 61 snapshots, every 10 ms from 0 to 600 ms. Each tile displays part of an independent infinite plane. Blue spheres have zero friction, green low friction, amber intermediate friction and red high friction. Three white markers on each sphere rotate with the measured body orientation. Gazebo's ordinary replay server can open the resulting archive.

```sh
RUSTC_WRAPPER= cargo run --offline --locked -p gz-sim-cli --bin gz-sim-rs -- \
  target/oblique-viewer/my-replay --server --replay --web-viewer 9037
```

In the local hardware run the GPU and independent directional CPU solve agreed within 8.1 × 10⁻¹⁵ m across 1,228,800 lane-steps. Against native Rapier the maximum center distance was 0.981 mm, the maximum impact offset was one tick, post-impact accumulated impulse differed by at most 0.00106 N·s, and post-impact rim speed differed by at most 0.00165 m/s. Two measured runs with dense readback produced 13.0–13.5 million lane-steps/s on the GPU and 14.1–15.9 million on a serial CPU, with 221.2 MB transferred in 500 measured ticks per GPU run. The optimized CPU was faster in both. The local logs and real viewer recording are under `artifacts/multifidelity-oblique`.

## Resident observation blocks

`SpherePlaneBlockBatch::step_block(ticks)` advances 1 through 4,096 one-millisecond physics ticks and publishes one observation. Each lane executes the same sphere/plane update in a bounded, register-local shader loop. Each call submits one compute dispatch, copies one endpoint buffer, and waits for its fence. Four bytes select the current block length. The complete state is uploaded at construction, reset or recovery. Between successful blocks the state remains on the device.

Each `SpherePlaneBlockObservation` includes the final `SpherePlaneLane`. Its instantaneous impulse fields describe the final 1 ms tick. The block also publishes the ordered sum of normal and tangential impulses, peak normal impulse, minimum sampled surface clearance including the initial endpoint, contact tick count and first/last one-based contact offsets. A contact tick applies a strictly positive normal impulse. All reductions begin again at the next observation. An invalid block length, failed readback or nonfinite intermediate preserves the last published state and tick. The next attempt reseeds from that snapshot if a device dispatch had started.

Use `simulation::sphere_plane_block_simulation(adapter, initial, ticks)` to schedule fixed-cadence observations through Bevy. At a cadence of 64, each successful public barrier advances 64 ms. Pausing freezes the clock and reset restores the original epoch state. The Rust batch API accepts a new block length at every call. Systems that need individual millisecond observations can use the dense sphere/plane simulation.

```sh
RUSTC_WRAPPER= cargo run --offline --locked --release -p gz-physics-gpu \
  --example resident_sphere_plane -- --adapter 'NVIDIA GeForce RTX 5090' \
  --out target/my-resident-qualification
```

The example checks 18 different observation boundaries covering 4,573 ticks, including the maximum 4,096-tick request. It compares every endpoint byte with repeated single-tick device steps over 2,048 lanes. It also reconstructs the ordered contact sums, maxima, discrete counts and first/last offsets from the dense observations. Minimum clearance uses a roundoff tolerance for the host/device dot product. Independent directional CPU and independently refined native Rapier trajectories supply two numerical references. A dedicated hardware test induces an overflow on the second internal tick, verifies rollback, and retries the valid prefix.

The measured grid uses 64, 512, 2,048 and 8,192 independent spheres. The largest batch repeats the 2,048 geometries four times. Each measurement warms up for 256 ticks and times another 8,192 ticks. The CPU computes the full block reductions. Both timed paths consume every endpoint and contact summary on the host. The separate dense GPU row observes normal one-tick states. The CSV reports p99 per host observation, every explicit transfer, GPU dispatch/copy timestamps and host scheduling/decoding/observation costs. Times are machine and driver specific.

| Local RTX 5090, 64 ticks per observation | GPU million lane-steps/s | Serial CPU million lane-steps/s | GPU p99 per result |
|---|---:|---:|---:|
| 64 spheres | 6.63 | 12.21 | 0.694 ms |
| 512 spheres | 48.79 | 11.96 | 0.782 ms |
| 2,048 spheres | 147.39 | 14.45 | 1.511 ms |
| 8,192 spheres | 200.92 | 11.13 | 6.721 ms |

The 2,048-sphere dense GPU baseline achieved 11.69 million lane-steps/s and read 3.624 GB over the measured horizon. At 64 ticks per observation the resident run read 77.60 MB, while preserving the block endpoint and contact integrals across the closed interval. It exposes a host observation once every 64 simulated milliseconds. Compared with the independent CPU oracle over the qualified horizon, maximum position error was 1.13 × 10⁻¹² m and maximum integrated normal or tangential impulse error was 5.35 × 10⁻¹² N·s. The 16 refined native worlds differed by up to 9.86 mm and 0.00190 m/s, with at most one public millisecond of impact timing difference. Their largest sign-invariant quaternion chord was 0.0556, equivalent to about 6.37° of spin-phase difference after the 4.573 s trajectory.

## Two finite spheres in free space

`simulation::sphere_pair_simulation` creates a paused Bevy world. Each `SpherePairLane` contains two independent solid-sphere states, common uniform vector gravity, a Coulomb friction coefficient and Newton restitution in [0, 1]. Sphere radii and positive inverse masses can differ. Supply initial orientation and spin for both bodies, zero initial impulse and contact normal, and initial `impact_time = -1` to indicate an empty observation.

Relative acceleration is zero between impacts, so the solver sweeps the exact relative center line against the sum of radii, including fast crossings and glancing strikes. A 64-epsilon band in relative-coordinate length closes a root at the 1 ms endpoint. The contact point exchanges one equal and opposite impulse. Its normal component implements restitution, and its tangential component uses both solid-sphere inertias and the full incident surface slip. A tick reports time of impact in seconds from its start, the unit direction A→B, normal impulse magnitude and a world tangential impulse on B. A tick with no impulse reports time -1. The two centers integrate ballistic motion under the common gravity, with the collision impulse applied over the remaining part of the timestep. This free-space profile contains exactly one pair per environment.

```sh
RUSTC_WRAPPER= cargo run --offline --locked --release -p gz-physics-gpu \
  --example spinning_spheres -- --adapter 'NVIDIA GeForce RTX 5090' \
  --capture target/sphere-pair-viewer/my-replay
```

The 2,048-lane fixture covers equal and unequal masses, spin, perfectly elastic and inelastic response, mixed friction and restitution, and near misses. Across 600 ticks, all 1,792 incident pairs and 256 misses agreed with the independent quadratic time-of-impact / directional point-mobility CPU reference. Maximum center, impulse and event-time differences were 3.82 × 10⁻¹⁴ m, 7.67 × 10⁻¹⁴ N·s and 4.78 × 10⁻¹⁵ s. The tests separately check closed-endpoint and 1 nm beyond-endpoint cases, rolling-point slip, gravity-driven center of mass, orbital plus spin angular momentum, energy loss and equal/opposite forces. The verification includes a 6,000 m/s relative-speed pair that crosses both spheres in one millisecond without the swept test.

Sixty-four native f64 Rapier worlds run at 16× finer resolution. Rapier applies restitution after its friction iterations. When both bodies slide and rebound, this can limit the native tangential impulse by the compression impulse. The GPU caps Coulomb friction using the complete normal impulse, including rebound. The example reports unfiltered native differences, timing of both impact samples, and that friction-cap difference separately. Both methods found all 56 native impacts. On lanes with the same friction law the maximum center difference was 0.556 mm and speed difference 0.00114 m/s. Across all lanes the maximum center and post-impact speed differences were 21.6 mm and 0.0612 m/s. The largest integrated impulse difference was 0.0781 N·s, while the largest difference between the two friction caps was 0.0779 N·s.

Two dense RTX 5090 runs measured 13.16–13.79 million GPU pair-ticks/s and 13.58–14.39 million CPU pair-ticks/s. Each read 352.27 MB back to the host over 500 measured ticks after 100 warmup ticks. In the replay, cyan marks sphere A, amber marks B, white dots show rotation, and red marks the reconstructed contact point for 40 ms after the measured event. The three rows are independent free-space experiments displayed with separate spatial offsets.

## Two spheres sharing a frictionless plane

`simulation::supported_sphere_pair_simulation` admits an invariant family with two simultaneous external supports and one possible internal collision. Both spheres have the same positive radius, have centers one radius above the plane, move tangentially, and spin around its normal. Their masses can differ. Gravity must press both bodies into a fixed frictionless plane and may also accelerate them along that plane. Construction rejects unequal radii, lifted centers, normal velocity, out-of-plane spin, outward or zero normal gravity, invalid geometry and prepopulated observations.

The pair’s normal and tangential contact impulses lie within the plane in this family. Each external support acts along the plane normal. Those subspaces are orthogonal, so the solver evolves the internal impact with both dynamic masses and publishes the external normal impulse on each sphere separately. It projects only the roundoff-sized residuals back onto the admitted manifold. A general oblique 3D stack, contacts with static plane friction, or lift-off requires a coupled contact solve beyond this profile.

```sh
RUSTC_WRAPPER= cargo run --offline --locked --release -p gz-physics-gpu \
  --example supported_spinning_spheres -- --adapter 'NVIDIA GeForce RTX 5090' \
  --out target/my-supported-qualification --capture target/my-supported-replay
```

The example compares each GPU step with an independent tangent-space CPU event solve and 64 native three-collider f64 Rapier worlds using actual per-collider support impulses. It tests total linear momentum including both external impulses, relative orbital plus spin angular momentum, zero normal work, support clearance, no lift-off, contact energy loss and the tangential friction cone. All 1,792 impacts in 2,048 environments occurred while both spheres carried a positive external load. Across 600 ticks, maximum GPU center error against the analytic fixture was 4.75 × 10⁻¹⁴ m and maximum accumulated native support impulse difference was 7.68 × 10⁻⁸ N·s. At 16× native refinement, maximum center error was 0.293 mm on shared-friction-law lanes and 29.4 mm including the mixed sliding/rebound lanes.

The hardware refinement study rescales time at fixed mass and length. Dividing initial linear and angular velocities by **f** and gravity by **f²**, and running **f** one-millisecond GPU ticks per physical millisecond, preserves the continuous sphere trajectories. Physical impulses are **f** times device impulses. Endpoint poses and integrated impulses are compared with a single swept event, piecewise analytic ballistic motion and exact quaternion exponentials. At 1, 1/2, 1/4 and 1/8 ms the measured quaternion RMS convergence order was 1.9997. The finer GPU timings accumulate sub-picometer roundoff in center position. Native comparisons at 1, 1/4, 1/16 and 1/64 ms separate temporal error from the persistent mixed-friction response. At 1/64 ms the common-law maximum center error was 0.0528 mm across 44 sampled bodies. The CSV and SVG are written under the selected output directory.

Three dense RTX 5090 runs measured 5.75–7.58 million supported GPU pair-ticks/s and 9.23–14.39 million serial CPU pair-ticks/s, including the final validation run during other workspace activity. Each GPU run read 401.42 MB over its 500 measured ticks. The two display tiles show physically independent equal-mass elastic and unequal-mass sliding/rebounding experiments, each on its own measured inclined support plane.

## Independent models in one Bevy batch

`simulation::mixed_batch_simulation` accepts explicitly tagged `Environment` values. Each has a caller-selected `u64` identity and a sphere/plane, free two-sphere, or supported two-sphere state. Registration validates every profile, rejects duplicate IDs, and sorts the published frame by ID. Physical device slots remain private. Each family retains its documented dynamics and admission bounds. The environments evolve independently.

One explicit hardware f64 Vulkan device runs the nonempty groups in one compute pass. Each binding covers its own aligned range in one resident storage buffer. A single dense copy and fence precede the publication of all three groups at each 1 ms Bevy tick. Any invalid family result, device error or mapping failure retains the previous complete frame. A direct batch retry reseeds all groups from the last committed state. The ordinary Bevy world retains its terminal fault semantics. A whole-world reset restores the construction frame, stable order, and ordinary Gazebo epoch behavior.

```sh
RUSTC_WRAPPER= cargo run --offline --locked --release -p gz-physics-gpu \
  --example mixed_environments -- --adapter 'NVIDIA GeForce RTX 5090' \
  --out target/my-mixed-timing --capture target/my-mixed-viewer
```

The example checks 683 sphere/plane, 683 free-pair, and 682 supported-pair environments for 600 consecutive public ticks. Against independent f64 scalar models the maximum center discrepancy was 3.53 × 10⁻¹⁴ m and the largest normal/tangent pair impulse discrepancy was 1.19 × 10⁻¹³ N·s. Ninety-six Rapier worlds ran at the same 1 ms observation cadence with 8 or 16 internal substeps. The complete log reports raw contact timing, position, speed, rotation and impulse discrepancies as well as the previously qualified mixed-friction-law differences. All 600 frames matched the three corresponding homogeneous GPU kernels byte-for-byte. Two failure controls exercise a late overflowing contact in a valid initial registration and a damaged final group. The latter recovers on direct batch retry.

On an RTX 5090 with driver 595.99.02, the 2,048 environments used 649,824 bytes of physical state with 48 bytes of binding alignment. The driver read back 649,904 bytes per epoch including timestamps. Resident ticks uploaded zero bytes. Allocated GPU buffers occupied 1,299,808 bytes. The three host snapshots, upload slab and offset table occupied 3,173,008 bytes.

Three release trials included a CPU consumer of every field and 500 measured ticks following 100 warmup ticks. Shared GPU throughput ranged from 3.81 to 8.23 million environment-steps/s, with p99 latency of 0.282 to 2.46 ms. Three serial homogeneous GPU submissions delivered 2.12 to 4.15 million environment-steps/s. The matched single-thread scalar CPU delivered 9.41 to 14.58 million. The first trial ran during concurrent workspace builds. The subsequent two trials produced 7.94 and 8.23 million environment-steps/s for the shared GPU. The output directory contains per-tick wall latencies and their empirical distribution. Native Rapier qualification runs outside the timed section.

The viewer displays three independent environments with vertical offsets. The upper green sphere falls onto an inclined plane, the middle cyan and amber spheres collide in free space, and the lower pair collides while each sphere carries a positive support load. The small white marks follow each sphere's measured rotation.

## Asynchronous reinforcement-learning episodes

The mixed Bevy world supports selected episode resets and center-of-mass impulse actions at a committed `BatchBoundary { epoch, tick }`. Pass that stamp to `simulation::reset_environments` or `queue_environment_impulses`. A selected reset keeps its ID and profile, can replace its state and physical parameters, and clears its previous episode's queued action. An impulse targets sphere A or B where present. The invariant supported-pair profile accepts tangential impulses. Admission validates the complete submitted set atomically. A paused tick retains pending commands. A successful advancing tick consumes each command once.

Selected replacements appear immediately in the committed frame at the same simulation time. Each untouched environment retains its state. At the next advance, the GPU receives only changed records, merging adjacent physical ranges without overwriting an intervening resident lane. Startup, full reset and failed advances require a full reseed. `get_many` returns references in requested ID order from the complete host frame. A minibatch observation still uses the underlying dense device readback.

`sampling::counter` and `uniform` derive each draw from a master seed, stable environment ID, committed step, stream and draw ordinal. Pausing, changing admission order or reading a different minibatch consumes no shared random state.

`simulation::checkpoint_mixed` captures the full-reset seeds, committed physical states, admitted actions, clock, epoch and pause mode. `MixedCheckpoint::to_bytes` and `from_bytes` use a versioned little-endian encoding with a 128 MiB upper bound and a SHA-256 integrity check. `restore_mixed_simulation` validates the saved physical profile and recreates the Vulkan and Bevy session at the saved public boundary. It checks the kernel source fingerprint, hardware IDs and driver. The execution guarantee covers the same build and target. Application resources such as a learner's Q table and per-episode state belong to the application. The example includes them in its durable experiment checkpoint, together with its independent CPU reference.

```sh
RUSTC_WRAPPER= cargo run --offline --locked --release -p gz-physics-gpu \
  --example episode_learning -- --adapter 'NVIDIA GeForce RTX 5090' \
  --seed 20261119 --out target/my-policy-training --capture target/my-policy-replay
```

The example trains a 27-state tabular Q policy in 2,048 mixed environments. It derives the state from profile and signed X velocity error. Three actions apply a center impulse to change X speed by −0.28, 0 or +0.28 m/s. Rewards penalize tracking error and control effort. It samples physical conditions, target, action period and episode duration independently. Episodes last 129–257 one-millisecond ticks and decisions occur every 12–20 ticks. Held-out evaluation freezes the learned policy and compares it with the zero-impulse policy on identical initial conditions and resets.

Training seed 20261119 performed 11,201 selected resets and admitted 103,646 impulses over 1,152 physics ticks. A 2,169,547-byte checkpoint at tick 257 retained 15 just-reset states, 112 pending impulses and the training state. Reversed registration and 73-ID minibatches reproduced every field bit and the complete policy through the final tick, including a fresh-device restore. The tests also recover queued actions after a damaged unrelated lane faults the Bevy world.

| Held-out profile | Mean absolute velocity error, trained | Same episodes without control |
| --- | ---: | ---: |
| Sphere on a plane | 0.192 m/s | 0.980 m/s |
| Free pair | 0.246 m/s | 0.983 m/s |
| Supported pair | 0.240 m/s | 0.973 m/s |

Three independent training seeds (20261119, 2718 and 42991) passed the 35% relative improvement gate for every profile. For the first seed, 96 native Rapier trajectories per evaluated policy received the same reset states and commands at the public 1 ms cadence. Mean absolute per-tick task discrepancies between the trained GPU and refined native controls were below 0.00036 m/s in all three families. The raw native differences include the impact tick; the peak instantaneous speed difference in the supported family was 1.31 m/s. The generated CSV and plot show the trajectory of one held-out environment. The viewer displays its trained sphere above its uncontrolled counterpart. Green markers follow the episode's requested X velocity from the initial position.

Across the three seeds the RTX 5090 delivered 4.24–4.67 million held-out policy environment-steps/s and 0.760–0.964 ms p99. The measurement includes policy decision, command admission, physical GPU simulation, full readback and reward calculation. Separate CPU and native qualification, checkpoint I/O and replay export run outside those timings. In the first held-out run, 5831 selected resets and 49,570 commanded impulses used 15,886,016 uploaded bytes over 640 epochs. Dense observations read back 415,938,560 bytes. This workload measures host-side tabular learning with GPU physics. It provides no policy-network training or inference benchmark.

## Guarded native staging in an SDF world

Enable `gz-sim-scene/gpu-batch` and mark independent free sphere pairs or sphere–plane worlds with `native_handoff="true"`. Settled contacts retain native manifolds until guarded handover becomes physically admissible. The ordinary SDF entry accepts `<native_staging workers="2" overlap_gpu="true"/>` inside the GpuBatch world plugin. Worker budgets span one through four, including the caller. Omitting the element selects serial CPU-first staging. Both execution orders commit one frame after the CPU results and device readback agree on a successful shared boundary. [Native handover and support](../../artifacts/multifidelity-native-support/README.md), [worker timing](../../artifacts/multifidelity-native-staging/README.md), [matched end-to-end overlap measurements](../../artifacts/multifidelity-native-overlap/README.md)
