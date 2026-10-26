# Performance and scalability plan

The owner selected balanced robotics as the primary workload: articulated robots,
many entities, and camera/lidar sensors. Headless batch throughput and large worlds
remain required benchmark categories. [HEADLESS_MEASUREMENT.md](HEADLESS_MEASUREMENT.md)
records a scoped native/Rust TPE streaming comparison; it does not accept the
balanced suite or the proposed budgets.

## Compare equal work

Pin upstream main SHAs and the complete environment. Compare C++ Gazebo+DART with
Rust Gazebo+DART using the same native engine version, timestep, substeps, solver
iterations, collision geometry, precision, sensor counts/rates/resolutions, and
output requirements. Do the same for MuJoCo/TPE where the upstream implementation
supports the workload. Report Rust-backend results separately.

Run CPU-only, offscreen GPU, and fully publishing configurations separately.
Use offline assets for steady-state tests and separate cold/warm cache startup
measurements. Disable real-time throttling for throughput, but keep paced runs
for latency and deadline measurements. Always report accuracy with performance.

Bevy versus Ogre may require different pipelines; compare equal sensor accuracy,
scene content, and output resolution rather than identical graphics settings.
Do not reduce fidelity, miss sensor deadlines, or drop messages unnoticed to
claim speedup. Log lockstep and throughput-mode results separately.

## Workload suite

| ID | Workload | Measures |
| --- | --- | --- |
| B01 | Empty world and no-op system/entity scaling | Scheduler/step floor, allocations, per-entity bookkeeping |
| B02 | Many static/sleeping bodies, then active bodies | Change tracking, spatial queries, render extraction, scaling |
| B03 | Pendulum, articulated manipulator, mobile robot fleet | Joint fidelity, control latency, DART/MuJoCo/selected Rust costs |
| B04 | Contact stacks and dense collision scene | Solver/contact costs, synchronization, accuracy at fixed budgets |
| B05 | RGB/depth/RGBD/segmentation camera suite | GPU time, readback latency, CPU copies, memory and sensor deadlines |
| B06 | CPU/GPU lidar plus cameras | Shared scene work, ray throughput, contention and publish latency |
| B07 | Spawn/remove/reset and nested includes | Startup, topology edits, resource/cache behavior, memory reclamation |
| B08 | High-rate transport and recording/replay | End-to-end latency, serialization cost, queue depth, loss and replay fidelity |
| B09 | Multiple worlds and batch exact-step runs | Isolation, per-world memory, thread oversubscription, aggregate throughput |
| B10 | Large-coordinate terrain/environment world | Precision, geospatial loading, memory, sensor accuracy and floating origin |

Select fixture worlds from upstream before inventing new ones: double_pendulum,
diff-drive/mecanum/swerve, quadcopter, contact, camera_distortion, force_torque,
fluid_added_mass, and related worlds in the pinned tree. Parameterized synthetic
scaling cases supplement them. Use several entity counts and sensor resolutions;
one favorable demo is not representative.

## Measurements and method

- Measure simulation steps/second, real-time factor, p50/p95/p99 step latency,
  per-stage wall and CPU time, solver time, sensor acquisition-to-publish latency,
  missed/dropped samples, and transport latency/throughput.
- Track peak/resident memory, steady-state allocations and bytes, GPU memory,
  CPU↔GPU transferred bytes, asset load time, startup latency, and package size.
- Record hardware/driver, power mode, core affinity, thread counts, compiler,
  optimization flags, engine settings, warmup, duration, and seeds.
- Use repeated independent runs and randomized/interleaved A/B order. Start with
  at least 10 repeats after warmup; extend if uncertainty is too large. Store raw
  samples, medians, spread, and bootstrap confidence intervals for ratios.
- Benchmark release builds without coverage instrumentation. Keep tracing
  controlled and quantify its overhead. Use native profilers/perf and GPU timing
  to attribute bottlenecks; use Criterion or equivalent for meaningful microbenchmarks.

Public CI runners can provide smoke checks, not tight latency claims. Use stable
dedicated hardware for regression gates. An inconclusive noisy comparison needs
another measurement or investigation, not a pass/fail chosen from a single run.

## Proposed numeric budgets

Ratify these after P0 measures baseline variance and identifies the representative
suite. They are targets, not promises and not measured results.

| Target | Proposed gate |
| --- | --- |
| Required correctness | Every performance workload also passes its accuracy/fidelity checks |
| Primary balanced suite | At least 20% improvement in geometric-mean equal-fidelity throughput over C++ on the fixed representative suite |
| Per-workload regressions | No statistically supported >5% regression in throughput or p99 step/sensor latency without an explicit reviewed tradeoff |
| Simulator overhead | Aim for at least 30% lower non-solver CPU cost on entity/system-heavy cases |
| Memory | No >5% peak-RSS regression for comparable workloads; target 15% improvement where duplicated state is removed |
| Bounded operation | No sustained memory growth or unbounded queues in a 2-hour mixed spawn/reset/sensor soak |
| Sensor integrity | Meet configured lockstep sample counts and accuracy; throughput losses are measured and policy-compliant |

Use paired ratios for each workload and a defined weighting; do not choose weights
after seeing results. Confidence intervals and effect size must support a claimed
improvement. Amdahl's law limits total gains when an unchanged native solver
dominates runtime; report that honestly while still pursuing measurable whole-
system improvement. If the target is missed, optimize or revisit the target with
the owner and evidence rather than quietly lowering it.

## Optimization sequence

1. Establish correct schedules and numerical contracts; profile the real workload.
2. Remove repeated full-world scans, entity string lookups, unnecessary component
   churn, and redundant transform/serialization work. Use Bevy queries/change data.
3. Batch native-engine transfers and transport samples. Reuse buffers and compact
   handle maps. Avoid per-entity virtual dispatch or serialization in hot loops.
4. Share asset decoding and render extraction; use asynchronous staging/readback
   and avoid GPU synchronization per sensor. Keep lockstep semantics explicit.
5. Parallelize independent systems/observations and worlds where measured useful.
   Control total worker counts across Bevy, backend, IO, and asset work.
6. Consider specialized memory layouts, SIMD, or unsafe optimization only after
   profiles justify them and semantic/coverage/safety tests exist.

Every optimization PR includes workload, before/after data, settings, accuracy,
and a short attribution. Do not advertise the speed of Rust or Bevy in the abstract
as evidence of a faster simulator.
