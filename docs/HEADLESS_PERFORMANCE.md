# Rust executor control and native context

This control measures two-model TPE stepping, one moving body, one noise-free
1000 Hz IMU and complete decoded process-local message delivery. It does not
measure network throughput, clock/pose services, joints, rendering or the final
balanced robotics workload. P2's full selected-fixture matrix remains open.

Both implementations use identical SDF bytes, a 1 ms unpaced step, the same
sphere/ground geometry, a world velocity of (1,0,0) m/s and one actual received
warmup frame. The measured interval advances 1000 further steps and receives all
1000 messages. Every component, covariance, frame/sequence field, timestamp,
logical protobuf size and final position is checked. Each application echoes its
consumed SDF bytes, so a stale Rust include cannot silently use different input.
Queues are bounded; a missing, extra, malformed or dropped frame fails the run.

The harnesses differ at the server boundary: native uses its full Server with
Physics/Imu plugins; Rust directly steps Simulation and calls the production IMU
converter and publisher. Rust does not execute Server::tick/publish_imus or its
control/removal/publication bookkeeping. Thus the native comparison is contextual,
not an equal-work server speedup or a P2 performance acceptance result.
Native may bypass wire serialization locally;
Rust currently encodes and decodes its queued data. That implementation difference
is retained. The requested delivered messages match; this is not a claim about
network delivery. The nine existing external C++ scenarios remain separate gates.

## Recorded policy experiment

On an AMD Ryzen 9 5950X, CPU 31, one warmup process pair precedes seven measured
pairs. Native/Rust order alternates. Both runs use the same Rust release binary;
only the explicitly selected Bevy step executor changes. Native application,
simulator, TPE and IMU builds use Release optimization. Other installed dependency
provenance remains qualified. This is a shared-host observation, not a release
performance budget or a machine-wide speedup claim.

| Rust step executor | Rust median | Improvement over parallel |
| --- | ---: | ---: |
| Bevy parallel | 54.244 us | 1.00x |
| Default single-threaded | 4.630 us | 11.72x |

The full-native context medians were 23.167 and 23.607 us respectively. Their
arithmetic ratios to this Rust harness are retained as context only. They are
not equal-work native/Rust speedup evidence. Independent replay reproduced
54.056/4.654 us within Rust on the same CPU; source and binary hashes match.

The paired counter supports avoiding worker handoffs in this ordered, lightweight
pipeline. The default now uses Bevy's single-threaded executor; callers can use
`simulation.set_step_executor(bevy_ecs::schedule::MultiThreadedExecutor::new())`.
Reset handlers retain their separate schedule. Backend-internal and Bevy query
parallelism are independent. Executor-switch tests preserve local system state,
deferred visibility and time, and verify fault barriers with both executors.
This is not evidence to serialize every future rendering or heavy system graph.

`planning/benchmarks/headless-parallel.json` and `headless-single.json` retain
all samples, ranges, output checksums, consumed-input proofs, CPU/compiler/build
settings, binary/source hashes and required loaded library identities. Complete
decoded message semantics and total logical protobuf sizes are checked; the
control does not compare every serialized byte across implementations. The first
attempt rejected a missing physics plugin even though sensor messages looked
valid. The corrected `GZ_SIM_PHYSICS_ENGINE_PATH`, required library checks and
per-step physical-position checks prevent that false positive.

## Limits that remain visible

The initialization command produces a native TPE warmup x acceleration of zero;
The measured Rust revision's finite-difference IMU reported 1000 m/s^2 for that velocity change.
Both values are retained in the reports. The measured interval starts after that
one setup frame; every measured sensor value matches. INPUT_IMU.md records the
separate native-backed history correction and its force countercontrol. These
historical reports remain unchanged and are not measurements of the correction.
The native installed dependency qualification and complete reference environment
also remain explicit. A scoped performance result does not complete P2 or the
rewrite. Scoped independent review reproduced both executor modes and approved
the within-Rust conclusion and these qualifications. The strict staged gate
passes 198 Rust tests, fourteen tooling checks and all nine C++ scenarios:
3169/3215 lines and 4922/5057 regions, every crate above 95%; core changed lines
are 7/7. The receipt is planning/reviews/headless-executor.json. Normal-hook
publication 84aef01 and exact hosted CI 40208808832 succeeded.

Reproduce the control by building native simulator Physics/Imu targets, native
TPE and sensor IMU libraries in separate Release directories. Build the Rust
release integration executable with Cargo JSON output, then run:

```sh
cargo test --locked --release -p gz-sim-server --lib \
  pipeline_benchmark::tpe_imu_process_local_pipeline --no-run \
  --message-format=json > artifacts/pipeline-build.jsonl
python3 scripts/benchmark-headless-pipeline.py --sim-build /path/to/sim-release \
  --physics-build /path/to/physics-release --sensors-build /path/to/sensors-release \
  --dependency-prefix /path/to/native-dependencies \
  --reference-prefix /path/to/pinned-transport-release/install \
  --rust-build-json artifacts/pipeline-build.jsonl --cpu 31 --executor default \
  --work-dir artifacts/pipeline-run --output artifacts/pipeline-report.json
```

FULL_SERVER_PERFORMANCE.md records the subsequent actual-server per-step
comparison and its remaining internal-path qualifications. These historical
direct-pipeline reports remain unchanged.
Use `--executor parallel` for the within-Rust counter. Select an available CPU and preserve
all raw logs, including failed attempts. No native source, test tolerance,
coverage exclusion or threshold is changed by this control.
