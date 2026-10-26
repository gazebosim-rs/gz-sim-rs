# Full-server step-to-observation measurement

The subsequent subscriber-driven server defaults are recorded in
[SUBSCRIBER_DEMAND.md](SUBSCRIBER_DEMAND.md). The historical harness explicitly
selects Always interest and zero pose interval to retain its unobserved-pose
workload. Historical reports and numerical results remain unchanged.


The full-server variant invokes `Server::tick`, including clock and pose
publication, IMU conversion/sequence, control polling and removal polling. It
receives and checks each IMU before admitting another step. The native control
runs actual Server + Physics + Imu; its process-local callback checks each reading
during Server.Run(true,1,false), followed by complete-receipt wait before another
step is admitted. Timing includes each actual public server call, complete
delivery and validation in both; no native loop internals are bypassed.
This measures step-to-observation latency for the selected fixture, not maximum
batch throughput or network delivery.

Both use unchanged headless_imu.sdf bytes: static box ground, one spherical body,
TPE, 1 m/s initial velocity, a 1000 Hz noise-free IMU, unpaced 1 ms steps and one
received warmup outside timing. The next 1000 steps must produce exactly 1000
messages. All named decoded fields, covariance entries, frame/sequence values,
timestamps, body positions and total logical protobuf sizes are checked. Warmup
acceleration must also match native zero after the separately reviewed history
fix. Missing, duplicate or different observations fail; no loss is tolerated in
an accepted measurement. Default Rust capacity and high-watermarks remain 1000;
native high-watermarks are explicitly 1000. Rust server publication counters and
all local transport error counters must show no losses or failures.

The initial batch attempt failed a sequence/timestamp check under queue pressure.
The completed per-step protocol prevents an unbounded producer burst while
retaining default bounded queues. The failed experiment is retained locally and
has no accepted timing result. There is no capacity increase or relaxed assertion.

Rust retains extra unobserved pose construction/encoding and clock enqueue work.
Native builtin clock skips unconnected subscribers and this native configuration
has no SceneBroadcaster. Native local delivery can bypass serialization; Rust
uses encoded worker queues. The requested IMU workload and complete observations
match, but these internal paths differ. This report does not qualify requested
pose streams, network throughput, contacts, joint workloads, dynamic backends,
rendering or balanced robotics. A simple microbenchmark is not whole-Gazebo
performance or full P2 acceptance.

The candidate adopts public, independently reviewed and hosted-green Transport
48a58eed. The canonical Messages identity remains 2cea810; Fuel 3747086 and SDF 4ed690b
are unchanged. Only the Transport revision changes in Cargo.toml/Cargo.lock.
Producer changes include command-woken polling and bounded inspection snapshots;
the consumer uses existing publishing/service APIs. No simulator production API,
wire schema, coverage gate, upstream fixture or tolerance changes.

## Measured scope

On shared AMD 5950X CPU 31, one warmup process pair precedes seven alternating
native/Rust pairs. The matched per-step candidate median is 28.403 µs in Rust
(range 27.587..29.815) and 24.177 µs native (23.624..25.252). Rust is about 17.5%
slower on this selected observation path. These are measured values, not a
whole-engine speedup claim.

The same Rust harness and production source with Transport 7ce measured 1903.899 µs
(1700.221..2021.534), versus 28.403 µs after adopting 48a, a 67.03 median ratio.
These are separate batches on a shared host, not interleaved producer versions;
initial exploratory 7ce timings were especially variable. All timed Rust source,
settings and checks match; Cargo changes only Transport. This is an observed
within-Rust improvement associated with the producer update, not proof that
command wakeup alone accounts for it. The old control's native side uses a batch
Server.Run and is contextual only; use full-server-48a.json for the matched
per-step native protocol. Both raw reports retain their distinct source hashes
and admission protocols. No historical reports were rewritten.

Independent review approved the scoped source/boundary and replayed both pins
with identical private Rust harness/production bytes. It measured native 25.098 µs
and Rust 27.962 µs for 48a, versus Rust 1952.232 µs for 7ce; separate-batch variability
is retained. A mutant omitting IMU publication fails the affected per-step receipt.
The strict staged gate passes 202 Rust tests, fifteen tooling checks and all nine
actual C++ scenarios. Production coverage remains 3182/3228 lines, 4941/5079 regions,
all crates strictly above 95%. The review receipt is
planning/reviews/full-server-pipeline.json. Normal hook and hosted qualification
remain required. The selected-fixture lifecycle/assertion matrix
is separate from this latency measurement.


## Current-source clean SDK observation

The current-source SDK batches in
[headless-streams-sdk-v3.json](../planning/benchmarks/headless-streams-sdk-v3.json)
use the actual Rust server and native Server/Physics/Imu/SceneBroadcaster with
requested clock, pose and IMU streams. Both consume the same two-model SDF,
TPE settings, 1 ms physics steps, 1 m/s free-body velocity and noise-free 1000 Hz
IMU. Every step waits for all decoded observations before another request.
The complete field, timestamp, position, count, queue and byte-accounting checks
remain unchanged. Opaque entity IDs are validated within each run; the pose byte
comparison removes only their encoded varint values.

Each batch contains eleven alternating native/Rust process pairs: one warmup
pair and ten measured pairs, each with 128 steps after one received setup frame.
Requests are paced by at least 20 ms on the host. Reported work latency sums the
server call, complete receipt and validation, excluding the separately recorded
pacing wait. Wall time and every observation remain in the report. This is not
unpaced throughput. Both batches ran on shared Ryzen 5950X CPU 31 with other
coordinated build and native jobs held.

| Batch | Native median work/step | Rust median work/step | Paired geometric Rust/native ratio | Bootstrap 95% interval |
| --- | ---: | ---: | ---: | ---: |
| Author | 60.452 us | 36.644 us | 0.61151 | 0.60021–0.62624 |
| Independent reviewer | 58.856 us | 36.311 us | 0.61747 | 0.60505–0.63152 |

The ratios use the ten paired observations, not a ratio of medians. Each interval
uses 10,000 paired bootstrap resamples with seed 42; the batches are separate and
are not pooled. Warmup is the only excluded pair. All extrema and failed earlier
attempts are retained. These observations do not establish a causal optimization,
a whole-simulator speedup or a balanced robotics performance budget.

Both runs use the immutable fourteen-repository SDK image
`sha256:11b7378c7cca2844a4b7b2c9f52525e983b19d6651d33c83375d220672462d1d`.
The reviewed 299-blob source export, fresh optimized Rust ELF, native compiler,
fixture and loaded native libraries are bound by hashes before and after runs.
SDK installed plugins and qualified build libraries are checked against the
explicit SDK inventories. The default legacy-reference profile is unchanged.
Native local transport may bypass serialization; Rust uses encoded queues.
Requested delivered work matches while those internal paths differ.

Independent review replayed the functional and measurement capsules and checked
all 44 measured/warmup process runs, source and library identities, observations
and statistics. The public projection replaces only repeated fixture bytes with
their verified SHA-256. Historical reports above remain unchanged and describe
their original sources and protocols. Full P0/P2 acceptance, broader fixtures,
DART/MuJoCo, articulated robots, cameras/lidar and image distribution remain open.
Strict source, protected publication and exact hosted checks are recorded
separately in the scoped review receipt.


## Runtime IMU rate candidate

[The current rate-control source measurements](../planning/benchmarks/headless-streams-imu-rates-v1.json)
retain the same two-model TPE fixture and complete requested clock, pose and IMU
checks. Each separate batch contains eleven alternating process pairs: one warmup
pair and ten measured pairs, with 128 steps after each process's setup step.
Host requests remain paced by at least 20 ms; reported work latency excludes that
pacing. All measured pairs and extremes are retained.

Author medians are 61.646 us native and 39.741 us Rust; independent medians are
61.163 us and 39.275 us. Paired Rust/native geometric ratios are 0.63979 and
0.64388, with separate 95% bootstrap intervals [0.61952, 0.65813] and
[0.63129, 0.65692]. These are selected shared-host observation measurements,
not a rate-change benchmark or a balanced robotics result. They do not establish
a causal change from earlier batches or unpaced throughput.

Independent review binds all 44 process runs, the exact source, compiler,
optimized executable, native libraries and decoded work. The timed tree is
`b08c406f`, reconstructed from base `22a536c6` and the 26 paths listed in the
report; subsequent evidence metadata is outside that tree. Historical datasets
remain separate. Strict source, final publication and hosted checks remain
required before accepting the rate-control increment.
