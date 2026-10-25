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
