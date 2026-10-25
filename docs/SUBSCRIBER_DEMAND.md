# Subscriber-driven pose and IMU work

Server defaults to `InterestPolicy::Subscribers`. It reads the pinned Transport
2a545f1 atomic `connection_snapshot`; it sends no worker command and waits for
no socket or reply. These are matching-registration hints, not authentication,
delivery acknowledgements or complete disconnect detection. Native remembered
remote registrations can outlive process-only BYE. Actual received frames remain
the readiness barrier in client tests.

An uninterested pose publisher skips packet construction without consuming or
resetting its wall-time publication slot. `poses_unobserved` counts those ticks;
`poses_throttled` counts interested ticks inside the interval. Failed snapshot
reads increment `interest_failures` separately. They do not advance the pose
timer. Physics can commit before existing transport control polling reports a
shutdown error; interest sampling itself never blocks physics.

## Native world-wide IMU rule

Pinned Imu::PostUpdate checks whether **any** due IMU has subscribers. If so, it
updates every IMU, with each sensor's own acquisition clock deciding whether to
sample. A fast unconnected IMU can therefore acquire and advance its sequence
when another, slower IMU triggers the world gate. Independently gating each
sensor by only its own subscribers would change native behavior.

The scene stores a sorted reusable selection of public sensor identities. The
server refreshes it from atomic registrations before each tick. Its clock check
does not mutate sampling phase. Physical velocity histories still advance every
physics tick; inactive acquisition creates no new reading, consumes no noise and
retains the preceding sample. Reactivation uses the elapsed acquisition gap for
dynamic bias, not just one physics interval. Public identities preserve the
selection across loaded reset. Runtime registration and SetActive/trigger APIs
remain separate work.

A scene used without Server keeps local sampling. `clear_imu_interest` restores
that behavior explicitly. Server's `InterestPolicy::Always` provides a local
publication/acquisition override. It does not force an early sample or bypass
pose rate limits. The policy is distinct from SDF always_on; pinned Sensor.cc
currently ignores that field and Imu::PostUpdate still applies its world gate.

## Reference and preserved tests

The same tracked `imu_interest.sdf` has a 10 Hz slow and 1000 Hz fast IMU.
Actual native TPE Server+Physics+Imu controls establish:

| Interest | First fast observation | After ten inactive steps and reconnect |
| --- | --- | --- |
| None for 50 ms | 51 ms, sequence0 | 62 ms, sequence1 |
| Slow connected for 100 ms | 101 ms, sequence2 | 112 ms, sequence3 |

The slow connected sensor produced exactly 1 ms/sequence0 and100 ms/sequence1.
Its unconnected fast peer also acquired those two times. Rust controls subscribe through server.node, exercising the same-Node
immediate bits. Native plugin/client Nodes share process-global NodeShared.
Immediate interest between separate Rust Node instances is not promised; their
registrations and remote readiness are eventual and verified by actual frames. Sources, fixture, compiler, binary and loaded libraries are hash-bound
in `planning/reference/imu-interest.json`; installed dependency/build provenance
remains qualified. Reproduce using `scripts/probe-imu-interest.py --help`.

Rust adds unknown/duplicate handle filtering, zero-rate and local mode, late
Rapier free-fall acceleration without a derivative spike, noise stream/elapsed
gap checks, pose timer/disconnect/reset and deterministic shutdown observability.
Existing packet/error/lifecycle helpers and the historical full-server pipeline
explicitly select Always, preserving every old assertion and fixed workload.
Two C++ scenarios without an IMU subscriber also select Always to retain their
Rust-only unobserved-acquisition counters; the other nine keep the native default.
All old C++ payload and Rust assertions remain. The initial eight private failures
and C++ unobserved-counter failure are retained before these explicit policy
selections. Independent review approved these dispositions after native and
private Rust replay; per-sensor gating, stopped velocity history and consumed
uninterested pose-slot mutants failed their intended assertions. The receipt
is `planning/reviews/subscriber-interest.json`.

This implements bounded subscriber-driven work, not complete IO-01/SENSOR-01/P2.
Root clock election, dynamic_pose/info, runtime sensors/activation APIs, metrics,
remaining services and complete backpressure/lifecycle policy remain open.

The exact staged gate passed 227 Rust tests, sixteen tooling checks and all eleven
C++ scenarios. Every production crate exceeds 95% line and region coverage;
workspace3505/3541 lines5353/5470 regions, with33/33 changed scene and33/33 server
executable lines covered. Normal protected hook and hosted verification remain
separate qualification requirements; performance parity is not established here.
