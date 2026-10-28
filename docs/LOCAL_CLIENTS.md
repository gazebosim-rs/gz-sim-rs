# Separate local Nodes and simulator streams

The simulator uses the reviewed Transport process-local routing implementation.
A live matching subscription on another Rust Node receives an IMU frame directly
and reports `intra_process=true`, matching the native Transport two-Node tests.
Local subscribers retain bounded queues. The ordinary wire send remains enabled;
Transport suppresses matching local wire echoes within its documented bounded
fingerprint window. That is not an unlimited exactly-once network guarantee.

Registration can arrive later. The simulator reads only the atomic interest hint
during a tick; it never waits for discovery or wire connection there. The new
integration test waits for registration outside physics, advances exactly one
1 ms step and receives sequence0 at1ms without publication warmup or retry. It
checks every motion/quaternion/covariance value, stamp/frame metadata, direct
origin and a quiet duplicate window on Rapier and TPE. Loaded reset repeats the
same first frame. The transport routing-error counter stays zero. Existing
subscriber-demand tests, bounded error paths and all fifteen C++ scenarios remain.
The producer also has a stronger500ms connected-wire duplicate control; the root
consumer's50ms quiet window alone does not establish that property.

The old2a development control receives the first frame over the wire and fails
the direct-origin assertion. A fresh private current-test replay on2a preserves
that failure; only the new producer-only `routing_errors` assertion is omitted
from that negative control because the old Statistics type lacks the field.
No lost-packet cause or broad transport equivalence is inferred. Recorded hashes
and producer evidence are in planning/reference/local-clients.json.

The producer retains per-Node discovery UUIDs and service workers. Its documented
identical-payload/address-reuse and bounded fingerprint limits remain applicable;
see the pinned producer's NET-02-process-local-direct.md. Cross-process readiness,
full sensor lifecycle, native callback scheduling and broad IO01 remain open.
The initial adoption gate is retained as a failure: under publication load a
service response could hit the shared bounded command queue and strand its
request. The producer now admits responses through a separate bounded lane;
QueueFull and shutdown remain observable. This is not an unconditional reply
latency guarantee. Consumer checks must exercise that repair with all existing
service assertions unchanged.

Publication optimizations retain unconditional pre-wire echo history, share each
publisher lease's history, and move the owned packet into the final local target.
No packet count, field, precision, pacing or overload assertion is reduced. The
predeclared final shared-host comparison reports a 1.03598 candidate/earlier-Rust
paired ratio, with 95% interval [0.99243, 1.07658]. Its point overhead is inside the
proposed 5% target; an upper-bound guarantee is not established. Earlier measured
regressions and setup failures remain recorded. This is not final performance
budget acceptance or a C++ speed comparison.

The separate historical public05/native comparison is published in
planning/benchmarks/current-native-streams.json with its independent source and
projection receipts. It retains all eleven pairs, including warmup and outliers;
its 0.70018 paired Rust/native ratio applies only to that exact two-model TPE/IMU
stream fixture and source revision. It is not a measurement of the new producer
pin or balanced robotics. Exact producer hosted40248292544 SUCCESS; independent
consumer review and the exact239 Rust/24 tooling/15 C++ staged gate pass. Normal
protected consumer commit and exact hosted verification remain separate gates.
