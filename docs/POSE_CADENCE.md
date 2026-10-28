# Pose publication cadence

Server pose/info defaults to a minimum wall-time interval of 16,666,667 ns,
matching pinned SceneBroadcaster's fixed 60 Hz Transport publisher. The first
valid packet is eligible. A reset retains the publisher's wall timer, so a reset
frame within that interval is intentionally suppressed. Physics, clocks and IMU
sampling keep their own cadence; tick never sleeps or waits for eligibility.

The interval is a ceiling: native Node.cc compares integer elapsed nanoseconds
against double 1e9/60. It records the actual publication attempt, without catching
up missed intervals. Rust does the same immediately before enqueueing a valid
packet. A rejected enqueue consumes the interval; malformed Rust scene data does
not. Counters distinguish queued packets, intentional throttling, queue failures
and invalid frames. `poses_queued` counts admitted attempts, not receiver delivery.

`Server::set_pose_publication_period` allows an explicit interval; zero selects
publication on every tick. Changing the policy makes the next valid frame
eligible. This Rust policy API is separate from native SceneBroadcaster's fixed
pose/info setting. A simulation reset does not change the policy or timer.

## Evidence and test disposition

`planning/reference/pose-rate.json` records five fresh native processes using the
unchanged 60 Hz Transport option. Each publishes an eligible first value, offers
four suppressed values through a copied publisher, and then sends the next
eligible value. All six Publish calls return true, but only values 0 and 99 arrive.
The runner pins ZeroMQ and disables statistics, relay and authentication.
The burst must finish before the interval; a scheduling overrun fails the probe.
The next eligibility is observed by the actual native readiness method, not a
fixed startup sleep. Source, compiler, binary and loaded libraries are hashed.

Pinned SceneBroadcaster setup and Reset code establish the fixed setting and
publisher reuse. This focused runtime control tests Transport, not a full native
Server reset. The earlier complete-stream report independently exercised actual
native pose publication and rejected an unpaced missing-pose attempt. Native
installed dependency/build provenance remains qualified under P0.

Rust boundary checks use supplied Instants for the exact nanosecond boundary,
first publication, past timestamps and zero/maximum intervals. An integrated test
checks clock and IMU delivery on every physics step while poses are suppressed,
then checks reset timer retention and an explicit policy change. A long selected
interval makes that integration test independent of host scheduling; it does not
claim the native default is one day. Rejected enqueue and invalid-then-repaired
scene controls check when an interval is consumed.

Existing private server helpers and the historical full-server IMU-only harness
explicitly select zero. They retain every original assertion, tolerance, payload,
queue limit and per-tick workload; their previous every-tick assumption was a
Rust-only default mismatch. The complete three-stream harness retains the native
default and its declared 20 ms request spacing. Historical performance reports
remain unchanged and do not measure this new source revision. Independent review approved these dispositions, reproduced all five native
controls under hostile caller settings, passed all 38 private server tests, and
confirmed guard-removal and reset-timer-clearing mutants fail. The exact staged
gate passed 208 Rust tests, sixteen tooling checks and nine actual C++ scenarios.
Workspace coverage is 3229/3264 lines and 5007/5122 regions; every production
crate exceeds 95% on both, with 43/43 changed server lines covered.
The receipt is `planning/reviews/pose-rate.json`; normal commit and hosted checks
are separate publication requirements.

## Remaining IO-01 scope

[SUBSCRIBER_DEMAND.md](SUBSCRIBER_DEMAND.md) records the subsequent atomic
interest policy: no pose construction or timer consumption without known
subscribers, with an explicit local Always override. Root clock election,
dynamic_pose/info, other controls/services, ordering/backpressure and the full
IO-01 packet remain open.

Reproduce with `python3 scripts/probe-pose-rate.py --help`, using the pinned native
sources, existing Release simulator build and dependency prefix. Native networking
must be available; sandbox socket denial is a setup failure, not passing evidence.
