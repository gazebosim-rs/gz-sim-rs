# Remote loaded-scene reset

The control service accepts all/time_only reset for worlds created by load_world.
Both restore the initial scene; model_only is ignored and acknowledged, matching
the pinned native runner. Seed changes and run_to_sim_time remain unsupported.
An arbitrary Simulation without the loader blueprint rejects a rewind request.

Accepted controls are processed after the current tick's publication. Every valid
control overwrites the pending rewind flag, so a later ordinary control cancels
an earlier reset. Rejected controls leave it unchanged. The next tick restores the
scene and calls Reset handlers without running normal simulation phases. Clock
and initial poses publish at time zero; active real time restarts. An unpaused
reset consumes one pending iteration: reset plus multi_step=3 yields two physical
steps and a final time of 2ms for a 1ms step. Future time validation starts at zero.

Initial IMU endpoint leases are prepared before domain mutation. Existing shared
publishers are reused; removed endpoints are recreated through the transport's
withdraw-before-advertise contract. Restored runtime sensor handles receive fresh
sequence and sample state. Reset publishes no fabricated IMU measurement. The
first sampled step has sequence zero. Already queued transport frames can arrive
before reset/new-epoch packets; reset cannot retract them.

A committed reset discards the admitted old removal batch. Ordinary callers have
already received acceptance. Blocking callers receive false if their response can
be queued; response failures are counted separately. removals_cancelled counts
both forms. This is an explicit new Rust cancellation boundary: native UserCommands
reconstruction discards queued commands, but its blocking admission/destruction
behavior is not reproduced or claimed. Unpolled service requests remain available
and can be admitted after reset; native endpoint reconstruction may differ.

Scene preflight failure keeps its clock, stream instances and admitted removal
batch intact; the queued rewind remains retryable. Endpoint preparation may
transiently advertise and release leases before a failed domain preflight. Custom
Reset handlers must preserve the prepared IMU endpoint configuration. A handler
that introduces an unprepared/invalid endpoint causes an explicit committed error;
the server retires stale streams and rejects further ticks. No rollback or silent
continuation is claimed after such a handler violates the stream contract.

The sixth real C++ scenario removes the original model, observes its disappearance,
resets and verifies restored original pose IDs, establishes the recreated IMU
connection using actual sampled steps, then resets the live lease and verifies a
1ms sample with sequence zero. Existing five scenarios retain their assertions.
Rust cases cover control precedence, both pause states, deferred cancellation,
failed preflight, overflow, shared leases, runtime additions and handler failures.
This bounded integration does not complete IO-01, CORE-02 or P2. Runtime sensor
registration, scheduler/noise, TPE, resources, the simple joint and velocity/force
inputs, and equal-work performance remain open.


A reset handler reporting a system fault invalidates the run after reconstruction.
The admitted old removal batch is cancelled, stale IMU streams retire, and no new
server frame is published. Later ticks and controls reject the faulted simulation.
This differs from a transactional preflight rejection; see STEP_FAULTS.md.
