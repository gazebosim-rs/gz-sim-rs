# Reported system faults

StepFault stores the first owned Send+Sync error in std::sync::OnceLock. Systems
report through shared Res<StepFault>; healthy systems do not acquire a global
mutable resource or serialize otherwise independent ECS work. Simulation::fault
borrows the retained diagnostic, including concrete error downcasting.

Every system registered through Simulation receives a separate healthy condition.
A reported fault prevents later guarded systems from starting. Already running
systems, deferred commands and earlier external effects may finish; they are not
rolled back or recalled. A normal failed tick retains the last committed elapsed
time, iteration and UpdateInfo. The simulation becomes terminal: tick, exact-step
and reset attempts fail with StepError::Faulted. Even replacing the raw fault
resource after that failure cannot clear the simulation's private terminal flag.
Installed resources must otherwise retain their documented ownership invariants.

Reset handlers can fail after reconstruction and reset time/epoch have changed.
Loaded-scene reset then removes its prepared-physics marker, retires pending
runtime additions, skips its immutable observer, and returns CommittedReset.
Preflight errors remain transactional. The server discards the admitted old
removal batch only after this committed failure, attempts blocking false replies,
retires stale sensor streams and does not publish a new clock/pose/IMU frame.
Subsequent ticks reject before endpoint preparation or command polling.

This is an explicit Rust error channel for fallible adapters, not exception or
process-fault recovery. Panicking callbacks still invalidate a run; native
unrecoverable faults cannot be caught by this mechanism. A new run requires a new
simulation. The TPE motion kernel remains separate until its Bevy adapter uses
this boundary; no TPE selection or D23 policy is implemented by this increment.

Tests exercise every normal phase, first-diagnostic retention, typed access,
pre-reported errors, zero-step rejection, terminal retries, last committed timing,
reset callback failure, pending-node cleanup, stream retirement, admitted-request
cancellation and publication suppression. All existing real backend assertions
and six C++ interoperability scenarios remain required.
