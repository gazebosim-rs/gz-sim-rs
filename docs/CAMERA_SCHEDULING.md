<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Single-camera application cadence

`ScheduledCameraOutput` adds application-owned sampling admission around the
existing explicit 64×64 RGB output. `prepare` finds loaded Bevy `CameraSensor`
metadata. No camera returns `None` before worker, key or session validation;
multiple cameras reject before declaring anything. The supplied Session and
writer scope remain shared caller-owned infrastructure. This is not a claim
that an application allocating those inputs has no allocations.

The caller invokes preparation, `request_due`, `pump`, retirement and destruction
outside the simulation's critical step. These methods can inspect Zenoh routing
state, start a supervisor or wait for process retirement. No callback, worker,
resource or schedule is added to `Simulation`; the existing headless dependency
closure is unchanged. Automatic Server integration is not part of this slice.

After an advancing committed frame, call `request_due`. It uses the loaded SDF
update rate and the existing native-bound `SensorSchedule` from Scene. The
committed `UpdateInfo`, rather than the configured pause flag, determines whether
the frame advanced. This permits explicit stepping while configured paused.
Initial time-zero metadata inspection and repeated paused frames do not acquire
samples. Repeated calls at one timestamp do not duplicate camera-info or images,
including when the update rate is zero. Manual `CameraOutput::request` is
unchanged and still permits explicit time-zero acquisitions.

There is one existing acquisition slot. Pump it until completion, including
after physics has advanced; its original epoch/time remains on the frame. A due
request while that slot is occupied returns Busy. Missed periods coalesce to
one sample of the current committed state, preserving the native clock phase;
there is no unbounded catch-up queue or frame history.

No demand skips sampling admission. Its observed negative snapshot returns
directly: a later subscriber cannot re-enter request and bypass clock accounting.
Info-only demand publishes at the same
cadence without starting a renderer. Once eligible demand has been observed, an
attempt consumes its clock slot even if it fails or subscribers disappear.
The original typed error is returned. This follows the native Sensor clock's
advancement after virtual Update, even when that Update returns false, and
prevents duplicate info after a partial attempt. Busy and retirement checks
occur before this admission. This is an explicit application overload contract,
not a claim about asynchronous native Manager locking or acknowledgements.
If an existing worker loses image demand, the observed snapshot instead requires
retirement before info-only admission. It emits no info and spends no sampling
slot; after retirement, that same committed timestamp remains eligible.

`set_active(false)` is a local admission gate, initially true. It discards the
pending acquisition and marks any live worker for explicit retirement. Until
`retire` returns, `request_due` returns RetireRequired even while inactive;
`pump` also exposes retirement. Re-enabling retains the previous clock phase.
It does not implement SDF always_on, triggering, forced updates or native Manager.

Every request and pump validates epoch and exact entity/metadata binding before
pause, inactive or due gates. Reset, removal, replacement or a fault invalidates
the owner and discards buffered frames. Retire and drop that owner, then prepare
again from the reconstructed Scene outside stepping. The new owner has a fresh
clock and sequence; no retired-camera list or reset template is retained. A
removed last camera produces `None` on preparation.

The native basis is gz-sensors `8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`,
`Sensor.cc` Update and `SensorUpdate.NextDataUpdateTime`, plus CameraSensor's
info-before-image-demand path. Existing Scene schedule tests retain that clock's
exact integer-nanosecond controls. `CameraSensorTest.ImagesWithBuiltinSDF`,
`SensorsFixture.UpdateRate` and reset sensor integration cases remain partial:
this path does not implement their complete dimensions, sensor families,
native Manager or rendering assertions. No new original-case credit follows.

The software-worker tests exercise bounded admission, acquisition stamps,
sequences, RGB output and cancellation after reset/removal. They do not qualify
physical GPUs, full sensor scheduling, no-wait shutdown or ACT-01 completion.
The existing renderer exit-timeout failure remains unresolved. No retry, deadline
or fixture assertion is relaxed by this change.

Author focused checks pass 34 camera-output library tests (nine new), five
unchanged shared-clock tests and three new software-worker tests. Eight fresh
native Sensor clock observations also confirm false-return advancement and the
inactive gate; that supplemental probe does not execute native camera rendering.
All-target Clippy passes for the three affected packages. A source-bound normal/
build dependency inspection still excludes renderer/GPU packages from the server.
The first sandboxed CPU run failed at loopback setup; its failure remains separate
from the subsequent authorized network execution. No worker failure was retried
or converted into a pass.

Two deterministic real-subscription controls capture no demand, then add info
or image interest before the production admission helper consumes that snapshot.
The previous request re-entry fails both controls; direct negative returns pass
without an unmetered publication or worker start. A subsequent public request
at the same committed timestamp consumes exactly one slot.

This is a private implementation. Independent source and retained-evidence
review approved the bounded slice and the demand-race repair. Strict production/
changed coverage, ordinary staged checks and exact hosted qualification remain
pending; the reviewer did not rerun the software worker.
