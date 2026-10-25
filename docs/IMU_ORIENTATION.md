# IMU orientation field presence

Loaded SDF `imu/enable_orientation=false` omits the orientation quaternion from
the published IMU message. Angular velocity, specific force, all three covariance
messages, timestamp, frame metadata and sequence remain present. Orientation
covariance still contains nine zeros, matching native; it is not replaced with a
negative sentinel. Enabled or absent settings preserve the existing quaternion.

This is an immutable loaded publication policy, carried by `ImuPublication`.
The stateless measurement model continues computing a relative orientation for
local consumers. The flag does not disable acquisition, dynamics, noise, demand
gating or schedule advancement. Loaded reset reconstructs the original flag and
replays the configured initial noise stream. Runtime orientation toggling and the
native sensor's cached Orientation property are separate, unimplemented APIs.
Custom reference frames and named localization modes remain unsupported.

The source contract is pinned `gz-sim/src/systems/imu/Imu.cc` applying the DOM
flag and `gz-sensors/src/ImuSensor.cc` constructing covariances independently of
its orientation branch. `scripts/probe-imu-orientation.py --help` reproduces four
actual TPE Server/Physics/Imu controls: enabled/disabled by plain/noisy, two exact
samples each, field presence, all covariance entries, motion channels, metadata,
stamps and sequences. Source, compiler, fixture, binary and loaded-library hashes
are recorded in `planning/reference/imu-orientation.json`. Build/dependency
provenance remains qualified; this does not accept the full P0 image.

The Rust suite preserves the exact old false-flag XML as a positive test on both
engines through reset. Its obsolete Rust-only unsupported row instead checks NED
localization with the same error and atomicity assertions. A packet equality test
checks that only the quaternion field changes. Four additional real C++ client
scenarios exercise disabled plain/noisy output on Rapier and TPE through slow
sampling and reset; all eleven existing scenarios and enabled assertions remain.
No complete upstream case, runtime-toggle behavior, full SENSOR01 or P2 acceptance
is claimed. No new performance improvement is claimed by this policy branch.
