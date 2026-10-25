# IMU history after a velocity replacement

A queued world-velocity replacement changes configuration before integration.
It must not appear as physical acceleration in an attached IMU. The moving
benchmark exposed an initial Rust x reading of 1000 m/s² after replacing 0 with
1 m/s before a 1 ms step; actual native DART and TPE report zero.

`probe-input-imu.py` rebuilds a native Server + Physics + Imu control against the
pinned sources using unchanged `headless_imu.sdf` bytes. It retains all 100 readings
at 1..100 ms for DART velocity, DART velocity plus a second-step COM force, and
TPE velocity. Every timestamp, orientation, angular velocity and specific-force
component is compared. A 2 N force on the 2 kg body produces 1 m/s² in DART,
then zero on the next step. Rapier is checked against that bounded physical
contract, not claimed to be a DART adapter. TPE has no wrench capability.
Source, binary, fixture and loaded-library hashes are in
`planning/reference/input-imu.json`; installed native dependency provenance
remains qualified as in HEADLESS_REFERENCE.md.

After successful command execution, the scene rebases each attached IMU's prior
world linear velocity before integration. Physical force deltas remain visible,
including a force queued for the same step. Link children provide the Bevy lookup;
there is no all-sensor scan for each command. Paused ticks retain commands without
changing samples; invalid admissions preserve pending inputs. Both reset paths,
model removal/reconstruction, multiple IMUs on a link and faulted execution are
covered. Rejected or failed commands do not rebase history.

The regression failed on the preceding implementation with x=1000 versus native
zero. The private pipeline benchmark's warmup assertion changes from that known
incorrect value to the native-backed zero; the 1e-9 bound, all other fields,
sequence/timing/count checks and measured interval stay unchanged. Historical
headless-single/parallel reports retain their original warmup observations and
qualified scope. They are not measurements of this fix. The executor improvement
remains a within-Rust experiment, and full-server equal-work measurement and the
complete selected-fixture matrix remain required for P2.

Independent review reproduced all three native controls and substantive source,
fixture, binary and loaded-library proof fields. A private mutant removing only
the history rebase restores the x=1000 failure. The exact author gate passes
201 Rust checks, fourteen tooling checks, all nine real C++ scenarios and every
production crate's strict coverage gates (3182/3228 lines, 4941/5079 regions;
changed scene lines12/12). The receipt is planning/reviews/input-imu.json.
Normal-hook publication and hosted qualification remain required.
