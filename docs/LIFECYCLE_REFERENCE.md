# Shared-SDF reset and removal controls

The earlier local native reset probes are now reproducible from tracked source.
They use the unchanged headless_imu.sdf with explicit native Physics and
UserCommands systems, actual Server/TestFixture APIs and both DART/TPE plugins.
No Imu or rendering system is required for these entity/control observations;
the separate acquisition and real C++ peer checks remain mandatory.

Each engine runs five controls:

| Control | Observations |
| --- | --- |
| Ordinary removal during queued reset | One reset callback and original model identity present after queued removal/reset |
| Later ordinary control | Cancels the earlier reset; model remains, no reset callback |
| model_only | Acknowledged without restoring a removed model; no reset callback |
| time_only | One reset callback reconstructs the removed model with its original identity |
| Reset plus three requested iterations | One reset consumes one iteration; two physical steps end at 2 ms |

The native program requires each control/removal request to succeed and checks
its outcomes before exiting. It uses the same public asynchronous RPC/future
lifetime pattern as the reviewed C++ client fix. Paused RunOnce calls process
queued controls without discovery sleeps; no arbitrary wait is treated as proof.
The runner rejects nonzero exits, stderr, wrong backend mappings, missing cases,
wrong identities/counts/times and unexpected model poses. Pinned source, fixture,
compiler command, build and loaded-library hashes accompany all ten rows in
planning/reference/lifecycle-controls.json. Installed dependency provenance remains
qualified; this is not a complete immutable reference image or performance test.

The Rust test uses actual loaded scenes and Server ticks, admitted removal
requests and a Reset phase observer. It compares all ten controls, actual reset
callback counts, simulation time, paused completion and preservation of each
implementation's original model identity. Identity values need not match across
processes. The API mapping admits Rust controls directly and drains the removal
admission queue before advancing; native queued controls require paused RunOnce
boundaries. The native ordinary row does not expose a cancellation counter or intermediate
removal application. Rust additionally requires removals_cancelled=1 as its own
admitted-request boundary; that count is not a native parity assertion.
Only completed selected observations are compared, not identical
internal callback scheduling or arbitrary queued-command destruction behavior.

Every model translation and quaternion component is compared. Initial/restored
poses and TPE motion use 1e-12. DART/Rapier free fall is a bounded cross-engine
physical check, not a DART adapter: two 1 ms steps permit half an Euler tick of
position bias per step, 0.5*g*dt²*N = 9.81e-6 m, only on vertical translation.
Other components retain 1e-12. No existing tolerance or assertion was changed.

No original upstream case is retired. Full world services, unsupported seed/run-to
controls, arbitrary topology and complete retained-test parity remain wider work.
Independent review reproduced all ten native rows and every substantive source,
build, fixture, binary, compiler, helper and loaded-library hash. The exclusive
Rust comparison passes. Three private mutants fail the Reset callback count,
reset multistep time and pose checks. The source-equivalent strict gate passes
203 Rust tests, fifteen tooling checks and all nine actual C++ scenarios:
3193/3228 production lines and 4964/5079 regions, every crate above 95%. There are
no changed production lines. The receipt is planning/reviews/lifecycle-controls.json.
Normal-hook publication and hosted qualification remain required. This supplements
the selected-fixture matrix; it does not accept the full P2 phase.

Reproduce with:

```sh
python3 scripts/probe-lifecycle-controls.py --sim-source /path/to/gz-sim \
  --physics-source /path/to/gz-physics --sim-build /path/to/native-build \
  --dependency-prefix /path/to/native-prefix --work-dir /tmp/lifecycle \
  --output artifacts/lifecycle-reference.json
cargo test --locked -p gz-sim-server --lib \
  tests::reset_tests::shared_sdf_native_control_precedence_removal_and_reset_observations_match \
  -- --exact
```
