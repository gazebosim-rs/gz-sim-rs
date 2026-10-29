<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# C++ peers with native physics backends

The selected headless CLI and server transport tests now exercise DART and
MuJoCo alongside TPE and Rapier. Every previous scenario, C++ client, SDF fixture,
assertion, queue bound and timeout is retained. This extends interoperability
qualification; it does not add physics features or retire an upstream test.

The server test runs 39 C++ peer scenarios:

- Six ordinary/IMU/custom-topic/removal/shared-IMU/reset scenarios on each of
  Rapier, DART and MuJoCo, plus the existing TPE reset scenario.
- Sixteen IMU sampling combinations: four engines, noise enabled/disabled and
  orientation enabled/disabled. Native dynamic engines use the existing C++
  gravity-dynamics assertions; TPE keeps its separate expectations.
- Four root-clock admission/publication scenarios, one per engine.

A separate C++ rate-service test also runs its functional phase/reset cadence
case on all four engines, retaining the three TPE delayed/exit/panic lifecycle
controls. Every timestamp, sequence, covariance and 1e-9 physical bound is kept.

The actual CLI executable also runs one C++ root-clock/remote-stepping/shutdown
scenario on each engine. Its finite advancing-step budget is checked through
pause/reset on all four engines with the Rust transport peer. The blocked-output
second-signal regression remains TPE-only; it does not qualify native-engine
large-scene capacity. The broader activation-service and physics conformance
matrices keep their separately documented scope.

The CLI retains byte-identical root/world Clock observations at 0/1/2 ms and
successful signal exit. Server checks retain pose/IMU field, frame, sequence,
noise, sampling, removal, reset and bounded-command assertions. No numerical
bounds or delivery guarantees are relaxed for the native engines.

## Discovery fixture ownership

The first staged run exposed a discovery-port race: a temporary UDP reservation
was dropped before native engine loading and transport startup, so another socket
could claim it. The gate failed with address-in-use; no result was accepted.

The fixtures now bind wildcard UDP ports exclusively, enable the same reusable
binding mode as transport, and hold those guards through each scenario. On Linux,
multicast-all delivery is disabled on the guards so they receive no discovery
traffic. A deterministic test verifies that bare loopback and wildcard binds stay
blocked while a real server can bind, tick and shut down. The test makes no
post-release availability assertion: another process may then legitimately take
the port. The pinned socket2 dependency is test-only.
No startup retries, longer timeouts or relaxed protocol assertions are introduced.

## Reproduction and native library scope

Run the two suites serially: their reference compiler writes the same peer
artifact directory. The required transport reference has its existing pinned
provenance metadata beside its install prefix.

```sh
export GZ_REFERENCE_PREFIX=/path/to/transport-reference/install
export MUJOCO_PREFIX=/path/to/from-source-mujoco
unset MUJOCO_LIBRARY
cargo test --locked -p gz-sim-cli --test native
cargo test --locked -p gz-sim-server --test native
cargo test --locked -p gz-sim-server --test native_rates
```

Author and independent runs pass all three CLI test functions, the 39-scenario
server clock/IMU matrix and the four-engine rate-service/reset case with its
three retained lifecycle controls. Source/assertion review also passed. The
mandatory staged/hosted gates remain required for the published change. The existing C++ peer compiler and reference checks remain
unchanged. DART pins the verified OSRF build through DT_RPATH and validates
loaded libraries at construction. The local MuJoCo runs explicitly clear its
higher-priority library override and select the vendored from-source prefix.
MuJoCo's public loader still checks the version rather than enforcing a new
file-identity policy.

These clients use actual C++ Gazebo transport and protobuf to control the Rust
simulator. They are not a new native Gazebo simulator comparison or a performance
benchmark. The existing same-engine physics fixtures and qualified historical
TPE measurements retain their original scopes. General articulations, limits,
contact properties and full retained-test/release qualification remain open.
