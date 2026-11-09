<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M33 acceptance: robot IMU and lidar coexistence

M33 is accepted at qualified Main `17dd62f27234eab1482abb233389d167dafa31b4`
(tree `5ebfe64ef30a8a5411af8122a6651243352585a7`, parent `679c9990`).
The full normal hook passed in 1,960.359 seconds. Exact review CI
[42667053571 attempt1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/42667053571)
passed all 16 steps; independent local and official DATA and retained-evidence
consistency passed before normal nonforced Main promotion and exact readback.

The separate robot fixture proves simultaneous canonical IMU/lidar with actual
C++ Twist and sensor streams, plus the maintained Primitive socket lifecycle,
credit/reset/reconnect controls. No new sensor model or backend was introduced.
Headless1,565 and renderer941 passes overlap; 366 Python passes are aggregate.
Local headless coverage is 27,937/28,677 lines and 42,116/43,738 regions;
hosted headless is 27,942/28,677 and 42,120/43,738. Renderer local is
16,998/17,475 lines and 25,705/26,800 regions; hosted is 17,000/17,475 and
25,707/26,800. Strict workspace, per-crate and actual-parent gates passed.
Local and hosted raw counts 1,785/378/23/33 were separately compared once.
The three original hosted ZIPs, 127,134,847 bytes, remain durably retained.

The first normal attempt failed because the maintained peer CLI omitted the new
client choice before compilation. Its original packet remains failed and intact.
The corrected CLI dispatch/profile guards and regression source were independently
reviewed before one full retry. The local audit invocation preflight mistake also
remains preserved; no test/native replay or relabeling was used to resolve it.

The machine acceptance record is `artifacts/m33-delivery/acceptance-main-promotion.json`.
Local DATA SHA256: `e56bc580d338a9023d83cb85b8176b427623f8d5028aa0344d902699f53ec445`.
Official DATA SHA256: `6053adab5e37354b8fbdf10a3c8bbb904f1463c5280e7e251ed0f53640b61bf4`.
M32 release439342072 stays installed Latest, read back after Main promotion.
No new package, browser replay, full compatibility or performance claim follows.
M34/M35 proceed as reviewed, disjoint bounded source lanes; their own full gates
and independent evidence remain required. Automatic Main CI is separate evidence.
