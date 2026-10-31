<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Consumer code-generation compatibility repair

Upstream registry prost-derive 0.14.4, archive SHA256
b570b25f7617e43d59005d0990ccb79e950a423952cea19671b7a876da390adf.
Among upstream package files, only src/field/scalar.rs differs. PATCHES.md and
signed-zero.patch are added local audit material. The attached patch makes
plain float/double encode and encoded_len preserve opposite signed zero, as
native protobuf does. Numeric inequality remains, preserving existing NaN/default
behavior. Other kinds and the runtime are unchanged. This is a consumer build
dependency repair; it does not restore the retired Messages SDK or runtime.
Original Apache-2.0 license and source notices remain unchanged.

The simulator consumer adopted this repair in
[`5870373`](https://github.com/gazebosim-rs/gz-sim-rs/commit/5870373f0d8280601b051f2cbb1bb0aba264f70f)
after native fixture tests, independent review, the ordinary full staged gate and
both strict workspace profiles. Exact hosted run 41248418524 and its downloaded
source, coverage and native/GPU evidence were independently audited. The standard
runtime and generated APIs remain unchanged. Recheck the exact patch on every
upstream version update.
