<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Consumer code-generation compatibility repair

Upstream registry prost-derive 0.14.4, archive SHA256
b570b25f7617e43d59005d0990ccb79e950a423952cea19671b7a876da390adf.
Only src/field/scalar.rs differs from the package. The attached patch makes
plain float/double encode and encoded_len preserve opposite signed zero, as
native protobuf does. Numeric inequality remains, preserving existing NaN/default
behavior. Other kinds and the runtime are unchanged. This is a consumer build
dependency repair; it does not restore the retired Messages SDK or runtime.
Original Apache-2.0 license and source notices remain unchanged.

This candidate requires native fixture tests and independent review before
production adoption. Recheck the exact patch on every upstream version update.
