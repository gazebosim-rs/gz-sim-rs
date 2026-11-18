<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Retained joint-effort reference

`joint_effort.json` is the unchanged native reference previously stored in
`gz-physics-dart/tests/fixtures`. Existing Rapier Scene and Server controls still
compare against its observations. D33 retires the DART and MuJoCo adapters and
campaigns; this data does not select or load either adapter.

The original generator is retained byte-for-byte at
`gz-physics-rapier/tests/fixtures/joint_effort.cc`. Its recorded source hash and
observations remain unchanged. This relocation does not recapture native data or
claim a new qualification result.
