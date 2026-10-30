<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Bounded camera calibration investigation

The private calibration candidate accepts explicit finite `fx`, `fy`, `cx`,
`cy`, positive ordered clipping distances, and a bounded image size. It
rejects skew and projection translation rather than dropping them. Pixel
coordinates start at the upper left; optical axes are +X right, +Y down, +Z
forward. A Gazebo camera pose uses +X forward, +Y left, +Z up. Conversion
rebases the authoritative world translation in f64 before narrowing and applies
the distinct Bevy camera basis. The current renderer remains a fixed 64×64
box scene; camera calibration does not add a sensor or an update scheduler.

The pinned gz-sensors source at `8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`
uses `CameraSensorUtil.cc` for its OpenGL projection. A native source probe
compiled that file with the pinned checkout's generated headers and the
canonical gz-math reference headers. At 640×480, fx=400, fy=450, cx=270,
cy=190, near=0.1 and far=100, it returned rows
`[1.25,0,0.15625,0]`, `[0,1.875,-0.20833333333333326,0]`,
`[0,0,-1.002002002002002,-0.20020020020020018]`, `[0,0,-1,0]`.
The retained probe is `crates/gz-render-frame/tests/fixtures/native_projection.cc`.
With the pinned sensor checkout in `SENSORS_SOURCE`, its generated build headers
in `SENSORS_BUILD/include`, and the canonical SDK prefix in `REFERENCE_PREFIX`,
it can be reproduced with:

```sh
PKG_CONFIG_PATH="$REFERENCE_PREFIX/lib/pkgconfig" c++ -O0 -g0 -Wall -Wextra -Werror \
  -I"$SENSORS_SOURCE/src" -I"$SENSORS_BUILD/include" \
  crates/gz-render-frame/tests/fixtures/native_projection.cc \
  "$SENSORS_SOURCE/src/CameraSensorUtil.cc" \
  $(PKG_CONFIG_PATH="$REFERENCE_PREFIX/lib/pkgconfig" pkg-config --cflags --libs gz-math) \
  -Wl,--disable-new-dtags -Wl,-rpath,"$REFERENCE_PREFIX/lib" \
  -o /tmp/gz-camera-native-projection
env -u LD_PRELOAD -u LD_LIBRARY_PATH -u LD_AUDIT /tmp/gz-camera-native-projection
```

The matrix is compared after converting OpenGL depth [-1,1] to finite reverse
[1,0]; the implementation uses glam's maintained asymmetric frustum. Native
skew 35 changes the first row's second entry to 0.109375 and remains explicitly
unsupported here. The probe directly compiled pinned source and did not load a
native camera, renderer or sensor library; it is not an upstream image test.

The pinned `CameraIntrinsics` test expects focal length 866.23 within 0.3 for
1000×1000 with horizontal FOV 1.0472, and exact explicit values 866.23/500
and 900/501. The numerical controls retain those tolerances and values. Native
`CameraSensor.cc` reports projection Tx/Ty but ignores them when building the
render matrix; this slice rejects nonzero values instead of claiming stereo
semantics. The fixed Gazebo default-camera basis is also visible in the pinned
Ogre2 camera's -90° yaw and roll, and the original image fixture observes a
box at +X. No original sensor test is credited by the numerical controls.

An explicit in-process renderer control produces actual 64×64 Vulkan images on
llvmpipe for a +X-facing rebased camera, an offset principal point, and finite
near/far clipping. It also verifies rejected updates preserve the previous frame
and that legacy fixed-camera snapshots still work. This is a software-adapter
control, not an upstream camera image port or physical GPU qualification.
The private protocol v2 candidate adds an atomic calibrated scene request. The
client requires the exact version advertised by the worker in Ready. A v2 client
rejects a v1 worker, and the existing v1 client rejects a v2 worker; deployed
client and worker pairs must upgrade together. A direct protocol user must check
Ready before sending requests. Existing 64×64 default-camera Snapshot requests remain
available in v2. Calibration and world pose are validated before mutation, and
unsupported skew or projection translation returns InvalidSnapshot. A real
same-graph worker test on llvmpipe verifies shifted pixels, the request stamp,
finite far clipping, and that rejected skew, dimensions and pose leave the
prior rendered image intact. A separate same-graph control extracts canonical
ECS visual geometry, combines it with an explicit camera request, and checks
the shifted rendered pixel and committed stamp. These tests are not physical-GPU
or native sensor image qualifications.

SDF sensor binding, variable-resolution images, RGB publication, camera-info
and timing semantics, demand activation, distortion, and depth remain separate
requirements. Full coverage, independent review, staged and hosted gates remain
pending.
