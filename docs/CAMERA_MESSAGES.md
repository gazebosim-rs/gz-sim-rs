<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Bounded camera messages

`gz-camera-output::camera_messages` converts an explicitly supplied 64×64 RGBA
frame into RGB and constructs monocular CameraInfo from caller-supplied finite
intrinsics. The caller supplies the acquisition time, frame ID and image sequence.
The functions do not schedule sampling, advertise topics, start a worker or
perform GPU work. They reject invalid frame lengths, timestamps and intrinsics
before allocating message data. Image output has 192-byte rows and strips alpha;
CameraInfo uses zero plumb-bob distortion, shared fx/fy/cx/cy with zero projection
translation, and identity rectification.

The CPU-only native fixture `crates/gz-camera-output/tests/fixtures/native_camera_info.cc`
calls pinned gz-sensors `8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`
`CameraSensor::PopulateInfo` and `PublishInfo` without a rendering scene. It
supplies K/P=(30,31,20,21), frame ID `frame` and time 4 seconds + 5 ns; its
controlled Transport subscriber captures `native_camera_info.bin`. The Rust test
compares the complete serialized message and decoded fields. This checks one
native CameraInfo layout and wire observation; it does not qualify native image
pixels, HFOV derivation, scheduling, GPU behavior or full sensor parity.

The following recipe was independently reconstructed and replayed on Linux
with the pinned source checkout at `$SENSORS`, its generated build headers, and
the selected SDK at `$SDK`. It compiles the three pinned source files directly;
it is not a clean build of the complete native sensor library. The SDK uses its
normal log location. Run from the repository root, with pkg-config able to find
the SDK dependencies:

```sh
SENSORS=/home/nkoenig/work/gazebo/rust/gz-sensors
SDK=/home/nkoenig/local
export PKG_CONFIG_PATH="$SDK/lib/pkgconfig"
test "$(git -C "$SENSORS" rev-parse HEAD)" = 8ddf4c17be8a434c51b51d5be7c866fb69aaa54f
g++ -std=c++17 -O2 -I"$SENSORS/include" -I"$SENSORS/build/include" \
  crates/gz-camera-output/tests/fixtures/native_camera_info.cc \
  "$SENSORS/src/Sensor.cc" "$SENSORS/src/CameraSensor.cc" \
  "$SENSORS/src/CameraSensorUtil.cc" \
  $(pkg-config --cflags --libs gz-sensors-rendering gz-transport gz-common-graphics gz-common-events) \
  -lfmt -Wl,--disable-new-dtags,-rpath,/home/nkoenig/local/lib:/opt/ros/jazzy/opt/zenoh_cpp_vendor/lib \
  -o /tmp/gz-camera-info-recipe
python3 crates/gz-camera-output/tests/fixtures/capture-camera-info.py \
  /tmp/gz-camera-info-recipe /tmp/camera-info-wire.bin /tmp/camera-info-loader.log
cmp /tmp/camera-info-wire.bin crates/gz-camera-output/tests/fixtures/native_camera_info.bin
```

The runner holds two distinct discovery ports, selects a unique partition and
loopback, removes loader overrides and display variables, and bounds the owned
process group. The observed executable SHA-256 was
`93836d779682e039083ba8ff2bc25a018a0c19b94db75525c99a445e99b04abd`;
the fixture source SHA-256 is
`eb30f47eed7be33e038d9ce8845bb24fb60c59220e34b78e770b37e6404c3796`
and the 325-byte captured wire SHA-256 is
`672c054dd340890150450aee40a80f105b25810165e8ff98161e60e842997e39`.
The compiler was Ubuntu g++ 11.5.0, executable SHA-256
`02ba98cc5feefe173cfb8c28c98089817737800537dc7189138ed66b07cf56ec`.
The compiled `Sensor.cc`, `CameraSensor.cc`, and `CameraSensorUtil.cc` hashes were
`5fb2ea0aaa27b960c463c82de3074ad0bb221341454fb999946eca9f5afff2a2`,
`1cd5bb523fe1c2434501c70df262f5841d973b807a7ac44ca80f788da81ca50a`, and
`bb5f1112909188687768f69860a9035c0c2555db8e60d041a749c26d3dd80c57`.
Generated `config.hh` and `Export.hh` hashes were
`6aaac30836c8ead85def1a1cc1ae8656bcbe7d748a95c874d0aeb300239100ef` and
`5108acfc6ad232b703029a9dcf6b0a73570341598a89f105c68f1adcb9ec6c75`.
The loader trace identified SDK `gz-sensors-rendering`, `gz-sensors`,
`gz-transport`, `gz-msgs`, `gz-rendering`, and `sdformat` library hashes:

- `800f1420a74ca34e6f53f846497d69b0601471ed5c282027466ae5aafa8bc802`
- `3d12bbc482784363616d903290a2b958cf28904a97871b49da0b6c6e663d09ad`
- `bc9b06e4337e638b4f15d2397c974d34dd99a33957a7756e2fc424f78f08309a`
- `0504be34705191623dce9654e7b659663d87cdb92a3b99c83db13a7f60f13659`
- `0518fe836bf8300ffbb4e400b23564da71080b24bed6d98d01a0eb1cabfef564`
- `bcc864bcee56b49c13081e29722051e26c036260e3dd037e7736aa2b8620358f`

System protobuf was `53a2a625a30418d6ea84438b04c9196214dec51a30476596ee9be45ccadb0404`;
the SDK's ROS-packaged Zenoh runtime was
`de63c0072f4186cf55d325773962ea6621d391629c8b33edf276bb09062b68f5`.
`nm -C` confirmed executable-defined `Sensor::SetFrameId`,
`CameraSensor::PopulateInfo` and `CameraSensor::PublishInfo`. The loader trace
showed no Ogre, EGL or Vulkan module. This proves neither the native renderer
nor a scheduled camera sample.
