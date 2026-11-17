<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Explicit directional illumination

The optional `gpu` API offers `set_directional_snapshot` and
`set_calibrated_directional_snapshot`. Each accepts the existing complete
`Snapshot` and exactly one `DirectionalWorldLight`. This is a programmatic
renderer input; Scene extraction continues to refuse world lighting.

`illuminance_lux` is nonnegative illuminance in lux, independent of SDF's relative
intensity. `color` is linear RGB in `[0, 1]`. The quaternion uses f64 Gazebo
world axes and maps local vectors to world vectors; `direction` is the local
propagation direction. Its magnitude has no effect. Direction is normalized and
rotated in f64 before the existing Gazebo-to-Bevy frame conversion and f32
rendering. A directional light has no position. Its nonzero ID must differ from
all visual IDs. Invalid orientations, directions, colors and illuminance are
rejected, including illuminance that overflows or underflows the f32 input.

A successful explicit lit snapshot owns one Bevy directional-light entity.
Updating its ID retires the old entity. Shadows and contact shadows are disabled.
Box materials use Bevy lighting with zero reflectance and maximum roughness;
the camera's ambient contribution is zero. Camera exposure retains Bevy 0.20's
default, EV100 9.7, with the existing disabled tonemapping and dithering. These
choices do not define an SDF intensity conversion or native pixel equivalence.

Zero lux keeps the lit-material path with no directional contribution. Calling
an existing unlit snapshot method removes the owned light and camera ambient
override, and restores the original unlit material settings. All light, camera
and visual validation precedes scene mutation and readback cancellation. The
existing bounded readback and snapshot-update admission rules apply. Lighting
adds no application callback, worker, endpoint, plugin or dependency.

The added tests cover f64 conversion, invalid inputs, replacement and removal,
atomic rejection, software face response, calibrated cameras and full-frame
unlit restoration. `cargo test --offline --locked -p gz-renderer --features gpu
-- --nocapture` passes all nine unit and three software tests, with no failures,
ignored tests or filtered tests. The Vulkan runner asserts a CPU adapter named
llvmpipe (Mesa 25.2.8, LLVM 20.1.2). This includes all retained renderer tests.
The build uses Rust 1.97.1, one build job, disabled incremental compilation and
zero debug information. Source-bound commands and logs are retained in ignored
`artifacts/explicit-lux/`.

Strict coverage, complete staged and exact hosted qualification remain pending.
Native radiometric or pixel parity, physical-GPU qualification, SDF-light
admission and original upstream case completion are not established.

A subsequent test-only lifecycle increment passes the complete package with nine
unit and four software tests on the same asserted llvmpipe runner. It submits a
real screenshot through an advanced Bevy update, replaces lighting while the
capture's delivery slot remains owned, then removes lighting during another
capture. Each cancelled slot remains busy until retirement and delivers no stale
frame. Successor stamps and full-frame calibrated unlit restoration are checked.
The device may already have completed when replacement arrives; this test does
not claim an unsignalled GPU operation at that instant. Separate source-bound
results are retained in `artifacts/explicit-lux-lifecycle/`; the earlier twelve-
test results remain unchanged. Complete integration qualification is still pending.

Snapshots that remain unlit skip lighting cleanup and retained-material lighting
writes. Only removal of an installed light restores those material defaults and
removes the ambient override. New unlit materials use the original default
construction. This avoids unnecessary admission work without a performance claim.
The unchanged thirteen-test package passes after this adjustment; separate logs
are retained in `artifacts/explicit-lux-transitions/`.

## Source-only calibrated metric depth

The new `set_calibrated_depth_snapshot`, `request_depth_readback` and
`take_depth_frame` API is an unqualified RENDER-02 candidate on accepted `c5dd02c0`.
After the initial attempt described below, the accepted primitive-inertia consumer
parent was fast-forwarded once. Its producer pin and publication changes touch no
renderer files. All four reviewed code/test payloads, including the appended
lifecycle correction controls, remained byte-identical across that integration;
only this factual README parent note then changed. The affected required gate
will qualify the corrected feature against the accepted consumer dependencies.
The first attempt compiled and passed the new depth controls but failed owned
region coverage. The amended controls and complete corrected qualification remain
unrun. The earlier light-only package logs above are historical evidence for
those changes, not depth qualification. No SDF sensor extraction, worker wire
format, scheduler, server, transport or intensity-to-lux policy changes.

This bounded programmatic API renders the existing opaque box snapshots through
the existing 64×64 calibrated camera. A `DepthFrame` contains epoch, simulation
time and 4096 row-major `f32` optical-forward axial distances in metres, starting
at the upper left. It is not Euclidean range. A zero reverse-Z depth sample maps
to positive infinity for background/far clipping. Nonfinite/out-of-range raw
samples or malformed delivery fail the capture rather than generating a frame.
Unprojection uses maintained glam with the inverse of the actual narrowed
projection matrix sent to Bevy; it does not ray-cast or synthesize a trajectory.
Authoritative camera/box transforms remain f64 before render conversion.

Native gz-sensors `8ddf4c17` integration `depth_camera.cc` requires centre-face
depth within `DEPTH_TOL=1e-4` and background/far positive infinity. Its near-range
negative-infinity behavior is outside this slice: a single raster depth texture
cannot distinguish near-clipped geometry from empty background. Every oriented
box must have all eight f64 camera-local corners strictly beyond the near plane.
Touching/intersecting/behind boxes are refused before snapshot mutation or
readback cancellation, even if outside the horizontal/vertical field of view.
Rotated cameras, world-origin rebasing and asymmetric pinhole intrinsics reuse
the existing frame/calibration API. No native original-case, native near-range,
physical GPU or whole depth/RGBD sensor completion is claimed.

Only explicit depth mode sets the camera to one sample and enables COPY_SRC on
its depth attachment. Bevy 0.20 `core_3d` uses Depth32Float, and the main opaque
and transparent nodes store that attachment. After an existing non-pipelined
update, a request copies that actual retained texture into one bounded staging
buffer; width64 makes its four-byte rows exactly 256-byte aligned. wgpu submits
the copy after rendering on the same queue. Its asynchronous map callback owns
the buffer until completion, copies the actual bytes, unmaps, and releases it.
Device polling is nonblocking and explicitly driven by `update`/delivery calls.
No render graph node, shader, plugin, scheduled callback, persistent worker,
background poller or dependency is added. No staging buffer exists without a
request. Empty optional depth state owns no heap resources.

Depth and RGB share the existing single-slot admission contract. Either pending
kind makes another request Busy. A new valid snapshot cancels delivery but leaves
the slot occupied until its actual callback retires. Epoch/time and projection
are captured at request, so a cancelled old frame cannot acquire successor
metadata. A poisoned completion lock retains slot ownership. Device loss is
terminal like the existing RGB path. Legacy snapshot methods restore the
original camera depth usages and default MSAA when leaving depth mode, and do no
depth-mode work when none was installed. `Snapshot`, RGBA `Frame`, all old test
assertions and protocol/client/worker contracts remain unchanged.

Source regression controls cover inverse-projection axial depth and invalid raw
samples, oriented f64 near-plane admission, actual GPU box-face/background/far
and empty output, asymmetric intrinsics/camera rotation/origin rebasing,
occlusion, invalid-admission atomicity during real readback, shared-slot
cancellation/retirement/successor stamps and full-frame calibrated RGBA restoration.
Their tolerances do not change any retained assertion. The mapped upstream
centre/background assertions are source references only until actual software
GPU execution and complete normal/hosted qualification pass.

The first completed-feature mandatory run compiled and passed all three new depth
controls (927 headless and 549 renderer tests passed), then stopped at the owned
renderer region threshold: 1009/1066 regions, below the unchanged >95% requirement.
It did not qualify the feature. Additional private controls now exercise missing
camera/attachment admission, terminal-device admission, overcapacity preserving a
pending capture, corrupt completion retirement and poisoned-slot ownership. These
amended controls are unrun; numeric bounds, production code and old assertions
remain unchanged pending the next consolidated required gate.


## M66 calibrated RGB spheres (Source complete, verification pending)

`SphereVisual { id, pose, radius, color }` joins Box and Cylinder in the existing
`PrimitiveSnapshot` RGB setter. The shared 128-visual count and ID namespace,
opaque finite RGB, f64 pose/origin/calibration and positive finite dimensions
are validated before any asset/entity/camera/stamp mutation or readback
cancellation. Radius and diameter must remain positive and finite after f32
narrowing. Legacy box structs/setters, directional boxes and box-only depth
remain unchanged; the RGB primitive setter does not enable depth output.

The locked Bevy 0.20 `Sphere::new(0.5).mesh().uv(32, 18)` produces one lazy
cached mesh (627 vertices and 1,088 triangles), scaled uniformly by diameter,
without an additional axis rotation. Box-only and Box/Cylinder paths allocate
no sphere. At most the three fixed shape meshes are cached; size changes or
same-ID shape replacement do not create per-size meshes. Entity mesh handles
and readiness follow the actual selected shape; removal releases only the
entity/material while retaining the bounded shared mesh cache.

New private controls cover invalid-input/pending-state purity, mixed shared
IDs/counts, all three replacement kinds, UV data, selected-mesh readiness and
bounded assets, Busy/device/depth and cancelled-buffer ownership. The software
pixel control uses sphere interior, an AABB-only corner and a near box occluder,
then checks origin/camera equivalence, real asynchronous cancellation/reset,
empty output and exact full-frame old Box restoration. Existing controls and
assertions are retained. These are unexecuted regression Sources until the
completed batch passes independent review and the consolidated required gates.

This is the fixed 32-sector/18-stack UV approximation, not an analytic sphere,
native Ogre image comparison, physical-GPU/performance claim, depth sphere or
installed-worker distribution. M65's installed 35-example Latest is unchanged.

## M67 calibrated RGB ellipsoids (Source complete, verification pending)

`EllipsoidVisual { id, pose, radii: DVec3, color }` joins the existing primitive
snapshot RGB path. Radii are local Gazebo X/Y/Z semiaxes. Every radius and its
diameter must be positive finite in f64 and after f32 narrowing, before any
asset/entity/camera/stamp mutation or pending readback cancellation. Shared IDs,
128 visuals, material, pose and calibration admission are unchanged. This API
validates canonical values; the consumed SDF parser's whole-unit-triple fallback
for any nonpositive XML radius remains outside renderer policy.

An ellipsoid reuses the exact existing radius-0.5 UV32×18 sphere mesh and its
readiness handle, including ellipsoid-first allocation. Local diameters map to
Bevy scale `(2rx, 2rz, 2ry)` through the unchanged frame conversion, without an
extra rotation. At most the same three Box/Cylinder/Sphere meshes are cached;
there is no fourth mesh, per-size cache or new tessellation/material/shader.
Equal radii preserve the Sphere path's full RGBA. Same-ID replacement preserves
entity/material ownership while updating the mesh and axis scale.

New regression Sources cover each-axis invalid/pending purity and mixed128/129,
ellipsoid-first cache/actual selected-mesh readiness, four-shape replacement,
Busy/depth/device/buffered retirement, asymmetric and rolled silhouettes,
near-box occlusion, actual asynchronous cancellation/reset, large-origin/common
camera-pose equivalence and exact old Sphere/Box full-frame restoration. Existing
controls are retained; two old exhaustive-match unreachable patterns only add
the new enum variant, preserving their inputs and assertions. No new controls
have run until independent Source review and consolidated mandatory gates.

This is opaque calibrated64×64 RGB using the existing finite UV approximation.
It does not add depth ellipsoids, lighting, physics, native Ogre/analytic geometry
or performance parity, new dependencies, or installed-worker distribution.
M65's public Latest and 35 installed examples remain unchanged.


### Calibrated primitive metric depth (M68 Source preparation)

`Renderer::set_calibrated_primitive_depth_snapshot` installs the actual existing
Box, capped Cylinder, Sphere and Ellipsoid meshes for the same 64×64 unlit
calibrated metric-depth path. The cache remains one unit box, one fixed
32-facet capped cylinder and one UV32×18 sphere shared by ellipsoids. The legacy
Box depth API and RGB APIs retain their behavior. This is Source preparation;
verification and M68 qualification remain pending.

Admission first checks the existing conservative oriented-box near policy,
using local dimensions `(2r,2r,length)`, `(2r,2r,2r)` or twice the ellipsoid radii.
Every corner must be forward of the near plane. This restriction can refuse a
curved surface that would otherwise have a safe visible patch; it does not add
native near-range −INF semantics. The complete dimension/f32/ID/color/pose/
calibration/count/busy preflight precedes scene mutation or capture cancellation.
The successful primitive install then restores single-sample depth COPY_SRC
mode. RGB transitions restore the old RGB camera mode, and both kinds share the
same bounded capture slot until actual pending callbacks retire.

Depth is optical-forward axial metres from the actual stored finite-mesh depth
attachment, with clear and far clipping represented by positive infinity.
Curved depth is not an ideal analytic sphere/cylinder/ellipsoid or a native
renderer parity claim. The new software regression uses a few interior rays and
independently constructed locked CPU meshes, explicit WorldPose/axis/scale and
narrowed calibration, preserving the existing 1e−4 metre bound. Its oracle does
not read installed transforms, GPU assets or output to derive expectations.
Canonical wire, scheduling, installation and native-consumer evidence belong to
their separate gates; this renderer API does not add packaging or depth pairs.
