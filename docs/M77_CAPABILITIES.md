<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M77 preview capabilities and limits

M77 remains unaccepted. Frozen Source `353dc9ae` passed its local normal/DATA,
four-role build, package audit and clean installation with 47 finite loads. Its
first installed camera operation failed at startup, before any SDK consumer:
the web Scene exporter rejected the M76 fixture's opaque visual flags 4/1/2.
Those results and the original hosted attempt-1 failure remain retained; they
do not qualify the current correction candidate. The public M70 payload stays
its original `7094a318` generation with 41 examples.

The selected correction lets the default primitive viewer's implicit all-bits
mask display nonzero visual flags. Zero flags, transparency and unsupported
materials remain refused; camera masks retain their separate bounded policy.
Viewer assets, protocols, GPU code and solvers stay unchanged. The same candidate
includes reviewed wrench-harness admission preconditions, without attributing
the earlier hosted failure to a proven cause. Complete Source review and fresh
mandatory qualification precede any new package/runtime or acceptance claim.
The reviewed `95ef6ced` correction's full normal attempt then failed in a new
wrench-test FIFO EOF assumption; Renderer was not reached. This successor fixes
that test-only finite-query contract and rejects incomplete/errored readiness
snapshots under the original five-second bound. It has no qualifying check yet.
The original `353dc9ae` unchanged hosted retry succeeded; neither that success
nor the preserved failed normal establishes acceptance of this successor.

## Package and runtime

The selected M77 inventory has 47 existing finite SDF examples and four release
executables: simulator with the existing `rgb-camera` feature, native-free drive
and joint helpers, and the offscreen camera worker. The default robot, six c404
viewer files and both 14,000,000-byte gzip budgets remain unchanged. Three native
roles remain GPU-free; the optional worker uses an admitted ordinary OS Vulkan
loader/lavapipe installation. No compiler, Gazebo SDK or checkout is needed by
the producer. Rapier f64 and TPE remain; DART and MuJoCo are retired.

The manifest and notices describe the actual delivered files, roles, source and
dependency graphs. Published license declarations and disclosed canonical-text,
dirty-VCS and empty-source cases remain preserved. Notice collection is technical
evidence, not a new author grant or legal verdict. Preserve the complete prefix
on relocation and verify it before use.

## Cameras

- Existing 64 by 64 RGB/depth output supports the retained single, homogeneous
  pair and explicit one-RGB/one-depth configurations. This release adds no count,
  resolution, shader, worker protocol or sensor implementation.
- M71–M76 examples expose independent mixed-pair demand/rates, explicit K and P,
  signed intrinsic skew, per-camera visibility masks and signed P translations.
  The six worlds need both explicit launcher camera options for live output.
- Explicit P selects rendering focal lengths/principal point with zero render
  skew and translation. Its signed tx/ty values affect only CameraInfo P[3]/P[7].
  Absent P preserves selected K/HFOV behavior. Finite metadata offsets do not
  imply a physical stereo baseline or move a camera.
- Without explicit P, representable signed K skew affects image projection.
  Canonical CameraInfo retains the pinned native publisher's zero skew entries;
  it does not encode that rendered skew. RGB bytes and all 4,096 axial-depth
  samples must be interpreted using the applicable documented camera convention.
- Low 28 visibility bits and the existing all-visible default are supported.
  Whole-world validity and the 128-visual limit apply before filtering. Expected
  finite depth and positive-infinity background depend on the actual scene;
  background infinity is not required where a surface fills the image.
- No image interest means no image worker. Information-only interest can publish
  calibration without starting a worker. Pause, loaded reset, per-owner demand
  and retirement retain their existing semantics. Live camera output conflicts
  with browser lifecycle editing; canonical world control is a separate route.

The default robot has the maintained live browser controls; camera images remain
canonical external-consumer streams, not a new browser panel. Ellipsoid RGB uses
the supported native-only server route. Wide-angle, distortion/noise, RGBD
registration, point clouds, physical stereo, remote Fuel loading and arbitrary
scene/graphics compatibility are outside this preview.

## Evidence boundary

M77 must obtain its own four-role build, notice/ELF closure, archive/relocation,
47-world clean-install load results, complete installed mixed-camera/skew frame
checks, separate default-browser demonstration and process/listener shutdown
evidence. Resource limits and observed exits belong to those actual operations;
an 8 GiB camera container is not proof of a fresh 1 GiB browser fit. Unknown
grandchild/SDK quiescence outcomes must remain unknown where unobserved.

Unchanged native fixtures and reviewed numerical oracles may be reused with their
source bindings. Old M70 failed combined operations, missing depth host waits and
other historical failures are not repaired or relabeled by M77. Main ancestry,
unit tests or package verification alone do not establish release acceptance.
