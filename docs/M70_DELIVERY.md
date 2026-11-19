<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M70: installed camera preview

Budget: at most five working days including independent review and all mandatory
checks. Parent: Main `8f13dc7446be4fded424b1eba83df4c8253d0cef`, which includes
accepted M69 and the completed D33 backend retirement. Rapier f64 and TPE remain;
DART and MuJoCo adapters, dependencies and dedicated jobs stay retired.

The frozen retained-backend candidate
`597ac3ddccf150cc1c45a60c3c72b838f943093a` passed its local normal and strict
coverage gates, release builds, package audit, clean installation and 41 finite
examples. Its hosted run failed before product checks at Ubuntu snapshot HTTP
401 responses. The first installed camera trial failed at its first packet
barrier after canonical Clock and Scene reached 1 ms; no numeric camera success
or complete application shutdown was established by that failed trial.

The shared-session correction over 597ac3d gave the CLI an explicit shared-world
Zenoh-session path while preserving separate-session constructors and ownership.
Its regressions cover the listener-only packaged configuration and camera
reset/removal without closing the world session. The harness retains the native
20-second deadline instead of cancelling after 10 seconds. The first normal on
tree `56b8e704` failed after
541.339 seconds: Clippy and 466 Python checks passed, and all 15 CLI camera cases
passed. The auxiliary qualifier's closed registry still expected 14 and rejected
the extra new regression. The complete failed packet and Source remain frozen.
The registry correction and its controls subsequently passed full local
normal/strict qualification and independent audit at `2e8d9dc`.

This isolated successor uses that frozen, locally qualified `2e8d9dc` parent and
corrects the WebSocket close handshake described below. M70 remains unaccepted;
new qualification, release and installed runtime evidence are required. Historical
trees, all original failures and Fuel work stay unchanged. No gate is weakened.

Live browser acceptance is held for explicit approval to access
`http://127.0.0.1:8082` and its launcher-generated URLs after the retained automatic
approval rejections. General continuation does not grant that approval. No browser
retry or alternative surface is released by this Source adoption.

## WebSocket close-handshake correction

The locally qualified `2e8d9dc` candidate and its package/installation evidence
remain frozen. A later camera trial completed both native cadence consumers and
proved worker retirement, but its Scene witness observed an abnormal WebSocket
close (1006) after explicit finish and confirmed clean input EOF. The pinned
Tungstenite implementation queues a close reply when reading the peer's Close;
the CLI previously dropped the socket before driving that reply.

This isolated successor attempts one flush on peer Close under the existing
100-millisecond write timeout, then retires the peer even if flushing fails.
Its real-socket regression requires the actual echoed empty or normal-code/reason
Close frame before clean transport termination, unchanged simulation state and
a working replacement connection. It does not change wire formats, command
admission, timeouts or numerical controls. Complete independent Source review and
one consolidated normal/strict pass before qualification; changed release and
installed runtime evidence remain required. No check has run on this successor.

## Bounded result

Create one Ubuntu 24.04 x86_64 preview generation with four manifest-bound release
ELFs: simulator with its existing `rgb-camera` feature, native-free drive and joint
helpers, and the existing GPU renderer worker. Reuse the six exact accepted c404
viewer files and unchanged 14,000,000-byte gzip cap per Wasm. Preserve all older
manifest/guide generations and old 33/35 fixture order; only M70 appends the six
camera fixtures for 41 finite examples. The runtime scope includes the explicit
CLI reuse of the existing upstream world session and completion of its automatic
WebSocket close reply. No Cargo, renderer, protocol, SDK, physics, dependency,
solver or new transport framework change is selected.

`run --camera-output` is explicit and M70-only, derives its fixed verified worker
path, and refuses lifecycle editing. The ordinary default robot path stays native
free. Camera admission rejects nonempty inherited LD_, VK_, WGPU_, MESA_, GALLIUM_
variables and DRI_PRIME before effects; it uses one ordinary OS lavapipe ICD with
an absolute allowed provider path or the exact system basename
`libvulkan_lvp.so`, and an owned private runtime. Basename admission requires one
canonical regular provider in the fixed system library directories; usrmerge
aliases count once. Other relative paths, missing or ambiguous providers and
escaping links are refused. The ICD is unchanged and the Vulkan loader performs
normal OS resolution; actual mapped-provider proof remains separate.
Uncertain spawn/retirement or runtime identity retains and reports
the path; HTTP/signal cleanup remains independently attempted. Admission does not
prove actual provider mapping or resource fit.

## Cache and final-wait correction

The prior candidate `916e8738` and its successful code/package/install records
remain frozen. Three browser attempts are retained: cached-decoder failure,
asset-evidence refusal, and observed robot controls/guest shutdown followed by
outer exit 143 without a final receipt. Docker wait and the termination sender
remain unknown; no complete browser qualification is inferred.

Each corrected launch prints a fresh `/__gz_preview__/<launch-token>/` directory
URL. Relative startup, module and Wasm requests serve the same six verified
files. Stale/escaping paths are refused and static responses use no-store rather
than conditional 304. Qualification uses the actual URL printed by that owned
launch. An artifact-only controller must preserve the actual terminal wait/reap and
bounded partial output; its exit remains unknown unless observed. This does not replace any application, camera or release gate.

## Ordinary OS manifest correction

The frozen `d0ba8656` candidate's code, package, default installation and robot
browser evidence remain retained. Its first camera profile failed admission:
the ordinary Ubuntu ICD uses `library_path: libvulkan_lvp.so`, while the launcher
and proof procedure required an absolute path. The stopped-container diagnostic
retained the unchanged ICD and its package-listed provider path; it did not load
a driver or prove camera frames.

The frozen fcd5 successor carried the d0ba tracked delta over accepted M69 and
corrected basename admission with meaningful Python regressions and these guides.
Those changes remain on the D33 parent. The shared-session correction retained
here preserves every absolute-path, override, cleanup and resource control. Complete
implementation, regressions, guides and independent review precede one consolidated
verification pass. No earlier result qualifies the changed candidate.

## Acceptance sequence

1. Finish implementation, meaningful regressions, guides and whole independent
   Source review; batch findings before one consolidated normal/strict coverage
   cycle. Then independent local DATA, exact hosted success, durable originals,
   official DATA and normal Main promotion are mandatory.
2. Build four separate source-bound release roles; retain exact Cargo metadata,
   all graph IDs/notices and ELF dependencies. Same frozen source root binds the
   assembler and build target paths. Assemble, verify, relocate and independently
   audit complete payload bytes, modes, notice graph and accepted viewer reuse.
3. Install in clean supported Ubuntu with ordinary signed OS packages, no SDK or
   compiler in the producer, unprivileged execution and actual package/provider
   mappings. Verify all 41 finite load/two-step results. They do not claim images.
4. Separately prove installed M69 paired depth: full Image/CameraInfo and numeric
   optical-axis depth, independent 10/20 Hz schedules, image/info demand laziness,
   pause/reset/stale refusal and worker/listener retirement. Use bounded external
   upstream consumers and existing controls; close first-stamp and reset-epoch
   observation barriers in reviewed procedures before invocation.
5. Prove one installed M67 RGB ellipsoid smoke and a separate default robot live
   browser forward/turn, Play/Pause/Reset and owned shutdown. Retain actual outputs,
   screenshots, process identities, cleanup exits and independent DATA. Worker or
   simulator-grandchild exits remain UNKNOWN where the parent does not expose them.
6. Publish only the fully checked release with actual readbacks and concise tracker
   evidence. M65 Latest stays unchanged until that publication succeeds.

The old robot resource profile stays 1 GiB/128 PIDs/2 CPUs/96 MiB writable layer.
The camera profile is initially proposed, unmeasured, at 8 GiB/512 PIDs/4 CPUs and
512 MiB writable layer plus 128 MiB margin. Actual fit is unknown; fail closed and
review any measured budget change. No silent ceiling increase or driver injection.

See [installation](M70_INSTALL.md), [capabilities](M70_CAPABILITIES.md) and
[separate demonstrations](M70_PREVIEW.md). No check/build/install/frame/browser
or release result is claimed for this corrected candidate at this Source checkpoint. Preserve all accepted
Source, old failures, frozen evidence and Fuel work; Root serializes heavy jobs.
