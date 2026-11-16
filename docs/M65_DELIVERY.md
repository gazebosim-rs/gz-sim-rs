<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M65: installed eight-owner fleet preview

M65 follows actually accepted M61–M64 at
`192ede6369ab6d83ecf88230944d2061f74b5e94`; see
[the predecessor acceptance](M61_M64_ACCEPTANCE.md). M1–M64 and197 slices are
accepted. M60 remains Latest with33 installed examples. M65 implementation,
regression Source and guides passed independent review, with four Rust/six Python
control declarations. The single affected formatter passed after accepted-parent metadata and whole
Source review. No M65 test, release build, installation, browser or release has
run. Final token, census and staged-tree bindings precede its complete
consolidated verification. No result transfers
between candidates, and frozen predecessor Source remains unchanged.

The budget is at most five working days including review and mandatory gates.
Deliver the smallest useful current native-free Ubuntu 24.04 x86_64 fleet
preview: the existing simulator and two command helpers, the exact M61 eight
independent differential-drive robots and M62 eight two-link controller chains,
and the unchanged accepted viewer. Do not expand RGB-worker distribution into
this increment. The source qualifier's 37 finite examples are distinct from
M65's 35 installed examples; copying a camera SDF does not prove a camera runtime.

## Implementation boundary

Preserve `NOISY_EXAMPLES` and its existing 33 entries exactly, including order.
Preserve the closed historical M44/M60 guide selection and actual guide bytes.
An explicit new M65 guide generation selects its own three guides and a closed
35-example registry: the old 33 followed by `m61_eight_diff_drives.sdf` and
`m62_eight_joint_controllers.sdf`. Reuse those two fixtures byte-for-byte. Every
selected example uses the existing file bounds, real copies, manifest/hash/mode
checks, path restrictions and relocation verification; no silent fallback or
arbitrary guide/fixture path is admitted. Existing generation behavior remains
unchanged. The default robot, duplicated mesh/notice obligations, six accepted
viewer bytes, both gzip budgets and source-bound three-ELF records stay required.

The existing installed `gz-drive-command --topic` addresses all eight drive
routes. Add only an optional trailing `--topic TOPIC` to `gz-joint-command`, using
the authoritative `ServerOptions::topic_key` mapping and concrete upstream Zenoh
key admission. With the option absent, all position/velocity/force topics and
semantics remain exact. Keep four positional arguments, finite target and
0..=3600-second duration admission, canonical `gz.msgs.Double` attachment/encoding,
five-second subscriber deadline, zero-duration single target, and velocity/force
terminal zero. Position keeps its target. Reject malformed option counts,
unknown flags, empty/invalid/wildcard topics before opening a session. Do not add
a new transport, interactive command framework or launcher subcommand; the
installed helper is sufficient. Its guides must set the actual matching
`GZ_PARTITION` and endpoint explicitly; the launcher does not modify the parent
shell or provide a joint-command subcommand. The real session remains upstream
Zenoh.

## Ownership and meaningful controls

Light owns `scripts/package-preview.py`, `tests/test_preview_package.py` and
`docs/M65_INSTALL.md`, `docs/M65_CAPABILITIES.md`, `docs/M65_PREVIEW.md` in the new
M65 checkout. Package controls must preserve old33/M44/M60 tests and cover exact
M65 registry/order, guide-source closure, real manifest-bound copies, missing or
tampered fleet input and installed-copy refusal, relocation and no linked files.

Messages owns only `crates/gz-sim-server/examples/joint_command.rs`, including its
private regression module. Preserve the existing actual canonical force
subscriber test. New controls must exercise parser boundaries and unchanged
default topics/modes, authoritative topic mapping, concrete-key rejection and
actual canonical delivery to an explicitly selected subscriber without delivering
to an unselected owner. Cover terminal zero and position target retention through
real helper behavior rather than duplicating the parser's implementation.

Root owns shared guides/planning, integration and all operations. SDF independently
reviews the completed package/helper batch and Root integration. These lanes
share one isolated checkout only because their exact owned leaves are disjoint;
no author edits accepted or frozen qualification trees. Any newly discovered
shared seam needs one explicit writer. Source authoring and review may overlap
predecessor hosted closeout under D31. Heavy execution remains serial under Root.

## Required exit evidence

Complete implementation, meaningful regression code, current documentation and
independent whole Source review; batch findings before one affected formatting
pass and the complete mandatory normal. Require unchanged strict workspace,
per-crate and actual accepted-parent line/region gates, independent local DATA,
exact-head hosted CI, original hosted evidence and independent official DATA.
The actual accepted predecessor must exist before M65 qualification/publication.
Do not reuse a predecessor's check as a pass for changed M65 Source.

After the complete M65 local normal and independent local DATA pass, build all
three actual release ELFs from that exact Source and collect exact dependency
notices/ELF closure using maintained procedures. Separately Source-reviewed local
provisional builds, packaging, installation and demonstration may overlap that
same head’s remote CI and official closeout; Root still serializes local heavy
operations. Bind every result to its real Source, ELF, Cargo, viewer and manifest
identities. A CI-driven Source correction requires fresh applicable bindings and
checks; no result transfers to changed inputs. Hosted success, official durable
originals/DATA and ordinary Main/readbacks remain required before acceptance,
public release or Latest. This does not reuse any prior operation approval.
Require actual
assembly/archive and relocated manifest verification, independent package DATA,
and one supported clean non-root Ubuntu installation without a development
checkout, compiler or physics SDK. Exercise all 35 installed examples finitely;
those loads are separate from actual command and browser evidence.

Use the existing live web Scene output for a bounded proof-side numeric reader;
retain full original JSON frames, actual IDs, timestamps, epochs and xyzw
quaternions. M62 base/tip visual rotations derive from actual world poses, so
base yaw and inverse(base) × tip relative yaw provide physical witnesses without
inventing a controller output or acknowledgement. The bridge coalesces ordinary
frames near 34 ms; do not claim 1 ms observation, every physics state, a command
ACK or complete delivery from these frames. Pin the exact admitted schema and
all names/limits in the independently reviewed future reader.

The web bridge admits one client. Complete and retire the numeric reader's
socket before attaching the actual browser to the same installed application;
keep the numeric and visual observation intervals explicitly separate. Reuse
maintained live framing/validation, not a replay-only reader unchanged. No viewer
bytes, product telemetry, recording API or launcher change is needed for this
proof. Require actual pause/reset epoch/time/pose and owned process/listener
retirement, including failure paths, in the reviewed future executor.

The installed command demonstration must address eight drive owners and all
16 selected M62 position/velocity inputs, with actual scoped physical/target or
application/sample evidence and cross-owner isolation. Extend a maintained bounded
observer only after Source review; topic discovery or successful publication alone
is not proof of physical application. Observe the real installed browser through
Play, command-driven motion, Pause, Reset and clean owned launcher/process/listener
shutdown. Preserve actual output, images and unknown exit limitations; never
invent an application or grandchild exit code. An independent retained DATA review
must clear those actual proofs before ordinary Main/tag/release/publication
readbacks and the new Latest. Keep M60 release assets and Latest unchanged until
the final successful publication boundary.

Exclude RGB/depth-worker or GPU/software-renderer distributions, SDK/provider
bundles, new viewer builds or protocols, new physics/transport/runtime/packaging
frameworks, trajectory or native-System parity and performance claims. Keep all
resource floors and current preserved evidence. No cleanup authorization is
created by this contract. If a required proof reveals a real limitation, batch
the correction or split optional scope before the candidate freezes; never weaken
the evidence or claim unit readiness is release acceptance.
