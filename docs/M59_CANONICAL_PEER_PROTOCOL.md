<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M59 canonical peer protocol — selected proof and retained partial execution

Build on `native_multiple_joint_controllers_peer.cc`'s bounded interactive
publisher/discovery protocol, `native_robot_sensors.cc`'s real upstream
`gz::msgs::Twist` and `Odometry` APIs, and `tests/support/zenoh_peer.rs`'s maintained
selected-client build/runtime contract. This is interoperability with canonical
messages and transport. It is not an original Gazebo DiffDrive implementation,
fresh original-System campaign, native motor oracle or trajectory-parity proof.
M58 full normal and independent retained DATA have passed. The reviewed peer
Source is adopted and its Rust consumer is under integration review. No M59
compiler/client/transport result is claimed before the required checks.

The Root-authored peer Source is
`crates/gz-sim-server/tests/native_multiple_diff_drives_peer.cc`, client key
`multiple_diff_drives`. Use the existing `gazebo-zenoh-client-v1` path, original
SDK installation and selected runtime-proof/environment dual bindings. Do not
copy a SDK, add a transport, weaken selection or bypass the maintained builder.
Keep all old client names, protocol outputs and Source bytes unchanged.

## Fixed owner and wire authority

Owner 0 is `robot_left`, 1 is `robot_right`. Use exactly
`/m59/{left,right}/cmd_vel` and `/m59/{left,right}/odom`; corresponding header
frames are `m59_left_odom`/`m59_left_base` and
`m59_right_odom`/`m59_right_base`. The Root fixture duplicates the actual M8 robot,
changes only agreed identifiers and model Y -2/+2, and keeps geometry, wheel
limits, initial Z/orientation, gravity and mathematics. No generated samples.

Advertise two genuine upstream `gz::msgs::Twist` publishers. Set linear.x and
angular.z from two finite binary64 words; explicitly set the other four vector
coordinates to zero. Retain the actual serialized command bytes. Subscribe to
Odometry only on explicit demand, using actual public Node subscriptions.
Use per-owner bounded callback state/mutex/condition variable; callback failure
becomes a recorded terminal peer failure. No callback exception crosses transport.

For each actual Odometry, retain serialized bytes and validate the complete
current Rust packet shape: header/stamp presence, normalized nonnegative integer
time, exactly ordered frame_id/child_frame_id/seq entries with one value each,
strict unsigned sequence text, finite pose and planar twist, zero unused axes,
unit yaw quaternion and default Pose id/name/header. Rust independently decodes
the retained actual wire and compares every numeric bit to the corresponding
real `DiffDriveSample` (yaw quaternion uses the unchanged packet formula).
Continuous encoder pose is not required to equal body ground truth. Sequence
starts at zero for a fresh Server stream, which Rust must assert explicitly.
Each new watch creates fresh callback/validation state: its first actual row
establishes that observation segment's valid stamp/sequence baseline, and later
rows must be strictly time-increasing and exactly sequence-contiguous with
wrapping-u64 semantics. Rewatching an existing Server stream does not imply seq0.
Do not claim wire epoch identity: Odometry has no epoch field; Rust owns reset
phase/epoch joins and the quiet drain/unwatch/reset/matched/watch sequence below.

## Proposed closed interactive protocol

Use one exact argv owner-count word `1` or `2`, fixed left-first owners. Reject
unknown/extra arguments, trailing phase words, noncanonical indices/numbers and
EOF before finish. Limit 4096 phases, 128-byte input lines, 128 queued Odometry
rows per owner, 512 bytes per raw message, and 1 MiB per captured stdout/stderr
channel. Fail on overflow; never silently discard a row. Every actual discovery
or row wait has one five-second deadline checked again after successful work.
Reuse the reviewed Rust child capture/actual wait/kill-and-reap/drop protocol;
no second monitor or transport framework.

| Command | Exact response / effect |
| --- | --- |
| startup | `READY multiple-diff-drives COUNT`, after actual typed command connections |
| `send OWNER LINEAR_HEX ANGULAR_HEX` | `SENT OWNER LINEAR_HEX ANGULAR_HEX WIRE_HEX`, after real typed Publish succeeds |
| `watch OWNER` | `WATCHING OWNER`, after creating the real Odometry subscription; this is not a Rust interest-listener acknowledgment |
| `take OWNER COUNT` | Exactly COUNT `ODOM OWNER WIRE_HEX` rows actually received, then `TAKEN OWNER COUNT`; bounded count 1..128 |
| `unwatch OWNER` | `UNWATCHED OWNER`, after public unsubscribe/callback-state retirement; this alone does not fence Rust's asynchronous matching listener |
| `matched OWNER` | `MATCHED OWNER`, after actual publisher HasConnections and TopicInfo command publisher/subscriber types agree |
| `retired OWNER` | `RETIRED OWNER`, after command HasConnections false/command subscribers absent and Odometry publishers absent |
| `finish` | `PASS canonical multiple diff drives`, then actual process exit0, with all recorded callback validity/caps checked |

`watch` requires an absent subscription; `unwatch` requires a present one and an
empty drained queue. Reject invalid owner/state transitions and repeated finish.
Each stdout record is emitted under one output lock; callback data is queued,
not interleaved with protocol lines. Wire hex is lowercase, exact, bounded and
never regenerated from expected values. `SENT` proves publication, not receipt
or actuation. TopicInfo is typed discovery, not a private command acknowledgment.
Rust establishes command receipt from actual Server counter deltas while paused,
then proves native wheel/body and encoder motion after explicit real steps.

## Root Rust case consumer and fences

Extend the genuine loaded two-owner Server harness with this peer; do not replace
or weaken its existing packet/physical/refusal assertions. Root owns actual
module routes and any additional test IDs. Start Server paused with unique
partition and loopback-only case-owned configuration. Use public
`apply_local_control` with asserted acceptance, not private `set_paused`.

1. Keep right Odometry NEVER subscribed initially. Watch left, advance until a
   genuine left Odometry callback establishes an actual output fence, then pause
   and drain every known queued left acquisition. Command both owners through
   canonical C++; pump paused Server until the exact two accepted counter deltas.
   Native q/v, sample cache and simulation time stay fixed during receipt.
2. For the initial physical-response observation, advance three consecutive
   1,000-step segments at the existing 1 ms step, checking and draining all 50
   demanded left wires per segment. Require the actual three-second simulation
   interval and 150 queued packets while retaining the original position baseline,
   commands and five-centimetre assertions. This extends the failed one-second
   test premise. Both three-second observations passed in the fifth failed full
   run; the earlier one-second production cause remains unknown and no full M59
   qualification follows from that partial result.
   Left demanded wire rows match its actual samples and sequence/cadence. Right real wheel/body motion and encoder samples
   progress independently despite no output subscriber. With right never watched,
   compare global queued increments to the actual left acquisition list. If using
   distinct 50/25 Hz rates, change only the explicitly reviewed case XML rate;
   keep the canonical finite example at its approved rates. Do not infer a
   listener fence from `watch`/`unwatch` acknowledgment or add a matching getter.
3. Watch right and use its first real received row as the demand fence. Subsequent
   uninterrupted windows compare both owners' full wires, independent frame pairs,
   contiguous sequences and sample timestamps. Paused equal-time updates produce
   no new acquisition, native movement or queued output. Keep genuine malformed
   frames/budget/reset-epoch negatives already covered by Rust controls.
4. Before reset, pause and collect all previously expected queued rows; require
   their actual received count and last known stamp. Send `unwatch OWNER` for
   EVERY active watch, require its existing `UNWATCHED OWNER` response and the
   empty drained queue, and retire that callback/validation state. Only then
   perform real public full reset + prepare_io + tick and assert Rust epoch
   increment/original IDs and zero origins/pending commands. Re-establish typed
   command connections with `matched`, then `watch` fresh observation segments.
   Their first actual rows establish new peer validation baselines; Rust separately
   requires the actual new Server stream seq0 and joins those rows to its observed
   reset epoch. Later rows are monotonic/contiguous within that new segment. Never
   retain a pre-reset watch validator, infer epoch from a wire stamp/sequence,
   or let old queued rows satisfy the new phase. No new protocol command or ACK.
5. First assert that local `remove_editable_model` correctly refuses the
   articulated M8 robot as `UnsupportedModel`, preserving the whole state. Then
   remove one genuine model through the existing `/world/default/remove/blocking`
   RPC using canonical `EntityRequest`/`Boolean`; require the actual true response,
   removal-counter deltas and original ID absence. Both positive RPC branches passed in the sixth headless run, which later
   failed strict Scene changed-line coverage. Full normal and independent DATA
   remain pending; the fifth API-premise failures stay retained. Assert
   its actual endpoint retirement via `retired`. Healthy peer connections,
   canonical commands, measured motion and Odometry continue. Loaded reset
   restores both originals with new stream sequences/leases. Drop Server, require
   remaining endpoint retirement, then finish and retain the real child exit.
   A printed PASS without actual wait0 cannot satisfy the case.

Use canonical C++ checks only for positive supported messages. Existing Rust
raw-frame controls remain the authority for wrong attachment, oversize, malformed
protobuf and unsupported axes. Do not fabricate a C++-generated negative message
or a numerical native trajectory to justify a Rust result.

## Root append-only integration points

- Add exactly one CLIENTS mapping to `scripts/zenoh_peer.py`, one matching explicit
  choice/Zenoh-only refusal in `scripts/native-peer.py`, and preserve all old
  dispatch entries. The selected helper/proof bundle remains centralized.
- Append the client to both maintained headless native-client expected sets in
  `scripts/verify.py`, Source input/retention/custody joins, profile/evidence
  enforcement and focused registry negatives. Renderer retains its existing
  camera-and-clock native peer selection; M59 is added only to headless sets. Do not claim a new original C++ System reference.
- Add the usual support route in the new Rust integration target only when the
  peer consumer is implemented. Its two unchanged support tests are separate
  new headless harness instantiations, outside feature counts/inherited pools.
- Append `examples/m59_multiple_diff_drives.sdf` as entry33 of the maintained
  finite tuple; preserve the entire old32 prefix. Validate exactly six sorted
  moving link names (base_link/left_wheel/right_wheel for each owner) as finite
  positions (`default::robot_left::{base_link,left_wheel,right_wheel}` and the
  corresponding `default::robot_right` paths) and `default::ground::body` at its
  real fixed pose; no fake static coordinates
  or solver-trajectory oracle. Update only the corresponding finite expectation
  32→33 and append-specific positive/negative fixtures/retention proof.
- Bind every Source/test/fixture/peer change and actual declaration route in the
  final integrated census after parent reconciliation. No current Source count,
  native build, runtime result, coverage, local qualification or acceptance is
  supplied by this design.
