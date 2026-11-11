<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Cancelled coverage run: containment and retained diagnostics

Exact review CI 42845720294 attempt 1 at 9a40768d ended cancelled on November 10,
after about six hours. Its setup completed; headless coverage did not finish.
The last visible ordinary log output was completed doc-tests at 15:45:46 UTC.
The original jobs ZIP and the sole partial artifact are retained. No finalized
headless test-output log exists. The exact hanging test and cancellation cause
are unknown; these Source findings do not establish a production deadlock.

The verifier previously buffered coverage command output in memory until exit,
with no command deadline. The new helper drains both channels concurrently into
exclusive files throughout execution, with a 64 MiB cap per channel and a one-hour
command deadline. It also covers renderer build-only preparation. Timeout,
output overflow or interruption retires the owned process group and reaps the
direct child, then fails. It never converts those outcomes into a pass. Caps apply
to transcripts, not compiler objects. Successful stdout/stderr and the established
concatenated test-output log remain unchanged. Partial channels are retained by
staged evidence even when a final log or coverage report is absent.

Controlled stalled-callback tests now release their gate on both normal exit and
unwind before owned callback/session cleanup. Existing callback, CPU lease,
noise, cadence and timeout assertions remain. New Source controls cover early
error return, intentional assertion unwind, release-before-owner cleanup,
idempotence and late callback entry. The native force harness separately bounds
isolated cases and peer retirement: receiving DONE is not a process-exit proof.
No production callback or Zenoh implementation is changed.

The repair is Source-only until the complete implementation, regression source,
documentation and independent review are finished. Then run one consolidated
required normal hook, retaining every assertion and strict workspace/per-crate/
actual-parent coverage gate. Diagnose and batch any actual failures before affected
reruns. Exact hosted success and independent local/official DATA remain required
before publication or M43/M44 acceptance. No retry was released by writing this
repair. Existing local/native/package/install/browser evidence stays scoped to
its original frozen candidate; reviewed unchanged-source reuse must be explicit.
M45 execution remains held for its actual accepted parent.

## First consolidated attempt and batched correction

The first normal attempt on tree c03c214f failed at Clippy after 172.118 seconds,
before tests. Six direct unsafe blocks violated the unchanged workspace forbid
policy, producing six corresponding documentation errors; the callback helper
also triggered duplicate-module inclusion. The original failed tree, packet and
log remain retained. No failed attempt is relabeled as qualification.

The correction uses safe APIs from the already locked nix 0.31.3. Its fs,
process and signal features are enabled only for Unix dev dependencies; the
existing runtime hostname/user selection and Cargo.lock stay unchanged. No
unsafe allowance or lint suppression is added. The callback guard is registered
once under the existing private Server test module, with crate-private aliases
from its nine test callers. No production Rust file needs to change. The
complete correction must pass independent Source review before one affected
normal-hook retry; hosted qualification and final acceptance remain pending.

The second normal attempt on tree f78137e8 completed the lint stages, then ran
401 Python tests with one failure in the new controller-signal regression.
The handler used InterruptedError, which Python's selector catches as its normal
EINTR retry path. The interruption was recorded but cleanup was not entered.
The correction uses a dedicated non-EINTR exception and preserves every timing,
retirement and partial-output assertion. The failed run and tree remain retained;
this is a newly observed verifier defect, not proof of the original hosted cause.
