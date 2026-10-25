# Native C++ request completion

The pinned native transport blocking Request implementation can race its own
response cleanup. `detail/Node.hh` waits for the reply then removes its handler.
`NodeShared.cc` obtains that handler, releases the shared mutex, notifies the
waiter, then reacquires the mutex to remove it. A successful waiter can remove
it first, causing `Error removing request handler` on stderr even when every
request value passed. The immutable installed header matches the raw pinned
41913b9 source. Native source/libraries are unchanged.

Our terminating C++ client now uses the public asynchronous Request callback
with a bounded future wait. The response thread is the sole handler remover.
The callback owns a shared promise and copies reply/result into it; a late
callback after timeout cannot reference a dead stack. Local callbacks may run
synchronously and are handled by the same future. An unanswered async handler
can remain until this short-lived test process exits; this helper is not a
production request API.

All eleven call sites retain their original request payloads, 7000 ms waits,
success/rejection and response-value checks. Removing only the helper/includes
and reversing call-site names reconstructs the original client byte-for-byte.
The existing native.rs empty-stderr assertion and all nine C++ scenarios remain
unchanged. There is no retry, sleep, ignored diagnostic or altered tolerance.
Author and independent private replay pass all nine scenarios with strict stderr.
The exact staged gate passes all 194 Rust and twelve tooling checks, all nine native
scenarios and unchanged 3162/3208 lines and 4914/5049 regions; every crate exceeds 95%.
The receipt is planning/reviews/native-client-requests.json. Normal-hook publication
and hosted qualification remain required.

The earlier failed shared-IMU gate is retained as failed evidence. Its new Rust
comparison passed independently, but that packet remains unpublished until this
harness fix and its own required checks pass. A passing rerun alone is not the
basis for this correction: the pinned source's two removers and the async path's
single remover establish the defect and the repair.
