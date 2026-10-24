# MSG-01 acceptance

Accepted by the coordinator on 2026-10-24 at gz-msgs-rs revision
[2f744840c4e5bb986bcc2237a0ba3dae2f66c19b](https://github.com/gazebosim-rs/gz-msgs-rs/tree/2f744840c4e5bb986bcc2237a0ba3dae2f66c19b).
This closes the message foundation packet defined in IMPLEMENTATION_TASKS.md.
MSG-02 and the overall P0/P1 phase gates remain open.

| Required result | Evidence |
| --- | --- |
| Canonical schema inventory | All 216 schema blobs and SHA256 entries independently compared to upstream commit 89c0c3d1474f545cb889335daf930bfdab6ab852; no missing or changed schema. |
| Runtime and reflection evaluation | Maintained prost/prost-reflect selected after recorded probes. Two narrow signed-zero fixes have reviewed provenance and measured owned changes. |
| Generated types and descriptors | Canonical types, nested descriptors, standard prost traits and reflection exposed without simulator or Bevy dependencies. |
| Basic conversions and boundary contract | Signed time, scalar and array constructors, Clock/Pose/IMU values, and explicit typed-versus-lossless forwarding behavior documented and tested. |
| Mixed-language conformance corpus | Required C++ peer tests cover both directions, full canonical top-level descriptor equality, unknown fields/enums, malformed input, bytes, presence, floats and duplicate fields. |
| Quality and consumer checks | Exact published revision passed hosted verification: 43 Rust tests, five snapshot regressions, clean downstream consumer, native peers, schema/vendor hashes, and 502/502 owned lines with 731/740 regions. |

The coordinator reviewed the immutable API, evidence, native test bodies and
verification entry point, independently checked all schema bytes against upstream,
and verified the hosted run's exact commit and successful conclusion. Earlier
runtime, conversion, dependency patch and interoperability reviews remain recorded
in the producer's evidence and test ledger.

- [API contract](https://github.com/gazebosim-rs/gz-msgs-rs/blob/2f744840c4e5bb986bcc2237a0ba3dae2f66c19b/docs/API.md)
- [Implementation and review evidence](https://github.com/gazebosim-rs/gz-msgs-rs/blob/2f744840c4e5bb986bcc2237a0ba3dae2f66c19b/docs/EVIDENCE.md)
- [Hosted verification](https://github.com/gazebosim-rs/gz-msgs-rs/actions/runs/40121346194)

The message boundary of REF-01/API-01 is sufficient for this packet. Completing
this producer boundary does not imply complete reference environments or contracts
for other repositories. The full 83-case retained ledger, remaining domain
conversions and cloud iterator behavior, external factory/tooling completeness,
packaging, historical qualification and performance remain MSG-02 obligations.
No performance improvement or complete message rewrite is claimed here.
