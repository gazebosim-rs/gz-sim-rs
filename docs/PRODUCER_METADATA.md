# Controlled producer integration

The workspace pins SDF 706e8fde, Transport 7ce88e5f and the same Messages 2cea810e
identity used by Transport. Fuel 3747086 remains unchanged. A single Messages
identity preserves sealed CanonicalMessage compatibility. Generated wire types,
canonical schemas and codecs are unchanged by the Messages update; it includes
the reviewed typed descriptor-default validation correction.

The Linux graph has one gz-msgs and one prost 0.14.4. There is one socket2=0.6.5;
Fuel's retained hyper-util 0.1.15 also uses socket2=0.5.10. Those different minor
versions are legitimate, not duplicate Messages identities. Fuel's crates.io
reflection library and Messages' reviewed vendored reflection remain separate.
Cargo's generated lock update also coalesces/prunes Windows-only dependencies;
this is not Windows execution evidence or a platform-support claim.

VisualMaterial owns parsed material data, including source, colors, shaders and
ordered script URIs. It performs no rendering, shader/script/image loading or
network requests. LinkProperties retains construction flags. Current construction
requires defaults; nondefault wind, disabled gravity, kinematic and self-collision
flags return typed Unsupported errors instead of being silently ignored. Editing
these metadata components does not reconfigure a running solver. Loaded reset
restores their original owned values after mutation or removal.

At this producer-adoption revision all joints remained unsupported. The later
bounded world-revolute consumer is documented in JOINTS.md. The producer's subsequent ContactSensor enum
change is not adopted. World physics selection, IMU behavior, existing lifecycle
and transport assertions remain required.

The author gate passes 184 Rust checks, twelve tooling checks and all nine actual
C++ scenarios: 2748/2783 production lines, 4329/4436 regions, every crate >95%
for each metric and 28/28 changed executable scene lines. Both engines exercise
metadata ownership/reset and nondefault flag rejection. No existing assertions,
coverage exclusions or thresholds changed. Published 67c1af4 passed independent review, the exact normal hook and hosted
consumer CI 40188844686; dependencies were independently qualified.
This prepares the selected simple-joint variant; it does not implement joints,
rendering, complete backend flag semantics or P2 acceptance.
