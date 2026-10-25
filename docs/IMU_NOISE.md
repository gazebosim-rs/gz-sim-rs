# Gaussian IMU noise

The implementation supports Gaussian and Gaussian-quantized noise on
all six IMU channels. It uses pinned rand_distr 0.6.0 and rand_chacha 0.10.0;
random generators and normal samplers are reused libraries. Scalar state adds
white noise and signed fixed bias, evolves correlated bias using the native
exact discrete equation, then rounds half away from zero when precision exceeds
1e-6. Native enables that precision rule for either Gaussian kind.

Nonpositive deviations disable their random contribution. Tiny positive normal
deviations at or below f64 epsilon return the mean, matching native Rand.
Nonfinite parameters/inputs, negative elapsed time and nonfinite derived results
are explicit errors. Failed application retains the channel's preceding bias;
consumed randomness is not rewound. These checks do not reproduce upstream
undefined floating-point behavior. Negative precision disables quantization;
native's diagnostic for that setting is not yet reproduced by the loader.

## Ownership, timing and reset

Each noisy sensor owns a ChaCha8 stream keyed by a 32-byte world seed and its
stable public sensor identity. Sensors and worlds do not share mutable RNG state.
`load_world_seeded` chooses the seed and optional timestep; existing loader entry
points use an all-zero seed. Loaded reset reconstructs the initial stream and
bias, so identical commands repeat samples. Pauses and skipped acquisitions do
not consume noise. Rust streams intentionally differ from native's process-global
MT19937 sequence; equal numeric seeds do not imply identical C++/Rust samples.
Channel initialization, application and reset use native acceleration X/Y/Z
then angular velocity X/Y/Z order.
The pinned dependency versions define Rust replay, not a permanent cross-version
wire promise. This follows the planned per-sensor ownership policy; full native
seed/control API compatibility remains open.

Noise follows gravity subtraction. Physics derivatives still update on every
physics step. Correlated bias instead uses elapsed acquisition time, with the
native integer-nanoseconds to duration<float> conversion; the first or backward
acquisition uses zero. A 10 Hz sensor therefore uses the 99 ms interval between
its first 1 ms and next 100 ms samples, not the physics 1 ms interval.

Covariance uses native diagonal stddev squared, rounded through f32. Cross terms
and orientation covariance remain zero. The loader rejects unrepresentable
covariance. Noise-free sensors allocate no random stream. A nonfinite result
reports a typed terminal step fault rather than publishing a nonfinite sample;
already committed physics effects are not rolled back.

## Evidence and remaining work

`planning/reference/noise.json` records six seeded native scalar controls and
seven unchanged native NoiseTest cases. Original negative cases intentionally
emit diagnostics, retained in the raw logs. Native source, compiler, libraries,
binary and runner are hashed. The reference proves scalar noise behavior;
acquisition-dt and covariance semantics are additionally source-backed. It is not
a full noisy native Server stream or complete immutable P0 environment.

Rust tests retain the native 100-sample, five-sigma mean/variance checks and
quantization examples, and add deterministic stream independence/clone replay,
innovation moments, numeric boundaries, both physics engines, slow acquisition,
pause/reset, covariance and terminal overflow. Two additional C++ client modes
exercise nonzero means, quantization, stochastic output, float covariance and
exact first-sample replay across reset; all nine older modes remain present.

The old Rust-only blanket Gaussian rejection is obsolete. Its unchanged error
and existing-world atomicity assertions now use finite stddev1e30, whose squared
covariance cannot fit the native float wire field. Its initial failing output is
retained. The exact old Gaussian XML payload remains in a positive regression comparing
100 complete samples against noise-free on both engines. Other positive cases
cover nonzero noise.
Independent review approved this disposition, seed policy and new modes after
private native, model, scene and C++ replays. Four separate precision,
acquisition-dt, covariance and reset mutants failed their intended assertions.
The receipt is `planning/reviews/imu-noise.json`. No complete original test port
is credited.

The exact staged gate passed 221 Rust tests, sixteen tooling checks and all
eleven actual C++ scenarios. Workspace coverage is 3436/3472 production lines
and 5255/5371 regions; every crate exceeds 95% on both. Changed executable
coverage is sensors79/79, scene133/134 and server15/15. Normal protected hook,
publication/hosted verification and Gaussian-workload performance qualification
remain separate requirements. Activation/subscriber interest, runtime
registration, trigger modes, custom noise callbacks, image noise, other sensor
families and the broad SENSOR-01/P2 contracts remain open.

Reproduce native evidence with `python3 scripts/probe-noise.py --help`; supply
pinned sources, the existing sensors build and all runtime dependency directories.
