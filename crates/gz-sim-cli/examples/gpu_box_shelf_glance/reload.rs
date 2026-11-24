// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Matched outgoing states after the corrected shelf strike.
use super::*;
use serde_json::json;
use std::path::Path;

#[derive(Deserialize)]
struct Trace {
    gpu: Vec<(usize, State)>,
    impulses: Vec<SensorLoad>,
}
#[derive(Deserialize)]
pub(super) struct SensorLoad {
    pub(super) vertices: [[f64; 3]; 8],
}
pub(super) type Samples<'a> = BTreeMap<usize, (&'a State, &'a SensorLoad)>;

pub(super) fn angle(q: Rotation) -> f64 {
    let x = q * Vector::X;
    x.y.atan2(x.x).to_degrees()
}
fn sweep(seed: &State, n: usize, tick: usize, end: usize) -> Value {
    let mut native = native(seed, n, false);
    let mut samples = Vec::new();
    for t in tick + 1..=end {
        let mut audit = corners::Witnesses::default();
        let before = native.world.bodies[native.body].linvel();
        let momentum = |b: &RigidBody| {
            let q = *b.rotation();
            q * (MOMENTS * (q.inverse() * b.angvel()))
        };
        let old_l = momentum(&native.world.bodies[native.body]);
        let mut angular_departure = [0.; 8];
        for sub in 0..n {
            native.world.step();
            let b = &native.world.bodies[native.body];
            native.contacts(|local, arm, normal, sign, data| {
                let (i, j) = audit.record(sub, local, arm, normal, sign, data);
                let u = b.linvel() + b.angvel().cross(arm);
                let shear = Vector::new(j.x, j.y, 0.);
                let slip = Vector::new(u.x, u.y, 0.);
                if shear.length() > 1e-14 && slip.length() > 1e-8 {
                    angular_departure[i] += j.z
                        * (-slip.normalize().dot(shear.normalize()))
                            .clamp(-1., 1.)
                            .acos()
                            .to_degrees();
                }
            });
        }
        let b = &native.world.bodies[native.body];
        let j = audit.impulses.iter().copied().sum::<Vector>();
        let a = audit.torques.iter().copied().sum::<Vector>();
        let jl = MASS * (b.linvel() - before + 9.81 * DT * Vector::Z);
        let al = momentum(b) - old_l;
        assert!(j.distance(jl) < 1e-11 && a.distance(al) < 5e-8);
        let departure = std::array::from_fn::<_, 8, _>(|i| {
            (audit.impulses[i].z > 1e-14).then(|| angular_departure[i] / audit.impulses[i].z)
        });
        samples.push(json!({
            "tick": t,
            "yaw_deg": angle(*b.rotation()),
            "omega_rad_s": b.angvel().to_array(),
            "origin_m": b.translation().to_array(),
            "corners_ns": audit.impulses.map(|j| j.to_array()),
            "angular_nms": a.to_array(),
            "first_substep": audit.first_loaded,
            "load_weighted_departure_deg": departure,
            "linear_balance_ns": j.distance(jl),
            "angular_balance_nms": a.distance(al)
        }));
    }
    json!({"steps_per_ms":n,"anchor":tick,"rows":samples})
}
pub(super) fn run() -> Result<(), Box<dyn std::error::Error>> {
    let src = std::env::args()
        .nth(2)
        .ok_or("expected --reload SDF_TRACE_DIRECTORY")?;
    let dir = Path::new(&src);
    let mut out = BTreeMap::new();
    for name in ["oblique_plus", "oblique_minus"] {
        let data: Trace =
            serde_json::from_reader(std::fs::File::open(dir.join(format!("{name}-trace.json")))?)?;
        if data.gpu.len() != data.impulses.len() {
            return Err("GPU samples and point loads have unequal lengths".into());
        }
        let observed = data
            .gpu
            .iter()
            .zip(&data.impulses)
            .map(|((tick, state), j)| (*tick, (state, j)))
            .collect::<Samples>();
        if observed.len() != data.gpu.len() || !(2194..=2215).all(|t| observed.contains_key(&t)) {
            return Err("expected unique GPU samples covering 2194 through 2215 ms".into());
        }
        let mut trials = Vec::new();
        let mut sliding = Vec::new();
        let ids = if name == "oblique_plus" {
            [1, 0]
        } else {
            [3, 2]
        };
        for (anchor, end, resolutions) in [
            (2194, 2245, &[32, 128, 512, 2048][..]),
            (2204, 2206, &[128, 512, 2048, 8192, 32768][..]),
            (2205, 2215, &[128, 512, 2048][..]),
        ] {
            let (seed, _) = observed[&anchor];
            for n in [8, 32, 128, 512] {
                sliding.push(sliding::integrate(
                    &observed,
                    ids,
                    n,
                    anchor,
                    2215,
                    if anchor == 2205 { 2 } else { 1 },
                ));
            }
            for &n in resolutions {
                trials.push(sweep(seed, n, anchor, end));
            }
        }
        out.insert(name, json!({"rapier":trials,"sliding":sliding}));
    }
    println!("{}", serde_json::to_string(&out)?);
    Ok(())
}
