// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bound the Euclidean distance of a measured planar wrench to the sum of four contact disks.
//! The primal disk projection supplies an admissible wrench, the dual support supplies a lower bound.
use super::{Case, ContactSample, Drive, G, Inertia, Result, point};
use rapier3d_f64::math::Vector;
use serde_json::{Value, json};
use std::{
    fs::File,
    io::{BufRead, BufReader, BufWriter, Write},
    path::Path,
};
struct Disk {
    x: f64,
    y: f64,
    cap: f64,
}
impl Disk {
    fn lift(&self, a: [f64; 2]) -> Vector {
        Vector::new(a[0], a[1], self.x * a[1] - self.y * a[0])
    }
    fn transpose(&self, v: Vector) -> [f64; 2] {
        [v.x - self.y * v.z, v.y + self.x * v.z]
    }
}
struct Bound {
    low: f64,
    high: f64,
    unit: Vector,
    primal: [[f64; 2]; 4],
}
fn project(w: Vector, disks: &[Disk]) -> Bound {
    let mut x = [[0.; 2]; 4];
    let mut y = x;
    let mut t = 1.0_f64;
    // Trace bounds the largest eigenvalue of the positive semidefinite Gram matrix.
    let lipschitz = disks
        .iter()
        .map(|d| 2. + d.x * d.x + d.y * d.y)
        .sum::<f64>();
    let wrench = |v: &[[f64; 2]; 4]| disks.iter().zip(v).map(|(d, &p)| d.lift(p)).sum::<Vector>();
    let mut tight = Bound {
        low: 0.,
        high: w.length(),
        unit: Vector::ZERO,
        primal: [[0.; 2]; 4],
    };
    for iter in 0..800 {
        let diff = wrench(&y) - w;
        let previous = x;
        let tn = (1. + (1. + 4. * t * t).sqrt()) * 0.5;
        for (i, d) in disks.iter().enumerate() {
            let dv = d.transpose(diff);
            let mut v = [y[i][0] - dv[0] / lipschitz, y[i][1] - dv[1] / lipschitz];
            let n = v[0].hypot(v[1]);
            let safe_cap = d.cap * (1. - 8. * f64::EPSILON);
            if n > safe_cap {
                v[0] *= safe_cap / n;
                v[1] *= safe_cap / n;
            }
            x[i] = v;
            let momentum = (t - 1.) / tn;
            y[i] = [
                v[0] + momentum * (v[0] - previous[i][0]),
                v[1] + momentum * (v[1] - previous[i][1]),
            ];
        }
        t = tn;
        if iter % 4 != 3 {
            continue;
        }
        let e = w - wrench(&x);
        let hi = e.length();
        if hi < tight.high {
            tight.high = hi;
            tight.primal = x;
        }
        if hi > 0. {
            let u = e / hi;
            let h = disks
                .iter()
                .map(|d| {
                    let z = d.transpose(u);
                    d.cap * z[0].hypot(z[1])
                })
                .sum::<f64>();
            let lo = (u.dot(w) - h).max(0.);
            if lo > tight.low {
                tight.low = lo;
                tight.unit = u;
            }
        }
        if tight.high - tight.low < 1e-9 {
            break;
        }
    }
    // Outward padding exceeds roundoff for the four-term f64 dot products at the recorded scale.
    let pad = 64.
        * f64::EPSILON
        * (1.
            + w.length()
            + disks
                .iter()
                .map(|d| d.cap * (1. + d.x.abs() + d.y.abs()))
                .sum::<f64>());
    tight.high += pad;
    tight.low = (tight.low - pad).max(0.);
    tight
}
fn sample(pts: &[ContactSample], wanted: Vector, scale: f64, mu: f64) -> Result<Bound> {
    let mut disks = vec![];
    for p in pts {
        if (p.normal - Vector::Z).length() > 1e-10 || p.impulse < 0. {
            return Err("planar disk fixture normal or load".into());
        }
        disks.push(Disk {
            x: p.lever.x / scale,
            y: p.lever.y / scale,
            cap: mu * p.impulse,
        });
    }
    Ok(project(wanted, &disks))
}
/// Return the same support-plane and feasible-disk bounds for a separately collected native sample.
pub(super) fn certificate(
    points: &[ContactSample],
    wanted: Vector,
    radius: f64,
    mu: f64,
) -> Result<Value> {
    let bound = sample(points, wanted, radius, mu)?;
    let pressure:Vec<_>=points.iter().map(|p|json!({"normal_ns":p.impulse,"lever_m":p.lever.to_array(),"normal_on_block":p.normal.to_array()})).collect();
    Ok(
        json!({"required":wanted.to_array(),"mu":mu,"radius_m":radius,"disks":pressure,
        "lower_ns":bound.low,"upper_ns":bound.high,"primal_tangents_ns":bound.primal,
        "dual_unit":bound.unit.to_array(),"mu_normal_ns":mu*points.iter().map(|p|p.impulse).sum::<f64>()}),
    )
}
pub fn run(source: &Path, directory: &Path) -> Result<()> {
    let mut schedule = vec![];
    for l in BufReader::new(File::open(source)?).lines() {
        let r: Value = serde_json::from_str(&l?)?;
        schedule.push(r["substeps"].as_u64().ok_or("native rate")?);
    }
    if schedule.len() != 120 {
        return Err("need full measured schedule".into());
    }
    let mut result = vec![];
    let mut certificates = BufWriter::new(File::create_new(directory.join("certificates.jsonl"))?);
    for drive in [Drive::Slide, Drive::Spin, Drive::Mixed, Drive::Reverse] {
        for pointwise in [false, true] {
            let c = Case {
                square: false,
                moving: true,
                smooth: false,
                coarse: false,
                drive,
            };
            let seeds = c.seeds();
            let inertia = Inertia::new(&seeds[1], &seeds[0]);
            let mut d = point::Direct::new(&seeds, pointwise);
            let mut max_low = 0.0_f64;
            let mut max_high = 0.0_f64;
            let mut max_gap = 0.0_f64;
            let mut bad = 0;
            let mut count = 0;
            let mut peak = Value::Null;
            let mut weighted = 0.;
            for (k, &rate) in schedule.iter().enumerate() {
                if k == 30 {
                    let (fx, tz) = drive.impulses();
                    d.kick([fx, 0., 0.], [0., 0., tz])
                }
                let dt = 0.01 / rate as f64;
                for sub in 0..rate {
                    let pre = d.state(1);
                    d.step(dt);
                    let post = d.state(1);
                    if k < 30 {
                        continue;
                    }
                    let pts = d.points();
                    if pts.len() != 4 {
                        continue;
                    }
                    let v = 0.5 * (Vector::from(pre.velocity) + Vector::from(post.velocity))
                        - inertia.table_v;
                    let w = 0.5
                        * (Vector::from(pre.angular_velocity)
                            + Vector::from(post.angular_velocity));
                    if v.x.hypot(v.y) + inertia.radius * w.z.abs() <= 0.15 {
                        continue;
                    }
                    let mut p = (Vector::from(post.velocity)
                        - Vector::from(pre.velocity)
                        - Vector::from(G) * dt)
                        * inertia.mass;
                    let mut l = inertia.momentum(post) - inertia.momentum(pre);
                    let mut total = 0.;
                    for point in &pts {
                        let j = point.normal * point.impulse;
                        p -= j;
                        l -= point.lever.cross(j);
                        total += point.impulse;
                    }
                    let cap = inertia.mu * total;
                    if cap < 1e-8 {
                        continue;
                    }
                    let wanted = Vector::new(p.x, p.y, l.z / inertia.radius);
                    let bound = sample(&pts, wanted, inertia.radius, inertia.mu)?;
                    // In this norm, one unit of yaw impulse/radius has the same unit as a linear impulse.
                    let low = bound.low / cap;
                    let high = bound.high / cap;
                    if high + 1e-9 < low {
                        return Err("dual lower bound above feasible primal upper bound".into());
                    }
                    max_gap = max_gap.max(high - low);
                    max_high = max_high.max(high);
                    if low > max_low {
                        max_low = low;
                        peak = json!({
                            "tick": k+1,
                            "sub": sub,
                            "required": wanted.to_array(),
                            "mu_normal_ns": cap,
                            "distance_lower_ns": bound.low,
                            "distance_upper_ns": bound.high,
                            "unit_dual": bound.unit.to_array()
                        });
                    }
                    let disks: Vec<_> = pts
                        .iter()
                        .map(|p| json!({"lever_m":p.lever.to_array(),"normal_ns":p.impulse}))
                        .collect();
                    writeln!(
                        certificates,
                        "{}",
                        json!({
                            "drive":format!("{drive:?}"),"pointwise":pointwise,"tick":k+1,"sub":sub,
                            "radius_m":inertia.radius,"mu":inertia.mu,"disks":disks,
                            "required":wanted.to_array(),"primal_tangents_ns":bound.primal,
                            "dual_unit":bound.unit.to_array(),"lower_ns":bound.low,"upper_ns":bound.high
                        })
                    )?;
                    count += 1;
                    if low > 0.01 {
                        bad += 1;
                        weighted += bound.low;
                    }
                }
            }
            result.push(json!({
                "drive": format!("{drive:?}"),
                "pointwise": pointwise,
                "sampled": count,
                "more_than_one_percent_beyond": bad,
                "maximum_certified_distance_fraction": max_low,
                "maximum_feasible_upper_fraction": max_high,
                "worst_certificate_interval_width_fraction": max_gap,
                "sum_certified_distance_over_threshold_ns": weighted,
                "peak": peak
            }));
        }
    }
    certificates.flush()?;
    serde_json::to_writer_pretty(File::create_new(directory.join("summary.json"))?, &result)?;
    Ok(())
}
