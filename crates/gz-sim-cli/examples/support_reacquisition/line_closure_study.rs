// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Determine whether ruling out simultaneous normal closure certifies a new
//! single carrying endpoint beyond the initial velocity-only pressure bound.
use super::*;
use gz_physics::{EdgeMobility, FiniteArcResult, LineClosure, PointImpact, exclude_closed_line};
pub(super) fn response(e: FiniteArcResult, s: FiniteContactSegment) -> Result<Value> {
    let [p, q] = s.moving_points;
    let n = s.normal;
    let x = (q - p).reject_from_normalized(n).normalize();
    let basis = [x, n.cross(x), n];
    let center = e.center;
    let arms = [p - center, q - center];
    let frame = |v: DVec3| basis.map(|axis| axis.dot(v));
    let world = |v: [f64; 3]| basis[0] * v[0] + basis[1] * v[1] + basis[2] * v[2];
    let w: EdgeMobility = std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            std::array::from_fn(|a| {
                std::array::from_fn(|b| {
                    basis[a].dot(
                        basis[b] / MATERIAL.mass
                            + arms[j].cross(basis[b]).cross(arms[i]) / MATERIAL.inertia,
                    )
                })
            })
        })
    });
    let before = arms.map(|r| e.velocity + e.angular_velocity.cross(r));
    let v = before.map(frame);
    let mu = MATERIAL.friction;
    let mut candidates = vec![];
    for i in 0..2 {
        let other = 1 - i;
        let own = DVec3::from(w[i][i][2]);
        let cross = DVec3::from(w[other][i][2]);
        let minimum = own.z - mu * own.truncate().length();
        let maximum = (cross.z + mu * cross.truncate().length()).max(0.);
        let tol = 1e-12 * (1. + before[i].length() + before[other].length());
        // A sole reaction there cannot close its own normal twice. Bound
        // its magnitude even for a barely closing or zero incoming velocity.
        if minimum > 0. && v[other][2] + (-v[i][2]).max(0.) / minimum * maximum < -tol {
            candidates.push(json!({"status":"cannot_stop_other"}));
            continue;
        }
        if v[i][2] >= -tol {
            candidates.push(json!({"status":"unresolved_near_zero"}));
            continue;
        }
        let law = PointImpact {
            inverse_mass: w[i][i],
            relative_velocity: v[i],
            friction: mu,
        };
        let Ok(h) = law.solve() else {
            candidates.push(json!({"status":"unresolved_point"}));
            continue;
        };
        let j = world(h.impulse);
        let post = before.map(|x| x);
        let dv = j / MATERIAL.mass;
        let dw = arms[i].cross(j) / MATERIAL.inertia;
        let speeds = [0, 1].map(|k| post[k] + dv + dw.cross(arms[k]));
        let opposite = speeds[other].dot(n);
        candidates.push(json!({"status":if opposite>tol{"opens_other"}else if opposite < -tol{"penetrates_other"}else{"marginal"},
            "j_ns":j.to_array(),"normal_ns":j.dot(n),"opposite_m_s":opposite,
            "post_v":(e.velocity+dv).to_array(),"post_w":(e.angular_velocity+dw).to_array(),"sticking":h.sticking,"work_loss_j":h.dissipated_energy}));
    }
    let basic = s.single_loaded_endpoint(e.center, e.velocity, e.angular_velocity, MATERIAL)?;
    let mut decisions = vec![];
    for budget in [1, 32, 512, 8192] {
        let h = exclude_closed_line(w, v, mu, budget)?;
        decisions.push(
            json!({"budget":budget,"visited":h.visited,"excluded":h.status==LineClosure::Excluded}),
        );
        if h.status == LineClosure::Excluded {
            break;
        }
    }
    Ok(
        json!({"pressure_bound":basic.map(|h|h.loaded_end),"candidates":candidates,"closure":decisions}),
    )
}
pub(crate) fn write(dir: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(dir, first, count, "continuous-line-closure-v1", |f| {
        let (cuts, disjoint) = survey::cuts(f)?;
        let mut trials = vec![];
        for (i, cut) in cuts.into_iter().enumerate() {
            if let Some(c) = cut {
                let (flight, defect) = kinematics(f, &c)?;
                if defect > 1e-8 {
                    return Err("invalid incoming twist".into());
                }
                let (mut r, hit) = locate(flight, true)?;
                if let Some((end, s)) = hit {
                    r["pressure"] = response(end, s)?
                }
                trials.push(json!({"stroke":i+1,"cut":snapshot(&c),"response":r}));
            }
        }
        Ok(json!({"fixture":f,"disjoint":disjoint,"trials":trials}))
    })
}
