// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Active constraints for the first transition from sliding to a rolling edge.
use super::*;
use rapier3d_f64::na::{DMatrix, DVector};

#[derive(Clone, Copy, Debug, PartialEq)]
enum Mode {
    Slide,
    Pivot(usize),
    Edge,
}

fn tangent(v: Vector) -> Vector {
    Vector::new(v.x, v.y, 0.)
}
fn slip(b: Body, ids: [usize; 2]) -> [Vector; 2] {
    ids.map(|i| tangent(b.point_velocity(b.arm(i))))
}
fn free_accel(b: Body, r: Vector) -> Vector {
    let w = b.inv_i(b.momentum);
    GRAVITY + b.inv_i(-w.cross(b.momentum)).cross(r) + w.cross(w.cross(r))
}
fn response(b: Body, arms: [Vector; 2], f: [Vector; 2], r: Vector) -> Vector {
    (f[0] + f[1]) / MASS + b.inv_i(arms[0].cross(f[0]) + arms[1].cross(f[1])).cross(r)
}
/// Each row constrains one component of point acceleration. Each column
/// describes a force direction, with the full Coulomb shear for a sliding point.
fn linear(
    b: Body,
    arms: [Vector; 2],
    rows: &[(usize, Vector)],
    cols: &[(usize, Vector)],
) -> [Vector; 2] {
    let k = DMatrix::from_fn(rows.len(), cols.len(), |i, j| {
        let (c, d) = cols[j];
        let mut f = [Vector::ZERO; 2];
        f[c] = d;
        rows[i].1.dot(response(b, arms, f, arms[rows[i].0]))
    });
    let rhs = DVector::from_iterator(
        rows.len(),
        rows.iter()
            .map(|&(i, axis)| -axis.dot(free_accel(b, arms[i]))),
    );
    let coeff = k.lu().solve(&rhs).expect("independent edge constraints");
    let mut force = [Vector::ZERO; 2];
    for (&(i, d), x) in cols.iter().zip(coeff.iter()) {
        force[i] += d * *x;
    }
    force
}
fn cone(f: Vector) -> f64 {
    MU * f.z - tangent(f).length()
}
fn contact_forces(b: Body, ids: [usize; 2], mode: Mode) -> [Vector; 2] {
    let arms = ids.map(|i| b.arm(i));
    match mode {
        Mode::Slide => b.forces(ids, 2),
        Mode::Pivot(i) => {
            let other = 1 - i;
            let rows = [
                (i, Vector::X),
                (i, Vector::Y),
                (i, Vector::Z),
                (other, Vector::Z),
            ];
            let mut cols = rows;
            cols[3].1 = Vector::Z - MU * slip(b, ids)[other].normalize();
            linear(b, arms, &rows, &cols)
        }
        Mode::Edge => {
            let axis = (arms[1] - arms[0]).normalize();
            let t = Vector::Z.cross(axis).normalize();
            let p = axis.cross(t);
            let rows = [(0, axis), (0, t), (0, p), (1, t), (1, p)];
            let mut f = linear(b, arms, &rows, &rows);
            // Opposite forces along the edge have zero net wrench. Intersect both
            // Coulomb disks with this one-dimensional freedom to test static support.
            let interval = |f: Vector, e: Vector| {
                let a = tangent(e).length_squared() - MU * MU * e.z * e.z;
                let b = tangent(f).dot(tangent(e)) - MU * MU * f.z * e.z;
                let c = tangent(f).length_squared() - MU * MU * f.z * f.z;
                let root = (b * b - a * c).sqrt();
                [(-b - root) / a, (-b + root) / a]
            };
            let lo = interval(f[0], axis);
            let hi = interval(f[1], -axis);
            let lower = lo[0].max(hi[0]);
            let upper = lo[1].min(hi[1]);
            assert!(lo[0].is_finite() && hi[0].is_finite() && lower <= upper);
            let x = 0.5 * (lower + upper);
            f[0] += x * axis;
            f[1] -= x * axis;
            assert!(f.iter().all(|f| f.z > 0. && cone(*f) > 0.));
            f
        }
    }
}
fn advance(b: Body, ids: [usize; 2], m: Mode, dt: f64) -> (Body, [Vector; 2]) {
    let mid = b.apply(
        b,
        ids.map(|i| b.arm(i)),
        contact_forces(b, ids, m),
        0.5 * dt,
    );
    let f = contact_forces(mid, ids, m);
    (
        b.apply(mid, ids.map(|i| mid.arm(i)), f, dt),
        f.map(|v| v * dt),
    )
}
#[derive(Clone, Copy)]
struct Resolution {
    steps: usize,
    speed_tolerance: f64,
    fractional_deceleration: f64,
}

fn run_one(
    samples: &reload::Samples,
    ids: [usize; 2],
    anchor: usize,
    end: usize,
    resolution: Resolution,
    extra_x: f64,
) -> Value {
    let Resolution {
        steps: n,
        speed_tolerance: tol,
        fractional_deceleration,
    } = resolution;
    let (mut b, start) = if anchor == 2194 {
        (super::integrate(samples, ids, n, anchor, 2215, 1).0, 2215)
    } else {
        (Body::from_state(samples[&anchor].0), anchor)
    };
    b.velocity.x += extra_x;
    let mut mode = Mode::Slide;
    let mut events = Vec::new();
    let mut rows = Vec::new();
    let mut device_boundaries = Vec::new();
    let mut max_accel_error = 0_f64;
    let mut max_hinge_error = 0_f64;
    let mut max_sliding_speed_error = 0_f64;
    let mut max_sliding_impulse_error = 0_f64;
    let mut max_holding_normal_error = 0_f64;
    let mut max_origin_error = 0_f64;
    let mut sliding_samples = 0;
    let mut max_yaw_error = 0_f64;
    for tick in start + 1..=end {
        let mut t = 0.;
        let mut loads = [Vector::ZERO; 2];
        while t < DT - 1e-16 {
            let speeds = slip(b, ids).map(|v| v.length());
            let before = mode;
            if speeds.iter().all(|s| *s <= tol * 1.1) {
                mode = Mode::Edge;
            } else if mode == Mode::Slide {
                for (i, speed) in speeds.iter().enumerate() {
                    if *speed <= tol && cone(contact_forces(b, ids, Mode::Pivot(i))[i]) >= 0. {
                        mode = Mode::Pivot(i);
                        break;
                    }
                }
            }
            let arms = ids.map(|i| b.arm(i));
            let f = contact_forces(b, ids, mode);
            let accel = arms.map(|r| free_accel(b, r) + response(b, arms, f, r));
            if before != mode {
                events.push(json!({
                    "tick": tick, "offset_us": t*1e6,
                    "sticking_vertices": match mode { Mode::Pivot(i)=>vec![ids[i]], Mode::Edge=>ids.to_vec(), Mode::Slide=>unreachable!() },
                    "slip_m_s": speeds, "forces_n": f.map(|r|r.to_array()), "cone_reserve_n": f.map(cone)
                }));
            }
            let mut dt = (DT - t).min(DT / n as f64);
            for i in 0..2 {
                assert!(f[i].z > 0.);
                max_accel_error = max_accel_error.max(accel[i].z.abs());
                if mode == Mode::Slide || mode == Mode::Pivot(1 - i) {
                    // Limit the change in slip during a step close to the singular
                    // zero-speed boundary, independently of the public clock.
                    dt = dt.min(
                        fractional_deceleration * speeds[i] / tangent(accel[i]).length().max(1e-8),
                    );
                } else {
                    max_accel_error = max_accel_error.max(accel[i].length());
                    assert!(cone(f[i]) > 0.);
                }
            }
            if mode == Mode::Edge {
                // A hard, stationary edge is also a physical hinge. Gravity alone
                // supplies torque along its axis; internal endpoint forces cancel.
                let e = (arms[1] - arms[0]).normalize();
                let axis_inertia = e.dot(b.q * (MOMENTS * (b.q.inverse() * e)))
                    + MASS * arms[0].cross(e).length_squared();
                let hinge_alpha = (-arms[0]).cross(MASS * GRAVITY).dot(e) / axis_inertia;
                let torque = arms[0].cross(f[0]) + arms[1].cross(f[1]);
                let alpha = b.inv_i(torque - b.inv_i(b.momentum).cross(b.momentum));
                max_hinge_error = max_hinge_error.max((alpha.dot(e) - hinge_alpha).abs());
            }
            // Sample the independently integrated solution at the same eight
            // device timestamps on the final matched and higher-speed controls.
            let boundary = (device_boundaries.len() + 1) as f64 * DT / 8.;
            let record = anchor == 2240 && tick == 2241 && boundary <= DT;
            if record {
                dt = dt.min(boundary - t);
            }
            assert!(dt > 1e-15);
            let (next, j) = advance(b, ids, mode, dt);
            b = next;
            for i in 0..2 {
                loads[i] += j[i];
            }
            t += dt;
            if record && (t - boundary).abs() < 1e-15 {
                device_boundaries.push(json!({
                    "end_us":t*1e6, "mode":format!("{mode:?}"),
                    "slip_m_s":slip(b,ids).map(|u|u.length()),
                    "yaw_deg":reload::angle(b.q)
                }));
            }
        }
        let dev = Body::from_state(samples[&tick].0);
        let reference = slip(b, ids).map(|v| v.length());
        let gpu = slip(dev, ids).map(|v| v.length());
        if extra_x == 0. {
            max_origin_error = max_origin_error
                .max((b.center - b.q * OFFSET).distance(samples[&tick].0.position.into()));
            if mode == Mode::Slide {
                sliding_samples += 1;
                for i in 0..2 {
                    max_sliding_impulse_error = max_sliding_impulse_error
                        .max(loads[i].distance(samples[&tick].1.vertices[ids[i]].into()));
                    max_sliding_speed_error =
                        max_sliding_speed_error.max((gpu[i] - reference[i]).abs());
                }
            }
            // The first stopping tick mixes kinetic and static force. Only compare
            // a full holding interval after both points have stopped.
            if tick >= 2242 {
                for i in 0..2 {
                    max_holding_normal_error = max_holding_normal_error
                        .max((loads[i].z - samples[&tick].1.vertices[ids[i]][2]).abs());
                }
            }
            max_yaw_error = max_yaw_error.max((reload::angle(dev.q) - reload::angle(b.q)).abs());
        }
        rows.push(json!({
            "tick": tick, "mode": format!("{mode:?}"), "slip_m_s": reference,
            "yaw_deg": reload::angle(b.q), "omega_rad_s": b.inv_i(b.momentum).to_array(),
            "origin_m": (b.center-b.q*OFFSET).to_array(), "point_impulses_ns": loads.map(|v|v.to_array()),
            "normal_n": contact_forces(b,ids,mode).map(|f|f.z)
        }));
    }
    if extra_x == 0. {
        assert!(
            events.len() == 2
                && events[0]["sticking_vertices"] == json!([ids[0]])
                && events[1]["sticking_vertices"] == json!(ids)
        );
        assert!(max_sliding_speed_error < 50e-6 && max_yaw_error < 0.009);
        assert!(
            max_origin_error < 15e-6
                && max_sliding_impulse_error < 0.2e-6
                && max_holding_normal_error < 0.08e-6
        );
    } else {
        assert!(events.is_empty() && mode == Mode::Slide);
    }
    assert!(max_accel_error < 1e-8 && max_hinge_error < 1e-8);
    json!({
        "anchor":anchor, "steps_per_ms":n, "capture_speed_m_s":tol, "deceleration_fraction": fractional_deceleration,
        "extra_x_m_s":extra_x, "events":events, "rows":rows, "device_boundaries":device_boundaries,
        "max_constrained_accel_residual_m_s2":max_accel_error,"max_hinge_accel_error_rad_s2":max_hinge_error,
        "matched_gpu_error":(extra_x==0.).then_some(json!({
            "sliding_public_ticks":sliding_samples,
            "sliding_speed_m_s":(sliding_samples>0).then_some(max_sliding_speed_error),
            "sliding_corner_impulse_ns":(sliding_samples>0).then_some(max_sliding_impulse_error),
            "holding_normal_impulse_ns":max_holding_normal_error,
            "origin_m":max_origin_error,
            "yaw_deg":max_yaw_error
        }))
    })
}
pub(in super::super) fn run(samples: &reload::Samples, ids: [usize; 2]) -> Value {
    let mut runs = Vec::new();
    for anchor in [2194, 2215, 2240] {
        for (steps, speed_tolerance) in [(16, 1e-6), (64, 1e-7), (256, 1e-8)] {
            runs.push(run_one(
                samples,
                ids,
                anchor,
                2245,
                Resolution {
                    steps,
                    speed_tolerance,
                    fractional_deceleration: 0.09,
                },
                0.,
            ));
        }
    }
    for (steps, speed_tolerance, fractional_deceleration) in [
        (64, 1e-10, 0.09),
        (256, 1e-10, 0.09),
        (1024, 1e-10, 0.045),
        (1024, 1e-11, 0.02),
    ] {
        runs.push(run_one(
            samples,
            ids,
            2240,
            2245,
            Resolution {
                steps,
                speed_tolerance,
                fractional_deceleration,
            },
            0.,
        ));
    }
    let negative = run_one(
        samples,
        ids,
        2240,
        2241,
        Resolution {
            steps: 256,
            speed_tolerance: 1e-10,
            fractional_deceleration: 0.045,
        },
        0.02,
    );
    json!({"corner_ids":ids,"runs":runs,"added_velocity_control":negative})
}
