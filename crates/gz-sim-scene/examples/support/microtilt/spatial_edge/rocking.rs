// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Follow a matched frictionless bearing to its next geometric event.
use super::*;
#[path = "rocking/mechanics.rs"]
mod arc;
#[path = "rocking/native.rs"]
mod native;
#[path = "rocking/replay.rs"]
mod replay;
#[path = "rocking/transaction.rs"]
pub(in super::super::super::super) mod transaction;
#[path = "rocking/two_bearing.rs"]
pub(in super::super::super::super) mod two_bearing;
#[path="rocking/face.rs"]
pub(in super::super::super::super) mod face;
use arc::{Arc, Arrival, End, Jump, Motion, Shape, Sites, State};

#[derive(Clone, Copy)]
struct Case {
    active: usize,
    pitch: f64,
    yaw: f64,
    opening: f64,
    pitch_rate: f64,
}
fn fixture(c: Case) -> (Input, Arc, usize) {
    let mut i = make_opening(Setup {
        active: c.active,
        sticking: false,
        mu: 0.,
        yaw: c.yaw,
        pitch: c.pitch,
        opening: c.opening,
    });
    let a = i.indices[c.active];
    let released = i.indices[1 - c.active];
    let q = i.seed.state.q;
    let r = i.edge.arm[c.active];
    let w = DQuat::from_rotation_z(c.yaw)
        * DVec3::new(
            (if c.active == 0 { 1. } else { -1. }) * c.opening / 0.2,
            c.pitch_rate,
            -0.44,
        );
    let v = DVec3::new(0.05, -0.03, -w.cross(r).z);
    i.after_v = v;
    i.after_w = w;
    i.edge.velocity = v - i.desired[c.active] / i.edge.mass;
    i.edge.spin = w - i.edge.inertia.inverse() * r.cross(i.desired[c.active]);
    i.seed.state.v = i.edge.velocity;
    i.seed.state.w = i.edge.spin;
    let corners = local_arms();
    let vertices = std::array::from_fn(|k| {
        let top = (k % 8) / 4;
        let id = k % 4;
        if k < 8 {
            corners[id] + DVec3::Z * (0.12 * top as f64)
        } else {
            DVec3::new(
                if id & 2 == 0 { -0.05 } else { 0.05 },
                if id & 1 == 0 { -0.06 } else { 0.06 },
                if top == 0 { 0.01 } else { 0.13 },
            )
        }
    });
    let shape = Shape {
        mass: i.edge.mass,
        inertia: i.edge.inertia,
        vertices,
        gravity: DVec3::new(0., 0., -9.81),
        pad_half: [0.41; 2],
    };
    let start = State {
        motion: Motion { q, w },
        c: i.seed.state.c,
        v,
    };
    (
        i,
        Arc {
            shape,
            start,
            sites: Sites::One(a),
        },
        released,
    )
}
fn planar_limit() -> Result<Value> {
    let pitch = 0.18;
    let (_, base, _) = fixture(Case {
        active: 0,
        pitch,
        yaw: 0.,
        opening: 0.13,
        pitch_rate: -1.2,
    });
    // Point moving in one vertical plane. Its tangential coordinate is free.
    // Eliminate the normal constraint and integrate dt/dtheta by energy.
    let r = DVec3::new(0.18, 0., -0.13);
    let mut sh = base.shape;
    let mut vertices = [r + DVec3::new(0., 0., 0.3); 16];
    vertices[0] = r;
    vertices[1] = DVec3::new(-0.18, 0., -0.13);
    sh.vertices = vertices;
    let q = DQuat::from_rotation_y(pitch);
    let w = -1.2 * DVec3::Y;
    let world = q * r;
    let s = State {
        motion: Motion { q, w },
        c: DVec3::Z * (-world.z),
        v: DVec3::Z * (-w.cross(world).z),
    };
    let path = Arc {
        start: s,
        shape: sh,
        sites: Sites::One(0),
    };
    let e = sh.energy(s);
    let inv_speed = |theta: f64| {
        let arm = DQuat::from_rotation_y(theta) * r;
        ((sh.inertia.y_axis.y + sh.mass * arm.x * arm.x)
            / (2. * (e - sh.mass * sh.gravity.z * arm.z)))
            .sqrt()
    };
    let n = 4096;
    let h = pitch / f64::from(n);
    let integral = (0..=n)
        .map(|k| {
            let weight = if k == 0 || k == n {
                1.
            } else if k % 2 == 0 {
                2.
            } else {
                4.
            };
            weight * inv_speed(f64::from(k) * h)
        })
        .sum::<f64>()
        * h
        / 3.;
    let re = path.next(0.0000625, 0.25)?;
    let End::Contact(1) = re.end else {
        return Err("planar reference missed opposite vertex".into());
    };
    let err = (re.time - integral).abs();
    let speed = (re.state.motion.w.y + 1. / inv_speed(0.)).abs();
    if err > 1e-10 || speed > 1e-9 {
        return Err(format!(
            "planar quadrature {integral}, RK root {} speed residual {speed:e}",
            re.time
        )
        .into());
    }
    Ok(
        json!({"quadrature_time_s":integral,"rk_time_s":re.time,"time_error_s":err,"speed_error_rad_s":speed}),
    )
}
fn kind(end: End) -> String {
    match end {
        End::Contact(k) => format!("contact {k}"),
        End::Loss => "loss".into(),
        End::Boundary => "pad limit".into(),
        End::Horizon => "horizon".into(),
        End::Ambiguous => "simultaneous".into(),
    }
}
fn state_wire(s: State) -> Value {
    json!({"q":s.motion.q.to_array(),"w":s.motion.w.to_array(),"c":s.c.to_array(),"v":s.v.to_array()})
}
fn outcome(path: Arc, re: Arrival, release: usize) -> Result<(Value, Option<Jump>)> {
    let k = path.sites.first();
    let s = re.state;
    let time = re.time;
    let reverse = Arc { start: s, ..path }.at(-time, 0.00025)?;
    let qa = reverse.motion.q;
    let qb = path.start.motion.q;
    let q_error = 4. * (0.5 * (qa - qb).length().min((qa + qb).length())).asin();
    let w_error = reverse.motion.w.distance(path.start.motion.w);
    let jump = if let End::Contact(new) = re.end {
        Some(path.shape.recollide(s, k, new)?)
    } else {
        None
    };
    let mut result = json!({"time_s":time,"end":kind(re.end),"active":k,"initially_opening":release,"recontact_is_original":jump.is_some_and(|j|j.new==release),"state":state_wire(s),"max_energy_j":re.max_energy,"max_constraint_drift":re.max_constraint,"min_sampled_normal_n":re.min_load,"reverse_orientation_rad":q_error,"reverse_spin_residual_rad_s":w_error,
 "jump":jump.map(|j|json!({"old":j.old,"new":j.new,"j_ns":j.normal,"dissipated_j":j.loss,"old_post_m_s":path.shape.point(j.old,j.after).1.z,"new_post_m_s":path.shape.point(j.new,j.after).1.z,"old_stays_loaded":matches!(j.new_sites,Sites::Two(_)),"pre_normal_m_s":[path.shape.point(j.old,s).1.z,path.shape.point(j.new,s).1.z],"after":state_wire(j.after)}))});
    if let Some(j) = jump {
        let arc = Arc {
            start: j.after,
            sites: j.new_sites,
            ..path
        };
        let loads = arc.load(j.after.motion)?;
        result["jump"]["following_bearing_forces_n"] = json!(loads);
        result["jump"]["both_active_immediately_after"] =
            (matches!(j.new_sites, Sites::Two(_)) && loads[0] > 0. && loads[1] > 0.).into();
    }
    Ok((result, jump))
}
pub(in super::super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut cases = vec![];
    let mut demo = None;
    let planar = planar_limit()?;
    fs::write(out.join("planar.json"), serde_json::to_vec_pretty(&planar)?)?;
    for active in [0, 1] {
        for pitch in [-0.18, 0.18] {
            for opening in [0.09, 0.3, 0.5] {
                for rate in [-3., -0.6, 0.6, 3.] {
                    for yaw in [0., 0.67] {
                        let c = Case {
                            active,
                            pitch,
                            yaw,
                            opening,
                            pitch_rate: rate,
                        };
                        let (i, path, rel) = fixture(c);
                        let law = match solve(i.edge) {
                            Ok(s) => s,
                            Err(why) => {
                                cases.push(json!({"initially_certified":false,"reason":why.to_string(),"active":active,"pitch":pitch,"yaw":yaw,"opening":opening,"pitch_rate":rate}));
                                continue;
                            }
                        };
                        if law.site != active
                            || law.velocity.distance(path.start.v) > 1e-12
                            || law.spin.distance(path.start.motion.w) > 1e-12
                        {
                            return Err("cold reference failed".into());
                        }
                        let coarse = path.next(0.00025, 0.4)?;
                        let fine = path.next(0.0000625, 0.4)?;
                        if kind(coarse.end) != kind(fine.end)
                            || (coarse.time - fine.time).abs() > 1e-8
                        {
                            return Err(format!(
                                "inconsistent geometry on refined integration: {}, {}",
                                kind(coarse.end),
                                kind(fine.end)
                            )
                            .into());
                        }
                        let (mut result, jump) = outcome(path, fine, rel)?;
                        if let Some(j) = jump {
                            let mut data = vec![];
                            for calls in [1, 8, 32] {
                                let ticks = ((fine.time + 0.008) / DT).ceil() as u64;
                                let (r, _) = native::native(
                                    i,
                                    path,
                                    calls,
                                    ticks,
                                    false,
                                    Some(((fine.time - 0.002) * 1e9).floor() as u64),
                                )?;
                                data.push(r);
                            }
                            result["native"] = data.into();
                            if demo.is_none()
                                && j.new == rel
                                && fine.time > 0.008
                                && fine.time < 0.15
                                && yaw > 0.
                            {
                                demo = Some((i, path, fine, j));
                            }
                        }
                        cases.push(json!({"initially_certified":true,"active":active,"pitch":pitch,"yaw":yaw,"opening":opening,"pitch_rate":rate,"public_tick_us":500,"coarse_time_s":coarse.time,"coarse_fine_event_error_s":(coarse.time-fine.time).abs(),"result":result}));
                    }
                }
            }
        }
    }
    fs::write(out.join("events.json"), serde_json::to_vec(&cases)?)?;
    let (i, path, event, jump) = demo.ok_or("no representative released-vertex recontact")?;
    replay::record(out, i, path, event, jump)
}
