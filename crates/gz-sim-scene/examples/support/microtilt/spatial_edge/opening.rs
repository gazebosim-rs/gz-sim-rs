// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Both edge sites are tangent and closing. One impulse can open the other.
use super::*;
use glam::{DMat2, DVec2};
use std::f64::consts::{PI, TAU};

#[derive(Clone, Copy)]
struct Setup {
    active: usize,
    sticking: bool,
    mu: f64,
    yaw: f64,
    pitch: f64,
    opening: f64,
}
fn make_opening(spec: Setup) -> Input {
    let Setup {
        active,
        sticking,
        mu,
        yaw,
        pitch,
        opening,
    } = spec;
    let qz = DQuat::from_rotation_z(yaw);
    let q = qz * DQuat::from_rotation_y(pitch);
    let x = usize::from(pitch > 0.) * 2;
    let ids = [x, x + 1];
    let arm = ids.map(|k| q * local_arms()[k]);
    let mut i = make(mu, yaw, 0.18, [0.48, 0.72], false);
    let n = if active == 0 { 0.8 } else { 1.1 };
    let slip = if sticking {
        DVec3::ZERO
    } else {
        qz * DVec3::new(1.8, -0.87, 0.)
    };
    let spin = qz
        * DVec3::new(
            (if active == 0 { 1. } else { -1. }) * opening / 0.2,
            0.17,
            -0.44,
        );
    let v = slip - spin.cross(arm[active]);
    let tangent = if sticking {
        mu * n * (qz * DVec3::new(0.28, -0.37, 0.))
    } else {
        -mu * n * slip.normalize()
    };
    let j = DVec3::Z * n + tangent;
    i.edge.arm = arm;
    i.seed.state.c = DVec3::Z * (-arm[0].z);
    i.seed.state.q = q;
    i.edge.velocity = v - j / i.edge.mass;
    i.edge.spin = spin - i.edge.inertia.inverse() * arm[active].cross(j);
    i.seed.state.v = i.edge.velocity;
    i.seed.state.w = i.edge.spin;
    i.indices = ids;
    i.after_v = v;
    i.after_w = spin;
    i.desired = [DVec3::ZERO; 2];
    i.desired[active] = j;
    i
}
#[derive(Clone, Copy)]
struct Solution {
    site: usize,
    j: DVec3,
    post: [DVec3; 2],
    velocity: DVec3,
    spin: DVec3,
    sticking: bool,
    loss: f64,
    cone_margin: f64,
    uniqueness: f64,
}
fn local_solution(w: DMat3, u: DVec3, mu: f64) -> Result<(DVec3, bool, f64)> {
    // Normal elimination followed by Coulomb monotonicity certifies the
    // projected point's unique response, including coexistence of modes.
    let normal = w.z_axis.z;
    let nt = DVec2::new(w.x_axis.z, w.y_axis.z);
    let s = DMat2::from_cols(
        w.x_axis.truncate() - w.z_axis.truncate() * (nt.x / normal),
        w.y_axis.truncate() - w.z_axis.truncate() * (nt.y / normal),
    );
    let a = nt / normal;
    let lambda = 0.5 * (s.x_axis.x + s.y_axis.y + (s.x_axis.x - s.y_axis.y).hypot(2. * s.y_axis.x));
    let beta = mu * (a.dot(s.inverse() * a) * lambda).sqrt();
    if !beta.is_finite() || beta >= 1. - 1e-12 {
        return Err("single-site friction uniqueness unresolved".into());
    }
    let js = -w.inverse() * u;
    if js.z > 0. && js.truncate().length() <= mu * js.z {
        return Ok((js, true, beta));
    }
    if mu == 0. {
        return Ok((DVec3::Z * (-u.z / normal), false, beta));
    }
    let ray = |theta: f64| {
        let (s, c) = theta.sin_cos();
        let d = DVec2::new(c, s);
        let v = (-mu * d).extend(1.);
        let j = v * (-u.z / (w * v).z);
        let vt = (u + w * j).truncate();
        (d.perp_dot(vt), d.dot(vt), j)
    };
    let mut answers = vec![];
    for k in 0..128 {
        let mut lo = TAU * k as f64 / 128.;
        let mut hi = TAU * (k + 1) as f64 / 128.;
        let a = ray(lo).0;
        if a * ray(hi).0 > 0. {
            continue;
        }
        for _ in 0..54 {
            let mid = 0.5 * (lo + hi);
            if a * ray(mid).0 > 0. {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let (_, along, j) = ray(0.5 * (lo + hi));
        if along > 1e-12
            && j.z > 0.
            && !answers
                .iter()
                .any(|other: &DVec3| other.distance(j) < 1e-12)
        {
            answers.push(j);
        }
    }
    match answers.as_slice() {
        [j] => Ok((*j, false, beta)),
        _ => Err("independent point slip unqualified".into()),
    }
}
fn solve(e: Edge) -> Result<Solution> {
    let pre = [e.point(0), e.point(1)];
    let close = DVec2::new(-pre[0].z, -pre[1].z);
    if close.min_element() <= 1e-12 {
        return Err("requires two closing tangent points".into());
    }
    let axes = [DVec3::X, DVec3::Y, DVec3::Z];
    let direction = close.normalize();
    // The convex hull of rays at the vertices of a regular circumscribed
    // polygon contains each full circular Coulomb cone. If every kick on
    // both cones lies strictly on one side of the required normal kick,
    // two active sites and the opposite singleton are both impossible.
    let sides = 64;
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for site in 0..2 {
        for d in 0..sides {
            let angle = TAU * (d as f64 + 0.5) / sides as f64;
            let (s, c) = angle.sin_cos();
            let factor = e.friction[site] / (PI / sides as f64).cos();
            let unit = DVec3::new(c * factor, s * factor, 1.);
            let kick = DVec2::new(e.mobility(0, site, unit).z, e.mobility(1, site, unit).z);
            let side = direction.perp_dot(kick);
            lo = lo.min(side);
            hi = hi.max(side);
        }
    }
    let (site, margin) = if lo > 1e-12 {
        (0, lo)
    } else if hi < -1e-12 {
        (1, -hi)
    } else {
        return Err("two-site alternative unresolved".into());
    };
    let w = DMat3::from_cols_array_2d(&axes.map(|a| e.mobility(site, site, a).to_array()));
    let (j, sticking, beta) = local_solution(w, pre[site], e.friction[site])?;
    let velocity = e.velocity + j / e.mass;
    let spin = e.spin + e.inertia.inverse() * e.arm[site].cross(j);
    let post = e.arm.map(|r| velocity + spin.cross(r));
    let other = 1 - site;
    let loss = -0.5 * j.dot(pre[site] + post[site]);
    if post[site].z.abs() > 1e-12 || post[other].z <= 1e-12 || loss < 0. {
        return Err("opening-site complementarity or energy failed".into());
    }
    Ok(Solution {
        site,
        j,
        post,
        velocity,
        spin,
        sticking,
        loss,
        cone_margin: margin,
        uniqueness: beta,
    })
}
fn sample(input: Input, law: Solution, rigid: bool, passes: usize) -> Result<Value> {
    let got = native_points(input, rigid, passes)?;
    let j = got.impulses;
    let post = input.edge.arm.map(|arm| got.end.v + got.end.w.cross(arm));
    let live = law.site;
    let other = 1 - live;
    Ok(
        json!({"ordinary_rigid":rigid,"passes":passes,"impulses_ns":j.map(|x|x.to_array()),"post_point_m_s":post.map(|x|x.to_array()),"active_error_ns":j[live].distance(law.j),"opening_vertex_load_ns":j[other].z,"opening_speed_error_m_s":post[other].z-law.post[other].z,"point_speed_error_m_s":post[live].distance(law.post[live]).max(post[other].distance(law.post[other])),"angular_balance_nms":got.angular_balance}),
    )
}
fn view(out: &Path) -> Result<()> {
    let spec = Setup {
        active: 0,
        sticking: false,
        mu: 0.25,
        yaw: 0.63,
        pitch: 0.18,
        opening: 0.3,
    };
    let i = make_opening(spec);
    let law = solve(i.edge)?;
    let mut summaries = vec![];
    for label in ["ordinary", "pgs32", "physical"] {
        let src = scene(i.seed, spec.mu, false, "ordinary");
        let mut sim = load(&src, true)?;
        let mut r = ids(&sim)?;
        if label == "pgs32" {
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .set_contact_pgs_passes(32)?;
        }
        let mut rows = vec![];
        let mut history = vec![];
        for tick in 0..=160 {
            let mut loads = [0.; 4];
            if tick == 1 && label == "physical" {
                let post = Seed {
                    state: PoseState {
                        v: law.velocity,
                        w: law.spin,
                        ..i.seed.state
                    },
                    ..i.seed
                };
                sim = load(&scene(post, spec.mu, false, "ordinary"), true)?;
                r = ids(&sim)?;
                loads[i.indices[law.site]] = law.j.z;
            }
            if tick > 0 {
                sim.step_exact(1)?;
                let n = engine(&sim);
                let q = DQuat::from_array(n.state(r.foot).unwrap().orientation);
                let m = moments(&n.rigid_contact_patches(), &r, q, false)?.2;
                for k in 0..4 {
                    loads[k] += m[k];
                }
            }
            let b = departure::ingest(engine(&sim), &r);
            let ss = Seed::read(b);
            let gaps = corner_gaps(ss.state);
            let (f, u, p) = b;
            let min = gaps.into_iter().fold(f64::INFINITY, f64::min);
            rows.push(json!({"tick":tick,"time_us":tick*500,"native_steps":u32::from(tick>0),"solver_iterations":4,"selected":false,"weak_start":false,"secondary_start":false,"normal_impulse_ns":loads.iter().sum::<f64>(),"corner_ns":loads,"foot":wire(f),"upper":wire(u.unwrap()),"pad":wire(p),"gap_m":min}));
            history.push(json!({"tick":tick,"corner_gaps_m":gaps,"corner_ns":loads}));
        }
        fs::write(
            out.join(format!("traces/{label}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":rows}),
            )?,
        )?;
        if label == "ordinary" {
            fs::write(out.join("edge.sdf"), &src)?;
        }
        summaries.push(json!({"label":label,"active_quadrant":i.indices[law.site],"opening_quadrant":i.indices[1-law.site],"history":history}));
    }
    let lanes = [
        ("ordinary", "Ordinary locked foot"),
        ("pgs32", "32 fixed-time PGS passes"),
        ("physical", "Single loaded site, then native"),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"One touching edge vertex lifts as the other takes the impact","capture_ticks":{"first_interval":1},"lanes":lanes}),
        )?,
    )?;
    fs::write(out.join("public.json"), serde_json::to_vec(&summaries)?)?;
    Ok(())
}
pub(in super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = vec![];
    for mu in [0., 0.08, 0.2, 0.35, 0.55] {
        for pitch in [-0.18, 0.18] {
            for yaw in [0., 0.47, 1.19] {
                for active in 0..2 {
                    for sticking in [false, true] {
                        if mu == 0. && sticking {
                            continue;
                        }
                        for opening in [0.04, 0.13, 0.3, 0.5, 0.72] {
                            let s = Setup {
                                mu,
                                pitch,
                                yaw,
                                active,
                                sticking,
                                opening,
                            };
                            let i = make_opening(s);
                            let e = i.edge;
                            let result = match solve(e) {
                                Err(why) => json!({"admitted":false,"reason":why.to_string()}),
                                Ok(law) => {
                                    if law.site != active
                                        || law.sticking != sticking
                                        || law.j.distance(i.desired[active]) > 1e-11
                                        || law.velocity.distance(i.after_v) > 1e-11
                                        || law.spin.distance(i.after_w) > 1e-11
                                    {
                                        return Err("manufactured solution disagrees with independent normal cone and point law".into());
                                    }
                                    let mut native = vec![];
                                    for rigid in [false, true] {
                                        for passes in [1, 8, 32] {
                                            native.push(sample(i, law, rigid, passes)?);
                                        }
                                    }
                                    json!({"admitted":true,"active_site":law.site,"physical_j_ns":law.j.to_array(),"post_m_s":law.post.map(|v|v.to_array()),"after_v":law.velocity.to_array(),"after_w":law.spin.to_array(),"loss_j":law.loss,"cone_margin_per_kg":law.cone_margin,"single_uniqueness_beta":law.uniqueness,"native":native})
                                }
                            };
                            rows.push(json!({"mu":mu,"pitch":pitch,"yaw":yaw,"active":active,"sticking":sticking,"manufactured_opening_m_s":opening,"arms_m":e.arm.map(|r|r.to_array()),"pre_m_s":[e.point(0).to_array(),e.point(1).to_array()],"result":result}));
                        }
                    }
                }
            }
        }
    }
    fs::write(out.join("opening.json"), serde_json::to_vec(&rows)?)?;
    view(out)
}

#[path="native_opening.rs"]
pub(in super::super::super) mod native;

#[path="rocking.rs"]
pub(in super::super::super) mod rocking;
