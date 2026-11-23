// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Public Bevy restarts inside a physically qualified two-impact sequence.
use super::native::{Wrench, at_foot, source};
use super::*;
#[path = "inertia_upper_return_view.rs"]
pub(super) mod ordered;

fn replace_tag(xml: &mut String, from: usize, tag: &str, v: &str) {
    let open = format!("<{tag}>");
    let close = format!("</{tag}>");
    let start = from + xml[from..].find(&open).unwrap() + open.len();
    let end = start + xml[start..].find(&close).unwrap();
    xml.replace_range(start..end, v);
}
pub(super) fn initial_sdf(
    i: &Impact,
    g: &Gate,
    mu: f64,
    t: f64,
    x: Flight,
    dt: u64,
    policy: bool,
) -> String {
    let mut s = source(i, g, mu);
    // This is a generated single-rigid SDF with known named models and a single
    // world policy. Run the ordinary public native solve from the matched cut.
    let start = s
        .find(r#"<plugin name="gz::sim::systems::PrimaryContactRefinement""#)
        .unwrap();
    let end = start + s[start..].find("</plugin>").unwrap() + "</plugin>".len();
    if !policy {
        s.replace_range(start..end, "");
    }
    replace_tag(
        &mut s,
        0,
        "max_step_size",
        &format!("{:.12}", dt as f64 * 1e-9),
    );
    let spec = at_foot(&i.r, x);
    let moving = s.find(r#"<model name="rocker">"#).unwrap();
    replace_tag(
        &mut s,
        moving,
        "pose",
        &format!(
            "{} {}",
            scene::coords(spec.position.into()),
            scene::rpy(x.q)
        ),
    );
    replace_tag(&mut s, moving, "linear_velocity", &scene::coords(x.v));
    replace_tag(
        &mut s,
        moving,
        "angular_velocity",
        &scene::coords(spec.angular_velocity.into()),
    );
    let lift = s.find(r#"<model name="lift">"#).unwrap();
    replace_tag(
        &mut s,
        lift,
        "pose",
        &format!(
            "{} {}",
            scene::coords(i.r.pad_v * t - i.r.n * PAD_HALF[2]),
            scene::rpy(i.r.pad_q)
        ),
    );
    let ceiling = s.find(r#"<model name="opponent">"#).unwrap();
    replace_tag(
        &mut s,
        ceiling,
        "pose",
        &format!(
            "{} {}",
            scene::coords(g.center + g.velocity * t),
            scene::rpy(i.r.pad_q)
        ),
    );
    s
}
pub(super) fn pose(p: DVec3, q: DQuat, v: DVec3) -> BodyState {
    BodyState {
        position: p.into(),
        orientation: q.into(),
        velocity: v.into(),
        angular_velocity: [0.; 3],
    }
}
pub(super) fn snap(
    tick: u64,
    dt: u64,
    count: u32,
    foot: BodyState,
    pad: BodyState,
    gate: BodyState,
    w: [f64; 4],
) -> Value {
    json!({"tick":tick,"time_ns":tick*dt,"time_us":tick*dt/1000,"native_steps":count,"selected":false,"secondary_start":false,"normal_impulse_ns":w.iter().sum::<f64>(),"corner_ns":w,"foot":body_view(foot),"pad":body_view(pad),"opponent":body_view(gate)})
}
fn reference(i: &Impact, first: Crossing, next: Crossing, t: f64) -> Flight {
    if t < first.at - 1e-16 {
        return from_primary(&i.r, t);
    }
    if t >= next.at - 1e-16 {
        return next.after();
    }
    let mut x = first.after();
    let d = t - first.at;
    let count = (d / RK_H).ceil().max(1.) as u32;
    for _ in 0..count {
        x.advance(&i.r, d / f64::from(count));
    }
    x
}
enum Display {
    Upper,
    Return,
    Projection,
}
pub(super) fn pair_loads(
    ids: Bodies,
    gate: BodyId,
    foot: BodyState,
    patches: Vec<RigidContactPatch>,
    lower: Option<gz_physics_multifidelity::RigidPointImpactReceipt>,
    upper: Option<gz_physics_multifidelity::OpposingCornerReceipt>,
) -> [Wrench; 2] {
    let mut w = [Wrench::default(); 2];
    for patch in patches {
        if patch.a != ids.foot && patch.b != ids.foot {
            continue;
        }
        let other = if patch.a == ids.foot {
            patch.b
        } else {
            patch.a
        };
        let k = if other == ids.pad {
            0
        } else if other == gate {
            1
        } else {
            continue;
        };
        w[k].add(&patch, ids.foot, DQuat::from_array(foot.orientation));
    }
    for (k, h) in [lower, upper.map(|r| r.point)].into_iter().enumerate() {
        if let Some(h) = h {
            let amount = DVec3::from(h.impact.impulse_on_foot).dot(h.impact.normal.into());
            w[k].scalar += amount;
            w[k].corners[4 * k + usize::from(h.corner)] += amount;
        }
    }
    w
}
fn render(
    path: &Path,
    i: &Impact,
    g: &Gate,
    mu: f64,
    upper: Crossing,
    lower: Crossing,
    mode: Display,
) -> crate::Result<()> {
    let top = !matches!(mode, Display::Return);
    let projected = matches!(mode, Display::Projection);
    let dt = (((lower.at - upper.at) * 1e9 / 12.).floor() as u64).min(20000);
    if dt < 8 {
        return Err("too little isolated time for matched replay".into());
    }
    let active = if top { upper } else { lower };
    let tick_hit = if top { 2 } else { 8 };
    let t0 = active.at - tick_hit as f64 * dt as f64 * 1e-9;
    let start = reference(i, upper, lower, t0);
    let xml = initial_sdf(i, g, mu, t0, start, dt, projected);
    fs::create_dir_all(path.join("traces"))?;
    fs::write(path.join("cut.sdf"), &xml)?;
    let geom = json!({"foot_half_extents":FOOT_HALF,"foot_com_local":ARM.to_array(),"pad_half_extents":PAD_HALF,"opponent_half_extents":HALF_GATE});
    let mut lanes = vec![];
    let mut scores = vec![];
    let comparison = if projected {
        [
            ("native", "Native named ceiling", false),
            ("projected", "Measured upper-corner projection", false),
            ("analytic", "Independent finite-point sequence", false),
        ]
    } else {
        [
            ("grouped", "Native grouped friction", false),
            ("point", "Native per-point friction", true),
            ("analytic", "Independent finite-point sequence", false),
        ]
    };
    for (key, name, point) in comparison {
        let analytical = key == "analytic";
        let lane_xml = if key == "projected" {
            let x = super::projected::enable(&xml);
            fs::write(path.join("projected.sdf"), &x)?;
            x
        } else {
            xml.clone()
        };
        let (mut sim, ids) = load(&lane_xml, point)?;
        let gate = id_named(&sim, "default::opponent::gate")?;
        let stream = Arc::new(Mutex::new(Vec::new()));
        if projected && !analytical {
            let record = stream.clone();
            set_refined_observer(&mut sim, move |_, s| {
                record.lock().unwrap().push(pair_loads(
                    ids,
                    gate,
                    s.state(ids.foot).unwrap(),
                    s.contact_patches(),
                    s.rigid_point_impact,
                    s.opposing_corner_impact,
                ));
            })?;
        }
        let mut frames = vec![];
        let mut loads = vec![];
        for tick in 0..=8 {
            let t = t0 + tick as f64 * dt as f64 * 1e-9;
            let exact = reference(i, upper, lower, t);
            let (foot, pad, ceiling, corners, other, count) = if analytical {
                let mut w = [0.; 4];
                if tick == tick_hit {
                    w[active.corner % 4] = active.law.impulse.dot(active.plane.frame * DVec3::Z);
                }
                (
                    at_foot(&i.r, exact),
                    pose(i.r.pad_v * t - i.r.n * PAD_HALF[2], i.r.pad_q, i.r.pad_v),
                    pose(g.center + g.velocity * t, i.r.pad_q, g.velocity),
                    w,
                    0.,
                    0,
                )
            } else {
                if tick > 0 {
                    sim.step_exact(1)?;
                }
                assert_eq!(sim.elapsed().as_nanos(), u128::from(tick * dt));
                let e = backend(&sim);
                let f = e.state(ids.foot).unwrap();
                if tick == 0 {
                    let expected = at_foot(&i.r, start);
                    assert!(DVec3::from(expected.position).distance(f.position.into()) < 1e-11);
                    assert!(DVec3::from(expected.velocity).distance(f.velocity.into()) < 1e-10);
                    assert!(angular_distance(start.q, DQuat::from_array(f.orientation)) < 1e-10);
                }
                let mut samples = std::mem::take(&mut *stream.lock().unwrap());
                if samples.is_empty() && tick > 0 {
                    let upper = gz_sim_scene::primary_contact::last_opposing_corner(&sim)
                        .filter(|r| r.sim_time == sim.elapsed())
                        .and_then(|r| r.interval.opposing_corner_impacts.first().copied());
                    samples.push(pair_loads(
                        ids,
                        gate,
                        f,
                        e.rigid_contact_patches(),
                        None,
                        upper,
                    ));
                }
                let count = samples.len() as u32;
                let mut w = Wrench::default();
                let mut other = Wrench::default();
                for pair in samples {
                    let [lower, upper] = pair;
                    let (a, b) = if top { (upper, lower) } else { (lower, upper) };
                    for (total, step) in w.corners.iter_mut().zip(a.corners) {
                        *total += step;
                    }
                    other.scalar += b.scalar;
                }
                let corners = std::array::from_fn(|k| w.corners[k] + w.corners[k + 4]);
                (
                    f,
                    e.state(ids.pad).unwrap(),
                    e.state(gate).unwrap(),
                    corners,
                    other.scalar,
                    count,
                )
            };
            let actual = Flight::from_state(&i.r, foot);
            loads.push(json!({"tick":tick,"global_ns":t*1e9,"selected_pair_ns":corners.iter().sum::<f64>(),"other_pair_ns":other,"com_error_m":actual.com.distance(exact.com),"spin_error_rad_s":DVec3::from(foot.angular_velocity).distance(i.r.body.omega(exact.q,exact.l))}));
            frames.push(snap(tick, dt, count, foot, pad, ceiling, corners));
        }
        let file = format!("traces/{key}.json");
        write_json(&path.join(&file), &json!({"geometry":geom,"frames":frames}))?;
        let mut lane = json!({"label":key,"name":name,"file":file});
        if analytical {
            lane["analytical"] = json!(true);
        }
        lanes.push(lane);
        scores.push(json!({"lane":key,"ticks":loads}));
    }
    let face = if top { "top" } else { "bottom" };
    write_json(
        &path.join("comparison.json"),
        &json!({"title":if projected{"Measuring sliding friction on the moving ceiling"}else if top{"Striking the underside of a moving ceiling"}else{"The first return to the lower pad after a ceiling strike"},"single_body":true,"public_dt_ns":dt,"load_face":face,"capture_ticks":{"physical_strike":tick_hit},"lanes":lanes}),
    )?;
    write_json(
        &path.join("scores.json"),
        &json!({"id":i.r.id,"mode":MODES[i.mode],"effective_upper_mu":upper.mu,"authored_gate_mu":mu,"cut_ns":t0*1e9,"dt_ns":dt,"upper":upper.data(),"return":lower.data(),"lanes":scores}),
    )
}
pub(super) fn write(
    path: &Path,
    i: &Impact,
    g: &Gate,
    gate_mu: f64,
    upper: Crossing,
    lower: Crossing,
) -> crate::Result<()> {
    render(
        &path.join("upper"),
        i,
        g,
        gate_mu,
        upper,
        lower,
        Display::Upper,
    )?;
    render(
        &path.join("return"),
        i,
        g,
        gate_mu,
        upper,
        lower,
        Display::Return,
    )
}

pub(super) fn projection(
    path: &Path,
    i: &Impact,
    g: &Gate,
    mu: f64,
    upper: Crossing,
    lower: Crossing,
) -> crate::Result<()> {
    render(path, i, g, mu, upper, lower, Display::Projection)
}
