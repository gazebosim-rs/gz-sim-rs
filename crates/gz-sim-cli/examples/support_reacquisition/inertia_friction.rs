// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Manufacture spatial sticking and sliding impulses independently of either contact solver.
use super::*;
#[path = "inertia_point.rs"]
pub(crate) mod projection;
use gz_physics_multifidelity::RigidContactPatch;
use gz_sim_scene::{WorldLoadOptions, WorldRuntimeOptions, load_world_with_runtime};
#[derive(Clone, Copy, Serialize)]
struct NativeProfile {
    point: bool,
    pgs: usize,
    relax: usize,
}
const PROFILES: [NativeProfile; 6] = [
    NativeProfile {
        point: false,
        pgs: 1,
        relax: 0,
    },
    NativeProfile {
        point: false,
        pgs: 1,
        relax: 16,
    },
    NativeProfile {
        point: false,
        pgs: 8,
        relax: 16,
    },
    NativeProfile {
        point: true,
        pgs: 1,
        relax: 0,
    },
    NativeProfile {
        point: true,
        pgs: 1,
        relax: 16,
    },
    NativeProfile {
        point: true,
        pgs: 8,
        relax: 16,
    },
];
const MODES: [&str; 4] = ["stick", "near_stick", "near_slide", "slide"];
struct Impact {
    r: Physical,
    mu: f64,
    mode: usize,
    impulse: DVec3,
    before: DVec3,
    after: DVec3,
    loss: f64,
    matrix: DMat3,
    unique_margin: f64,
}
fn manufactured(id: usize, mode: usize) -> Impact {
    manufacture_for(fixture(id, if id.is_multiple_of(2) { 0 } else { 1 }), mode)
}
fn manufacture_for(mut r: Physical, mode: usize) -> Impact {
    let id = r.id;
    let mu = [0.2, 0.5, 0.8][(id / 4) % 3];
    let arm = r.q * (corners()[r.corner] - ARM);
    let n = r.n;
    let theta = std::f64::consts::TAU * radical(id + 137, 83);
    let (s, c) = theta.sin_cos();
    let slip = r.pad_q * DVec3::new(c, s, 0.);
    let normal = r.j;
    let extent = [0.35, 0.9999, 1., 1.][mode];
    let residual = [0., 0., 0.0001, 0.35 + radical(id + 291, 11)][mode];
    let impulse = normal * (n - mu * extent * slip);
    let after = slip * residual;
    let response = |j: DVec3| j / M + r.body.omega(r.q, arm.cross(j)).cross(arm);
    let basis = [r.pad_q * DVec3::X, r.pad_q * DVec3::Y, n];
    let [x, y, z] = basis.map(|a| r.pad_q.conjugate() * response(a));
    let matrix = DMat3::from_cols(x, y, z);
    let before = after - response(impulse);
    let w = r.body.omega(r.q, r.pre.l);
    r.pre.v = r.pad_v + before - w.cross(arm);
    r.post.v = r.pre.v + impulse / M;
    r.post.l = r.pre.l + arm.cross(impulse);
    r.closing = -before.dot(n);
    let loss = -0.5 * (before + after).dot(impulse);
    // After eliminating normal closure, the tangent system is
    // (S+lambda*I) jt = -d. Its cone residual is strictly decreasing
    // if wn*cos(jt,(S+lambda*I)^-1 jt) > mu*|wnt|.
    let wn = matrix.z_axis.z;
    let x = matrix.x_axis.x - matrix.z_axis.x.powi(2) / wn;
    let y = matrix.y_axis.y - matrix.z_axis.y.powi(2) / wn;
    let xy = matrix.y_axis.x - matrix.z_axis.x * matrix.z_axis.y / wn;
    let diag = (x - y).hypot(2. * xy);
    let ratio = (x + y + diag) / (x + y - diag);
    let unique_margin =
        wn * 2. * ratio.sqrt() / (1. + ratio) - mu * matrix.z_axis.truncate().length();
    Impact {
        r,
        mu,
        mode,
        impulse,
        before,
        after,
        loss,
        matrix,
        unique_margin,
    }
}
fn configured(i: &Impact, p: NativeProfile) -> String {
    let (_, text) = super::configured(&i.r, 8, true, false);
    text.replace(
        "<mu>0</mu><mu2>0</mu2>",
        &format!("<mu>{}</mu><mu2>{}</mu2>", i.mu, i.mu),
    )
    .replace(
        "<pgs_iterations>1</pgs_iterations>",
        &format!("<pgs_iterations>{}</pgs_iterations>", p.pgs),
    )
    .replace(
        "<relaxation_iterations>16</relaxation_iterations>",
        &format!("<relaxation_iterations>{}</relaxation_iterations>", p.relax),
    )
}
fn load(text: &str, point: bool) -> Result<(Simulation, Bodies)> {
    let sim = load_world_with_runtime(
        text,
        "triaxial_friction.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions {
            rapier_per_point_friction: point,
            ..Default::default()
        },
    )?;
    let ids = sdf_features::finite_ids(&sim, false)?;
    Ok((sim, ids))
}
struct Boundary {
    cut: gyro::Cut,
    hit: Option<gz_physics_multifidelity::RigidPointImpactReceipt>,
    normal: DVec3,
    tangent: Option<DVec3>,
    corners: [f64; 4],
}
fn measured(cut: gyro::Cut, ids: Bodies, patches: Vec<RigidContactPatch>) -> Boundary {
    let mut out = Boundary {
        cut,
        hit: None,
        normal: DVec3::ZERO,
        tangent: Some(DVec3::ZERO),
        corners: [0.; 4],
    };
    let q = DQuat::from_array(cut.foot.orientation);
    for p in patches {
        let a = p.a == ids.foot && p.b == ids.pad;
        if !(a || p.b == ids.foot && p.a == ids.pad) {
            continue;
        }
        let sign = if a { 1. } else { -1. };
        for x in p.points {
            if x.normal_impulse == 0. {
                continue;
            }
            let local = q.conjugate() * DVec3::from(if a { x.lever_a } else { x.lever_b }) + ARM;
            let k = 2 * usize::from(local.x >= 0.) + usize::from(local.y >= 0.);
            out.corners[k] += x.normal_impulse;
            out.normal += sign * x.normal_impulse * DVec3::from(p.normal_on_a);
            out.tangent = out
                .tangent
                .zip(x.tangent_impulse_on_a)
                .map(|(this, v)| this + sign * DVec3::from(v));
        }
    }
    out
}
fn run(i: &Impact, p: NativeProfile, text: &str) -> Result<(Value, Vec<Snapshot>)> {
    let r = &i.r;
    let (mut sim, ids) = load(text, p.point)?;
    let initial = state(backend(&sim), ids)?;
    let mut frames = vec![frame(0, initial, 0, false, [0.; 4])];
    let log = Arc::new(Mutex::new(Vec::new()));
    let share = log.clone();
    set_refined_observer(&mut sim, move |_, s| {
        let cut = gyro::Cut {
            foot: s.state(ids.foot).unwrap(),
            pad: s.state(ids.pad).unwrap(),
            ns: s.elapsed.as_nanos() as u64,
        };
        let mut boundary = measured(cut, ids, s.contact_patches());
        if let Some(hit) = s.rigid_point_impact {
            assert_eq!(
                boundary.corners, [0.; 4],
                "filtered native pulse had a second reaction"
            );
            let normal = DVec3::from(hit.impact.normal);
            let impulse = DVec3::from(hit.impact.impulse_on_foot);
            let j = normal.dot(impulse);
            boundary.corners[usize::from(hit.corner)] = j;
            boundary.normal = j * normal;
            boundary.tangent = Some(impulse - boundary.normal);
            boundary.hit = Some(hit);
        }
        share.lock().unwrap().push(boundary);
    })?;
    let mut loads = vec![];
    let mut public = vec![];
    let mut attempts = vec![];
    let mut departures = vec![];
    let mut calls = 0;
    let mut j = DVec3::ZERO;
    let mut point_j = Some(DVec3::ZERO);
    let mut no_reaction = 0f64;
    let mut summed = [0.; 4];
    let mut selected = vec![];
    for tick in 1..=8u64 {
        let mut before = gyro::Cut::bodies(backend(&sim), ids, 0);
        sim.step_exact(1)?;
        let mut steps = std::mem::take(&mut *log.lock().unwrap());
        if steps.is_empty() {
            let e = backend(&sim);
            steps.push(measured(
                gyro::Cut::bodies(e, ids, 500000),
                ids,
                e.rigid_contact_patches(),
            ));
        }
        let count = steps.len() as u32;
        calls += count;
        if count > 1 {
            selected.push(tick);
        }
        let mut corner_impulses = [0.; 4];
        let mut interval_n = DVec3::ZERO;
        let mut interval_t = Some(DVec3::ZERO);
        let mut projected = 0;
        let mut first = None;

        for s in steps {
            let dt = (s.cut.ns - before.ns) as f64 * 1e-9;
            let impulse = M
                * (DVec3::from(s.cut.foot.velocity)
                    - DVec3::from(before.foot.velocity)
                    - r.gravity * dt);
            j += impulse;
            interval_n += s.normal;
            interval_t = interval_t.zip(s.tangent).map(|(a, b)| a + b);

            if s.corners.iter().sum::<f64>() > 0. {
                for k in 0..4 {
                    corner_impulses[k] += s.corners[k];
                    summed[k] += s.corners[k];
                }
                let q = DQuat::from_array(s.cut.foot.orientation);
                let rel = DVec3::from(s.cut.foot.velocity) - DVec3::from(s.cut.pad.velocity)
                    + DVec3::from(s.cut.foot.angular_velocity)
                        .cross(q * (corners()[r.corner] - ARM));
                let native = s.tangent.map(|t| s.normal + t);
                if p.point {
                    assert!(native.is_some(), "per-point impulse missing");
                }
                point_j = point_j.zip(native).map(|(a, b)| a + b);
                let event_ns = s
                    .hit
                    .map_or(s.cut.ns, |h| h.impact.elapsed.as_nanos() as u64);
                first.get_or_insert(event_ns);
                let projection=s.hit.map(|h|{projected+=1;json!({"corner":h.corner,"application_ns":(tick-1)*500000+event_ns,"after_cut_s":h.after_cut_s,"pulse_start_ns":h.pulse_start.as_nanos()as u64,"pulse_duration_ns":h.pulse_duration.as_nanos()as u64,"outward_acceleration":h.outward_acceleration,"sticking":h.impact.sticking,"friction":h.impact.friction,"gap_m":h.impact.gap,"point":h.impact.point,"before":h.impact.before_relative_velocity,"after":h.impact.after_relative_velocity,"loss_j":h.impact.dissipated_energy,"impulse":h.impact.impulse_on_foot})});
                loads.push(json!({"projection":projection,"end_ns":(tick-1)*500000+s.cut.ns,"duration_ns":(s.cut.ns-before.ns),"corners":s.corners,"reported_normal":s.normal.to_array(),"reported_tangent":s.tangent.map(|v|v.to_array()),"native_normal":s.hit.is_none().then(||s.normal.to_array()),"native_tangent":s.hit.is_none().then(||s.tangent.map(|v|v.to_array())).flatten(),"momentum_impulse":impulse.to_array(),"outgoing_corner_velocity":rel.to_array()}));
            } else {
                no_reaction = no_reaction.max(impulse.length());
            }
            before = s.cut;
        }
        if let Some(log) = last_box_timing(&sim).filter(|x| x.sim_time == sim.elapsed())
            && let Some(list) = &log.timing.point_attempts
        {
            if let Some(spans) = &log.timing.departure_calls {
                for (start, h) in spans {
                    assert!(
                        log.interval
                            .rigid_point_impacts
                            .iter()
                            .any(|hit| hit.pulse_start + hit.pulse_duration == *start)
                    );
                    let absolute = (tick - 1) * 500000 + start.as_nanos() as u64;
                    departures.push(json!({"start_ns":absolute,"duration_ns":h.as_nanos()as u64}));
                }
            }
            attempts.extend(list.iter().map(|(cut,decline)|json!({"tick":tick,"cut_ns":cut,"decline":decline.map(|r|format!("{r:?}"))})));
            assert_eq!(log.interval.native_steps, count);
            assert_eq!(log.interval.rigid_point_impacts.len(), projected);
            if let Some(pair) =
                log.interval.pairs.iter().find(|p| {
                    (p.a, p.b) == (ids.foot, ids.pad) || (p.b, p.a) == (ids.foot, ids.pad)
                })
            {
                let sign = if pair.a == ids.foot { 1. } else { -1. };
                assert!(interval_n.distance(sign * DVec3::from(pair.normal_impulse_on_a)) < 1e-11);
                if let Some(t) = interval_t {
                    let t_native = pair
                        .tangent_impulse_on_a
                        .expect("public explicit tangent response");
                    assert!(t.distance(sign * DVec3::from(t_native)) < 1e-11);
                }
                assert_eq!(pair.first_positive_end.map(|t| t.as_nanos() as u64), first);
                public.push(json!({"tick":tick,"steps":count,"first_ns":first,"projected":projected,"normal":(sign*DVec3::from(pair.normal_impulse_on_a)).to_array(),"tangent":pair.tangent_impulse_on_a.map(|a|(sign*DVec3::from(a)).to_array())}));
            }
        }
        frames.push(frame(
            tick,
            state(backend(&sim), ids)?,
            count,
            count > 1,
            corner_impulses,
        ));
    }
    let final_state = frames.last().unwrap().state;
    let (l, e) = r.body.surface(final_state);
    let (_, e0) = r.body.surface(initial);
    let exact = r.state(WINDOW);
    let gravity_work = M * r.gravity.dot(final_state.com() - initial.com());
    let loss = e0 + r.pad_v.dot(j) + gravity_work - e;
    let wrong = summed.iter().sum::<f64>() - summed[r.corner];
    let sum = point_j.map(|n| j.distance(n));
    let angular = l.distance(r.post.l);
    let vector_error = j.distance(i.impulse);
    let expected = frames[0].state.velocity() + j / M + r.gravity * WINDOW;
    assert!(expected.distance(final_state.velocity()) < 1e-11 && no_reaction < 1e-10);
    Ok((
        json!({"profile":p,"calls":calls,"public":public,"attempts":attempts,"departures":departures,"selected":selected,"loads":loads,"world_impulse":j.to_array(),"native_point_closure_ns":sum,
      "unloaded_max_delta_ns":no_reaction,"normal_error_ns":(j-i.impulse).dot(r.n).abs(),"tangent_error_ns":((j-i.impulse)-r.n*(j-i.impulse).dot(r.n)).length(),"vector_error_ns":vector_error,"initial_energy_j":e0,"final_energy_j":e,"gravity_work_j":gravity_work,"pad_work_j":r.pad_v.dot(j),"loss_j":loss,"loss_error_j":(loss-i.loss).abs(),"wrong_corner_ns":wrong,"angular_momentum_error_nms":angular,"final_com_m":exact.com().distance(final_state.com()),"final_spin_rad_s":DVec3::from(exact.foot.angular_velocity).distance(final_state.foot.angular_velocity.into())}),
        frames,
    ))
}

fn law(i: &Impact) -> Value {
    json!({"id":i.r.id,"mode":MODES[i.mode],"mu":i.mu,"gravity":i.r.gravity.to_array(),"target_impulse":i.impulse.to_array(),"normal":i.r.n.to_array(),"before":i.before.to_array(),"after":i.after.to_array(),"delassus_pad_columns":i.matrix.to_cols_array(),"pad_q":i.r.pad_q.to_array(),"monotonic_margin":i.unique_margin,"dissipation_j":i.loss,"arrival_ns":(1e9*i.r.arrival).round()as u64,"corner":i.r.corner})
}
fn views(path: &Path, i: &Impact, certain: bool) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut lanes = vec![];
    for (name, profile, title) in [
        ("grouped", PROFILES[1], "Native grouped friction"),
        ("point", PROFILES[4], "Native per-point friction"),
    ] {
        let text = configured(i, profile);
        let (_, frames) = run(i, profile, &text)?;
        separated::single_rigid_trace(path, name, &frames)?;
        fs::write(path.join(format!("{name}.sdf")), text)?;
        lanes.push(json!({"name":title,"label":name,"file":format!("traces/{name}.json")}));
    }
    if certain {
        separated::single_rigid_trace(path, "analytic", &i.r.trace())?;
        lanes.push(json!({"name":"Independent Coulomb and Euler reference","label":"analytic","file":"traces/analytic.json","analytical":true}));
    }
    write_json(
        &path.join("comparison.json"),
        &json!({"single_body":true,"title":if certain{"Sliding triaxial impact under gravity"}else{"After the separately qualified triaxial impact"},"lanes":lanes}),
    )
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    let mut excluded = std::collections::BTreeMap::new();
    let mut chosen = None;
    let mut ambiguous = None;
    for id in (0..192).chain(1152..1344) {
        for mode in 0..4 {
            let i = manufactured(id, mode);
            let r = &i.r;
            let pre = r.certify_branches(&[(0., r.pre)]);
            let post = r.certify_branches(&[(WINDOW, r.post)]);
            let why = if r.closing <= 0.05 || i.loss <= 0. {
                Some("insufficient closing velocity or dissipated energy")
            } else if i.unique_margin <= 1e-8 {
                Some("Coulomb uniqueness not established")
            } else {
                pre.reason.or(post.reason)
            };
            if let Some(reason) = why {
                *excluded.entry(reason).or_insert(0usize) += 1;
                if ambiguous.is_none()
                    && r.closing > 0.05
                    && i.loss > 0.
                    && i.unique_margin > 1e-8
                    && pre.reason.is_none()
                    && post.reason.is_some()
                    && r.certify_branches(&[(r.arrival + 0.0001, r.post)])
                        .reason
                        .is_none()
                {
                    let mut lo = r.arrival + 0.0001;
                    let mut hi = WINDOW;
                    for _ in 0..12 {
                        let cut = (lo + hi) * 0.5;
                        if r.certify_branches(&[(cut, r.post)]).reason.is_none() {
                            lo = cut;
                        } else {
                            hi = cut;
                        }
                    }
                    let mut traces = vec![];
                    for p in [PROFILES[1], PROFILES[4]] {
                        let (row, _) = run(&i, p, &configured(&i, p))?;
                        traces.push(json!({"profile":p,"loads":row["loads"],"world_impulse":row["world_impulse"],"calls":row["calls"]}));
                    }
                    ambiguous = Some(
                        json!({"reference":law(&i),"last_certified_post_s":lo,"first_unresolved_post_s":hi,"reason":post.reason,"native_observations":traces}),
                    );
                    views(&path.join("unresolved"), &i, false)?;
                }
                continue;
            }
            let mut trials = vec![];
            for p in PROFILES {
                let text = configured(&i, p);
                let (row, _) = run(&i, p, &text)?;
                trials.push(row);
            }
            if chosen.is_none()
                && id >= 1152
                && mode == 3
                && r.body.omega(r.q, r.pre.l).length() > 5.
            {
                chosen = Some([id, mode]);
                views(path, &i, true)?;
            }
            rows.push(json!({"reference":law(&i),"pre":pre,"post":post,"trials":trials}));
        }
    }
    write_json(
        &path.join("study.json"),
        &json!({"cases":rows,"declines":excluded,"view":chosen,"uncertain":ambiguous}),
    )
}
pub(crate) fn cost(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let data: Value = serde_json::from_slice(&fs::read(
        path.parent()
            .ok_or("need output subdirectory")?
            .join("study.json"),
    )?)?;
    let mut rows = vec![];
    for c in data["cases"].as_array().unwrap() {
        let r = &c["reference"];
        let id = r["id"].as_u64().unwrap() as usize;
        let mode = MODES.iter().position(|x| r["mode"] == *x).unwrap();
        let i = manufactured(id, mode);
        let profiles = [PROFILES[1], PROFILES[2], PROFILES[4], PROFILES[5]];
        let mut fixtures = vec![];
        let clocked = |p: NativeProfile, text: &str| -> Result<(Vec<u64>, State)> {
            let (mut s, ids) = load(text, p.point)?;
            let mut times = vec![];
            for _ in 0..8 {
                let t = Instant::now();
                s.step_exact(1)?;
                times.push(t.elapsed().as_nanos() as u64);
            }
            Ok((times, state(backend(&s), ids)?))
        };
        for p in profiles {
            let text = configured(&i, p);
            let (_, s) = clocked(p, &text)?;
            fixtures.push((p, text, s));
        }
        let mut times: [Vec<Vec<u64>>; 4] = Default::default();
        for rep in 0..4 {
            for j in 0..4 {
                let k = (if rep % 2 == 0 { j } else { 3 - j } + id + mode + rep) % 4;
                let (p, text, end) = &fixtures[k];
                let (t, s) = clocked(*p, text)?;
                if s != *end {
                    return Err("frictional native replay not deterministic".into());
                }
                times[k].push(t);
            }
        }
        for ((p, _, _), cost) in fixtures.into_iter().zip(times) {
            rows.push(json!({"id":id,"mode":MODES[mode],"profile":p,"steps_ns":cost}));
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":rows}))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn independently_manufactured_cone_impulses_satisfy_the_separate_solver_and_energy_identity() {
        for id in 0..64 {
            for mode in 0..4 {
                let i = manufactured(id, mode);
                if i.unique_margin <= 1e-8 || i.r.closing <= 0.05 {
                    continue;
                }
                let q = i.r.pad_q;
                let reference = gz_physics::PointImpact {
                    inverse_mass: i.matrix.transpose().to_cols_array_2d(),
                    relative_velocity: (q.conjugate() * i.before).to_array(),
                    friction: i.mu,
                }
                .solve()
                .unwrap();
                let j = q * DVec3::from(reference.impulse);
                assert!(
                    i.impulse.distance(j) < 1e-9,
                    "{id} {mode} {j:?} {:?}",
                    i.impulse
                );
                assert!(
                    i.after
                        .distance(q * DVec3::from(reference.relative_velocity))
                        < 1e-8
                );
                assert!((i.loss - reference.dissipated_energy).abs() < 1e-9);
                assert_eq!(reference.sticking, mode < 2);
            }
        }
    }
}
