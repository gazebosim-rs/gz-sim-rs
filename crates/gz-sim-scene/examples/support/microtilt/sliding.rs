// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Isolate the metric used by saturated tangent rows at one cold, oblique impact.
use super::*;
use gz_physics::PointImpact;
use std::f64::consts::TAU;

#[derive(Clone, Copy)]
enum Metric {
    Physical,
    Diagonal,
    Coupled,
}
fn tangent_metric(w: [[f64; 3]; 3], u: glam::DVec2, m: Metric) -> glam::DVec2 {
    match m {
        Metric::Physical => u,
        Metric::Diagonal => glam::DVec2::new(u.x / w[0][0], u.y / w[1][1]),
        Metric::Coupled => {
            let d = w[0][0] * w[1][1] - w[0][1] * w[1][0];
            glam::DVec2::new(w[1][1] * u.x - w[0][1] * u.y, w[0][0] * u.y - w[1][0] * u.x) / d
        }
    }
}
fn velocity(w: [[f64; 3]; 3], u: DVec3, j: DVec3) -> DVec3 {
    u + DVec3::from_array(w.map(|a| DVec3::from(a).dot(j)))
}
/// Solve the measured saturated-row fixed point independently. For each
/// friction-cone ray, normal complementarity determines the impulse scale.
fn surface(w: [[f64; 3]; 3], u: DVec3, mu: f64, physical: DVec3, metric: Metric) -> Result<DVec3> {
    if velocity(w, u, physical).truncate().length() < 1e-12 {
        return Ok(physical);
    }
    let on_ray = |theta: f64| {
        let (s, c) = theta.sin_cos();
        let d = glam::DVec2::new(c, s);
        let direction = DVec3::new(-mu * c, -mu * s, 1.);
        let jn = -u.z / DVec3::from(w[2]).dot(direction);
        let j = jn * direction;
        let v = tangent_metric(w, velocity(w, u, j).truncate(), metric);
        (d.perp_dot(v), d.dot(v), j)
    };
    let mut candidates = Vec::new();
    for k in 0..256 {
        let mut lo = TAU * (k as f64) / 256.;
        let mut hi = TAU * ((k + 1) as f64) / 256.;
        let sign = on_ray(lo).0;
        if sign * on_ray(hi).0 > 0. {
            continue;
        }
        for _ in 0..55 {
            let t = 0.5 * (lo + hi);
            if sign * on_ray(t).0 > 0. {
                lo = t;
            } else {
                hi = t;
            }
        }
        let (_, along, j) = on_ray(0.5 * (lo + hi));
        if along > 0. && j.z > 0. && !candidates.iter().any(|p: &DVec3| p.distance(j) < 1e-9) {
            candidates.push(j);
        }
    }
    match candidates.as_slice() {
        [j] => Ok(*j),
        _ => Err("ambiguous predicted projected-row solution".into()),
    }
}
fn w_at(r: DVec3) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            f64::from(i == j) * (0.5 + r.length_squared() / 0.05) - r[i] * r[j] / 0.05
        })
    })
}
fn source_at(seed: Seed, mu: f64, single: bool, policy: &str) -> String {
    // Keep every rotated sole over the finite pad interior.
    let s = seed
        .scene(mu, single, policy)
        .replace("0.66 0.36 0.12", "0.82 0.82 0.12");
    if policy == "project" {
        s.replace(
            "</support>",
            "</support><project_corner>true</project_corner>",
        )
    } else {
        s
    }
}
fn measure(seed: Seed, mu: f64, single: bool, pass: u32, ideal: DVec3, k: usize) -> Result<Value> {
    let proj = pass == 0;
    let mut sim = load(&source_at(seed, mu, single, "ordinary"), true)?;
    let r = ids(&sim)?;
    let mut owner = sim.world_mut().resource_mut::<PhysicsBackend>();
    let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
    let actual = Seed::read(departure::ingest(n, &r)).state;
    if near(actual, seed.state) > 1e-12 {
        return Err("single-point initial state mismatch".into());
    }
    let mut quadrants = [0.; 4];
    let impulse = if proj {
        let i = n
            .project_corner_impact(r.foot, r.pad)?
            .ok_or("cold locked corner projection declined")?;
        quadrants[k] = DVec3::from(i.normal).dot(DVec3::from(i.impulse_on_foot));
        DVec3::from(i.impulse_on_foot)
    } else {
        let mut j = DVec3::ZERO;
        let mut error = None;
        n.observe_contact_interval(
            Duration::from_nanos(1),
            1,
            ContactSolverIterations {
                temporal: 4,
                pgs: Some(pass as usize),
                relaxation: Some(pass as usize),
            },
            |s| match moments(&s.contact_patches(), &r, actual.q, single) {
                Ok((v, _, p)) => {
                    j = v;
                    quadrants = p;
                }
                Err(e) => error = Some(e),
            },
        )?;
        if let Some(e) = error {
            return Err(e);
        }
        j
    };
    if quadrants
        .iter()
        .enumerate()
        .any(|(i, v)| i != k && *v > 1e-12)
    {
        return Err("another corner acquired impulse in isolated sliding trial".into());
    }
    let endpoint = Seed::read(departure::ingest(n, &r)).state;
    let arm = actual.q * local_arms()[k];
    let post = endpoint.v + endpoint.w.cross(arm);
    let v = post.truncate();
    let jt = impulse.truncate();
    let angle = if v.length() > 1e-7 && jt.length() > 1e-7 {
        Some(v.perp_dot(-jt).abs().atan2(v.dot(-jt)).to_degrees())
    } else {
        None
    };
    Ok(
        json!({"single_rigid":single,"passes":pass,"projected":proj,"impulse_ns":impulse.to_array(),"post_contact_velocity_m_s":post.to_array(),
 "coulomb_slip_misalignment_deg":angle,"physical_impulse_error_ns":(impulse-ideal).length(),"point_normal_velocity_m_s":post.z,"normal_by_corner_ns":quadrants,
 "net_angular_impulse_nms":((endpoint.w-actual.w)*0.05).to_array(),"angular_balance_nms":((endpoint.w-actual.w)*0.05-arm.cross(impulse)).length()}),
    )
}
pub(super) fn run_all(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = vec![];
    for (material, mu) in [(None, 0.5), (Some(0.2), 0.2), (Some(0.8), 0.8)] {
        let (_, donor) = donor(material)?;
        let onset = oracle::solve(donor.state, mu)?;
        let k = onset.corner;
        for (slip_label, velocity_offset) in [
            ("measured", DVec3::ZERO),
            ("along-x", DVec3::new(0.9, 0., 0.)),
            ("oblique", DVec3::new(-0.65, 0.85, 0.)),
            ("reverse", DVec3::new(0.65, -0.85, 0.)),
        ] {
            for yaw_deg in [0., 23., 47., 71., 90., 139.] {
                let yaw = DQuat::from_rotation_z(f64::to_radians(yaw_deg));
                let c = PoseState {
                    c: yaw * onset.incoming.c,
                    q: yaw * onset.incoming.q,
                    v: yaw * (onset.incoming.v + velocity_offset),
                    w: yaw * onset.incoming.w,
                };
                let mut pad = donor.pad;
                pad.position =
                    (DVec3::from(pad.position) + DVec3::from(pad.velocity) * onset.time).into();
                let seed = Seed { state: c, pad };
                let arm = c.q * local_arms()[k];
                let w = w_at(arm);
                let u = c.v + c.w.cross(arm);
                let solution = PointImpact {
                    inverse_mass: w,
                    relative_velocity: u.to_array(),
                    friction: mu,
                }
                .solve()?;
                let physical = DVec3::from(solution.impulse);
                let polar = surface(w, u, mu, physical, Metric::Physical)?;
                if (polar - physical).length() > 1e-10 {
                    return Err("independent polar and mobility physical laws disagree".into());
                }
                let diagonal = surface(w, u, mu, physical, Metric::Diagonal)?;
                let coupled = surface(w, u, mu, physical, Metric::Coupled)?;
                let mut native = vec![];
                for (single, passes) in [(false, 8), (false, 32), (false, 0), (true, 8), (true, 32)]
                {
                    native.push(measure(seed, mu, single, passes, physical, k)?);
                }
                rows.push(json!({"mu":mu,"slip":slip_label,"yaw_deg":yaw_deg,"corner":k,"r":arm.to_array(),"inverse_mass":w,"relative_incoming_m_s":u.to_array(),"physical_sticking":solution.sticking,"physical_j_ns":physical.to_array(),"polar_j_ns":polar.to_array(),"diagonal_j_ns":diagonal.to_array(),"coupled_j_ns":coupled.to_array(),"native":native}));
            }
        }
    }
    fs::write(out.join("law.json"), serde_json::to_vec(&rows)?)?;
    produce_view(out)?;
    Ok(())
}
fn produce_view(out: &Path) -> Result<()> {
    // Keep the unique corner and a finite post-impact opening for the recorded
    // interval. The oblique rigid motion gives a visible frictional roll response.
    const SDF: &str =
        include_str!("../../../../../examples/multifidelity_locked_sliding_corner.sdf");
    let mut results = vec![];
    for policy in ["ordinary", "relax8", "project"] {
        let mut xml = SDF.to_string();
        if policy == "ordinary" {
            let a = xml
                .find("    <plugin name=\"gz::sim::systems::PrimaryContactRefinement\"")
                .unwrap();
            let b = a + xml[a..].find("</plugin>").unwrap() + "</plugin>".len();
            xml.replace_range(a..b, "");
        } else if policy == "relax8" {
            xml = xml.replace("<project_corner>true</project_corner>", "");
        }
        if policy == "project" {
            fs::write(out.join("scene.sdf"), &xml)?;
        }
        let mut sim = load(&xml, true)?;
        let r = ids(&sim)?;
        let mut frames = vec![];
        let mut events = vec![];
        for tick in 0..=25 {
            if tick > 0 {
                sim.step_exact(1)?;
            }
            let (f, u, p) = departure::ingest(engine(&sim), &r);
            let e = last_refinement(&sim).filter(|e| e.iteration == tick);
            let loaded = if let Some(e) = e {
                e.requested_pairs.iter().map(|p| p.normal_impulse).sum()
            } else {
                engine(&sim)
                    .rigid_contact_observations()
                    .iter()
                    .map(|p| p.normal_impulse)
                    .sum::<f64>()
            };
            if let Some(e) = e {
                events.push(json!({"tick":tick,"native_calls":e.interval.native_steps,"projected":e.interval.corner_impacts.iter().map(|p|json!({"at_ns":p.elapsed.as_nanos() as u64,"j":p.impulse_on_foot,"post":p.after_relative_velocity})).collect::<Vec<_>>()}));
            }
            frames.push(json!({"tick":tick,"time_us":tick*500,"native_steps":if tick==0{0}else{e.map_or(1,|e|e.interval.native_steps)},"solver_iterations":4,"selected":e.is_some(),
       "weak_start":false,"secondary_start":false,"foot":wire(f),"upper":wire(u.unwrap()),"pad":wire(p),"normal_impulse_ns":loaded,"gap_m":f.position[2]-p.position[2]-0.06-q_up(f).abs().dot(DVec3::new(0.18,0.10,0.06))}));
        }
        if policy == "project"
            && !events
                .iter()
                .any(|e| e["projected"].as_array().unwrap().len() == 1)
        {
            return Err("authored locked tree lost scheduled projection".into());
        }
        let geometry = json!({"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]});
        fs::write(
            out.join(format!("traces/{policy}.json")),
            serde_json::to_vec(&json!({"frames":frames,"geometry":geometry}))?,
        )?;
        results.push(json!({"policy":policy,"events":events}));
    }
    fs::write(
        out.join("public.json"),
        serde_json::to_vec_pretty(&results)?,
    )?;
    let lanes = [
        ("ordinary", "Ordinary locked foot"),
        ("relax8", "Eight native calls"),
        ("project", "Projected Coulomb corner"),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"Oblique sliding at a locked corner","pair_totals_only":true,"capture_ticks":{"first_impulse":5},"lanes":lanes}),
        )?,
    )?;
    Ok(())
}
