// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_multifidelity::EdgeImpactReceipt;
pub(super) struct Replay<'a> {
    pub title: &'a str,
    pub refinement_name: &'a str,
    pub registered_name: &'a str,
    pub capture_name: &'a str,
    pub capture_tick: usize,
    pub impact_tick: usize,
    pub last_tick: u64,
    pub sdf: &'a str,
}
pub(super) fn record(
    out: &Path,
    spec: Replay<'_>,
    source: impl Fn(&str) -> String,
    site_reaction: impl Fn(&EdgeImpactReceipt, &mut [f64; 4]) -> Result<(Duration, Value)>,
) -> Result<()> {
    let mut all = vec![];
    for label in ["ordinary", "refined", "edge"] {
        let src = source(label);
        let mut sim = load(&src, true)?;
        let id = ids(&sim)?;
        let loads = std::sync::Arc::new(std::sync::Mutex::new([0f64; 4]));
        if label != "ordinary" {
            let cap = loads.clone();
            gz_sim_scene::primary_contact::set_refined_observer(&mut sim, move |_, sub| {
                let q = DQuat::from_array(sub.state(id.foot).unwrap().orientation);
                let per = moments(&sub.contact_patches(), &id, q, false).unwrap().2;
                let mut sum = cap.lock().unwrap();
                for i in 0..4 {
                    sum[i] += per[i];
                }
            })?;
        }
        let mut rows = vec![];
        let mut events = vec![];
        let mut smooth = 0usize;
        let mut smooth_last = 0;
        for tick in 0..=spec.last_tick {
            let mut sites = [0.; 4];
            let mut steps = 0;
            if tick > 0 {
                *loads.lock().unwrap() = [0.; 4];
                sim.step_exact(1)?;
                if let Some(r) = last_refinement(&sim).filter(|r| r.iteration == tick) {
                    sites = *loads.lock().unwrap();
                    steps = r.interval.native_steps;
                    for h in &r.interval.two_bearing_edges {
                        smooth += 1;
                        smooth_last = tick;
                        for (id, j) in h.sole_vertices.into_iter().zip(h.normal_impulses) {
                            if id >= 4 {
                                return Err("replay requires bottom sole bearing sites".into());
                            }
                            sites[usize::from(2 * (id & 1) + (id >> 1))] += j;
                        }
                    }
                    for h in &r.interval.edge_impacts {
                        let (elapsed, mut data) = site_reaction(h, &mut sites)?;
                        data["tick"] = tick.into();
                        data["at_ns"] = ((r.start_time + elapsed).as_nanos() as u64).into();
                        data["normal_public_ns"] = r.requested_pairs[0].normal_impulse.into();
                        data["last_native_ns"] =
                            r.requested_pairs[0].last_native_normal_impulse.into();
                        events.push(data);
                    }
                    if (sites.iter().sum::<f64>() - r.requested_pairs[0].normal_impulse).abs()
                        > 1e-10
                    {
                        return Err("public interval reaction differs from quadrants".into());
                    }
                } else {
                    steps = 1;
                    let n = engine(&sim);
                    let q = DQuat::from_array(n.state(id.foot).unwrap().orientation);
                    sites = moments(&n.rigid_contact_patches(), &id, q, false)?.2;
                }
            }
            let (f, u, p) = departure::ingest(engine(&sim), &id);
            rows.push(json!({"tick":tick,"time_us":tick*500,"native_steps":steps,"solver_iterations":4,"selected":steps>1,"weak_start":false,"secondary_start":false,"normal_impulse_ns":sites.iter().sum::<f64>(),"corner_ns":sites,"foot":wire(f),"upper":wire(u.unwrap()),"pad":wire(p),"gap_m":f.position[2]-p.position[2]-0.06-q_up(f).abs().dot(DVec3::new(0.18,0.1,0.06))}));
        }
        if label == "edge" {
            fs::write(out.join(spec.sdf), &src)?;
        }
        fs::write(
            out.join(format!("traces/{label}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":rows}),
            )?,
        )?;
        all.push(json!({"label":label,"events":events,"first_tick_reaction_ns":rows[spec.impact_tick]["normal_impulse_ns"]}));
        if smooth > 0 {
            let row = all.last_mut().unwrap();
            row["two_bearing_segments"] = smooth.into();
            row["last_bearing_tick"] = smooth_last.into();
        }
    }
    let lanes = [
        ("ordinary", "Ordinary Bevy solve"),
        ("refined", spec.refinement_name),
        ("edge", spec.registered_name),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":spec.title,"capture_ticks":{(spec.capture_name):spec.capture_tick},"lanes":lanes}),
        )?,
    )?;
    fs::write(out.join("public.json"), serde_json::to_vec(&all)?)?;
    Ok(())
}
