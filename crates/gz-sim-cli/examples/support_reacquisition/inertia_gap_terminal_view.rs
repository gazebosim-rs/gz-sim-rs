// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Authored SDF restarts at the preceding primary-pad return.
use super::super::super::native::at_foot;
use super::super::super::views::{initial_sdf, pair_loads, pose, snap};
use super::*;
pub(super) fn write(
    path: &Path,
    i: &Impact,
    g: &Gate,
    mu: f64,
    lower: Crossing,
    s: &Terminal,
    early: Option<&Terminal>,
) -> crate::Result<()> {
    let dt = (s.offset * 1e9 / 3.4).ceil() as u64;
    let span = dt as f64 * 1e-9;
    let last = 8. * span;
    if last - s.offset > s.free || span <= 0. {
        return Err("display outside post-limit clearance".into());
    }
    let raw = initial_sdf(i, g, mu, lower.at, lower.after(), dt, false);
    let enabled = projected::returns::enroll(&projected::enable(&initial_sdf(
        i,
        g,
        mu,
        lower.at,
        lower.after(),
        dt,
        true,
    )));
    let geo = json!({"foot_half_extents":FOOT_HALF,"foot_com_local":ARM.to_array(),"pad_half_extents":PAD_HALF,"opponent_half_extents":HALF_GATE});
    let mut lanes = vec![];
    let mut metrics = vec![];
    for (name, label, xml) in [
        ("ordinary", "Ordinary SDF response", &raw),
        if early.is_some() {
            ("early", "Early bounded two-site closure", &raw)
        } else {
            (
                "enrolled",
                "Enrolled ceiling and lower-return response",
                &enabled,
            )
        },
        ("analytic", "Independent two-site impulse limit", &raw),
    ] {
        let analytic = name == "analytic" || name == "early";
        let trajectory = if name == "early" { early.unwrap() } else { s };
        let (mut sim, ids) = load(xml, false)?;
        let ceiling = id_named(&sim, "default::opponent::gate")?;
        let store = Arc::new(Mutex::new(Vec::new()));
        if name == "enrolled" {
            let slot = store.clone();
            set_refined_observer(&mut sim, move |_, s| {
                let mut loads = pair_loads(
                    ids,
                    ceiling,
                    s.state(ids.foot).unwrap(),
                    s.contact_patches(),
                    s.rigid_point_impact,
                    s.opposing_corner_impact,
                );
                if let Some(r) = s.opposing_return_impact {
                    let h = r.lower.point;
                    let j = DVec3::from(h.impact.impulse_on_foot).dot(h.impact.normal.into());
                    loads[0].corners[usize::from(h.corner)] += j;
                }
                slot.lock().unwrap().push(loads);
            })?;
        }
        let mut frames: [Vec<Value>; 2] = [vec![], vec![]];
        let mut numbers = vec![];
        for tick in 0..=8 {
            let d = tick as f64 * span;
            let t = lower.at + d;
            let x = s.state(&i.r, lower, d);
            let mut quadrants = [[0.; 4]; 2];
            let (floor, roof, foot, calls) = if analytic {
                let js = if tick > 0 {
                    if d >= trajectory.offset && d < trajectory.offset + trajectory.gap_time {
                        return Err("public boundary intersects unresolved impact times".into());
                    }
                    trajectory.bin(d - span, d)
                } else {
                    [DVec3::ZERO; 2]
                };
                for j in 0..2 {
                    quadrants[j][s.corner[j] % 4] = js[j].dot(if j == 0 { i.r.n } else { -i.r.n });
                }
                (
                    pose(i.r.pad_v * t - i.r.n * PAD_HALF[2], i.r.pad_q, i.r.pad_v),
                    pose(g.center + g.velocity * t, i.r.pad_q, g.velocity),
                    at_foot(&i.r, trajectory.state(&i.r, lower, d)),
                    0,
                )
            } else {
                if tick > 0 {
                    sim.step_exact(1)?;
                }
                let e = backend(&sim);
                let foot = e.state(ids.foot).unwrap();
                let mut samples = std::mem::take(&mut *store.lock().unwrap());
                if tick > 0 && samples.is_empty() {
                    let record = gz_sim_scene::primary_contact::last_opposing_corner(&sim)
                        .filter(|h| h.sim_time == sim.elapsed());
                    let mut w = pair_loads(
                        ids,
                        ceiling,
                        foot,
                        e.rigid_contact_patches(),
                        None,
                        record.and_then(|r| r.interval.opposing_corner_impacts.first().copied()),
                    );
                    if let Some(r) = record.and_then(|r| r.interval.opposing_return_impacts.first())
                    {
                        let h = r.lower.point;
                        let j = DVec3::from(h.impact.impulse_on_foot).dot(h.impact.normal.into());
                        w[0].corners[usize::from(h.corner)] += j;
                    }
                    samples.push(w);
                }
                let calls = samples.len() as u32;
                for pair in samples {
                    for (j, w) in pair.iter().enumerate() {
                        for (k, load) in quadrants[j].iter_mut().enumerate() {
                            *load += w.corners[k] + w.corners[k + 4];
                        }
                    }
                }
                (
                    e.state(ids.pad).unwrap(),
                    e.state(ceiling).unwrap(),
                    foot,
                    calls,
                )
            };
            let m = Flight::from_state(&i.r, foot);
            for j in 0..2 {
                frames[j].push(snap(tick, dt, calls, foot, floor, roof, quadrants[j]));
            }
            numbers.push(json!({"tick":tick,"native_calls":calls,"pair_normal_ns":quadrants.map(|q|q.iter().sum::<f64>()),"com_error_m":m.com.distance(x.com),"spin_error_rad_s":DVec3::from(foot.angular_velocity).distance(i.r.body.omega(x.q,x.l))}));
        }
        let file = format!("traces/{name}.json");
        for (j, dir) in ["bottom", "top"].into_iter().enumerate() {
            let dest = path.join(dir);
            fs::create_dir_all(dest.join("traces"))?;
            write_json(
                &dest.join(&file),
                &json!({"geometry":geo,"frames":frames[j]}),
            )?;
            if !analytic {
                fs::write(dest.join(format!("{name}.sdf")), xml)?;
            }
        }
        let mut lane = json!({"label":name,"name":label,"file":file});
        if analytic {
            lane["analytical"] = json!(true);
        }
        lanes.push(lane);
        metrics.push(json!({"lane":name,"samples":numbers}));
    }
    for (j, dir) in ["bottom", "top"].into_iter().enumerate() {
        write_json(
            &path.join(dir).join("comparison.json"),
            &json!({"title":if early.is_some(){if j==0{"Bounding the lower impulse before the returns finish"}else{"Bounding the ceiling impulse before the returns finish"}}else if j==0{"Lower reaction through alternating impacts"}else{"Ceiling reaction through alternating impacts"},"public_dt_ns":dt,"single_body":true,"load_face":dir,"capture_ticks":{"impulse_limit":4,"free_release":8},"lanes":lanes}),
        )?;
    }
    write_json(
        &path.join("scores.json"),
        &json!({"id":i.r.id,"mode":MODES[i.mode],"gate_mu":mu,"start_ns":lower.at*1e9,"last_return_s":s.offset,"public_dt_ns":dt,"finite_offsets_s":s.finite.iter().map(|h|h.0).collect::<Vec<_>>(),"finite_impulses":s.finite.iter().map(|(_,h)|h.data()).collect::<Vec<_>>(),"frozen_tail_ns":s.impulses.map(|j|j.to_array()),"vector_radii_ns":s.radius,"final_bounds":s.bounds(&i.r,last-s.offset),"early":early.map(|e|json!({"finite_events":e.finite.len(),"offset_s":e.offset,"tail_ns":e.impulses.map(|j|j.to_array()),"radii":e.radius,"bounds":e.bounds(&i.r,last-e.offset)})),"lanes":metrics}),
    )
}
