// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::super::native::{Wrench, at_foot};
use super::super::views::{initial_sdf, pair_loads, pose, snap};
use super::*;
pub(super) fn write(
    path: &Path,
    i: &Impact,
    g: &Gate,
    mu: f64,
    lower: Crossing,
    third: Crossing,
    other: Interval,
) -> crate::Result<()> {
    let sites = [Site::of(lower), Site::of(third)];
    let dt = 8u64;
    let h = dt as f64 * 1e-9;
    let start = third.at - 4. * h;
    assert!(start > lower.at);
    let certificate = open_after(i, faces(&i.r, g), sites, third, other, 4. * h)
        .ok_or("post-third display horizon inconclusive")?;
    let exact = |k: u64| {
        let t = start + k as f64 * h;
        let state = if k < 4 {
            advance(&i.r, lower.after(), t - lower.at)
        } else {
            advance(&i.r, third.after(), (k - 4) as f64 * h)
        };
        (t, state)
    };
    let plain = initial_sdf(i, g, mu, start, exact(0).1, dt, false);
    let enabled = projected::returns::enroll(&projected::enable(&initial_sdf(
        i,
        g,
        mu,
        start,
        exact(0).1,
        dt,
        true,
    )));
    let dir = path.join("visual");
    fs::create_dir_all(dir.join("traces"))?;
    fs::write(dir.join("ordinary.sdf"), &plain)?;
    fs::write(dir.join("enrolled.sdf"), &enabled)?;
    let geometry = json!({"foot_half_extents":FOOT_HALF,"foot_com_local":ARM.to_array(),"pad_half_extents":PAD_HALF,"opponent_half_extents":HALF_GATE});
    let mut lanes = vec![];
    let mut scores = vec![];
    for (lane, name, xml) in [
        ("ordinary", "Ordinary local solve", &plain),
        ("enrolled", "Enrolled upper and return policy", &enabled),
        ("analytic", "Independent released-point reference", &plain),
    ] {
        let (mut sim, ids) = load(xml, false)?;
        let gate = id_named(&sim, "default::opponent::gate")?;
        let seen = Arc::new(Mutex::new(Vec::new()));
        if lane == "enrolled" {
            let out = seen.clone();
            set_refined_observer(&mut sim, move |_, s| {
                let mut w = pair_loads(
                    ids,
                    gate,
                    s.state(ids.foot).unwrap(),
                    s.contact_patches(),
                    s.rigid_point_impact,
                    s.opposing_corner_impact,
                );
                if let Some(r) = s.opposing_return_impact {
                    let h = r.lower.point;
                    let impulse = DVec3::from(h.impact.impulse_on_foot).dot(h.impact.normal.into());
                    w[0].corners[usize::from(h.corner)] += impulse;
                    w[0].scalar += impulse;
                }
                out.lock().unwrap().push(w);
            })?;
        }
        let analytic = lane == "analytic";
        let mut frames = vec![];
        let mut metrics = vec![];
        for k in 0..=8 {
            let (t, x) = exact(k);
            let (f, p, b, w, below, calls) = if analytic {
                let mut j = [0.; 4];
                if k == 4 {
                    j[third.corner % 4] = third.law.impulse.dot(third.plane.frame * DVec3::Z);
                }
                (
                    at_foot(&i.r, x),
                    pose(i.r.pad_v * t - i.r.n * PAD_HALF[2], i.r.pad_q, i.r.pad_v),
                    pose(g.center + g.velocity * t, i.r.pad_q, g.velocity),
                    j,
                    0.,
                    0,
                )
            } else {
                if k > 0 {
                    sim.step_exact(1)?;
                }
                let e = backend(&sim);
                let f = e.state(ids.foot).unwrap();
                let mut samples = std::mem::take(&mut *seen.lock().unwrap());
                if k > 0 && samples.is_empty() {
                    let record = gz_sim_scene::primary_contact::last_opposing_corner(&sim)
                        .filter(|r| r.sim_time == sim.elapsed());
                    let mut w = pair_loads(
                        ids,
                        gate,
                        f,
                        e.rigid_contact_patches(),
                        None,
                        record.and_then(|r| r.interval.opposing_corner_impacts.first().copied()),
                    );
                    if let Some(r) = record.and_then(|r| r.interval.opposing_return_impacts.first())
                    {
                        let hit = r.lower.point;
                        let j =
                            DVec3::from(hit.impact.impulse_on_foot).dot(hit.impact.normal.into());
                        w[0].scalar += j;
                        w[0].corners[usize::from(hit.corner)] += j;
                    }
                    samples.push(w);
                }
                let n = samples.len() as u32;
                let mut total = Wrench::default();
                let mut other = 0.;
                for [pad, gate] in samples {
                    other += pad.scalar;
                    for (a, b) in total.corners.iter_mut().zip(gate.corners) {
                        *a += b;
                    }
                }
                (
                    f,
                    e.state(ids.pad).unwrap(),
                    e.state(gate).unwrap(),
                    std::array::from_fn(|j| total.corners[j] + total.corners[j + 4]),
                    other,
                    n,
                )
            };
            let m = Flight::from_state(&i.r, f);
            frames.push(snap(k, dt, calls, f, p, b, w));
            metrics.push(json!({"tick":k,"at_ns":t*1e9,"steps":calls,"lower_normal_ns":below,"upper_normal_ns":w.iter().sum::<f64>(),"com_m":m.com.distance(x.com),"spin_rad_s":DVec3::from(f.angular_velocity).distance(i.r.body.omega(x.q,x.l))}));
        }
        let file = format!("traces/{lane}.json");
        write_json(
            &dir.join(&file),
            &json!({"geometry":geometry,"frames":frames}),
        )?;
        let mut label = json!({"label":lane,"name":name,"file":file});
        if analytic {
            label["analytical"] = json!(true);
        }
        lanes.push(label);
        scores.push(json!({"lane":lane,"frames":metrics}));
    }
    write_json(
        &dir.join("comparison.json"),
        &json!({"title":"A third strike while the lower corner is separated","public_dt_ns":dt,"single_body":true,"load_face":"top","capture_ticks":{"third_ceiling_strike":4,"released":8},"lanes":lanes}),
    )?;
    write_json(
        &dir.join("scores.json"),
        &json!({"id":i.r.id,"mode":MODES[i.mode],"start_ns":start*1e9,"lower_at_ns":lower.at*1e9,"third":third.data(),"lower_clearance_at_third_m":other.data(),"post_release":certificate,"lanes":scores}),
    )
}
