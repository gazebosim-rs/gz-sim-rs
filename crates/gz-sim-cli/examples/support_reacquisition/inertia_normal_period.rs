// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Long-clock control from a physically classified two-site release.
use super::super::super::{
    native::at_foot,
    views::{pair_loads, pose, snap},
};
use super::*;
const DT: u64 = 500_000;
pub(super) struct FreeSpan {
    pub(super) states: Vec<(Flight, uncertainty::Error)>,
    pub(super) until: f64,
    lower_gap: f64,
    attempts: u32,
}
pub(super) fn certify(
    r: &Physical,
    faces: [Face; 2],
    l: Crossing,
    t: &terminal::Terminal,
) -> FreeSpan {
    let seed = (0.5 * t.free).max(t.gap_time);
    let b = t.bounds(r, seed);
    let mut e = uncertainty::Error {
        position: b["com_error_m"].as_f64().unwrap(),
        angle: b["angle_error_rad"].as_f64().unwrap(),
        velocity: t.speed_err,
        momentum: t.momentum_err,
        ..Default::default()
    };
    let mut x = t.cut;
    (x, e) = e.flight(
        r,
        x,
        Interval {
            lo: seed,
            mid: seed,
            hi: seed,
        },
    );
    let mut time = t.offset + seed;
    let mut min = f64::INFINITY;
    let mut attempts = 0;
    let mut states = vec![];
    for public in 1..=8 {
        let bound = public as f64 * DT as f64 * 1e-9;
        while time < bound {
            let mut h = (bound - time).min(8e-6);
            let minimum = loop {
                attempts += 1;
                let mut least = f64::INFINITY;
                for (side, p) in faces.into_iter().enumerate() {
                    for (k, arm) in vertices().into_iter().enumerate() {
                        let rr = arm.length();
                        let position = x.com + x.q * arm;
                        let raw = p.coordinates(position, l.at + time).z;
                        let gap = raw
                            - e.position
                            - e.angle * rr
                            - 256.
                                * f64::EPSILON
                                * (1.
                                    + position.length()
                                    + p.origin.length()
                                    + p.v.length() * (l.at + time));
                        let j = jet(r, x, Site { side, k }, p);
                        let v = j.v - j.veps - e.velocity(r, x, rr);
                        let a = j.a - j.aeps - e.accel(r, x, rr, h) - (j.jerk + j.jeps) * h;
                        let at = |h: f64| gap + h * (v + 0.5 * a * h);
                        let mut lowest = gap.min(at(h));
                        if a > 0. && v < 0. && -v / a < h {
                            lowest = lowest.min(at(-v / a));
                        }
                        least = least.min(lowest);
                    }
                }
                if least.is_finite() && least > 0. {
                    break least;
                }
                if h <= 1e-9 {
                    return FreeSpan {
                        states,
                        until: time,
                        lower_gap: min,
                        attempts,
                    };
                }
                h *= 0.5;
            };
            min = min.min(minimum);
            (x, e) = e.flight(
                r,
                x,
                Interval {
                    lo: h,
                    mid: h,
                    hi: h,
                },
            );
            time += h;
        }
        states.push((x, e));
    }
    FreeSpan {
        states,
        until: time,
        lower_gap: min,
        attempts,
    }
}
pub(super) fn survey(
    i: &Impact,
    g: &Gate,
    l: Crossing,
    t: &terminal::Terminal,
) -> crate::Result<Value> {
    let free = certify(&i.r, faces(&i.r, g), l, t);
    let mut reactions = t.bin(0., DT as f64 * 1e-9);
    reactions[0] += l.law.impulse;
    let totals = [reactions[0].dot(i.r.n), reactions[1].dot(-i.r.n)];
    let mut runs = vec![];
    for n in [1, 4, 8, 32, 64, 128] {
        for on in [false, true] {
            let xml = authored(i, g, l.at, l.before, DT, on).replace(
                "<native_steps>1</native_steps>",
                &format!("<native_steps>{n}</native_steps>"),
            );
            let (mut sim, ids) = load(&xml, false)?;
            let ceiling = id_named(&sim, "default::opponent::gate")?;
            let mut frames = vec![];
            for k in 0..=8 {
                if k > 0 {
                    sim.step_exact(1)?;
                }
                assert_eq!(sim.elapsed(), Duration::from_nanos(k * DT));
                let e = backend(&sim);
                let foot = e.state(ids.foot).unwrap();
                let r = gz_sim_scene::primary_contact::last_refinement(&sim)
                    .filter(|r| r.sim_time == sim.elapsed())
                    .map(|x| &x.interval)
                    .or_else(|| {
                        gz_sim_scene::primary_contact::last_opposing_corner(&sim)
                            .filter(|r| r.sim_time == sim.elapsed())
                            .map(|x| &x.interval)
                    });
                let mut qs = [[0.; 4]; 2];
                let loads = if k == 0 {
                    [0.; 2]
                } else if let Some(r) = r {
                    [ids.pad, ceiling].map(|s| {
                        r.pairs
                            .iter()
                            .filter(|p| (p.a, p.b) == (s.min(ids.foot), s.max(ids.foot)))
                            .map(|p| p.normal_impulse)
                            .sum()
                    })
                } else {
                    let p = pair_loads(ids, ceiling, foot, e.rigid_contact_patches(), None, None);
                    p.map(|v| v.corners.iter().sum())
                };
                // This wide-period dashboard uses pair totals only. Preserve
                // the raw patch quadrant labels exclusively in fine-cut studies.
                for (k, js) in qs.iter_mut().enumerate() {
                    js[0] = loads[k];
                }
                let calls = if k == 0 {
                    0
                } else {
                    r.map_or(1, |x| x.native_steps)
                };
                let snap = |q| {
                    snap(
                        k,
                        DT,
                        calls,
                        foot,
                        e.state(ids.pad).unwrap(),
                        e.state(ceiling).unwrap(),
                        q,
                    )
                };
                let mut pair_frames = qs.map(snap);
                for f in &mut pair_frames {
                    f.as_object_mut().unwrap().remove("corner_ns");
                }
                frames.push(json!({"data":pair_frames,"loads":loads,"requested":n,"calls":calls,"closed":r.map_or(0,|r|r.opposing_normal_series.len()),"failures":r.map(|r|r.opposing_normal_declines.iter().map(|v|format!("{:?}",v.stage)).collect::<Vec<_>>())}));
            }
            runs.push(json!({"n":n,"enabled":on,"frames":frames}));
        }
    }
    let origin = i.r.pad_v * l.at - i.r.n * PAD_HALF[2];
    let mut analytic = vec![];
    for (k, (x, e)) in std::iter::once((l.before, uncertainty::Error::default()))
        .chain(free.states.iter().copied())
        .enumerate()
    {
        let now = k as f64 * DT as f64 * 1e-9;
        let mut f = [0, 1].map(|site| {
            let mut j = [0.; 4];
            if k == 1 {
                j[0] = totals[site];
            }
            let mut v = snap(
                k as u64,
                DT,
                0,
                at_foot(&i.r, x),
                pose(origin + i.r.pad_v * now, i.r.pad_q, i.r.pad_v),
                pose(g.center + g.velocity * (l.at + now), i.r.pad_q, g.velocity),
                j,
            );
            v.as_object_mut().unwrap().remove("corner_ns");
            v
        });
        analytic.push(json!({"data":&mut f,"position_radius_m":e.position,"velocity_radius_m_s":e.velocity,"orientation_radius_rad":e.angle,"momentum_radius_nms":e.momentum}));
    }
    Ok(
        json!({"free_until_s":free.until,"minimum_clearance_after_seed_m":free.lower_gap,"geometry_checks":free.attempts,"initial_certified_s":t.free,"pair_reactions":totals,"pair_radii":t.radius,"analytic":analytic,"runs":runs}),
    )
}

pub(super) fn replay(
    path: &Path,
    i: &Impact,
    g: &Gate,
    l: Crossing,
    world: &Value,
) -> crate::Result<()> {
    let analytic = world["analytic"].as_array().unwrap();
    let count = analytic.len();
    assert!((2..=9).contains(&count));
    let geometry = json!({"foot_half_extents":FOOT_HALF,"foot_com_local":ARM.to_array(),"pad_half_extents":PAD_HALF,"opponent_half_extents":HALF_GATE});
    let mut lanes = vec![];
    for (name, label) in [
        ("native", "Registered 32-call budget"),
        ("projected", "Opposing normals, same 32-call budget"),
        (
            "analytic",
            "Certified free motion after impact accumulation",
        ),
    ] {
        let mut lane = json!({"label":name,"name":label,"file":format!("traces/{name}.json")});
        let frames = if name == "analytic" {
            lane["analytical"] = json!(true);
            analytic
        } else {
            &world["runs"]
                .as_array()
                .unwrap()
                .iter()
                .find(|x| x["n"] == 32 && x["enabled"] == (name == "projected"))
                .unwrap()["frames"]
                .as_array()
                .unwrap()[..count]
        };
        for (k, dir) in ["bottom", "top"].into_iter().enumerate() {
            let out = path.join(dir);
            fs::create_dir_all(out.join("traces"))?;
            let f = frames.iter().map(|x| &x["data"][k]).collect::<Vec<_>>();
            write_json(
                &out.join(format!("traces/{name}.json")),
                &json!({"geometry":geometry,"frames":f}),
            )?;
            if name != "analytic" {
                fs::write(
                    out.join(format!("{name}.sdf")),
                    authored(i, g, l.at, l.before, DT, name == "projected").replace(
                        "<native_steps>1</native_steps>",
                        "<native_steps>32</native_steps>",
                    ),
                )?;
            }
        }
        lanes.push(lane);
    }
    for (j, dir) in ["bottom", "top"].into_iter().enumerate() {
        write_json(
            &path.join(dir).join("comparison.json"),
            &json!({"title":if j==0{"Floor reaction on the 500 microsecond actuator clock"}else{"Ceiling reaction on the 500 microsecond actuator clock"},"pair_totals_only":true,"public_dt_ns":DT,"single_body":true,"capture_ticks":{"first_public_landing":1,"last_certified_update":count-1},"lanes":lanes}),
        )?;
    }
    write_json(
        &path.join("featured.json"),
        &json!({"id":i.r.id,"mode":MODES[i.mode],"certified_until_s":world["free_until_s"],"public_samples":count,"oracle_loads":world["pair_reactions"],"admitted_native_steps":world["runs"].as_array().unwrap().iter().find(|x|x["n"]==32&&x["enabled"]==true).unwrap()["frames"][1]}),
    )
}
