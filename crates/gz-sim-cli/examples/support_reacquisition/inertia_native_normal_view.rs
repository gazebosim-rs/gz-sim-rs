// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Public Bevy updates, independently resolved half-space events and actual finite boxes.
use super::super::super::native::at_foot;
use super::super::super::views::{pair_loads, pose, snap};
use super::*;
pub(super) fn write(
    path: &Path,
    i: &Impact,
    g: &Gate,
    l: Crossing,
    dt: u64,
    term: &terminal::Terminal,
    initial: Option<(f64, Flight)>,
) -> crate::Result<()> {
    let (shift, incoming) = initial.unwrap_or((0., l.before));
    let h = dt as f64 * 1e-9;
    if h - shift <= term.offset + term.gap_time || 8. * h - shift > term.offset + term.free {
        return Err("replay extends outside validated reference".into());
    }
    let geometry = json!({"foot_half_extents":FOOT_HALF,"foot_com_local":ARM.to_array(),"pad_half_extents":PAD_HALF,"opponent_half_extents":HALF_GATE});
    let mut lanes = vec![];
    let mut readings = vec![];
    for (name, label) in [
        ("native", "Registered native contacts"),
        ("projected", "Bounded opposing normals"),
        ("analytic", "Independent finite-return reference"),
    ] {
        let analytic = name == "analytic";
        let raw = authored(i, g, l.at - shift, incoming, dt, name == "projected");
        let (mut sim, ids) = load(&raw, false)?;
        let ceiling = id_named(&sim, "default::opponent::gate")?;
        let mut frames: [Vec<Value>; 2] = [vec![], vec![]];
        let mut samples = vec![];
        for k in 0..=8 {
            let t = k as f64 * h - shift;
            let x = if k == 0 {
                incoming
            } else {
                term.state(&i.r, l, t)
            };
            let mut qs = [[0.; 4]; 2];
            let (foot, pad, roof, calls) = if analytic {
                if k > 0 {
                    let mut j = term.bin(t - h, t);
                    if k == 1 {
                        j[0] += l.law.impulse;
                    }
                    for side in 0..2 {
                        qs[side][term.corner[side] % 4] =
                            j[side].dot(if side == 0 { i.r.n } else { -i.r.n });
                    }
                }
                let at = l.at + t;
                (
                    at_foot(&i.r, x),
                    pose(i.r.pad_v * at - i.r.n * PAD_HALF[2], i.r.pad_q, i.r.pad_v),
                    pose(g.center + g.velocity * at, i.r.pad_q, g.velocity),
                    0,
                )
            } else {
                if k > 0 {
                    sim.step_exact(1)?;
                }
                let e = backend(&sim);
                let foot = e.state(ids.foot).unwrap();
                let rec = gz_sim_scene::primary_contact::last_opposing_corner(&sim)
                    .filter(|a| a.sim_time == sim.elapsed());
                let calls = if k == 0 {
                    0
                } else {
                    rec.map_or(1, |a| a.interval.native_steps)
                };
                if k > 0 {
                    if let Some(c) = rec.and_then(|a| a.interval.opposing_normal_series.first()) {
                        for (side, vertices) in qs.iter_mut().enumerate() {
                            vertices[c.corners[side] as usize] = c.normal_impulse[side];
                        }
                    } else {
                        let w = pair_loads(
                            ids,
                            ceiling,
                            foot,
                            e.rigid_contact_patches(),
                            None,
                            rec.and_then(|a| a.interval.opposing_corner_impacts.first().copied()),
                        );
                        for (side, vertices) in qs.iter_mut().enumerate() {
                            for (n, f) in vertices.iter_mut().enumerate() {
                                *f = w[side].corners[n] + w[side].corners[n + 4];
                            }
                        }
                    }
                    if let Some(r) = rec {
                        for (side, support) in [ids.pad, ceiling].into_iter().enumerate() {
                            let recorded = r
                                .interval
                                .pairs
                                .iter()
                                .filter(|p| {
                                    (p.a, p.b) == (ids.foot.min(support), ids.foot.max(support))
                                })
                                .map(|p| p.normal_impulse)
                                .sum::<f64>();
                            assert!((recorded - qs[side].iter().sum::<f64>()).abs() < 1e-9);
                        }
                    }
                }
                (
                    foot,
                    e.state(ids.pad).unwrap(),
                    e.state(ceiling).unwrap(),
                    calls,
                )
            };
            for side in 0..2 {
                frames[side].push(snap(k, dt, calls, foot, pad, roof, qs[side]));
            }
            let actual = Flight::from_state(&i.r, foot);
            samples.push(json!({"tick":k,"calls":calls,"normal":qs.map(|q|q.iter().sum::<f64>()),"com_error_m":actual.com.distance(x.com),"spin_error_rad_s":DVec3::from(foot.angular_velocity).distance(i.r.body.omega(x.q,x.l))}));
        }
        let file = format!("traces/{name}.json");
        for (side, dir) in ["bottom", "top"].into_iter().enumerate() {
            let p = path.join(dir);
            fs::create_dir_all(p.join("traces"))?;
            write_json(
                &p.join(&file),
                &json!({"geometry":geometry,"frames":frames[side]}),
            )?;
            if !analytic {
                fs::write(p.join(format!("{name}.sdf")), &raw)?;
            }
        }
        let mut lane = json!({"label":name,"name":label,"file":file});
        if analytic {
            lane["analytical"] = json!(true);
        }
        lanes.push(lane);
        readings.push(json!({"lane":name,"frames":samples}));
    }
    for (dir, title) in [
        (
            "bottom",
            if shift > 0. {
                "Timing the floor strike after the start of the tick"
            } else {
                "Measuring the floor reaction across opposing impacts"
            },
        ),
        (
            "top",
            if shift > 0. {
                "Resolving the ceiling return inside one nanosecond"
            } else {
                "Measuring the ceiling reaction across opposing impacts"
            },
        ),
    ] {
        write_json(
            &path.join(dir).join("comparison.json"),
            &json!({"single_body":true,"public_dt_ns":dt,"load_face":dir,"title":title,"capture_ticks":{"compound_landing":1,"free_release":8},"lanes":lanes}),
        )?;
    }
    write_json(
        &path.join("scores.json"),
        &json!({"id":i.r.id,"mode":MODES[i.mode],"dt_ns":dt,"start_to_first_lower_s":shift,"reference_offset_s":term.offset+shift,"free_after_s":term.free,"samples":readings}),
    )
}
