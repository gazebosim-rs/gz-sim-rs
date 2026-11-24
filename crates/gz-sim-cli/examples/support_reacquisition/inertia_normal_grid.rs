// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare native grid placement on the same 500 µs public clock without projected impulses.
use super::super::super::{native::at_foot, views::snap};
use super::*;
use gz_physics_multifidelity::ContactSegment;
use std::time::Instant;
const DT: u64 = 500_000;
#[derive(Clone, Copy)]
struct Grid {
    calls: u32,
    prefix: u64,
}
impl Grid {
    fn segments(self) -> Vec<ContactSegment> {
        let make = |t, c| ContactSegment {
            duration: Duration::from_nanos(t),
            native_steps: c,
            iterations: ContactSolverIterations {
                temporal: 4,
                pgs: Some(1),
                relaxation: Some(16),
            },
        };
        if self.prefix == 0 {
            vec![make(DT, self.calls)]
        } else {
            let fine = self.calls / 2;
            vec![
                make(self.prefix, fine),
                make(DT - self.prefix, self.calls - fine),
            ]
        }
    }
}
fn solve(raw: &str, grid: Grid, ticks: usize) -> crate::Result<(u64, Vec<Value>)> {
    let (mut sim, ids) = load(raw, false)?;
    let roof = id_named(&sim, "default::opponent::gate")?;
    let segments = grid.segments();
    let mut physics = sim.world_mut().resource_mut::<PhysicsBackend>();
    let native = physics.downcast_mut::<MultiFidelityBackend>().unwrap();
    let mut frames = vec![];
    let mut duration = 0;
    for k in 0..=ticks {
        let (loads, calls) = if k == 0 {
            ([0.; 2], 0)
        } else {
            let stamp = Instant::now();
            let (out, _) =
                native.step_contact_schedule(Duration::from_nanos(DT), &segments, &[])?;
            if k == 1 {
                duration = stamp.elapsed().as_nanos() as u64;
            }
            assert_eq!(out.native_steps, grid.calls);
            (
                [ids.pad, roof].map(|s| {
                    out.pairs
                        .iter()
                        .filter(|p| (p.a, p.b) == (ids.foot.min(s), ids.foot.max(s)))
                        .map(|p| p.normal_impulse)
                        .sum()
                }),
                out.native_steps,
            )
        };
        let foot = native.state(ids.foot).unwrap();
        let floor = native.state(ids.pad).unwrap();
        let ceiling = native.state(roof).unwrap();
        let f = loads.map(|total| {
            let mut v = snap(
                k as u64,
                DT,
                calls,
                foot,
                floor,
                ceiling,
                [total, 0., 0., 0.],
            );
            v.as_object_mut().unwrap().remove("corner_ns");
            v
        });
        frames.push(json!({"data":f,"loads":loads}));
    }
    Ok((duration, frames))
}
pub(super) fn survey(
    i: &Impact,
    g: &Gate,
    l: Crossing,
    t: &terminal::Terminal,
    feature: Option<&Path>,
) -> crate::Result<Value> {
    let raw = xml(i, g, l, DT, false);
    let free = long_period::certify(&i.r, faces(&i.r, g), l, t);
    let mut totals = t.bin(0., DT as f64 * 1e-9);
    totals[0] += l.law.impulse;
    let expected = [totals[0].dot(i.r.n), totals[1].dot(-i.r.n)];
    let mut settings = vec![Grid {
        calls: 1,
        prefix: 0,
    }];
    for calls in [2, 4, 8, 16, 32] {
        for prefix in [0, 64, 256, 1024, 4096, 16384] {
            settings.push(Grid { calls, prefix });
        }
    }
    let mut rows = vec![];
    let mut repeats = vec![Vec::new(); settings.len()];
    for pass in 0..=4 {
        // Alternate order, including a warmup per setting. Measure only the native call.
        for j in 0..settings.len() {
            let index = if pass % 2 == 0 {
                j
            } else {
                settings.len() - 1 - j
            };
            let grid = settings[index];
            let (ns, frames) = solve(&raw, grid, 1)?;
            if pass == 0 {
                rows.push(json!({"calls":grid.calls,"prefix_ns":grid.prefix,"loads":frames[1]["loads"],"after":frames[1]["data"][0]["foot"]}));
            } else {
                assert_eq!(rows[index]["after"], frames[1]["data"][0]["foot"]);
                assert_eq!(rows[index]["loads"], frames[1]["loads"]);
                repeats[index].push(ns);
            }
        }
    }
    for (row, measured) in rows.iter_mut().zip(repeats) {
        row["wall_ns"] = json!(measured);
    }
    if let Some(dir) = feature {
        let geometry = json!({"foot_half_extents":FOOT_HALF,"foot_com_local":ARM.to_array(),"pad_half_extents":PAD_HALF,"opponent_half_extents":HALF_GATE});
        let mut lanes = vec![];
        for (lane, label, prefix) in [
            ("uniform", "Four equally spaced native solves", 0),
            ("leading", "Two native solves in the first 256 ns", 256),
            ("analytic", "Independent free continuation", 0),
        ] {
            let mut entry = json!({"label":lane,"name":label,"file":format!("traces/{lane}.json")});
            let frames = if lane == "analytic" {
                entry["analytical"] = json!(true);
                let mut list = vec![];
                for (k, x) in std::iter::once(l.before)
                    .chain(free.states.iter().map(|a| a.0))
                    .enumerate()
                {
                    let at = l.at + (k as u64 * DT) as f64 * 1e-9;
                    let p = super::super::super::views::pose;
                    let floor = p(i.r.pad_v * at - i.r.n * PAD_HALF[2], i.r.pad_q, i.r.pad_v);
                    let ceiling = p(g.center + g.velocity * at, i.r.pad_q, g.velocity);
                    let data = expected.map(|j| {
                        let mut f = snap(
                            k as u64,
                            DT,
                            0,
                            at_foot(&i.r, x),
                            floor,
                            ceiling,
                            [if k == 1 { j } else { 0. }, 0., 0., 0.],
                        );
                        f.as_object_mut().unwrap().remove("corner_ns");
                        f
                    });
                    list.push(json!({"data":data}));
                }
                list
            } else {
                solve(&raw, Grid { calls: 4, prefix }, free.states.len())?.1
            };
            for (site, d) in ["bottom", "top"].into_iter().enumerate() {
                let p = dir.join(d);
                fs::create_dir_all(p.join("traces"))?;
                let v = frames.iter().map(|x| &x["data"][site]).collect::<Vec<_>>();
                write_json(
                    &p.join(format!("traces/{lane}.json")),
                    &json!({"geometry":geometry,"frames":v}),
                )?;
            }
            lanes.push(entry);
        }
        for (k, d) in ["bottom", "top"].into_iter().enumerate() {
            write_json(
                &dir.join(d).join("comparison.json"),
                &json!({"title":if k==0{"Floor load with four native solves per actuator tick"}else{"Ceiling load with four native solves per actuator tick"},"public_dt_ns":DT,"single_body":true,"pair_totals_only":true,"capture_ticks":{"landing":1,"free_continuation":free.states.len()},"lanes":lanes}),
            )?;
            fs::write(dir.join(d).join("source.sdf"), &raw)?;
        }
        write_json(
            &dir.join("featured.json"),
            &json!({"id":i.r.id,"mode":MODES[i.mode],"expected":expected,"free_until_s":free.until,"four_calls":rows.iter().filter(|a|a["calls"]==4).collect::<Vec<_>>()}),
        )?;
    }
    Ok(
        json!({"free_until_s":free.until,"expected_first":expected,"reference_end":free.states.first().map(|x|body_view(at_foot(&i.r,x.0))),"rows":rows}),
    )
}
