// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Expose four 64-body sites within the viewer bound, with all physical receipts accounted for.
use super::{Bodies, Rendered, Result, Transform, View, capture, floor, read};
use serde_json::json;
use std::{collections::BTreeSet, path::Path};
pub fn render(root: &Path, full: bool) -> Result<Rendered> {
    let dir = root.join("resident-clip");
    let proof = read(&dir.join("summary.json"))?;
    let (frames, digest) = capture(&dir.join("frames.jsonl"), 256, &[])?;
    if frames.len() != 21
        || proof["trajectory_sha256"] != digest
        || proof["body_states"] != 5376
        || proof["peak_resident_pair_receipts"] != 576
        || proof["peak_resident_floor_receipts"] != 64
        || proof["sphere_radius_m"] != 0.2
    {
        return Err("unexpected 256-body native qualification".into());
    }
    let mut panels = vec![];
    let mut ids: Vec<BTreeSet<u64>> = vec![];
    for i in 0..if full { 1 } else { 4 } {
        let scale = if full { 1. } else { 2.6 };
        let x = if full { 0. } else { [-5., 5., -5., 5.][i] };
        let y = if full {
            0.65
        } else {
            [3.2, 3.2, -2.6, -2.6][i]
        };
        let center = proof["groups"][i]["center_x_m"]
            .as_f64()
            .ok_or("missing site center")?;
        panels.push(Bodies::new(
            ["site0", "site1", "site2", "site3"][i],
            10_000 + i as u64 * 30_000,
            Transform {
                center: [center - x / scale, 0.],
                scale,
                offset_y: y,
            },
            256,
        ));
        let lo = 64 * i as u64 + 1;
        ids.push(if full {
            (lo..lo + 64).collect()
        } else {
            ((lo + 16)..lo + 32).collect()
        });
    }
    let mut stream = vec![];
    let mut samples = vec![];
    let mut n_pair = 0;
    let mut n_floor = 0;
    for f in &frames {
        let mut v = View::default();
        floor(
            &mut v,
            if full {
                [5., 5., 0.12]
            } else {
                [20., 16., 0.12]
            },
        );
        if f.bodies.iter().any(|b| b.size != [0.4; 3]) {
            return Err("unexpected physical radius".into());
        }
        for (i, p) in panels.iter_mut().enumerate() {
            p.draw(&mut v, f, None, Some(&ids[i]))?;
        }
        let base = 200_000 + f.tick * 96;
        let mut account = vec![];
        let group = |id: u64| ((id - 1) / 64) as usize;
        if f.pairs.iter().any(|p| group(p.a) != group(p.b)) {
            return Err("unexpected cross-site pair".into());
        }
        for i in 0..4 {
            let start = i * 64;
            let owner = f.owners[start];
            if f.owners[start..start + 64].iter().any(|&w| w != owner) {
                return Err("partial site ownership".into());
            }
            let pi: Vec<_> = f.pairs.iter().filter(|p| group(p.a) == i).collect();
            let fl: Vec<_> = f.floors.iter().filter(|p| group(p.body) == i).collect();
            let np = pi.len();
            let nf = fl.len();
            let positive = pi.iter().filter(|p| p.normal_impulse > 0.).count();
            let ground = fl.iter().filter(|p| p.normal_impulse > 0.).count();
            account.push(json!({"site":i,"owner":owner,"pairs":np,"positive_pairs":positive,"floors":nf,"positive_floors":ground}));
            if !full {
                let x = [-7.2, 2.8, -7.2, 2.8][i];
                let y = [0.8, 0.8, -5., -5.][i];
                for (j, (tag, count, cap)) in [("pair", np, 144), ("floor", nf, 16)]
                    .into_iter()
                    .enumerate()
                {
                    let part = if j == 0 { positive } else { ground };
                    let w = 4. * count as f64 / cap as f64;
                    v.cuboid(
                        base + 4 * i as u64 + 2 * j as u64,
                        &format!("site_{i}_owner_{owner}_{tag}_receipts_{count}_positive_{part}"),
                        [x + w / 2., y - 0.42 * j as f64, 0.21],
                        [w.max(0.05), 0.23, 0.26],
                        [[0.89, 0.76, 0.26, 1.], [0.15, 0.67, 0.46, 1.]][j],
                    );
                }
            }
        }
        let mut residency = vec![];
        for w in 0..f
            .components
            .as_array()
            .ok_or("missing native world info")?
            .len()
        {
            let n = f.owners.iter().filter(|&&o| o as usize == w).count();
            let pairs = f
                .pairs
                .iter()
                .filter(|p| f.owners[p.a as usize - 1] as usize == w)
                .count();
            let floors = f
                .floors
                .iter()
                .filter(|p| f.owners[p.body as usize - 1] as usize == w)
                .count();
            let worlds = f.components[w].as_array().ok_or("missing solver owners")?;
            if worlds
                .iter()
                .map(|c| c["ids"].as_array().unwrap().len())
                .sum::<usize>()
                != n
            {
                return Err("owned component membership differs from actual body map".into());
            }
            residency.push(
                json!({"worker":w,"bodies":n,"pairs":pairs,"floors":floors,"worlds":worlds.len()}),
            );
            if full {
                let length = 1.0 * n as f64 / 256.;
                v.cuboid(
                    base + 30 + 2 * w as u64,
                    &format!(
                        "w{w}_resident_{n}_bodies_{}_worlds_{pairs}_pairs_{floors}_floors",
                        worlds.len()
                    ),
                    [[-2., -0.5, 1.][w] + length / 2., -1.15, 0.12],
                    [length.max(0.025), 0.13, 0.2],
                    [
                        [0.14, 0.48, 0.86, 1.],
                        [0.98, 0.50, 0.11, 1.],
                        [0.62, 0.30, 0.76, 1.],
                    ][w],
                );
            }
        }
        let tag = proof["events"]
            .as_array()
            .unwrap()
            .iter()
            .rev()
            .find(|e| e["first_recorded_tick"].as_u64().unwrap() <= f.tick);
        let event = match tag.map(|e| e["committed_tick"].as_u64().unwrap()) {
            Some(5) => "after_barrier5_handoff_w0_to_w1_all_64_pending_impulse_body17".to_owned(),
            Some(7) => {
                let e = tag.unwrap();
                format!(
                    "after_barrier7_worker0_recovered_pid_{}_to_{}",
                    e["prior_pid"], e["replacement_pid"]
                )
            }
            Some(9) => "after_barrier9_full_pool_disk_resume_3_workers".to_owned(),
            _ => format!("native_tick_{}_w0_starts_with_256_resident_bodies", f.tick),
        };
        v.cuboid(
            base + 48,
            &event,
            if full {
                [0., -1.70, 0.13]
            } else {
                [0., 0., 0.26]
            },
            if full {
                [0.23, 0.17, 0.23]
            } else {
                [0.46, 0.3, 0.46]
            },
            [0.74, 0.53, 0.22, 1.],
        );
        n_pair += f.pairs.len();
        n_floor += f.floors.len();
        samples.push(json!({"tick":f.tick,"sites":account,"residency":residency,"event":event}));
        stream.push(v.finish(f.tick));
    }
    if proof["pair_receipts"] != n_pair || proof["floor_receipts"] != n_floor {
        return Err("incomplete physical receipt totals".into());
    }
    let pos = panels.into_iter().map(|p| p.transform).collect::<Vec<_>>();
    Ok((
        stream,
        json!({"mode":if full{"resident_handoff"}else{"resident_sites"},"summary":proof,"panels":pos,"sites":ids,"frames":samples.len(),"samples":samples}),
    ))
}
