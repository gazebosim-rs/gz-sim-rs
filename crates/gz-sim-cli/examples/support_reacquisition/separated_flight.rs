// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the native timing path with and without the named separation guard.
use super::*;
pub(super) fn configured(r: &Physical, tree: bool, n: u32, guard: bool) -> (Profile, String) {
    let p = Profile {
        calls: n,
        prospective: true,
    };
    let mut text = scene::xml(r, tree, p);
    if guard {
        text = text.replace(
            "</prospective_timing>",
            "</prospective_timing><guard_separated_flight>true</guard_separated_flight>",
        );
    }
    (p, text)
}
pub(super) fn single_rigid_trace(path: &Path, label: &str, snapshots: &[Snapshot]) -> Result<()> {
    let mut frames = serde_json::to_value(snapshots)?;
    for frame in frames.as_array_mut().unwrap() {
        frame.as_object_mut().unwrap().remove("upper");
    }
    write_json(
        &path.join("traces").join(format!("{label}.json")),
        &json!({
            "geometry":{"foot_half_extents":FOOT_HALF,"pad_half_extents":PAD_HALF,"foot_com_local":ARM.to_array()},
            "frames":frames
        }),
    )
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut cases = vec![];
    for id in 0..CASES {
        let r = Physical::new(id);
        let proof = r.certify();
        if proof.reason.is_some() {
            continue;
        }
        let mut trials = vec![];
        for tree in [false, true] {
            for n in [4, 8] {
                for guard in [false, true] {
                    let (p, text) = configured(&r, tree, n, guard);
                    let (mut row, frames, _) = scene::run(&r, tree, p, &text)?;
                    row["guard_enabled"] = guard.into();
                    trials.push(row);
                    if id == 85 && !tree && n == 8 {
                        let name = if guard { "guarded" } else { "original" };
                        single_rigid_trace(path, name, &frames)?;
                        fs::write(path.join(format!("{name}.sdf")), text)?;
                    }
                }
            }
        }
        if id == 85 {
            single_rigid_trace(path, "analytic", &r.trace())?;
        }
        cases.push(json!({"id":id,"trials":trials}));
    }
    write_json(
        &path.join("guard.json"),
        &json!({"cases":cases,"view_id":85}),
    )?;
    write_json(
        &path.join("comparison.json"),
        &json!({"single_body":true,"title":"Premature rigid impact and certified separation","lanes":[
        {"label":"original","name":"Original prospective timing","file":"traces/original.json"},
        {"label":"guarded","name":"Guarded prospective timing","file":"traces/guarded.json"},
        {"label":"analytic","name":"Euler flight and plastic impact","file":"traces/analytic.json","analytical":true}]}),
    )
}
pub(crate) fn cost(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut out = vec![];
    for id in 0..CASES {
        let r = Physical::new(id);
        if r.certify().reason.is_some() {
            continue;
        }
        for tree in [false, true] {
            let mut variants = vec![];
            for n in [4, 8] {
                for guard in [false, true] {
                    let (p, text) = configured(&r, tree, n, guard);
                    let end = scene::run(&r, tree, p, &text)?.2;
                    scene::measure(&text, tree, end)?;
                    variants.push((p, guard, text, end));
                }
            }
            let mut samples: [Vec<[u64; 8]>; 4] = Default::default();
            for rep in 0..8 {
                for pos in 0..4 {
                    let k = (rep + pos) % 4;
                    let k = if (rep / 4 + id) % 2 == 0 { k } else { 3 - k };
                    let (_, _, text, end) = &variants[k];
                    samples[k].push(scene::measure(text, tree, *end)?);
                }
            }
            for ((p, guard, _, _), raw) in variants.into_iter().zip(samples) {
                out.push(json!({"id":id,"locked":tree,"calls":p.calls,"guard_enabled":guard,"primary_tick":(r.arrival*2000.).ceil()as usize,"tick_ns":raw}));
            }
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":out}))
}
