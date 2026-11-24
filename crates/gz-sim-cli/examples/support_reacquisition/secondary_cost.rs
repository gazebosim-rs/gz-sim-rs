// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time the Bevy/SDF step itself. Every recorded public receipt is checked
//! against the independently archived two-, eight- and 64-call field.
use super::*;
fn quantile(mut v: Vec<u64>) -> Value {
    v.sort_unstable();
    let q = |p: usize| v[(v.len() * p).div_ceil(100) - 1];
    json!({"n":v.len(),"median_ns":q(50),"p95_ns":q(95),"p99_ns":q(99),"max_ns":v.last()})
}
pub(crate) fn write(source: &Path, dir: &Path) -> Result<()> {
    let mut by_class = [0; 2];
    let mut selected = vec![];
    // Balance the two mechanisms across both release histories, without
    // looking at policy times or at any subsequent trajectory.
    let mut counts = std::collections::BTreeMap::new();
    for k in 0..1250 {
        let doc: Value = serde_json::from_reader(fs::File::open(
            source.join("cases").join(format!("case-{k:04}.json")),
        )?)?;
        if doc["format"] != "finite-second-registered-work-v1" {
            return Err("expected registered pressure field".into());
        }
        let f = survey::fixture(k);
        for t in doc["result"]["trials"]
            .as_array()
            .ok_or("missing strokes")?
        {
            let res = &t["response"];
            let hit = &res["registered"][1]["projected"];
            if hit.is_null() {
                continue;
            }
            let added = hit["work_boxes"].as_u64().ok_or("missing pressure proof")? > 0;
            let stroke = t["stroke"].as_u64().ok_or("missing stroke")? as usize;
            let bin = counts.entry((added, f.switched, stroke)).or_insert(0);
            if *bin < 3 {
                selected.push((k, stroke, added, res["registered"].clone()));
                *bin += 1;
                by_class[usize::from(added)] += 1;
            }
        }
        if by_class == [18, 18] {
            break;
        }
    }
    if by_class != [18, 18] {
        return Err("incomplete balanced cost panel".into());
    }
    let mut groups = std::collections::BTreeMap::<(bool, usize), Vec<u64>>::new();
    let mut records = vec![];
    for (index, (k, stroke, new, archive)) in selected.into_iter().enumerate() {
        let f = survey::fixture(k);
        let cut = survey::cuts(f)?.0[stroke - 1].ok_or("missing cut")?;
        let (motion, _) = kinematics(f, &cut)?;
        let (_, hit) = locate(motion, true)?;
        let (e, _) = hit.ok_or("missing line")?;
        let bare = sdf(e, motion.support)?;
        let work = with_work(bare.clone());
        for rep in 0..14 {
            for j in 0..6 {
                let i = (index + rep + j) % 6;
                let enhanced = i % 2 == 1;
                let width = i / 2;
                let calls = [2, 8, 64][width];
                let mut time = 0;
                let got = registered_measured(
                    if enhanced { &work } else { &bare },
                    calls,
                    Some(&mut time),
                )?;
                let round: Value = serde_json::from_slice(&serde_json::to_vec(&got)?)?;
                if enhanced {
                    if round != archive[width] {
                        return Err("timing changed an archived scene receipt".into());
                    }
                } else if new {
                    if !round["projected"].is_null() || round["scheduled"] != true {
                        return Err("ordinary comparison changed enrollment".into());
                    }
                } else {
                    let mut expected = archive[width].clone();
                    expected["projected"]
                        .as_object_mut()
                        .unwrap()
                        .remove("work_boxes");
                    if round != expected {
                        return Err("timing changed a direct opening certificate".into());
                    }
                }
                if rep >= 2 {
                    groups.entry((new, i)).or_default().push(time);
                    records.push(json!({"case":k,"stroke":stroke,"needs_work":new,"calls":calls,"enhanced":enhanced,"rep":rep-2,"wall_ns":time}))
                }
            }
        }
    }
    let summary=groups.into_iter().map(|((new,i),v)|json!({"needs_work":new,"calls":([2,8,64][i/2]),"enhanced":i%2==1,"timing":quantile(v)})).collect::<Vec<_>>();
    fs::create_dir_all(dir)?;
    write_json(
        &dir.join("cost.json"),
        &json!({"measured":"sim.step_exact(1), SDF construction excluded","summary":summary,"rows":records}),
    )
}
