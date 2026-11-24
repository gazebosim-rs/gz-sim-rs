// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare entire original-clock histories to independently repeated nominal
//! controls, including three high-resolution grids and fresh native graphs.
use super::{
    internal::{Sample, sample, verify},
    *,
};

const NAMES: [&str; 7] = [
    "endpoint8_warm512",
    "launch2_warm512",
    "launch8_warm512",
    "retimed_launch8_warm512",
    "endpoint8_cold512",
    "launch8_cold512",
    "mechanics_cold512",
];

fn distance(series: &[Sample], reference: &[Sample]) -> (f64, f64) {
    let squared = series
        .iter()
        .zip(reference)
        .map(|(x, y)| x.com.distance_squared(y.com))
        .sum::<f64>();
    (
        (squared / series.len() as f64).sqrt(),
        series
            .last()
            .unwrap()
            .com
            .distance(reference.last().unwrap().com),
    )
}
pub(crate) fn summarize(
    dir: &Path,
    short: &Path,
    deep: &Path,
    previous: Option<&Path>,
) -> Result<()> {
    let compact = previous.is_some();
    let names = if compact {
        [
            "subdivided8_warm512",
            "subdivided2_warm512",
            "compact8_warm512",
            "compact64_warm512",
            "subdivided8_cold512",
            "compact8_cold512",
            "mechanics_cold512",
        ]
    } else {
        NAMES
    };
    let mut rows = Rows::new();
    let mut counts = BTreeMap::<String, usize>::new();
    let mut largest = None;
    for k in 0..50 {
        if !file(dir, k).exists() {
            continue;
        }
        let a = read(&file(dir, k))?;
        let dg = read(&file(deep, k))?;
        let d = &dg["result"];
        cross_checks(&a, d)?;
        if a["format"]
            != if compact {
                "compact-primary-three-stroke-v1"
            } else {
                "full-primary-three-stroke-v1"
            }
            || a["case"] != k
            || a["source"] != dg["source"]
        {
            return Err("incorrect primary source".into());
        }
        let j = 17 * k % 50;
        let donor = (0..1250)
            .find(|i| i * 613 % 1250 == 50 * (j / 2) + 24 + j % 2)
            .unwrap();
        let sc = read(&short.join("cases").join(format!("case-{donor:04}.json")))?;
        let t = &sc["result"]["trials"][0];
        if a["cut"] != t["cut"]
            || t["stroke"] != 1
            || !compact && a["mechanics"] != t["mechanics"][1]
        {
            return Err("short primary mechanical source changed".into());
        }
        let cut = uint(&a["cut_tick"])?;
        let n = uint(&a["end_tick"])? - cut;
        let receipts = if compact { [2, 0, 3, 5] } else { [3, 1, 5, 6] };
        let mut starts = [8, 2, 8, 8, 8, 8, 0];
        for (i, ix) in receipts.into_iter().enumerate() {
            starts[i] = uint(&t["paths"][ix]["first"]["native_steps"])?;
        }
        starts[5] = starts[2];
        let p = labels(&a["paths"], &names, n, &starts, &[512; 7])?;
        if !compact
            && (!repeated(&p[0], &d["paths"][4], false) || p[0]["first"] != d["paths"][4]["first"])
        {
            return Err("whole endpoint control changed".into());
        }
        for (i, ix) in receipts.into_iter().enumerate() {
            let mut prior = t["paths"][ix].clone();
            prior
                .as_object_mut()
                .ok_or("missing short receipt")?
                .remove("profile");
            if prior != p[i]["contact_receipt"] || prior["first"] != p[i]["first"] {
                return Err("short primary first differs".into());
            }
        }
        if p[0]["first"] != p[4]["first"] || p[2]["first"] != p[5]["first"] {
            return Err("cold restart first differs".into());
        }
        let file = format!("case-{k:03}.json");
        let traj = read(&dir.join("trajectories").join(&file))?;
        let grid = read(&deep.join("trajectories").join(file))?;
        if traj["first_tick"] != cut + 1 || traj["profiles"] != json!(names) {
            return Err("wrong original clock".into());
        }
        if let Some(prev) = previous {
            let other = read(&super::file(prev, k))?;
            cross_checks(&a, &other)?;
            let prior = read(&prev.join("trajectories").join(format!("case-{k:03}.json")))?;
            if other["mechanics"] != a["mechanics"]
                || !repeated(&p[6], &other["paths"][6], false)
                || prior["series"][6] != traj["series"][6]
            {
                return Err("mechanical cold graph changed".into());
            }
            for i in [2, 3] {
                let accepted = p[i]["contact_receipt"]["compact"].is_object();
                if accepted || i == 2 {
                    let repeat = if accepted { 1 } else { 0 };
                    if traj["series"][repeat] != traj["series"][i]
                        || !repeated(&p[i], &p[repeat], false)
                        || p[i]["first"] != p[repeat]["first"]
                    {
                        return Err(
                            "compact warm causal graph differs from exact chosen native grid"
                                .into(),
                        );
                    }
                }
                if accepted {
                    *counts.entry(format!("{}_accepted", names[i])).or_default() += 1;
                }
            }
        }
        let series = array(&traj["series"])?
            .iter()
            .map(|lane| array(lane)?.iter().map(sample).collect::<Result<Vec<_>>>())
            .collect::<Result<Vec<_>>>()?;
        if series.len() != 7 || series.iter().any(|l| l.len() != n as usize) {
            return Err("missing original-clock histories".into());
        }
        for (i, lane) in series.iter().enumerate() {
            for (sample, end) in [
                (lane.first().unwrap(), &p[i]["first"]),
                (lane.last().unwrap(), &p[i]["end"]),
            ] {
                close(sample.com.distance(com(end)?), 0.)?;
                close(
                    sample
                        .spin
                        .distance(vec3(&end["foot"]["angular_velocity"])?),
                    0.,
                )?;
            }
            close(
                lane.iter().map(|x| x.normal).sum(),
                num(&p[i]["outcomes"]["foot_normal_ns"])?,
            )?;
            close(
                lane.iter()
                    .map(|x| x.tangent)
                    .sum::<DVec3>()
                    .distance(vec3(&p[i]["outcomes"]["foot_tangent_ns"])?),
                0.,
            )?;
            for (r, idx) in [0, 2, 6].into_iter().enumerate() {
                let q = &p[i]["comparisons"][r];
                verify(lane, &series[idx], q, cut)?;
                let label = format!("{}_vs_{}", names[i], names[idx]);
                endpoint(&mut rows, &label, &p[i], &p[idx], k)?;
                store(&mut rows, &label, "rms_com_m", num(&q["rms_com_m"])?, k);
            }
        }
        for (ix, label) in [
            (0, "8192"),
            (1, "16384"),
            (2, "32768"),
            (4, "finite8_then512"),
        ] {
            let g = &grid["paths"][ix];
            if g["profile"] != label || array(&g["ticks"])?.len() != n as usize {
                return Err("wrong native fine trajectory".into());
            }
            let reference = array(&g["ticks"])?
                .iter()
                .enumerate()
                .map(|(i, v)| {
                    let [tick, c, w, n, t, _] = array(v)? else {
                        return Err("wrong deep sample".into());
                    };
                    if uint(tick)? != cut + 1 + i as u64 {
                        return Err("unmatched native clock".into());
                    }
                    Ok(Sample {
                        com: vec3(c)?,
                        spin: vec3(w)?,
                        normal: num(n)?,
                        tangent: vec3(t)?,
                    })
                })
                .collect::<Result<Vec<_>>>()?;
            if ix == 4 {
                if compact {
                    continue;
                }
                for (x, y) in reference.iter().zip(&series[0]) {
                    if x.com != y.com
                        || x.spin != y.spin
                        || x.normal != y.normal
                        || x.tangent != y.tangent
                    {
                        return Err("repeated warm public sample differs".into());
                    }
                }
                continue;
            }
            let base = distance(&series[0], &reference);
            for (i, lane) in series.iter().enumerate() {
                let d = distance(lane, &reference);
                for (key, z, b) in [("rms_com_m", d.0, base.0), ("final_com_m", d.1, base.1)] {
                    let id = format!("{}_vs_{label}", names[i]);
                    store(&mut rows, &id, key, z, k);
                    if i > 0 {
                        let rank = if z < b - 1e-12 {
                            "closer"
                        } else if z > b + 1e-12 {
                            "farther"
                        } else {
                            "tie"
                        };
                        *counts
                            .entry(format!(
                                "{id}_{key}_{rank}_than_{}",
                                if compact { "subdivided8" } else { "endpoint8" }
                            ))
                            .or_default() += 1;
                    }
                }
            }
        }
        let d = distance(&series[2], &series[0]).1;
        if largest.is_none_or(|(v, _)| d > v) {
            largest = Some((d, k))
        }
        *counts.entry("cases".into()).or_default() += 1;
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":counts.get("cases")==Some(&50),"counts":counts,"largest_launch8_vs_endpoint8_final_m_case":if compact{None}else{largest},"largest_compact_vs_subdivided_final_m_case":if compact{largest}else{None},"rows":rows_value(rows)}),
    )
}
