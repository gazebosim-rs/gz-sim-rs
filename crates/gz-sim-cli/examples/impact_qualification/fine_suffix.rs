// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Control the entire first landing and native graph before refining the suffix.
use super::{
    internal::{Sample, sample, verify},
    *,
};
const DONORS: [usize; 8] = [0, 7, 14, 21, 28, 35, 42, 47];
fn frame(v: &Value) -> Result<Sample> {
    Ok(Sample {
        com: com(v)?,
        spin: vec3(&v["foot"]["angular_velocity"])?,
        normal: num(&v["normal_impulse_ns"])?,
        tangent: vec3(&v["tangent_on_foot_ns"])?,
    })
}
pub(crate) fn summarize(dir: &Path, whole: &Path, deep: &Path) -> Result<()> {
    let mut rows = Rows::new();
    let mut cases = vec![];
    let mut largest = None;
    for k in DONORS {
        if !file(dir, k).exists() {
            continue;
        }
        let doc = read(&file(dir, k))?;
        let original = read(&file(whole, k))?;
        let dg = read(&file(deep, k))?;
        let dp = &dg["result"];
        cross_checks(&doc, &original)?;
        cross_checks(&doc, dp)?;
        if doc["format"] != "matched-fine-suffix-v1"
            || doc["case"] != k
            || doc["mechanics"] != original["mechanics"]
            || doc["source"] != dg["source"]
        {
            return Err("wrong physical intervention".into());
        }
        let cut = uint(&doc["cut_tick"])?;
        let end = uint(&doc["end_tick"])?;
        let dirk = dir.join(format!("case-{k:03}")).join("traces");
        let baseline = read(&whole.join("trajectories").join(format!("case-{k:03}.json")))?;
        let control = array(&baseline["series"][6])?;
        let warm = read(&deep.join("trajectories").join(format!("case-{k:03}.json")))?;
        let mut series = vec![];
        let mut warm_com = vec![];
        let mut crossings = vec![];
        for (i, n) in [512, 8192, 16384, 32768].into_iter().enumerate() {
            let q = &doc["paths"][i];
            let tr = read(&dirk.join(format!("native{n}.json")))?;
            let ticks = array(&tr["frames"])?;
            if q["calls_per_later_tick"] != n
                || q["native_calls"] != (end - cut - 1) * n
                || q["first"] != original["paths"][6]["first"]
                || ticks.len() as u64 != end - cut
                || ticks.first() != Some(&q["first"])
                || ticks.last() != Some(&q["end"])
            {
                return Err("bad suffix clock, state or call count".into());
            }
            let mut l = vec![];
            let mut wc = vec![];
            for (j, v) in ticks.iter().enumerate() {
                if v["tick"] != cut + 1 + j as u64
                    || v["native_steps"] != if j == 0 { 0 } else { n }
                {
                    return Err("nonuniform suffix clock".into());
                }
                let a = frame(v)?;
                if i == 0 {
                    let b = sample(&control[j])?;
                    if a.com != b.com
                        || a.spin != b.spin
                        || a.normal != b.normal
                        || a.tangent != b.tangent
                    {
                        return Err("cold 512 trajectory did not repeat".into());
                    }
                } else {
                    let prev = &warm["paths"][i - 1]["ticks"][j];
                    if prev[0] != v["tick"] {
                        return Err("unmatched uninterrupted clock".into());
                    }
                    wc.push(vec3(&prev[1])?);
                }
                l.push(a);
            }
            if i == 0
                && (q["end"] != original["paths"][6]["end"]
                    || q["outcomes"] != original["paths"][6]["outcomes"])
            {
                return Err("different 512 reactions".into());
            }
            close(
                l.iter().map(|v| v.normal).sum(),
                num(&q["outcomes"]["foot_normal_ns"])?,
            )?;
            if i > 0 {
                let d = l.last().unwrap().com.distance(*wc.last().unwrap());
                store(
                    &mut rows,
                    &format!("common_first_vs_uninterrupted_{n}"),
                    "final_com_m",
                    d,
                    k,
                );
            }
            series.push(l);
            warm_com.push(wc);
        }
        for i in 0..4 {
            verify(&series[i], &series[3], &doc["against_finest"][i], cut)?
        }
        for (i, j) in [(1, 2), (1, 3), (2, 3)] {
            let pair = format!(
                "{}_vs_{}",
                [512, 8192, 16384, 32768][i],
                [512, 8192, 16384, 32768][j]
            );
            let mut stats = vec![];
            for (name, values) in [
                (
                    "matched_first",
                    series[i]
                        .iter()
                        .zip(&series[j])
                        .map(|(x, y)| x.com.distance(y.com))
                        .collect::<Vec<_>>(),
                ),
                (
                    "uninterrupted",
                    warm_com[i]
                        .iter()
                        .zip(&warm_com[j])
                        .map(|(x, y)| x.distance(*y))
                        .collect::<Vec<_>>(),
                ),
            ] {
                let label = format!("{name}_{pair}");
                let at = [1e-9, 1e-7, 1e-6, 1e-4, 1e-3, 1e-2].map(|threshold| {
                    values
                        .iter()
                        .position(|&v| v > threshold)
                        .map(|idx| cut + 1 + idx as u64)
                });
                let terminal = *values.last().unwrap();
                let rms = (values.iter().map(|v| v * v).sum::<f64>() / values.len() as f64).sqrt();
                store(&mut rows, &label, "final_com_m", terminal, k);
                store(&mut rows, &label, "rms_com_m", rms, k);
                if i == 2
                    && j == 3
                    && name == "matched_first"
                    && largest.is_none_or(|(v, _)| terminal > v)
                {
                    largest = Some((terminal, k))
                }
                stats.push(json!({"control":name,"terminal_m":terminal,"rms_m":rms,"first_threshold_ticks":at}));
            }
            crossings.push(json!({"pair":pair,"controls":stats}));
        }
        cases.push(json!({"case":k,"cut":cut,"end_tick":end,"pairs":crossings}));
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":cases.len()==DONORS.len(),"donors":DONORS,"cases":cases,"largest_common_finest_final_m_case":largest,"thresholds_m":[1e-9,1e-7,1e-6,1e-4,1e-3,1e-2],"rows":rows_value(rows)}),
    )
}
