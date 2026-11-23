// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently compare every saved public clock and check native repeats
//! against the original short, warm-graph and cold-graph campaigns.
use super::*;

const NAMES: [&str; 9] = [
    "native8_warm512",
    "bound8_warm512",
    "transfer8_warm512",
    "transfer2_warm512",
    "continuous4096",
    "continuous8192",
    "native8_cold512",
    "transfer8_cold512",
    "mechanics_cold512",
];
#[derive(Clone, Copy)]
pub(super) struct Sample {
    pub(super) com: DVec3,
    pub(super) spin: DVec3,
    pub(super) normal: f64,
    pub(super) tangent: DVec3,
}
pub(super) fn sample(v: &Value) -> Result<Sample> {
    let [c, w, n, t] = array(v)? else {
        return Err("missing trajectory component".into());
    };
    Ok(Sample {
        com: vec3(c)?,
        spin: vec3(w)?,
        normal: num(n)?,
        tangent: vec3(t)?,
    })
}
pub(super) fn verify(a: &[Sample], b: &[Sample], q: &Value, cut: u64) -> Result<()> {
    if a.len() != b.len() || a.is_empty() {
        return Err("different trajectory lengths".into());
    }
    let (mut sq, mut spin_sq, mut peak, mut peak_j, mut final_d, mut final_spin) =
        (0., 0., 0., 0.0_f64, 0., 0.);
    let mut first = [None; 4];
    let mut max_tick = 0;
    for (i, (x, y)) in a.iter().zip(b).enumerate() {
        let tick = cut + 1 + i as u64;
        final_d = x.com.distance(y.com);
        final_spin = x.spin.distance(y.spin);
        sq += final_d * final_d;
        spin_sq += final_spin * final_spin;
        peak_j = peak_j.max((x.normal - y.normal).abs());
        if final_d > peak {
            peak = final_d;
            max_tick = tick;
        }
        for (limit, at) in [1e-6, 1e-4, 1e-3, 1e-2].into_iter().zip(&mut first) {
            if final_d > limit && at.is_none() {
                *at = Some(tick)
            }
        }
    }
    for (key, v) in [
        ("rms_com_m", (sq / a.len() as f64).sqrt()),
        ("rms_spin_rad_s", (spin_sq / a.len() as f64).sqrt()),
        ("max_com_m", peak),
        ("final_com_m", final_d),
        ("final_spin_rad_s", final_spin),
        ("max_public_reaction_difference_ns", peak_j),
    ] {
        close(v, num(&q[key])?)?;
    }
    if json!(first) != q["first_above_1um_100um_1mm_10mm_ticks"]
        || uint(&q["max_tick"])? != max_tick
    {
        return Err("different peak or first divergence tick".into());
    }
    for c in array(&q["curve"])? {
        let i = uint(&c["public_ticks_after_cut"])? as usize;
        let (x, y) = (a.get(i - 1).ok_or("invalid curve clock")?, b[i - 1]);
        close(x.com.distance(y.com), num(&c["com_m"])?)?;
        close(x.spin.distance(y.spin), num(&c["spin_rad_s"])?)?;
    }
    Ok(())
}
pub(crate) fn summarize(dir: &Path, warm: &Path, cold: &Path, short: &Path) -> Result<()> {
    let mut counts = BTreeMap::<String, usize>::new();
    let mut rows = Rows::new();
    let mut largest = None;
    for k in 0..50 {
        if !file(dir, k).exists() {
            continue;
        }
        let (a, w, c) = (
            read(&file(dir, k))?,
            read(&file(warm, k))?,
            read(&file(cold, k))?,
        );
        let j = 17 * k % 50;
        let donor = (0..1250)
            .find(|i| i * 613 % 1250 == 50 * (j / 2) + 24 + j % 2)
            .unwrap();
        let s = read(&short.join("cases").join(format!("case-{donor:04}.json")))?;
        let t = &s["result"]["trials"][0];
        if a["case"] != k
            || a["format"] != "internal-contact-transfer-three-strokes-v1"
            || a["cut"] != t["cut"]
            || t["stroke"] != 1
        {
            return Err("wrong transfer donor".into());
        }
        cross_checks(&a, &w)?;
        cross_checks(&a, &c)?;
        for key in ["offset_ns", "initial"] {
            if a[key] != w[key] || a[key] != c[key] || a[key] != t[key] {
                return Err(format!("{k} different {key}").into());
            }
        }
        if a["mechanics"] != c["mechanics"] {
            return Err("different integrated reference".into());
        }
        *counts.entry("cases".into()).or_default() += 1;
        if a["paths"].is_null() {
            continue;
        }
        let cut = uint(&a["cut_tick"])?;
        let n = uint(&a["end_tick"])? - cut;
        let p = labels(
            &a["paths"],
            &NAMES,
            n,
            &[8, 8, 8, 2, 4096, 8192, 8, 8, 0],
            &[512, 512, 512, 512, 4096, 8192, 512, 512, 512],
        )?;
        for (i, other) in [
            (0, &w["paths"][0]),
            (4, &w["paths"][4]),
            (5, &w["paths"][5]),
            (6, &c["paths"][0]),
            (8, &c["paths"][4]),
        ] {
            if p[i]["first"] != other["first"] || !repeated(&p[i], other, false) {
                return Err("independent whole history changed".into());
            }
        }
        for (i, other) in [(1, 4), (2, 5), (3, 1)] {
            let prev = &t["paths"][other];
            if p[i]["first"] != prev["first"] {
                return Err("different short first boundary".into());
            }
            for key in ["spans", "transfers", "world_impulse_ns", "impulse"] {
                if p[i]["delivered"][key] != prev[key] {
                    return Err(format!("different short receipt {key}").into());
                }
            }
        }
        if p[0]["first"] != p[6]["first"] || p[2]["first"] != p[7]["first"] {
            return Err("cold start has different state".into());
        }
        let raw = read(&dir.join("trajectories").join(format!("case-{k:03}.json")))?;
        if raw["profiles"] != json!(NAMES) || raw["first_tick"] != cut + 1 {
            return Err("wrong trajectory origin".into());
        }
        let series = array(&raw["series"])?
            .iter()
            .map(|l| array(l)?.iter().map(sample).collect::<Result<Vec<_>>>())
            .collect::<Result<Vec<_>>>()?;
        if series.len() != 9 || series.iter().any(|x| x.len() != n as usize) {
            return Err("incomplete trajectories".into());
        }
        *counts.entry("matched".into()).or_default() += 1;
        for (i, lane) in series.iter().enumerate() {
            for (v, st) in [
                (&lane[0], &p[i]["first"]),
                (lane.last().unwrap(), &p[i]["end"]),
            ] {
                close(v.com.distance(com(st)?), 0.)?;
                close(v.spin.distance(vec3(&st["foot"]["angular_velocity"])?), 0.)?;
                close(v.normal, num(&st["normal_impulse_ns"])?)?;
            }
            close(
                lane.iter().map(|v| v.normal).sum(),
                num(&p[i]["outcomes"]["foot_normal_ns"])?,
            )?;
            close(
                lane.iter()
                    .map(|v| v.tangent)
                    .sum::<DVec3>()
                    .distance(vec3(&p[i]["outcomes"]["foot_tangent_ns"])?),
                0.,
            )?;
            for (r, index) in [0, 1, 4, 5, 8].into_iter().enumerate() {
                let q = &p[i]["comparisons"][r];
                verify(lane, &series[index], q, cut)?;
                let name = format!("{}_vs_{}", NAMES[i], NAMES[index]);
                endpoint(&mut rows, &name, &p[i], &p[index], k)?;
                for key in [
                    "rms_com_m",
                    "rms_spin_rad_s",
                    "max_com_m",
                    "max_public_reaction_difference_ns",
                ] {
                    store(&mut rows, &name, key, num(&q[key])?, k)
                }
            }
        }
        let change = num(&p[2]["comparisons"][1]["final_com_m"])?;
        if largest.is_none_or(|(x, _)| change > x) {
            largest = Some((change, k))
        }
        for (i, j) in [(2, 1), (2, 0), (3, 0)] {
            for (refidx, name) in [(2, "4096"), (3, "8192"), (4, "mechanical_first_cold")] {
                for key in ["rms_com_m", "final_com_m"] {
                    let d = num(&p[i]["comparisons"][refidx][key])?
                        - num(&p[j]["comparisons"][refidx][key])?;
                    let rank = if d < -1e-12 {
                        "closer"
                    } else if d > 1e-12 {
                        "farther"
                    } else {
                        "tie"
                    };
                    let label = format!("{}_minus_{}_vs_{name}", NAMES[i], NAMES[j]);
                    *counts.entry(format!("{label}_{key}_{rank}")).or_default() += 1;
                    store(&mut rows, &label, key, d, k);
                }
            }
        }
        endpoint(&mut rows, "transfer8_cold_vs_warm", &p[7], &p[2], k)?;
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":counts.get("cases")==Some(&50),"counts":counts,"largest_transfer8_vs_bound8_final_m_case":largest,"rows":rows_value(rows)}),
    )
}
