// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check every executed return-to-native clock. A timing intervention changes
//! both the physical state and the instant at which native warmstarting begins.
use super::*;
const FINE: [usize; 4] = [0, 7, 42, 47];
fn trace(root: &Path, grid: u64, k: usize, n: u64) -> PathBuf {
    root.join(format!("grid-{grid}/case-{k:03}/traces/physical-{n}.json"))
}
fn doc(root: &Path, grid: u64, k: usize) -> PathBuf {
    root.join(format!("grid-{grid}/cases/case-{k:03}.json"))
}
fn centers(tr: &Value) -> Result<Vec<DVec3>> {
    array(&tr["frames"])?.iter().map(com).collect()
}
fn compare(rows: &mut Rows, key: &str, a: &[DVec3], b: &[DVec3], k: usize) -> Result<f64> {
    if a.len() != b.len() || a.is_empty() {
        return Err("different public clocks".into());
    }
    let mut sum = 0.;
    let mut peak = 0f64;
    for (x, y) in a.iter().zip(b) {
        let d = x.distance(*y);
        sum += d * d;
        peak = peak.max(d);
    }
    let d = a.last().unwrap().distance(*b.last().unwrap());
    for (what, x) in [
        ("final_com_m", d),
        ("peak_com_m", peak),
        ("rms_com_m", (sum / a.len() as f64).sqrt()),
    ] {
        store(rows, key, what, x, k);
    }
    Ok(d)
}
fn replay(
    dir: &Path,
    k: usize,
    label: &str,
    lanes: &[(PathBuf, String, u64)],
    cut: u64,
) -> Result<()> {
    let folder = dir.join(format!("case-{k:03}-{label}"));
    fs::create_dir_all(folder.join("traces"))?;
    let mut config = Vec::new();
    for (i, (src, title, n)) in lanes.iter().enumerate() {
        let name = format!("traces/lane-{i}.json");
        fs::copy(src, folder.join(&name))?;
        config.push(json!({"label":format!("handoff{i}"),"name":title,"file":name,"analytical_ticks":(cut+1..=cut+n).collect::<Vec<_>>()}));
    }
    write(
        &folder.join("comparison.json"),
        &json!({"title":"Returning to native physics after the same material contact","pair_totals_only":true,"lanes":config}),
    )
}
pub(crate) fn summarize(dir: &Path, episodes: &Path, whole: &Path, suffix: &Path) -> Result<()> {
    let mut rows = Rows::new();
    let mut counts = BTreeMap::<String, usize>::new();
    let mut largest = None;
    let mut max_fine = None;
    for k in 0..50 {
        if !doc(dir, 512, k).exists() {
            continue;
        }
        let q = read(&doc(dir, 512, k))?;
        let refs = read(&file(episodes, k))?;
        let original = read(&file(whole, k))?;
        let old = read(&whole.join("trajectories").join(format!("case-{k:03}.json")))?;
        if q["format"] != "matched-physical-handoff-v1"
            || q["case"] != k
            || q["source"] != original["source"]
            || q["cut"] != original["cut"]
            || q["cut"] != refs["cut"]
        {
            return Err("foreign handoff donor".into());
        }
        let cut = uint(&q["cut"]["tick"])?;
        let end = uint(&q["end"])?;
        let limit = uint(&q["qualified_prefix"])?;
        let mut paths = BTreeMap::new();
        let mut origin = None;
        for r in array(&q["paths"])? {
            let n = uint(&r["physical_ticks"])?;
            if ![1, 2, 8, 26].contains(&n)
                || n > limit
                || r["native_calls"] != (end - cut - n) * 512
            {
                return Err("wrong paid budget".into());
            }
            let tr = read(&trace(dir, 512, k, n))?;
            let frames = array(&tr["frames"])?;
            if frames.len() as u64 != end - cut
                || frames.last() != Some(&r["end"])
                || frames[0] != original["paths"][6]["first"]
            {
                return Err("invalid handoff timing".into());
            }
            let control = array(&old["series"][6])?;
            let mut pressure = 0.;
            let mut tangent = DVec3::ZERO;
            let mut c = Vec::new();
            for (i, f) in frames.iter().enumerate() {
                let at = i as u64 + 1;
                if f["native_steps"] != if at <= n { 0 } else { 512 }
                    || f["tick"] != cut + at
                    || f["time_us"] != (cut + at) * 500
                {
                    return Err("wrong actuator clock".into());
                }
                let center = com(f)?;
                pressure += num(&f["normal_impulse_ns"])?;
                tangent += vec3(&f["tangent_on_foot_ns"])?;
                if at <= n {
                    let b = &refs["references"][i]["fine"];
                    close(
                        center.distance(
                            vec3(&b["com"])? + (at as f64 * 0.0005) * vec3(&refs["drive"])?,
                        ),
                        0.,
                    )?;
                    close(pressure, num(&b["jn"])?)?;
                    close(
                        tangent.distance(vec3(&b["j"])? - vec3(&b["normal_vector"])?),
                        0.,
                    )?;
                }
                if n == 1 {
                    let p = internal::sample(&control[i])?;
                    if p.com != center
                        || p.spin != vec3(&f["foot"]["angular_velocity"])?
                        || p.normal != num(&f["normal_impulse_ns"])?
                        || p.tangent != vec3(&f["tangent_on_foot_ns"])?
                    {
                        return Err("original 512 suffix differs".into());
                    }
                }
                c.push(center);
            }
            close(pressure, num(&r["outcomes"]["foot_normal_ns"])?)?;
            if n == 1 {
                origin = Some(c.clone());
            }
            if n > 1 {
                let d = compare(
                    &mut rows,
                    &format!("physical{n}_vs_physical1_then512"),
                    &c,
                    origin.as_ref().ok_or("missing primary control")?,
                    k,
                )?;
                if n == 8 && largest.is_none_or(|(v, _)| d > v) {
                    largest = Some((d, k));
                }
            }
            *counts
                .entry(format!("donors_physical{n}_then512"))
                .or_default() += 1;
            paths.insert(n, c);
        }
        if FINE.contains(&k) && [16384, 32768].iter().all(|&g| doc(dir, g, k).exists()) {
            let mut controls = Vec::new();
            let mut extended = Vec::new();
            let physical = read(&trace(dir, 512, k, 8))?;
            for n in [16384u64, 32768] {
                let q = read(&doc(dir, n, k))?;
                let p = &q["paths"][0];
                let raw = read(&trace(dir, n, k, 8))?;
                let old = read(&suffix.join(format!("case-{k:03}/traces/native{n}.json")))?;
                let frames = array(&raw["frames"])?;
                if q["cut"] != refs["cut"]
                    || q["grid"] != n
                    || p["physical_ticks"] != 8
                    || p["native_calls"] != (end - cut - 8) * n
                    || frames.len() != array(&old["frames"])?.len()
                    || frames[..8] != array(&physical["frames"])?[..8]
                    || frames.iter().enumerate().any(|(i, x)| {
                        x["tick"] != i as u64 + cut + 1
                            || x["native_steps"] != if i < 8 { 0 } else { n }
                    })
                    || frames.last() != Some(&p["end"])
                {
                    return Err("different fine handoff".into());
                }
                controls.push(centers(&old)?);
                extended.push(centers(&raw)?);
            }
            let orig = compare(
                &mut rows,
                "physical1_then_16384_vs_32768",
                &controls[0],
                &controls[1],
                k,
            )?;
            let revised = compare(
                &mut rows,
                "physical8_then_16384_vs_32768",
                &extended[0],
                &extended[1],
                k,
            )?;
            if max_fine.is_none_or(|(v, _)| revised > v) {
                max_fine = Some((revised, k));
            }
            *counts
                .entry(
                    if revised < orig {
                        "extended_finest_pair_closer"
                    } else {
                        "extended_finest_pair_farther"
                    }
                    .into(),
                )
                .or_default() += 1;
            *counts.entry("fine_donors".into()).or_default() += 1;
        }
    }
    if let Some((_, k)) = largest {
        let q = read(&doc(dir, 512, k))?;
        let cut = uint(&q["cut"]["tick"])?;
        let ns = if q["qualified_prefix"].as_u64().unwrap() >= 26 {
            [1, 8, 26]
        } else {
            [1, 2, 8]
        };
        replay(
            dir,
            k,
            "native512",
            &ns.map(|n| {
                (
                    trace(dir, 512, k, n),
                    format!("Physical {:.1} ms, then 512 calls", n as f64 * 0.5),
                    n,
                )
            }),
            cut,
        )?;
    }
    if let Some((_, k)) = max_fine {
        let cut = uint(&read(&doc(dir, 512, k))?["cut"]["tick"])?;
        replay(
            dir,
            k,
            "fine",
            &[
                (
                    suffix.join(format!("case-{k:03}/traces/native32768.json")),
                    "Physical 0.5 ms, then 32768 calls".into(),
                    1,
                ),
                (
                    trace(dir, 16384, k, 8),
                    "Physical 4 ms, then 16384 calls".into(),
                    8,
                ),
                (
                    trace(dir, 32768, k, 8),
                    "Physical 4 ms, then 32768 calls".into(),
                    8,
                ),
            ],
            cut,
        )?;
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":counts.get("donors_physical8_then512")==Some(&50)&&counts.get("fine_donors")==Some(&4),"counts":counts,"largest_physical8_vs_first_m_case":largest,"largest_fine_pair_m_case":max_fine,"rows":rows_value(rows)}),
    )
}
