// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Follow matched first-impact choices from their true common 512-call continuation through three strokes.
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    env, fs,
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const LABELS: [&str; 5] = [
    "Better on both",
    "Worse on both",
    "Opposite signs",
    "Both below band",
    "One clears band",
];
const PROFILES: [&str; 5] = [
    "8r1_then512",
    "8r8_then512",
    "finite8_then512",
    "continuous4096r1",
    "continuous4096r8",
];
const TEST_OFFSETS: [u64; 5] = [175, 200, 225, 250, 275];
fn read(path: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}
fn series(v: &Value) -> Result<&Vec<Value>> {
    v.as_array().ok_or_else(|| "expected data array".into())
}
fn one(v: &Value, f: impl Fn(&Value) -> bool) -> Result<&Value> {
    let mut it = series(v)?.iter().filter(|x| f(x));
    let hit = it.next().ok_or("missing observation")?;
    if it.next().is_some() {
        return Err("duplicate observation".into());
    }
    Ok(hit)
}
fn scalar(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|x| x.is_finite())
        .ok_or_else(|| "nonfinite or absent measurement".into())
}
fn id(v: &Value) -> Result<u64> {
    v.as_u64()
        .ok_or_else(|| "expected unsigned coordinate".into())
}
fn vector(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from(serde_json::from_value::<[f64; 3]>(v.clone())?))
}
fn signed_delta(a: DVec3, b: DVec3, r: DVec3) -> f64 {
    1000. * (b.distance(r) - a.distance(r))
}
fn com(t: &Value) -> Result<DVec3> {
    Ok((vector(&t["foot"]["position"])? + vector(&t["upper"]["position"])?) * 0.5)
}
fn spin(t: &Value) -> Result<DVec3> {
    vector(&t["foot"]["angular_velocity"])
}
fn lanes<'a>(result: &'a Value, label: &str) -> Result<&'a Value> {
    one(&result["paths"], |r| r["profile"] == label)
}
fn trial(v: &Value) -> Result<&Value> {
    one(&v["result"]["trials"], |r| r["stroke"] == 1)
}
fn equal(a: &Value, b: &Value, what: &str) -> Result<()> {
    if a != b {
        return Err(format!("different {what}").into());
    }
    Ok(())
}
fn close(a: f64, b: f64, what: &str) -> Result<()> {
    if (a - b).abs() > 1e-11 * a.abs().max(b.abs()).max(1.) {
        return Err(format!("{what}: {a} versus {b}").into());
    }
    Ok(())
}
fn code(d: [f64; 2], threshold: f64) -> usize {
    if d.iter().all(|&x| x <= -threshold) {
        0
    } else if d.iter().all(|&x| x >= threshold) {
        1
    } else if d.iter().all(|x| x.abs() < threshold) {
        3
    } else if d[0] * d[1] < 0. {
        2
    } else {
        4
    }
}
fn stats(mut v: Vec<f64>) -> Value {
    v.sort_by(f64::total_cmp);
    let n = v.len();
    let q = |p: usize| v.get((n * p).div_ceil(100).saturating_sub(1));
    json!({"n":n,"min":v.first(),"p50":q(50),"p95":q(95),"max":v.last(),"mean":(!v.is_empty()).then(||v.iter().sum::<f64>()/n as f64)})
}
struct Files {
    prefix: PathBuf,
    native: PathBuf,
    finite: PathBuf,
    short_finite: PathBuf,
    short_relax: PathBuf,
    short_primary: PathBuf,
    medium: BTreeMap<u64, PathBuf>,
    deep: BTreeMap<u64, PathBuf>,
}
fn index_by_source(dir: &Path) -> Result<BTreeMap<u64, PathBuf>> {
    let mut files = BTreeMap::new();
    if !dir.exists() {
        return Ok(files);
    }
    for entry in fs::read_dir(dir)? {
        let p = entry?.path();
        if p.extension().and_then(|e| e.to_str()) != Some("json") {
            continue;
        }
        let j = read(&p)?;
        let k = id(&j["source"])?;
        if files.insert(k, p).is_some() {
            return Err("a native subset has duplicate source fixture IDs".into());
        }
    }
    Ok(files)
}
impl Files {
    fn new(root: &Path) -> Result<Self> {
        let cases = |dir: &str| root.join(format!("multifidelity-{dir}")).join("cases");
        Ok(Self {
            prefix: cases("landing-persistence"),
            native: cases("primary-propagation"),
            finite: cases("finite-propagation"),
            short_finite: cases("finite-policy-refined"),
            short_relax: cases("relaxation-survey"),
            short_primary: cases("primary-survey"),
            medium: index_by_source(&cases("three-stroke-convergence"))?,
            deep: index_by_source(&cases("deep-convergence"))?,
        })
    }
}
#[derive(Clone)]
struct Treatment {
    early: [f64; 2],
    terminal: [f64; 2],
    higher: [Option<f64>; 2],
    // [treatment, ordinary, 4096r1, 4096r8] or zero when return unobserved
    arrival: [[Option<f64>; 4]; 3],
    footprint: [bool; 4],
    load: [bool; 4],
    weight: [bool; 4],
}
struct Row {
    index: u64,
    short_index: u64,
    offset: u64,
    drive: u64,
    holds: bool,
    group: String,
    initial_drift: [f64; 2],
    p: [Treatment; 2],
}
fn short_id(offset: u64, drive: u64, holds: bool) -> Result<u64> {
    (0..1250u64)
        .find(|&i| {
            let j = 613 * i % 1250;
            165 + 5 * (j / 50) == offset
                && 76 + 2 * ((j / 2) % 25) == drive
                && (j % 2 == 1) == holds
        })
        .ok_or_else(|| "fixture missing from short survey".into())
}
fn sample(p: &Value, tick: u64) -> Result<&Value> {
    one(&p["samples"], |s| s["tick"] == tick)
}
fn against<'a>(m: &'a Value, profile: &str, w: u64) -> Result<&'a Value> {
    one(&m[profile]["curve"], |p| p["public_ticks_after_cut"] == w)
}
fn observation(m: &Value, stroke: u64) -> Result<Option<f64>> {
    let event = one(&m["outcomes"]["strokes"], |s| s["stroke"] == stroke)?;
    Ok(event["first_native_end_ns"]
        .as_u64()
        .map(|v| v as f64 * 1e-6))
}
fn validate_end(f: &Value, a: &Value, b: &Value, path: &str) -> Result<()> {
    equal(&a["cut"], &b["cut"], "shared pre-impact record")?;
    equal(&f["cut"], &a["cut"], "re-executed donor")?;
    equal(
        &lanes(a, path)?["end"],
        &lanes(b, path)?["end"],
        "repeated full endpoint",
    )
}
fn one_case(files: &Files, k: u64) -> Result<Row> {
    let name = format!("case-{k:03}.json");
    let pre = read(&files.prefix.join(&name))?;
    let orig = read(&files.native.join(&name))?;
    let fp = read(&files.finite.join(&name))?;
    if pre["format"] != "common-512-landing-prefix-v2"
        || [&pre, &orig, &fp].iter().any(|v| v["case"] != k)
    {
        return Err("counter or schema mismatch".into());
    }
    let (f, n, a) = (&pre["result"], &orig["result"], &fp["result"]);
    equal(&f["fixture"], &n["fixture"], "prefix and long coordinates")?;
    equal(&a["fixture"], &n["fixture"], "two propagation coordinates")?;
    validate_end(f, n, a, PROFILES[0])?;
    let fixture = &f["fixture"];
    let offset = (scalar(&fixture["offset_m"])? * 1000.).round() as u64;
    let drive = (scalar(&fixture["speed_scale"])? * 100.).round() as u64;
    let holds = fixture["switched"]
        .as_bool()
        .ok_or("missing tier history")?;
    let i = short_id(offset, drive, holds)?;
    let short_name = format!("case-{i:04}.json");
    let sf = read(&files.short_finite.join(&short_name))?;
    let sr = read(&files.short_relax.join(&short_name))?;
    let sp = read(&files.short_primary.join(&short_name))?;
    for case in [&sf, &sr, &sp] {
        equal(&trial(case)?["cut"], &f["cut"], "short and long donor")?;
    }
    let geo = &trial(&sf)?["geometry"];
    let raw = geo["status"].as_str().unwrap_or("missing");
    let group = if raw == "crossing" {
        let feat = geo["feature"]
            .as_str()
            .ok_or("missing feature")?
            .split('(')
            .next()
            .unwrap();
        format!(
            "{feat}/{}",
            match geo["sticking"].as_bool() {
                Some(true) => "sticking",
                Some(false) => "sliding",
                None => "no isolated law",
            }
        )
    } else {
        raw.to_owned()
    };
    let src = [n, n, a, n, n];
    let t = id(&f["cut"]["tick"])?;
    let mut small = Vec::new();
    let mut big = Vec::new();
    for (j, label) in PROFILES.iter().enumerate() {
        let p = lanes(f, label)?;
        let full = lanes(src[j], label)?;
        equal(
            sample(p, t + 1)?,
            &full["first"],
            "first public pose, twist, native budget and reaction",
        )?;
        if p["native_calls"] != if j < 3 { 12808 } else { 106496 } {
            return Err("unequal prefix budget".into());
        }
        small.push(sample(p, t + 26)?);
        big.push(full);
    }
    for j in [0, 1] {
        for g in 0..2 {
            let comparison = against(big[j], ["vs_uniform4096r1", "vs_uniform4096r8"][g], 26)?;
            close(
                scalar(&comparison["com_m"])?,
                com(small[j])?.distance(com(small[3 + g])?),
                "13ms native COM-reference curve",
            )?;
            close(
                scalar(&comparison["spin_rad_s"])?,
                spin(small[j])?.distance(spin(small[3 + g])?),
                "13ms native spin-reference curve",
            )?;
        }
    }
    let finite_comparison = against(big[2], "trajectory_change", 26)?;
    close(
        scalar(&finite_comparison["com_m"])?,
        com(small[2])?.distance(com(small[0])?),
        "same-work finite COM curve",
    )?;
    close(
        scalar(&finite_comparison["spin_rad_s"])?,
        spin(small[2])?.distance(spin(small[0])?),
        "same-work finite spin curve",
    )?;
    let s1 = &one(&trial(&sp)?["paths"], |p| {
        p["method"] == "uniform" && p["calls"] == 4096 && p["iterations"] == 4
    })?["record"];
    let s8 = one(&trial(&sr)?["paths"], |p| {
        p["profile"] == json!({"calls":4096,"relaxation":8,"onset_only":false})
    })?;
    let mut initial_drift = [0.; 2];
    for (g, m) in [s1, s8].into_iter().enumerate() {
        let sample = one(&m["samples"], |r| r["i"] == 26)?;
        initial_drift[g] = 1000. * vec_distance(sample, small[3 + g])?;
    }
    let mut higher = [None, None];
    for (rank, table, pname) in [(0, &files.medium, "8192"), (1, &files.deep, "32768")] {
        if let Some(path) = table.get(&k) {
            let hi = read(path)?;
            let hr = &hi["result"];
            validate_end(f, n, hr, PROFILES[0])?;
            validate_end(
                f,
                if rank == 0 { n } else { a },
                hr,
                if rank == 0 { PROFILES[1] } else { PROFILES[2] },
            )?;
            if rank == 0 {
                equal(
                    &lanes(hr, "4096")?["end"],
                    &big[3]["end"],
                    "shared continuous 4096 endpoint",
                )?;
            }
            higher[rank] = Some(com(&lanes(hr, pname)?["end"])?);
        }
    }
    let mut treatments = Vec::new();
    for j in [1, 2] {
        let ea = com(small[0])?;
        let eb = com(small[j])?;
        let la = com(&big[0]["end"])?;
        let lb = com(&big[j]["end"])?;
        let mut early = [0.; 2];
        let mut terminal = [0.; 2];
        for g in 0..2 {
            early[g] = signed_delta(ea, eb, com(small[3 + g])?);
            terminal[g] = signed_delta(la, lb, com(&big[3 + g]["end"])?);
        }
        let mut arrival = [[None; 4]; 3];
        let mut footprint = [false; 4];
        let mut load = [false; 4];
        let mut weight = [false; 4];
        for (l, &index) in [j, 0, 3, 4].iter().enumerate() {
            let m = big[index];
            let e = &m["end"];
            if e["tick"] != 2171 {
                return Err("terminal time mismatch".into());
            }
            footprint[l] = !e["footprint_gap_m"].is_null();
            let impulse = scalar(&e["normal_impulse_ns"])?;
            load[l] = impulse > 0.;
            weight[l] = impulse >= 2. * 9.81 * 0.0005;
            for (s, r) in arrival.iter_mut().enumerate() {
                r[l] = observation(m, (s + 1) as u64)?;
            }
        }
        treatments.push(Treatment {
            early,
            terminal,
            higher: higher.map(|v| v.map(|r| signed_delta(la, lb, r))),
            arrival,
            footprint,
            load,
            weight,
        });
    }
    Ok(Row {
        index: k,
        short_index: i,
        offset,
        drive,
        holds,
        group,
        initial_drift,
        p: treatments.try_into().ok().expect("two treatments"),
    })
}
fn vec_distance(stored: &Value, measured: &Value) -> Result<f64> {
    Ok(vector(&stored["com_m"])?.distance(com(measured)?))
}
fn summarize(rows: &[&Row], policy: usize) -> Value {
    let samples: Vec<_> = rows.iter().map(|r| &r.p[policy]).collect();
    let mut joint = [[0; 5]; 5];
    let mut endpoint_bits = [[0; 4]; 3];
    for r in &samples {
        joint[code(r.early, 0.01)][code(r.terminal, 1.)] += 1;
        for (i, flags) in [r.footprint, r.load, r.weight].iter().enumerate() {
            endpoint_bits[i][usize::from(flags[1]) + 2 * usize::from(flags[0])] += 1;
        }
    }
    let by_reference=[0,1].map(|g|json!({
        "early_delta_mm":stats(samples.iter().map(|r|r.early[g]).collect()),
        "terminal_delta_mm":stats(samples.iter().map(|r|r.terminal[g]).collect()),
        "endpoint_footprint_disagreements_treatment_and_ordinary":[samples.iter().filter(|r|r.footprint[0]!=r.footprint[g+2]).count(),samples.iter().filter(|r|r.footprint[1]!=r.footprint[g+2]).count()],
        "endpoint_load_disagreements_treatment_and_ordinary":[samples.iter().filter(|r|r.load[0]!=r.load[g+2]).count(),samples.iter().filter(|r|r.load[1]!=r.load[g+2]).count()]
    }));
    let deep=[0,1].map(|rank|{
        let mut m=[[0;3];5];let mut v=vec![];
        for r in &samples{if let Some(d)=r.higher[rank]{m[code(r.early,0.01)][if d<=-1.{0}else if d>=1.{1}else{2}]+=1;v.push(d);}}
        let eligible = rows.iter().filter(|r| if rank == 0 { [76,100,124].contains(&r.drive) } else { r.drive == 100 }).count();
        json!({"eligible":eligible,"outside_grid":rows.len()-eligible,"available":v.len(),"pending":eligible-v.len(),"terminal_delta_mm":stats(v),"early_to_better_worse_small":m})
    });
    let returns=[0,1,2].map(|s|{
        let mut mask=[0;4];let mut change=vec![];let mut effects:[Vec<f64>;2]=Default::default();
        for r in &samples{let a=r.arrival[s];mask[usize::from(a[1].is_some())+2*usize::from(a[0].is_some())]+=1;
            if let(Some(b),Some(ordinary))=(a[0],a[1]){change.push(b-ordinary);for g in 0..2{if let Some(reference)=a[2+g]{effects[g].push((b-reference).abs()-(ordinary-reference).abs());}}}
        }
        json!({"stroke":s+1,"both_absent_ordinary_only_treatment_only_both":mask,"signed_shift_ms":stats(change),"absolute_timing_error_change_ms":effects.map(stats)})
    });
    json!({"n":samples.len(),"early_to_terminal":joint,"reference_order":["4096 r1","4096 r8"],"per_reference":by_reference,"higher_terminal_references":deep,"endpoint_status_by_ordinary_treatment":endpoint_bits,"returns":returns})
}
fn heat(rows: &[&Row], path: &Path) -> Result<()> {
    let root = SVGBackend::new(path, (1140, 815)).into_drawing_area();
    root.fill(&WHITE)?;
    root.draw(&Text::new(
        "Does a better first landing stay better across three returns?",
        (26, 36),
        ("sans-serif", 24).into_font(),
    ))?;
    root.draw(&Text::new(format!("{} held-out matched donors • each branch takes 512 native calls after its first public interval",rows.len()),(26,63),("sans-serif",15).into_font()))?;
    for policy in 0..2 {
        let x0 = 175 + (policy as i32) * 515;
        let y0 = 162;
        let w = 79;
        let h = 88;
        let mut count = [[0usize; 5]; 5];
        for row in rows {
            let r = &row.p[policy];
            count[code(r.early, 0.01)][code(r.terminal, 1.)] += 1;
        }
        root.draw(&Text::new(
            [
                "Eight relaxation passes at onset",
                "Finite geometry at onset",
            ][policy],
            (x0, y0 - 65),
            ("sans-serif", 18).into_font(),
        ))?;
        root.draw(&Text::new(
            "Final endpoint →",
            (x0, y0 - 33),
            ("sans-serif", 13).into_font(),
        ))?;
        for (j, label) in ["Better", "Worse", "Opposite", "Small", "One clear"]
            .into_iter()
            .enumerate()
        {
            root.draw(&Text::new(
                label,
                (x0 + j as i32 * w, y0 - 8),
                ("sans-serif", 12).into_font(),
            ))?;
        }
        for (i, bin) in count.iter().enumerate() {
            let total: usize = bin.iter().sum();
            if policy == 0 {
                root.draw(&Text::new(
                    LABELS[i],
                    (14, y0 + i as i32 * h + 44),
                    ("sans-serif", 14).into_font(),
                ))?;
            }
            for (j, &n) in bin.iter().enumerate() {
                let t = if total == 0 {
                    0.
                } else {
                    n as f64 / total as f64
                };
                let left = x0 + j as i32 * w;
                let top = y0 + i as i32 * h;
                let rgb = if total == 0 {
                    RGBColor(235, 236, 236)
                } else {
                    RGBColor(
                        (247. - 165. * t) as u8,
                        (249. - 85. * t) as u8,
                        (248. - 96. * t) as u8,
                    )
                };
                root.draw(&Rectangle::new(
                    [(left, top), (left + w - 3, top + h - 3)],
                    rgb.filled(),
                ))?;
                root.draw(&Text::new(
                    if total == 0 {
                        "—".into()
                    } else {
                        n.to_string()
                    },
                    (left + 29, top + 45),
                    ("sans-serif", 19).into_font(),
                ))?;
            }
        }
    }
    for (k,s) in ["Row = COM error change at 13 ms (10 µm band). Column = change at 1.0855 s (1 mm band).",
        "Better/worse require both 4,096-call references to agree and exceed the band. Opposite = different signs with at least one outside the band.",
        "Small = both absolute changes below the band. One clear = only one reference clears the band and signs agree. Gray dashes mark empty rows.",
        "Each cell gives its donor count. Color is the proportion of donors in that early class."]
        .into_iter().enumerate(){root.draw(&Text::new(s,(26,655+k as i32*34),("sans-serif",15).into_font()))?;}
    root.present()?;
    Ok(())
}
fn main() -> Result<()> {
    let args = env::args_os().skip(1).collect::<Vec<_>>();
    if args.len() != 2 {
        return Err("landing_persistence SIM_ARTIFACT_ROOT REPORT_DIR".into());
    }
    let root = Path::new(&args[0]);
    let out = Path::new(&args[1]);
    let files = Files::new(root)?;
    fs::create_dir_all(out)?;
    let mut rows = vec![];
    let mut failed = vec![];
    let mut missing = [0usize; 8];
    let mut censored = vec![];
    for i in 0..250 {
        let name = format!("case-{i:03}.json");
        let paths = [
            files.prefix.join(&name),
            files.native.join(&name),
            files.finite.join(&name),
        ];
        let mask = paths
            .iter()
            .enumerate()
            .fold(0, |m, (j, p)| m + (usize::from(p.is_file()) << j));
        missing[mask] += 1;
        if mask != 7 {
            continue;
        }
        if read(&paths[0])?["result"]["cut"].is_null() {
            censored.push(i);
            continue;
        }
        match one_case(&files, i) {
            Ok(row) => rows.push(row),
            Err(e) => failed.push(json!({"source":i,"reason":e.to_string()})),
        }
    }
    let (held, train): (Vec<_>, Vec<_>) =
        rows.iter().partition(|r| TEST_OFFSETS.contains(&r.offset));
    let groups = rows
        .iter()
        .map(|r| r.group.as_str())
        .collect::<std::collections::BTreeSet<_>>();
    let stratification=groups.iter().map(|g|{let subset=held.iter().copied().filter(|r|r.group.as_str()==*g).collect::<Vec<_>>();json!({"feature":g,"held":subset.len(),"treatments":[summarize(&subset,0),summarize(&subset,1)]})}).collect::<Vec<_>>();
    let is_final = missing[7] == 250 && failed.is_empty();
    let winner=held.iter().filter_map(|r|{let q=&r.p[1];(code(q.early,0.01)==0&&code(q.terminal,1.)==1).then_some((r,(-q.early[0]).min(-q.early[1])))})
        .max_by(|(a,x),(b,y)|x.total_cmp(y).then(b.index.cmp(&a.index))).map(|(r,score)|json!({"source":r.index,"short_index":r.short_index,"offset_mm":r.offset,"drive_percent":r.drive,"holds":r.holds,"group":r.group,"early_improvement_mm":score,"early_change_mm":r.p[1].early,"terminal_change_mm":r.p[1].terminal,"finer_terminal_change_mm":r.p[1].higher}));
    let summary = json!({"format":"landing-persistence-field-v1","required":250,"available_source_bits":["common 512 prefix","original propagation","finite propagation"],"available_source_bitmask":missing,"all_three_campaigns_published":missing[7],"joined":rows.len(),"failed_common_controls":failed,"no_donor":censored,"final":is_final,
        "held":held.len(),"training":train.len(),"held_offsets_mm":TEST_OFFSETS,"held_classes":LABELS,"threshold_early_mm":0.01,"threshold_terminal_mm":1.,
        "early_references":["continuous native 4096 r1","continuous native 4096 r8"],"later_extra_reference_order":["8192 r1","32768 r1"],"treatments":["8r8_then512","finite8_then512"],"terminal_status_order":["projected footprint","positive normal reaction","normal reaction >= static weight impulse"],"status_cells":["neither","ordinary only","treatment only","both"],
        "held_out":[summarize(&held,0),summarize(&held,1)],"calibration":[summarize(&train,0),summarize(&train,1)],"held_by_initial_feature":stratification,"early_native_to_short_driver_reference_drift_mm":[stats(rows.iter().map(|r|r.initial_drift[0]).collect()),stats(rows.iter().map(|r|r.initial_drift[1]).collect())],"select_after_all_sources":is_final,"largest_early_benefit_among_dual_terminal_reversals":winner});
    fs::write(
        out.join("persistence.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    let mut csv = std::io::BufWriter::new(fs::File::create(out.join("donors.jsonl"))?);
    use std::io::Write;
    for r in &rows {
        serde_json::to_writer(
            &mut csv,
            &json!({"source":r.index,"short_index":r.short_index,"offset_mm":r.offset,"drive_percent":r.drive,"holds":r.holds,"held":TEST_OFFSETS.contains(&r.offset),"feature":r.group,"treatments":r.p.iter().map(|p|json!({"early_mm":p.early,"terminal_mm":p.terminal,"finer_terminal_mm":p.higher,"arrival_ms":p.arrival,"footprint":p.footprint,"positive_load":p.load,"above_weight":p.weight})).collect::<Vec<_>>() }),
        )?;
        csv.write_all(b"\n")?;
    }
    csv.flush()?;
    heat(&held, &out.join("persistence.svg"))?;
    if !failed.is_empty() {
        return Err(format!(
            "{} shared controls failed; see persistence.json",
            failed.len()
        )
        .into());
    }
    Ok(())
}
