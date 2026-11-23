// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
pub(super) fn summarize(dir: &Path, source: &Path) -> Result<()> {
    let mut groups: BTreeMap<(String, String), Series> = BTreeMap::new();
    let mut seen = BTreeSet::new();
    let mut censored = vec![];
    let mut compared = vec![];
    let mut awaiting = vec![];
    let mut ranking = BTreeMap::<(usize, usize), u64>::new();
    for entry in fs::read_dir(dir.join("cases"))? {
        let p = entry?.path();
        if p.extension().is_none_or(|s| s != "json") {
            continue;
        }
        let doc = read(&p)?;
        let k = uint(&doc["case"])? as usize;
        let i = k * 71 % 150;
        let original = uint(&doc["source"])? as usize;
        if doc["format"] != "continuous-three-stroke-grid-v1"
            || k >= 150
            || original * 117 % 250 != 10 * (i / 6) + 4 * ((i / 2) % 3) + i % 2
            || !seen.insert(k)
        {
            return Err("invalid source grid binding".into());
        }
        let d = &doc["result"];
        if d["cut"].is_null() {
            censored.push(k);
            continue;
        }
        let paths = array(&d["paths"])?;
        if paths.len() != 5 {
            return Err("missing continuous or numerical treatment".into());
        }
        let old = source
            .join("cases")
            .join(format!("case-{original:03}.json"));
        if old.exists() {
            let prior = read(&old)?;
            let same = &prior["result"];
            if same["cut"] != d["cut"] || same["fixture"] != d["fixture"] {
                return Err("native donor changed between campaigns".into());
            }
            for (here, there) in [(1, 6), (3, 0), (4, 1)] {
                if paths[here]["end"] != same["paths"][there]["end"]
                    || paths[here]["events"] != same["paths"][there]["outcomes"]
                    || paths[here]["calls"] != same["paths"][there]["native_calls"]
                {
                    return Err(format!("native repeat failed in case {k}, profile {here}").into());
                }
            }
            compared.push(k);
        } else {
            awaiting.push(k);
        }
        let ticks = uint(&d["end_tick"])? - uint(&d["cut_tick"])?;
        for (j, path) in paths.iter().enumerate() {
            let n = [2048, 4096, 8192, 8, 8][j];
            let suffix = [2048, 4096, 8192, 512, 512][j];
            if uint(&path["calls"])? != n + (ticks - 1) * suffix {
                return Err("wrong native budget".into());
            }
            let name = path["profile"].as_str().ok_or("unnamed profile")?;
            for (r, ref_path) in paths.iter().take(3).enumerate() {
                let result = &path["against"][r];
                let label = format!("{name}_vs_{}", [2048, 4096, 8192][r]);
                for key in [
                    "rms_com_m",
                    "max_com_m",
                    "final_com_m",
                    "rms_spin_rad_s",
                    "max_public_reaction_difference_ns",
                ] {
                    let value = num(&result[key])?;
                    if value < 0. || (j == r && value != 0.) {
                        return Err("invalid self-comparison".into());
                    }
                    groups
                        .entry((label.clone(), key.into()))
                        .or_default()
                        .push(value, k);
                }
                for stroke in 0..3 {
                    let local = &path["events"]["strokes"][stroke];
                    let other = &ref_path["events"]["strokes"][stroke];
                    if let (Some(a), Some(b)) = (
                        local["first_native_end_ns"].as_u64(),
                        other["first_native_end_ns"].as_u64(),
                    ) {
                        groups
                            .entry((label.clone(), format!("arrival_{}_ms", stroke + 1)))
                            .or_default()
                            .push(a.abs_diff(b) as f64 * 1e-6, k);
                    }
                }
            }
        }
        // Metric and tie handling are fixed before either native campaign finishes.
        let winner = |r: usize| -> Result<usize> {
            let a = num(&paths[3]["against"][r]["rms_com_m"])?;
            let b = num(&paths[4]["against"][r]["rms_com_m"])?;
            Ok(if (a - b).abs() < 1e-12 {
                2
            } else {
                usize::from(b < a)
            })
        };
        *ranking.entry((winner(1)?, winner(2)?)).or_default() += 1;
    }
    let rows=groups.into_iter().map(|((paths,metric),s)|json!({"comparison":paths,"metric":metric,"statistics":s.stats()})).collect::<Vec<_>>();
    let ranks = ranking
        .into_iter()
        .map(|((r1, r2), n)| json!({"winner_vs_4096":r1,"winner_vs_8192":r2,"count":n}))
        .collect::<Vec<_>>();
    write(
        &dir.join("summary.json"),
        &json!({"cases":seen.len(),"complete":seen.len()==150,"censored":censored,"cross_campaign_checked":compared,"awaiting_original":awaiting,"winner_key":["8r1_then512","8r8_then512","tie"],"rankings":ranks,"rows":rows}),
    )
}
