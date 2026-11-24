// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read back the cold secondary cut in laboratory coordinates independently
//! from the surveyed SDF endpoint and the prior primary-landing campaigns.
use super::*;
pub(crate) fn summarize(dir: &Path, registered: &Path, deep: &Path) -> Result<()> {
    let names = [
        "native8_then512",
        "endpoint8_then512",
        "native8r8_then512",
        "native4096_then512",
        "continuous4096",
        "continuous8192",
    ];
    let mut stats = Rows::new();
    let mut ranks = BTreeMap::<String, usize>::new();
    let (mut n, mut eligible, mut cross) = (0, 0, 0);
    let mut largest = None;
    for k in 0..50 {
        if !file(dir, k).exists() {
            continue;
        }
        let d = read(&file(dir, k))?;
        let j = 17 * k % 50;
        let coord = 10 * (j / 2) + 4 + j % 2;
        let original = (0..250).find(|i| i * 117 % 250 == coord).unwrap();
        let donor = (0..1250)
            .find(|i| i * 613 % 1250 == 50 * (j / 2) + 24 + j % 2)
            .unwrap();
        let prior = read(
            &registered
                .join("cases")
                .join(format!("case-{donor:04}.json")),
        )?;
        let t = &prior["result"]["trials"][0];
        if d["format"] != "cold-secondary-three-strokes-v1"
            || d["case"] != k
            || d["source"] != original
            || d["cut"] != t["cut"]
            || t["stroke"] != 1
        {
            return Err(format!("secondary donor mismatch {k}").into());
        }
        for key in ["offset_m", "switched", "speed_scale"] {
            if d["fixture"][key] != prior["result"]["fixture"][key] {
                return Err("secondary fixture coordinates changed".into());
            }
        }
        if file(deep, k).exists() {
            let reference = read(&file(deep, k))?;
            cross_checks(&d, &reference["result"])?;
            if reference["source"] != original {
                return Err("deep-control identity mismatch".into());
            }
            cross += 1;
        }
        n += 1;
        if d["paths"].is_null() {
            continue;
        }
        eligible += 1;
        let t = &t["response"];
        for key in ["arc_end", "geometry"] {
            if d["prehistory"][key] != t[key] {
                return Err("secondary physical prehistory changed".into());
            }
        }
        let arc = &t["arc_end"];
        let clock = uint(&d["offset_ns"])?;
        let h = num(&d["geometric_time_s"])?;
        close(
            h,
            num(&t["geometry"]["hi_ns"])? * 1e-9 + num(&arc["elapsed_interval_s"][0])?,
        )?;
        if clock != (h * 1e9).round() as u64 || clock >= 500_000 {
            return Err("partial clock outside original actuator interval".into());
        }
        let cut = uint(&d["cut_tick"])?;
        let end = uint(&d["end_tick"])?;
        let x = &d["initial"];
        if uint(&x["time_ns"])? != 500_000 * cut + clock
            || uint(&x["time_us"])? != uint(&x["time_ns"])? / 1000
            || x["tick"] != cut
        {
            return Err("secondary clock quantization mismatch".into());
        }
        let pad = vec3(&x["pad"]["velocity"])?;
        let v = (vec3(&x["foot"]["velocity"])? + vec3(&x["upper"]["velocity"])?) * 0.5;
        let shift = h * pad;
        close(com(x)?.distance(vec3(&arc["com_m"])? + shift), 0.)?;
        close(
            vec3(&x["foot"]["position"])?.distance(vec3(&arc["foot_m"])? + shift),
            0.,
        )?;
        close(v.distance(vec3(&arc["velocity_m_s"])? + pad), 0.)?;
        close(
            vec3(&x["foot"]["angular_velocity"])?.distance(vec3(&arc["spin_rad_s"])?),
            0.,
        )?;
        let paths = labels(
            &d["paths"],
            &names,
            end - cut,
            &[8, 8, 8, 4096, 4096, 8192],
            &[512, 512, 512, 512, 4096, 8192],
        )?;
        let candidate = &paths[1]["impulse"];
        let expected = &t["registered"][1]["projected"];
        if candidate.is_null() != expected.is_null() {
            return Err("secondary native admission changed in translation".into());
        }
        if !candidate.is_null() {
            close(
                vec3(&candidate["j_ns"])?.distance(vec3(&expected["j_ns"])?),
                0.,
            )?;
            close(
                vec3(&candidate["point_m"])?.distance(vec3(&expected["point"])? + shift),
                0.,
            )?;
            if candidate["stick"] != expected["sticking"]
                || paths[1]["first"]["first_local_ns"] != clock
            {
                return Err("secondary impulse mode or application clock mismatch".into());
            }
        }
        for (i, p) in paths.iter().enumerate() {
            if i != 1 && !p["impulse"].is_null() {
                return Err("unrequested projected impulse".into());
            }
            if p["first"]["tick"] != cut + 1
                || p["end"]["tick"] != end
                || p["first"]["time_us"] != (cut + 1) * 500
                || p["end"]["time_us"] != end * 500
            {
                return Err("actuator boundary changed".into());
            }
            let next = vec3(&x["pad"]["position"])? + pad * ((500_000 - clock) as f64 * 1e-9);
            if vec3(&p["first"]["pad"]["position"])?.distance(next) > 2e-11 {
                return Err("support missed original actuator change".into());
            }
            for (idx, peer) in [0, 4, 5].into_iter().enumerate() {
                let c = &p["comparisons"][idx];
                let name = format!("{}_vs_{}", names[i], names[peer]);
                close(
                    num(&c["final_com_m"])?,
                    com(&p["end"])?.distance(com(&paths[peer]["end"])?),
                )?;
                for key in [
                    "rms_com_m",
                    "max_com_m",
                    "rms_spin_rad_s",
                    "max_public_reaction_difference_ns",
                ] {
                    store(&mut stats, &name, key, num(&c[key])?, k)
                }
                endpoint(&mut stats, &name, p, &paths[peer], k)?;
            }
        }
        let diff = num(&paths[1]["comparisons"][0]["final_com_m"])?;
        if largest.is_none_or(|(v, _)| diff > v) {
            largest = Some((diff, k))
        }
        for (r, label) in [(1, "4096"), (2, "8192")] {
            for metric in [
                "rms_com_m",
                "final_com_m",
                "max_public_reaction_difference_ns",
            ] {
                let change = num(&paths[1]["comparisons"][r][metric])?
                    - num(&paths[0]["comparisons"][r][metric])?;
                store(
                    &mut stats,
                    &format!("endpoint_minus_native_vs_{label}"),
                    metric,
                    change,
                    k,
                );
                let a = if change < -1e-12 {
                    "closer"
                } else if change > 1e-12 {
                    "farther"
                } else {
                    "tie"
                };
                *ranks.entry(format!("{label}_{metric}_{a}")).or_default() += 1;
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"cases":n,"complete":n==50,"matched_second":eligible,"matched_deep_primary":cross,"largest_causal_terminal_m_case":largest,"ranking":ranks,"rows":rows_value(stats)}),
    )
}
