// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::transfer::{fixture, matched, vector};
use super::*;
pub(crate) fn summarize(dir: &Path, registered: &Path, previous: Option<&Path>) -> Result<()> {
    let mut counts = BTreeMap::<String, usize>::new();
    let mut fields = BTreeMap::<(String, String), Series>::new();
    for k in 0..1250 {
        let d = fixture(dir, k)?;
        let reference = fixture(registered, k)?;
        let prev = previous.map(|path| fixture(path, k)).transpose()?;
        if d["format"] != "finite-after-secondary-point-v1"
            || d["result"]["fixture"] != reference["result"]["fixture"]
        {
            return Err("invalid outgoing fixture".into());
        }
        for t in array(&d["result"]["trials"])? {
            let key = 3 * k + uint(&t["stroke"])? as usize - 1;
            let r = &t["response"];
            let b = &matched(&reference, t)?["response"];
            if r["geometry"] != b["geometry"] || r["arc_end"] != b["arc_end"] {
                return Err("incoming trajectory changed".into());
            }
            let h = &b["registered"][1]["projected"];
            let s = &r["secondary"];
            if h.is_null() != s.is_null() {
                return Err("secondary admission disagrees with registered endpoint".into());
            }
            if s.is_null() {
                *counts.entry("no_incident_line".into()).or_default() += 1;
                continue;
            }
            if vector(&s["impulse_ns"])?.distance(vector(&h["j_ns"])?) > 1e-11
                || h["end"] != s["loaded_end"]
                || h["sticking"] != s["sticking"]
            {
                // Clipping an authored re-import may reverse the two endpoint indices.
                if vector(&s["point_m"])?.distance(vector(&h["point"])?) > 1e-11
                    || vector(&s["impulse_ns"])?.distance(vector(&h["j_ns"])?) > 1e-11
                    || h["sticking"] != s["sticking"]
                {
                    return Err("secondary state disagrees with registered impulse".into());
                }
            }
            if let Some(prior) = &prev {
                let p = &matched(prior, t)?["response"];
                if p["outgoing"] != r["outgoing"]
                    || p["load"] != r["load"]
                    || (!p["arc"].is_null() && p["arc"] != r["arc"])
                {
                    return Err("a previously resolved outgoing arc changed".into());
                }
            }
            let mode = r["load"]["mode"].as_str().unwrap_or("unresolved");
            let feature = r["outgoing"]["feature"].as_str().unwrap_or("unresolved");
            *counts.entry(format!("{feature}_{mode}")).or_default() += 1;
            let Some(runs) = r["arc"].as_array() else {
                continue;
            };
            if runs.len() != 3 {
                return Err("missing integration meshes".into());
            }
            let z = &runs[2];
            let fine = num(&z["elapsed_interval_s"][0])?;
            let fin = z["end"].as_str().ok_or("missing event")?;
            *counts.entry(format!("end_{mode}_{fin}")).or_default() += 1;
            let mut add = |name: &str, v: f64| {
                for class in ["all", mode] {
                    fields
                        .entry((class.into(), name.into()))
                        .or_default()
                        .push(v, key)
                }
            };
            let horizon = num(&s["remaining_s"])?;
            for (i, row) in runs.iter().enumerate() {
                let lo = num(&row["elapsed_interval_s"][0])?;
                let hi = num(&row["elapsed_interval_s"][1])?;
                if row["subdivisions"] != [8, 32, 128][i]
                    || lo < 0.
                    || lo > hi
                    || hi > horizon + 1e-15
                    || hi - lo > 2e-16
                {
                    return Err("invalid local clock".into());
                }
                if row["end"] != z["end"] {
                    *counts.entry(format!("event_mismatch_{i}")).or_default() += 1
                }
                if i < 2 {
                    let tag = if i == 0 { "8_vs128" } else { "32_vs128" };
                    for (field, key) in [
                        ("com_m", "com_m"),
                        ("velocity_m_s", "velocity_m_s"),
                        ("linear_impulse_ns", "force_integral_ns"),
                        ("spin_rad_s", "spin_rad_s"),
                    ] {
                        add(
                            &format!("{tag}_{key}"),
                            vector(&row[field])?.distance(vector(&z[field])?),
                        );
                    }
                    add(&format!("{tag}_clock_s"), (lo - fine).abs());
                }
            }
            add("fine_elapsed_s", fine);
            for field in [
                "energy_balance_j",
                "linear_balance_ns",
                "angular_balance_nms",
                "normal_speed_m_s",
                "signed_gap_m",
            ] {
                add(field, num(&z[field])?.abs())
            }
            let f = vector(&r["load"]["force_n"])?;
            let n = vector(&s["normal"])?;
            let normal = f.dot(n);
            if normal < 0. || (f - normal * n).length() > 0.5 * normal + 1e-10 {
                return Err("outgoing contact force outside cone".into());
            }
        }
    }
    let rows = fields
        .into_iter()
        .map(|((group, metric), s)| json!({"group":group,"metric":metric,"statistics":s.stats()}))
        .collect::<Vec<_>>();
    write(
        &dir.join("summary.json"),
        &json!({"counts":counts,"rows":rows}),
    )
}
