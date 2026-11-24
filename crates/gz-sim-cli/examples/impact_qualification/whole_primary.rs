// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify complete original-clock comparisons and quantify the numerical
//! reference disagreements separately from the registered solver responses.
use super::*;
const LABELS: [&str; 10] = [
    "native2",
    "launch2",
    "native8",
    "endpoint8",
    "spatial8",
    "launch8",
    "retimed_launch8",
    "launch64",
    "native4096",
    "native8192",
];
fn body(t: &Value, key: &str) -> Result<DVec3> {
    Ok(0.5 * (vec3(&t["foot"][key])? + vec3(&t["upper"][key])?))
}
pub(crate) fn summarize(dir: &Path, previous: &Path, initial: &Path) -> Result<()> {
    let mut fields = Fields::new();
    let mut counts = BTreeMap::new();
    let mut largest = None;
    let mut regression = None;
    for k in 0..1250 {
        if !dir.join("cases").join(format!("case-{k:04}.json")).exists() {
            continue;
        }
        let (a, other, ordinary) = (
            fixture(dir, k)?,
            fixture(previous, k)?,
            fixture(initial, k)?,
        );
        if a["format"] != "continuous-original-primary-v1"
            && a["format"] != "subdivided-original-primary-v1"
            && a["format"] != "directed-original-primary-v1"
        {
            return Err("unrecognized primary panel".into());
        }
        count(&mut counts, "fixtures");
        for t in array(&a["result"]["trials"])? {
            let id = 3 * k + uint(&t["stroke"])? as usize - 1;
            let old = matched(&other, t)?;
            let nat = matched(&ordinary, t)?;
            if t["geometry"] != old["geometry"] {
                return Err(format!("primary geometry changed {id}").into());
            }
            let rows = array(&t["paths"])?;
            if rows.len() != 10 {
                return Err("incomplete native grid ladder".into());
            }
            count(&mut counts, "landings");
            for (i, n) in [(2, 0), (8, 3)] {
                let q = &nat["paths"][n];
                if q["calls"] != rows[i]["first"]["native_steps"] {
                    return Err("wrong repeated grid".into());
                }
                close(vec3(&q["total_ns"])?.distance(vec3(&rows[i]["j"])?), 5e-11)?;
                close(
                    (num(&q["total_normal_ns"])? - num(&rows[i]["first"]["normal_impulse_ns"])?)
                        .abs(),
                    5e-11,
                )?;
            }
            for (i, q) in [(2, 0), (3, 2)] {
                let start = &rows[i]["first"];
                let prior = &old["paths"][q]["samples"][0];
                if prior["i"] != 1 {
                    return Err("wrong retained primary clock".into());
                }
                close(
                    body(start, "position")?.distance(vec3(&prior["com_m"])?),
                    3e-12,
                )?;
                close(
                    body(start, "velocity")?.distance(vec3(&prior["velocity_m_s"])?),
                    3e-11,
                )?;
                close(
                    vec3(&start["foot"]["angular_velocity"])?.distance(vec3(&prior["spin_rad_s"])?),
                    3e-10,
                )?;
                close(
                    (num(&start["normal_impulse_ns"])? - num(&prior["normal_ns"])?).abs(),
                    3e-11,
                )?;
            }
            let (coarse, reference) = (&t["mechanics"][0], &t["mechanics"][1]);
            let deck = vec3(&t["deck_v"])?;
            let mut matched_ref = false;
            if coarse["end"] == "original_clock" && reference["end"] == "original_clock" {
                let (a, b) = (array(&reference["events"])?, array(&coarse["events"])?);
                let same = a.len() == b.len()
                    && a.iter().zip(b).all(|(x, y)| {
                        x["feature"] == y["feature"]
                            && x["stuck"] == y["stuck"]
                            && x["mode"] == y["mode"]
                    });
                let dc = vec3(&reference["com"])?.distance(vec3(&coarse["com"])?);
                let dj = vec3(&reference["j"])?.distance(vec3(&coarse["j"])?);
                matched_ref = same
                    && dc < 1e-7
                    && dj < 1e-5
                    && num(&reference["arc_energy_defect"])?.abs() < 1e-7;
                for (key, z) in [
                    ("mesh_j_ns", dj),
                    ("mesh_com_m", dc),
                    (
                        "closure_inclusive_energy_j",
                        num(&reference["energy_defect"])?.abs(),
                    ),
                    ("arc_energy_j", num(&reference["arc_energy_defect"])?.abs()),
                ] {
                    add(&mut fields, "mechanics", key, z, id);
                }
                close((num(&reference["elapsed"])? - 0.0005).abs(), 1e-14)?;
                let sum = vec3(&reference["j"])?;
                let angular = vec3(&reference["angular"])?;
                close(
                    (2. * (vec3(&reference["velocity"])? + deck
                        - vec3(&t["cut"]["com_velocity_m_s"])?
                        + 0.0005 * 9.81 * DVec3::Z)
                        - sum)
                        .length(),
                    2e-9,
                )?;
                close(
                    (0.05
                        * (vec3(&reference["omega"])?
                            - vec3(&t["cut"]["foot_angular_velocity_rad_s"])?)
                        - angular)
                        .length(),
                    2e-9,
                )?;
                if matched_ref {
                    count(&mut counts, "qualified_physical");
                }
            } else {
                let end = reference["end"].as_str().ok_or("no physical end")?;
                count(&mut counts, &format!("unresolved_{end}"));
            }
            let origin = uint(&t["cut"]["tick"])?;
            let mut errs = vec![];
            for (i, r) in rows.iter().enumerate() {
                let name = LABELS[i];
                let end = &r["first"];
                if r["profile"] != name
                    || end["tick"] != origin + 1
                    || end["native_steps"] != [2, 2, 8, 8, 8, 8, 8, 64, 4096, 8192][i]
                    || num(&end["other_pair_normal_ns"])? != 0.
                {
                    return Err("wrong primary native call count or clock".into());
                }
                let all = array(&r["hits"])?;
                let mut ns = 0;
                let mut covered = false;
                let mut previous_time = 0.;
                if let Some(h) = all.first() {
                    let at = num(&h["at_s"])?;
                    ns = (at * 1e9).round() as u64;
                    covered = t["geometry"]["hi_ns"] == ns
                        && vec3(&t["geometry"]["instantaneous_impulse_ns"])?
                            .distance(vec3(&h["j"])?)
                            < 1e-8;
                    count(&mut counts, &format!("{name}_projected"));
                }
                for h in all {
                    let at = num(&h["at_s"])?;
                    if at < previous_time
                        || at > 0.0005
                        || vec3(&h["j"])?.dot(vec3(&h["normal"])?) <= 0.
                    {
                        return Err("invalid original-clock impulse".into());
                    }
                    previous_time = at;
                }
                for s in array(&r["spans"])? {
                    let at = uint(&s[0])?;
                    let dt = uint(&s[1])?;
                    if at < ns || dt == 0 || at + dt > 500000 {
                        return Err("invalid original-clock span".into());
                    }
                    covered &= at == ns;
                    ns = at + dt;
                }
                covered &= ns == 500000;
                if covered {
                    close(vec3(&r["j"])?.distance(vec3(&r["resolved_j"])?), 5e-11)?;
                    close(
                        (num(&r["resolved_normal"])? - num(&end["normal_impulse_ns"])?).abs(),
                        5e-11,
                    )?;
                }
                if !matched_ref {
                    continue;
                }
                let target = vec3(&reference["com"])? + 0.0005 * deck;
                let dc = body(end, "position")?.distance(target);
                let dj = vec3(&r["j"])?.distance(vec3(&reference["j"])?);
                for (key, z) in [
                    ("com_m", dc),
                    ("j_ns", dj),
                    (
                        "normal_ns",
                        (num(&end["normal_impulse_ns"])? - num(&reference["jn"])?).abs(),
                    ),
                    (
                        "velocity_m_s",
                        body(end, "velocity")?.distance(vec3(&reference["velocity"])? + deck),
                    ),
                    (
                        "spin_rad_s",
                        vec3(&end["foot"]["angular_velocity"])?
                            .distance(vec3(&reference["omega"])?),
                    ),
                ] {
                    add(&mut fields, name, key, z, id);
                }
                if covered {
                    if all.len() != array(&reference["events"])?.len() || dc > 1e-8 || dj > 5e-6 {
                        return Err(format!(
                            "fully covered primary differs {id} {name}: {dc:e} {dj:e}"
                        )
                        .into());
                    }
                    for (e, b) in all.iter().zip(array(&reference["events"])?) {
                        let time = num(&e["at_s"])?;
                        let j = vec3(&e["j"])?.distance(vec3(&b["j"])?);
                        if (time - num(&b["at_s"])?).abs() > 2e-9
                            || j > 5e-6
                            || e["stuck"] != b["stuck"]
                            || vec3(&e["point"])?.distance(vec3(&b["point"])? + time * deck) > 2e-8
                        {
                            return Err(
                                format!("primary event readback changed {id} {name}").into()
                            );
                        }
                    }
                    count(&mut counts, &format!("{name}_whole"));
                    add(&mut fields, name, "whole_j_error_ns", dj, id);
                }
                errs.push(dc);
            }
            if !matched_ref {
                continue;
            }
            for (i, j) in [
                (1, 0),
                (3, 2),
                (4, 3),
                (5, 3),
                (5, 4),
                (6, 5),
                (7, 5),
                (1, 2),
                (5, 9),
            ] {
                let d = errs[i] - errs[j];
                let group = format!("{}_vs_{}", LABELS[i], LABELS[j]);
                let rank = if d < -1e-13 {
                    "closer"
                } else if d > 1e-13 {
                    "farther"
                } else {
                    "tie"
                };
                count(&mut counts, &format!("{group}_{rank}"));
                add(&mut fields, &group, "signed_com_error_change_m", d, id);
            }
            let delta =
                body(&rows[5]["first"], "position")?.distance(body(&rows[3]["first"], "position")?);
            if largest.is_none_or(|(v, _)| delta > v) {
                largest = Some((delta, id));
            }
            let worse = errs[5] - errs[3];
            if regression.is_none_or(|(v, _)| worse > v) {
                regression = Some((worse, id));
            }
        }
    }
    write(
        &dir.join("summary.json"),
        &json!({"complete":counts.get("fixtures")==Some(&1250),"counts":counts,"largest_launch8_vs_endpoint8_m_case":largest,"largest_launch8_regression_m_case":regression,"rows":values(fields)}),
    )
}
