// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Relate the effect of fixed-time relaxation to the independently located first finite feature.
use super::*;
const REFS: [Profile; 2] = [FINE, ALT];
#[derive(Clone, Deserialize)]
struct State {
    i: u64,
    com_m: [f64; 3],
    velocity_m_s: [f64; 3],
    spin_rad_s: [f64; 3],
    orientation_xyzw: [f64; 4],
    cumulative_normal_ns: f64,
    cumulative_tangent_ns: [f64; 3],
}
#[derive(Deserialize)]
struct PathRecord {
    profile: String,
    first_loaded_native_end_ns: Option<u64>,
    samples: Vec<State>,
    impacts: Vec<Value>,
}
#[derive(Deserialize)]
struct FiniteTrial {
    stroke: usize,
    cut: Value,
    geometry: Value,
    paths: Vec<PathRecord>,
}
#[derive(Deserialize)]
struct FiniteCase {
    fixture: Fixture,
    trials: Vec<FiniteTrial>,
}
struct Joined<'a> {
    cell: &'a Cell,
    group: String,
    feature: String,
    mode: String,
    admitted: bool,
    comparison: [PathRecord; 2],
    roundoff: [f64; 6],
    same_time: bool,
    jump: Option<f64>,
    jump_time: Option<i64>,
    min_window: Option<u64>,
    stable: bool,
}
fn vector(v: &Value) -> Result<DVec3> {
    let x: [f64; 3] = serde_json::from_value(v.clone())?;
    Ok(DVec3::from(x))
}
fn labels(v: &Value) -> Result<(String, String)> {
    let status = if v.is_null() {
        "missing_geometry"
    } else {
        v["status"]
            .as_str()
            .ok_or("missing prospect classification")?
    };
    if status != "crossing" {
        return Ok((status.to_string(), "no_isolated_law".into()));
    }
    let feature = v["feature"]
        .as_str()
        .ok_or("missing geometric first feature")?
        .split('(')
        .next()
        .unwrap();
    if !["SupportFace", "MovingFace", "EdgePair"].contains(&feature) {
        return Err("unrecognized finite contact family".into());
    }
    let mode = match v["sticking"].as_bool() {
        Some(true) => "sticking",
        Some(false) => "sliding",
        None => "no_isolated_law",
    };
    Ok((feature.into(), mode.into()))
}
fn footprint(com: [f64; 3], orientation: [f64; 4], offset_m: f64) -> bool {
    // SAT for the projected sole parallelogram and the known horizontal support rectangle.
    let q = DQuat::from_array(orientation);
    let center = DVec3::from(com) - q * DVec3::new(0., 0., 0.13) - DVec3::X * offset_m;
    let a = q * DVec3::X * 0.18;
    let b = q * DVec3::Y * 0.10;
    [
        DVec3::X,
        DVec3::Y,
        DVec3::new(a.y, -a.x, 0.),
        DVec3::new(b.y, -b.x, 0.),
    ]
    .into_iter()
    .all(|axis| {
        center.dot(axis).abs()
            <= a.dot(axis).abs() + b.dot(axis).abs() + 0.33 * axis.x.abs() + 0.18 * axis.y.abs()
    })
}
impl Joined<'_> {
    fn viable(&self, w: usize) -> bool {
        self.min_window.is_none_or(|t| t > WINDOWS[w])
            && [BASE, CAND, Profile::onset(8), FINE, ALT]
                .iter()
                .all(|&p| self.cell.endpoint(p, w).is_ok())
    }
    fn physical(&self, lane: usize, w: usize) -> &State {
        &self.comparison[lane].samples[[0, 2, 3][w]]
    }
    fn diffs(&self, g: Profile, w: usize, policy: usize) -> [f64; 4] {
        let reference = self.cell.paths[&g].ends[w];
        let measure = |com: [f64; 3], spin: [f64; 3], j: f64, tangent: [f64; 3]| {
            [
                1000. * DVec3::from(com).distance(DVec3::from(reference.com_m)),
                DVec3::from(spin).distance(DVec3::from(reference.spin_rad_s)),
                (j - reference.cumulative_normal_ns).abs(),
                DVec3::from(tangent).distance(DVec3::from(reference.cumulative_tangent_ns)),
            ]
        };
        if policy == 2 {
            let [base, active] = [0, 1].map(|i| {
                let s = self.physical(i, w);
                measure(
                    s.com_m,
                    s.spin_rad_s,
                    s.cumulative_normal_ns,
                    s.cumulative_tangent_ns,
                )
            });
            std::array::from_fn(|i| active[i] - base[i])
        } else {
            let [base, active] =
                [BASE, if policy == 0 { CAND } else { Profile::onset(8) }].map(|p| {
                    let s = self.cell.paths[&p].ends[w];
                    measure(
                        s.com_m,
                        s.spin_rad_s,
                        s.cumulative_normal_ns,
                        s.cumulative_tangent_ns,
                    )
                });
            std::array::from_fn(|i| active[i] - base[i])
        }
    }
    fn projected_matches(&self) -> bool {
        self.admitted && self.jump.is_some()
    }
}
fn join<'a>(cell: &'a Cell, tr: FiniteTrial) -> Result<Joined<'a>> {
    if cell.cut != tr.cut {
        return Err("finite and relaxation incoming state differ".into());
    }
    let mut paths = BTreeMap::new();
    for path in tr.paths {
        if paths.insert(path.profile.clone(), path).is_some() {
            return Err("duplicate finite path".into());
        }
    }
    let regular = paths
        .remove("native8")
        .ok_or("no finite-study eight-call baseline")?;
    let finite = paths
        .remove("finite8")
        .ok_or("no finite-study first-interval projection")?;
    let incoming = cell.paths[&BASE];
    let archived = [
        incoming.ends[0],
        incoming.second,
        incoming.ends[1],
        incoming.ends[2],
    ];
    let mut roundoff = [0f64; 6];
    let offset = f64::from(cell.site.offset_mm) * 0.001;
    let alternate = cell.paths[&ALT];
    let mut min_window = [
        alternate.ends[0],
        alternate.second,
        alternate.ends[1],
        alternate.ends[2],
    ]
    .into_iter()
    .find(|s| !footprint(s.com_m, s.orientation_xyzw, offset))
    .map(|s| s.i);
    for (p, samples) in [(&regular, &regular.samples), (&finite, &finite.samples)] {
        if samples.len() != 4 {
            return Err("missing finite checkpoints".into());
        }
        for (s, other) in samples.iter().zip(archived) {
            if s.i != other.i {
                return Err("different sampling clocks".into());
            }
            if !footprint(s.com_m, s.orientation_xyzw, offset) {
                min_window = Some(min_window.map_or(s.i, |old: u64| old.min(s.i)));
            }
            if p.profile == "native8" {
                let dq = s
                    .orientation_xyzw
                    .iter()
                    .zip(other.orientation_xyzw)
                    .map(|(a, b)| (a - b).abs())
                    .fold(0f64, f64::max);
                let deltas = [
                    DVec3::from(s.com_m).distance(other.com_m.into()),
                    DVec3::from(s.velocity_m_s).distance(other.velocity_m_s.into()),
                    DVec3::from(s.spin_rad_s).distance(other.spin_rad_s.into()),
                    dq,
                    (s.cumulative_normal_ns - other.cumulative_normal_ns).abs(),
                    DVec3::from(s.cumulative_tangent_ns)
                        .distance(other.cumulative_tangent_ns.into()),
                ];
                for (max, x) in roundoff.iter_mut().zip(deltas) {
                    if !x.is_finite() {
                        return Err("nonfinite cross-driver difference".into());
                    }
                    *max = max.max(x);
                }
            }
        }
    }
    if finite.impacts.iter().any(|h| h["i"] != 1) || !regular.impacts.is_empty() {
        return Err("misidentified first-interval treatments".into());
    }
    let (feature, mode) = labels(&tr.geometry)?;
    let mut jump = None;
    let mut jump_time = None;
    let admitted = !finite.impacts.is_empty();
    if let Some(first) = finite.impacts.first()
        && tr.geometry["instantaneous_impulse_ns"].is_array()
    {
        // All signed vectors are on the foot in world coordinates. The instantaneous
        // comparison only applies when the independently predicted feature also matches.
        if first["feature"] == tr.geometry["feature"]
            && first["sticking"] == tr.geometry["sticking"]
        {
            jump = Some(
                vector(&first["impulse_ns"])?
                    .distance(vector(&tr.geometry["instantaneous_impulse_ns"])?),
            );
            jump_time = Some(
                first["local_ns"]
                    .as_i64()
                    .ok_or("missing first finite time")?
                    - tr.geometry["hi_ns"]
                        .as_i64()
                        .ok_or("missing geometric time")?,
            );
        }
    }
    let stable =
        roundoff[0] <= 1e-6 && roundoff[2] <= 1e-3 && roundoff[4] <= 1e-4 && roundoff[5] <= 1e-4;
    let same_time = regular.first_loaded_native_end_ns == cell.baseline_first;
    Ok(Joined {
        cell,
        group: format!("{feature}/{mode}"),
        feature,
        mode,
        admitted,
        comparison: [regular, finite],
        roundoff,
        same_time,
        jump,
        jump_time,
        min_window,
        stable,
    })
}
fn outcomes(rows: &[&Joined]) -> Value {
    let windows = [1, 2].map(|window| {
        let eligible: Vec<_> = rows.iter().copied().filter(|row| row.viable(window)).collect();
        let treatments = ["eight_continuous", "eight_onset", "finite_onset"]
            .into_iter()
            .enumerate()
            .map(|(policy, name)| {
                let mut improved = [0; 4];
                let mut differences = [Vec::new(), Vec::new()];
                for row in &eligible {
                    let [a, b] = REFS.map(|reference| row.diffs(reference, window, policy));
                    improved[usize::from(a[0] < 0.) + 2 * usize::from(b[0] < 0.)] += 1;
                    differences[0].push(a);
                    differences[1].push(b);
                }
                let by_reference: Vec<_> = REFS.into_iter().zip(differences)
                    .map(|(reference, values)| {
                        let metrics: [Value; 4] = std::array::from_fn(|i| stats(values.iter().map(|v| v[i]).collect()));
                        json!({"reference":reference.label(),"delta_distance":metrics})
                    })
                    .collect();
                json!({"treatment":name,"delta_by_reference":by_reference,"improved_reference_bitmask":improved})
            })
            .collect::<Vec<_>>();
        json!({"step":WINDOWS[window],"qualified":eligible.len(),"excluded":rows.len()-eligible.len(),"treatments":treatments})
    });
    json!({"n":rows.len(),"windows":windows})
}
fn decision(rows: &[&Joined]) -> bool {
    let matched = rows
        .iter()
        .copied()
        .filter(|r| r.viable(2))
        .collect::<Vec<_>>();
    !matched.is_empty()
        && REFS
            .into_iter()
            .all(|g| matched.iter().map(|r| r.diffs(g, 2, 0)[0]).sum::<f64>() < 0.)
}
fn partition<'row, 'cell>(
    rows: &[&'row Joined<'cell>],
    key: impl Fn(&Joined<'cell>) -> String,
) -> BTreeMap<String, Vec<&'row Joined<'cell>>> {
    let mut bins = BTreeMap::<String, Vec<_>>::new();
    for &r in rows {
        bins.entry(key(r)).or_default().push(r);
    }
    bins
}
pub(in super::super) fn export(
    finite: &Path,
    relax: &Path,
    original: &Path,
    out: &Path,
) -> Result<()> {
    let census = Census::load(relax, Some(original))?;
    let existing: BTreeMap<_, _> = census
        .cells
        .iter()
        .map(|c| ((c.site.index, c.site.stroke), c))
        .collect();
    let mut published = vec![];
    let mut joined = vec![];
    let mut absent = 0;
    let mut pending_ref = 0;
    for index in 0..FIXTURE_COUNT {
        let Some(batch) = load_file::<Saved<FiniteCase>>(
            &finite.join("cases").join(format!("case-{index:04}.json")),
        )?
        else {
            continue;
        };
        if batch.index != index
            || batch.format != "finite-projection-primary-v1"
            || batch.horizon != 26
            || batch.outer_us != 500
        {
            return Err("invalid finite source".into());
        }
        let k = 613 * index % 1250;
        if batch.result.fixture.offset_m != (165 + 5 * (k / 50)) as f64 / 1000.
            || batch.result.fixture.speed_scale != (76 + 2 * ((k / 2) % 25)) as f64 / 100.
            || batch.result.fixture.switched != (k % 2 == 1)
        {
            return Err("incorrect finite axis".into());
        }
        let trials = batch.result.trials;
        if trials.len() > 3 {
            return Err("extra donor stroke".into());
        }
        absent += 3 - trials.len();
        published.push(index);
        if !census.matched_default.contains(&index) {
            pending_ref += trials.len();
            continue;
        }
        let mut seen = BTreeSet::new();
        for tr in trials {
            if !seen.insert(tr.stroke) {
                return Err("duplicate finite trial".into());
            }
            let cell = *existing
                .get(&(index, tr.stroke))
                .ok_or("finite donor missing from relaxation field")?;
            joined.push(
                join(cell, tr)
                    .map_err(|e| format!("feature join {index}/{}: {e}", cell.site.stroke))?,
            );
        }
        if (1..=3).any(|s| existing.contains_key(&(index, s)) != seen.contains(&s)) {
            return Err("differing donor coverage".into());
        }
    }
    let (held, train): (Vec<_>, Vec<_>) = joined
        .iter()
        .partition(|r| strata::HELD.contains(&r.cell.site.offset_mm));
    let training = partition(&train, |r| r.group.clone());
    let evaluation = partition(&held, |r| r.group.clone());
    let keys = training
        .keys()
        .chain(evaluation.keys())
        .cloned()
        .collect::<BTreeSet<_>>();
    let mut rules = BTreeMap::new();
    let mut bins = vec![];
    for key in keys {
        let tr = training.get(&key).map_or(&[][..], Vec::as_slice);
        let test = evaluation.get(&key).map_or(&[][..], Vec::as_slice);
        let eight = decision(tr);
        rules.insert(key.clone(), eight);
        bins.push(json!({"feature_and_mode":key,"training":outcomes(tr),"held":outcomes(test),"choose_eight":eight}));
    }
    let all = joined.iter().collect::<Vec<_>>();
    let stable = held
        .iter()
        .copied()
        .filter(|r| r.stable)
        .collect::<Vec<_>>();
    let classify = |r: &Joined| rules.get(&r.group).copied().unwrap_or(false);
    let policies = REFS.map(|reference| {
        [1, 2].map(|window| {
            let mut one = Vec::new();
            let mut eight = Vec::new();
            let mut selected = Vec::new();
            let mut differences = Vec::new();
            let mut selected_eight = 0;
            for row in held.iter().copied().filter(|row| row.viable(window)) {
                let decision = classify(row);
                let baseline = row.cell.paths[&BASE].ends[window];
                let target = row.cell.paths[&reference].ends[window];
                let low = 1000. * DVec3::from(baseline.com_m).distance(target.com_m.into());
                let high = 1000. * DVec3::from(row.cell.paths[&CAND].ends[window].com_m).distance(target.com_m.into());
                selected_eight += usize::from(decision);
                one.push(low);
                eight.push(high);
                selected.push(if decision { high } else { low });
                differences.push(if decision { high - low } else { 0. });
            }
            json!({
                "reference":reference.label(),"step":WINDOWS[window],"one_mm":stats(one),"eight_mm":stats(eight),
                "feature_selected_mm":stats(selected),"change_vs_one_mm":stats(differences),"selected_eight":selected_eight
            })
        })
    });
    let winner = held
        .iter()
        .copied()
        .filter(|r| {
            r.projected_matches()
                && r.viable(2)
                && REFS.iter().all(|&g| r.diffs(g, 2, 2)[0] < 0.)
                && r.diffs(FINE, 2, 0)[0] > 0.
        })
        .max_by(|a, b| {
            a.diffs(FINE, 2, 0)[0]
                .total_cmp(&b.diffs(FINE, 2, 0)[0])
                .then(
                    (b.cell.site.index, b.cell.site.stroke)
                        .cmp(&(a.cell.site.index, a.cell.site.stroke)),
                )
        });
    let max_drift: [Value; 6] =
        std::array::from_fn(|i| stats(joined.iter().map(|r| r.roundoff[i]).collect()));
    let by_accept = [false, true].map(|admit| {
        let rows = held
            .iter()
            .copied()
            .filter(|r| r.admitted == admit)
            .collect::<Vec<_>>();
        json!({"admitted":admit,"held":outcomes(&rows)})
    });
    let by_status = partition(&all, |r| r.feature.clone())
        .into_iter()
        .map(|(k, v)| (k, v.len()))
        .collect::<BTreeMap<_, _>>();
    fs::create_dir_all(out)?;
    let mut file = BufWriter::new(fs::File::create(out.join("feature-cuts.jsonl"))?);
    for r in &joined {
        serde_json::to_writer(
            &mut file,
            &json!({"site":r.cell.site,"feature":r.feature,"point_mode":r.mode,"native_admitted":r.admitted,
            "stable_controls":r.stable,"driver_drift":r.roundoff,"matched_point_jump_error_ns":r.jump,"matched_point_time_difference_ns":r.jump_time,"selected_eight":classify(r)}),
        )?;
        file.write_all(b"\n")?;
    }
    file.flush()?;
    let result = json!({"format":"physical-first-feature-and-native-relaxation-v1","finite_fixtures":published.len(),"finite_indices":published,"pending_reference_strokes":pending_ref,
        "missing_donor_strokes":absent,"joined":joined.len(),"training":train.len(),"held":held.len(),"held_offsets_mm":strata::HELD,"final":published.len()==1250&&pending_ref==0,
        "feature_counts":by_status,"groups":bins,"held_rules":policies,"held_all":outcomes(&held),"held_stable_controls":outcomes(&stable),"held_by_native_admission":by_accept,
        "ordinary_control_drift_fields":["COM m","velocity m/s","spin rad/s","quaternion component","cumulative normal N s","cumulative tangent vector N s"],
        "worst_per_cut_drift":max_drift,"ordinary_first_positive_time_exact":joined.iter().filter(|r|r.same_time).count(),"stable_ordinary_controls":joined.iter().filter(|r|r.stable).count(),
        "geometry_vs_transaction":{"native_admitted":joined.iter().filter(|r|r.admitted).count(),"matched_feature_and_law":joined.iter().filter(|r|r.projected_matches()).count(),
            "instantaneous_vector_error_ns":stats(joined.iter().filter_map(|r|r.jump).collect()),"signed_time_difference_ns":stats(joined.iter().filter_map(|r|r.jump_time.map(|t|t as f64)).collect())},
        "metric_order":["change in COM distance mm","change in spin distance rad/s","change in absolute cumulative normal error N s","change in cumulative tangential vector distance N s"],
        "reference_bitmask_order":["neither","4096 eight only","4096 one only","both"],"finite_eligibility":"finite paths and historical 4096 r1 reference checked for projected sole overlap at saved endpoints 1,2,10,26; relaxation-field paths record first projected footprint loss at any public endpoint",
        "outlier":winner.map(|r|json!({"site":r.cell.site,"group":r.group,"finite_minus_own_baseline_mm":REFS.map(|g|r.diffs(g,2,2)[0]),"continuous_eight_minus_one_mm":REFS.map(|g|r.diffs(g,2,0)[0])}))});
    fs::write(
        out.join("feature-relaxation.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    plot(&result, &out.join("feature-relaxation.svg"))
}
fn plot(report: &Value, path: &Path) -> Result<()> {
    use plotters::prelude::*;
    let groups = report["groups"]
        .as_array()
        .ok_or("missing feature strata")?;
    let root = SVGBackend::new(path, (1180, 180 + groups.len() as u32 * 75)).into_drawing_area();
    root.fill(&WHITE)?;
    root.draw(&Text::new(
        "Relaxation after different first finite features",
        (25, 39),
        ("sans-serif", 25).into_font(),
    ))?;
    root.draw(&Text::new("Held-out means at 13 ms • change in COM error (µm) relative to each treatment's own ordinary eight-call baseline",(25,69),("sans-serif",15).into_font()))?;
    let mut m = 1f64;
    for g in groups {
        for &p in &[0, 2] {
            for r in 0..2 {
                if let Some(x)=g["held"]["windows"][1]["treatments"][p]["delta_by_reference"][r]["delta_distance"][0]["mean"].as_f64(){m=m.max(1000.*x.abs());}
            }
        }
    }
    for (j, g) in groups.iter().enumerate() {
        let y = 131 + j as i32 * 75;
        let name = g["feature_and_mode"].as_str().ok_or("feature name")?;
        let n = &g["held"]["windows"][1]["qualified"];
        root.draw(&Text::new(
            format!("{name}  (n={n})"),
            (22, y),
            ("sans-serif", 15).into_font(),
        ))?;
        for (k, (p, r, title, color)) in [
            (0, 0, "Eight passes vs 4096 r8", RGBColor(25, 115, 102)),
            (0, 1, "Eight passes vs 4096 r1", RGBColor(185, 85, 28)),
            (2, 0, "Finite vs 4096 r8", RGBColor(25, 115, 102)),
            (2, 1, "Finite vs 4096 r1", RGBColor(185, 85, 28)),
        ]
        .into_iter()
        .enumerate()
        {
            let x = 338 + k as i32 * 215;
            if j == 0 {
                root.draw(&Text::new(
                    title,
                    (x - 80, 95),
                    ("sans-serif", 12).into_font(),
                ))?;
            }
            root.draw(&PathElement::new(
                [(x, y - 15), (x, y + 10)],
                BLACK.mix(0.35),
            ))?;
            let v =
                g["held"]["windows"][1]["treatments"][p]["delta_by_reference"][r]["delta_distance"]
                    [0]["mean"]
                    .as_f64();
            if let Some(delta) = v {
                let um = 1000. * delta;
                let tip = x + (um / m * 77.).round() as i32;
                root.draw(&PathElement::new([(x, y), (tip, y)], color.stroke_width(4)))?;
                root.draw(&Circle::new((tip, y), 3, color.filled()))?;
                root.draw(&Text::new(
                    format!("{um:+.1}"),
                    (x - 20, y + 25),
                    ("sans-serif", 13).into_font(),
                ))?;
            } else {
                root.draw(&Text::new(
                    "No data",
                    (x - 26, y + 25),
                    ("sans-serif", 12).into_font(),
                ))?;
            }
        }
    }
    root.present()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tilted_sole_uses_its_projected_extent() {
        let com = [0., 0., 0.13];
        assert!(footprint(com, DQuat::IDENTITY.to_array(), 0.509_999));
        assert!(!footprint(com, DQuat::IDENTITY.to_array(), 0.510_001));
        let sideways = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2).to_array();
        assert!(footprint(com, sideways, 0.429_999));
        assert!(!footprint(com, sideways, 0.430_001));
    }

    #[test]
    fn a_geometric_crossing_can_lack_a_unique_point_law() {
        let prospect = json!({"status":"crossing","feature":"EdgePair(0, 1)","sticking":null});
        assert_eq!(
            labels(&prospect).unwrap(),
            ("EdgePair".into(), "no_isolated_law".into())
        );
        assert_eq!(
            labels(&Value::Null).unwrap(),
            ("missing_geometry".into(), "no_isolated_law".into())
        );
        assert!(labels(&json!({"status":"crossing","feature":"Invalid"})).is_err());
    }
}
