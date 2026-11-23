// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare fixed-duration and causal receipt-only relaxation from verified identical donor states.
use super::*;

const SPANS: [Option<usize>; 6] = [Some(1), Some(2), Some(4), Some(8), Some(26), None];
const WEIGHT: f64 = 2. * 9.81 * 0.0005;
const LIM: [f64; 3] = [0., 1e-4, WEIGHT];
const REFERENCES: [Profile; 2] = [FINE, ALT];
#[derive(Deserialize)]
struct Campaign {
    fixture: Fixture,
    trials: Vec<Trial>,
}
#[derive(Deserialize)]
struct Trial {
    stroke: usize,
    cut: Value,
    paths: Vec<Branch>,
}
#[derive(Deserialize)]
struct Branch {
    span: Option<usize>,
    eight_pass_ticks: usize,
    weight_threshold_ns: f64,
    normal_ns: Vec<f64>,
    first_positive_end_ns: Option<u64>,
    first_disjoint_tick: Option<u64>,
    samples: Vec<State>,
}
#[derive(Deserialize, PartialEq)]
struct State {
    i: u64,
    com_m: [f64; 3],
    velocity_m_s: [f64; 3],
    orientation_xyzw: [f64; 4],
    spin_rad_s: [f64; 3],
    normal_ns: f64,
    cumulative_normal_ns: f64,
    footprint_gap_m: Option<f64>,
}
struct Row<'a> {
    cell: &'a Cell,
    paths: Vec<Branch>,
}
impl State {
    fn verify(&self, e: Endpoint) -> Result<()> {
        if self.i != e.i
            || self.com_m != e.com_m
            || self.velocity_m_s != e.velocity_m_s
            || self.orientation_xyzw != e.orientation_xyzw
            || self.spin_rad_s != e.spin_rad_s
            || (self.normal_ns - e.normal_ns).abs() > 1e-11
            || (self.cumulative_normal_ns - e.cumulative_normal_ns).abs() > 1e-11
            || self.footprint_gap_m != e.footprint_gap_m
        {
            return Err(
                "ordinary interval and independently observed native state diverged".into(),
            );
        }
        Ok(())
    }
}
fn loaded(j: f64, t: f64) -> bool {
    if t == WEIGHT { j >= t } else { j > t }
}
fn release(normals: &[f64], threshold: f64) -> (Option<usize>, Option<usize>) {
    let off = normals.iter().position(|&j| !loaded(j, threshold));
    let on = off.and_then(|off| {
        normals
            .iter()
            .enumerate()
            .skip(off + 1)
            .find(|(_, j)| loaded(**j, threshold))
            .map(|(i, _)| i + 1)
    });
    (off.map(|i| i + 1), on)
}
impl Row<'_> {
    fn verify(&self) -> Result<()> {
        if self.paths.len() != SPANS.len() {
            return Err("missing or extra span pathway".into());
        }
        let first = &self.paths[0]
            .samples
            .first()
            .ok_or("missing first sample")?;
        let common_end = self.paths[0]
            .first_positive_end_ns
            .filter(|&x| x <= 500_000);
        for (i, p) in self.paths.iter().enumerate() {
            if p.span != SPANS[i]
                || p.normal_ns.len() != 26
                || p.samples.len() != 4
                || p.normal_ns.iter().any(|x| !x.is_finite() || *x < 0.)
                || p.first_positive_end_ns.filter(|&x| x <= 500_000) != common_end
                || &p.samples[0] != *first
                || (p.weight_threshold_ns - WEIGHT).abs() > 1e-15
            {
                return Err("span pathway missing a shared receipt, full clock or schedule".into());
            }
            let budget = SPANS[i].unwrap_or_else(|| release(&p.normal_ns, WEIGHT).0.unwrap_or(26));
            if budget != p.eight_pass_ticks {
                return Err("configured span does not follow causal release rule".into());
            }
            // Prefixes must agree down to each public pair reaction before the first different solve.
            for other in &self.paths {
                let n = budget.min(other.eight_pass_ticks);
                if p.normal_ns[..n] != other.normal_ns[..n] {
                    return Err("native public prefixes differ prior to treatment".into());
                }
            }
            for (&step, s) in [1, 2, 10, 26].iter().zip(&p.samples) {
                if s.i as usize != step
                    || (s.normal_ns - p.normal_ns[step - 1]).abs() > 1e-12
                    || (s.cumulative_normal_ns - p.normal_ns[..step].iter().sum::<f64>()).abs()
                        > 1e-11
                    || !DVec3::from(s.com_m).is_finite()
                    || !DVec3::from(s.spin_rad_s).is_finite()
                {
                    return Err("invalid physical sample".into());
                }
            }
            if matches!(i, 0 | 4) {
                let archived = &self.cell.paths[&if i == 0 { Profile::onset(8) } else { CAND }];
                if archived.overlap_lost != p.first_disjoint_tick {
                    return Err("footprint loss differs from observation".into());
                }
                for (a, b) in [0, 2, 3].into_iter().zip(archived.ends) {
                    p.samples[a].verify(b)?;
                }
            }
        }
        Ok(())
    }
    fn all_match(&self, w: usize) -> bool {
        let stop = self.cell.site.cut_tick + WINDOWS[w];
        self.paths
            .iter()
            .all(|p| p.first_disjoint_tick.is_none_or(|t| t > stop))
            && REFERENCES.iter().all(|&r| self.cell.endpoint(r, w).is_ok())
    }
    fn com_error(&self, p: usize, r: Profile, w: usize) -> f64 {
        let a = DVec3::from(self.paths[p].samples[[0, 2, 3][w]].com_m);
        a.distance(DVec3::from(self.cell.paths[&r].ends[w].com_m)) * 1000.
    }
}
fn means(rows: &[&Row], p: usize, r: Profile) -> f64 {
    let mut sum = 0.;
    let mut n = 0;
    for row in rows.iter().filter(|row| row.all_match(2)) {
        sum += row.com_error(p, r, 2);
        n += 1;
    }
    sum / n as f64
}
fn scores(rows: &[&Row], p: usize) -> Value {
    let windows=[1,2].map(|w|{
        let joined=rows.iter().copied().filter(|r|r.all_match(w)).collect::<Vec<_>>();
        let dr=REFERENCES.map(|reference|{
            let mut errors=vec![];let mut deltas=vec![];let mut spins=vec![];let mut impulse_delta=vec![];
            for r in &joined {
                let err=r.com_error(p,reference,w);
                let q=r.cell.paths[&reference].ends[w];let a=&r.paths[p].samples[[0,2,3][w]];
                errors.push(err);deltas.push(err-r.com_error(0,reference,w));
                spins.push(DVec3::from(a.spin_rad_s).distance(DVec3::from(q.spin_rad_s)));
                impulse_delta.push(a.cumulative_normal_ns-q.cumulative_normal_ns);
            }
            json!({"reference":reference.label(),"com_error_mm":stats(errors),"change_vs_onset_mm":stats(deltas),"spin_error_rad_s":stats(spins),"cumulative_normal_delta_ns":stats(impulse_delta)})
        });
        json!({"tick":WINDOWS[w],"joined":joined.len(),"excluded":rows.len()-joined.len(),"scores":dr})
    });
    let releases=LIM.map(|lim|{
        let mut first=vec![]; let mut reacquired=vec![]; let mut active=0;
        for r in rows{
            let reactions=&r.paths[p].normal_ns;
            let (off,on)=release(reactions,lim);
            active+=reactions.iter().filter(|&&j|loaded(j,lim)).count();
            if let Some(off)=off {first.push(off as f64);}
            if let Some(on)=on {reacquired.push(on as f64);}
        }
        json!({"threshold_ns":lim,"loaded_public_intervals":active,"first_subthreshold_tick":stats(first),"first_later_reload_tick":stats(reacquired)})
    });
    json!({"policy":SPANS[p],"cuts":rows.len(),"eight_pass_intervals":stats(rows.iter().map(|r|r.paths[p].eight_pass_ticks as f64).collect()),
        "configured_unbiased_passes_per_native_call_summed_over_public_intervals":stats(rows.iter().map(|r|(26+7*r.paths[p].eight_pass_ticks)as f64).collect()),"windows":windows,"release":releases})
}
fn summarize(rows: &[&Row], selected: Option<usize>) -> Value {
    json!({"cuts":rows.len(),"policies":(0..6).map(|p|scores(rows,p)).collect::<Vec<_>>(),"selected_fixed_span":selected.and_then(|i|SPANS[i])})
}
// Similar center positions can hide a different supporting quadrant, angular velocity, or slip.
fn close_com_modes(rows: &[&Row]) -> Result<Value> {
    let mut paired = vec![];
    for r in rows.iter().copied().filter(|r| r.all_match(2)) {
        let classify = |p: usize| -> Result<(u8, u8, bool, bool)> {
            let path = &r.paths[p];
            let state = &path.samples[3];
            let motion = signature::measure(
                state.com_m,
                state.velocity_m_s,
                state.orientation_xyzw,
                state.spin_rad_s,
                state.footprint_gap_m.ok_or("missing supported endpoint")?,
                f64::from(r.cell.site.offset_mm) / 1000.,
            )?
            .ok_or("missing supported polygon")?;
            let load = if state.normal_ns == 0. {
                0
            } else if state.normal_ns <= 1e-4 {
                1
            } else if state.normal_ns < WEIGHT {
                2
            } else {
                3
            };
            // The level finite pad has no horizontal motion. Report in-plane point speed and spin separately from load.
            let still = u8::from(motion.prospective_tangent_m_s <= 1e-3)
                + 2 * u8::from(motion.spin_rad_s <= 1e-3);
            Ok((
                load,
                still,
                motion.supported_projected_area_fraction < 0.75,
                path.normal_ns.iter().skip(1).any(|&j| j >= WEIGHT),
            ))
        };
        let aperture = |p: usize| {
            DVec3::from(r.paths[p].samples[3].com_m)
                .distance(DVec3::from(r.paths[0].samples[3].com_m))
                * 1e6
        };
        paired.push((aperture(5), classify(0)?, classify(5)?));
    }
    Ok(json!([1.,10.,100.].map(|um|{
        let group=paired.iter().filter(|(d,_,_)|*d<=um).collect::<Vec<_>>();
        let mut loads=[[0;4];4];let mut still=[[0;4];4];let mut edges=[[0;2];2];let mut reloads=[[0;2];2];
        for (_,a,b) in &group {loads[a.0 as usize][b.0 as usize]+=1;still[a.1 as usize][b.1 as usize]+=1;
            edges[a.2 as usize][b.2 as usize]+=1;reloads[a.3 as usize][b.3 as usize]+=1;}
        json!({"com_distance_upper_um":um,"cuts":group.len(),"load_mode_onset_by_causal":loads,"stationarity_bits_onset_by_causal":still,
            "clipped_footprint_onset_by_causal":edges,"later_tick_at_or_above_weight_onset_by_causal":reloads})
    })))
}
pub(in super::super) fn export(
    dir: &Path,
    relax: &Path,
    original: &Path,
    out: &Path,
) -> Result<()> {
    let census = Census::load(relax, Some(original))?;
    let lookup: BTreeMap<_, _> = census
        .cells
        .iter()
        .map(|c| ((c.site.index, c.site.stroke), c))
        .collect();
    let mut rows = vec![];
    let mut published = Vec::new();
    let mut joined_indices = BTreeSet::new();
    let mut absent = 0;
    let mut missing_reference = 0;
    for index in 0..FIXTURE_COUNT {
        let Some(saved) =
            load_file::<Saved<Campaign>>(&dir.join("cases").join(format!("case-{index:04}.json")))?
        else {
            continue;
        };
        if saved.index != index
            || saved.format != "primary-continuation-span-v1"
            || saved.horizon != 26
            || saved.outer_us != 500
        {
            return Err("incompatible span artifact".into());
        }
        let k = index * 613 % 1250;
        let f = saved.result.fixture;
        if f.offset_m != (165 + 5 * (k / 50)) as f64 / 1000.
            || f.speed_scale != (76 + 2 * ((k / 2) % 25)) as f64 / 100.
            || f.switched != (k % 2 == 1)
        {
            return Err("span fixture location mismatch".into());
        }
        published.push(index);
        if census.matched_default.contains(&index) {
            joined_indices.insert(index);
        }
        if saved.result.trials.len() > 3 {
            return Err("too many donors".into());
        }
        absent += 3 - saved.result.trials.len();
        let mut seen = BTreeSet::new();
        for tr in saved.result.trials {
            if !(1..=3).contains(&tr.stroke) || !seen.insert(tr.stroke) {
                return Err("invalid stroke index".into());
            }
            let Some(&cell) = lookup.get(&(index, tr.stroke)) else {
                if census.completed.contains(&index) {
                    return Err("span/observer donor mismatch".into());
                }
                missing_reference += 1;
                continue;
            };
            if tr.cut != cell.cut {
                return Err("mismatched incoming native cut".into());
            }
            if !census.matched_default.contains(&index) {
                missing_reference += 1;
                continue;
            }
            let row = Row {
                cell,
                paths: tr.paths,
            };
            row.verify()
                .map_err(|e| format!("span fixture {index} stroke {}: {e}", tr.stroke))?;
            rows.push(row);
        }
        if census.completed.contains(&index)
            && (1..=3).any(|s| lookup.contains_key(&(index, s)) != seen.contains(&s))
        {
            return Err("span/observer stroke coverage differs".into());
        }
    }
    let (held, train): (Vec<_>, Vec<_>) = rows
        .iter()
        .partition(|r| strata::HELD.contains(&r.cell.site.offset_mm));
    let calibration_available = train.iter().any(|r| r.all_match(2));
    let scores_by_ref = REFERENCES.map(|g| (0..5).map(|p| means(&train, p, g)).collect::<Vec<_>>());
    let minima = scores_by_ref
        .each_ref()
        .map(|v| v.iter().copied().fold(f64::INFINITY, f64::min));
    let regrets = (0..5)
        .map(|p| (scores_by_ref[0][p] - minima[0]).max(scores_by_ref[1][p] - minima[1]))
        .collect::<Vec<_>>();
    let selected = (0..5)
        .min_by(|&a, &b| regrets[a].total_cmp(&regrets[b]).then(a.cmp(&b)))
        .filter(|_| calibration_available);
    let mut groups = BTreeMap::<String, Vec<&Row>>::new();
    for &r in &held {
        if let Some(incoming) = r.cell.incoming {
            let edge = if incoming.supported_projected_area_fraction < 0.75 {
                "clipped"
            } else {
                "higher_overlap"
            };
            let motion = if incoming.prospective_tangent_m_s < 0.0001 {
                "slow"
            } else {
                "moving"
            };
            groups
                .entry(format!("{edge}_{motion}"))
                .or_default()
                .push(r);
        }
        if let continuation::Receipt::Loaded {
            effective_quadrants,
            ..
        } = r.cell.first_receipt
        {
            groups
                .entry(
                    if effective_quadrants <= 2.5 {
                        "at_most_2_5_effective_quadrants"
                    } else {
                        "more_than_2_5_effective_quadrants"
                    }
                    .to_owned(),
                )
                .or_default()
                .push(r);
        }
    }
    let mut by_group = BTreeMap::new();
    for label in [
        "clipped_slow",
        "clipped_moving",
        "higher_overlap_slow",
        "higher_overlap_moving",
        "at_most_2_5_effective_quadrants",
        "more_than_2_5_effective_quadrants",
    ] {
        let g = groups.get(label).map_or(&[][..], Vec::as_slice);
        by_group.insert(label, summarize(g, selected));
    }
    let distance = |r: &Row| {
        DVec3::from(r.paths[5].samples[3].com_m).distance(DVec3::from(r.paths[4].samples[3].com_m))
            * 1000.
    };
    let worst = held
        .iter()
        .copied()
        .filter(|r| r.all_match(2))
        .max_by(|a, b| {
            distance(a).total_cmp(&distance(b)).then(
                (b.cell.site.index, b.cell.site.stroke)
                    .cmp(&(a.cell.site.index, a.cell.site.stroke)),
            )
        });
    fs::create_dir_all(out)?;
    let close_modes = close_com_modes(&held)?;
    let complete = published.len();
    let result = json!({"format":"continued-relaxation-duration-analysis-v1","scheduled":1250,"complete":complete,"completed_indices":published,"paired_reference_indices":joined_indices,"pending":1250-complete,
        "missing_donor_strokes":absent,"pending_reference_strokes":missing_reference,"joined_strokes":rows.len(),"selection_final":complete==1250&&missing_reference==0,
        "held_offsets_mm":strata::HELD,"configured_native_calls_per_public_tick":8,"temporal_iterations_per_native_call":4,"public_tick_us":500,
        "weight_threshold_ns":WEIGHT,"held_close_com_modes":close_modes,
        "mode_labels":{"loads":["zero","positive up to 1e-4 N s","above 1e-4 below weight","at or above weight"],
        "stationarity_bits":["moving point and rotating","point at most 1 mm/s only","spin at most 1 mrad/s only","both point and spin below thresholds"],
        "footprint":["at least 75% projected overlap","below 75% projected overlap"]},"span_order":SPANS,"reference_order":REFERENCES.map(Profile::label),"training_reference_mean_com_mm_by_fixed_span":scores_by_ref,
        "training_worst_mean_regret_mm":regrets,"selected_fixed_span":selected.and_then(|i|SPANS[i]),"training":summarize(&train,selected),"held_out":summarize(&held,selected),"held_strata":by_group,
        "outlier":worst.map(|r|json!({"site":r.cell.site,"causal_to_continuous_13ms_mm":distance(r),"eight_pass_ticks":r.paths[5].eight_pass_ticks,"per_policy_normal_ns":r.paths.iter().map(|p|&p.normal_ns).collect::<Vec<_>>()}))});
    fs::write(
        out.join("continuation-spans.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    plot(&result, &out.join("continuation-spans.svg"))?;
    plot_changes(&result, &out.join("continuation-span-effects.svg"))
}
fn plot(result: &Value, path: &Path) -> Result<()> {
    use plotters::prelude::*;
    let root = SVGBackend::new(path, (1030, 660)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root
        .titled(
            "How long to retain extra passes after touchdown",
            ("sans-serif", 24),
        )?
        .split_evenly((2, 1));
    let policies = result["held_out"]["policies"]
        .as_array()
        .ok_or("missing policies")?;
    let labels = ["1", "2", "4", "8", "26", "weight"];
    for (area, (w, title)) in areas.iter().zip([
        (0, "5 ms after the matched cut"),
        (1, "13 ms after the matched cut"),
    ]) {
        let value = |p: usize, r: usize, s: &str| {
            policies[p]["windows"][w]["scores"][r]["com_error_mm"][s]
                .as_f64()
                .unwrap_or(0.)
        };
        let max = (0..6)
            .flat_map(|p| (0..2).map(move |r| (p, r)))
            .map(|(p, r)| value(p, r, "p95"))
            .fold(0.001, f64::max)
            * 1.15;
        let n = policies[0]["windows"][w]["joined"]
            .as_u64()
            .ok_or("held-out count")?;
        if n == 0 {
            area.draw(&Text::new(
                format!("{title} · No matched held-out windows"),
                (35, 66),
                ("sans-serif", 20).into_font(),
            ))?;
            continue;
        }
        let mut chart = ChartBuilder::on(area)
            .margin(10)
            .caption(
                format!("{title} · {n} held-out paired landings"),
                ("sans-serif", 18),
            )
            .x_label_area_size(48)
            .y_label_area_size(66)
            .build_cartesian_2d(-0.5f64..5.5, 0.0f64..max)?;
        chart
            .configure_mesh()
            .max_light_lines(0)
            .x_labels(6)
            .x_label_formatter(&|v| {
                let i = v.round() as isize;
                if (v - i as f64).abs() < 1e-5 && (0..6).contains(&i) {
                    labels[i as usize].into()
                } else {
                    String::new()
                }
            })
            .x_desc("First public intervals with eight passes, or causal weight release")
            .y_desc("COM error (mm)")
            .draw()?;
        for (r, color, name) in [
            (0, RGBColor(18, 121, 112), "4096 calls, eight passes"),
            (1, RGBColor(187, 83, 26), "4096 calls, one pass"),
        ] {
            for p in 0..6 {
                let x = p as f64 + if r == 0 { -0.10 } else { 0.10 };
                chart.draw_series([PathElement::new(
                    [(x, value(p, r, "mean")), (x, value(p, r, "p95"))],
                    color.stroke_width(2),
                )])?;
                chart.draw_series([TriangleMarker::new(
                    (x, value(p, r, "p95")),
                    5,
                    color.filled(),
                )])?;
            }
            chart
                .draw_series((0..6).map(|p| {
                    Circle::new(
                        (
                            p as f64 + if r == 0 { -0.10 } else { 0.10 },
                            value(p, r, "mean"),
                        ),
                        5,
                        color.filled(),
                    )
                }))?
                .label(format!("{name} · mean ● / p95 ▲"))
                .legend(move |(x, y)| Circle::new((x + 8, y), 4, color.filled()));
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::LowerLeft)
            .background_style(WHITE.mix(0.94))
            .border_style(BLACK.mix(0.2))
            .draw()?;
    }
    root.present()?;
    Ok(())
}

/// Paired changes can be orders of magnitude smaller than the absolute error.
fn plot_changes(data: &Value, path: &Path) -> Result<()> {
    use plotters::prelude::*;
    let root = SVGBackend::new(path, (1030, 660)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root
        .titled(
            "Mean change from one relaxed landing interval",
            ("sans-serif", 23),
        )?
        .split_evenly((2, 1));
    let values = data["held_out"]["policies"]
        .as_array()
        .ok_or("post-contact treatments")?;
    let labels = ["1", "2", "4", "8", "26", "weight"];
    for (area, w) in areas.iter().zip(0..2) {
        if values[0]["windows"][w]["joined"] == 0 {
            area.draw(&Text::new(
                format!("{} ms · No matched held-out windows", [5, 13][w]),
                (35, 66),
                ("sans-serif", 20).into_font(),
            ))?;
            continue;
        }
        let mut bounds = [0f64; 2];
        let mut series = vec![];
        for g in 0..2 {
            let points = values
                .iter()
                .enumerate()
                .map(|(k, p)| {
                    let y = 1000.
                        * p["windows"][w]["scores"][g]["change_vs_onset_mm"]["mean"]
                            .as_f64()
                            .ok_or("no held paired windows")?;
                    bounds[0] = bounds[0].min(y);
                    bounds[1] = bounds[1].max(y);
                    Ok((k as f64, y))
                })
                .collect::<Result<Vec<_>>>()?;
            series.push(points);
        }
        let pad = ((bounds[1] - bounds[0]) * 0.18).max(0.02);
        let mut c = ChartBuilder::on(area)
            .margin(13)
            .caption(
                if w == 0 {
                    "5 ms after the incoming cut"
                } else {
                    "13 ms after the incoming cut"
                },
                ("sans-serif", 18),
            )
            .x_label_area_size(43)
            .y_label_area_size(82)
            .build_cartesian_2d(-0.4f64..5.4f64, bounds[0] - pad..bounds[1] + pad)?;
        c.configure_mesh()
            .max_light_lines(0)
            .x_labels(6)
            .x_label_formatter(&|x| {
                let i = x.round() as isize;
                if (x - i as f64).abs() < 1e-7 && (0..6).contains(&i) {
                    labels[i as usize].into()
                } else {
                    String::new()
                }
            })
            .x_desc("First intervals with eight passes, or causal weight release")
            .y_desc("Mean COM error change (µm)")
            .draw()?;
        c.draw_series([PathElement::new(
            [(-0.4, 0.), (5.4, 0.)],
            BLACK.mix(0.5).stroke_width(1),
        )])?;
        for (points, (name, color)) in series.into_iter().zip([
            ("4,096 calls, eight passes", RGBColor(18, 121, 112)),
            ("4,096 calls, one pass", RGBColor(187, 83, 26)),
        ]) {
            c.draw_series(LineSeries::new(points.clone(), color.stroke_width(2)))?
                .label(name)
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 20, y)], color.stroke_width(2))
                });
            c.draw_series(
                points
                    .into_iter()
                    .map(|p| Circle::new(p, 4, color.filled())),
            )?;
        }
        c.configure_series_labels()
            .position(SeriesLabelPosition::UpperLeft)
            .background_style(WHITE.mix(0.92))
            .draw()?;
    }
    root.present()?;
    Ok(())
}
