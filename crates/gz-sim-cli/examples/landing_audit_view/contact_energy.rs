// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Paired recordings of corner loads, lift work and the change in mechanical energy.
use super::*;
use clap::ValueEnum;
#[path = "contact_energy_figures.rs"]
mod plots;
pub(super) use plots::draw as figures;

#[derive(Copy, Clone, ValueEnum)]
pub(super) enum Comparison {
    /// Always-dynamic 125 µs versus 500 µs on the same incoming state.
    Rate,
    /// Native reduced-coordinate articulation versus equivalent free box.
    Rigid,
    /// Free box with measured tilt and spin versus level, spin-free control.
    Posture,
    /// Switched and always-dynamic articulation at 250 µs, including the secondary impact.
    Rebound,
    /// Prospective two-tick iteration policy versus the fixed ten-tick 4× solve.
    Policy,
}
impl Comparison {
    fn lanes(self) -> [(&'static str, &'static str); 2] {
        let fine = ("energy/traces/reference-native-4x.json", "articulated_fine");
        let rigid = ("rigid/traces/reference-matched-4x.json", "matched_rigid");
        match self {
            Self::Rate => [
                fine,
                (
                    "energy/traces/reference-native-1x.json",
                    "articulated_coarse",
                ),
            ],
            Self::Rigid => [fine, rigid],
            Self::Posture => [
                rigid,
                (
                    "rigid/traces/reference-level_no_spin-4x.json",
                    "level_without_spin",
                ),
            ],
            Self::Policy => [
                ("controls/gate_2_iter16.json", "prospective_two_ticks"),
                ("energy/traces/reference-native-4x.json", "fixed_ten_ticks"),
            ],
            Self::Rebound => [
                ("energy/traces/switched-native-2x.json", "switched_history"),
                ("energy/traces/reference-native-2x.json", "dynamic_history"),
            ],
        }
    }
}

#[derive(Deserialize)]
#[serde(untagged)]
enum Balance {
    Articulated {
        lift_work_j: f64,
        mechanical_change_j: f64,
    },
    Rigid {
        support_work_j: f64,
        kinetic_translation_j: f64,
        kinetic_rotation_j: f64,
        potential_j: f64,
    },
}
impl Balance {
    fn work(&self) -> f64 {
        match self {
            Self::Articulated { lift_work_j, .. } => *lift_work_j,
            Self::Rigid { support_work_j, .. } => *support_work_j,
        }
    }
    fn mechanical(&self) -> f64 {
        match self {
            Self::Articulated {
                mechanical_change_j,
                ..
            } => *mechanical_change_j,
            Self::Rigid {
                kinetic_translation_j,
                kinetic_rotation_j,
                potential_j,
                ..
            } => kinetic_translation_j + kinetic_rotation_j + potential_j,
        }
    }
}

#[derive(Deserialize)]
struct Sample {
    time_us: u64,
    dt_us: u64,
    foot: Body,
    upper: Body,
    pad: Body,
    normal_impulse_ns: f64,
    corner_ns: [f64; 4],
    #[serde(flatten)]
    balance: Balance,
}
#[derive(Deserialize)]
struct Recording {
    geometry: Geometry,
    frames: Vec<Sample>,
}
struct Reading<'a> {
    sample: &'a Sample,
    corner: [f64; 4],
    work: f64,
    change: f64,
}
fn common<'a>(record: &'a Recording, times: &[u64]) -> Result<Vec<Reading<'a>>> {
    let first = record.frames.first().ok_or("empty impact recording")?;
    if first.dt_us != 0 || times.first() != Some(&first.time_us) {
        return Err("impact cuts disagree".into());
    }
    let mut index = 0;
    let mut rows = Vec::with_capacity(times.len());
    for &time in times {
        let mut corner = [0.; 4];
        let mut matched = None;
        while let Some(s) = record.frames.get(index).filter(|s| s.time_us <= time) {
            if index > 0
                && (s.dt_us == 0
                    || record.frames[index - 1].time_us.checked_add(s.dt_us) != Some(s.time_us))
            {
                return Err("discontinuous native clock".into());
            }
            if s.corner_ns.iter().any(|x| !x.is_finite() || *x < 0.)
                || !s.normal_impulse_ns.is_finite()
                || (s.corner_ns.iter().sum::<f64>() - s.normal_impulse_ns).abs() > 1e-9
            {
                return Err("native point sum differs from recorded pair".into());
            }
            for (sum, amount) in corner.iter_mut().zip(s.corner_ns) {
                *sum += amount;
            }
            matched = Some(s);
            index += 1;
        }
        let sample = matched
            .filter(|s| s.time_us == time)
            .ok_or("missing common boundary")?;
        let work = sample.balance.work() - first.balance.work();
        let change = sample.balance.mechanical() - first.balance.mechanical();
        if !work.is_finite() || !change.is_finite() {
            return Err("nonfinite impact energy".into());
        }
        rows.push(Reading {
            sample,
            corner,
            work,
            change,
        });
    }
    if index != record.frames.len() {
        return Err("different recording duration".into());
    }
    Ok(rows)
}

/// Cross-check the viewer’s binned impulses with the original policy experiment.
fn policy_activity(
    dir: &Path,
    rows: &[Vec<Reading<'_>>; 2],
    clocks: &[u64],
    marks: &mut BTreeMap<String, u64>,
) -> Result<[Vec<u64>; 2]> {
    let cut = clocks[0];
    let summary = read::<Value>(&dir.join("controls/summary.json"))?;
    let runs = summary["runs"].as_array().ok_or("missing policy results")?;
    let mut active = [Vec::new(), Vec::new()];
    for (k, (r, policy)) in rows.iter().zip(["gate_2_iter16", "fixed_10"]).enumerate() {
        let matched = runs
            .iter()
            .find(|s| {
                s["cycle"] == 3
                    && s["history"] == "reference"
                    && s["bottom_dwell_shift_us"] == 0
                    && s["policy"] == policy
            })
            .ok_or("missing nominal policy result")?;
        for (offset, key) in [(500, "first_outer_ns"), (5_000, "window_ns")] {
            let point = clocks
                .binary_search(&(cut + offset))
                .map_err(|_| "missing policy checkpoint")?;
            let actual: f64 = r[..=point].iter().flat_map(|v| v.corner).sum();
            let expected = matched[key].as_f64().ok_or("missing policy impulse")?;
            if (actual - expected).abs() > 1e-8 {
                return Err(format!("{policy} {key}: trace {actual}, summary {expected}").into());
            }
        }
        active[k] = serde_json::from_value(matched["selected_ticks"].clone())?;
    }
    let last = active[0].last().copied().ok_or("no selected solve")?;
    for (name, tick) in [("last_selected", last), ("resumed_default", last + 1)] {
        let us = u64::checked_mul(tick, 500).ok_or("invalid selection tick")?;
        if clocks.binary_search(&us).is_err() {
            return Err("missing solve boundary".into());
        }
        marks.insert(
            name.to_owned(),
            us.checked_sub(cut).ok_or("wrong policy cut")?,
        );
    }
    Ok(active)
}

pub(super) fn export(dir: &Path, comparison: Comparison, out: &Path) -> Result<()> {
    let lanes = comparison.lanes();
    let [a, b] = lanes.map(|(file, _)| read::<Recording>(&dir.join(file)));
    let (a, b) = (a?, b?);
    let clocks: Vec<_> = b
        .frames
        .iter()
        .map(|s| s.time_us)
        .filter(|t| a.frames.binary_search_by_key(t, |s| s.time_us).is_ok())
        .collect();
    let rows = [common(&a, &clocks)?, common(&b, &clocks)?];
    let cut = *clocks.first().ok_or("empty clock")?;
    let conf = &a.geometry;
    if conf.foot_half_extents != b.geometry.foot_half_extents
        || conf.pad_half_extents != b.geometry.pad_half_extents
        || conf.upper_half_extents != b.geometry.upper_half_extents
    {
        return Err("geometry mismatch".into());
    }
    let all = rows.iter().flatten();
    let full = all.clone().flat_map(|r| r.corner).fold(0f64, f64::max);
    let lo = all
        .clone()
        .flat_map(|r| [r.work, r.change])
        .fold(0f64, f64::min);
    let hi = all
        .clone()
        .flat_map(|r| [r.work, r.change])
        .fold(0f64, f64::max);
    if full <= 0. || hi <= lo {
        return Err("empty contact or energy ledger".into());
    }
    let spacing = 2.5 * conf.pad_half_extents[0];
    let [hx, hy, _] = conf.foot_half_extents;
    let sole_z = all
        .map(|r| r.sample.pad.position[2] - conf.pad_half_extents[2])
        .fold(f64::INFINITY, f64::min)
        - 0.52 * conf.pad_half_extents[0];
    let mut marks = BTreeMap::from([("first_outer".to_owned(), 500u64), ("five_ms".into(), 5000)]);
    let policies = matches!(comparison, Comparison::Policy)
        .then(|| policy_activity(dir, &rows, &clocks, &mut marks))
        .transpose()?;
    if matches!(comparison, Comparison::Rebound) {
        let reload = read::<Value>(&dir.join("energy/summary.json"))?;
        let events = reload["runs"]
            .as_array()
            .ok_or("missing runs")?
            .iter()
            .find(|r| r["receiver"] == "reference" && r["motion"] == "native" && r["factor"] == 2)
            .and_then(|r| r["recontacts"].as_array())
            .ok_or("missing reload data")?;
        for (i, e) in events.iter().enumerate() {
            let t = e["time_us"].as_u64().ok_or("missing reload clock")?;
            if clocks.binary_search(&t).is_err() {
                return Err("missing reload frame".into());
            }
            marks.insert(format!("reload_{}", i + 1), t - cut);
        }
    }
    let mut frames = Vec::new();
    for (i, time) in clocks.iter().copied().enumerate() {
        let mut f = Frame::default();
        for (k, (_, name)) in lanes.iter().enumerate() {
            let r = &rows[k][i];
            let p = r.sample;
            let x = (k as f64 - 0.5) * spacing;
            LaneDisplay {
                label: name,
                color: LANE[k],
                offset: x,
                sole_z,
                full,
            }
            .add(&mut f, conf, [&p.foot, &p.upper, &p.pad], r.corner)?;
            if let Some(policies) = &policies {
                let selected = policies[k].contains(&time.div_ceil(500));
                let h = sole_z - 2.65 * hy;
                f.edge(
                    &format!("{name}_solver_activity_scale"),
                    DVec3::new(0., x - hx, h),
                    DVec3::new(0., x + hx, h),
                    2. * hx,
                    0.004,
                    IDLE,
                );
                f.add(
                    &format!("{name}_solver_activity"),
                    DVec3::new(0.023, if selected { x + hx } else { x - hx }, h),
                    DQuat::IDENTITY,
                    cuboid([0.013; 3]),
                    PAD,
                );
            }
            for (n, (quantity, value, color)) in [
                ("lift_work", r.work, PAD),
                ("mechanical_change", r.change, LANE[k]),
            ]
            .into_iter()
            .enumerate()
            {
                let h = sole_z - (1.61 + 0.52 * n as f64) * hy;
                let gauge_x = |v: f64| x + hx * (2. * (v - lo) / (hi - lo) - 1.);
                f.edge(
                    &format!("{name}_{quantity}_scale"),
                    DVec3::new(0., x - hx, h),
                    DVec3::new(0., x + hx, h),
                    2. * hx,
                    0.004,
                    IDLE,
                );
                f.add(
                    &format!("{name}_{quantity}_zero"),
                    DVec3::new(0.012, gauge_x(0.), h),
                    DQuat::IDENTITY,
                    cuboid([0.0025, 0.0025, 0.010]),
                    UPPER,
                );
                f.add(
                    &format!("{name}_{quantity}"),
                    DVec3::new(0.023, gauge_x(value), h),
                    DQuat::IDENTITY,
                    cuboid([0.010; 3]),
                    color,
                );
            }
        }
        frames.push(f.finish(Duration::from_micros(time - cut), i + 1));
    }
    fs::create_dir_all(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
    let mut index = json!({
            "kind":"contact-energy-v1","dt_ns":1000,"origin_us":cut,"ticks":clocks.iter().map(|t|t-cut).collect::<Vec<_>>(),
            "captures":marks,"spacing":spacing,"sole_z":sole_z,"full_corner_ns":full,"energy_min_j":lo,"energy_max_j":hi,
            "lanes":lanes.map(|(file,label)|json!({"file":file,"label":label}))
    });
    if policies.is_some() {
        index["activity_source"] = json!("controls/summary.json");
        index["lanes"][0]["policy"] = json!("gate_2_iter16");
        index["lanes"][1]["policy"] = json!("fixed_10");
    }
    fs::write(out.join("frames.json"), serde_json::to_vec_pretty(&index)?)?;
    Ok(())
}
