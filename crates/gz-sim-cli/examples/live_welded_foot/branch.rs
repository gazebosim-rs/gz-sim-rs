// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::Value;
const START: usize = 168;

fn from_donor(welded: bool, factor: u32, donor: &Sample) -> Result<String> {
    if donor.tick != START
        || donor.foot.q != [0., 0., 0., 1.]
        || donor.upper.q != donor.foot.q
        || donor.angular_about_com_nms != [0.; 3]
        || donor.normal_impulse_ns != 0.
        || donor.upper_normal_impulse_ns != 0.
    {
        return Err("expected the level, nonspinning 84 ms pre-impact state".into());
    }
    Ok(scene_at(welded, true, factor, Some(donor)))
}
fn read<T: serde::de::DeserializeOwned>(p: &Path) -> Result<T> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn write<T: Serialize>(p: &Path, data: &T) -> Result<()> {
    Ok(fs::write(p, serde_json::to_vec(data)?)?)
}
fn check_initial(a: &Sample, b: &Sample) -> Result<()> {
    let largest = [
        delta(a.foot.p, b.foot.p),
        delta(a.upper.p, b.upper.p),
        delta(a.com, b.com),
        delta(a.p_kg_m_s, b.p_kg_m_s),
        delta(a.angular_about_com_nms, b.angular_about_com_nms),
        orientation(a.foot.q, b.foot.q),
        orientation(a.upper.q, b.upper.q),
    ]
    .into_iter()
    .fold(0., f64::max);
    if largest > 1e-12 || a.tick != b.tick {
        return Err(
            format!("failed to restore donor on its incoming public clock: {largest}").into(),
        );
    }
    Ok(())
}
#[derive(Serialize, Default)]
struct Difference {
    first_linear_ns: f64,
    first_angular_nms: f64,
    first_normal_ns: f64,
    max_com_m: f64,
    max_link_m: f64,
    max_angle_rad: f64,
    max_linear_impulse_ns: f64,
    max_normal_impulse_ns: f64,
    max_angular_impulse_nms: f64,
    end_com_m: f64,
    end_angular_momentum_nms: f64,
    end_normal_impulse_ns: f64,
}
fn first_contact(run: &Trial) -> Result<usize> {
    run.frames
        .iter()
        .position(|f| f.normal_impulse_ns > 0.)
        .filter(|n| *n > 0)
        .ok_or_else(|| "missing first loaded interval".into())
}
fn jump(run: &Trial, index: usize) -> (DVec3, DVec3) {
    momentum_jump(&run.frames[index - 1], &run.frames[index])
}
fn first_sample(run: &Trial) -> Result<Value> {
    let i = first_contact(run)?;
    let f = &run.frames[i];
    let (p, h) = jump(run, i);
    Ok(
        json!({"tick":f.tick,"linear_impulse_world_ns":p.to_array(),"angular_impulse_about_com_world_nms":h.to_array(),"normal_impulse_ns":f.normal_impulse_ns,"after_solve":f}),
    )
}
fn differences(w: &Trial, s: &Trial) -> Result<Difference> {
    if w.frames.len() != s.frames.len() || w.frames.len() < 2 {
        return Err("incomplete branch".into());
    }
    check_initial(&w.frames[0], &s.frames[0])?;
    let (wi, si) = (first_contact(w)?, first_contact(s)?);
    if wi != si || w.frames[wi].tick != START + 1 {
        return Err("impact clock mismatch".into());
    }
    let ((wp, wh), (sp, sh)) = (jump(w, wi), jump(s, si));
    let mut d = Difference {
        first_linear_ns: wp.distance(sp),
        first_angular_nms: wh.distance(sh),
        first_normal_ns: (w.frames[wi].normal_impulse_ns - s.frames[si].normal_impulse_ns).abs(),
        ..Difference::default()
    };
    let mut normal = 0.;
    for (i, (a, b)) in w.frames.iter().zip(&s.frames).enumerate() {
        if a.tick != b.tick || a.upper_normal_impulse_ns != 0. || b.upper_normal_impulse_ns != 0. {
            return Err("clock or contact ownership mismatch".into());
        }
        d.end_com_m = delta(a.com, b.com);
        d.max_com_m = d.max_com_m.max(d.end_com_m);
        d.max_link_m = d
            .max_link_m
            .max(delta(a.foot.p, b.foot.p))
            .max(delta(a.upper.p, b.upper.p));
        d.max_angle_rad = d
            .max_angle_rad
            .max(orientation(a.foot.q, b.foot.q))
            .max(orientation(a.upper.q, b.upper.q));
        d.end_angular_momentum_nms = delta(a.angular_about_com_nms, b.angular_about_com_nms);
        let dj = a.normal_impulse_ns - b.normal_impulse_ns;
        normal += dj;
        d.max_normal_impulse_ns = d.max_normal_impulse_ns.max(dj.abs());
        if i > 0 {
            let (ap, ah) = jump(w, i);
            let (bp, bh) = jump(s, i);
            d.max_linear_impulse_ns = d.max_linear_impulse_ns.max(ap.distance(bp));
            d.max_angular_impulse_nms = d.max_angular_impulse_nms.max(ah.distance(bh));
        }
    }
    d.end_normal_impulse_ns = normal.abs();
    Ok(d)
}
fn archived_before(g: &Value) -> Result<Difference> {
    let b = &g["first_divergent_contact"];
    let vector = |part: &str, key: &str| -> Result<DVec3> {
        Ok(DVec3::from_array(serde_json::from_value(
            b[part][key].clone(),
        )?))
    };
    let linear = "contact_linear_impulse_world_ns";
    let angular = "contact_angular_impulse_about_com_world_nms";
    Ok(Difference {
        first_linear_ns: vector("welded", linear)?.distance(vector("consolidated", linear)?),
        first_angular_nms: vector("welded", angular)?.distance(vector("consolidated", angular)?),
        max_com_m: g["max_com_error_m"]
            .as_f64()
            .ok_or("missing archived continuous COM error")?,
        ..Difference::default()
    })
}
fn check_reduction(before: &Difference, after: &Difference, kind: &str, us: f64) -> Result<()> {
    let checks: [(&str, f64, f64, f64); 3] = [
        (
            "first linear impulse",
            before.first_linear_ns,
            after.first_linear_ns,
            0.002,
        ),
        (
            "first angular impulse",
            before.first_angular_nms,
            after.first_angular_nms,
            0.0001,
        ),
        ("maximum COM", before.max_com_m, after.max_com_m, 0.000020),
    ];
    for (metric, old, new, absolute) in checks {
        let limit = absolute.min(0.05 * old);
        if !new.is_finite() || new > limit {
            return Err(format!(
                "{kind} at {us} µs: {metric} {new:.8e} exceeds {limit:.8e} (before {old:.8e})"
            )
            .into());
        }
    }
    if after.max_link_m > 0.000025
        || after.max_angle_rad > 0.00025
        || after.max_normal_impulse_ns > 0.002
        || after.max_linear_impulse_ns > 0.003
        || after.max_angular_impulse_nms > 0.0002
        || after.end_normal_impulse_ns > 0.0002
    {
        return Err(format!(
            "{kind} at {us} µs failed trajectory/impulse bounds: {}",
            serde_json::to_string(after)?
        )
        .into());
    }
    Ok(())
}
pub(super) fn campaign(archive: &Path, prior: Option<&Path>, out: &Path) -> Result<bool> {
    let grids: Vec<Value> = read(&archive.join("ground_convergence.json"))?;
    if grids.len() != 4 {
        return Err("expected all four archived timestep checks".into());
    }
    let mut reports = Vec::new();
    let mut qualified = true;
    for (factor, g) in [1, 2, 4, 8].into_iter().zip(grids) {
        let us = 500. / f64::from(factor);
        if g["native_step_us"].as_f64() != Some(us) {
            return Err("wrong donor grid".into());
        }
        let pair = &g["first_divergent_contact"];
        let donor: Sample = serde_json::from_value(pair["welded"]["incoming"].clone())?;
        let single: Sample = serde_json::from_value(pair["consolidated"]["incoming"].clone())?;
        check_initial(&donor, &single)?;
        let mut runs = vec![];
        for welded in [true, false] {
            let source = from_donor(welded, factor, &donor)?;
            let run = run_from(welded, true, &source, factor, START)?;
            check_initial(&run.frames[0], &donor)?;
            if run.frames.len() != END - START + 1
                || !run.frames.iter().any(|s| s.normal_impulse_ns > 0.)
            {
                return Err("incomplete landing branch".into());
            }
            runs.push(run);
        }
        let file = format!("branches-{factor}.json");
        write(&out.join(&file), &runs)?;
        if let Some(original_dir) = prior {
            let previous: Vec<Trial> = read(&original_dir.join(&file))?;
            if previous.len() != 2 {
                return Err("missing both pre-change cold branches".into());
            }
            check_initial(&previous[0].frames[0], &runs[0].frames[0])?;
            let old = differences(&previous[0], &previous[1])?;
            if old.first_linear_ns < 0.005
                || old.first_angular_nms < 0.0001
                || old.max_com_m < 0.00005
            {
                return Err("pre-change inputs do not reproduce the archived divergence".into());
            }
            let new = differences(&runs[0], &runs[1])?;
            let mut continuous = vec![];
            for welded in [true, false] {
                let mut trial = run(welded, true, &scene(welded, true, factor), factor)?;
                trial.frames.drain(..START);
                check_initial(&trial.frames[0], &donor)?;
                continuous.push(trial);
            }
            let continuous_new = differences(&continuous[0], &continuous[1])?;
            let continuous_old = archived_before(&g)?;
            let failures = [
                check_reduction(&old, &new, "matched restart", us),
                check_reduction(&continuous_old, &continuous_new, "uninterrupted", us),
            ]
            .into_iter()
            .filter_map(Result::err)
            .map(|e| e.to_string())
            .collect::<Vec<_>>();
            qualified &= failures.is_empty();
            for error in &failures {
                eprintln!("{error}");
            }
            let old_single = &previous[1];
            let new_single = &runs[1];
            let control = differences(old_single, new_single)?;
            if control.max_linear_impulse_ns > 1e-9
                || control.max_com_m > 1e-12
                || control.max_angular_impulse_nms > 1e-10
            {
                return Err("single-body restart changed across solver versions".into());
            }
            reports.push(json!({"qualified":failures.is_empty(),"failures":failures,"native_step_us":us,"matched_physical_clocks":runs[0].frames.len(),
              "incoming":donor,"before_cold":old,"after_cold":new,"old_single_vs_updated_single":control,
              "before_continuous_first_linear_error_ns":continuous_old.first_linear_ns,"before_continuous_first_angular_error_nms":continuous_old.first_angular_nms,
              "before_continuous_max_com_error_m":continuous_old.max_com_m,"after_continuous":continuous_new,
              "first_contact":{"old_weld":first_sample(&previous[0])?,"new_weld":first_sample(&runs[0])?,"single":first_sample(&runs[1])?,
                "continuous_weld":first_sample(&continuous[0])?,"continuous_single":first_sample(&continuous[1])?},
              "single_cold_vs_continuous":differences(&runs[1],&continuous[1])?}));
            if factor == 1 {
                super::branch_view::write(out, &previous[0], &runs[0], &runs[1])?;
            }
        }
    }
    if prior.is_some() {
        fs::write(
            out.join("qualification.json"),
            serde_json::to_vec_pretty(&reports)?,
        )?;
    }
    Ok(qualified)
}
