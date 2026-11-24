// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Qualify a recorded force-exchange rate by checking both individual jaw loads and sheet motion.
use super::*;
use serde::Deserialize;
use std::collections::BTreeMap;

const POSE_TOLERANCE_M: f64 = 0.0001;
const LOAD_TOLERANCE_WEIGHT: f64 = 0.01;

#[derive(Deserialize)]
struct Frame {
    time_ns: u64,
    positions: Vec<[f64; 3]>,
}
#[derive(Deserialize)]
struct Clock {
    frames: usize,
    steps: u64,
    step_ns: u64,
    simulated_ns: u64,
}
#[derive(Deserialize)]
struct Metrics {
    mean_hold_jaw_up_n: [f64; 2],
    weight_n: f64,
    step_p95_ns: u64,
}
#[derive(Deserialize)]
struct Trial {
    completed: bool,
    #[serde(default)]
    meets_grasp_tolerances: bool,
    actuator_updates: Option<usize>,
    metrics: Option<Metrics>,
    refused_at_tick: Option<usize>,
}
#[derive(Deserialize)]
struct Recording {
    actuator_period_ns: u64,
    native_step_ns: u64,
    substeps: u64,
    jaw_density_scale: f64,
    jaw_damping_ns_per_m: f64,
    pad_compliance_m_per_n: f64,
    #[serde(default)]
    pad_damping_ns_per_m: f64,
    #[serde(default)]
    panel_balance: bool,
    #[serde(default)]
    bend_damping_nms_per_rad: f64,
    trial: Trial,
    #[serde(skip)]
    frames: Vec<Frame>,
}
impl Recording {
    fn read(path: &Path) -> Result<Self> {
        let mut result: Self = serde_json::from_slice(&fs::read(path.join("jaw-contact.json"))?)?;
        if result.trial.completed {
            result.frames = serde_json::from_slice(&fs::read(path.join("particle-frames.json"))?)?;
            let clock: Clock = serde_json::from_slice(&fs::read(path.join("recording.json"))?)?;
            if clock.step_ns != result.native_step_ns
                || clock.steps.checked_mul(clock.step_ns) != Some(3_400_000_000)
                || clock.simulated_ns != 3_400_000_000
                || clock.frames != result.frames.len()
                || result.trial.actuator_updates != Some(3400)
                || result.trial.metrics.is_none()
                || result.frames.len() != 137
                || result.frames.iter().enumerate().any(|(i, f)| {
                    f.time_ns != i as u64 * 25_000_000
                        || f.positions.len() != arm_sheet::COUNT
                        || f.positions.iter().flatten().any(|x| !x.is_finite())
                })
            {
                return Err("incomplete sheet positions or actuator clock".into());
            }
        } else if result.trial.refused_at_tick.is_none() || result.trial.metrics.is_some() {
            return Err("missing refusal tick".into());
        }
        Ok(result)
    }
}

/// The two finest records must agree before either can qualify coarser runs.
pub(crate) fn compare(paths: &[PathBuf]) -> Result<Value> {
    let mut records = BTreeMap::new();
    let mut setup = None;
    for path in paths {
        let r = Recording::read(path)?;
        let settings = (
            r.jaw_density_scale,
            r.jaw_damping_ns_per_m,
            r.pad_compliance_m_per_n,
            r.pad_damping_ns_per_m,
            r.panel_balance,
            r.bend_damping_nms_per_rad,
        );
        if setup.replace(settings).is_some_and(|s| s != settings)
            || r.actuator_period_ns != 1_000_000
            || r.native_step_ns == 0
            || !r.actuator_period_ns.is_multiple_of(r.native_step_ns)
        {
            return Err("incompatible gripper configuration or physics clock".into());
        }
        let rate = r.actuator_period_ns / r.native_step_ns;
        let case = Case {
            substeps: usize::try_from(r.substeps)?,
            ..Default::default()
        };
        let expected = super::timed_scene(
            r.jaw_density_scale,
            r.pad_compliance_m_per_n,
            case,
            usize::try_from(rate)?,
            r.panel_balance,
            r.bend_damping_nms_per_rad,
        )?;
        let expected = super::damped_pads(expected, r.pad_damping_ns_per_m)?;
        if fs::read_to_string(path.join("world.sdf"))? != expected {
            return Err(
                "recording SDF differs from the expected gripper, material or clock".into(),
            );
        }
        if records.insert(rate, r).is_some() {
            return Err("duplicate exchange rate".into());
        }
    }
    if records.len() < 3 {
        return Err("three distinct rates are required".into());
    }
    let mut fine = records.iter().rev();
    let (&reference_rate, reference) = fine.next().unwrap();
    let (&verification_rate, verification) = fine.next().unwrap();
    let reference_metrics = reference
        .trial
        .metrics
        .as_ref()
        .filter(|_| reference.trial.meets_grasp_tolerances)
        .ok_or("the finest recording has no qualified grasp")?;
    let mut entries = vec![];
    let mut acceptable = vec![];
    for (&rate, r) in &records {
        let Some(m) = &r.trial.metrics else {
            entries
                .push(json!({"exchanges_per_ms":rate,"refused_at_tick":r.trial.refused_at_tick}));
            continue;
        };
        if r.frames[0].positions != reference.frames[0].positions {
            return Err("recordings have different initial material positions".into());
        }
        if m.weight_n != reference_metrics.weight_n || m.weight_n <= 0. {
            return Err("incompatible sheet weights".into());
        }
        let mut maximum = 0_f64;
        let mut worst_sample = None;
        let mut first_outside = None;
        for (a, b) in r.frames.iter().zip(&reference.frames) {
            for (particle, (x, y)) in a.positions.iter().zip(&b.positions).enumerate() {
                let error = DVec3::from(*x).distance((*y).into());
                if error > maximum {
                    maximum = error;
                    worst_sample = Some((a.time_ns, particle));
                }
                if error > POSE_TOLERANCE_M && first_outside.is_none() {
                    first_outside = Some((a.time_ns, particle));
                }
            }
        }
        let load_error = std::array::from_fn::<_, 2, _>(|i| {
            (m.mean_hold_jaw_up_n[i] - reference_metrics.mean_hold_jaw_up_n[i]).abs() / m.weight_n
        });
        let pass = r.trial.meets_grasp_tolerances
            && maximum <= POSE_TOLERANCE_M
            && load_error.iter().all(|&e| e <= LOAD_TOLERANCE_WEIGHT);
        if pass {
            acceptable.push(rate);
        }
        let location =
            |(time_ns, particle): (u64, usize)| json!({"time_ns":time_ns,"particle":particle});
        entries.push(json!({
            "exchanges_per_ms": rate, "soft_steps_per_exchange": r.substeps,
            "passes_against_finest": pass, "max_particle_error_m": maximum,
            "first_particle_outside_tolerance": first_outside.map(location),
            "max_particle_error_location": worst_sample.map(location),
            "jaw_load_error_fraction_of_weight": load_error,
            "mean_hold_jaw_load_n": m.mean_hold_jaw_up_n,
            "p95_physics_cost_per_actuator_period_ns": m.step_p95_ns
        }));
    }
    let verified = verification.trial.completed && acceptable.contains(&verification_rate);
    // Require every measured finer rate to pass as well. One lucky coarse match
    // cannot override a disagreement at a higher resolution.
    let selected = verified
        .then(|| {
            records
                .keys()
                .rev()
                .take_while(|rate| acceptable.contains(rate))
                .last()
                .copied()
        })
        .flatten();
    Ok(
        json!({"reference_exchanges_per_ms":reference_rate,"verification_exchanges_per_ms":verification_rate,"reference_checked":verified,"selected_exchanges_per_ms":selected,"max_particle_tolerance_m":POSE_TOLERANCE_M,"per_jaw_tolerance_fraction_of_sheet_weight":LOAD_TOLERANCE_WEIGHT,"trials":entries}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(root: &Path, rates: &[u64]) -> Result<Vec<PathBuf>> {
        let mut paths = vec![];
        for rate in rates {
            let dir = root.join(rate.to_string());
            fs::create_dir_all(&dir)?;
            fs::write(
                dir.join("world.sdf"),
                timed_scene(1., 0.001, Case::default(), *rate as usize, false, 0.)?,
            )?;
            let step = 1_000_000 / rate;
            write_json(
                &dir.join("recording.json"),
                &json!({
                    "steps":3400*rate,"simulated_ns":3_400_000_000_u64,"frames":137,"step_ns":step
                }),
            )?;
            let frames: Vec<_> = (0..137_u64)
                .map(|i| {
                    json!({
                        "time_ns":i*25_000_000,"positions":vec![[0.;3];arm_sheet::COUNT]
                    })
                })
                .collect();
            write_json(&dir.join("particle-frames.json"), &json!(frames))?;
            write_json(
                &dir.join("jaw-contact.json"),
                &json!({
                    "actuator_period_ns":1_000_000,"native_step_ns":step,
                    "substeps":Case::default().substeps,"jaw_density_scale":1.,
                    "jaw_damping_ns_per_m":12.,"pad_compliance_m_per_n":0.001,
                    "trial":{"completed":true,"actuator_updates":3400,"meets_grasp_tolerances":true,
                        "metrics":{"mean_hold_jaw_up_n":[0.5,0.5],"weight_n":1.,"step_p95_ns":96000}}
                }),
            )?;
            paths.push(dir);
        }
        Ok(paths)
    }

    fn alter(dir: &Path, file: &str, edit: impl FnOnce(&mut Value)) -> Result<()> {
        let path = dir.join(file);
        let mut value = serde_json::from_slice(&fs::read(&path)?)?;
        edit(&mut value);
        write_json(&path, &value)
    }

    #[test]
    fn selection_requires_individual_loads_and_an_unbroken_sequence_of_finer_matches() -> Result<()>
    {
        let root = tempfile::tempdir()?;
        let records = fixture(root.path(), &[1, 2, 4, 8])?;
        assert_eq!(compare(&records)?["selected_exchanges_per_ms"], 1);
        alter(&records[1], "jaw-contact.json", |v| {
            v["trial"]["metrics"]["mean_hold_jaw_up_n"] = json!([0.7, 0.3]);
        })?;
        let report = compare(&records)?;
        assert_eq!(report["selected_exchanges_per_ms"], 4);
        assert_eq!(report["trials"][1]["max_particle_error_m"], 0.);
        alter(&records[2], "particle-frames.json", |v| {
            v[70]["positions"][1][2] = json!(0.00011);
        })?;
        let report = compare(&records)?;
        for key in [
            "first_particle_outside_tolerance",
            "max_particle_error_location",
        ] {
            assert_eq!(
                report["trials"][2][key],
                json!({"time_ns":1_750_000_000_u64,"particle":1})
            );
            assert!(report["trials"][3][key].is_null());
        }
        assert_eq!(report["reference_checked"], false);
        assert!(report["selected_exchanges_per_ms"].is_null());
        alter(&records[2], "jaw-contact.json", |v| {
            v["trial"] = json!({"completed":false,"refused_at_tick":71});
        })?;
        assert_eq!(compare(&records)?["trials"][2]["refused_at_tick"], 71);
        Ok(())
    }

    #[test]
    fn reject_incompatible_recordings_before_selecting_a_rate() -> Result<()> {
        let root = tempfile::tempdir()?;
        let rates = [1, 2, 4];
        let records = fixture(root.path(), &rates)?;
        assert!(compare(&records[..2]).is_err());
        assert!(compare(&[records[0].clone(), records[0].clone()]).is_err());
        for (file, pointer, bad) in [
            ("jaw-contact.json", "/native_step_ns", json!(0)),
            ("jaw-contact.json", "/actuator_period_ns", json!(750000)),
            ("jaw-contact.json", "/jaw_damping_ns_per_m", json!(3.)),
            ("jaw-contact.json", "/trial/actuator_updates", json!(3401)),
            ("jaw-contact.json", "/trial/metrics", Value::Null),
            ("jaw-contact.json", "/trial/metrics/weight_n", json!(0.)),
            ("jaw-contact.json", "/trial", json!({"completed":false})),
            ("particle-frames.json", "/0/positions/0/0", json!(1.)),
            ("particle-frames.json", "/1/time_ns", json!(123)),
            ("particle-frames.json", "/2/positions", json!([])),
            ("recording.json", "/steps", json!(u64::MAX)),
            ("recording.json", "/simulated_ns", json!(123)),
            ("recording.json", "/frames", json!(136)),
        ] {
            alter(&records[1], file, |v| {
                *v.pointer_mut(pointer).unwrap() = bad
            })?;
            assert!(compare(&records).is_err(), "{file} {pointer}");
            fixture(root.path(), &rates[1..2])?;
        }
        fs::write(records[0].join("world.sdf"), "<sdf/>")?;
        assert!(compare(&records).is_err());
        fixture(root.path(), &rates[..1])?;
        alter(&records[2], "jaw-contact.json", |v| {
            v["trial"]["meets_grasp_tolerances"] = json!(false)
        })?;
        assert!(compare(&records).is_err());
        Ok(())
    }
}
