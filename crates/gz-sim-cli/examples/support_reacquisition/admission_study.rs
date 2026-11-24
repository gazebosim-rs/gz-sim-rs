// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure continuous eligibility dwell against identical fixed-clock lift commands.
use super::*;
use gz_sim_scene::fidelity_zone::{self, FidelityZone};
use std::{
    path::Path,
    time::{Duration, Instant},
};

const STEP_US: u64 = 500;
const STEPS: usize = 1771;
const DWELLS_MS: [u64; 5] = [0, 2, 8, 20, 40];

fn zone_for(lane: &mut Lane, active: bool, dwell_ms: u64) -> Result<()> {
    let zone = FidelityZone {
        center: if active { [0.; 3] } else { [10., 0., 0.] },
        promote_radius: 0.5,
        demote_radius: 1.,
    };
    fidelity_zone::configure_with_eligibility_dwell(
        &mut lane.sim,
        lane.model,
        vec![zone],
        Duration::ZERO,
        Duration::from_millis(dwell_ms),
    )?;
    Ok(())
}

fn setup(source: &str, dwell: Option<u64>) -> Result<Lane> {
    let mut lane = Lane::from_sdf_at(source, STEP_US)?;
    if let Some(ms) = dwell {
        zone_for(&mut lane, false, ms)?;
    }
    Ok(lane)
}

fn command(lane: &mut Lane, tick: usize, previous: &mut f64) -> Result<()> {
    let (_, v) = sweep::command((tick as u64 - 1) * STEP_US, 0);
    if v != *previous {
        lane.drive(v)?;
        *previous = v;
    }
    Ok(())
}

fn reference(source: &str) -> Result<Vec<Sample>> {
    let mut lane = setup(source, None)?;
    let mut samples = vec![lane.sample()?];
    let mut previous = 0.;
    for t in 1..=STEPS {
        command(&mut lane, t, &mut previous)?;
        lane.sim.step_exact(1)?;
        samples.push(lane.sample()?);
    }
    Ok(samples)
}

#[derive(Serialize)]
struct Impact {
    tick: u64,
    normal_impulse_ns: f64,
    window_normal_impulse_ns: f64,
}

fn impacts(
    frames: &[Frame],
    first: usize,
    end: usize,
    lane: fn(&Frame) -> Sample,
) -> Result<Vec<Impact>> {
    let mut airborne = false;
    let mut hits = Vec::new();
    for t in first..end {
        let s = lane(&frames[t]);
        if s.normal_impulse_ns == 0. && s.vertical_gap_m > 1e-5 {
            airborne = true;
        }
        if airborne && s.normal_impulse_ns > 0. {
            let samples = frames
                .get(t..t + WINDOW_STEPS)
                .ok_or("incomplete impact window")?;
            hits.push(Impact {
                tick: frames[t].tick,
                normal_impulse_ns: s.normal_impulse_ns,
                window_normal_impulse_ns: samples.iter().map(|f| lane(f).normal_impulse_ns).sum(),
            });
            airborne = false;
        }
    }
    Ok(hits)
}

fn measure(
    source: &str,
    fixture: &str,
    dwell_ms: u64,
    lead_us: Option<u64>,
    dynamic: &[Sample],
) -> Result<Value> {
    let mut lane = setup(source, Some(dwell_ms))?;
    let mut frames = Vec::with_capacity(dynamic.len());
    let mut previous_v = 0.;
    let mut roundtrips = 0;
    let mut first_pose_delta = None;
    for (tick, &control) in dynamic.iter().enumerate() {
        if tick > 0 {
            if let Some(lead) = lead_us {
                let now = (tick as u64 - 1) * STEP_US;
                for (i, departure) in sweep::DEPART_US.into_iter().enumerate() {
                    if now == departure - lead {
                        zone_for(&mut lane, true, dwell_ms)?;
                    }
                    if now == departure + STROKES[i].start_park() * STEP_US {
                        zone_for(&mut lane, false, dwell_ms)?;
                    }
                }
            }
            command(&mut lane, tick, &mut previous_v)?;
            lane.sim.step_exact(1)?;
        }
        let sample = lane.sample()?;
        if sample.pad != control.pad || (tick == 0 && sample != control) {
            return Err(
                "policy and reference differ in initial state or commanded lift motion".into(),
            );
        }
        if (sample.foot != control.foot || sample.upper != control.upper)
            && first_pose_delta.is_none()
        {
            first_pose_delta = Some(tick);
        }
        let prescribed = lane.tier()? == Prescribed;
        if tick > 0
            && frames.last().is_some_and(|f: &Frame| !f.prescribed)
            && support_loss_promotion::last_promotion(&lane.sim, lane.model)
                .is_some_and(|e| e.sim_time == lane.sim.elapsed())
        {
            roundtrips += 1;
        }
        frames.push(Frame {
            tick: tick as u64,
            time_s: tick as f64 * 0.0005,
            cycle: 0,
            solved_prescribed: prescribed,
            prescribed,
            switched: sample,
            reference: control,
        });
    }
    let mut cycles = Vec::new();
    let pct = |x: f64, y: f64| 100. * (x - y) / y;
    for (i, depart) in sweep::DEPART_US.into_iter().enumerate() {
        let first = (depart / STEP_US + 1) as usize;
        let stop = first + STROKES[i].start_park() as usize;
        let held = frames[..first]
            .iter()
            .rev()
            .take_while(|f| f.prescribed)
            .count();
        let next = sweep::DEPART_US
            .get(i + 1)
            .map_or(frames.len(), |x| (x / STEP_US + 1) as usize);
        let switched = impacts(&frames, first, next, |f| f.switched)?;
        let reference = impacts(&frames, first, next, |f| f.reference)?;
        let impact = switched.first().ok_or("switching foot missed the lift")?;
        let control = reference.first().ok_or("reference foot missed the lift")?;
        let readmit = frames[stop..next]
            .iter()
            .find(|f| f.prescribed)
            .map(|f| (f.tick - 1) * STEP_US);
        cycles.push(json!({
            "cycle":i+1, "held_steps_before_departure":held,
            "admit_us": (held>0).then(|| ((first-held) as u64-1)*STEP_US),
            "readmission_after_stroke_us":readmit,
            "landing_tick_offset":impact.tick as i64-control.tick as i64,
            "first_pct":pct(impact.normal_impulse_ns,control.normal_impulse_ns),
            "window_pct":pct(impact.window_normal_impulse_ns,control.window_normal_impulse_ns),
            "switched":impact,"reference":control, "switched_landings":switched, "reference_landings":reference,
        }));
    }
    Ok(
        json!({"fixture":fixture, "dwell_ms":dwell_ms, "lead_us":lead_us,
        "prescribed_steps":frames.iter().filter(|f| f.prescribed).count(),"total_steps":STEPS,"cycles":cycles,"same_boundary_support_promotions":roundtrips,"first_link_state_difference_tick":first_pose_delta}),
    )
}

/// Time only the advancing step, excluding lift command submission.
fn time_trial(dwell: Option<u64>) -> Result<Value> {
    let mut lane = setup(SDF, dwell)?;
    let mut previous_v = 0.;
    let mut times = Vec::with_capacity(STEPS);
    for tick in 1..=STEPS {
        command(&mut lane, tick, &mut previous_v)?;
        let start = Instant::now();
        lane.sim.step_exact(1)?;
        times.push(start.elapsed().as_nanos() as u64);
    }
    times.sort_unstable();
    Ok(json!({"dwell_ms":dwell,"p50_ns":times[STEPS.div_ceil(2)-1],
        "p95_ns":times[(95*STEPS).div_ceil(100)-1], "total_ns":times.iter().sum::<u64>()}))
}

pub(super) fn write(output: &Path) -> Result<()> {
    let mut runs = Vec::new();
    let mut heldout = Vec::new();
    let mut early_wake = Vec::new();
    for (name, pose) in [
        ("canonical", "0 0 0.14"),
        ("raised_upper", "0 0 0.18"),
        ("offset_upper", "0.025 0 0.14"),
    ] {
        let source = SDF.replace(
            "<pose>0 0 0.14 0 0 0</pose>",
            &format!("<pose>{pose} 0 0 0</pose>"),
        );
        let control = reference(&source)?;
        for ms in DWELLS_MS {
            let result = measure(&source, name, ms, None, &control)?;
            if name == "canonical" {
                runs.push(result);
            } else {
                heldout.push(result);
            }
        }
        for lead_us in [0, 500, 2000, 5000, 10000, 20000] {
            early_wake.push(measure(&source, name, 0, Some(lead_us), &control)?);
        }
    }
    fs::create_dir_all(output)?;
    fs::write(
        output.join("summary.json"),
        serde_json::to_vec_pretty(&json!({"dt_us":STEP_US,
        "command_schedule":"fixed historical departure times","runs":runs,"heldout":heldout, "early_wake":early_wake}))?,
    )?;

    // Rotate the order over complete blocks to distribute host warm-up and load.
    let policies = [None, Some(0), Some(2), Some(8), Some(20), Some(40)];
    let mut trials = Vec::new();
    for shift in 0..policies.len() {
        for j in 0..policies.len() {
            let mut trial = time_trial(policies[(j + shift) % policies.len()])?;
            trial["block"] = json!(shift + 1);
            trials.push(trial);
        }
    }
    fs::write(
        output.join("timings.json"),
        serde_json::to_vec_pretty(&json!({
        "step_count_per_trial":STEPS,"unit":"ns", "trials":trials}))?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn dormant_budget_and_reference_are_measured_on_the_same_clock() -> Result<()> {
        let control = reference(SDF)?;
        let held = measure(SDF, "canonical", 0, None, &control)?;
        let dynamic = measure(SDF, "canonical", 2000, None, &control)?;
        assert!(held["prescribed_steps"].as_u64().unwrap() > 0);
        assert_eq!(dynamic["prescribed_steps"], 0);
        // The first 40 ms dwell ends on the first departure command. Applying
        // the lift input first eliminates the otherwise observable round trip.
        let simultaneous = measure(SDF, "canonical", 40, None, &control)?;
        assert_eq!(simultaneous["same_boundary_support_promotions"], 0);
        assert_eq!(simultaneous["cycles"][0]["held_steps_before_departure"], 0);
        assert_eq!(
            simultaneous["cycles"][0]["switched_landings"],
            simultaneous["cycles"][0]["reference_landings"]
        );
        assert!(
            simultaneous["first_link_state_difference_tick"]
                .as_u64()
                .is_none_or(|t| t > sweep::DEPART_US[1] / STEP_US)
        );
        for (held, always) in held["cycles"]
            .as_array()
            .unwrap()
            .iter()
            .zip(dynamic["cycles"].as_array().unwrap())
        {
            assert_eq!(held["reference_landings"], always["switched_landings"]);
            assert_eq!(always["first_pct"], 0.0);
            assert_eq!(always["window_pct"], 0.0);
            assert!(held["held_steps_before_departure"].as_u64().unwrap() > 0);
        }
        Ok(())
    }
}
