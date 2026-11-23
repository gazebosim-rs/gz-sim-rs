// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Refine only the third touchdown, starting each treatment at the same measured cut.
use super::*;
use gz_physics_multifidelity::LandingFork;
use std::{path::Path, time::Duration};

#[path = "contact_controls.rs"]
pub(super) mod contact_controls;
#[path = "energy.rs"]
pub(super) mod energy;
#[path = "rigid_energy.rs"]
pub(super) mod rigid_energy;
#[path = "touchdown_timing.rs"]
mod timing;

const STEP_US: u64 = 500;
const CUT_TICK: u64 = 1423;
const LABELS: [&str; 2] = ["reference", "switched"];

struct Outer {
    tick: u64,
    command: Option<f64>,
}

impl Outer {
    fn substeps(&self, factor: u32) -> u32 {
        if self.tick <= CUT_TICK + WINDOW_STEPS as u64 {
            factor
        } else {
            1
        }
    }
}

fn schedule() -> Vec<Outer> {
    let stop = sweep::DEPART_US[2] / STEP_US + STROKES[2].start_park();
    let mut previous = sweep::command((CUT_TICK - 1) * STEP_US, 0).1;
    (CUT_TICK + 1..=stop)
        .map(|tick| {
            let speed = sweep::command((tick - 1) * STEP_US, 0).1;
            let command = (speed != previous).then_some(speed);
            previous = speed;
            Outer { tick, command }
        })
        .collect()
}

struct Cut {
    cases: [matched::Case; 2],
}

impl Cut {
    fn history(mask: u8) -> Result<matched::Case> {
        let mut case = matched::Case::new(mask, CUT_TICK as usize + 1)?;
        let mut previous = 0.;
        for tick in 0..=CUT_TICK {
            let speed = sweep::command(tick.saturating_sub(1) * STEP_US, 0).1;
            let command = (speed != previous).then_some(speed);
            case.advance(tick as usize, command)?;
            previous = speed;
        }
        let state = case.samples[CUT_TICK as usize];
        if case.lane.tier()? != Dynamic
            || state.normal_impulse_ns != 0.
            || state.vertical_gap_m <= 0.
        {
            return Err("cut must precede contact and have both histories dynamic".into());
        }
        Ok(case)
    }

    fn live_history(switched: bool) -> Result<Lane> {
        Ok(Self::history(if switched { 7 } else { 0 })?.lane)
    }

    fn new() -> Result<Self> {
        Ok(Self {
            cases: [Self::history(0)?, Self::history(7)?],
        })
    }

    fn fork(&self, receiver: usize, motion: Option<usize>) -> Result<LandingFork> {
        let target = &self.cases[receiver];
        Ok(target.backend().landing_fork(
            motion.map(|i| self.cases[i].backend()),
            target.lane.foot,
            target.lane.pad,
            Duration::from_micros(STEP_US),
        )?)
    }
}

#[derive(Serialize)]
struct State {
    time_us: u64,
    outer_tick: u64,
    dt_us: u64,
    foot: Body,
    upper: Body,
    pad: Body,
    normal_impulse_ns: f64,
    /// Local sign quadrants in ascending (X,Y) order.
    corner_ns: [f64; 4],
}

impl State {
    fn read(
        fork: &LandingFork,
        foot_id: BodyId,
        pad_id: BodyId,
        time_us: u64,
        tick: u64,
        dt_us: u64,
    ) -> Self {
        let [foot, upper, pad] = fork.states().map(Body::from);
        let rot = DQuat::from_array(foot.orientation).conjugate();
        let mut corner_ns = [0.; 4];
        for patch in fork.contact_patches() {
            let foot_a = (patch.a, patch.b) == (foot_id, pad_id);
            if !foot_a && (patch.b, patch.a) != (foot_id, pad_id) {
                continue;
            }
            for p in patch.points {
                let local = rot * DVec3::from_array(if foot_a { p.lever_a } else { p.lever_b });
                let corner = usize::from(local.x > 0.) * 2 + usize::from(local.y > 0.);
                corner_ns[corner] += p.normal_impulse;
            }
        }
        Self {
            time_us,
            outer_tick: tick,
            dt_us,
            foot,
            upper,
            pad,
            corner_ns,
            normal_impulse_ns: fork.normal_impulse(),
        }
    }
}

#[derive(Serialize)]
struct Trace {
    receiver: &'static str,
    motion: &'static str,
    factor: u32,
    geometry: Value,
    generalized_velocity: Vec<f64>,
    joint_coordinates: Vec<Vec<f64>>,
    frames: Vec<State>,
}

fn simulate(
    cut: &Cut,
    receiver: usize,
    motion: Option<usize>,
    factor: u32,
    schedule: &[Outer],
) -> Result<Trace> {
    let lane = &cut.cases[receiver].lane;
    let mut fork = cut.fork(receiver, motion)?;
    let state = |f: &LandingFork, t, tick, dt| State::read(f, lane.foot, lane.pad, t, tick, dt);
    let first = state(&fork, CUT_TICK * STEP_US, CUT_TICK, 0);
    let donor = cut.cases[motion.unwrap_or(receiver)].samples[CUT_TICK as usize];
    if first.foot != donor.foot
        || first.upper != donor.upper
        || first.pad != donor.pad
        || first.normal_impulse_ns != 0.
    {
        return Err("native fork differs from the donor or the cut has a nonzero reaction".into());
    }
    let generalized_velocity = fork.generalized_velocity();
    let joint_coordinates = fork.joint_coordinates();
    let mut frames = vec![first];
    for outer in schedule {
        let n = outer.substeps(factor);
        let dt = STEP_US / u64::from(n);
        let mut t = (outer.tick - 1) * STEP_US;
        fork.step_outer(n, outer.command, |fork| {
            t += dt;
            frames.push(state(fork, t, outer.tick, dt));
        })?;
        if t != outer.tick * STEP_US {
            return Err("native substeps did not cover the actuator interval".into());
        }
    }
    Ok(Trace {
        receiver: LABELS[receiver],
        motion: motion.map_or("native", |i| LABELS[i]),
        factor,
        generalized_velocity,
        joint_coordinates,
        frames,
        geometry: json!({"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF}),
    })
}

#[derive(Default)]
struct Impulses {
    sum: f64,
    corners: [f64; 4],
}

fn impulses(frames: &[State]) -> Impulses {
    let mut j = Impulses::default();
    for s in frames {
        j.sum += s.normal_impulse_ns;
        for (total, point) in j.corners.iter_mut().zip(s.corner_ns) {
            *total += point;
        }
    }
    j
}

/// A new positive reaction qualifies as a recontact only after a zero-load
/// sample with more than ten micrometres of free clearance.
fn recontacts(after_landing: &[State]) -> Vec<Value> {
    let mut free_since = None;
    let mut peak = 0.0_f64;
    let mut loaded = Vec::new();
    for s in after_landing {
        let gap = sole_gap(s.foot, s.pad);
        if s.normal_impulse_ns == 0. && gap > 1e-5 {
            free_since.get_or_insert(s.time_us);
            peak = peak.max(gap);
        } else if s.normal_impulse_ns > 0.
            && let Some(free) = free_since.take()
        {
            loaded.push(json!({
                "first_free_us":free, "peak_free_gap_m":peak,"outer_tick":s.outer_tick,
                "time_us":s.time_us, "dt_us":s.dt_us, "normal_impulse_ns":s.normal_impulse_ns,
                "corner_ns":s.corner_ns,
            }));
            peak = 0.;
        }
    }
    loaded
}

fn summarize(trace: &Trace, live: &[Sample], prefix_ns: f64) -> Result<Value> {
    let start = &trace.frames[0];
    let solves = &trace.frames[1..];
    let first_index = solves
        .iter()
        .position(|s| s.normal_impulse_ns > 0.)
        .ok_or("no positive landing impulse")?;
    let first = &solves[first_index];
    let outer_count = solves.partition_point(|s| s.outer_tick == CUT_TICK + 1);
    let window_count = solves.partition_point(|s| s.outer_tick <= CUT_TICK + WINDOW_STEPS as u64);
    if window_count != trace.factor as usize * WINDOW_STEPS
        || first.time_us > solves[window_count - 1].time_us
    {
        return Err("refinement missed touchdown".into());
    }
    let outer = impulses(&solves[..outer_count]);
    let window = impulses(&solves[..window_count]);
    let stroke = impulses(solves);
    let end = &solves[window_count - 1];
    let balance = end.foot.velocity[2] + end.upper.velocity[2]
        - start.foot.velocity[2]
        - start.upper.velocity[2]
        + 2. * 9.81 * 0.005;
    let mut max_pad_error = 0.0_f64;
    let mut outer_live_matches = true;
    for s in solves {
        if !s.normal_impulse_ns.is_finite()
            || s.corner_ns.iter().any(|j| !j.is_finite() || *j < 0.)
            || (s.normal_impulse_ns - s.corner_ns.iter().sum::<f64>()).abs() > 1e-12
        {
            return Err("native point readback does not reproduce the pair impulse".into());
        }
        if s.time_us == s.outer_tick * STEP_US {
            let l = live[(s.outer_tick - CUT_TICK - 1) as usize];
            let error = DVec3::from(s.pad.position).distance(l.pad.position.into());
            max_pad_error = max_pad_error.max(error);
            if s.pad.velocity != l.pad.velocity
                || s.pad.orientation != l.pad.orientation
                || error > 1e-12
            {
                return Err(
                    "actuator command or lift trajectory changed at an outer boundary".into(),
                );
            }
            outer_live_matches &= s.foot == l.foot
                && s.upper == l.upper
                && s.pad == l.pad
                && s.normal_impulse_ns == l.normal_impulse_ns;
        }
    }
    if trace.factor == 1 && trace.motion == "native" && !outer_live_matches {
        return Err(format!(
            "untreated {} continuation differs from live Bevy",
            trace.receiver
        )
        .into());
    }
    Ok(json!({
        "receiver":trace.receiver,"motion":trace.motion,"factor":trace.factor,"native_solves":solves.len(),
        "first_native_ns":first.normal_impulse_ns,"first_native_time_us":first.time_us,"first_native_corners_ns":first.corner_ns,
        "first_outer_ns":outer.sum,"first_outer_corners_ns":outer.corners,"window_ns":window.sum,"window_corners_ns":window.corners,
        "window_vertical_momentum_ns":balance,"window_momentum_residual_ns":window.sum-balance,
        "first_post_window_ns":solves[window_count].normal_impulse_ns,
        "recontacts":recontacts(&solves[first_index+1..]),
        "remaining_stroke_ns":stroke.sum,"stroke_prefix_ns":prefix_ns,"stroke_ns":prefix_ns+stroke.sum,
        "max_outer_pad_position_error_m":max_pad_error,"one_x_live_exact":(trace.factor==1).then_some(outer_live_matches),
        "trace":format!("traces/{}-{}-{}x.json",trace.receiver,trace.motion,trace.factor),
    }))
}

fn write_json(path: &Path, value: &impl Serialize) -> Result<()> {
    let mut writer = BufWriter::new(fs::File::create(path)?);
    serde_json::to_writer(&mut writer, value)?;
    writer.flush()?;
    Ok(())
}

fn study(cut: &Cut, output: Option<&Path>) -> Result<Value> {
    let schedule = schedule();
    let mut live_cut = Cut::new()?;
    for t in &schedule {
        for case in &mut live_cut.cases {
            case.advance(t.tick as usize, t.command)?;
        }
    }
    if let Some(out) = output {
        fs::create_dir_all(out.join("traces"))?;
    }
    let mut runs = Vec::new();
    for receiver in 0..2 {
        let live = &live_cut.cases[receiver].samples[CUT_TICK as usize + 1..];
        for motion in [None, Some(0), Some(1)] {
            let donor = motion.unwrap_or(receiver);
            let native = cut.fork(donor, None)?;
            let incoming = &cut.cases[donor].samples;
            let departure = (sweep::DEPART_US[2] / STEP_US) as usize;
            let prefix = incoming[departure + 1..]
                .iter()
                .map(|s| s.normal_impulse_ns)
                .sum();
            for factor in [1, 2, 4] {
                let trace = simulate(cut, receiver, motion, factor, &schedule)?;
                if native.generalized_velocity() != trace.generalized_velocity
                    || native.joint_coordinates() != trace.joint_coordinates
                {
                    return Err("incoming native generalized motion differs from donor".into());
                }
                let row = summarize(&trace, live, prefix)?;
                if let Some(out) = output {
                    write_json(&out.join(row["trace"].as_str().unwrap()), &trace)?;
                }
                runs.push(row);
            }
        }
    }
    Ok(
        json!({"outer_us":STEP_US,"cut_tick":CUT_TICK,"cut_us":CUT_TICK*STEP_US,
        "refinement_ends_us":(CUT_TICK+WINDOW_STEPS as u64)*STEP_US,
        "stroke_ends_us":schedule.last().ok_or("missing stroke")?.tick*STEP_US,
        "outer_steps":schedule.len(),
        "commands":schedule.iter().filter_map(|s|s.command.map(|vz|json!({"time_us":(s.tick-1)*STEP_US,"vz_m_s":vz}))).collect::<Vec<_>>(),
        "runs":runs}),
    )
}

fn write_measurement(cut: &Cut, output: &Path) -> Result<()> {
    let report = study(cut, Some(output))?;
    fs::write(
        output.join("summary.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    Ok(())
}

pub(super) fn write(output: &Path) -> Result<()> {
    let cut = Cut::new()?;
    write_measurement(&cut, output)?;
    timing::write(&cut, output)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn untouched_cuts_reproduce_all_eighteen_recordings_and_the_summary() -> Result<()> {
        let outputs = [tempfile::tempdir()?, tempfile::tempdir()?];
        for out in &outputs {
            write_measurement(&Cut::new()?, out.path())?;
        }
        let compare = |name: &Path| -> Result<()> {
            if fs::read(outputs[0].path().join(name))? != fs::read(outputs[1].path().join(name))? {
                return Err(
                    format!("non-deterministic touchdown measurement {}", name.display()).into(),
                );
            }
            Ok(())
        };
        compare(Path::new("summary.json"))?;
        let files =
            fs::read_dir(outputs[0].path().join("traces"))?.collect::<std::io::Result<Vec<_>>>()?;
        assert_eq!(files.len(), 18);
        for file in files {
            compare(&Path::new("traces").join(file.file_name()))?;
        }
        Ok(())
    }

    #[test]
    fn refinement_confines_extra_solves_to_touchdown_from_matched_donors() -> Result<()> {
        let cut = Cut::new()?;
        let report = study(&cut, None)?;
        let rows = report["runs"].as_array().unwrap();
        assert_eq!(rows.len(), 18);
        let n = report["outer_steps"].as_u64().unwrap();
        for r in rows {
            let factor = r["factor"].as_u64().unwrap();
            assert_eq!(r["native_solves"], n + WINDOW_STEPS as u64 * (factor - 1));
            assert_eq!(r["stroke_prefix_ns"], 0.);
            assert_eq!(r["first_native_time_us"], 712000);
            let recontacts = r["recontacts"].as_array().unwrap();
            let reference_motion = r["motion"] == "reference"
                || (r["motion"] == "native" && r["receiver"] == "reference");
            if factor == 2 && reference_motion {
                assert_eq!(recontacts.len(), 1);
                assert_eq!(recontacts[0]["time_us"], 725000);
                assert_eq!(recontacts[0]["dt_us"], 500);
                assert!(recontacts[0]["peak_free_gap_m"].as_f64().unwrap() > 0.00014);
                assert!(
                    (0.029..0.030).contains(&recontacts[0]["normal_impulse_ns"].as_f64().unwrap())
                );
            } else {
                assert!(recontacts.is_empty());
            }
            if factor == 1 && (r["motion"] == "native" || r["motion"] == r["receiver"]) {
                assert_eq!(r["one_x_live_exact"], true);
            }
        }
        for receiver in LABELS {
            for factor in [1, 2, 4] {
                let native = rows
                    .iter()
                    .find(|r| {
                        r["receiver"] == receiver
                            && r["motion"] == "native"
                            && r["factor"] == factor
                    })
                    .unwrap();
                let sham = rows
                    .iter()
                    .find(|r| {
                        r["receiver"] == receiver
                            && r["motion"] == receiver
                            && r["factor"] == factor
                    })
                    .unwrap();
                for key in [
                    "first_native_time_us",
                    "first_outer_ns",
                    "window_ns",
                    "stroke_ns",
                ] {
                    assert_eq!(native[key], sham[key]);
                }
            }
        }
        Ok(())
    }
}
