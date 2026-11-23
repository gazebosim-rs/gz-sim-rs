// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Turn prospective weak recontact into native work, then measure its consequences.
use super::*;
#[path = "outer_study.rs"]
pub(crate) mod outer;
#[path = "weak_timing.rs"]
mod timing;
pub(crate) fn write_timing(out: &Path) -> Result<()> {
    timing::write(out)
}

const BUDGET: Budget = Budget {
    onset: Effort::Split(4),
    follow: Effort::Iterations(16),
    follow_ticks: 1,
};
const DURATION: u64 = sweep::END_US / STEP_US;
const FOLLOW: usize = 12;

#[derive(Clone, Copy, Serialize)]
struct Fixture {
    name: &'static str,
    offset_m: f64,
    speed_scale: f64,
    switched: bool,
}
impl Fixture {
    const fn new(name: &'static str, offset_m: f64, speed_scale: f64, switched: bool) -> Self {
        Self {
            name,
            offset_m,
            speed_scale,
            switched,
        }
    }
}
// One established case and a crossed, held-out offset × lift-speed × tier-history matrix.
const CASES: [Fixture; 9] = [
    Fixture::new("previous-edge", 0.22, 1.0, false),
    Fixture::new("inner-slow", 0.185, 0.85, false),
    Fixture::new("inner-fast", 0.185, 1.15, false),
    Fixture::new("outer-slow", 0.265, 0.85, false),
    Fixture::new("outer-fast", 0.265, 1.15, false),
    Fixture::new("inner-slow-holds", 0.185, 0.85, true),
    Fixture::new("inner-fast-holds", 0.185, 1.15, true),
    Fixture::new("outer-slow-holds", 0.265, 0.85, true),
    Fixture::new("outer-fast-holds", 0.265, 1.15, true),
];

fn start(f: Fixture, mode: Mode) -> Result<Rig> {
    let source = SDF.replacen(
        "<pose>0 0 -0.06",
        &format!("<pose>{} 0 -0.06", f.offset_m),
        1,
    );
    if source == SDF {
        return Err("missing lift placement in source fixture".into());
    }
    Rig::from_lane(Lane::from_sdf_at(&source, STEP_US)?, f.switched, mode)
}
fn command(f: Fixture, tick: u64) -> f64 {
    f.speed_scale * sweep::command(tick * STEP_US, 0).1
}
fn parked(tick: u64) -> bool {
    let (cycle, _) = sweep::command((tick - 1) * STEP_US, 0);
    tick <= sweep::DEPART_US[0] / STEP_US
        || tick > sweep::DEPART_US[cycle] / STEP_US + STROKES[cycle].start_park()
}

fn run(f: Fixture, mode: Mode) -> Result<(Vec<ReboundFrame>, usize)> {
    let mut r = start(f, mode)?;
    let mut frames = vec![r.frame()?];
    let mut holds = 0;
    while r.tick < DURATION {
        r.advance_at(command(f, r.tick))?;
        frames.push(r.frame()?);
        holds += usize::from(r.case.lane.tier()? == Prescribed);
    }
    Ok((frames, holds))
}

fn com(f: &ReboundFrame) -> DVec3 {
    (DVec3::from(f.foot.position) + DVec3::from(f.upper.position)) * 0.5
}
fn same_state(a: &ReboundFrame, b: &ReboundFrame) -> bool {
    a.tick == b.tick
        && a.foot == b.foot
        && a.upper == b.upper
        && a.pad == b.pad
        && a.normal_impulse_ns == b.normal_impulse_ns
        && a.corner_ns == b.corner_ns
}

fn fork(f: Fixture, cut: &ReboundFrame, mode: Mode) -> Result<Vec<ReboundFrame>> {
    let mut r = start(f, Mode::Weak(BUDGET))?;
    while r.tick < cut.tick {
        r.advance_at(command(f, r.tick))?;
    }
    if !same_state(&r.frame()?, cut) {
        return Err("weak landing replay failed to recover cut".into());
    }
    r.set_mode(mode);
    let mut frames = vec![r.frame()?];
    for _ in 0..FOLLOW {
        r.advance_at(command(f, r.tick))?;
        frames.push(r.frame()?);
    }
    Ok(frames)
}

fn continuous_metrics(frames: &[ReboundFrame], reference: &[ReboundFrame], holds: usize) -> Value {
    let mut sum_squared = 0.;
    let mut max_com = 0.0_f64;
    let mut cumulative = 0.;
    let mut max_cumulative = 0.0_f64;
    let mut strokes = vec![];
    for (i, (f, r)) in frames.iter().zip(reference).enumerate().skip(1) {
        let d = com(f).distance(com(r));
        sum_squared += d * d;
        max_com = max_com.max(d);
        cumulative += f.normal_impulse_ns - r.normal_impulse_ns;
        max_cumulative = max_cumulative.max(cumulative.abs());
        if sweep::DEPART_US[1..].contains(&(i as u64 * STEP_US)) || i + 1 == frames.len() {
            strokes.push(json!({"end_tick":i,"signed_impulse_difference_ns":cumulative,"end_com_distance_m":d}));
            cumulative = 0.;
        }
    }
    let after = &frames[1..];
    let weak = after.iter().filter(|r| r.weak_start).collect::<Vec<_>>();
    json!({"rms_com_error_m":(sum_squared/after.len() as f64).sqrt(),"max_com_error_m":max_com,
        "max_abs_within_stroke_cumulative_impulse_difference_ns":max_cumulative,"strokes":strokes,
        "prescribed_boundaries":holds,"weak_ticks":weak.iter().map(|r|r.tick).collect::<Vec<_>>(),
        "weak_without_load":weak.iter().filter(|r|r.normal_impulse_ns==0.).map(|r|r.tick).collect::<Vec<_>>(),
        "parked_weak_ticks":weak.iter().filter(|r|parked(r.tick)).map(|r|r.tick).collect::<Vec<_>>(),
        "parked_weak_without_load":weak.iter().filter(|r|parked(r.tick)&&r.normal_impulse_ns==0.).count(),
        "selected_parked_ticks":after.iter().filter(|r|r.selected && parked(r.tick)).count(),
        "parked_ticks":after.iter().filter(|r|parked(r.tick)).count(),"native_calls":after.iter().map(|r|r.native_steps as u64).sum::<u64>()})
}

// Frame zero is the common incoming boundary. Compare identical absolute-time
// windows, including when a finer contact grid moves first load into the next tick.
#[derive(Serialize)]
struct Window {
    start_tick: u64,
    impulses_ns: [f64; WINDOW_STEPS],
    end_com_m: [f64; 3],
}
impl Window {
    fn from_frames(frames: &[ReboundFrame]) -> Self {
        Self {
            start_tick: frames[1].tick,
            impulses_ns: std::array::from_fn(|i| frames[i + 1].normal_impulse_ns),
            end_com_m: com(&frames[WINDOW_STEPS]).to_array(),
        }
    }
    fn first(&self) -> Option<(u64, f64)> {
        self.impulses_ns
            .iter()
            .enumerate()
            .find(|(_, n)| **n > 0.)
            .map(|(i, n)| (self.start_tick + i as u64, *n))
    }
    fn against(&self, reference: &Self) -> Value {
        let sum: f64 = self.impulses_ns.iter().sum();
        let first = self.first();
        json!({"onset_ns":self.impulses_ns[0],"onset_error_ns":self.impulses_ns[0]-reference.impulses_ns[0],
            "first_positive_tick":first.map(|(tick,_)|tick),"first_positive_ns":first.map(|(_,n)|n),
            "first_positive_error_ns":first.zip(reference.first()).map(|((_,a),(_,b))|a-b),
            "window_ns":sum,"window_error_ns":sum-reference.impulses_ns.iter().sum::<f64>(),
            "window_end_com_error_m":DVec3::from(self.end_com_m).distance(DVec3::from(reference.end_com_m))})
    }
}

fn trace(out: &Path, name: &str, frames: &[ReboundFrame]) -> Result<()> {
    let measured = frames
        .iter()
        .map(|r| {
            let mut v = serde_json::to_value(r.with_effort())?;
            v["weak_start"] = json!(r.weak_start);
            Ok(v)
        })
        .collect::<Result<Vec<Value>>>()?;
    write_json(
        &out.join("traces").join(format!("{name}.json")),
        &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":measured}),
    )
}

pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut cases = vec![];
    let mut trajectories = vec![];
    let mut visual = None;
    for f in CASES {
        let (current, c_holds) = run(f, Mode::Budget(BUDGET))?;
        let (weak, w_holds) = run(f, Mode::Weak(BUDGET))?;
        let (fine, f_holds) = run(f, Mode::Uniform(Effort::Split(4)))?;
        let mut events = vec![];
        for (i, receipt) in weak.iter().enumerate().filter(|(_, r)| r.weak_start) {
            let cut = &weak[i - 1];
            let w = weak
                .get(i - 1..i + FOLLOW)
                .ok_or("incomplete weak landing interval")?;
            let c = fork(f, cut, Mode::Budget(BUDGET))?;
            let grid4 = fork(f, cut, Mode::Uniform(Effort::Split(4)))?;
            let grid8 = fork(f, cut, Mode::Uniform(Effort::Split(8)))?;
            let grid16 = fork(f, cut, Mode::Uniform(Effort::Split(16)))?;
            if !same_state(&grid4[1], receipt)
                || receipt.native_steps != 4
                || cut.normal_impulse_ns != 0.
            {
                return Err(
                    "live weak admission diverged from four native steps on the matched cut".into(),
                );
            }
            let mut pad = state(cut.pad);
            pad.velocity[2] = command(f, cut.tick);
            let (_, closing) = rebound_gate::projected_clearance_and_speed(
                state(cut.foot),
                pad,
                STEP_US as f64 * 1e-6,
            )
            .ok_or("weak crossing lacks supported footprint")?;
            if visual.is_none() && f.name != "previous-edge" {
                trace(out, "current", &c)?;
                trace(out, "weak", w)?;
                trace(out, "fine", &grid4)?;
                visual = Some(json!({"fixture":f.name,"onset_tick":receipt.tick}));
            }
            let [current_window, weak_window, four, eight, sixteen] =
                [c.as_slice(), w, &grid4, &grid8, &grid16].map(Window::from_frames);
            events.push(json!({"tick":receipt.tick,"parked":parked(receipt.tick),"incoming_angular_velocity_rad_s":cut.foot.angular_velocity,
                "projected_closing_m_s":closing,"onset_native_steps":receipt.native_steps,
                "native_windows":{"current":current_window,"weak":weak_window,"four":four,"eight":eight,"sixteen":sixteen},
                "vs_four":{"current":current_window.against(&four),"weak":weak_window.against(&four)},
                "vs_eight":{"current":current_window.against(&eight),"weak":weak_window.against(&eight),"fine":four.against(&eight)},
                "vs_sixteen":{"current":current_window.against(&sixteen),"weak":weak_window.against(&sixteen),"eight":eight.against(&sixteen)}}));
        }
        let motion = |frames: &[ReboundFrame]| {
            frames
                .iter()
                .skip(1)
                .map(|r| (com(r).to_array(), r.normal_impulse_ns))
                .collect::<Vec<_>>()
        };
        trajectories.push(json!({"fixture": f.name, "current":motion(&current),
            "weak":motion(&weak),"four":motion(&fine)}));
        cases.push(json!({"fixture":f,"events":events,"continuous_vs_four":{
            "current":continuous_metrics(&current,&fine,c_holds),"weak":continuous_metrics(&weak,&fine,w_holds),"fine":continuous_metrics(&fine,&fine,f_holds)}}));
    }
    if visual.is_none() {
        return Err("no weak event on the held-out support cases".into());
    }
    write_json(
        &out.join("continuous.json"),
        &json!({
        "actuator_tick_us":STEP_US,"first_tick":1,"sample_fields":["com_xyz_m","delivered_normal_impulse_ns"],
        "fixtures":trajectories}),
    )?;
    write_json(
        &out.join("study.json"),
        &json!({"actuator_tick_us":STEP_US,"ticks_per_trajectory":DURATION,"matched_window_ticks":WINDOW_STEPS,
        "visual":visual,"fixtures":cases}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn a_late_first_impulse_preserves_its_absolute_time_and_its_window_total() {
        let mut candidate = Window {
            start_tick: 1489,
            impulses_ns: [0.; WINDOW_STEPS],
            end_com_m: [0.; 3],
        };
        let mut reference = Window {
            start_tick: 1489,
            impulses_ns: [0.; WINDOW_STEPS],
            end_com_m: [0.; 3],
        };
        assert!(candidate.against(&reference)["first_positive_error_ns"].is_null());
        candidate.impulses_ns[1] = 0.125;
        reference.impulses_ns[0] = 0.125;
        let result = candidate.against(&reference);
        assert_eq!(result["first_positive_tick"], 1490);
        assert_eq!(result["onset_error_ns"], -0.125);
        assert_eq!(result["first_positive_error_ns"], 0.0);
        assert_eq!(result["window_error_ns"], 0.0);
    }
}
