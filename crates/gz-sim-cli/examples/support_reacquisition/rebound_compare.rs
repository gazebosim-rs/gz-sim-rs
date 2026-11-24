// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare responses at the same secondary landing, under a common legacy prefix.
use super::*;

#[path = "rebound_flight.rs"]
pub(super) mod flight;
#[path = "rebound_budget_study.rs"]
pub(super) mod budget_study;
#[path = "rebound_free_control.rs"]
pub(super) mod free_control;
#[path = "rebound_grid_validation.rs"]
mod grid_validation;

const BLOCKS: usize = 5;
const REPETITIONS: usize = 5;
const WINDOW: u64 = 10;
const FREE_GAP_M: f64 = 10e-6;

fn moving_end(stroke: usize) -> u64 {
    sweep::DEPART_US[stroke] / STEP_US + STROKES[stroke].start_park()
}
fn history(switched: bool) -> &'static str {
    if switched {
        "switched"
    } else {
        "always_dynamic"
    }
}

#[derive(Clone, Copy)]
struct Target {
    deck: Deck,
    switched: bool,
    stroke: usize,
    first_landing: u64,
    cut: ReboundFrame,
    onset: u64,
    peak_gap: f64,
    onset_impulse: f64,
}

fn target(deck: Deck, switched: bool, stroke: usize) -> Result<Option<Target>> {
    let mut rig = Rig::new(deck, switched, Mode::Legacy)?;
    let depart = sweep::DEPART_US[stroke] / STEP_US;
    while rig.tick < depart {
        rig.advance()?;
    }
    let mut cut = rig.frame()?;
    let mut peak_gap = 0.0_f64;
    let mut first = None;
    let mut largest: Option<Target> = None;
    while rig.tick < moving_end(stroke) {
        rig.advance()?;
        let frame = rig.frame()?;
        if frame.normal_impulse_ns > 0. {
            if peak_gap > FREE_GAP_M {
                if let Some(primary) = first {
                    if !frame.selected
                        && largest.is_none_or(|r| frame.normal_impulse_ns > r.onset_impulse)
                    {
                        largest = Some(Target {
                            deck,
                            switched,
                            stroke,
                            first_landing: primary,
                            cut,
                            onset: frame.tick,
                            peak_gap,
                            onset_impulse: frame.normal_impulse_ns,
                        });
                    }
                } else {
                    first = Some(frame.tick);
                }
            }
            cut = frame;
            peak_gap = 0.;
        } else if frame.selected {
            // Finish the preceding two-tick intervention before changing policy.
            cut = frame;
            peak_gap = 0.;
        } else {
            peak_gap = peak_gap.max(frame.gap_m);
        }
    }
    Ok(largest)
}

fn policies(onset: u64) -> [(&'static str, Mode); 4] {
    [
        ("coarse", Mode::Coarse),
        ("legacy", Mode::Legacy),
        ("candidate", Mode::Candidate),
        ("four_substeps", Mode::Fixed { onset }),
    ]
}

fn cut_matches(f: &ReboundFrame, cut: &ReboundFrame) -> bool {
    f.tick == cut.tick
        && f.foot == cut.foot
        && f.upper == cut.upper
        && f.pad == cut.pad
        && f.normal_impulse_ns == cut.normal_impulse_ns
        && f.corner_ns == cut.corner_ns
}

fn prefix(t: Target) -> Result<Rig> {
    let mut rig = Rig::new(t.deck, t.switched, Mode::Legacy)?;
    while rig.tick < t.cut.tick {
        rig.advance()?;
    }
    if !cut_matches(&rig.frame()?, &t.cut) {
        return Err("reconstructed incoming two-link pose, twists or reaction changed".into());
    }
    Ok(rig)
}

fn continuation(t: Target, mode: Mode) -> Result<Vec<ReboundFrame>> {
    let mut rig = prefix(t)?;
    rig.set_mode(mode);
    let mut frames = Vec::new();
    while rig.tick < moving_end(t.stroke).max(t.onset + WINDOW - 1) {
        rig.advance()?;
        frames.push(rig.frame()?);
    }
    Ok(frames)
}

fn com(f: &ReboundFrame) -> DVec3 {
    // Two equal-mass links; their body origins are their centers of mass.
    (DVec3::from(f.foot.position) + DVec3::from(f.upper.position)) * 0.5
}

fn score(t: Target, frames: &[ReboundFrame], fine: &[ReboundFrame]) -> Result<Value> {
    let start = usize::try_from(t.onset - t.cut.tick - 1)?;
    let end = start + WINDOW as usize;
    let window = &frames[start..end];
    let fixed_window = &fine[start..end];
    let mut witnessed = false;
    let first_resumed = frames.iter().find(|f| {
        if f.normal_impulse_ns == 0. && f.gap_m > FREE_GAP_M {
            witnessed = true;
        }
        witnessed && f.normal_impulse_ns > 0.
    });
    let sum = |fs: &[ReboundFrame]| fs.iter().map(|f| f.normal_impulse_ns).sum::<f64>();
    let first_selected = frames.iter().find(|f| f.selected).map(|f| f.tick);
    let diff = |last: usize| com(&frames[last]).distance(com(&fine[last])) * 1000.;
    let all = frames
        .iter()
        .zip(fine)
        .map(|(a, b)| com(a).distance(com(b)) * 1000.)
        .fold(0., f64::max);
    Ok(json!({"first_resumed_tick":first_resumed.map(|f|f.tick),
        "first_resumed_ns":first_resumed.map(|f|f.normal_impulse_ns),"common_first_ns":window[0].normal_impulse_ns,
        "common_first_delta_ns":window[0].normal_impulse_ns-fixed_window[0].normal_impulse_ns,
        "window_ns":sum(window),"window_delta_ns":sum(window)-sum(fixed_window),
        "corner_window_ns":std::array::from_fn::<_,4,_>(|i|window.iter().map(|f|f.corner_ns[i]).sum::<f64>()),
        "cut_to_stop_ns":sum(frames),"cut_to_stop_delta_ns":sum(frames)-sum(fine),
        "window_end_com_delta_mm":diff(end-1),"stroke_end_com_delta_mm":diff(frames.len()-1),
        "max_com_delta_mm":all,"first_selected_tick":first_selected,
        "selected_ticks":frames.iter().filter(|f|f.selected).map(|f|f.tick).collect::<Vec<_>>() }))
}

fn actuator_parity(
    t: Target,
    frames: &[ReboundFrame],
    fine: &[ReboundFrame],
) -> Result<(f64, f64)> {
    if frames.len() != fine.len() {
        return Err("matched continuations differ in length".into());
    }
    let mut pose_error = 0.0_f64;
    let mut speed_error = 0.0_f64;
    for (a, b) in frames.iter().zip(fine) {
        if a.tick != b.tick
            || a.time_us != a.tick * STEP_US
            || b.time_us != a.time_us
            || a.pad.orientation != b.pad.orientation
        {
            return Err("different matched actuator clock or orientation".into());
        }
        let command = DVec3::Z * sweep::command((a.tick - 1) * STEP_US, 0).1;
        pose_error =
            pose_error.max(DVec3::from(a.pad.position).distance(DVec3::from(b.pad.position)));
        speed_error = speed_error
            .max(command.distance(DVec3::from(a.pad.velocity)))
            .max(command.distance(DVec3::from(b.pad.velocity)));
    }
    if pose_error > 1e-12 || speed_error > 1e-11 {
        return Err(format!("actuator mismatch across matched continuations of {} {}: {pose_error} m, {speed_error} m/s",t.deck.label(),history(t.switched)).into());
    }
    Ok((pose_error, speed_error))
}

fn compare(t: Target) -> Result<Value> {
    let fine = continuation(t, Mode::Fixed { onset: t.onset })?;
    let mut rows = Vec::new();
    for (label, mode) in policies(t.onset) {
        let own;
        let frames = if label == "four_substeps" {
            &fine
        } else {
            own = continuation(t, mode)?;
            &own
        };
        let (pose_error, speed_error) = actuator_parity(t, frames, &fine)?;
        let mut row = score(t, frames, &fine)?;
        row["max_pad_position_difference_m"] = pose_error.into();
        row["max_pad_command_error_m_s"] = speed_error.into();
        row["policy"] = label.into();
        rows.push(row);
    }
    Ok(
        json!({"deck":t.deck.label(),"history":history(t.switched),"stroke":t.stroke+1,"primary_tick":t.first_landing,
        "cut_tick":t.cut.tick,"cut_foot":t.cut.foot,"cut_upper":t.cut.upper,"cut_pad":t.cut.pad,
        "peak_unloaded_gap_m":t.peak_gap,"legacy_onset_tick":t.onset,"legacy_first_ns":t.onset_impulse,
        "stop_tick":fine.last().ok_or("empty continuation")?.tick,"rows":rows}),
    )
}

#[derive(Default, Serialize)]
struct Timings {
    touchdown_ticks_ns: Vec<Vec<u64>>,
    parked_ticks_ns: Vec<Vec<u64>>,
}

fn timed(t: Target, mode: Mode, records: &mut Timings) -> Result<()> {
    let mut rig = Rig::new(t.deck, t.switched, mode)?;
    let mut parked = Vec::new();
    for _ in 0..WINDOW {
        parked.push(rig.advance()?);
        if rig.frame()?.selected {
            return Err("unexpected work selection on the initial parked support".into());
        }
    }
    rig.set_mode(Mode::Legacy);
    while rig.tick < t.cut.tick {
        rig.advance()?;
    }
    if !cut_matches(&rig.frame()?, &t.cut) {
        return Err("bench diverged from matched legacy prefix".into());
    }
    rig.set_mode(mode);
    let mut window = Vec::new();
    while rig.tick < t.onset + WINDOW - 1 {
        let ns = rig.advance()?;
        if rig.tick >= t.onset {
            window.push(ns);
        }
    }
    records.touchdown_ticks_ns.push(window);
    records.parked_ticks_ns.push(parked);
    Ok(())
}

fn p95(values: impl Iterator<Item = u64>) -> f64 {
    let mut values = values.collect::<Vec<_>>();
    values.sort_unstable();
    values[(values.len() * 95).div_ceil(100) - 1] as f64 * 1e-3
}

fn benchmark(t: Target) -> Result<Value> {
    benchmark_settings(t, policies(t.onset))
}

fn benchmark_settings<const N: usize>(t: Target, settings: [(&str, Mode); N]) -> Result<Value> {
    let mut records: [Timings; N] = std::array::from_fn(|_| Timings::default());
    for block in 0..BLOCKS {
        for r in 0..REPETITIONS {
            for pos in 0..settings.len() {
                let direction = if block % 2 == 0 {
                    pos
                } else {
                    settings.len() - 1 - pos
                };
                let index = (direction + block + r) % settings.len();
                timed(t, settings[index].1, &mut records[index])?;
            }
        }
    }
    let rows=settings.iter().zip(records).map(|((label,_),r)|{
        let windows=&r.touchdown_ticks_ns;
        let window_us=|w:&[Vec<u64>]|p95(w.iter().flatten().copied());
        json!({"policy":label,"window_tick_p95_us":window_us(windows),
            "first_two_tick_p95_us":p95(windows.iter().flat_map(|w|w[..2].iter().copied())),
            "parked_tick_p95_us":window_us(&r.parked_ticks_ns),
            "mean_window_total_us":windows.iter().flatten().sum::<u64>() as f64*1e-3/windows.len() as f64,
            "block_window_p95_us":windows.chunks_exact(REPETITIONS).map(window_us).collect::<Vec<_>>(),"raw":r})
    }).collect::<Vec<_>>();
    Ok(
        json!({"deck":t.deck.label(),"history":history(t.switched),"stroke":t.stroke+1,"onset_tick":t.onset,"rows":rows}),
    )
}

pub(super) fn write(out: &Path) -> Result<()> {
    let mut controls = Vec::new();
    let mut targets = Vec::new();
    for deck in [Deck::Flat, Deck::Edge] {
        for switched in [false, true] {
            if let Some(t) = target(deck, switched, 2)? {
                controls.push(compare(t)?);
                targets.push(t);
            } else {
                controls.push(json!({"deck":deck.label(),"history":history(switched),"rows":[],
                    "stroke":3,"reason":"no unselected secondary after a measured unloaded 10 µm separation in third stroke"}));
                if let Some(t) = target(deck, switched, 1)? {
                    controls.push(compare(t)?);
                    targets.push(t);
                }
            }
        }
    }
    write_json(
        &out.join("compare.json"),
        &json!({"outer_us":STEP_US,"window_ticks":WINDOW,"cases":controls}),
    )?;
    let timings = targets
        .into_iter()
        .map(benchmark)
        .collect::<Result<Vec<_>>>()?;
    write_json(
        &out.join("bench.json"),
        &json!({"blocks":BLOCKS,"repetitions_per_block":REPETITIONS,
        "outer_us":STEP_US,"window_ticks":WINDOW,"cases":timings}),
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn matched_rebound_control_reconstructs_the_previously_missed_landing() -> Result<()> {
        let t = target(Deck::Flat, true, 2)?.ok_or("missing flat switched rebound")?;
        assert_eq!(t.onset, 1454);
        assert!((t.onset_impulse - 0.6018516252850385).abs() < 1e-9);
        let result = compare(t)?;
        let rows = result["rows"].as_array().unwrap();
        assert_eq!(rows[0]["window_ns"], rows[1]["window_ns"]);
        assert_eq!(rows[1]["first_resumed_tick"], t.onset);
        assert_eq!(rows[2]["first_selected_tick"], t.onset);
        for key in [
            "common_first_delta_ns",
            "window_delta_ns",
            "stroke_end_com_delta_mm",
        ] {
            assert!(rows[2][key].as_f64().unwrap().abs() < rows[1][key].as_f64().unwrap().abs());
        }
        assert_eq!(rows[3]["window_delta_ns"], 0.);
        assert_eq!(rows[3]["selected_ticks"].as_array().unwrap().len(), 10);
        Ok(())
    }
}
