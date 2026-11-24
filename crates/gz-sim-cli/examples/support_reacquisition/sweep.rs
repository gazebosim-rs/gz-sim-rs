// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare contact solves on different grids with the same physical lift commands.
use super::*;
use std::path::Path;

const COMMAND_US: u64 = 500;
const WINDOW_US: u64 = 5_000;
pub(super) const DEPART_US: [u64; 3] = [40_000, 386_500, 620_500];
pub(super) const END_US: u64 = 1_085_500;

// The actuator updates every 0.5 ms in every run, including during braking.
// The finer worlds take extra solver steps without changing the physical drive.
pub(super) fn command(start_us: u64, dwell_shift_us: i64) -> (usize, f64) {
    let index = DEPART_US
        .partition_point(|&t| t <= start_us)
        .saturating_sub(1);
    if start_us < DEPART_US[index] {
        return (index, 0.);
    }
    let stroke = &STROKES[index];
    let age = start_us - DEPART_US[index];
    let down_end = stroke.down * COMMAND_US;
    let return_start = (down_end + 24 * COMMAND_US)
        .checked_add_signed(dwell_shift_us)
        .unwrap();
    let speed = if age < down_end {
        -1.25
    } else if age < return_start {
        0.
    } else {
        stroke.velocity((age - return_start) / COMMAND_US + stroke.down + 24)
    };
    (index, speed)
}

#[derive(Serialize)]
struct Point {
    foot_local_m: [f64; 3],
    foot_corner: [i8; 2],
    lever_foot_m: [f64; 3],
    lever_pad_m: [f64; 3],
    normal_on_foot: [f64; 3],
    world_m: [f64; 3],
    normal_impulse_ns: f64,
}

#[derive(Serialize)]
struct ContactSample {
    #[serde(flatten)]
    state: Sample,
    points: Vec<Point>,
}

impl ContactSample {
    fn read(lane: &Lane) -> Result<Self> {
        let state = lane.sample()?;
        let solver = lane
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing rigid backend")?;
        let world_to_foot = DQuat::from_array(state.foot.orientation).conjugate();
        let mut points = Vec::new();
        for patch in solver.rigid_contact_patches() {
            let foot_is_a = patch.a == lane.foot && patch.b == lane.pad;
            if !(foot_is_a || patch.a == lane.pad && patch.b == lane.foot) {
                continue;
            }
            let n = DVec3::from(patch.normal_on_a);
            for p in patch.points {
                let (foot, pad, normal) = if foot_is_a {
                    (p.lever_a, p.lever_b, n)
                } else {
                    (p.lever_b, p.lever_a, -n)
                };
                let local = world_to_foot * DVec3::from(foot);
                if !p.normal_impulse.is_finite() || p.normal_impulse < 0. {
                    return Err("invalid solved point impulse".into());
                }
                points.push(Point {
                    foot_local_m: local.to_array(),
                    foot_corner: [local.x.signum() as i8, local.y.signum() as i8],
                    lever_foot_m: foot,
                    lever_pad_m: pad,
                    normal_on_foot: normal.to_array(),
                    world_m: (DVec3::from(state.pad.position) + DVec3::from(pad)).to_array(),
                    normal_impulse_ns: p.normal_impulse,
                });
            }
        }
        let point_sum: f64 = points.iter().map(|p| p.normal_impulse_ns).sum();
        if (point_sum - state.normal_impulse_ns).abs() > 1e-12 {
            return Err("solved point impulses do not account for the pair impulse".into());
        }
        Ok(Self { state, points })
    }
}

#[derive(Serialize)]
struct Snapshot {
    tick: u64,
    time_s: f64,
    cycle: usize,
    prescribed: bool,
    switched: ContactSample,
    reference: ContactSample,
}

#[derive(Default, Serialize)]
struct Admission {
    promotion_us: Option<u64>,
    demotion_us: Option<u64>,
    demotion_deferrals: usize,
    last_deferral: Option<&'static str>,
}

fn request_demotion(lane: &mut Lane, end_us: u64, entry: &mut Admission) -> Result<()> {
    match lane.demote() {
        Ok(()) => entry.demotion_us = Some(end_us),
        Err(FidelityError::Physics(PhysicsError::TransitionDeferred { reason })) => {
            entry.demotion_deferrals += 1;
            entry.last_deferral = Some(reason);
        }
        Err(e) => return Err(e.into()),
    }
    Ok(())
}

fn summed_momentum(s: &ContactSample) -> f64 {
    s.state.foot.velocity[2] + s.state.upper.velocity[2]
}

struct Touchdown {
    tick: usize,
    first_free_tick: u64,
    peak_free_gap_m: f64,
}

fn locate_landings(
    frames: &[Snapshot],
    departure_tick: usize,
    end_tick: usize,
    lane: fn(&Snapshot) -> &ContactSample,
) -> Result<Vec<Touchdown>> {
    let mut found = Vec::new();
    let mut first_free = None;
    let mut peak_gap = 0.0_f64;
    for f in &frames[departure_tick..=end_tick] {
        let s = &lane(f).state;
        if s.normal_impulse_ns == 0. && s.vertical_gap_m > 1e-5 {
            first_free.get_or_insert(f.tick);
            peak_gap = peak_gap.max(s.vertical_gap_m);
        } else if s.normal_impulse_ns > 0.
            && let Some(first_free_tick) = first_free.take()
        {
            found.push(Touchdown {
                tick: f.tick as usize,
                first_free_tick,
                peak_free_gap_m: peak_gap,
            });
            peak_gap = 0.;
        }
    }
    if found.is_empty() {
        return Err("missing landing after measured detachment".into());
    }
    Ok(found)
}

fn landing_result(
    frames: &[Snapshot],
    step_us: u64,
    event: &Touchdown,
    reference_tick: Option<usize>,
    lane: fn(&Snapshot) -> &ContactSample,
) -> Result<Value> {
    let contact = event.tick;
    let dt = step_us as f64 * 1e-6;
    let count = (WINDOW_US / step_us) as usize;
    let long_count = (50_000 / step_us) as usize;
    if contact == 0
        || contact + count > frames.len()
        || reference_tick.is_some_and(|tick| tick.max(contact) + long_count > frames.len())
    {
        return Err("incomplete incoming sample or fixed landing window".into());
    }
    let first = lane(&frames[contact]);
    let second = lane(&frames[contact + 1]);
    let incoming = lane(&frames[contact - 1]);
    let sum = |tick: usize, steps: usize| {
        frames[tick..tick + steps]
            .iter()
            .map(|f| lane(f).state.normal_impulse_ns)
            .sum::<f64>()
    };
    let balance = |steps: usize| {
        summed_momentum(lane(&frames[contact + steps - 1])) - summed_momentum(incoming)
            + 2. * 9.81 * (steps as f64 * dt)
    };
    let impulse = sum(contact, count);
    // The longer primary window includes native rebounds that fall beyond 5 ms.
    let long_sum = reference_tick.map(|_| sum(contact, long_count));
    let window_residual = impulse - balance(count);
    // Solver normal sums and total vertical momentum agree to the tilt/integration residual;
    // record the residual without using it to identify the contact.
    let closing = incoming.state.pad.velocity[2] - incoming.state.foot.velocity[2];
    let previous_interval = frames[contact - 1].time_s;
    let geometric_gap = incoming.state.vertical_gap_m;
    let predicted_crossing = geometric_gap / closing;
    let patch = |s: &ContactSample| json!({"selected":s.points.len(),"positive":s.points.iter().filter(|p|p.normal_impulse_ns>0.).count(),"points":s.points});
    Ok(json!({
        "tick":contact,"time_s":frames[contact].time_s,"start_s":previous_interval,
        "first_ns":first.state.normal_impulse_ns,"second_ns":second.state.normal_impulse_ns,
        "sum_1ms_ns":sum(contact,(1000/step_us) as usize),"sum_5ms_ns":impulse,
        "reference_clock_5ms_ns":reference_tick.map(|tick|sum(tick,count)),
        "sum_50ms_ns":long_sum, "reference_clock_50ms_ns":reference_tick.map(|tick| sum(tick,long_count)),
        "first_free_tick":event.first_free_tick,"peak_free_gap_m":event.peak_free_gap_m,
        "first_momentum_balance_ns":balance(1),"window_momentum_residual_ns":window_residual,
        "incoming_gap_m":geometric_gap,"incoming_closing_z_m_s":closing,
        "estimated_gap_crossing_fraction":predicted_crossing/dt,
        "incoming":incoming,"first_patch":patch(first),"second_patch":patch(second),
        "window":frames[contact..contact+count].iter().map(|f| {
            let s=lane(f);
            json!({"time_us":f.tick *step_us,"normal_impulse_ns":s.state.normal_impulse_ns,"positive_points":s.points.iter().filter(|p|p.normal_impulse_ns>0.).count()})
        }).collect::<Vec<_>>(),
    }))
}

struct SweepRun {
    cycles: Vec<Value>,
    frames: Vec<Snapshot>,
    initial: Admission,
    pad_max_error_m: f64,
}

fn simulate(step_us: u64, shift_us: i64) -> Result<SweepRun> {
    let mut a = Lane::new_at(step_us)?;
    let mut b = Lane::new_at(step_us)?;
    let dt = a.sim.step_duration().as_secs_f64();
    let mut initial = Admission::default();
    let start_a = ContactSample::read(&a)?;
    let start_b = ContactSample::read(&b)?;
    if start_a.state != start_b.state {
        return Err("initial controls disagree".into());
    }
    request_demotion(&mut a, 0, &mut initial)?;
    let mut frames = vec![Snapshot {
        tick: 0,
        time_s: 0.,
        cycle: 1,
        prescribed: a.tier()? == Prescribed,
        switched: start_a,
        reference: start_b,
    }];
    let mut admission: [Admission; 3] = Default::default();
    let mut last_v = 0.;
    let mut commanded_z = frames[0].switched.state.pad.position[2];
    let mut pad_max_error_m = 0.0_f64;
    for tick in 1..=END_US / step_us {
        let us = tick * step_us;
        let start = us - step_us;
        let (index, v) = command(start, shift_us);
        let stop = stop_us(index, shift_us);
        if v != last_v {
            a.drive(v)?;
            b.drive(v)?;
            last_v = v;
        }
        a.sim.step_exact(1)?;
        b.sim.step_exact(1)?;
        if let Some(event) = support_loss_promotion::last_promotion(&a.sim, a.model)
            && event.iteration == SETTLE_STEPS * COMMAND_US / step_us + tick
        {
            if event.support != a.pad
                || event.link != a.foot
                || event.reason != support_loss_promotion::SupportLossReason::MotionStarted
                || start != DEPART_US[index]
            {
                return Err("unexpected support-loss receipt or timing".into());
            }
            admission[index].promotion_us = Some(us);
        }
        let switched = ContactSample::read(&a)?;
        let reference = ContactSample::read(&b)?;
        commanded_z += dt * v;
        if switched.state.pad != reference.state.pad {
            return Err("unequal commands in native lanes".into());
        }
        pad_max_error_m = pad_max_error_m.max((commanded_z - switched.state.pad.position[2]).abs());
        if a.tier()? == Dynamic {
            if start < DEPART_US[0] {
                request_demotion(&mut a, us, &mut initial)?;
            } else if start >= stop {
                request_demotion(&mut a, us, &mut admission[index])?;
            }
        }
        frames.push(Snapshot {
            tick,
            time_s: tick as f64 * dt,
            cycle: index + 1,
            prescribed: a.tier()? == Prescribed,
            switched,
            reference,
        });
    }
    let mut cycles = vec![];
    for (index, c) in admission.into_iter().enumerate() {
        let depart = DEPART_US[index] as usize / step_us as usize + 1;
        let end = DEPART_US.get(index + 1).copied().unwrap_or(END_US) as usize / step_us as usize;
        let sw_events = locate_landings(&frames, depart, end, |f| &f.switched)?;
        let ref_events = locate_landings(&frames, depart, end, |f| &f.reference)?;
        let s = sw_events[0].tick;
        let r = ref_events[0].tick;
        let switched = landing_result(&frames, step_us, &sw_events[0], Some(r), |f| &f.switched)?;
        let reference =
            landing_result(&frames, step_us, &ref_events[0], Some(r), |f| &f.reference)?;
        if frames[s].switched.state.pad.velocity[2] <= 0.
            || frames[r].reference.state.pad.velocity[2] <= 0.
        {
            return Err("primary landing did not occur on the ascending lift".into());
        }
        let sw_follow = sw_events[1..]
            .iter()
            .map(|e| landing_result(&frames, step_us, e, None, |f| &f.switched))
            .collect::<Result<Vec<_>>>()?;
        let ref_follow = ref_events[1..]
            .iter()
            .map(|e| landing_result(&frames, step_us, e, None, |f| &f.reference))
            .collect::<Result<Vec<_>>>()?;
        let stop = stop_us(index, shift_us);
        let moving = &frames[depart..=stop as usize / step_us as usize];
        // Integrate both lanes on exactly the same clock, including any lift-off
        // and reacquisition after the primary landing, before the first parked step.
        let motion_sum = |lane: fn(&Snapshot) -> &ContactSample| {
            moving
                .iter()
                .map(|f| lane(f).state.normal_impulse_ns)
                .sum::<f64>()
        };
        let switched_sum = switched["sum_5ms_ns"].as_f64().unwrap();
        let reference_sum = reference["sum_5ms_ns"].as_f64().unwrap();
        cycles.push(json!({
            "cycle":index+1,"departure_us":DEPART_US[index],"stop_us":stop_us(index,shift_us),
            "demotion_us":c.demotion_us,"promotion_us":c.promotion_us,"demotion_deferrals":c.demotion_deferrals,"last_deferral":c.last_deferral,
            "switched_landing":switched,"reference_landing":reference,
            "switched_recontacts":sw_follow,"reference_recontacts":ref_follow,"landing_offset_us":(s as i64-r as i64)*step_us as i64,
            "window_difference_ns":switched_sum-reference_sum,
            "motion_interval_impulses_ns":{"switched":motion_sum(|f|&f.switched),"reference":motion_sum(|f|&f.reference)},
        }));
    }
    Ok(SweepRun {
        cycles,
        frames,
        initial,
        pad_max_error_m,
    })
}

fn stop_us(index: usize, shift_us: i64) -> u64 {
    (DEPART_US[index] + STROKES[index].start_park() * COMMAND_US)
        .checked_add_signed(shift_us)
        .unwrap()
}

pub(super) fn write(output: &Path) -> Result<()> {
    fs::create_dir_all(output)?;
    let mut matrix = Vec::new();
    for shift in [0, -500, 500] {
        for step in [500, 250, 125] {
            let run = simulate(step, shift)?;
            if run.pad_max_error_m > 1e-10 {
                return Err("command integration and pad motion diverged".into());
            }
            if shift == 0 {
                let mut selected = std::collections::BTreeSet::new();
                selected.insert(END_US / step);
                for tick in (0..=END_US / step).step_by((10_000 / step) as usize) {
                    selected.insert(tick);
                }
                for c in &run.cycles {
                    let mut events = vec![
                        c["departure_us"].as_u64().unwrap() / step + 1,
                        c["stop_us"].as_u64().unwrap() / step + 1,
                    ];
                    events.extend(c["demotion_us"].as_u64().map(|us| us / step));
                    for tick in events {
                        selected.extend(tick.saturating_sub(1)..=(tick + 1).min(END_US / step));
                    }
                    for name in ["switched_landing", "reference_landing"] {
                        let hit = c[name]["tick"].as_u64().unwrap();
                        selected.extend(hit - 2000 / step..=hit + 5000 / step);
                    }
                    for name in ["switched_recontacts", "reference_recontacts"] {
                        for event in c[name].as_array().unwrap() {
                            let hit = event["tick"].as_u64().unwrap();
                            selected.extend(hit - 2000 / step..=hit + 5000 / step);
                        }
                    }
                }
                let sampled: Vec<_> = selected
                    .into_iter()
                    .map(|tick| &run.frames[tick as usize])
                    .collect();
                let file = fs::File::create(output.join(format!("{step}us.json")))?;
                let mut out = BufWriter::new(file);
                serde_json::to_writer(
                    &mut out,
                    &json!({"step_us":step,"dt_s":step as f64/1e6,
                    "geometry":{"foot_half_extents":FOOT_HALF,"pad_half_extents":PAD_HALF,"upper_half_extents":[0.05,0.06,0.06]},
                    "cycles":run.cycles,"frames":sampled}),
                )?;
                out.flush()?;
            }
            matrix.push(
                json!({"step_us":step,"bottom_dwell_shift_us":shift,"initial_demotion":run.initial,
                "max_commanded_pad_position_error_m":run.pad_max_error_m,"cycles":run.cycles}),
            );
        }
    }
    fs::write(
        output.join("matrix.json"),
        serde_json::to_vec_pretty(
            &json!({"time_origin":"after 1.2 s settle","command_interval_us":COMMAND_US,
        "window_us":WINDOW_US,"end_us":END_US,"runs":matrix}),
        )?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_locked_half_millisecond_run_reproduces_original_and_all_pad_commands() -> Result<()> {
        let (_, original) = run()?;
        let fixed = simulate(500, 0)?;
        assert!(fixed.frames.len() >= original.frames.len());
        for (a, b) in original.frames.iter().zip(&fixed.frames) {
            assert!(
                a.tick == b.tick
                    && a.switched == b.switched.state
                    && a.reference == b.reference.state
                    && a.prescribed == b.prescribed,
                "original and time-locked worlds diverged at tick {}",
                a.tick
            );
        }
        for shift in [-500, 0, 500] {
            for (index, &start) in DEPART_US.iter().enumerate() {
                let s = &STROKES[index];
                let stop = stop_us(index, shift);
                let mut z = 0.;
                let mut minimum = 0.0_f64;
                for t in (start..stop).step_by(500) {
                    let (cycle, v) = command(t, shift);
                    assert_eq!(cycle, index);
                    for delta in [125, 250, 375] {
                        assert_eq!(command(t + delta, shift), (index, v));
                    }
                    z += v * 0.0005;
                    minimum = minimum.min(z);
                }
                assert!((minimum + 1.25 * s.down as f64 * 0.0005).abs() < 1e-12);
                assert!(z.abs() < 1e-12);
                assert_eq!(command(stop, shift).1, 0.);
            }
        }
        Ok(())
    }

    #[test]
    fn quarter_millisecond_measurement_retains_secondary_impacts_and_missed_readmission()
    -> Result<()> {
        let run = simulate(250, 0)?;
        assert!(run.pad_max_error_m < 1e-10);
        assert_eq!(run.cycles.len(), 3);
        for c in &run.cycles {
            for name in ["switched_landing", "reference_landing"] {
                let hit = &c[name];
                let window = hit["window"].as_array().unwrap();
                assert_eq!(window.len(), 20);
                let first = hit["first_ns"].as_f64().unwrap();
                let sum: f64 = hit["first_patch"]["points"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|p| p["normal_impulse_ns"].as_f64().unwrap())
                    .sum();
                assert!((first - sum).abs() < 1e-12);
                assert!(hit["peak_free_gap_m"].as_f64().unwrap() > 0.01);
                let integrated: f64 = window
                    .iter()
                    .map(|s| s["normal_impulse_ns"].as_f64().unwrap())
                    .sum();
                assert!((integrated - hit["sum_5ms_ns"].as_f64().unwrap()).abs() < 1e-12);
                assert!(hit["sum_50ms_ns"].as_f64().unwrap() >= integrated);
            }
            let depart = c["departure_us"].as_u64().unwrap() / 250 + 1;
            let stop = c["stop_us"].as_u64().unwrap() / 250;
            let measured: f64 = run.frames[depart as usize..=stop as usize]
                .iter()
                .map(|f| f.reference.state.normal_impulse_ns)
                .sum();
            assert_eq!(
                measured,
                c["motion_interval_impulses_ns"]["reference"]
                    .as_f64()
                    .unwrap()
            );
        }
        let second = &run.cycles[1];
        assert!(second["demotion_us"].is_null());
        let secondary = &second["switched_recontacts"].as_array().unwrap()[0];
        assert!(
            secondary["first_free_tick"].as_u64().unwrap()
                > second["switched_landing"]["tick"].as_u64().unwrap()
        );
        assert!(secondary["peak_free_gap_m"].as_f64().unwrap() > 1e-5);
        assert!(run.cycles[2]["promotion_us"].is_null());
        Ok(())
    }
}
