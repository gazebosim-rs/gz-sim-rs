// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fixed spans and a one-way contact-receipt release at the ordinary public cadence.
use super::*;

const FORMAT: &str = "primary-continuation-span-v1";
// Two one-kilogram links on the vertical lift under Earth gravity.
const WEIGHT_IMPULSE: f64 = 2.0 * 9.81 * DT.as_secs_f64();
const SPANS: [Option<u64>; 6] = [Some(1), Some(2), Some(4), Some(8), Some(HORIZON), None];

fn run(
    f: Fixture,
    cut: &ReboundFrame,
    span: Option<u64>,
    mut frames: Option<&mut Vec<Value>>,
    mut timings: Option<&mut Vec<u64>>,
) -> Result<Value> {
    let mut rig = restart(f, ADAPTIVE, cut)?;
    let lane = &mut rig.case.lane;
    if lane.tier()? != Dynamic {
        return Err("span requires a dynamic touchdown donor".into());
    }
    let (foot, top, pad) = (lane.foot, lane.upper, lane.pad);
    let mut backend = lane.sim.world_mut().resource_mut::<PhysicsBackend>();
    let engine = backend
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("missing local engine")?;
    let mut speed = rig.last_speed;
    let mut relaxed = true;
    let mut rich_ticks = 0;
    let mut normals = Vec::with_capacity(HORIZON as usize);
    let mut samples = Vec::new();
    let mut first_end = None;
    let mut first_disjoint = None;
    let mut total = 0.;
    if let Some(ref mut out) = frames {
        let mut initial = serde_json::to_value(cut.with_effort())?;
        initial
            .as_object_mut()
            .ok_or("invalid cut frame")?
            .remove("corner_ns");
        out.push(initial);
    }
    for step in 1..=HORIZON {
        let tick = cut.tick + step;
        let next = command(f, tick - 1);
        if next != speed {
            engine.set_world_velocity(pad, [0., 0., next], [0.; 3])?;
            speed = next;
        }
        let passes = if relaxed { 8 } else { 1 };
        rich_ticks += i32::from(relaxed);
        let start = timings.as_ref().map(|_| std::time::Instant::now());
        let receipt = engine.step_contact_interval(
            DT,
            8,
            ContactSolverIterations {
                temporal: 4,
                pgs: None,
                relaxation: Some(passes),
            },
        )?;
        if let (Some(clock), Some(times)) = (start, timings.as_mut()) {
            times.push(clock.elapsed().as_nanos().try_into()?);
        }
        if receipt.native_steps != 8 {
            return Err("unexpected native call budget in span treatment".into());
        }
        let pair = receipt
            .pairs
            .iter()
            .find(|p| (p.a, p.b) == (foot.min(pad), foot.max(pad)));
        let impulse = pair.map_or(0., |p| p.normal_impulse);
        if !impulse.is_finite() || impulse < 0. {
            return Err("invalid native pair reaction".into());
        }
        let local_end = pair.and_then(|p| p.first_positive_end);
        if first_end.is_none()
            && let Some(end) = local_end
        {
            first_end = Some((step - 1) * STEP_US * 1000 + u64::try_from(end.as_nanos())?);
        }
        normals.push(impulse);
        total += impulse;
        {
            let foot = engine.state(foot).ok_or("missing foot state")?;
            let upper = engine.state(top).ok_or("missing upper state")?;
            let deck = engine.state(pad).ok_or("missing deck state")?;
            let gap = supported_gap(foot, deck);
            if gap.is_none() {
                first_disjoint.get_or_insert(tick);
            }
            if matches!(step, 1 | 2 | 10 | HORIZON) {
                let position = (DVec3::from(foot.position) + DVec3::from(upper.position)) * 0.5;
                let velocity = (DVec3::from(foot.velocity) + DVec3::from(upper.velocity)) * 0.5;
                samples.push(
                    json!({"i":step,"com_m":position.to_array(),"velocity_m_s":velocity.to_array(),
                    "orientation_xyzw":foot.orientation,"spin_rad_s":foot.angular_velocity,
                    "normal_ns":impulse,"cumulative_normal_ns":total,"footprint_gap_m":gap}),
                );
            }
            if let Some(ref mut out) = frames {
                out.push(json!({"tick":tick,"time_us":tick*STEP_US,"foot":Body::from(foot),"upper":Body::from(upper),"pad":Body::from(deck),
                    "normal_impulse_ns":impulse,"gap_m":gap,"native_steps":8,"solver_iterations":4,"relaxation_passes":passes,"selected":true}));
            }
        }
        // Only the completed public pair receipt affects the NEXT interval.
        relaxed &= span.map_or(impulse >= WEIGHT_IMPULSE, |n| step < n);
    }
    Ok(
        json!({"span":span,"weight_threshold_ns":WEIGHT_IMPULSE,"eight_pass_ticks":rich_ticks,
        "normal_ns":normals,"first_disjoint_tick":first_disjoint,"first_positive_end_ns":first_end,"samples":samples}),
    )
}

pub(crate) fn survey(out: &Path, first: usize, count: usize) -> Result<()> {
    survey::write_cases(out, first, count, FORMAT, |f| {
        let (cuts, disjoint) = survey::cuts(f)?;
        let mut trials = vec![];
        for (i, cut) in cuts.iter().enumerate() {
            if let Some(cut) = cut {
                let paths = SPANS
                    .iter()
                    .map(|&s| run(f, cut, s, None, None))
                    .collect::<Result<Vec<_>>>()?;
                let shared = &paths[0]["samples"][0];
                if paths.iter().any(|p| &p["samples"][0] != shared) {
                    return Err("span treatments disagree before intervention".into());
                }
                trials.push(json!({"stroke":i+1,"cut":snapshot(cut),"paths":paths}));
            }
        }
        Ok(json!({"fixture":f,"first_disjoint_tick":disjoint,"trials":trials}))
    })
}

pub(crate) fn replay(dir: &Path, index: usize, stroke: usize) -> Result<()> {
    let (f, cut, paths) = survey::archived_cut(dir, index, stroke, FORMAT)?;
    let out = dir.join(format!("case-{index:04}-stroke-{stroke}"));
    fs::create_dir_all(out.join("traces"))?;
    let mut lanes = vec![];
    let mut ticks = json!({"shared_receipt":cut.tick+1});
    for (label, name, span) in [
        ("onset", "Eight passes on the landing interval", Some(1)),
        ("weight", "Release after less than one weight impulse", None),
        ("full", "Eight passes throughout", Some(HORIZON)),
    ] {
        let mut frames = vec![];
        let path = run(f, &cut, span, Some(&mut frames), None)?;
        let serialized: Value = serde_json::from_slice(&serde_json::to_vec(&path)?)?;
        if !paths.contains(&serialized) {
            return Err("ordinary span replay differs from archived path".into());
        }
        if span.is_none() {
            let n = path["eight_pass_ticks"]
                .as_u64()
                .ok_or("missing span budget")?;
            if n < HORIZON {
                ticks["first_released_interval"] = json!(cut.tick + n + 1);
            }
        }
        let file = format!("traces/{label}.json");
        write_json(
            &out.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"file":file,"label":label,"name":name}));
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"title":format!("When to release postcontact relaxation · fixture {index}, stroke {stroke}"),"pair_totals_only":true,"lanes":lanes,"capture_ticks":ticks}),
    )
}

/// Warm each native branch once. Rotate/reverse the order so each span occupies every position twice.
pub(crate) fn cost(source: &Path, out: &Path) -> Result<()> {
    use std::{collections::BTreeMap, io::Write as _};
    let mut donors = BTreeMap::new();
    for index in 0..1250 {
        let f = survey::fixture(index);
        let offset = (1000. * f.offset_m).round() as u16;
        if [175, 200, 225, 250, 275].contains(&offset) {
            donors.entry((offset, f.switched)).or_insert(index);
        }
    }
    fs::create_dir_all(out)?;
    let mut file = BufWriter::new(fs::File::create(out.join("span-cost.jsonl"))?);
    for &index in donors.values() {
        let saved: Value = serde_json::from_reader(fs::File::open(
            source.join("cases").join(format!("case-{index:04}.json")),
        )?)?;
        if saved["format"] != FORMAT || saved["index"] != index {
            return Err("wrong span timing case".into());
        }
        let f = survey::fixture(index);
        let (cuts, _) = survey::cuts(f)?;
        for (position, cut) in cuts.iter().enumerate() {
            let stroke = position + 1;
            let trial = saved["result"]["trials"]
                .as_array()
                .ok_or("missing span trial")?
                .iter()
                .find(|s| s["stroke"] == stroke);
            let Some(cut) = cut else {
                if trial.is_some() {
                    return Err("missing timed donor".into());
                }
                continue;
            };
            let trial = trial.ok_or("missing archived span timing donor")?;
            if trial["cut"]["tick"] != cut.tick {
                return Err("span timing incoming clock differs".into());
            }
            for repeat in 0..13 {
                for pos in 0..6 {
                    let k = if repeat == 0 {
                        pos
                    } else {
                        let slot = (index + stroke + (repeat - 1) % 6 + pos) % 6;
                        if repeat > 6 { 5 - slot } else { slot }
                    };
                    let mut times = Vec::with_capacity(HORIZON as usize);
                    let path = run(f, cut, SPANS[k], None, Some(&mut times))?;
                    let encoded: Value = serde_json::from_slice(&serde_json::to_vec(&path)?)?;
                    if encoded != trial["paths"][k] || times.len() != 26 {
                        return Err("timed path differs from archived complete native state".into());
                    }
                    if repeat > 0 {
                        serde_json::to_writer(
                            &mut file,
                            &json!({"index":index,"stroke":stroke,"repeat":repeat,"position":pos,
                            "policy":k,"eight_pass_ticks":path["eight_pass_ticks"],"tick_ns":times}),
                        )?;
                        file.write_all(b"\n")?;
                    }
                }
            }
            file.flush()?;
        }
    }
    write_json(
        &out.join("design.json"),
        &json!({"format":"native-continuation-span-cost-v1","fixtures":donors.values().collect::<Vec<_>>(),"held_offsets_mm":[175,200,225,250,275],
        "native_calls_per_tick":8,"public_ticks":26,"measured_repeats":12,"warm_repeats":1,"order":"six forward rotations then six reversed rotations per donor"}),
    )
}
