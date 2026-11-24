// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Interleaved timing of unobserved native suffix steps, checked against archived observer histories.
use super::*;
use std::{collections::BTreeMap, time::Instant};

const REPEATS: usize = 16;
const OFFSETS: [u16; 5] = [175, 200, 225, 250, 275];

#[derive(Serialize)]
struct Timing {
    suffix_passes: u32,
    tick_ns: Vec<u64>,
    loaded: Vec<bool>,
}
fn check(raw: &Value, actual: f64) -> Result<()> {
    let observed = raw.as_f64().ok_or("missing measured reference component")?;
    if (observed - actual).abs() > 1e-11 || !actual.is_finite() {
        return Err(format!(
            "ordinary interval differs from archived observed result: {observed} vs {actual}"
        )
        .into());
    }
    Ok(())
}
fn run(f: Fixture, cut: &ReboundFrame, passes: u32, saved: &Value) -> Result<Timing> {
    let mut r = restart(f, ADAPTIVE, cut)?;
    let lane = &mut r.case.lane;
    if lane.tier()? != Dynamic {
        return Err("non-dynamic timing donor".into());
    }
    let (foot, pad, top) = (lane.foot, lane.pad, lane.upper);
    let mut resource = lane.sim.world_mut().resource_mut::<PhysicsBackend>();
    let engine = resource
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("missing local backend")?;
    let mut speed = r.last_speed;
    let mut sum_j = 0.;
    let mut tick_ns = Vec::with_capacity(HORIZON as usize);
    let mut loaded = Vec::with_capacity(HORIZON as usize);
    let mut num_checked = 0;
    for step in 1..=HORIZON {
        let tick = cut.tick + step;
        let next = command(f, tick - 1);
        if speed != next {
            engine.set_world_velocity(pad, [0., 0., next], [0.; 3])?;
            speed = next;
        }
        let settings = ContactSolverIterations {
            temporal: 4,
            pgs: None,
            relaxation: Some(if step == 1 { 8 } else { passes as usize }),
        };
        let start = Instant::now();
        let receipt = engine.step_contact_interval(DT, 8, settings)?;
        let elapsed = start.elapsed();
        if receipt.native_steps != 8 {
            return Err("native budget drift during timed suffix".into());
        }
        tick_ns.push(elapsed.as_nanos().try_into()?);
        let impulse = receipt
            .pairs
            .iter()
            .find(|p| (p.a, p.b) == (foot.min(pad), foot.max(pad)))
            .map_or(0., |p| p.normal_impulse);
        loaded.push(impulse > 0.);
        sum_j += impulse;
        if let Some(s) = saved["samples"]
            .as_array()
            .ok_or("missing samples")?
            .iter()
            .find(|v| v["i"] == step)
        {
            num_checked += 1;
            let a = engine.state(foot).ok_or("missing foot")?;
            let b = engine.state(top).ok_or("missing upper")?;
            let pos = (DVec3::from(a.position) + DVec3::from(b.position)) * 0.5;
            let vel = (DVec3::from(a.velocity) + DVec3::from(b.velocity)) * 0.5;
            for (field, values) in [
                ("com_m", &pos.to_array()[..]),
                ("velocity_m_s", &vel.to_array()),
                ("spin_rad_s", &a.angular_velocity),
                ("orientation_xyzw", &a.orientation),
            ] {
                for (i, &v) in values.iter().enumerate() {
                    check(&s[field][i], v)?;
                }
            }
            check(&s["normal_ns"], impulse)?;
            check(&s["cumulative_normal_ns"], sum_j)?;
        }
    }
    if num_checked != 4 {
        return Err("expected four archived state samples".into());
    }
    Ok(Timing {
        suffix_passes: passes,
        tick_ns,
        loaded,
    })
}

pub(crate) fn write(source: &Path, out: &Path) -> Result<()> {
    let mut fixtures = BTreeMap::new();
    for index in 0..1250 {
        let f = survey::fixture(index);
        let offset = (1000. * f.offset_m).round() as u16;
        if OFFSETS.contains(&offset) {
            fixtures.entry((offset, f.switched)).or_insert(index);
        }
    }
    if fixtures.len() != 10 {
        return Err("missing offset/history timing stratum".into());
    }
    fs::create_dir_all(out)?;
    let mut archive = BufWriter::new(fs::File::create(out.join("suffix-cost.jsonl"))?);
    let mut total = 0;
    let mut absent = vec![];
    for (&(offset, holds), &index) in &fixtures {
        let file = source.join("cases").join(format!("case-{index:04}.json"));
        let case: Value = serde_json::from_reader(fs::File::open(file)?)?;
        let f = survey::fixture(index);
        if case["index"] != index
            || case["format"] != FORMAT
            || case["result"]["fixture"] != serde_json::to_value(f)?
        {
            return Err("invalid held-out timing fixture".into());
        }
        let (cuts, _) = survey::cuts(f)?;
        let trials = case["result"]["trials"]
            .as_array()
            .ok_or("missing trials")?;
        for (pos, cut) in cuts.iter().enumerate() {
            let stroke = pos + 1;
            let record = trials.iter().find(|v| v["stroke"] == stroke);
            let Some(cut) = cut else {
                if record.is_some() {
                    return Err("archived donor no longer reproduced".into());
                }
                absent.push([index, stroke]);
                continue;
            };
            let record = record.ok_or("missing archived timing donor")?;
            if record["cut"]["tick"] != cut.tick {
                return Err("timing donor cut tick differs".into());
            }
            let paths = record["paths"].as_array().ok_or("missing treatments")?;
            let treatment = |passes: u32| -> Result<&Value> {
                paths
                    .iter()
                    .find(|v| {
                        v["profile"]["calls"] == 8
                            && v["profile"]["relaxation"] == 8
                            && v["profile"]["onset_only"] == (passes == 1)
                    })
                    .ok_or_else(|| "missing native timing profile".into())
            };
            for passes in [1, 8] {
                run(f, cut, passes, treatment(passes)?)?;
            }
            for repeat in 0..REPEATS {
                let order = if (index + stroke + repeat).is_multiple_of(2) {
                    [1, 8]
                } else {
                    [8, 1]
                };
                let pair = [
                    run(f, cut, order[0], treatment(order[0])?)?,
                    run(f, cut, order[1], treatment(order[1])?)?,
                ];
                serde_json::to_writer(
                    &mut archive,
                    &json!({"index":index,"offset_mm":offset,"holds":holds,"drive_percent":(f.speed_scale*100.).round() as u32,"stroke":stroke,"cut_tick":cut.tick,"repeat":repeat,"runs":pair}),
                )?;
                archive.write_all(b"\n")?;
                total += 1;
            }
        }
    }
    archive.flush()?;
    write_json(
        &out.join("design.json"),
        &json!({"first_fixture_for_each_offset_and_history":fixtures.values().collect::<Vec<_>>(),"held_out_offsets_mm":OFFSETS,
        "repetitions":REPEATS,"matched_pairs":total,"unavailable_strokes":absent,"native_calls_per_500us":8,"temporal_iterations":4,
        "timing":"monotonic elapsed wall time around ordinary step_contact_interval only; no substep observation, no state checks or command updates inside timer",
        "validation":"first, second, tenth and twenty-sixth public states and normal impulse receipts compared to stored observed treatments on every repeat",
        "order":"one discarded warmup per candidate and donor, then alternating paired AB/BA on fresh identical donor states",
        "unit":"nanoseconds; index 0 is the common eight-pass first tick, indices 1..25 are the differing suffix"}),
    )
}
