// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

const MS: u64 = 1_000_000;

#[derive(Debug, PartialEq, Serialize, serde::Deserialize)]
struct Sample {
    com_m: [f64; 3],
    momentum_ns: [f64; 3],
    outside_nodes: usize,
    lowest_skin_m: f64,
    top_edge_y_m: f64,
    deck_y_m: f64,
    deck_impulse_ns: [f64; 3],
    deck_moment_nms: [f64; 3],
    jaws_impulse_ns: [f64; 3],
    peak_speed_m_s: f64,
}

fn measure(
    r: &mut Recording,
    shift: u32,
    frames: &mut Option<Vec<arm_sheet::Frame>>,
) -> Result<(Vec<Reading>, Vec<Sample>)> {
    let bench = entity_id(&r.sim, "default::source::body")?;
    let half = arm_cable::half_size(&r.sdf, "source", "body")?;
    let mut trace = Vec::new();
    let mut previous_center = DVec3::from(state(&r.sim, bench)?.position);
    let mut interval_origin = previous_center;
    let (mut receiver_impulse, mut receiver_moment, mut jaw_impulse) =
        (DVec3::ZERO, DVec3::ZERO, DVec3::ZERO);
    let rows = episode_native(r, Mode::Finite, frames, |r, _, _| {
        let t = r.sim.committed_update_info()?.sim_time.as_nanos() as u64;
        if t == 1500 * MS || t == 1900 * MS {
            let vy = if t == 1500 * MS {
                f64::from(shift) / 400.
            } else {
                0.
            };
            gz_sim_scene::queue_link_world_velocity(&mut r.sim, bench, [0., vy, 0.], [0.; 3])?;
        }
        let cloth = arm_sheet::cloth(&r.sim)?;
        let [a, b, deck] = cloth.last_boundary_impulses() else {
            return Err("missing measured boundaries".into());
        };
        receiver_impulse -= DVec3::from(deck.linear);
        // A native reaction is about the receiver's start-of-step center.
        // Express the separate commits about the same origin as the public wrench.
        receiver_moment -= DVec3::from(deck.angular)
            + (previous_center - interval_origin).cross(deck.linear.into());
        previous_center = DVec3::from(state(&r.sim, bench)?.position);
        jaw_impulse -= DVec3::from(a.linear) + DVec3::from(b.linear);
        if t.is_multiple_of(MS) {
            let center = previous_center;
            interval_origin = center;
            let edge = center.y - half.y;
            let p = cloth.particles();
            let mut com = DVec3::ZERO;
            let mut momentum = DVec3::ZERO;
            let mut mass = 0.;
            let mut outside = 0;
            let mut low = f64::INFINITY;
            let mut peak = 0_f64;
            for node in p {
                let m = node.inverse_mass.recip();
                mass += m;
                com += m * DVec3::from(node.position);
                momentum += m * DVec3::from(node.velocity);
                outside += usize::from(node.position[1] < edge);
                low = low.min(node.position[2] - arm_sheet::RADIUS - center.z - half.z);
                peak = peak.max(DVec3::from(node.velocity).length());
            }
            trace.push(Sample {
                com_m: (com / mass).to_array(),
                momentum_ns: momentum.to_array(),
                outside_nodes: outside,
                lowest_skin_m: low,
                top_edge_y_m: edge,
                deck_y_m: center.y,
                deck_impulse_ns: receiver_impulse.to_array(),
                deck_moment_nms: receiver_moment.to_array(),
                jaws_impulse_ns: jaw_impulse.to_array(),
                peak_speed_m_s: peak,
            });
            (receiver_impulse, receiver_moment, jaw_impulse) =
                (DVec3::ZERO, DVec3::ZERO, DVec3::ZERO);
        }
        Ok(())
    })?;
    Ok((rows, trace))
}

pub(crate) fn run(
    out: &Path,
    shift: u32,
    exchanges: usize,
    public: bool,
    reset: bool,
) -> Result<()> {
    if !(1..=1000).contains(&exchanges)
        || !1_000_000usize.is_multiple_of(exchanges)
        || shift > 200
        || (public && exchanges > 256)
    {
        return Err(
            "choose a 0–200 mm offset and an exchange count dividing 1 ms (at most 256 internally)"
                .into(),
        );
    }
    let xml = world(exchanges, true, public)?;
    let mut r = Recording::new(&xml, "sheet-edge.sdf", "default")?;
    let mut frames = Some(Vec::new());
    let (rows, trace) = measure(&mut r, shift, &mut frames).map_err(|e| {
        format!(
            "after {} commits: {}",
            r.costs_ns.len(),
            gz_sim_cli::with_fault(&r.sim, e)
        )
    })?;
    if reset {
        gz_sim_scene::reset_loaded_world(&mut r.sim, |_| ())?;
        let (b, s) = measure(&mut r, shift, &mut None)?;
        if rows != b || trace != s {
            return Err("reset changed the sheet edge trajectory".into());
        }
        r.costs_ns
            .truncate(if public { END } else { END * exchanges });
    }
    let half_z = arm_cable::half_size(&r.sdf, "source", "body")?.z;
    let summary = summarize(
        &rows,
        &trace,
        shift,
        exchanges,
        half_z,
        r.sim.step_duration().as_nanos() as u64,
        reset,
    )?;
    write_json(
        &out.join("comparison.json"),
        &serde_json::to_value(&summary)?,
    )?;
    write_json(&out.join("ticks.json"), &serde_json::to_value(&rows)?)?;
    write_json(&out.join("edge.json"), &serde_json::to_value(&trace)?)?;
    fs::write(out.join("world.sdf"), xml)?;
    let mut captures = json!({"lift":60,"deck_stopped":76,"lowering":100,"end":172});
    if let Some(tick) = summary.pad_free_tick {
        captures["released"] = json!(tick.div_ceil(25));
    }
    if summary.outcome == Outcome::Fell
        && let Some(tick) = summary.last_table_contact_tick
    {
        captures["edge_departure"] = json!((tick + 1).div_ceil(25));
    }
    arm_sheet::save_archive_with_camera(r, frames.unwrap(), out, captures, "front")
}

#[derive(PartialEq, Debug, Serialize, serde::Deserialize)]
enum Outcome {
    Resting,
    Fell,
    Held,
    Unsettled,
}
#[derive(Serialize, serde::Deserialize)]
struct Summary {
    actuator_ns: u64,
    #[serde(default)]
    bevy_step_ns: u64,
    #[serde(default)]
    reset_verified: bool,
    deck_shift_mm: u32,
    exchanges: usize,
    outcome: Outcome,
    first_opening_tick: Option<usize>,
    pad_free_tick: Option<usize>,
    last_table_contact_tick: Option<usize>,
    com_inside_at_release_m: Option<f64>,
    overhanging_nodes_at_release: Option<usize>,
    final_overhanging_nodes: usize,
    final_com_inside_m: f64,
    final_lowest_skin_m: f64,
    final_peak_speed_m_s: f64,
    mean_final_table_n: f64,
    mean_final_jaws_n: f64,
    peak_simultaneous_support_n: f64,
    minimum_table_impulse_ns: f64,
    max_linear_balance_residual_ns: f64,
    free_gravity_residual_m_s2: Option<f64>,
    released_line_of_action_inside_m: Option<[f64; 2]>,
}
fn summarize(
    rows: &[Reading],
    trace: &[Sample],
    shift: u32,
    exchanges: usize,
    half_z: f64,
    bevy_step_ns: u64,
    reset_verified: bool,
) -> Result<Summary> {
    if rows.len() != END || trace.len() != END {
        return Err("a complete 4.3 s public robot trajectory is required".into());
    }
    let mass = arm_sheet::NODE_MASS * arm_sheet::COUNT as f64;
    let gravity = DVec3::new(0., 0., -9.81);
    let final_state = &trace[END - 1];
    if (final_state.deck_y_m - 0.001 * f64::from(shift)).abs() > 1e-8 {
        return Err("receiver did not complete its commanded translation".into());
    }
    let mut previous = DVec3::ZERO;
    let mut residual = 0_f64;
    for sample in trace {
        let p = DVec3::from(sample.momentum_ns);
        residual = residual.max(
            (p - previous
                - gravity * mass * 0.001
                - DVec3::from(sample.deck_impulse_ns)
                - DVec3::from(sample.jaws_impulse_ns))
            .length(),
        );
        previous = p;
    }
    if residual > 1e-11 {
        return Err(
            "measured material momentum disagrees with gravity and both owners' impulses".into(),
        );
    }
    let tail = &trace[END - 100..];
    let mean = |f: fn(&Sample) -> f64| tail.iter().map(f).sum::<f64>() / 0.1;
    let table = mean(|s| s.deck_impulse_ns[2]);
    let jaws = mean(|s| s.jaws_impulse_ns[2]);
    let free = rows
        .iter()
        .position(|r| r.opening_ms > 0 && r.grip_n.iter().sum::<f64>() < 1e-9);
    let no_external = tail.iter().all(|s| {
        DVec3::from(s.jaws_impulse_ns).length() < 1e-12
            && DVec3::from(s.deck_impulse_ns).length() < 1e-12
    });
    let ballistic = no_external.then(|| {
        ((DVec3::from(final_state.momentum_ns) - DVec3::from(trace[END - 101].momentum_ns))
            / (mass * 0.1)
            - gravity)
            .length()
    });
    let peak = tail.iter().map(|s| s.peak_speed_m_s).fold(0., f64::max);
    let supports_weight = |load: f64| (load - WEIGHT).abs() < 0.02 * WEIGHT;
    let outcome =
        if free.is_some() && peak < 0.001 && supports_weight(table) && jaws.abs() < 0.01 * WEIGHT {
            Outcome::Resting
        } else if free.is_some()
            && final_state.outside_nodes == arm_sheet::COUNT
            && final_state.lowest_skin_m < -0.02
            && ballistic.is_some_and(|a| a < 1e-7)
        {
            Outcome::Fell
        } else if rows[END - 1].grip_n.iter().sum::<f64>() > 0.1 && supports_weight(jaws + table) {
            Outcome::Held
        } else {
            Outcome::Unsettled
        };
    let mut cop: Option<[f64; 2]> = None;
    for (r, s) in rows.iter().zip(trace) {
        // Intersect the cumulative table wrench with the top plane. The receiver
        // has stopped before opening. This includes the moment from tangential load.
        if r.opening_ms > 0
            && r.grip_n.iter().sum::<f64>() < 1e-9
            && s.deck_impulse_ns[2] > 0.25 * WEIGHT * 0.001
        {
            let y = s.deck_y_m - s.top_edge_y_m
                + (s.deck_moment_nms[0] + half_z * s.deck_impulse_ns[1]) / s.deck_impulse_ns[2];
            let r = cop.get_or_insert([y, y]);
            r[0] = r[0].min(y);
            r[1] = r[1].max(y);
        }
    }
    let margin = |s: &Sample| s.com_m[1] - s.top_edge_y_m;
    Ok(Summary {
        actuator_ns: MS,
        bevy_step_ns,
        reset_verified,
        deck_shift_mm: shift,
        exchanges,
        outcome,
        first_opening_tick: rows.iter().find(|r| r.opening_ms > 0).map(|r| r.tick),
        pad_free_tick: free.map(|i| i + 1),
        last_table_contact_tick: rows
            .iter()
            .rfind(|r| r.tick > 2500 && r.table_n > 1e-6)
            .map(|r| r.tick),
        com_inside_at_release_m: free.map(|i| margin(&trace[i])),
        overhanging_nodes_at_release: free.map(|i| trace[i].outside_nodes),
        final_overhanging_nodes: final_state.outside_nodes,
        final_com_inside_m: margin(final_state),
        final_lowest_skin_m: final_state.lowest_skin_m,
        final_peak_speed_m_s: peak,
        mean_final_table_n: table,
        mean_final_jaws_n: jaws,
        peak_simultaneous_support_n: rows[2500..]
            .iter()
            .map(|r| r.jaws_up_n.min(r.table_n))
            .fold(0., f64::max),
        minimum_table_impulse_ns: trace
            .iter()
            .map(|s| s.deck_impulse_ns[2])
            .fold(0., f64::min),
        max_linear_balance_residual_ns: residual,
        free_gravity_residual_m_s2: ballistic,
        released_line_of_action_inside_m: cop,
    })
}

pub(crate) fn compare(recording: &Path, reference: &Path) -> Result<Value> {
    let read = |dir: &Path| -> Result<(Summary, Vec<Sample>)> {
        let summary = serde_json::from_slice(&fs::read(dir.join("comparison.json"))?)?;
        let rows = serde_json::from_slice(&fs::read(dir.join("edge.json"))?)?;
        Ok((summary, rows))
    };
    let (a, coarse) = read(recording)?;
    let (b, fine) = read(reference)?;
    if a.deck_shift_mm != b.deck_shift_mm
        || a.actuator_ns != b.actuator_ns
        || a.exchanges >= b.exchanges
        || coarse.len() != END
        || fine.len() != END
        || [a.pad_free_tick, b.pad_free_tick]
            .into_iter()
            .flatten()
            .any(|n| !(1..=END).contains(&n))
    {
        return Err("comparison requires complete matching decks and a finer reference on the same actuator clock".into());
    }
    let distances = |x: &[Sample], y: &[Sample]| {
        x.iter()
            .zip(y)
            .map(|(a, b)| DVec3::from(a.com_m).distance(DVec3::from(b.com_m)))
            .fold(0., f64::max)
    };
    let released = a
        .pad_free_tick
        .zip(b.pad_free_tick)
        .map(|(x, y)| (&coarse[x - 1..], &fine[y - 1..]));
    Ok(json!({
        "offset_mm": a.deck_shift_mm,
        "exchanges": [a.exchanges, b.exchanges],
        "outcomes": [a.outcome, b.outcome],
        "pad_free_ticks": [a.pad_free_tick, b.pad_free_tick],
        "last_table_contact_ticks": [a.last_table_contact_tick, b.last_table_contact_tick],
        "max_public_com_error_m": distances(&coarse, &fine),
        "max_release_aligned_com_error_m": released.map(|(x,y)| distances(x,y)),
        "max_first_100ms_after_pad_unloading_com_error_m": released.map(|(x,y)| {
            let n = x.len().min(y.len()).min(101);
            distances(&x[..n], &y[..n])
        }),
        "final_com_error_m": DVec3::from(coarse[END-1].com_m).distance(fine[END-1].com_m.into()),
        "final_table_force_difference_n": a.mean_final_table_n - b.mean_final_table_n,
        "final_jaw_force_difference_n": a.mean_final_jaws_n - b.mean_final_jaws_n
    }))
}

#[cfg(test)]
#[path = "edge_tests.rs"]
mod tests;
