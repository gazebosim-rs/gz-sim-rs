// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Carry the deformable edge through the incoming corner impact without a velocity reset.
use super::*;

/// A one-nanometre threshold distinguishes genuine load from the donor plane's
/// rounding error. Each switch records its actual integration-time bracket.
#[derive(Default)]
pub(super) struct EdgeLoadHistory {
    previous: Option<(f64, [bool; 2])>,
    changes: Vec<Value>,
    previous_positive: bool,
    last_release: Option<([f64; 2], State)>,
}
impl EdgeLoadHistory {
    pub(super) fn observe(&mut self, time: f64, overlap: [f64; 2], state: State) {
        let positive = overlap.into_iter().any(|z| z > 0.);
        if self.previous_positive && !positive {
            self.last_release = Some((
                [self.previous.expect("previous load sample").0, time],
                state,
            ));
        }
        self.previous_positive = positive;
        let closed = overlap.map(|z| z > 1e-9);
        if self.previous.is_none_or(|(_, before)| before != closed) {
            self.changes.push(json!({"time_bracket_s":[self.previous.map_or(time,|(t,_)|t),time],"compressed_endpoints":closed}));
        }
        self.previous = Some((time, closed));
    }
    pub(super) fn finish(
        self,
        geometry: &Geometry,
        final_state: State,
        horizon: f64,
    ) -> Result<Value> {
        if self.previous_positive {
            return Err("fixture still carries force at the end".into());
        }
        let (bracket, from) = self.last_release.ok_or("no lift-off during the study")?;
        let dt = horizon - bracket[1];
        let center = from.center + dt * from.velocity + 0.5 * dt * dt * GRAVITY;
        let rotation = (DQuat::from_scaled_axis(dt * from.spin) * from.orientation).normalize();
        let mut acceleration_ceiling = f64::NEG_INFINITY;
        let mut departure_gap = f64::INFINITY;
        let mut horizon_gap = f64::INFINITY;
        let mut agreement = 0.0_f64;
        // During free flight each vertex has z'' <= -g + |ω|² r < 0,
        // so the minimum vertical clearance is attained at a time endpoint.
        // Check the entire material box, including the overhanging part.
        for &arm in geometry.edge_local.iter().chain(&geometry.other_vertices) {
            acceleration_ceiling =
                acceleration_ceiling.max(GRAVITY.z + from.spin.length_squared() * arm.length());
            departure_gap =
                departure_gap.min((from.center + from.orientation * arm).z - geometry.height);
            let terminal = center + rotation * arm;
            horizon_gap = horizon_gap.min(terminal.z - geometry.height);
            agreement = agreement
                .max(terminal.distance(final_state.center + final_state.orientation * arm));
        }
        if !(acceleration_ceiling < 0.
            && departure_gap > 0.
            && horizon_gap > 0.
            && agreement < 1e-10)
        {
            return Err(format!("terminal separation not verified: acceleration {acceleration_ceiling}, departure {departure_gap}, horizon {horizon_gap}, trajectory {agreement}").into());
        }
        Ok(
            json!({"endpoint_compression_threshold_m":1e-9,"transitions":self.changes,
           "last_positive_to_zero_force_bracket_s":bracket,"free_flight_to_s":horizon,
           "min_vertex_departure_gap_m":departure_gap,"min_vertex_horizon_gap_m":horizon_gap,
           "max_vertex_vertical_acceleration_m_s2":acceleration_ceiling,
           "max_ballistic_pose_discrepancy_m":agreement}),
        )
    }
}

fn state(donor: &Value) -> Result<State> {
    let orientation = DQuat::from_array(serde_json::from_value(donor["orientation"].clone())?);
    if !orientation.is_normalized() {
        return Err("unnormalized donor".into());
    }
    Ok(State {
        center: vector(&donor["com"])?,
        velocity: vector(&donor["velocity"])?,
        spin: vector(&donor["omega"])?,
        orientation,
    })
}

fn sample(rows: &[Value], time: f64) -> Result<&Value> {
    rows.iter()
        .find(|r| {
            r["original_time_s"]
                .as_f64()
                .is_some_and(|t| (t - time).abs() < 1e-13)
        })
        .ok_or_else(|| "missing common clock".into())
}

fn clocks(start: f64, line: f64, horizon: f64, extra: &[f64]) -> Vec<f64> {
    let first = (start / CLOCK_S).floor() as usize;
    let mut times: Vec<_> = (first + 1..=(horizon / CLOCK_S).round() as usize)
        .map(|i| i as f64 * CLOCK_S)
        .filter(|t| *t > start + 1e-13)
        .collect();
    times.extend([start, line]);
    times.extend(extra.iter().copied());
    times.sort_by(f64::total_cmp);
    times.dedup_by(|a, b| (*a - *b).abs() < 1e-13);
    times
}

// Both impulses cover the same interval. At a cold start the original control
// already received the rigid impulses accumulated between the two launch times.
fn compare(early: &[Value], late: &[Value], prior_rigid_j: DVec3) -> Result<Vec<Value>> {
    late.iter().map(|b|{
        let t=num(&b["original_time_s"])?;
        let a=sample(early,t)?;
        let dj=vector(&a["support_frame_j_ns"])?-prior_rigid_j-vector(&b["support_frame_j_ns"])?;
        let dc=vector(&a["com_m"])?-vector(&b["com_m"])?;
        Ok(json!({"original_time_s":t,"com_shift_m":dc.to_array(),"com_distance_m":dc.length(),
            "contact_impulse_shift_ns":dj.to_array(),"normal_impulse_shift_ns":dj.z,
            "tangential_impulse_distance_ns":dj.truncate().length(),"impulse_distance_ns":dj.length(),
            "earlier_normal_force_n":num(&a["normal_force_n"])?,"later_normal_force_n":num(&b["normal_force_n"])?,
            "earlier_strain_energy_j":num(&a["strain_energy_j"])?,"later_strain_energy_j":num(&b["strain_energy_j"])?}))
    }).collect()
}

fn checked_run(
    initial: State,
    g: &Geometry,
    layers: Layers,
    times: &[f64],
) -> Result<(Run, Value)> {
    let run =
        |cells, time, display| integrate_at(initial, g, layers, times, cells, time, display, true);
    let fine = run(512, 1024, true)?;
    let a = run(256, 1024, false)?;
    let b = run(512, 512, false)?;
    let space = differences(&a, &fine);
    let time = differences(&b, &fine);
    if num(&space["max_com_delta_m"])? > 5e-9
        || num(&time["max_com_delta_m"])? > 1e-9
        || num(&space["max_impulse_delta_ns"])? > 5e-6
        || num(&time["max_impulse_delta_ns"])? > 1e-6
        || num(&space["max_force_delta_n"])? > 0.002
        || num(&time["max_force_delta_n"])? > 0.002
        || num(&fine.checks["max_frame_energy_balance_j"])? > 1e-9
    {
        return Err(format!(
            "{} failed refinement/energy checks at launch {}: space {space} time {time} energy {}",
            layers.name(),
            times[0],
            fine.checks["max_frame_energy_balance_j"]
        )
        .into());
    }
    Ok((fine, json!({"spatial":space,"temporal":time})))
}

pub(in super::super) fn study(
    reference: &Path,
    corner_donor: &Path,
    late_material: &Path,
    output: &Path,
) -> Result<()> {
    let doc = read(reference)?;
    let corner = read(corner_donor)?;
    let late = read(late_material)?;
    if [&doc, &late]
        .iter()
        .any(|d| d["case"] != 80 || d["stroke"] != 2)
        || corner["case"] != 80
        || corner["stroke"] != 2
    {
        return Err("fixture or stroke differs".into());
    }
    let corner = &corner["fine"];
    let corner_s = num(&corner["elapsed"])?;
    let incoming = state(corner)?;
    let refs = array(&doc["references"])?;
    let last = refs.last().ok_or("missing rigid line")?;
    let line = &last["fine"];
    let geometry = Geometry::from_reference(&doc, line, &last["unresolved_line"], state(line)?)?;
    let line_s = num(&line["elapsed"])?;
    let horizon = num(&late["horizon_s"])?;
    if !(corner_s < line_s && line_s < horizon && (line_s - num(&late["start_s"])?).abs() < 1e-13) {
        return Err("invalid restart clocks".into());
    }
    let events = array(&line["events"])?;
    let arrival = events
        .iter()
        .find(|e| {
            e["feature"] == "SupportFace(2)"
                && e["at_s"]
                    .as_f64()
                    .is_some_and(|t| (t - corner_s).abs() < 1e-9)
        })
        .ok_or("unmatched corner event")?;
    let seg = geometry.segment(incoming)?;
    let end = incoming.center + seg.start_arm + seg.hi * seg.delta;
    if end.distance(vector(&arrival["point"])?) > 1e-8 {
        return Err("corner donor does not land on right ridge endpoint".into());
    }
    let profile_clocks = clocks(corner_s, line_s, horizon, &[]);
    let rigid_before_line = vector(&line["j"])? - vector(&corner["j"])?;
    let mut profiles = Vec::new();
    let mut sensitivity = Value::Null;
    for layers in [Layers::Uniform, Layers::StifferDeck, Layers::GradedFoot] {
        let (early, checks) = checked_run(incoming, &geometry, layers, &profile_clocks)?;
        let old = array(&late["profiles"])?
            .iter()
            .find(|p| p["name"] == layers.name())
            .ok_or("missing late-start material")?;
        let late_rows = array(&old["frames"])?;
        let line_frame = sample(&early.frames, line_s)?;
        let differences = compare(&early.frames, late_rows, rigid_before_line)?;
        let (foot, deck) = layers.layers();
        if old["foot_endpoint_stiffness_n_m2"] != json!(foot)
            || old["deck_stiffness_n_m2"] != json!(deck)
        {
            return Err("different material parameters at the two starts".into());
        }
        if matches!(layers, Layers::Uniform) {
            let pre = refs
                .iter()
                .rev()
                .find(|r| {
                    r["qualified"] == true
                        && r["fine"]["elapsed"].as_f64().is_some_and(|t| t < corner_s)
                })
                .ok_or("no preceding public clock")?;
            let pre = &pre["fine"];
            let pre_s = num(&pre["elapsed"])?;
            let old_j = vector(&corner["j"])? - vector(&pre["j"])?;
            let sensitivity_clocks = clocks(pre_s, line_s, horizon, &[corner_s]);
            let (preloaded, convergence) =
                checked_run(state(pre)?, &geometry, layers, &sensitivity_clocks)?;
            sensitivity = json!({"name":"Uniform layers, preceding public clock","start_s":pre_s,"rigid_impulse_until_corner_ns":old_j.to_array(),
                "cornertime_state":sample(&preloaded.frames,corner_s)?,
                "comparison_to_corner_start":compare(&preloaded.frames,&early.frames,old_j)?,
                "checks":preloaded.checks,"mesh":convergence,"frames":preloaded.frames});
        }
        profiles.push(json!({"name":layers.name(),"foot_endpoint_stiffness_n_m2":foot,"deck_stiffness_n_m2":deck,
            "checks":early.checks,"mesh":checks,"force_at_old_line_n":line_frame["normal_force_n"],
            "energy_at_old_line_j":line_frame["strain_energy_j"],"comparison_to_late_start":differences,"frames":early.frames}));
    }
    fs::create_dir_all(output)?;
    write(
        &output.join("reference.json"),
        &json!({"case":80,"stroke":2,"start_s":corner_s,
        "rigid_line_time_s":line_s,"horizon_s":horizon,"public_step_s":CLOCK_S,"rigid_impulse_corner_to_line_ns":rigid_before_line.to_array(),
        "coordinate_frame":late["coordinate_frame"],"material_model":late["material_model"],"calibration":late["calibration"],
        "initial_condition":"Start at the limiting rigid pose before the corner impact, without applying its impulse. Previous skew-edge load has no prescribed precompression.",
        "earlier_start_control":sensitivity,"profiles":profiles}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn equal_zero_force_at_samples_does_not_establish_ballistic_clearance() -> Result<()> {
        let geometry = Geometry {
            edge_local: [DVec3::X, -DVec3::X],
            other_vertices: vec![DVec3::Z],
            foot_local: DVec3::ZERO,
            deck_center: DVec3::ZERO,
            deck_half: DVec3::splat(2.),
            height: 0.,
        };
        let state = State {
            center: DVec3::Z * 0.001,
            velocity: DVec3::Z,
            orientation: DQuat::IDENTITY,
            spin: DVec3::ZERO,
        };
        let history = || {
            let mut events = EdgeLoadHistory::default();
            events.observe(0., [0.0001, 0.], state);
            events.observe(1e-4, [-0.001, -0.001], state);
            events
        };
        let dt = 0.1;
        let end = State {
            center: state.center + dt * state.velocity + 0.5 * dt * dt * GRAVITY,
            ..state
        };
        let result = history().finish(&geometry, end, 1e-4 + dt)?;
        assert!(num(&result["min_vertex_horizon_gap_m"])? > 0.);
        // Over this longer flight gravity would bring the ridge back through the deck.
        assert!(history().finish(&geometry, end, 1.).is_err());
        Ok(())
    }
}
