// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Cross measured incoming second-landing states with the native continuation grid.
use super::*;

#[path = "secondary_contact_points.rs"]
pub(crate) mod points;
use gz_physics_multifidelity::LandingRestartState;

const SOURCES: [(Profile, ProjectionSpan); 3] = [
    (PATHS[0], ProjectionSpan::Native),
    (
        p("finite8_then512", b(8, 1), b(512, 1)),
        ProjectionSpan::FirstTick,
    ),
    (PATHS[6], ProjectionSpan::Native),
];
const CLOCK_NS: u64 = STEP_US * 1000;
fn load(file: &Path) -> Result<Value> {
    Ok(serde_json::from_reader(fs::File::open(file)?)?)
}
fn motion(s: &LandingRestartState) -> Value {
    json!({"links":s.links.map(Body::from),"generalized_velocity":s.generalized_velocity,
        "reduced_joint_coordinates":s.joint_coordinates,"hinge_q":s.hinge.position,"hinge_qdot":s.hinge.velocity,
        "motor_enabled":s.motor_enabled,"pending_hinge_torque_nm":s.pending_hinge_effort,"gravity":s.gravity,
        "user_forces_n":s.user_forces,"user_torques_nm":s.user_torques,"last_native_dt_s":s.last_native_dt_s})
}
fn cache(s: &LandingRestartState) -> Value {
    json!(s.history.each_ref().map(
        |h| json!({"pair_present":h.pair_present,"manifolds":h.manifolds,
       "native_points":h.native_points,"solver_points":h.solver_points,
       "absolute_normal_ns":h.absolute_seeds[0],"absolute_tangent_ns":h.absolute_seeds[1],
       "absolute_torsion_nms":h.absolute_seeds[2],"discard_pending":h.discard_pending})
    ))
}
fn scalar(v: &Value) -> Result<f64> {
    v.as_f64().ok_or_else(|| "missing measured scalar".into())
}
fn vec3(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(v.clone())?))
}

struct Cut {
    tick: u64,
    source: Tick,
    ids: [BodyId; 3],
    control: MultiFidelityBackend,
    warm_swapped: MultiFidelityBackend,
    cold: [MultiFidelityBackend; 2],
    state: Value,
}
fn capture(native: &MultiFidelityBackend, ids: [BodyId; 3], row: Tick, vz: f64) -> Result<Cut> {
    let [foot, _, pad] = ids;
    let s = native.airborne_restart_state(foot, pad)?;
    let expected = [row.foot, row.upper, row.pad];
    if expected != s.links.map(Body::from)
        || row.normal != 0.
        || row.extra != 0.
        || row.tangent != DVec3::ZERO
        || row.gap.is_none_or(|v| v <= 0.)
        || row.pad.velocity != [0., 0., vz]
        || row.pad.angular_velocity != [0.; 3]
        || s.gravity != [0., 0., -9.81]
        || s.hinge.position != 0.
        || s.hinge.velocity != 0.
        || s.motor_enabled
        || s.pending_hinge_effort.is_some()
        || s.user_forces != [[0.; 3]; 3]
        || s.user_torques != [[0.; 3]; 3]
        || s.history
            .iter()
            .any(|h| h.absolute_seeds != [0.; 3] || h.discard_pending)
    {
        return Err(format!("invalid unloaded, passive native cut at tick {}", row.tick).into());
    }
    let control = native.copy_airborne_landing(foot, pad, false)?;
    if control.airborne_restart_state(foot, pad)? != s {
        return Err("warm restart changed native cut".into());
    }
    let cold = [
        native.copy_airborne_landing(foot, pad, true)?,
        native.copy_airborne_landing(foot, pad, true)?,
    ];
    let mut cold_cache = Value::Null;
    for fork in &cold {
        let got = fork.airborne_restart_state(foot, pad)?;
        if motion(&got) != motion(&s)
            || got
                .history
                .iter()
                .any(|h| h.pair_present || h.native_points != 0 || h.solver_points != 0)
        {
            return Err("cold restart changed motion or retained narrow-phase state".into());
        }
        cold_cache = cache(&got);
    }
    let state = json!({"source":motion(&s),"source_cache":cache(&s),"cold_cache":cold_cache,
        "pair_order":["foot_pad","upper_pad","foot_upper"], "source_gap_m":row.gap,"held_pad_vz_m_s":vz});
    Ok(Cut {
        tick: row.tick,
        source: row,
        ids,
        control,
        warm_swapped: native.copy_airborne_landing(foot, pad, false)?,
        cold,
        state,
    })
}
fn continuation(
    native: &mut MultiFidelityBackend,
    ids: [BodyId; 3],
    source: Tick,
    last: u64,
    n: u32,
    f: Fixture,
) -> Result<Vec<Tick>> {
    let mut run = vec![source];
    let mut speed = source.pad.velocity[2];
    let mut impacts = vec![];
    for tick in source.tick + 1..=last {
        let next = command(f, tick - 1);
        let frame = propagate_one(
            native,
            ids,
            tick,
            b(n, 1),
            (next != speed).then_some(next),
            None,
            &mut impacts,
        )?;
        if frame.pad.velocity != [0., 0., next]
            || frame.pad.angular_velocity != [0.; 3]
            || frame.extra != 0.
        {
            return Err(format!("unexpected support command or other load at {tick}").into());
        }
        run.push(frame);
        speed = next;
    }
    if !impacts.is_empty() {
        return Err("unrequested projection on native suffix".into());
    }
    Ok(run)
}
fn live_rows(trace: &[Tick], cut: u64) -> Vec<Tick> {
    trace.iter().filter(|t| t.tick >= cut).copied().collect()
}
fn twin(a: &[Tick], b: &[Tick]) -> Result<Value> {
    let mut first = None;
    let mut count = 0;
    let mut first_time_mismatch = None;
    let mut maxima = [0.0_f64; 7];
    if a.len() != b.len() {
        return Err("unequal control lengths".into());
    }
    for (x, y) in a.iter().zip(b) {
        if x.tick != y.tick || x.calls != y.calls {
            return Err("unequal control clocks".into());
        }
        if x.foot != y.foot
            || x.upper != y.upper
            || x.pad != y.pad
            || x.normal != y.normal
            || x.tangent != y.tangent
            || x.extra != y.extra
            || x.first != y.first
            || x.gap != y.gap
        {
            first.get_or_insert(x.tick);
            count += 1;
        }
        if x.first != y.first {
            first_time_mismatch.get_or_insert(x.tick);
        }
        for (i, j) in [(x.foot, y.foot), (x.upper, y.upper), (x.pad, y.pad)] {
            let quat_a = DQuat::from_array(i.orientation);
            let quat_b = DQuat::from_array(j.orientation);
            let rot = (quat_a * quat_b.conjugate())
                .normalize()
                .to_scaled_axis()
                .length();
            for (k, v) in [
                DVec3::from(i.position).distance(j.position.into()),
                rot,
                DVec3::from(i.velocity).distance(j.velocity.into()),
                DVec3::from(i.angular_velocity).distance(j.angular_velocity.into()),
            ]
            .into_iter()
            .enumerate()
            {
                maxima[k] = maxima[k].max(v);
            }
        }
        maxima[4] = maxima[4].max((x.normal - y.normal).abs());
        maxima[5] = maxima[5].max(x.tangent.distance(y.tangent));
        maxima[6] = maxima[6].max(x.com().distance(y.com()));
    }
    Ok(
        json!({"public_samples":a.len(),"exact_differences":count,"first_difference_tick":first,"first_load_time_mismatch_tick":first_time_mismatch,
      "maximum_link_position_m":maxima[0],"maximum_link_rotation_rad":maxima[1],"maximum_link_velocity_m_s":maxima[2],"maximum_link_spin_rad_s":maxima[3],
      "maximum_normal_error_ns":maxima[4],"maximum_tangent_error_ns":maxima[5],"maximum_com_error_m":maxima[6]}),
    )
}
fn verify_parent(t: &Tick, row: &Value, lane: usize) -> Result<()> {
    if row["tick"] != t.tick
        || t.com().distance(vec3(&row["com_m"][lane])?) > 1e-12
        || t.tangent.distance(vec3(&row["tangent_ns"][lane])?) > 1e-12
        || (t.normal - scalar(&row["normal_ns"][lane])?).abs() > 1e-12
        || DVec3::from(t.foot.angular_velocity).distance(vec3(&row["spin_rad_s"][lane])?) > 1e-11
        || row["first_local_ns"][lane] != json!(t.first.map(|v| v.as_nanos() as u64))
    {
        return Err(format!(
            "uninterrupted path mismatches archive at {} on lane {lane}",
            t.tick
        )
        .into());
    }
    Ok(())
}
pub(crate) fn run(dir: &Path, index: usize) -> Result<()> {
    if index >= COUNT {
        return Err("invalid donor index".into());
    }
    fs::create_dir_all(dir)?;
    let root = dir
        .parent()
        .ok_or("output requires sibling source archives")?;
    let tracking = load(&root.join(format!(
        "multifidelity-landing-persistence/case-{index:03}/tracking.json"
    )))?;
    let archive = load(&root.join(format!(
        "multifidelity-primary-propagation/cases/case-{index:03}.json"
    )))?;
    let f = fixture(index);
    let (cuts, _) = survey::cuts(f)?;
    let primary = cuts[0].ok_or("missing primary cut")?;
    if tracking["case"] != index
        || tracking["cut_tick"] != primary.tick
        || tracking["label"] != json!(SOURCES.map(|(p, _)| p.label))
    {
        return Err("unexpected source history or primary donor".into());
    }
    let frames = tracking["frames"]
        .as_array()
        .ok_or("missing original tracking")?;
    let source_start = primary.tick + 1;
    let source_row = |tick: u64| -> Result<&Value> {
        frames
            .get((tick - source_start) as usize)
            .ok_or_else(|| "archive ends before study".into())
    };
    let second = archive["result"]["paths"][0]["outcomes"]["strokes"][1]["first_native_end_ns"]
        .as_u64()
        .ok_or("second return censored")?
        .div_ceil(CLOCK_NS);
    let before = second - 1;
    let last = before + 180;
    let last_airborne_gap = |minimum: f64| -> Result<u64> {
        let second_stroke = sweep::DEPART_US[1] / STEP_US;
        (second_stroke + 1..before)
            .rev()
            .find(|&tick| {
                command(f, tick - 1) > 0.
                    && source_row(tick).ok().is_some_and(|r| {
                        r["projected_gap_m"].as_array().is_some_and(|v| {
                            v.len() == 3
                                && v.iter().all(|x| x.as_f64().is_some_and(|d| d >= minimum))
                        }) && r["normal_ns"] == json!([0., 0., 0.])
                    })
            })
            .ok_or_else(|| {
                format!("no common unloaded rising-pad cut at {minimum} m of clearance").into()
            })
    };
    let wide = last_airborne_gap(0.002)?;
    let far = last_airborne_gap(0.015)?;
    if source_row(second)?["first_local_ns"]
        .as_array()
        .is_none_or(|a| a.iter().any(Value::is_null))
    {
        return Err("three second arrivals do not share the same public tick".into());
    }
    let mut audits = vec![];
    for (lane, (profile, scope)) in SOURCES.into_iter().enumerate() {
        let mut saved = vec![];
        let (live, _) = observe_path_until(f, &primary, profile, scope, last, |n, ids, t| {
            verify_parent(t, source_row(t.tick)?, lane)?;
            if [far, wide, before].contains(&t.tick) {
                saved.push(capture(n, ids, *t, command(f, t.tick - 1))?);
            }
            Ok(())
        })?;
        for Cut {
            tick,
            source,
            ids,
            mut control,
            mut warm_swapped,
            mut cold,
            state,
        } in saved
        {
            let expected = live_rows(&live, tick);
            let last_loaded = live
                .iter()
                .rev()
                .find(|t| t.tick < tick && t.normal > 0.)
                .ok_or("missing last loaded native receipt")?;
            let unloaded: Vec<_> = live
                .iter()
                .filter(|t| t.tick > last_loaded.tick && t.tick <= tick)
                .collect();
            if unloaded.iter().any(|t| {
                t.normal != 0. || t.extra != 0. || t.tangent != DVec3::ZERO || t.first.is_some()
            }) {
                return Err("previous loading has not cleared".into());
            }
            let outputs = dir.join(format!("case-{index:03}/{}/{tick}", profile.label));
            fs::create_dir_all(outputs.join("traces"))?;
            let warm = continuation(&mut control, ids, source, last, profile.suffix.n, f)?;
            let warm_audit = twin(&expected, &warm)?;
            if warm_audit["exact_differences"] != 0 {
                return Err(format!("warm fork failed at {tick}: {warm_audit}").into());
            }
            let swapped_budget = if profile.suffix.n == 512 { 4096 } else { 512 };
            let switched = continuation(&mut warm_swapped, ids, source, last, swapped_budget, f)?;
            let geometry = json!({"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF});
            write_json(
                &outputs.join("traces/warm_swapped.json"),
                &json!({"geometry":geometry,
                "frames":switched.iter().map(Tick::as_json).collect::<Vec<_>>() }),
            )?;
            let mut lanes = vec![];
            let mut cold_same = Value::Null;
            for (label, name, run) in [
                (
                    "uninterrupted",
                    "Uninterrupted native continuation",
                    expected,
                ),
                (
                    "cold512",
                    "Cold collision graph, 512 calls",
                    continuation(&mut cold[0], ids, source, last, 512, f)?,
                ),
                (
                    "cold4096",
                    "Cold collision graph, 4,096 calls",
                    continuation(&mut cold[1], ids, source, last, 4096, f)?,
                ),
            ] {
                if label == format!("cold{}", profile.suffix.n) {
                    cold_same = twin(&warm, &run)?;
                }
                let file = format!("traces/{label}.json");
                let frames: Vec<_> = run.iter().map(Tick::as_json).collect();
                write_json(
                    &outputs.join(&file),
                    &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
                )?;
                lanes.push(json!({"label":label,"name":name,"file":file}));
            }
            let schedule: Vec<_> = (tick + 1..=last)
                .map(|k| json!([k, command(f, k - 1)]))
                .collect();
            let audit = json!({"profile":profile.label,"original_suffix":profile.suffix.n,"cut_tick":tick,"last_tick":last,"first_load_tick":second,"snapshot":state,
              "last_positive_receipt":last_loaded.as_json(),"unloaded_public_intervals":unloaded.len(),"warm_twin":warm_audit,"cold_same_grid":cold_same,"public_vz_schedule":schedule});
            write_json(&outputs.join("audit.json"), &audit)?;
            audits.push(audit);
            write_json(
                &outputs.join("comparison.json"),
                &json!({"title":"Second landing from an independently restarted native contact graph","pair_totals_only":true,
             "capture_ticks":{"second_impact":second,"at_47_ms":before+94,"at_74_ms":before+148,"at_90_ms":last},"lanes":lanes}),
            )?;
            eprintln!(
                "{} cut {} warm/cold differing endpoints 0/{}",
                profile.label, tick, cold_same["exact_differences"]
            );
        }
    }
    write_json(
        &dir.join(format!("case-{index:03}/restart.json")),
        &json!({"format":"matched-secondary-restart-v2","case":index,"primary_tick":primary.tick,
      "far_cut":far,"wide_cut":wide,"near_cut":before,"second_tick":second,"last_tick":last,"fixture":f,"controls":audits}),
    )
}
