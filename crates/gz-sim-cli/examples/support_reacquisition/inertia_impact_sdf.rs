// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Author and load ordinary Bevy scenes from independent spinning-impact initial conditions.
use super::*;
use gz_sim_scene::primary_contact::{
    last_box_flight, last_box_timing, last_refinement, set_refined_observer,
};
use std::sync::{Arc, Mutex};
const PLUGIN: &str = r#"<plugin name="gz::sim::systems::SwitchableArticulation" filename="gz-sim-switchable-articulation-system"><exact_revolute_locks>true</exact_revolute_locks></plugin>"#;
const HINGE: &str = r#"<joint name="hinge" type="revolute"><parent>foot</parent><child>upper</child><axis><xyz>1 0 0</xyz><limit><lower>0</lower><upper>0</upper></limit></axis></joint>"#;
pub(super) fn coords(x: DVec3) -> String {
    format!("{} {} {}", x.x, x.y, x.z)
}
pub(super) fn rpy(q: DQuat) -> String {
    let (yaw, pitch, roll) = q.to_euler(glam::EulerRot::ZYX);
    format!("{roll} {pitch} {yaw}")
}
fn pose(b: Body) -> String {
    format!(
        "{} {}",
        coords(b.position.into()),
        rpy(DQuat::from_array(b.orientation))
    )
}
pub(super) fn tensor(m: DMat3, center: DVec3, q: DQuat, mass: f64) -> String {
    format!(
        "<inertial><pose>{} {}</pose><mass>{mass}</mass><inertia><ixx>{}</ixx><iyy>{}</iyy><izz>{}</izz><ixy>{}</ixy><ixz>{}</ixz><iyz>{}</iyz></inertia></inertial>",
        coords(center),
        rpy(q),
        m.x_axis.x,
        m.y_axis.y,
        m.z_axis.z,
        m.y_axis.x,
        m.z_axis.x,
        m.z_axis.y
    )
}
fn link(name: &str, half: [f64; 3], offset: DVec3, inertia: &str) -> String {
    let size = coords(2. * DVec3::from(half));
    let geometry = format!("<geometry><box><size>{size}</size></box></geometry>");
    format!(
        r#"<link name="{name}"><pose>{} 0 0 0</pose>{inertia}<collision name="contact">{geometry}<surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision><visual name="visual">{geometry}</visual></link>"#,
        coords(offset)
    )
}
pub(super) fn xml(r: &Physical, tree: bool, profile: Profile) -> String {
    let s = r.state(0.);
    let (joint, links) = if tree {
        let m = tensor(
            DMat3::from_diagonal(r.body.member),
            DVec3::ZERO,
            r.body.principal_frame,
            1.,
        );
        (
            format!("{PLUGIN}{HINGE}"),
            link("foot", FOOT_HALF, DVec3::ZERO, &m) + &link("upper", UPPER_HALF, 2. * ARM, &m),
        )
    } else {
        (
            String::new(),
            link(
                "foot",
                FOOT_HALF,
                DVec3::ZERO,
                &tensor(r.body.tensor, ARM, DQuat::IDENTITY, M),
            ),
        )
    };
    let transform = pose(s.foot);
    let linear = coords(r.pre.v - r.gravity * r.arrival);
    let gravity = coords(r.gravity);
    let angular = coords(s.foot.angular_velocity.into());
    let moving = format!(
        r#"<model name="rocker"><pose>{transform}</pose><plugin name="gz::sim::systems::InitialVelocity" filename="gz-sim-initial-velocity-system"><linear_velocity>{linear}</linear_velocity><angular_velocity>{angular}</angular_velocity></plugin>{links}{joint}</model>"#
    );
    let support = format!(
        r#"<model name="lift"><pose>{}</pose><plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>{}</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>{}</model>"#,
        pose(s.pad),
        coords(r.pad_v),
        link("pad", PAD_HALF, DVec3::ZERO, "")
    );
    let models = if r.id.is_multiple_of(2) {
        moving + &support
    } else {
        support + &moving
    };
    let live = profile.prospective;
    let calls = profile.calls;
    format!(
        r#"<?xml version="1.0"?>
<sdf version="1.12"><world name="default"><gravity>{gravity}</gravity>
<physics name="fixed" type="ignored"><max_step_size>0.0005</max_step_size></physics>
<plugin name="gz::sim::systems::PrimaryContactRefinement" filename="gz-sim-primary-contact-refinement-system">
  <native_steps>{calls}</native_steps><solver_iterations>4</solver_iterations><pgs_iterations>1</pgs_iterations><relaxation_iterations>16</relaxation_iterations>
  <pair><foot>default::rocker::foot</foot><support>default::lift::pad</support><refresh_box_features>true</refresh_box_features><prospective_timing>{live}</prospective_timing></pair></plugin>
{models}</world></sdf>
"#
    )
}
pub(super) fn angles(a: [f64; 4], b: [f64; 4]) -> f64 {
    angular_distance(DQuat::from_array(a), DQuat::from_array(b))
}
fn verify_start(actual: State, expected: State) -> Result<()> {
    for (a, b) in [
        (actual.foot, expected.foot),
        (actual.upper, expected.upper),
        (actual.pad, expected.pad),
    ] {
        if DVec3::from(a.position).distance(b.position.into()) > 5e-12
            || DVec3::from(a.velocity).distance(b.velocity.into()) > 1e-10
            || DVec3::from(a.angular_velocity).distance(b.angular_velocity.into()) > 1e-10
            || angles(a.orientation, b.orientation) > 1e-11
        {
            return Err(format!(
                "loaded spinning fixture initial mismatch p {:e} v {:e} w {:e} q {:e}",
                DVec3::from(a.position).distance(b.position.into()),
                DVec3::from(a.velocity).distance(b.velocity.into()),
                DVec3::from(a.angular_velocity).distance(b.angular_velocity.into()),
                angles(a.orientation, b.orientation)
            )
            .into());
        }
    }
    Ok(())
}
pub(super) fn run(
    r: &Physical,
    tree: bool,
    p: Profile,
    text: &str,
) -> Result<(Value, Vec<Snapshot>, State)> {
    let (mut sim, ids) = sdf_features::load_finite_box(text, tree)?;
    let s0 = state(backend(&sim), ids)?;
    verify_start(s0, r.state(0.))?;
    let data = Arc::new(Mutex::new(Observed::default()));
    let shared = data.clone();
    set_refined_observer(&mut sim, move |_, s| shared.lock().unwrap().record(s, ids))?;
    let mut frames = vec![frame(0, s0, 0, false, [0.; 4])];
    let mut ticks = vec![];
    let mut first = None;
    let mut j = DVec3::ZERO;
    let mut calls = 0;
    let mut accepted = 0;
    let mut declines = 0;
    let mut retimed = 0;
    let mut guarded = 0;
    let mut pre_impulse = 0.;
    let mut free_l = 0.0_f64;
    let mut free_e = 0.0_f64;
    let mut free_q = 0.0_f64;
    let mut free_center = 0.0_f64;
    let mut free_speed = 0.0_f64;
    let mut rel_q = 0.0_f64;
    let mut rel_w = 0.0_f64;
    let (l0, e0) = r.body.surface(s0);
    for tick in 1..=8 {
        sim.step_exact(1)?;
        let t = sim.elapsed();
        let primary = last_refinement(&sim).filter(|x| x.sim_time == t);
        let live = last_box_timing(&sim).filter(|x| x.sim_time == t);
        let receipt = live
            .map(|x| &x.interval)
            .or_else(|| primary.map(|x| &x.interval));
        let o = std::mem::take(&mut *data.lock().unwrap());
        let mut count = 1;
        let (corners, vector) = if let Some(c) = receipt {
            count = c.native_steps;
            if o.elapsed.len() != count as usize || o.elapsed.last() != Some(&500000) {
                return Err("missing selected Euler run observation".into());
            }
            let reaction = c
                .pairs
                .iter()
                .find(|x| (x.a, x.b) == (ids.foot, ids.pad) || (x.b, x.a) == (ids.foot, ids.pad));
            if (o.corners.iter().sum::<f64>() - reaction.map_or(0., |x| x.normal_impulse)).abs()
                > 1e-10
            {
                return Err("corner sum differs from production reaction".into());
            }
            (
                o.corners,
                reaction.map_or(DVec3::ZERO, |x| {
                    DVec3::from(x.normal_impulse_on_a) * if x.a == ids.foot { 1. } else { -1. }
                }),
            )
        } else {
            assert!(o.elapsed.is_empty());
            sdf::unrefined(backend(&sim), ids)
        };
        if corners.iter().sum::<f64>() > 0. {
            first.get_or_insert((tick - 1) * 500000 + o.first_load.unwrap_or(500000));
        }
        j += vector;
        calls += count;
        let s = state(backend(&sim), ids)?;
        let reference = r.state(t.as_secs_f64());
        if t.as_secs_f64() < r.arrival && first.is_none() {
            let (l, e) = r.body.surface(s);
            free_l = free_l.max(l.distance(l0));
            let gravity_work = M * r.gravity.dot(s.com() - s0.com());
            free_e = free_e.max((e - e0 - gravity_work).abs());
            free_center = free_center.max(s.com().distance(reference.com()));
            free_speed = free_speed.max(s.velocity().distance(reference.velocity()));
            free_q = free_q.max(angles(s.foot.orientation, reference.foot.orientation));
        }
        rel_q = rel_q.max(angles(s.foot.orientation, s.upper.orientation));
        rel_w = rel_w
            .max(DVec3::from(s.foot.angular_velocity).distance(s.upper.angular_velocity.into()));
        let separation = last_box_flight(&sim).filter(|v| v.sim_time == t);
        let mut ends = o.elapsed.clone();
        let mut loads = o.normal_by_end;
        let mut native_gaps = o.minimum_gap_by_end;

        if ends.is_empty() {
            ends.push(500000);
            loads.push(corners.iter().sum());
            native_gaps.push(lowest_gap(
                backend(&sim).state(ids.foot).unwrap(),
                backend(&sim).state(ids.pad).unwrap(),
            ));
        }
        if let Some(g) = separation {
            let mut elapsed = 0;
            for ((&protected, &duration), (&end, &load)) in g
                .flight
                .guarded
                .iter()
                .zip(&g.flight.durations)
                .zip(ends.iter().zip(&loads))
            {
                elapsed += duration.as_nanos() as u64;
                if elapsed != end || protected && load != 0. {
                    return Err("guarded native boundary or delivered reaction mismatch".into());
                }
                guarded += usize::from(protected);
            }
            if ends.len() != g.flight.guarded.len() {
                return Err("guard receipt missing native call".into());
            }
        }
        for (&end, &load) in ends.iter().zip(&loads) {
            if (tick - 1) * 500000 + end < (r.arrival * 1e9).round() as u64 {
                pre_impulse += load;
            }
        }
        let timing = live.map(|x| &x.timing);
        if let Some(clock) = timing {
            if !clock.bounded_rotation {
                return Err("non-spherical test did not use inertia bounds".into());
            }
            if clock.decline.is_some() {
                declines += 1;
            } else {
                accepted += 1;
            }
            retimed += clock.retimed;
        }
        ticks.push(json!({"tick":tick,"calls":count,"guarded":separation.map(|g|&g.flight.guarded),"native_end_ns":ends,"native_normal_ns":loads,"native_end_gap_m":native_gaps,"rotation_slices":separation.and_then(|g|g.flight.rotation_slices.as_ref()),"end_ns":o.elapsed,"first_positive_end_ns":o.first_load,"primary":primary.is_some(),"retimed":timing.map_or(0,|x|x.retimed),"decline":timing.and_then(|x|x.decline).map(|x|format!("{x:?}")),"decline_after":timing.and_then(|x|x.decline_after),"gap_bound_nm":timing.map_or(0,|x|x.max_gap_uncertainty_nm),"time_bound_ns":timing.map_or(0,|x|x.max_time_uncertainty_ns),"j":vector.to_array()}));
        if let Some(ballistic) = separation.and_then(|g| g.flight.ballistic_steps.as_ref()) {
            assert_eq!(ballistic.len(), count as usize);
            ticks.last_mut().unwrap()["ballistic"] = json!(ballistic);
        }
        if let Some(mark) = timing.and_then(|t| t.compact) {
            ticks.last_mut().unwrap()["compact"] = compact_record(mark);
        }

        frames.push(frame(
            tick,
            s,
            count,
            timing.is_some_and(|t| t.retimed > 0),
            corners,
        ));
    }
    let s = frames.last().unwrap().state;
    let (l, e) = r.body.surface(s);
    let exact = r.state(WINDOW);
    let work = j.dot(r.pad_v);
    let expected_loss = r.closing * r.j / 2.;
    let gravity_work = M * r.gravity.dot(s.com() - s0.com());
    let mut metrics = json!({"impulse_vector_error_ns":j.distance(r.n*r.j),"com_error_m":s.com().distance(exact.com()),"orientation_error_rad":angles(s.foot.orientation,exact.foot.orientation),"spin_error_rad_s":DVec3::from(s.foot.angular_velocity).distance(exact.foot.angular_velocity.into()),"angular_momentum_error_nms":l.distance(r.post.l),"linear_impulse_closure_ns":((s.velocity()-s0.velocity()-r.gravity*WINDOW)*M-j).length(),"kinetic_change_j":e-e0,"pad_work_j":work,"loss_after_pad_work_j":e0+work+gravity_work-e,"dissipation_error_j":(e0+work+gravity_work-e-expected_loss).abs(),"free_angular_momentum_drift_nms":free_l,"free_energy_drift_j":free_e,"free_orientation_drift_rad":free_q,"max_link_relative_rotation_rad":rel_q,"max_link_spin_difference_rad_s":rel_w});
    if r.gravity != DVec3::ZERO {
        metrics["gravity_work_j"] = json!(gravity_work);
        metrics["free_com_error_m"] = json!(free_center);
        metrics["free_velocity_error_m_s"] = json!(free_speed);
    }
    Ok((
        json!({"locked":tree,"profile":p,"native_calls":calls,"accepted_ticks":accepted,"declined_ticks":declines,"retimed":retimed,"guarded_calls":guarded,"preimpact_measured_ns":pre_impulse,"first_positive_end_ns":first,"metrics":metrics,"ticks":ticks,"end":s}),
        frames,
        s,
    ))
}
pub(super) fn summary(r: &Physical, c: &Certificate) -> Value {
    let s = r.state(0.);
    let frozen = DQuat::from_scaled_axis(DVec3::from(s.foot.angular_velocity) * r.arrival)
        * DQuat::from_array(s.foot.orientation);
    json!({"id":r.id,"certificate":c,"member_principal_moments":r.body.member.to_array(),"member_frame":r.body.principal_frame.to_array(),"aggregate_tensor_columns":r.body.tensor.to_cols_array(),"arrival_s":r.arrival,"normal":r.n.to_array(),"impulse_ns":r.j,"corner":r.corner,"closing_speed_m_s":r.closing,"initial":s,"exact_at":r.state(r.arrival),"exact_end":r.state(WINDOW),"incoming_L":r.pre.l.to_array(),"outgoing_L":r.post.l.to_array(),"frozen_incoming_orientation_error_rad":angles(frozen.into(),r.q.into())})
}
pub(super) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut rows = vec![];
    let mut view = None;
    for id in 0..CASES {
        let r = Physical::new(id);
        let proof = r.certify();
        let mut row = summary(&r, &proof);
        if proof.reason.is_none() {
            let mut cases = vec![];
            for tree in [false, true] {
                for p in PROFILES {
                    let text = xml(&r, tree, p);
                    let (trial, frames, _) = run(&r, tree, p, &text)?;
                    if tree
                        && p.calls == 8
                        && (view == Some(id)
                            || view.is_none()
                                && r.body.omega(r.q, r.pre.l).length() > 5.
                                && r.body.omega(r.q, r.pre.l).length() < 15.
                                && id % 4 == 2)
                    {
                        view = Some(id);
                        let name = if p.prospective {
                            "prospective"
                        } else {
                            "uniform"
                        };
                        trace(path, name, &frames)?;
                        fs::write(path.join(format!("{name}.sdf")), &text)?;
                    }
                    cases.push(trial);
                }
            }
            row["trials"] = json!(cases);
            if view == Some(id) {
                trace(path, "analytic", &r.trace())?;
                write_json(
                    &path.join("comparison.json"),
                    &json!({"title":"Triaxial spinning foot on a moving support","lanes":[{"label":"uniform","name":"Uniform authored impact","file":"traces/uniform.json"},{"label":"prospective","name":"Torque-free bounded timing","file":"traces/prospective.json"},{"label":"analytic","name":"Euler flight and plastic impact","file":"traces/analytic.json","analytical":true}]}),
                )?;
            }
        }
        rows.push(row);
    }
    write_json(
        &path.join("inertia.json"),
        &json!({"view_id":view,"cases":rows}),
    )
}
pub(super) fn measure(text: &str, tree: bool, end: State) -> Result<[u64; 8]> {
    let (mut sim, ids) = sdf_features::load_finite_box(text, tree)?;
    let mut samples = [0; 8];
    for s in &mut samples {
        let now = Instant::now();
        sim.step_exact(1)?;
        *s = now.elapsed().as_nanos() as u64;
    }
    if state(backend(&sim), ids)? != end {
        return Err("clocked SDF endpoint differs from observed run".into());
    }
    Ok(samples)
}
pub(super) fn cost(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    for id in 0..CASES {
        let r = Physical::new(id);
        if r.certify().reason.is_some() {
            continue;
        }
        for tree in [false, true] {
            let mut fixtures = vec![];
            for p in PROFILES {
                let text = xml(&r, tree, p);
                let end = run(&r, tree, p, &text)?.2;
                measure(&text, tree, end)?;
                fixtures.push((text, end));
            }
            let mut samples: [Vec<[u64; 8]>; 4] = Default::default();
            for rep in 0usize..8 {
                for pos in 0..4 {
                    let mut i = (rep + pos) % 4;
                    if (rep / 4 + id) % 2 == 1 {
                        i = 3 - i;
                    }
                    samples[i].push(measure(&fixtures[i].0, tree, fixtures[i].1)?);
                }
            }
            for (i, v) in samples.into_iter().enumerate() {
                rows.push(json!({"id":id,"locked":tree,"profile":PROFILES[i],"primary_tick":(r.arrival*2000.).ceil() as usize,"tick_ns":v}));
            }
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":rows}))
}

pub(super) fn compact_record(m: gz_physics_multifidelity::CompactImpactReceipt) -> Value {
    json!({"approach":m.approach_planned,"pulse":m.pulse_planned,"pulse_loaded":m.pulse_loaded,"tail_checks":m.tail_checks,"complete":m.complete,"decline":m.decline.map(|s|format!("{s:?}")),"tail_gap_pm":m.tail_gap_pm})
}
