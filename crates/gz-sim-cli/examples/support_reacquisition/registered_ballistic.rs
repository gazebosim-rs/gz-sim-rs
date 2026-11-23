// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently bracket fractional ballistic arrivals from the preceding live boundary.
use super::*;
#[derive(Clone, Copy, Serialize, Default)]
struct ResultRow {
    loaded_tick: Option<u64>,
    first_report_ns: Option<u64>,
    projected: u32,
    released_time_error_ns: Option<f64>,
    matched_time_error_ns: Option<f64>,
    released_velocity_error_m_s: Option<f64>,
    matched_velocity_error_m_s: Option<f64>,
    released_impulse_error_ns: Option<f64>,
    matched_impulse_error_ns: Option<f64>,
    end_impulse_error_ns: Option<f64>,
    max_prior_spin_rad_s: f64,
    max_unloaded_position_error_m: f64,
}
fn target(c: Case, relative: DVec3) -> Result<DVec3> {
    if relative.z >= 0. {
        return Err("impact target needs a closing state".into());
    }
    let law = Approach {
        frame: Setup::new(1., DVec3::new(0.2, 1., 0.25), 0.31, -relative.z),
        slip: DVec3::new(relative.x, relative.y, 0.),
        spin: DVec3::ZERO,
        hinge: 0.,
        mu: c.mu,
    }
    .target()?;
    Ok(c.rot * DVec3::from(law.impulse))
}
fn bracket(gap: f64, velocity: f64, accel: f64) -> Option<f64> {
    if gap <= 0. {
        return (velocity < 0.).then_some(0.);
    }
    let mut hi = 0.0005_f64;
    if accel > 0. {
        hi = hi.min((-velocity / accel).max(0.));
    }
    let distance = |t: f64| gap + t * (velocity + 0.5 * accel * t);
    if distance(hi) > 0. {
        return None;
    }
    let mut lo = 0.;
    for _ in 0..64 {
        let t = (hi + lo) * 0.5;
        if distance(t) > 0. {
            lo = t;
        } else {
            hi = t;
        }
    }
    Some((lo + hi) * 0.5)
}
fn bodies(s: &Simulation, ids: [BodyId; 3]) -> [Body; 3] {
    let n = s.world().resource::<PhysicsBackend>();
    ids.map(|id| Body::from(n.state(id).unwrap()))
}
fn solve(c: Case, g: DVec3, steps: u32, save: bool) -> Result<(ResultRow, Vec<Value>)> {
    let xml = c.xml_with_gravity(g).replace(
        "<native_steps>8",
        &format!("<native_steps>{}", steps.max(2)),
    );
    let mut sim = load_world_with_sdf_step(
        &xml,
        "ballistic.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    primary_contact::set_enabled(&mut sim, steps != 0)?;
    #[allow(deprecated)]
    let ids = [
        "default::rocker::foot",
        "default::rocker::upper",
        "default::lift::pad",
    ]
    .map(|name| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|x| x.0 == name))
            .unwrap()
            .get::<BodyHandle>()
            .unwrap()
            .id()
    });
    let init = bodies(&sim, ids);
    let center = |b: [Body; 3]| 0.5 * (DVec3::from(b[0].position) + DVec3::from(b[1].position));
    let momentum = |b: [Body; 3]| DVec3::from(b[0].velocity) + DVec3::from(b[1].velocity);
    let real_v = g * (c.ns() * 1e-9) - c.pad_v;
    let predicted = target(c, real_v)?;
    let actual_g = c.rot * g;
    let mut rows = vec![];
    let mut r = ResultRow::default();
    for tick in 0..=c.prefix + if save { 24 } else { 4 } {
        let before = bodies(&sim, ids);
        let mut calls = 0;
        let mut positive = 0.;
        if tick > 0 {
            let start = (tick - 1) * 500_000;
            if r.loaded_tick.is_none() {
                let t = start as f64 * 1e-9;
                r.max_unloaded_position_error_m = r
                    .max_unloaded_position_error_m
                    .max(center(before).distance(center(init) + 0.5 * actual_g * t * t));
                r.max_prior_spin_rad_s = r.max_prior_spin_rad_s.max(
                    before[0..2]
                        .iter()
                        .map(|b| DVec3::from(b.angular_velocity).length())
                        .fold(0., f64::max),
                );
            }
            let frame = c.rot.inverse();
            let q = frame * DQuat::from_array(before[0].orientation);
            let extent = (q.inverse() * DVec3::Z).abs().dot(DVec3::from(FOOT_HALF));
            let gap = (frame * (DVec3::from(before[0].position) - DVec3::from(before[2].position)))
                .z
                - extent
                - PAD_HALF[2];
            let rel = frame * (DVec3::from(before[0].velocity) - DVec3::from(before[2].velocity));
            let root = bracket(gap, rel.z, g.z);
            sim.step_exact(1)?;
            calls = 1;
            let mut end = tick * 500_000;
            if let Some(p) = primary_contact::last_refinement(&sim).filter(|p| p.iteration == tick)
            {
                calls = p.interval.native_steps;
                for pair in &p.requested_pairs {
                    positive += pair.normal_impulse;
                    if let Some(h) = pair.first_positive_end {
                        end = start + h.as_nanos() as u64;
                    }
                }
                for hit in &p.interval.corner_impacts {
                    r.projected += 1;
                    let measured = DVec3::from(hit.impulse_on_foot);
                    let vel = DVec3::from(hit.before_relative_velocity);
                    r.released_velocity_error_m_s = Some(vel.distance(c.rot * real_v));
                    r.released_impulse_error_ns = Some(measured.distance(predicted));
                    if let Some(t) = root {
                        r.matched_time_error_ns = Some((hit.elapsed.as_secs_f64() - t).abs() * 1e9);
                        let at = rel + g * hit.elapsed.as_secs_f64();
                        r.matched_velocity_error_m_s = Some(vel.distance(c.rot * at));
                        r.matched_impulse_error_ns = Some(measured.distance(target(c, at)?));
                    }
                }
            } else {
                let n = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .unwrap();
                positive = n
                    .rigid_contact_observations()
                    .iter()
                    .filter(|x| (x.a, x.b) == (ids[0].min(ids[2]), ids[0].max(ids[2])))
                    .map(|x| x.normal_impulse)
                    .sum();
            }
            if positive > 0. && r.loaded_tick.is_none() {
                r.loaded_tick = Some(tick);
                r.first_report_ns = Some(end);
                r.released_time_error_ns = Some((end as f64 - c.ns()).abs());
                r.end_impulse_error_ns = Some(
                    (momentum(bodies(&sim, ids)) - 2. * actual_g * (tick as f64 * 0.0005))
                        .distance(predicted),
                );
            }
        }
        if save {
            let [a, b, d] = bodies(&sim, ids);
            rows.push(json!({"tick":tick,"time_us":tick*500,"foot":a,"upper":b,"pad":d,"normal_impulse_ns":positive,"native_steps":calls,"solver_iterations":4}));
        }
    }
    Ok((r, rows))
}
pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut cases = vec![];
    let mut lanes = vec![];
    for (gravity, g) in [
        DVec3::new(0., 0., -9.81),
        DVec3::new(2.6, -3.1, -9.81),
        DVec3::new(-3.7, 2.1, 1.25),
        DVec3::new(-3.4, 2.2, 0.),
    ]
    .into_iter()
    .enumerate()
    {
        for (orient, rot) in [
            DQuat::IDENTITY,
            DQuat::from_euler(glam::EulerRot::ZYX, 0.22, -0.68, 0.31),
            DQuat::from_rotation_y(1.88),
        ]
        .into_iter()
        .enumerate()
        {
            for pad_first in [false, true] {
                for prefix in [0, 1, 5, 25, 128] {
                    for phase in [197.25, 62_499.8, 138_197.65, 499_997.35] {
                        for mu in [0., 0.6, 1.] {
                            for closing in [0.62, 1.4] {
                                let c = Case {
                                    rot,
                                    pad_v: DVec3::new(1.1, -0.7, closing),
                                    mu,
                                    prefix,
                                    phase,
                                    pad_first,
                                };
                                let show = gravity == 1
                                    && orient == 0
                                    && !pad_first
                                    && prefix == 5
                                    && phase == 138_197.65
                                    && mu == 0.6
                                    && closing == 1.4;
                                let mut measurements = vec![];
                                for (steps, label, name) in [
                                    (0, "ordinary", "Ordinary gravity and contact"),
                                    (2, "two", "Ballistic event, two native calls"),
                                    (8, "eight", "Ballistic event, eight native calls"),
                                ] {
                                    let (r, frames) = solve(c, g, steps, show)?;
                                    measurements.push(r);
                                    if show {
                                        write_json(
                                            &out.join("traces").join(format!("{label}.json")),
                                            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":UPPER_HALF,"pad_half_extents":PAD_HALF},"frames":frames}),
                                        )?;
                                        lanes.push(json!({"label":label,"name":name,"file":format!("traces/{label}.json"),"measured":r}));
                                    }
                                }
                                cases.push(json!({"gravity_index":gravity,"support_frame_gravity":g.to_array(),"orientation_index":orient,"prefix":prefix,"phase_ns":phase,"mu":mu,"closing":closing,"pad_first":pad_first,"paths":measurements}));
                            }
                        }
                    }
                }
            }
        }
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"title":"A free ankle approaches under oblique gravity","pair_totals_only":true,"lanes":lanes}),
    )?;
    write_json(
        &out.join("study.json"),
        &json!({"cases":cases.len(),"native_steps":[0,2,8],"rows":cases}),
    )
}
