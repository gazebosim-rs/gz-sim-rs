// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native cold-edge transaction and independent observable/feasible-set checks.
use super::super::transaction::{Frame, fixture, inertial};
use super::*;
use gz_physics::EdgeStickMode;

fn measure(mut input: Input, frame: Frame, free: bool, mode: Mode) -> Result<Value> {
    if free {
        let axis = input.seed.state.q * DVec3::X;
        input.edge.inertia -=
            DMat3::from_cols(axis * axis.x, axis * axis.y, axis * axis.z) * 0.0201;
        input.edge.spin = input.after_w
            - input.edge.inertia.inverse()
                * (input.edge.arm[0].cross(input.desired[0])
                    + input.edge.arm[1].cross(input.desired[1]));
        input.seed.state.w = input.edge.spin;
    }
    let e = input.edge;
    let reference = solve(e);
    let mut sim = load(&fixture(input, frame, free), true)?;
    let id = ids(&sim)?;
    let mut owner = sim.world_mut().resource_mut::<PhysicsBackend>();
    let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
    let before = departure::ingest(n, &id);
    let applied = n.project_mixed_edge_impact(id.foot, id.pad)?;
    let after = departure::ingest(n, &id);
    let Some(hit) = applied else {
        if reference.is_ok() || before != after {
            return Err("native declined certified edge or mutated fallback state".into());
        }
        return Ok(json!({"admitted":false,"reason":reference.err().unwrap().to_string()}));
    };
    let truth = reference.map_err(|e| format!("native accepted unresolved edge {e}"))?;
    let declared = match hit.mode {
        EdgeStickMode::BothStick => Mode::BothStick,
        EdgeStickMode::FirstSticks => Mode::Stick(0),
        EdgeStickMode::SecondSticks => Mode::Stick(1),
    };
    if declared != mode || declared != truth.mode {
        return Err("mode differs from manufactured physical solution".into());
    }
    let center = (DVec3::from(before.0.position) + DVec3::from(before.1.unwrap().position)) * 0.5;
    let points = hit.points.map(DVec3::from);
    let normal = frame.q * DVec3::Z;
    let position_error = (0..2)
        .map(|k| points[k].distance(center + frame.q * e.arm[k]))
        .fold(0., f64::max);
    let direction_error = (frame.q * truth.axis)
        .distance(DVec3::from(hit.edge_direction))
        .max(normal.distance(DVec3::from(hit.normal)));
    let (p0, h0, k0) = inertial(before);
    let (p1, h1, k1) = inertial(after);
    let mut endpoint_error = 0f64;
    let mut h_error = 0f64;
    let mut loss_error = 0f64;
    let mut speeds = 0f64;
    let mut work_error = 0f64;
    let lohi = hit.impulses.first_longitudinal_range;
    let mut endpoints = vec![];
    for (side, endpoint) in lohi.into_iter().enumerate() {
        let here = hit
            .split_world(endpoint)
            .ok_or("invalid interval endpoint")?
            .map(DVec3::from);
        let physical = truth.split(truth.range[side]).map(|j| frame.q * j);
        let mut hsum = DVec3::ZERO;
        let mut work = 0.;
        let mut relative = 0.;
        for i in 0..2 {
            endpoint_error = endpoint_error.max(here[i].distance(physical[i]));
            hsum += (points[i] - center).cross(here[i]);
            let pre = frame.q * e.point(i);
            let post = frame.q * (truth.velocity + truth.spin.cross(e.arm[i]));
            speeds = speeds
                .max(DVec3::from(hit.before_relative_velocity[i]).distance(pre))
                .max(DVec3::from(hit.after_relative_velocity[i]).distance(post));
            let deck = DVec3::from(before.2.position);
            let motor_velocity = DVec3::from(before.2.velocity)
                + DVec3::from(before.2.angular_velocity).cross(points[i] - deck);
            relative += physical[i].dot(0.5 * (pre + post));
            work += physical[i].dot(motor_velocity);
        }
        h_error = h_error.max((h1 - h0).distance(hsum));
        loss_error = loss_error.max((relative + hit.dissipated_energy).abs());
        work_error = work_error.max((relative + work - hit.kinetic_energy_change).abs());
        endpoints.push(json!({"native_ns":here.map(|p|p.to_array()),"independent_ns":physical.map(|p|p.to_array())}));
    }
    let p_error = (p1 - p0).distance(DVec3::from(hit.total_impulse_on_foot()));
    let k_error = (k1 - k0 - hit.kinetic_energy_change).abs();
    if [
        position_error,
        direction_error,
        endpoint_error,
        h_error,
        p_error,
        k_error,
        speeds,
        loss_error,
        work_error,
    ]
    .into_iter()
    .any(|x| !x.is_finite() || x > 1e-10)
    {
        return Err(format!("native validation failed endpoint={endpoint_error:e}, angle={h_error:e}, speed={speeds:e}, loss={loss_error:e}, work={work_error:e}").into());
    }
    if n.project_mixed_edge_impact(id.foot, id.pad)?.is_some() || departure::ingest(n, &id) != after
    {
        return Err("edge operation was repeated after closure".into());
    }
    Ok(
        json!({"admitted":true,"native_mode":name(declared),"axis":hit.edge_direction,"pad_normal":hit.normal,"before_relative_m_s":hit.before_relative_velocity,"after_relative_m_s":hit.after_relative_velocity,"loss_j":hit.dissipated_energy,"range_ns":lohi,"physical_range_ns":truth.range,"normal_ns":hit.impulses.normal,"longitudinal_total_ns":hit.impulses.longitudinal_total,"endpoints":endpoints,"uniqueness_bound":hit.uniqueness_bound,"endpoint_error_ns":endpoint_error,"momentum_residual_ns":p_error,"angular_residual_nms":h_error,"energy_residual_j":k_error,"relative_loss_residual_j":loss_error,"world_work_residual_j":work_error,"point_speed_residual_m_s":speeds,"site_position_residual_m":position_error}),
    )
}
pub(in super::super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut results = vec![];
    for mode in [Mode::BothStick, Mode::Stick(0), Mode::Stick(1)] {
        for mu in [0.08, 0.2, 0.35, 0.5, 0.65, 0.8] {
            for (roll, reverse, n) in [(0.18, false, [0.48, 0.72]), (-0.18, true, [0.72, 0.48])] {
                for angle in [0., 27., 68., 112.] {
                    let i = model(mu, f64::to_radians(angle), roll, mode, reverse, n);
                    for free in [false, true] {
                        for (family, frame) in [
                            (
                                "base",
                                Frame {
                                    q: DQuat::IDENTITY,
                                    shift: DVec3::ZERO,
                                    spin: 0.,
                                },
                            ),
                            (
                                "inclined spinning",
                                Frame {
                                    q: DQuat::from_euler(glam::EulerRot::ZYX, -0.4, 0.34, -0.18),
                                    shift: DVec3::new(-1.1, 1.8, -0.2),
                                    spin: 1.3,
                                },
                            ),
                        ] {
                            let observation = measure(i, frame, free, mode).map_err(|e| {
                                format!(
                                    "{} mu={mu} roll={roll} yaw={angle} free={free} {family}: {e}",
                                    name(mode)
                                )
                            })?;
                            results.push(json!({"mode":name(mode),"q":frame.q.to_array(),"mu":mu,"roll":roll,"yaw_deg":angle,"free_child":free,"frame":family,"result":observation}));
                        }
                    }
                }
            }
        }
    }
    fs::write(out.join("native.json"), serde_json::to_vec(&results)?)?;
    public_view(out, true)
}
