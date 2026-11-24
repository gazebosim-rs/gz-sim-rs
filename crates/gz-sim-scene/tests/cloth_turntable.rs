// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::PhysicsBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, deformable::bodies, load_world_with_sdf_step,
    reset_loaded_world,
};
use serde_json::json;
use std::{collections::BTreeMap, fs};
const SOURCE: &str = include_str!("../../../examples/worlds/local_cloth_turntable.sdf");
const DT: f64 = 0.002;
const DRIFT: f64 = 0.18;
const OMEGA: f64 = 5.;
const N: usize = 300;
/// Apply the same Galilean shift to every material and the native center.
fn rest_frame() -> String {
    let xml = SOURCE.replace("0.18 0 0</linear_velocity>", "0 0 0</linear_velocity>");
    xml.lines()
        .map(|line| {
            let Some((a, b)) = line.split_once(" velocity=\"") else {
                return line.to_owned();
            };
            let (x, rest) = b.split_once(' ').unwrap();
            format!(
                "{a} velocity=\"{:.12} {rest}",
                x.parse::<f64>().unwrap() - DRIFT
            )
        })
        .collect::<Vec<_>>()
        .join("\n")
}

fn load(xml: &str) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_with_sdf_step(
        xml,
        "turntable.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
#[test]
#[allow(deprecated)]
fn centrifugally_limited_sheet_hold_and_slip_on_a_drifting_turntable() {
    let mut sim = load(SOURCE).unwrap();
    let mut control = load(&rest_frame()).unwrap();
    let mut max_hold = 0_f64;
    let mut max_velocity_error = 0_f64;
    let mut max_frame = 0_f64;
    let mut max_frame_velocity = 0_f64;
    let mut max_force_error = 0_f64;
    let mut max_moment_error = 0_f64;
    let mut last_loaded = 0;
    let mut max_kinetic = 0_f64;
    let mut max_balance = 0_f64;
    let mut max_angular_balance = 0_f64;
    let id = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::conveyor::deck")
        })
        .unwrap()
        .get::<BodyHandle>()
        .unwrap()
        .id();
    let mut history: Vec<
        Vec<(
            Vec<gz_physics_xpbd::Particle>,
            gz_physics_xpbd::BoundaryImpulse,
        )>,
    > = Vec::new();
    let mut samples = Vec::new();
    let mut observations = Vec::new();
    for epoch in 0..2 {
        if epoch == 1 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            reset_loaded_world(&mut control, |_| ()).unwrap();
        }
        for tick in 0..=N {
            if tick != 0 {
                sim.step_exact(1)
                    .unwrap_or_else(|e| panic!("tick {tick}: {e:?} {:?}", sim.fault()));
                control.step_exact(1).unwrap();
            }
            let t = tick as f64 * DT;
            let rotation = DQuat::from_rotation_z(t * OMEGA);
            let center = DVec3::new(t * DRIFT, 0., 0.3);
            let native = sim.world().resource::<PhysicsBackend>().state(id).unwrap();
            let board = "default::conveyor::deck";
            let mut poses = BTreeMap::from([(
                format!("{board}::board"),
                json!({"translation":center.to_array(),"rotation":native.orientation}),
            )]);
            let mut states = Vec::new();
            for ((name, s), (other, rest)) in bodies(&sim).zip(bodies(&control)) {
                assert_eq!(name, other);
                let carried = name == "carried";
                let idx = usize::from(!carried);
                let q = DQuat::from_array(native.orientation);
                let omega = DVec3::Z * OMEGA;
                let axis_com = (if carried {
                    DVec3::new(0.055, -0.025, 0.)
                } else {
                    DVec3::new(0.19, 0.012, 0.)
                }) + DVec3::Z * 0.028;
                let prior_p =
                    |i: usize| -> gz_physics_xpbd::Particle { history[tick - 1][idx].0[i] };
                let mut com = DVec3::ZERO;
                let mut v = DVec3::ZERO;
                for (i, p) in s.particles().iter().enumerate() {
                    let point = DVec3::from(p.position);
                    let speed = DVec3::from(p.velocity);
                    let peer = rest.particles()[i];
                    max_frame = max_frame
                        .max((point - t * DRIFT * DVec3::X).distance(peer.position.into()));
                    max_frame_velocity = max_frame_velocity
                        .max((speed - DRIFT * DVec3::X).distance(peer.velocity.into()));
                    if carried {
                        let local = axis_com
                            + DVec3::new(
                                if i % 2 == 0 { -0.006 } else { 0.006 },
                                if i / 2 == 0 { -0.006 } else { 0.006 },
                                0.,
                            );
                        max_hold = max_hold.max(point.distance(center + q * local));
                        max_velocity_error = max_velocity_error
                            .max(speed.distance(DRIFT * DVec3::X + omega.cross(q * local)));
                    }
                    com += point * 0.25;
                    v += speed * 0.25;
                    poses.insert(
                        format!("{board}::{name}_{i}"),
                        json!({"translation":p.position,"rotation":native.orientation}),
                    );
                }
                let j = s
                    .last_boundary_impulses()
                    .first()
                    .copied()
                    .unwrap_or_default();
                let local = q.conjugate() * (com - center);
                let force = DVec3::from(j.linear) / DT;
                if tick > 0 {
                    let origin = center - DVec3::X * DRIFT * DT;
                    let mut dp = DVec3::ZERO;
                    let mut dl = DVec3::ZERO;
                    let mut gravity_moment = DVec3::ZERO;
                    let gravity = DVec3::NEG_Z * 9.81;
                    for (i, p) in s.particles().iter().enumerate() {
                        let prior = prior_p(i);
                        let x0 = DVec3::from(prior.position);
                        let x1 = DVec3::from(p.position);
                        let v0 = DVec3::from(prior.velocity);
                        let v1 = DVec3::from(p.velocity);
                        let mass = p.inverse_mass.recip();
                        dp += mass * (v1 - v0);
                        dl += mass * ((x1 - origin).cross(v1) - (x0 - origin).cross(v0));
                        // Gravity is sampled at the start of each of eight
                        // substeps. Integrate its lever arm by linear interpolation.
                        let mean = x0 + (x1 - x0) * (7. / 16.);
                        gravity_moment += (mean - origin).cross(mass * gravity) * DT;
                    }
                    max_balance =
                        max_balance.max((dp + DVec3::from(j.linear) - 0.1 * gravity * DT).length());
                    max_angular_balance = max_angular_balance
                        .max((dl + DVec3::from(j.angular) - gravity_moment).length());
                    if carried {
                        let q0 = DQuat::from_rotation_z((t - DT) * OMEGA);
                        let centripetal = -(omega.cross(rotation * axis_com)
                            - omega.cross(q0 * axis_com))
                            * (0.1 / DT);
                        let expected = centripetal + gravity * 0.1;
                        max_force_error = max_force_error.max(force.distance(expected));
                        let mean = 0.5 * (rotation * axis_com + q0 * axis_com)
                            + DVec3::X * DRIFT * DT * 0.5;
                        max_moment_error = max_moment_error
                            .max(DVec3::from(j.angular).distance(DT * mean.cross(expected)) / DT);
                    } else {
                        if force.length() > 1e-8 {
                            last_loaded = tick;
                        }
                        if (2..=60).contains(&tick) {
                            max_kinetic = max_kinetic
                                .max((force.truncate().length() - 0.1 * 0.12 * 9.81).abs());
                        }
                        if tick > 120 {
                            assert_eq!(j.linear, [0.; 3]);
                        }
                    }
                }
                if epoch == 0 {
                    observations.push(json!({"tick":tick,"name":name,"local":local.to_array(),"com":com.to_array(),"v":v.to_array(),"force":(DVec3::from(j.linear)/DT).to_array(),"torque":(DVec3::from(j.angular)/DT).to_array(),"rotation":native.orientation}));
                }
                states.push((s.particles().to_vec(), j));
            }
            if epoch == 0 {
                history.push(states);
                samples.push(json!({"time_ns":tick*2_000_000,"poses":poses}));
            } else {
                assert_eq!(history[tick], states);
            }
        }
    }
    let receipt = json!({"last_loaded_tick":last_loaded,"max_hold_position_error_m":max_hold,"max_angular_surface_speed_error_m_s":max_velocity_error,"max_galilean_position_error_m":max_frame,"max_galilean_velocity_error_m_s":max_frame_velocity,"max_hold_force_error_n":max_force_error,"max_hold_moment_error_nm":max_moment_error,"max_kinetic_cone_error_n":max_kinetic,"max_signed_impulse_residual_ns":max_balance,"max_angular_balance_residual_nms":max_angular_balance});
    println!("{receipt}");
    assert!(max_hold < 1e-8 && max_velocity_error < 1e-6);
    assert!(max_frame < 1e-8 && max_frame_velocity < 1e-7);
    assert!(max_force_error < 2e-5 && max_moment_error < 8e-5);
    assert!(max_kinetic < 5e-6 && (72..90).contains(&last_loaded));
    assert!(max_balance < 1e-12 && max_angular_balance < 3e-9);
    if let Some(out) = std::env::var_os("GZ_TURNTABLE_MEASUREMENT") {
        let d = std::path::Path::new(&out);
        fs::create_dir_all(d).unwrap();
        fs::write(
            d.join("measurements.json"),
            serde_json::to_vec_pretty(&receipt).unwrap(),
        )
        .unwrap();
        fs::write(
            d.join("samples.json"),
            serde_json::to_vec(&observations).unwrap(),
        )
        .unwrap();
        fs::write(
            d.join("visual-reference.json"),
            serde_json::to_vec(&samples).unwrap(),
        )
        .unwrap();
    }
}

#[test]
#[allow(deprecated)]
fn admission_and_paused_commands_respect_the_authored_axis() {
    use gz_sim_core::EntityId;
    use gz_sim_scene::queue_link_world_velocity;
    for (xml, why) in [
        (
            SOURCE.replace("<angular_velocity>0 0 5", "<angular_velocity>0.3 0 5"),
            "rotation about",
        ),
        (
            SOURCE.replace("<pose>0 0 0.30 0 0 0", "<pose>0.005 0 0.30 0 0 0"),
            "center of mass",
        ),
        (
            SOURCE.replace("motion=\"turntable\"", "motion=\"spindle\""),
            "one-way prescribed",
        ),
    ] {
        let error = load(&xml).err().unwrap().to_string();
        assert!(error.contains(why), "{error}");
    }
    let mut sim = load(SOURCE).unwrap();
    let e = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::conveyor::deck")
        })
        .unwrap();
    let (id, body) = (
        e.get::<EntityId>().unwrap().get(),
        e.get::<BodyHandle>().unwrap().id(),
    );
    sim.set_paused(true);
    queue_link_world_velocity(&mut sim, id, [0.12, 0., 0.], [0., 0., -3.]).unwrap();
    assert!(queue_link_world_velocity(&mut sim, id, [0.; 3], [0., 0., 1800.]).is_err());
    assert!(queue_link_world_velocity(&mut sim, id, [0.; 3], [1., 0., 0.]).is_err());
    sim.tick().unwrap();
    assert!(sim.elapsed().is_zero());
    sim.step_exact(1).unwrap();
    let backend = sim.world_mut().resource_mut::<PhysicsBackend>();
    let state = backend.state(body).unwrap();
    assert!((state.position[0] - 0.12 * DT).abs() < 1e-10);
    assert!((state.angular_velocity[2] + 3.).abs() < 1e-9);
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_world_velocity(body, [0.; 3], [1., 0., 0.])
        .unwrap();
    assert!(sim.step_exact(1).is_err());
    assert_eq!(sim.elapsed(), std::time::Duration::from_millis(2));
}

#[test]
fn outer_sheet_matches_independent_slide_dynamics_and_refined_edge_departure() {
    let mut coarse = load(SOURCE).unwrap();
    let mut fine = load(&SOURCE.replace("substeps=\"8\"", "substeps=\"32\"")).unwrap();
    let mut x = DVec3::new(0.19, 0.012, 0.);
    let spin = DVec3::Z * OMEGA;
    let mut v = spin.cross(x) + DRIFT * DVec3::X;
    // Integrate Coulomb sliding for a point mass on a horizontal rotating
    // rectangle while the whole sheet is inside its flat top. The initial
    // acceleration points inward at breakaway, opposing incipient outward slip.
    let derivative = |time: f64, x: DVec3, v: DVec3| {
        let r = x - DRIFT * time * DVec3::X;
        let slip = v - DRIFT * DVec3::X - spin.cross(r);
        -0.12 * 9.81 * slip.try_normalize().unwrap_or_else(|| r.normalize())
    };
    let mut total = [DVec3::ZERO; 2];
    let mut last = [0; 2];
    let mut max_analytic = 0_f64;
    let mut max_refined = 0_f64;
    let mut max_refined_speed = 0_f64;
    let mut end = [DVec3::ZERO; 2];
    for tick in 1..=N {
        let mut velocity = [DVec3::ZERO; 2];
        for (k, sim) in [&mut coarse, &mut fine].into_iter().enumerate() {
            sim.step_exact(1).unwrap();
            let (_, s) = bodies(sim).find(|(n, _)| *n == "sliding").unwrap();
            let j = DVec3::from(s.last_boundary_impulses()[0].linear);
            if j.length() > 1e-10 {
                last[k] = tick;
            }
            total[k] += j;
            end[k] = s
                .particles()
                .iter()
                .map(|p| DVec3::from(p.position) * 0.25)
                .sum();
            velocity[k] = s
                .particles()
                .iter()
                .map(|p| DVec3::from(p.velocity) * 0.25)
                .sum();
        }
        if tick <= 60 {
            let h = DT / 100.;
            for n in 0..100 {
                let t = (tick - 1) as f64 * DT + f64::from(n) * h;
                let a = derivative(t, x, v);
                let v_mid = v + 0.5 * h * a;
                let a_mid = derivative(t + 0.5 * h, x + 0.5 * h * v, v_mid);
                x += h * v_mid;
                v += h * a_mid;
            }
            max_analytic = max_analytic.max(x.truncate().distance(end[0].truncate()));
        }
        max_refined = max_refined.max(end[0].distance(end[1]));
        max_refined_speed = max_refined_speed.max(velocity[0].distance(velocity[1]));
    }
    assert!(max_analytic < 3e-5 && max_refined < 1e-4 && max_refined_speed < 0.002);
    assert_eq!(last, [80, 80]);
    assert!(total[0].distance(total[1]) < 0.0002 && total[0].z < -0.14);
    let receipt = json!({"outer_analytical_com_error_before_edge_m":max_analytic,"max_refined_com_error_m":max_refined,"max_refined_com_speed_error_m_s":max_refined_speed,"contact_impulse_ns":total.map(|v|v.to_array()),"last_loaded_ticks":last,"final_positions_m":end.map(|v|v.to_array())});
    println!("{receipt}");
    if let Some(dir) = std::env::var_os("GZ_TURNTABLE_MEASUREMENT") {
        let p = std::path::Path::new(&dir);
        fs::create_dir_all(p).unwrap();
        fs::write(
            p.join("refinement.json"),
            serde_json::to_vec_pretty(&receipt).unwrap(),
        )
        .unwrap();
    }
}

#[test]
fn reauthoring_the_link_at_the_axial_collision_center_preserves_the_measured_moment() {
    let shifted = SOURCE.replace("<pose>0 0 0.30 0 0 0</pose>", "").replace(
        "<link name=\"deck\">",
        "<link name=\"deck\"><pose>0 0 0.30 0 0 0</pose>",
    );
    let mut centered = load(&shifted).unwrap();
    let mut offset = load(SOURCE).unwrap();
    for _ in 0..=100 {
        centered.step_exact(1).unwrap();
        offset.step_exact(1).unwrap();
        for ((_, a), (_, b)) in bodies(&centered).zip(bodies(&offset)) {
            for (p, q) in a.particles().iter().zip(b.particles()) {
                assert!(DVec3::from(p.position).distance(q.position.into()) < 1e-7);
            }
            let ja = a.last_boundary_impulses()[0];
            let jb = b.last_boundary_impulses()[0];
            assert!(DVec3::from(ja.linear).distance(jb.linear.into()) < 1e-7);
            assert!(DVec3::from(ja.angular).distance(jb.angular.into()) < 1e-7);
        }
    }
}

#[test]
#[allow(deprecated)]
fn only_the_bound_turntable_restricts_the_direction_of_an_angular_command() {
    use gz_sim_core::EntityId;
    let free = r#"<model name="free"><pose>5 0 0 0 0 0</pose><plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system">
    <linear_velocity>0 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>
    <link name="free"><collision name="c"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model>"#;
    let xml = SOURCE
        .replace(
            "<model name=\"conveyor\">",
            &format!("{free}<model name=\"conveyor\">"),
        )
        .replace(
            "<box link=",
            "<plane point=\"0 0 -5\" normal=\"0 0 1\"/><box link=",
        );
    let mut sim = load(&xml).unwrap();
    let e = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::free::free")
        })
        .unwrap();
    let (id, body) = (
        e.get::<EntityId>().unwrap().get(),
        e.get::<BodyHandle>().unwrap().id(),
    );
    gz_sim_scene::queue_link_world_velocity(&mut sim, id, [0.; 3], [1., 0., 0.]).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        (sim.world()
            .resource::<PhysicsBackend>()
            .state(body)
            .unwrap()
            .angular_velocity[0]
            - 1.)
            .abs()
            < 1e-9
    );
}
