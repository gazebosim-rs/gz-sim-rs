// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_physics_xpbd::Particle;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, deformable::bodies, load_world_with_sdf_step,
    queue_link_world_velocity, reset_loaded_world,
};
use sdformat::NoResources;

const XML: &str = include_str!("../../../examples/rotating_cuboid_cable.sdf");
fn link(sim: &Simulation) -> bevy_ecs::world::EntityRef<'_> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::bar::body"))
        .unwrap()
}
fn launch(sim: &mut Simulation) {
    let id = link(sim).get::<EntityId>().unwrap().get();
    queue_link_world_velocity(sim, id, [0.; 3], [0., 0., 8.]).unwrap();
}
fn snapshot(sim: &Simulation) -> (BodyState, Vec<Particle>) {
    let id = link(sim).get::<BodyHandle>().unwrap().id();
    let state = sim.world().resource::<PhysicsBackend>().state(id).unwrap();
    (state, bodies(sim).next().unwrap().1.particles().to_vec())
}
#[test]
fn loaded_swept_cable_matches_analytic_speed_momenta_and_exact_reset_replay() {
    let corner = 1f64.hypot(0.08);
    let radial = 1.0429948963187562;
    let polar = 0.08_f64.atan2(1.)
        + ((radial * radial + corner * corner - 0.04f64.powi(2)) / (2. * radial * corner)).acos();
    let n =
        (DVec3::X * radial - DQuat::from_rotation_z(-polar) * DVec3::new(1., 0.08, 0.)).normalize();
    let arm = radial * n.y;
    let speed = 8. * arm / (1. + 0.01 * (0.5 + arm * arm / 0.16));
    let loss = 0.005 * speed * 8. * arm;
    // The inflated box is clear on both sides of this 10 ms public tick.
    for angle in [-0.12, -0.04] {
        let local = DQuat::from_rotation_z(-angle) * DVec3::new(radial, 0., 0.);
        assert!(
            (local - local.clamp(-DVec3::new(1., 0.08, 0.14), DVec3::new(1., 0.08, 0.14))).length()
                > 0.04
        );
    }
    for count in [1, 2, 4, 16] {
        let xml = XML.replace("substeps=\"1\"", &format!("substeps=\"{count}\""));
        let mut sim = load_world_with_sdf_step(
            &xml,
            "cable.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        let stable = link(&sim).get::<EntityId>().unwrap().get();
        let mut expected = Vec::new();
        for epoch in 0..=1 {
            launch(&mut sim);
            sim.set_paused(true);
            sim.tick().unwrap();
            assert_eq!(snapshot(&sim).1[0].velocity, [0.; 3]);
            let mut contacts = Vec::new();
            let mut max_dv = f64::default();
            let mut max_l = f64::default();
            let mut max_loss_error = f64::default();
            for tick in 1..=120 {
                sim.step_exact(1)
                    .unwrap_or_else(|e| panic!("tick {tick} {e:?} {:?}", sim.fault()));
                let (s, p) = snapshot(&sim);
                let (_, body) = bodies(&sim).next().unwrap();
                if body.last_boundary_impulses()[0].linear != [0.; 3] {
                    contacts.push(tick);
                }
                let com = DVec3::from(s.position);
                let q = DQuat::from_array(s.orientation);
                let w = DVec3::from(s.angular_velocity);
                let spin = q * (DVec3::new(0.1, 0.12, 0.16) * (q.conjugate() * w));
                let mut m = 2. * DVec3::from(s.velocity);
                let mut l = com.cross(m) + spin;
                let mut k = 0.25 * m.length_squared() + 0.5 * w.dot(spin);
                for point in &p {
                    let v = DVec3::from(point.velocity);
                    let mv = v / point.inverse_mass;
                    m += mv;
                    l += DVec3::from(point.position).cross(mv);
                    k += 0.5 * v.dot(mv);
                    if tick >= 4 {
                        max_dv = max_dv.max(v.distance(n * speed));
                    }
                }
                assert!(m.length() < 1e-11);
                max_l = max_l.max(l.distance(DVec3::Z * 1.28));
                if tick >= 4 {
                    max_loss_error = max_loss_error.max((5.12 - k - loss).abs());
                }
                if epoch == 0 {
                    expected.push((s, p));
                } else {
                    assert_eq!((s, p), expected[tick - 1]);
                }
            }
            println!(
                "scene substeps={count} epoch={epoch} contacts={contacts:?} max dv={max_dv} m/s, dL={max_l} kg m²/s, loss error={max_loss_error} J"
            );
            assert_eq!(contacts, [4]);
            assert!(max_dv < 2e-4 && max_l < 1e-10 && max_loss_error < 1e-5);
            if epoch == 0 {
                reset_loaded_world(&mut sim, |_| ()).unwrap();
                assert_eq!(link(&sim).get::<EntityId>().unwrap().get(), stable);
            }
        }
    }
}
#[test]
fn unsupported_frictional_graze_does_not_publish_partial_cable_or_rigid_poses() {
    let xml = XML.replace("friction=\"0\"", "friction=\"0.2\"");
    let mut sim = load_world_with_sdf_step(
        &xml,
        "cable.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    launch(&mut sim);
    sim.step_exact(3).unwrap();
    let elapsed = sim.elapsed();
    let selected = bodies(&sim).next().unwrap().1.particles().to_vec();
    let pose = *link(&sim).get::<gz_sim_scene::WorldPose>().unwrap();
    assert!(sim.step_exact(1).is_err());
    assert_eq!(sim.elapsed(), elapsed);
    assert_eq!(bodies(&sim).next().unwrap().1.particles(), selected);
    let last = link(&sim);
    let last = last.get::<gz_sim_scene::WorldPose>().unwrap();
    assert_eq!(last.translation, pose.translation);
    assert_eq!(last.rotation, pose.rotation);
}
