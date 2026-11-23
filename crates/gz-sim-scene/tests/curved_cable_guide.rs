// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldLoadOptions, deformable, load_world_configured,
    queue_link_world_velocity, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;
const SDF: &str = include_str!("../../../examples/multifidelity_curved_cable_guide.sdf");
fn load(s: &str, step: u64) -> Simulation {
    load_world_configured(
        s,
        "guide.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            step: Some(Duration::from_nanos(step)),
            engine: PhysicsEngine::MultiFidelity,
            distributed_worker: None,
            noise_seed: [0; 32],
        },
    )
    .unwrap()
}
fn state(
    s: &Simulation,
) -> (
    Vec<gz_physics_xpbd::Particle>,
    gz_physics_xpbd::BoundaryImpulse,
    u32,
) {
    let a = deformable::bodies(s).next().unwrap().1;
    (
        a.particles().to_vec(),
        a.last_boundary_impulses()[0],
        a.last_path_substeps(),
    )
}
#[test]
fn curved_guide_impacts_inside_an_endpoint_clear_tick_with_convergent_impulse_and_reset() {
    let first = (1. - 0.5f64.sqrt()) * 0.05;
    let speed = 4.8 - 96. * first;
    let mut reference = None;
    for ns in [100_000_000, 1_000_000, 100_000] {
        let mut s = load(SDF, ns);
        for epoch in 0..2 {
            if epoch == 1 {
                reset_loaded_world(&mut s, |_| ()).unwrap();
            }
            let mut impulse = [0.; 3];
            let mut torque = [0.; 3];
            let mut sub = 0;
            let mut hit = None;
            for tick in 1..=100000000 / ns {
                s.step_exact(1)
                    .unwrap_or_else(|e| panic!("{e} {:?}", s.fault()));
                let (_, r, n) = state(&s);
                sub += n;
                if r.linear[2] < -1e-5 {
                    hit.get_or_insert(tick);
                }
                for k in 0..3 {
                    impulse[k] += r.linear[k];
                    torque[k] += r.angular[k];
                }
            }
            let (ps, ..) = state(&s);
            assert!(sub > 100);
            for p in &ps {
                assert!(
                    (p.velocity[2] - speed).abs() < 0.012,
                    "{ns}: {p:?}, {speed}"
                );
                assert!((p.position[2] - (0.65 + speed * (0.1 - first))).abs() < 0.0011);
            }
            assert!(
                (impulse[2]
                    + ps.iter()
                        .map(|p| p.velocity[2] / p.inverse_mass)
                        .sum::<f64>())
                .abs()
                    < 2e-12
            );
            assert!((torque[0] - impulse[2] * 0.05).abs() < 2e-11);
            if ns < 100000000 {
                let t = hit.unwrap() as f64 * ns as f64 * 1e-9;
                assert!(t >= first && t - first < ns as f64 * 1e-9 + 0.00001);
            }
            if ns == 100000000 && epoch == 0 {
                reference = Some((ps.clone(), impulse, torque));
            }
            if ns == 100000000 && epoch == 1 {
                assert_eq!(reference.as_ref().unwrap(), &(ps, impulse, torque));
            }
        }
    }
}
#[test]
fn public_velocity_override_and_loaded_reset_change_one_way_path_authority() {
    let mut s = load(SDF, 100000000);
    let e = s
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::guide::roller")
        })
        .unwrap();
    let id = e.get::<EntityId>().unwrap().get();
    let b = e.get::<BodyHandle>().unwrap().id();
    queue_link_world_velocity(&mut s, id, [0.; 3], [0.; 3]).unwrap();
    s.step_exact(1).unwrap();
    let (ps, r, n) = state(&s);
    assert_eq!(r.linear, [0.; 3]);
    assert_eq!(n, 0);
    assert!(ps.iter().all(|p| p.velocity == [0.; 3]));
    assert_eq!(
        s.world()
            .resource::<PhysicsBackend>()
            .state(b)
            .unwrap()
            .position,
        [0., 0., 0.5]
    );
    reset_loaded_world(&mut s, |_| ()).unwrap();
    s.step_exact(1).unwrap();
    assert!(state(&s).1.linear[2] < -0.6);
}

#[test]
fn ordinary_particle_visuals_follow_material_in_world_and_rotating_parent_coordinates() {
    use glam::{DQuat, DVec3};
    use gz_sim_scene::{LocalPose, WorldPose};
    let mut s = load(SDF, 1_000_000);
    let initial = s
        .world()
        .iter_entities()
        .filter(|e| e.get::<Name>().is_some_and(|n| n.0.contains("::node_")))
        .count();
    assert_eq!(initial, 16);
    for tick in 0..180 {
        if tick == 170 {
            reset_loaded_world(&mut s, |_| ()).unwrap();
        }
        let a = deformable::bodies(&s).next().unwrap().1;
        for e in s.world().iter_entities() {
            let Some(leaf) = e
                .get::<Name>()
                .and_then(|n| n.0.strip_prefix("default::guide::roller::node_"))
            else {
                continue;
            };
            let position = a.particles()[leaf.parse::<usize>().unwrap()].position;
            let visual = e.get::<WorldPose>().unwrap();
            assert_eq!(visual.translation, position);
            let link = e.get::<bevy_ecs::prelude::ChildOf>().unwrap().parent();
            let parent = s.world().get::<WorldPose>(link).unwrap();
            let local = e.get::<LocalPose>().unwrap().0;
            let reconstructed = DVec3::from(parent.translation)
                + DQuat::from_array(parent.rotation) * DVec3::from(local.translation);
            assert!(reconstructed.distance(position.into()) < 2e-14);
        }
        s.step_exact(1).unwrap();
    }
    for broken in [
        SDF.replace("particle=\"15\"", "particle=\"16\""),
        SDF.replace("::node_15\"", "::node_14\""),
        SDF.replace("::node_15\"", "::barrel\""),
        SDF.replace("material=\"line\"", "material=\"absent\""),
    ] {
        assert!(
            gz_sim_scene::load_world_with_sdf_step(
                &broken,
                "invalid.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}
