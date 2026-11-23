// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! First native contact after a clear fall, observed through the normal SDF scene.
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, queue_link_world_wrench,
    remove_model, reset_loaded_world,
};
use sdformat::NoResources;

const H: f64 = 0.0005;
const ARM: f64 = 0.14;

fn link(name: &str, height: f64, com: f64, size: &str, mass: f64, inertia: [f64; 3]) -> String {
    let [ix, iy, iz] = inertia;
    format!(
        r#"<link name="{name}"><pose>0 0 {height} 0 0 0</pose>
        <inertial><pose>0 0 {com} 0 0 0</pose><mass>{mass}</mass>
        <inertia><ixx>{ix}</ixx><iyy>{iy}</iyy><izz>{iz}</izz></inertia></inertial>
        <collision name="box"><geometry><box><size>{size}</size></box></geometry></collision></link>"#
    )
}
fn scene_xml(welded: bool, foot_is_child: bool) -> String {
    let foot_i = [0.004533333333333334, 0.012, 0.014133333333333333];
    let upper_i = [0.0024, 0.002033333333333333, 0.002033333333333333];
    let links = if welded {
        let (parent, child) = if foot_is_child {
            ("upper", "foot")
        } else {
            ("foot", "upper")
        };
        link("foot", 0., 0., "0.36 0.2 0.12", 1., foot_i)
            + &link("upper", ARM, 0., "0.1 0.12 0.12", 1., upper_i)
            + &format!(
                r#"<joint name="weld" type="fixed"><parent>{parent}</parent><child>{child}</child></joint>"#
            )
    } else {
        let mut i: [f64; 3] = std::array::from_fn(|j| foot_i[j] + upper_i[j]);
        i[0] += ARM * ARM / 2.;
        i[1] += ARM * ARM / 2.;
        link("foot", 0., ARM / 2., "0.36 0.2 0.12", 2., i)
    };
    format!(
        r#"<sdf version="1.12"><world name="w"><gravity>0 0 -9.81</gravity>
        <physics name="fixed" type="ignored"><max_step_size>{H}</max_step_size></physics>
        <model name="rig"><pose>0 0 0.095 0 0 0</pose>{links}</model>
        <model name="ground"><static>true</static><pose>0 0 -0.025 0 0 0</pose>
        <link name="base"><collision name="top"><geometry><box><size>4 4 0.05</size></box></geometry></collision></link></model></world></sdf>"#
    )
}

fn load(sdf: &str) -> Simulation {
    load_world_with_sdf_step(
        sdf,
        "weld-impact.sdf",
        "w",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn native(sim: &Simulation) -> &MultiFidelityBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
}
#[allow(deprecated)]
fn body(sim: &Simulation, name: &str) -> BodyId {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<BodyHandle>()
        .unwrap()
        .id()
}
fn state(sim: &Simulation, id: BodyId) -> BodyState {
    sim.world().resource::<PhysicsBackend>().state(id).unwrap()
}

#[derive(Debug)]
struct Hit {
    normal: f64,
    momentum: DVec3,
    angular_velocity: DVec3,
    center: DVec3,
}

fn first_hit(sim: &mut Simulation, welded: bool) -> Hit {
    sim.step_exact(168).unwrap();
    assert!(
        native(sim)
            .rigid_contact_observations()
            .iter()
            .all(|p| p.normal_impulse == 0.)
    );
    impact_step(sim, welded)
}

fn impact_step(sim: &mut Simulation, welded: bool) -> Hit {
    let foot = body(sim, "w::rig::foot");
    let floor = body(sim, "w::ground::base");
    let upper = welded.then(|| body(sim, "w::rig::upper"));
    sim.step_exact(1).unwrap();
    let mut normal = 0.;
    for p in native(sim).rigid_contact_observations() {
        if p.normal_impulse > 0. {
            assert!(
                (p.a == floor && p.b == foot) || (p.b == floor && p.a == foot),
                "active shape lost its public identity: {p:?}"
            );
            normal += p.normal_impulse;
        }
    }
    assert!(normal > 0.);
    let f = state(sim, foot);
    let (momentum, center) = if let Some(upper) = upper {
        let u = state(sim, upper);
        (
            DVec3::from(u.velocity) + DVec3::from(f.velocity),
            (DVec3::from(u.position) + DVec3::from(f.position)) * 0.5,
        )
    } else {
        (
            2. * DVec3::from(f.velocity),
            DVec3::from(f.position) + DQuat::from_array(f.orientation) * DVec3::Z * (ARM * 0.5),
        )
    };
    Hit {
        normal,
        momentum,
        center,
        angular_velocity: f.angular_velocity.into(),
    }
}

#[test]
fn first_impact_matches_single_body_for_root_or_child_sole_after_reset() {
    let expected = first_hit(&mut load(&scene_xml(false, false)), false);
    for child in [false, true] {
        let mut welded = load(&scene_xml(true, child));
        for pass in 0..2 {
            if pass > 0 {
                reset_loaded_world(&mut welded, |_| ()).unwrap()
            }
            let hit = first_hit(&mut welded, true);
            assert!(
                (expected.normal - hit.normal).abs() < 1e-7,
                "foot_is_child={child}, reference {expected:?}, weld {hit:?}"
            );
            assert!(expected.momentum.distance(hit.momentum) < 1e-7);
            assert!(expected.angular_velocity.distance(hit.angular_velocity) < 1e-5);
            assert!(expected.center.distance(hit.center) < 1e-8);
        }
    }
}

#[test]
fn upper_link_remains_an_independent_shape_for_contacts_and_ray_exclusion() {
    // Reverse gravity to strike the opposite face without rotating the geometry.
    // Starting 35 mm away from the ceiling gives the same incident speed as the floor drop.
    let sdf = scene_xml(true, false)
        .replace("<gravity>0 0 -9.81", "<gravity>0 0 9.81")
        .replace("0 0 -0.025 0 0 0", "0 0 0.355 0 0 0");
    let mut s = load(&sdf);
    let foot = body(&s, "w::rig::foot");
    let upper = body(&s, "w::rig::upper");
    let ceiling = body(&s, "w::ground::base");
    s.step_exact(168).unwrap();
    // At this boundary the horizontal ray passes through just the upper solid.
    let from = [-1., 0., state(&s, upper).position[2]];
    let physics = s.world().resource::<PhysicsBackend>();
    assert!(
        (physics
            .cast_ray(from, [1., 0., 0.], 2., Some(foot))
            .unwrap()
            .unwrap()
            - 0.95)
            .abs()
            < 1e-8
    );
    assert!(
        physics
            .cast_ray(from, [1., 0., 0.], 2., Some(upper))
            .unwrap()
            .is_none()
    );
    s.step_exact(1).unwrap();
    let contacts: Vec<_> = native(&s)
        .rigid_contact_observations()
        .into_iter()
        .filter(|p| p.normal_impulse > 0.)
        .collect();
    assert_eq!(contacts.len(), 1);
    let p = &contacts[0];
    assert!((p.a == upper && p.b == ceiling) || (p.a == ceiling && p.b == upper));
    let hist = native(&s).native_contact_warmstart(upper, ceiling).unwrap();
    assert!(hist.points > 0 && hist.normal_ns > 0.);
    assert_eq!(
        native(&s)
            .native_contact_warmstart(foot, ceiling)
            .unwrap()
            .normal_ns,
        0.
    );
}

#[test]
#[allow(deprecated)]
fn removing_a_loaded_floor_releases_the_weld_without_a_stale_contact() {
    let mut sim = load(&scene_xml(true, false));
    let first = first_hit(&mut sim, true);
    let foot = body(&sim, "w::rig::foot");
    let upper = body(&sim, "w::rig::upper");
    let ground = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "w::ground"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    sim.set_paused(true);
    remove_model(&mut sim, ground).unwrap();
    sim.step_exact(1).unwrap();
    assert!(native(&sim).rigid_contact_observations().is_empty());
    let p = DVec3::from(state(&sim, foot).velocity) + DVec3::from(state(&sim, upper).velocity);
    assert!(p.distance(first.momentum - 2. * 9.81 * H * DVec3::Z) < 1e-9);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let repeated = first_hit(&mut sim, true);
    assert_eq!(first.normal, repeated.normal);
    assert_eq!(first.angular_velocity, repeated.angular_velocity);
}

#[test]
#[allow(deprecated)]
fn a_load_on_the_upper_link_reaches_the_floor_during_the_first_impact() {
    let force = DVec3::new(120., -60., -150.);
    let couple = DVec3::new(3., 1., -2.);
    let run = |welded: bool| {
        let mut sim = load(&scene_xml(welded, false));
        sim.step_exact(168).unwrap();
        let (link_name, moment) = if welded {
            ("w::rig::upper", couple)
        } else {
            // Transfer the applied load from the upper link to the assembly COM.
            (
                "w::rig::foot",
                couple + (DVec3::Z * (ARM * 0.5)).cross(force),
            )
        };
        let link = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == link_name))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        queue_link_world_wrench(&mut sim, link, force.into(), moment.into()).unwrap();
        impact_step(&mut sim, welded)
    };
    let original = first_hit(&mut load(&scene_xml(false, false)), false);
    let single = run(false);
    let weld = run(true);
    assert!((single.normal - original.normal).abs() > 0.01);
    assert!((weld.normal - single.normal).abs() < 1e-7);
    assert!(weld.momentum.distance(single.momentum) < 1e-7);
    assert!(weld.angular_velocity.distance(single.angular_velocity) < 1e-5);
}
