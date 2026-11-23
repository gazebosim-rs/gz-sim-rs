// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, PhysicsEngine, Velocity, WorldPose, deformable::bodies, load_world_with_sdf_step,
    remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

const SLING: &str = include_str!("../../../examples/multifidelity_deformables.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "soft.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn ball(sim: &Simulation) -> ([f64; 3], Velocity) {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::payload::ball")
        })
        .unwrap();
    (
        e.get::<WorldPose>().unwrap().translation,
        *e.get::<Velocity>().unwrap(),
    )
}
fn positions(sim: &Simulation) -> Vec<[f64; 3]> {
    bodies(sim)
        .flat_map(|(_, s)| s.particles().iter().map(|p| p.position))
        .collect()
}

#[test]
fn public_clock_support_load_cloth_sag_and_reset_replay() {
    let mut sim = load(SLING);
    let initial = positions(&sim);
    sim.tick().unwrap();
    assert_eq!(positions(&sim), initial);
    sim.step_exact(300).unwrap();
    let carried = positions(&sim);
    let payload = ball(&sim).0;
    assert!(
        payload[2] > 0.85 && payload[2] < 1.21,
        "payload {payload:?}"
    );
    assert_eq!(sim.elapsed(), Duration::from_millis(300));
    let panel = bodies(&sim).find(|(n, _)| *n == "panel").unwrap().1;
    let sag = 1.
        - panel
            .particles()
            .iter()
            .map(|p| p.position[2])
            .fold(f64::INFINITY, f64::min);
    assert!(sag > 0.04 && sag < 0.6, "panel sag={sag}");
    assert_eq!(panel.particles()[0].position, [0., 0.9, 1.]);
    println!(
        "t=0.300 s panel sag={sag:.6} m, supported sphere height={:.6} m",
        payload[2]
    );
    sim.reset().unwrap();
    assert_eq!(positions(&sim), initial);
    sim.step_exact(300).unwrap();
    assert_eq!(carried, positions(&sim));
    assert_eq!(ball(&sim).0, payload);
    let id = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::payload"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    remove_model(&mut sim, id).unwrap();
    sim.step_exact(1).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(positions(&sim), initial);
    sim.step_exact(300).unwrap();
    assert_eq!(carried, positions(&sim));
    assert_eq!(ball(&sim).0, payload);
}

fn impact(substeps: u32, mode: &str) -> String {
    format!(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 0</gravity>
      <physics name="p" type="ignored"><max_step_size>0.001</max_step_size></physics>
      <model name="payload"><link name="ball"><inertial><mass>4</mass><inertia><ixx>0.4</ixx><iyy>0.4</iyy><izz>0.4</izz></inertia></inertial>
      <collision name="c"><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link></model>
      <plugin name="gz::sim::systems::Deformables" filename="gz-sim-deformables-system">
      <sphere link="default::payload::ball" coupling="{mode}"/>
      <cable name="projectile" substeps="{substeps}" iterations="4" radius="0.01" self_contact="false" friction="0.5" stretch="0" bend="0">
      <particle position="-0.55 0.2 -0.04" velocity="2 0 0" mass="0.05"/>
      <particle position="-0.55 0.2 0.04" velocity="2 0 0" mass="0.05"/>
      </cable></plugin></world></sdf>"#
    )
}
fn totals(sim: &Simulation, mass: f64) -> (DVec3, DVec3, f64) {
    let (pos, vel) = ball(sim);
    let v = DVec3::from_array(vel.linear);
    let w = DVec3::from_array(vel.angular);
    let mut p = mass * v;
    let mut l = DVec3::from_array(pos).cross(p) + (0.1 * mass) * w;
    let mut k = 0.5 * mass * (v.length_squared() + 0.1 * w.length_squared());
    for (_, soft) in bodies(sim) {
        for pt in soft.particles() {
            let v = DVec3::from_array(pt.velocity);
            let m = pt.inverse_mass.recip();
            p += m * v;
            l += DVec3::from_array(pt.position).cross(m * v);
            k += 0.5 * m * v.length_squared();
        }
    }
    (p, l, k)
}
#[test]
fn moving_contact_conserves_linear_and_angular_momentum_with_one_two_and_four_substeps() {
    for n in [1, 2, 4] {
        let mut sim = load(&impact(n, "two_way"));
        let (p, l, k) = totals(&sim, 4.);
        let mut max_p: f64 = 0.;
        let mut max_l: f64 = 0.;
        let mut max_k: f64 = k;
        for _ in 0..120 {
            sim.step_exact(1).unwrap();
            let (pi, li, ki) = totals(&sim, 4.);
            max_p = max_p.max(pi.distance(p));
            max_l = max_l.max(li.distance(l));
            max_k = max_k.max(ki);
        }
        let (_, velocity) = ball(&sim);
        println!(
            "substeps={n} peak momentum residuals P={max_p:e}, L={max_l:e}; peak kinetic energy={max_k:.8} J (initial {k:.8})"
        );
        assert!(
            velocity.linear[0] > 0.025 && velocity.angular[2].abs() > 0.005,
            "{velocity:?}"
        );
        assert!(max_p < 1e-8 && max_l < 2e-7 && max_k <= k + 1e-5);
    }
    let mut sim = load(&impact(4, "one_way"));
    sim.step_exact(120).unwrap();
    assert_eq!(ball(&sim).1.linear, [0.; 3]);
    assert!(bodies(&sim).next().unwrap().1.particles()[0].velocity[0] < 1.);
}
#[test]
fn loaded_configuration_refuses_invalid_or_ambiguous_physics() {
    let xml = impact(2, "two_way");
    for bad in [
        xml.replace("two_way", "unknown"),
        xml.replace("default::payload::ball", "foreign"),
        xml.replace("substeps=\"2\"", "substeps=\"1000001\""),
        xml.replace("friction=", "unknown="),
        xml.replace("0.05\"", "-0.05\""),
        xml.replace(
            "<model name=\"payload\">",
            "<model name=\"payload\"><static>true</static>",
        ),
    ] {
        assert!(
            load_world_with_sdf_step(
                &bad,
                "bad.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
    assert!(
        load_world_with_sdf_step(
            &xml,
            "bad.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
}

#[test]
fn stiff_cable_impact_on_a_light_free_receiver() {
    let mass = 0.01;
    let mut xml = impact(4, "two_way")
        .replace("<mass>4</mass>", "<mass>0.01</mass>")
        .replace(">0.4</", ">0.001</");
    // Impact is radial; the symmetric two-particle cable cancels any torque on the sphere.
    xml = xml
        .replace("0.2 -0.04", "0 -0.04")
        .replace("0.2 0.04", "0 0.04");
    let mut sim = load(&xml);
    let (p, l, k) = totals(&sim, mass);
    let mut peak: f64 = k;
    for _ in 0..120 {
        sim.step_exact(1).unwrap();
        let (pi, li, ki) = totals(&sim, mass);
        assert!(pi.distance(p) < 1e-8 && li.distance(l) < 1e-5);
        peak = peak.max(ki);
    }
    println!(
        "receiver mass={mass} kg, cable mass=0.1 kg, initial energy={k:.6} J, peak={peak:.6} J"
    );
    assert!(ball(&sim).1.linear[0] > 0.1);
    assert!(peak <= k + 1e-5);
}

#[test]
fn cable_carries_payload_weight_and_reaction_impulse_matches_rigid_momentum() {
    let mut sim = load(SLING);
    sim.step_exact(650).unwrap();
    let velocity0 = ball(&sim).1.linear[2];
    let mut reaction = 0.0;
    let mut zmin = f64::INFINITY;
    let mut zmax = f64::NEG_INFINITY;
    let window = 2.0;
    for _ in 0..2000 {
        sim.step_exact(1).unwrap();
        reaction += bodies(&sim)
            .map(|(_, body)| body.last_boundary_impulses()[0].linear[2])
            .sum::<f64>();
        let z = ball(&sim).0[2];
        zmin = zmin.min(z);
        zmax = zmax.max(z);
    }
    let velocity1 = ball(&sim).1.linear[2];
    let mean = reaction / window;
    println!(
        "payload 650–2650 ms height range {zmin:.6}..{zmax:.6} m, mean cable support {mean:.6} N vs weight 9.810000 N"
    );
    assert!((reaction - (velocity1 - velocity0 + window * 9.81)).abs() < 1e-8);
    assert!((mean - 9.81).abs() < 0.8 && zmin > 0.9 && zmax < 1.25);
}

#[test]
fn failures_preserve_the_last_published_particle_state_and_timestamp() {
    // A soft impact on a sphere already in contact with a rigid wall would
    // require a simultaneous rigid/particle solve. The first such tick faults.
    let source=impact(2,"two_way").replace("</model>",r#"</model>
    <model name="wall"><static>true</static><pose>0.5498 0 0 0 0 0</pose><link name="w"><collision name="c"><geometry><box><size>0.1 1 1</size></box></geometry></collision></link></model>"#);
    let mut sim = load(&source);
    let mut failed = false;
    for _ in 0..120 {
        let before = positions(&sim);
        let time = sim.elapsed();
        if sim.step_exact(1).is_err() {
            let msg = sim.fault().unwrap().to_string();
            assert!(msg.contains("third-body"), "{msg}");
            assert_eq!(positions(&sim), before);
            let sample = bodies(&sim).next().unwrap().1;
            assert_eq!(sample.sim_time(), time);
            assert_eq!(sim.elapsed(), time);
            failed = true;
            break;
        }
    }
    assert!(failed);
}

#[test]
fn finite_spheres_admit_principal_inertia_and_require_shared_substeps() {
    let anisotropic = SLING.replace("<ixx>0.009</ixx>", "<ixx>0.012</ixx>");
    let mixed = SLING.replace(
        "columns=\"5\" substeps=\"4\"",
        "columns=\"5\" substeps=\"2\"",
    );
    assert_ne!(mixed, SLING);
    let mut finite = load(&anisotropic);
    finite.step_exact(2).unwrap();
    assert!(
        load_world_with_sdf_step(
            &mixed,
            "mixed.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity
        )
        .is_err()
    );
    for xml in [&anisotropic, &mixed] {
        let mut sim = load(&xml.replace("two_way", "one_way"));
        sim.step_exact(2).unwrap();
        assert!(bodies(&sim).all(|(_, sample)| sample.sim_time() == sim.elapsed()));
    }
}
#[test]
fn removing_a_lone_boundary_preserves_its_reaction_slot_and_reset_rebinds_it() {
    let mut sim = load(&impact(2, "one_way"));
    let id = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::payload"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    remove_model(&mut sim, id).unwrap();
    sim.step_exact(110).unwrap();
    let sample = bodies(&sim).next().unwrap().1;
    let impulses = sample.last_boundary_impulses();
    assert_eq!(impulses.len(), 1);
    assert_eq!(impulses[0].boundary, 0);
    assert_eq!(impulses[0].linear, [0.; 3]);
    assert_eq!(sample.particles()[0].velocity, [2., 0., 0.]);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let sample = bodies(&sim).next().unwrap().1;
    assert_eq!(sample.epoch(), sim.epoch());
    assert_eq!(sample.sim_time(), Duration::ZERO);
    assert!(sample.last_boundary_impulses().is_empty());
    sim.step_exact(110).unwrap();
    assert!(bodies(&sim).next().unwrap().1.particles()[0].velocity[0] < 1.);
}
