// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_sim_core::{Simulation, StepFault};
use gz_sim_scene::{
    PhysicsEngine,
    deformable::{MaterialForceError::*, bodies, material_energy, set_material_force},
    load_world_with_sdf_step, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;

fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "force.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn scene(substeps: u32, pins: [bool; 2]) -> Simulation {
    load(&format!(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 -1</gravity>
        <physics name="clock" type="ignored"><max_step_size>0.001</max_step_size></physics>
        <plugin name="gz::sim::systems::Deformables" filename="gz-sim-deformables-system">
        <cable name="loaded" substeps="{substeps}" iterations="4" stretch="1e12" bend="1" radius="0.01" self_contact="false" friction="0">
        <particle position="0 0 1" mass="1" pinned="{}"/><particle position="1 0 1" mass="3" pinned="{}"/>
        </cable>
        <cable name="free" substeps="{substeps}" iterations="4" stretch="0" bend="1" radius="0.01" self_contact="false" friction="0">
        <particle position="0 0 2" mass="0.5"/><particle position="1 0 2" mass="1.5"/>
        </cable></plugin></world></sdf>"#,
        pins[0], pins[1]
    ))
}

fn check_motion(sim: &Simulation, name: &str, expected: [(f64, f64); 2]) {
    let samples = bodies(sim).find(|(n, _)| *n == name).unwrap().1.particles();
    for (particle, (y, vy)) in samples.iter().zip(expected) {
        assert!(
            (particle.position[1] - y).abs() < 1e-11,
            "{particle:?}, y={y}"
        );
        assert!(
            (particle.velocity[1] - vy).abs() < 1e-9,
            "{particle:?}, v={vy}"
        );
    }
}

#[test]
fn a_mass_weighted_force_spans_all_substeps_once_and_keeps_the_authored_gravity() {
    for substeps in [1, 4, 8] {
        let mut sim = scene(substeps, [false, false]);
        set_material_force(&mut sim, "loaded", [0., 12., 0.]).unwrap();
        set_material_force(&mut sim, "loaded", [0., 8., 0.]).unwrap();
        set_material_force(&mut sim, "free", [0., 4., 0.]).unwrap();
        set_material_force(&mut sim, "free", [0.; 3]).unwrap();
        sim.set_paused(true);
        sim.tick().unwrap();
        check_motion(&sim, "loaded", [(0., 0.); 2]);
        let before = material_energy(&sim, "loaded").unwrap().unwrap();
        sim.step_exact(1).unwrap();
        let dt = 0.001;
        // Semi-implicit Euler with constant acceleration 8 N / 4 kg.
        let y = dt * dt * (1. + 1. / f64::from(substeps));
        check_motion(&sim, "loaded", [(y, 2. * dt); 2]);
        check_motion(&sim, "free", [(0., 0.); 2]);
        let sample = bodies(&sim).find(|(n, _)| *n == "loaded").unwrap().1;
        let momentum = sample
            .particles()
            .iter()
            .map(|p| DVec3::from(p.velocity) / p.inverse_mass)
            .sum::<DVec3>();
        assert!((momentum - DVec3::new(0., 8. * dt, -4. * dt)).length() < 1e-9);
        let after = material_energy(&sim, "loaded").unwrap().unwrap();
        assert!((before.gravitational - 4.).abs() < 1e-10);
        assert!((after.gravitational - (4. - 2. * y)).abs() < 1e-10);
        sim.step_exact(1).unwrap();
        check_motion(&sim, "loaded", [(y + 2. * dt * dt, 2. * dt); 2]);
        set_material_force(&mut sim, "loaded", [0., 8., 0.]).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        sim.step_exact(1).unwrap();
        check_motion(&sim, "loaded", [(0., 0.); 2]);
    }
}

#[test]
fn pinned_mass_does_not_dilute_the_pull_and_invalid_commands_preserve_the_last_valid_force() {
    let mut sim = scene(4, [false, true]);
    set_material_force(&mut sim, "loaded", [0., 2., 0.]).unwrap();
    for bad in [f64::NAN, f64::INFINITY] {
        assert_eq!(
            set_material_force(&mut sim, "loaded", [0., bad, 0.]),
            Err(InvalidForce)
        );
    }
    assert_eq!(
        set_material_force(&mut sim, "other", [0.; 3]),
        Err(UnknownMaterial)
    );
    sim.step_exact(1).unwrap();
    check_motion(&sim, "loaded", [(1.25e-6, 0.002), (0., 0.)]);
    let mut pinned = scene(2, [true, true]);
    assert_eq!(
        set_material_force(&mut pinned, "loaded", [1.; 3]),
        Err(InvalidMass)
    );
    set_material_force(&mut pinned, "loaded", [0.; 3]).unwrap();
    let mut empty = Simulation::new(Duration::from_millis(1)).unwrap();
    assert_eq!(
        set_material_force(&mut empty, "unknown", [1.; 3]),
        Err(UnknownMaterial)
    );
    empty
        .world()
        .resource::<StepFault>()
        .report(std::io::Error::other("step fault"));
    assert_eq!(
        set_material_force(&mut empty, "unknown", [1.; 3]),
        Err(Faulted)
    );
}

#[test]
fn impact_intervals_cannot_silently_drop_force_commands() {
    let xml = include_str!("../../../examples/articulated_material_cloth_impact.sdf");
    let mut sim = load(xml);
    assert_eq!(
        set_material_force(&mut sim, "material", [0., 1., 0.]),
        Err(UnsupportedOwner)
    );
}
