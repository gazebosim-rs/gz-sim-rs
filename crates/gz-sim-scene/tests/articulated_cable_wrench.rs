// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, deformable::bodies, load_world_with_sdf_step,
    queue_link_world_wrench, reset_loaded_world,
};
use sdformat::NoResources;

const ARM: &str = include_str!("../../../examples/local_manipulation_arm.sdf");
const PLUGIN: &str = r#"<plugin name="gz::sim::systems::Deformables" filename="gz-sim-deformables-system">
  <box link="default::arm::left" coupling="articulated_wrench"/>
  <box link="default::arm::right" coupling="articulated_wrench"/>
  <cable name="cable" substeps="2" iterations="8" radius="0.005" self_contact="false" friction="0" stretch="0" bend="0">
    <particle position=".2273 .015 .197" velocity="-1 0 0" mass=".004"/>
    <particle position=".2273 .030 .197" velocity="-1 0 0" mass=".004"/>
  </cable>
</plugin>"#;

fn scene(plugin: &str) -> String {
    ARM.replace("</world>", &format!("{plugin}</world>"))
}

fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "arm.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn jaw(sim: &Simulation) -> (u64, BodyState) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::arm::left"))
        .unwrap();
    let id = entity.get::<BodyHandle>().unwrap().id();
    (
        entity.get::<EntityId>().unwrap().get(),
        sim.world().resource::<PhysicsBackend>().state(id).unwrap(),
    )
}

#[test]
fn off_center_cable_load_reaches_jaw_on_next_advancing_tick_and_clears_on_reset() {
    let mut coupled = load(&scene(PLUGIN));
    let mut reference = load(ARM);
    let mut unloaded = load(ARM);
    let (id, start) = jaw(&reference);
    for sim in [&mut coupled, &mut reference, &mut unloaded] {
        sim.step_exact(1).unwrap();
    }
    let (_, end) = jaw(&reference);
    assert_eq!(jaw(&coupled).1, end);
    let loads = bodies(&coupled).next().unwrap().1.last_boundary_impulses();
    let j = DVec3::from(loads[0].linear);
    let l = DVec3::from(loads[0].angular);
    assert!(j.x < -0.001 && l.z > 1e-5, "{loads:?}");
    assert_eq!(loads[1].linear, [0.; 3]);
    let torque = (l + (DVec3::from(start.position) - DVec3::from(end.position)).cross(j)) / 0.001;
    queue_link_world_wrench(
        &mut reference,
        id,
        (j / 0.001).to_array(),
        torque.to_array(),
    )
    .unwrap();
    coupled.set_paused(true);
    coupled.tick().unwrap();
    assert_eq!(jaw(&coupled).1, end);
    for sim in [&mut coupled, &mut reference, &mut unloaded] {
        sim.step_exact(1).unwrap();
    }
    assert_eq!(jaw(&coupled).1, jaw(&reference).1);
    assert!((jaw(&coupled).1.velocity[0] - jaw(&unloaded).1.velocity[0]).abs() > 1e-4);

    // Reset with a measured reaction still queued, before native consumes it.
    reset_loaded_world(&mut coupled, |_| ()).unwrap();
    coupled.step_exact(1).unwrap();
    reset_loaded_world(&mut coupled, |_| ()).unwrap();
    coupled.step_exact(1).unwrap();
    assert_eq!(jaw(&coupled).1, end);
}

#[test]
fn articulated_wrench_rejects_free_cuboid() {
    let free = include_str!("../../../examples/rotating_cuboid_cable.sdf")
        .replace("two_way", "articulated_wrench");
    let result = load_world_with_sdf_step(
        &free,
        "free.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    );
    assert!(
        result
            .err()
            .unwrap()
            .to_string()
            .contains("dynamic jointed cuboid")
    );
}

#[test]
fn articulated_normal_compliance_is_nonnegative_finite_and_explicitly_scoped() {
    for (mode, value) in [
        ("articulated_wrench", "NaN"),
        ("articulated_wrench", "-0.001"),
        ("two_way", "0.001"),
        ("one_way", "0"),
    ] {
        let plugin = PLUGIN.replace(
            r#"coupling="articulated_wrench""#,
            &format!(r#"coupling="{mode}" normal_compliance="{value}""#),
        );
        let result = load_world_with_sdf_step(
            &scene(&plugin),
            "arm.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        );
        assert!(
            result
                .err()
                .unwrap()
                .to_string()
                .contains("normal_compliance")
        );
    }
    let [rigid, compliant] = ["0", "0.03"].map(|value| {
        let plugin = PLUGIN.replace(
            r#"coupling="articulated_wrench""#,
            &format!(r#"coupling="articulated_wrench" normal_compliance="{value}""#),
        );
        let mut sim = load(&scene(&plugin));
        sim.step_exact(1).unwrap();
        -bodies(&sim).next().unwrap().1.last_boundary_impulses()[0].linear[0]
    });
    assert!(
        compliant > 0. && compliant < rigid,
        "compliant {compliant} rigid {rigid}"
    );
}

#[test]
fn articulated_pads_require_a_centered_mass_and_explicitly_supported_contact_geometry() {
    let xml = scene(PLUGIN);
    let collision = "<box>\n              <size>0.024 0.085 0.14</size>\n            </box>";
    let cases = [
        (
            xml.replacen(
                "<mass>0.18</mass>",
                "<pose>0.002 0 0 0 0 0</pose><mass>0.18</mass>",
                1,
            ),
            "centered mass",
        ),
        (
            xml.replacen(collision, "<sphere><radius>0.012</radius></sphere>", 1)
                .replacen("<box link=", "<sphere link=", 1),
            "dynamic jointed cuboid",
        ),
        (
            xml.replacen("<box link=", "<box stiffness=\"1000\" link=", 1),
            "unexpected content",
        ),
        (
            xml.replacen("<box link=", "<box normal_compliance=\"soft\" link=", 1),
            "invalid normal_compliance",
        ),
    ];
    for (xml, expected) in cases {
        let error = load_world_with_sdf_step(
            &xml,
            "pads.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .err()
        .unwrap()
        .to_string();
        assert!(error.contains(expected), "{error}");
    }
}
