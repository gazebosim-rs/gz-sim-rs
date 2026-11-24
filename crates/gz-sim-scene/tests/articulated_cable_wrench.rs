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

#[test]
fn measured_pad_loads_keep_authored_ids_after_boundary_removal_and_clear_on_reset() {
    let plugin = PLUGIN
        .replacen(
            "<box link=",
            "<sphere link=\"default::far::ball\" coupling=\"one_way\"/>\n<box link=",
            1,
        )
        .replace(
            "coupling=\"articulated_wrench\"",
            "coupling=\"articulated_wrench\" normal_compliance=\"0.001\"",
        )
        .replace("<cable name=", "<cable pad_loads=\"true\" name=");
    let far = r#"<model name="far"><static>true</static><pose>10 0 0 0 0 0</pose><link name="ball"><collision name="ball"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model>"#;
    let xml = scene(&plugin).replace("</world>", &format!("{far}</world>"));
    let model = |sim: &Simulation, name: &str| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get()
    };
    let mut sim = load(&xml);
    let id = model(&sim, "default::far");
    gz_sim_scene::remove_model(&mut sim, id).unwrap();
    sim.step_exact(1).unwrap();
    let sample = bodies(&sim).next().unwrap().1;
    let first = sample.last_pad_loads().to_vec();
    assert_eq!(first.len(), 2);
    let mut total = DVec3::ZERO;
    for (i, load) in first.iter().enumerate() {
        assert_eq!((load.boundary, load.particle), (1, i));
        assert!(load.normal[0] < 0.);
        assert_eq!(load.tangent, [0.; 3]);
        total += DVec3::from(load.normal);
    }
    assert!(total.distance(DVec3::from(sample.last_boundary_impulses()[1].linear)) < 1e-15);
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(bodies(&sim).next().unwrap().1.last_pad_loads(), first);

    let id = model(&sim, "default::arm");
    gz_sim_scene::remove_model(&mut sim, id).unwrap();
    sim.step_exact(1).unwrap();
    assert!(bodies(&sim).next().unwrap().1.last_pad_loads().is_empty());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(bodies(&sim).next().unwrap().1.last_pad_loads().is_empty());
    sim.step_exact(1).unwrap();
    assert_eq!(bodies(&sim).next().unwrap().1.last_pad_loads(), first);

    let malformed = xml.replace("pad_loads=\"true\"", "pad_loads=\"all\"");
    assert!(
        load_world_with_sdf_step(
            &malformed,
            "bad.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity
        )
        .err()
        .unwrap()
        .to_string()
        .contains("pad_loads must be true or false")
    );
}

#[test]
fn changing_material_resolution_preserves_live_observations_and_reset_restores_authored_clock() {
    use gz_sim_scene::deformable::{MaterialResolutionError::*, set_material_resolution};
    let mut sim = load(&scene(PLUGIN));
    let mut control = load(&scene(PLUGIN));
    for s in [&mut sim, &mut control] {
        s.step_exact(1).unwrap();
    }
    let sample = |s: &Simulation| {
        let b = bodies(s).next().unwrap().1;
        (
            s.elapsed(),
            jaw(s).1,
            b.particles().to_vec(),
            b.last_boundary_impulses().to_vec(),
        )
    };
    let initial = sample(&sim);
    assert_eq!(
        set_material_resolution(&mut sim, "absent", 2, 8),
        Err(UnknownMaterial)
    );
    for (n, i) in [(0, 8), (2, 0), (1_000_001, 8)] {
        assert_eq!(
            set_material_resolution(&mut sim, "cable", n, i),
            Err(InvalidResolution)
        );
    }
    set_material_resolution(&mut sim, "cable", 4, 16).unwrap();
    assert_eq!(sample(&sim), initial);
    sim.step_exact(1).unwrap();
    control.step_exact(1).unwrap();
    assert_eq!(jaw(&sim), jaw(&control)); // the pending native wrench was identical
    assert_ne!(sample(&sim).2, sample(&control).2);
    for s in [&mut sim, &mut control] {
        reset_loaded_world(s, |_| ()).unwrap();
        s.step_exact(2).unwrap();
    }
    assert_eq!(sample(&sim), sample(&control));
    let mut empty = load(ARM);
    assert_eq!(
        set_material_resolution(&mut empty, "cable", 2, 8),
        Err(UnknownMaterial)
    );
    empty
        .world()
        .resource::<gz_sim_core::StepFault>()
        .report(std::io::Error::other("simulation fault"));
    assert_eq!(
        set_material_resolution(&mut empty, "cable", 2, 8),
        Err(Faulted)
    );
}

#[test]
fn coupled_cables_keep_the_same_clock_while_each_can_change_iterations() {
    use gz_sim_scene::deformable::{MaterialResolutionError::SharedClock, set_material_resolution};
    let cable = &PLUGIN[PLUGIN.find("<cable").unwrap()..PLUGIN.find("</plugin>").unwrap()];
    let two = PLUGIN.replace(
        "</plugin>",
        &format!(
            "{}</plugin>",
            cable.replace("name=\"cable\"", "name=\"other\"")
        ),
    );
    let mut sim = load(&scene(&two));
    assert_eq!(
        set_material_resolution(&mut sim, "other", 4, 2),
        Err(SharedClock)
    );
    assert_eq!(
        set_material_resolution(&mut sim, "missing", 4, 2),
        Err(gz_sim_scene::deformable::MaterialResolutionError::UnknownMaterial)
    );
    set_material_resolution(&mut sim, "other", 2, 2).unwrap();
    sim.step_exact(1).unwrap();
}

#[test]
fn self_contact_observations_require_an_enabled_material_surface() {
    let xml = include_str!("../../../examples/sliding_folded_cloth.sdf");
    for (attr, ok) in [("false", true), ("true", true), ("bogus", false)] {
        let src = xml.replace(
            "<cloth name=",
            &format!("<cloth self_contact_loads=\"{attr}\" name="),
        );
        let result = load_world_with_sdf_step(
            &src,
            "fold.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        );
        assert_eq!(result.is_ok(), ok, "{attr} {:?}", result.err());
    }
    let src = xml.replace("<cloth name=", "<cloth self_contact_loads=\"true\" name=");
    let start = src.find("self_friction=\"").unwrap();
    let end = start
        + "self_friction=\"".len()
        + src[start + "self_friction=\"".len()..].find('"').unwrap()
        + 1;
    let mut no_friction = src;
    no_friction.replace_range(start..end, "");
    let err = load_world_with_sdf_step(
        &no_friction,
        "fold.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .err()
    .unwrap();
    assert!(err.to_string().contains("self_contact_loads"));
}

fn sampled_sheet(sample: &str) -> String {
    let box_link = r#"<box link="default::arm::left" coupling="articulated_wrench" normal_compliance="0.001"/>"#;
    let plugin = format!(
        r#"<plugin name="gz::sim::systems::Deformables" filename="gz-sim-deformables-system">{box_link}
        <cloth name="sheet" columns="2" stretch="0.0001" shear="0.0001" bend="0.01" substeps="2" iterations="8" radius="0.005" self_contact="false" friction="0" pad_loads="true">
        <particle position=".2273 .015 .197" velocity="-1 0 0" mass=".004"/>
        {sample}
        <particle position=".2273 .03 .197" velocity="-1 0 0" mass=".004"/>
        <particle position=".2273 .015 .210" velocity="-1 0 0" mass=".004"/>
        <particle position=".2273 .03 .210" velocity="-1 0 0" mass=".004"/>
        </cloth></plugin>"#
    );
    scene(&plugin)
}
const SAMPLE: &str =
    r#"<pad_sample particles="0 1 3" weights="0.25 0.25 0.5" stiffness_weight="2"/>"#;

#[test]
fn authored_cloth_patch_transfers_the_measured_jaw_reaction_and_repeats_after_reset() {
    let mut sim = load(&sampled_sheet(SAMPLE));
    let measure = |sim: &mut Simulation| {
        sim.step_exact(1).unwrap();
        let (_, sheet) = bodies(sim).next().unwrap();
        let loads = sheet.last_pad_loads();
        let impulse = DVec3::from(sheet.last_boundary_impulses()[0].linear);
        assert!(impulse.x < -1e-5, "{impulse:?}");
        assert_eq!(loads.len(), 3);
        for (load, (node, weight)) in loads.iter().zip([(0, 0.25), (1, 0.25), (3, 0.5)]) {
            assert_eq!((load.boundary, load.particle), (0, node));
            assert!(DVec3::from(load.normal).distance(impulse * weight) < 1e-12);
        }
        impulse
    };
    let first = measure(&mut sim);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(bodies(&sim).next().unwrap().1.last_pad_loads().is_empty());
    assert_eq!(measure(&mut sim), first);
}

#[test]
fn malformed_material_patch_is_rejected_at_scene_load() {
    for (sample, reason) in [
        (SAMPLE.replace("0 1 3", "0 1"), "three unsigned"),
        (SAMPLE.replace("0 1 3", "0 1 3 2"), "exactly three"),
        (SAMPLE.replace("0 1 3", "0 -1 3"), "three unsigned"),
        (SAMPLE.replace("0 1 3", "0 1 2"), "invalid pad_sample"),
        (
            SAMPLE.replace("0.25 0.25 0.5", "1 NaN 0"),
            "finite coordinates",
        ),
        (
            SAMPLE.replace("stiffness_weight", "weight"),
            "unexpected content",
        ),
        (
            SAMPLE.replace(r#" stiffness_weight="2""#, ""),
            "missing stiffness_weight",
        ),
        (
            SAMPLE.replace(r#"stiffness_weight="2""#, r#"stiffness_weight="0""#),
            "invalid pad_sample",
        ),
    ] {
        let result = load_world_with_sdf_step(
            &sampled_sheet(&sample),
            "sampled.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        );
        let err = result.err().unwrap().to_string();
        assert!(err.contains(reason), "{err}");
    }
    let cable = PLUGIN.replace("</cable>", &format!("{SAMPLE}</cable>"));
    let load = load_world_with_sdf_step(
        &scene(&cable),
        "sampled.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    );
    assert!(
        load.err()
            .unwrap()
            .to_string()
            .contains("unexpected <pad_sample>")
    );
}

#[test]
fn patch_reactions_on_both_jaws_change_the_opening_under_identical_motor_efforts() {
    use gz_sim_scene::queue_joint_effort;
    // Two mirror-image material faces load opposite jaws. The articulated arm,
    // gravity and both joint efforts are identical in all three worlds.
    let mut xml = sampled_sheet(SAMPLE);
    let begin = xml.find("<cloth ").unwrap();
    let end = xml[begin..].find("</cloth>").unwrap() + begin + "</cloth>".len();
    let right = xml[begin..end]
        .replace(r#"name="sheet""#, r#"name="mirror""#)
        .replace(".2273 ", ".4127 ")
        .replace(r#"velocity="-1 "#, r#"velocity="1 "#);
    xml.insert_str(end, &right);
    xml.insert_str(begin, r#"<box link="default::arm::right" coupling="articulated_wrench" normal_compliance="0.001"/>"#);
    let mut coupled = load(&xml);
    let mut replayed_wrench = load(ARM);
    let mut empty = load(ARM);
    let named = |s: &Simulation, name: &str| {
        s.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
            .id()
    };
    let state = |s: &Simulation, name| {
        let ent = named(s, name);
        s.world()
            .resource::<PhysicsBackend>()
            .state(s.world().get::<BodyHandle>(ent).unwrap().id())
            .unwrap()
    };
    let links = ["default::arm::left", "default::arm::right"];
    let start = links.map(|name| state(&coupled, name));
    let drive = |s: &mut Simulation| {
        for name in ["default::arm::close_left", "default::arm::close_right"] {
            let id = s.world().get::<EntityId>(named(s, name)).unwrap().get();
            queue_joint_effort(s, id, 4.).unwrap();
        }
        s.step_exact(1).unwrap();
    };
    for s in [&mut coupled, &mut replayed_wrench, &mut empty] {
        drive(s);
    }
    let end = links.map(|name| state(&coupled, name));
    assert_eq!(end, links.map(|name| state(&empty, name)));
    for (j, name) in links.into_iter().enumerate() {
        let mut force = DVec3::ZERO;
        let mut torque = DVec3::ZERO;
        for (_, sheet) in bodies(&coupled) {
            let reaction = sheet.last_boundary_impulses()[j];
            let impulse = DVec3::from(reaction.linear);
            force += impulse / 0.001;
            torque += (DVec3::from(reaction.angular)
                + (DVec3::from(start[j].position) - DVec3::from(end[j].position)).cross(impulse))
                / 0.001;
        }
        assert!(
            if j == 0 {
                force.x < -0.01
            } else {
                force.x > 0.01
            },
            "{name}: {force:?}"
        );
        let id = replayed_wrench
            .world()
            .get::<EntityId>(named(&replayed_wrench, name))
            .unwrap()
            .get();
        queue_link_world_wrench(
            &mut replayed_wrench,
            id,
            force.to_array(),
            torque.to_array(),
        )
        .unwrap();
    }
    for s in [&mut coupled, &mut replayed_wrench, &mut empty] {
        drive(s);
    }
    let actual = links.map(|name| state(&coupled, name));
    assert_eq!(actual, links.map(|name| state(&replayed_wrench, name)));
    let unloaded = links.map(|name| state(&empty, name));
    let opening = |jaws: [BodyState; 2]| jaws[1].position[0] - jaws[0].position[0] - 0.024;
    assert!(opening(actual) > opening(unloaded) + 1e-7);
}

#[test]
fn pad_sensor_sums_material_wrenches_at_native_com_and_expires_on_reset_removal_or_fault() {
    use gz_sim_scene::deformable::{ArticulatedPadWrench, articulated_pad_wrench};
    let cable = &PLUGIN[PLUGIN.find("<cable").unwrap()..PLUGIN.find("</plugin>").unwrap()];
    let plug = PLUGIN.replace(
        "</plugin>",
        &format!(
            "{}</plugin>",
            cable.replace("name=\"cable\"", "name=\"second\"")
        ),
    );
    let xml = scene(&plug);
    let mut sim = load(&xml);
    let (id, start) = jaw(&sim);
    let lookup = |s: &Simulation, name| {
        s.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get()
    };
    let right = lookup(&sim, "default::arm::right");
    let arm = lookup(&sim, "default::arm");
    assert_eq!(articulated_pad_wrench(&sim, id), None);
    assert_eq!(articulated_pad_wrench(&load(ARM), id), None);
    sim.step_exact(1).unwrap();
    let (_, end) = jaw(&sim);
    let (mut j, mut l) = (DVec3::ZERO, DVec3::ZERO);
    assert_eq!(bodies(&sim).count(), 2);
    for (_, material) in bodies(&sim) {
        let r = material.last_boundary_impulses()[0];
        assert!(r.linear[0] < -0.001 && r.angular[2] > 1e-5);
        j += DVec3::from(r.linear);
        l += DVec3::from(r.angular);
    }
    let about_end = l + (DVec3::from(start.position) - DVec3::from(end.position)).cross(j);
    let expected = ArticulatedPadWrench {
        force_n: (j / 0.001).into(),
        torque_nm: (about_end / 0.001).into(),
    };
    assert_eq!(articulated_pad_wrench(&sim, id), Some(expected));
    assert_eq!(
        articulated_pad_wrench(&sim, right),
        Some(Default::default())
    );
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(articulated_pad_wrench(&sim, id), Some(expected));
    gz_sim_scene::remove_model(&mut sim, arm).unwrap();
    assert_eq!(articulated_pad_wrench(&sim, id), None);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(articulated_pad_wrench(&sim, id), None);
    sim.step_exact(1).unwrap();
    assert_eq!(articulated_pad_wrench(&sim, id), Some(expected));
    sim.world()
        .resource::<gz_sim_core::StepFault>()
        .report(std::io::Error::other("failed step"));
    assert_eq!(articulated_pad_wrench(&sim, id), None);
}
