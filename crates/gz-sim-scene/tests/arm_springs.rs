// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    JointSample, Name, PhysicsEngine, load_world_with_sdf_step, reset_loaded_world,
};

const ARM: &str = include_str!("../../../examples/local_manipulation_arm.sdf");
fn load(s: &str) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_with_sdf_step(
        s,
        "local_manipulation_arm.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}

#[allow(deprecated)]
#[test]
fn authored_lift_spring_moves_the_arm_at_rest_and_resets_its_physical_row() {
    let xml = ARM
        .replace("0 0 -9.81", "0 0 0")
        .replace("<spring_reference>-0.012", "<spring_reference>0.05")
        .replace("<spring_stiffness>2.0", "<spring_stiffness>20.0");
    let mut sim = load(&xml).unwrap();
    let id = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::arm::lift"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let mut repeat = Vec::new();
    for _ in 0..2 {
        sim.step_exact(1).unwrap();
        let e = entity_by_id(sim.world(), id).unwrap();
        let s = sim.world().get::<JointSample>(e).unwrap().state;
        // 1 N from the preloaded spring accelerates the 0.96 kg palm/jaws.
        assert!((s.velocity - 0.001 / 0.96).abs() < 0.000015, "{s:?}");
        repeat.push(s);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
    }
    assert_eq!(repeat[0], repeat[1]);
}

#[test]
fn unsupported_or_invalid_sdf_passive_fields_are_rejected() {
    for (a, b) in [
        ("<damping>0.8", "<damping>-1"),
        ("<spring_stiffness>2.0", "<spring_stiffness>-1"),
        ("<spring_stiffness>2.0", "<spring_stiffness>0"),
        (
            "<damping>0.8</damping>",
            "<damping>0.8</damping><friction>1</friction>",
        ),
    ] {
        assert!(load(&ARM.replace(a, b)).is_err(), "{b}");
    }
}

#[test]
fn extra_constraint_passes_accept_boundaries_and_reject_bad_budgets_or_duplicates() {
    let field = "<additional_pgs_iterations>41</additional_pgs_iterations>";
    assert!(ARM.contains(field));
    for value in ["0", "128"] {
        assert!(
            load(&ARM.replace(
                field,
                &format!("<additional_pgs_iterations>{value}</additional_pgs_iterations>")
            ))
            .is_ok()
        );
    }
    for value in ["-1", "129", "256", "1.5", "", "NaN"] {
        assert!(
            load(&ARM.replace(
                field,
                &format!("<additional_pgs_iterations>{value}</additional_pgs_iterations>")
            ))
            .is_err()
        );
    }
    assert!(load(&ARM.replace(field, &format!("{field}{field}"))).is_err());
}

#[test]
fn precision_option_requires_world_mounted_mixed_robot() {
    let other = include_str!("../../../examples/multifidelity_articulation.sdf");
    let tag = r#"filename="gz-sim-switchable-articulation-system""#;
    assert!(other.contains(tag));
    let start = other.find(tag).unwrap() + tag.len();
    let end = other[start..].find("/>").unwrap() + start + 2;
    let xml = format!(
        "{}><additional_pgs_iterations>41</additional_pgs_iterations></plugin>{}",
        &other[..end - 2],
        &other[end..]
    );
    let world = sdformat::load_str(&xml, "floating.sdf", &sdformat::NoResources).unwrap();
    let name = &world.worlds[0].name;
    let err = load_world_with_sdf_step(
        &xml,
        "floating.sdf",
        name,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .err()
    .unwrap();
    assert!(
        err.to_string().contains("additional PGS iterations"),
        "{err}"
    );
}
