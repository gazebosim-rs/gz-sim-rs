// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{
    PhysicsEngine,
    damped_sole::{last_step, panel_steps},
    load_world_with_sdf_step, reset_loaded_world,
};
const SDF: &str = include_str!("../../../examples/worlds/local_compliant_upper.sdf");
fn load(xml: &str) -> Result<gz_sim_core::Simulation, gz_sim_scene::LoadError> {
    load_world_with_sdf_step(
        xml,
        "upper.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
#[test]
fn oblique_spring_and_loaded_sole_repeat_after_reset() {
    let mut sim = load(SDF).unwrap();
    let mut impulses = Vec::new();
    for run in 0..2 {
        let mut sum = 0.;
        let mut had_both = false;
        for i in 0..100 {
            if let Err(e) = sim.step_exact(1) {
                panic!(
                    "tick {i}: {e} {:?}",
                    sim.world().resource::<gz_sim_core::StepFault>().error()
                );
            }
            let upper = last_step(&sim).unwrap().upper_side.unwrap();
            sum -= upper.impulse.x;
            had_both |= upper.impulse.x < 0.
                && panel_steps(&sim).iter().any(|p| p.step.normal_impulse > 0.);
            assert_eq!(
                panel_steps(&sim)
                    .iter()
                    .find(|p| p.support == upper.support)
                    .unwrap()
                    .step
                    .normal_impulse,
                0.
            );
            assert!(
                (upper.end_elastic_energy_j - upper.end_force_n.powi(2) / 80000.).abs() < 1e-13
            );
            assert_eq!(upper.impulse.y, 0.);
            assert_eq!(upper.impulse.z, 0.);
        }
        assert!(sum > 0.5 && had_both);
        impulses.push(sum);
        if run == 0 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert!(last_step(&sim).is_none());
        }
    }
    assert_eq!(impulses[0], impulses[1]);
}

#[test]
fn bounded_miss_and_removed_target_leave_lower_support_active() {
    use gz_sim_core::EntityId;
    use gz_sim_scene::{Name, remove_model};
    let mut s = load(&SDF.replace("0.350 0 0.16", "0.350 0.65 0.16")).unwrap();
    for _ in 0..120 {
        s.step_exact(1).unwrap();
        let row = last_step(&s).unwrap();
        assert_eq!(row.upper_side.unwrap().peak_force_n, 0.);
        let n: f64 = panel_steps(&s).iter().map(|p| p.step.normal_impulse).sum();
        assert!((n - 0.0005 * 19.62).abs() < 1e-11);
    }
    #[allow(deprecated)]
    let ledge = s
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::ledge"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    remove_model(&mut s, ledge).unwrap();
    assert!(last_step(&s).unwrap().upper_side.is_none());
    s.step_exact(1).unwrap();
    assert!(last_step(&s).unwrap().upper_side.is_none());
    assert!(last_step(&s).unwrap().step.normal_impulse > 0.);
}
#[test]
fn edge_or_second_corner_and_unresolved_stiffness_refuse_atomically() {
    let two = SDF.replace("0 0.15 0.23", "0 0 0.23");
    // Raise the ledge so the leading upper corner passes just below its lower rim.
    let edge = SDF.replace("0.350 0 0.16", "0.350 0.115 0.335");
    let stiff = SDF.replace(
        "40000</upper_corner_stiffness>",
        "400000000</upper_corner_stiffness>",
    );
    let max_x =
        0.23_f64.cos() * (0.28 * 0.15_f64.cos() + 0.04 * 0.15_f64.sin()) + 0.07 * 0.23_f64.sin();
    let grazing = SDF
        .replace("0.350 0 0.16", &format!("{} 0 0.16", max_x + 0.0500005))
        .replace("0.65 0.20 0", "0 0.20 0");
    for (xml, reason) in [
        (grazing, "unresolved grazing"),
        (two, "one corner"),
        (edge, "one corner"),
        (stiff, "finer time grid"),
    ] {
        let mut sim = load(&xml).unwrap();
        use gz_physics::PhysicsBackend;
        use gz_physics_multifidelity::{MultiFidelityBackend, SoleFoot};
        use gz_sim_core::EntityId;
        use gz_sim_scene::{BodyHandle, Name, queue_link_world_wrench};
        #[allow(deprecated)]
        let find = |name: &str| {
            let e = sim
                .world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap();
            (
                e.get::<EntityId>().unwrap().get(),
                e.get::<BodyHandle>().unwrap().id(),
            )
        };
        let (upper_id, upper) = find("default::boot::upper");
        let (_, lower) = find("default::boot::sole");
        let (_, deck) = find("default::deck::plate");
        let (_, ledge) = find("default::ledge::side");
        let poses = |scene: &gz_sim_core::Simulation| {
            [upper, lower, deck, ledge]
                .map(|b| scene.world().resource::<PhysicsBackend>().state(b).unwrap())
        };
        let mut rejected = false;
        for _ in 0..130 {
            let before = sim.elapsed();
            let old = poses(&sim);
            queue_link_world_wrench(&mut sim, upper_id, [0., 0.01, 0.], [0.; 3]).unwrap();
            let initial = last_step(&sim).map(|p| (p.state.center, p.upper_side.unwrap().impulse));
            if sim.step_exact(1).is_err() {
                let fault = sim
                    .world()
                    .resource::<gz_sim_core::StepFault>()
                    .error()
                    .unwrap()
                    .to_string();
                assert!(fault.contains(reason), "{fault}");
                assert_eq!(before, sim.elapsed());
                assert_eq!(old, poses(&sim));
                let pending = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .unwrap()
                    .sole_wrench(SoleFoot::Weld { lower, upper }, [deck, ledge].into_iter())
                    .unwrap();
                assert_eq!(pending.0, [0., 0.01, 0.]);
                assert_eq!(
                    initial,
                    last_step(&sim).map(|p| (p.state.center, p.upper_side.unwrap().impulse))
                );
                rejected = true;
                break;
            }
        }
        assert!(rejected, "expected {reason}");
    }
}
#[test]
fn invalid_side_material_and_ownership_fail_on_load() {
    let start = SDF.find("      <link name=\"upper\">").unwrap();
    let end = SDF[start..].find("</joint>").unwrap() + start + "</joint>".len();
    let only_lower = format!("{}{}", &SDF[..start], &SDF[end..]);
    assert!(
        load(&only_lower)
            .err()
            .unwrap()
            .to_string()
            .contains("requires a second collision")
    );

    for xml in [
        SDF.replace(
            "<upper_negative_x_support>default::ledge::side",
            "<upper_negative_x_support>unknown",
        ),
        SDF.replace("<upper_corner_stiffness>40000</upper_corner_stiffness>", ""),
        SDF.replace(
            "40000</upper_corner_stiffness>",
            "-1</upper_corner_stiffness>",
        ),
        SDF.replace(
            "<substeps>8",
            "<flat_damped_fast_path>true</flat_damped_fast_path><substeps>8",
        ),
    ] {
        assert!(load(&xml).is_err());
    }
}

#[test]
fn moving_ledge_delivers_the_same_relative_impact_and_its_measured_work() {
    let twist = r#"<plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>0.12 0.08 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>"#;
    let xml = SDF
        .replace("<static>true</static>", twist)
        .replace("0.65 0.20 0", "0.77 0.28 0");
    let mut moving = load(&xml).unwrap();
    let mut still = load(SDF).unwrap();
    let mut work = 0.;
    for tick in 1..=120 {
        moving.step_exact(1).unwrap();
        still.step_exact(1).unwrap();
        let a = last_step(&moving).unwrap();
        let b = last_step(&still).unwrap();
        let u = a.upper_side.unwrap();
        assert!(u.impulse.distance(b.upper_side.unwrap().impulse) < 1e-11);
        assert!((u.support_work_j - 0.12 * u.impulse.x).abs() < 1e-15);
        let translation = glam::DVec3::new(0.12, 0.08, 0.) * (0.0005 * f64::from(tick));
        assert!((a.state.center - b.state.center).distance(translation) < 1e-12);
        work += u.support_work_j;
    }
    assert!(work < -0.27);
}
