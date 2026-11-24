// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics::{PhysicsBackend, PhysicsError};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    JointHandle, LoadError, Name, PhysicsEngine, WorldPose, joint_posture::JointPostures,
    load_world_with_sdf_step, queue_link_world_wrench, reset_loaded_world,
};

const ROBOT: &str = include_str!("../../../examples/humanoid_stand.sdf");
const WALK: &str = r#"<footsteps motion="walk" lateral_m="0.08" length_m="0.10" count="2" placement_feedback="true" support_relative="true" heading_gain="0.6"/>"#;
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn fixture() -> String {
    ROBOT.replacen("<actuator", &format!("{WALK}\n      <actuator"), 1)
}
fn load(xml: &str) -> std::result::Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        xml,
        "walk.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
fn public_id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find_map(|e| (e.get::<Name>()?.0 == name).then(|| e.get::<EntityId>().unwrap().get()))
        .unwrap()
}
fn heading(sim: &Simulation, id: u64) -> f64 {
    let node = entity_by_id(sim.world(), id).unwrap();
    let pose = sim.world().get::<WorldPose>(node).unwrap();
    let forward = DQuat::from_array(pose.rotation) * DVec3::X;
    forward.y.atan2(forward.x)
}
fn hips(sim: &Simulation) -> [u64; 2] {
    [
        "default::humanoid::left_hip_yaw",
        "default::humanoid::right_hip_yaw",
    ]
    .map(|name| public_id(sim, name))
}
fn targets(sim: &Simulation, hip_ids: [u64; 2]) -> [f64; 2] {
    let model = public_id(sim, "default::humanoid");
    let goals = sim
        .world()
        .resource::<JointPostures>()
        .get(model)
        .unwrap()
        .targets();
    hip_ids.map(|id| {
        goals
            .axes
            .iter()
            .find(|goal| goal.joint_id == id)
            .unwrap()
            .target_rad
    })
}
fn positions(sim: &Simulation, hip_ids: [u64; 2]) -> [f64; 2] {
    hip_ids.map(|id| {
        let entity = entity_by_id(sim.world(), id).unwrap();
        let joint = sim.world().get::<JointHandle>(entity).unwrap().id();
        sim.world()
            .resource::<PhysicsBackend>()
            .joint_state(joint)
            .unwrap()
            .position
    })
}
#[test]
fn pelvis_and_swing_foot_headings_set_hip_targets_and_reproduce_after_reset() -> Result<()> {
    let mut sim = load(&fixture())?;
    let links = ["pelvis", "left_foot", "right_foot"]
        .map(|name| public_id(&sim, &format!("default::humanoid::{name}")));
    let hip_ids = hips(&sim);
    let initial = links.map(|id| heading(&sim, id));
    queue_link_world_wrench(&mut sim, links[0], [0.; 3], [0., 0., 20.])?;
    sim.step_exact(1)?;
    let mut initial_step = [0.; 2];
    // First stride: the left foot remains the stance foot. Sample the start,
    // midpoint and end of the right-foot heading transfer.
    for (phase, airborne) in [(1u64, 0.), (2725, 0.5), (3100, 1.)] {
        let current = sim.elapsed().as_millis() as u64;
        if phase > current {
            sim.step_exact(phase - current)?;
        }
        let absolute = links.map(|id| heading(&sim, id));
        let error = std::array::from_fn::<_, 3, _>(|i| {
            let d = absolute[i] - initial[i];
            d.sin().atan2(d.cos())
        });
        let q = positions(&sim, hip_ids);
        let correction = [error[0], (1. - airborne) * error[0] - airborne * error[2]];
        assert!(error[0].abs() > 1e-8);
        if airborne > 0. {
            assert!(error[2].abs() > 1e-8);
        }
        sim.step_exact(1)?;
        let actual = targets(&sim, hip_ids);
        for side in 0..2 {
            let expected = q[side] + 0.6 * correction[side];
            assert!(
                (actual[side] - expected).abs() < 1e-12,
                "phase {phase}, hip {side}: {} != {expected}",
                actual[side]
            );
        }
        if phase == 1 {
            initial_step = actual;
        }
        eprintln!(
            "phase={phase} pelvis_yaw={} right_foot_yaw={} q={q:?} targets={actual:?}",
            error[0], error[2]
        );
    }
    reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(links.map(|id| heading(&sim, id)), initial);
    let model = public_id(&sim, "default::humanoid");
    assert!(
        sim.world()
            .resource::<JointPostures>()
            .get(model)
            .unwrap()
            .targets()
            .axes
            .is_empty()
    );
    queue_link_world_wrench(&mut sim, links[0], [0.; 3], [0., 0., 20.])?;
    sim.step_exact(2)?;
    assert_eq!(targets(&sim, hip_ids), initial_step);
    Ok(())
}
#[test]
fn yaw_request_saturates_at_the_authored_hip_limit() -> Result<()> {
    let xml = fixture();
    let (prefix, hip) = xml.split_once("<joint name=\"right_hip_yaw\"").unwrap();
    let narrow = hip.replacen(
        "<lower>-0.65</lower><upper>0.65</upper>",
        "<lower>-0.0000001</lower><upper>0.0000001</upper>",
        1,
    );
    assert_ne!(hip, narrow);
    let mut sim = load(&format!("{prefix}<joint name=\"right_hip_yaw\"{narrow}"))?;
    let root = public_id(&sim, "default::humanoid::pelvis");
    let hip_ids = hips(&sim);
    let start = heading(&sim, root);
    for impulse_sign in [-1., 1.] {
        queue_link_world_wrench(&mut sim, root, [0.; 3], [0., 0., 20. * impulse_sign])?;
        sim.step_exact(1)?;
        let desired = positions(&sim, hip_ids)[1] + 0.6 * (heading(&sim, root) - start);
        assert!(desired * impulse_sign > 1e-7);
        sim.step_exact(1)?;
        let bounded = targets(&sim, hip_ids)[1];
        assert_eq!(bounded, impulse_sign * 1e-7);
        eprintln!("unbounded_yaw_request={desired} bounded_target={bounded}");
        reset_loaded_world(&mut sim, |_| ())?;
    }
    Ok(())
}
#[test]
fn invalid_heading_gains_and_hips_fail_during_sdf_load() {
    let xml = fixture();
    for invalid in [
        xml.replace("heading_gain=\"0.6\"", "heading_gain=\"-0.01\""),
        xml.replace("heading_gain=\"0.6\"", "heading_gain=\"1.01\""),
        xml.replace("support_relative=\"true\"", "support_relative=\"false\""),
    ] {
        assert!(matches!(
            load(&invalid),
            Err(LoadError::JointPosture { .. })
        ));
    }
    for side in ["left", "right"] {
        let invalid = xml.replace(&format!("{side}_hip_yaw"), &format!("{side}_other_axis"));
        assert!(matches!(
            load(&invalid),
            Err(LoadError::Physics(PhysicsError::MissingJoint))
        ));
    }
    let axis = r#"<pose relative_to="left_hip_yaw_housing">0.00000 0.00000 0.00000 0 0 0</pose><axis><xyz>0.00000 0.00000 1.00000</xyz>"#;
    assert!(xml.contains(axis));
    assert!(matches!(
        load(&xml.replace(axis, &axis.replace("1.00000</xyz>", "-1.00000</xyz>"))),
        Err(LoadError::Physics(PhysicsError::InvalidValue))
    ));
}

#[test]
fn a_walk_cannot_rebind_or_drive_with_a_retired_heading_body() -> Result<()> {
    use gz_sim_scene::{
        BodyHandle,
        joint_posture::footsteps::{FootstepController, FootstepMotion, FootstepPlan},
    };
    let xml = fixture();
    let sdf = sdformat::load_str(&xml, "walk.sdf", &sdformat::NoResources)?;
    let world = &sdf.worlds[0];
    let model = world.models.iter().find(|m| m.name == "humanoid").unwrap();
    let plan = FootstepPlan {
        motion: FootstepMotion::Walk,
        lateral_m: 0.08,
        length_m: 0.10,
        count: 2,
        placement_feedback: true,
        support_relative: true,
        lateral_correction_limit_m: None,
        heading_gain: Some(0.6),
        recovery_landing: false,
    };
    for name in ["pelvis", "left_foot", "right_foot"] {
        let mut sim = load(&xml)?;
        let model_id = public_id(&sim, "default::humanoid");
        let rule = FootstepController::bind(&sdf, world, model, &sim, plan)?;
        let mut controller = sim
            .world()
            .resource::<JointPostures>()
            .get(model_id)
            .unwrap()
            .clone();
        let id = public_id(&sim, &format!("default::humanoid::{name}"));
        let entity = entity_by_id(sim.world(), id).unwrap();
        let body = sim.world().get::<BodyHandle>(entity).unwrap().id();
        // A cached physical walking program can outlive a native body. Both
        // binding and driving use the live backend, even while an ECS ID exists.
        assert!(
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .remove(body)
        );
        assert!(matches!(
            FootstepController::bind(&sdf, world, model, &sim, plan),
            Err(LoadError::Physics(PhysicsError::MissingBody))
        ));
        assert!(matches!(
            rule.drive(&mut controller, sim.world().resource::<PhysicsBackend>(), 1),
            Err(PhysicsError::MissingBody)
        ));
    }
    Ok(())
}
