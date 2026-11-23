// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "distributed-physics")]
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::entity_by_id;
use gz_sim_scene::{
    BodyHandle, JointHandle, JointSample, PhysicsEngine, distributed_floating_tree_spec,
    load_world_with_sdf_step, queue_joint_effort,
};
use std::error::Error;

const BRANCH: &str = include_str!("../../../examples/distributed_branched_tree.sdf");
const CHAIN: &str = include_str!("../../../examples/distributed_serial_tree.sdf");

#[test]
fn authored_branch_and_chain_rebuild_native_pose_twist_and_joint_state()
-> Result<(), Box<dyn Error>> {
    for (xml, n) in [(BRANCH, 5), (CHAIN, 6)] {
        let spec = distributed_floating_tree_spec(xml, "authored.sdf", "default")?;
        assert_eq!(spec.links.len(), n);
        assert_eq!(spec.joints.len() + 1, n);
        assert!(spec.links.windows(2).all(|p| p[0].id < p[1].id));
        assert!(
            spec.joints
                .windows(2)
                .any(|p| p[0].joint_id > p[1].joint_id),
            "fixture should distinguish solver and public order"
        );
        let load = || {
            load_world_with_sdf_step(
                xml,
                "test.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity,
            )
        };
        let mut regular = load()?;
        let mut recreated = load()?;
        for tick in 1..=320 {
            for (i, joint) in spec.joints.iter().enumerate() {
                if (tick + i) % 4 == 0 {
                    queue_joint_effort(
                        &mut regular,
                        joint.joint_id,
                        if i % 2 == 0 { 0.67 } else { -0.49 },
                    )?;
                }
            }
            regular.step_exact(1)?;
        }
        for (i, joint) in spec.joints.iter().enumerate() {
            if i != 1 {
                queue_joint_effort(&mut regular, joint.joint_id, (i as f64) - 1.5)?;
            }
        }
        let image = regular
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .checkpoint_floating_revolute_tree()?;
        {
            let mut b = recreated.world_mut().resource_mut::<PhysicsBackend>();
            b.downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .restore_floating_revolute_tree(&image)?;
        }
        for k in 0..=200 {
            let a = regular.world().resource::<PhysicsBackend>();
            let b = recreated.world().resource::<PhysicsBackend>();
            for l in &spec.links {
                let id = regular
                    .world()
                    .get::<BodyHandle>(entity_by_id(regular.world(), l.id).unwrap())
                    .unwrap()
                    .id();
                let other = recreated
                    .world()
                    .get::<BodyHandle>(entity_by_id(recreated.world(), l.id).unwrap())
                    .unwrap()
                    .id();
                assert_eq!(id, other);
                let s = a.state(id).unwrap();
                let t = b.state(id).unwrap();
                assert_eq!(s.position.map(f64::to_bits), t.position.map(f64::to_bits));
                assert_eq!(s.velocity.map(f64::to_bits), t.velocity.map(f64::to_bits));
                assert_eq!(
                    s.orientation.map(f64::to_bits),
                    t.orientation.map(f64::to_bits)
                );
                assert_eq!(
                    s.angular_velocity.map(f64::to_bits),
                    t.angular_velocity.map(f64::to_bits)
                );
            }
            for j in &spec.joints {
                let entity = entity_by_id(regular.world(), j.joint_id).unwrap();
                let id = regular.world().get::<JointHandle>(entity).unwrap().id();
                assert_eq!(a.joint_state(id), b.joint_state(id));
                let sample = regular.world().get::<JointSample>(entity).unwrap();
                assert_eq!(sample.state, a.joint_state(id).unwrap());
                if k != 0 {
                    let other = recreated
                        .world()
                        .get::<JointSample>(entity_by_id(recreated.world(), j.joint_id).unwrap())
                        .unwrap();
                    assert_eq!(sample.state, other.state);
                }
            }
            assert_eq!(
                a.downcast_ref::<MultiFidelityBackend>()
                    .unwrap()
                    .checkpoint_floating_revolute_tree()?,
                b.downcast_ref::<MultiFidelityBackend>()
                    .unwrap()
                    .checkpoint_floating_revolute_tree()?
            );
            if k < 200 {
                regular.step_exact(1)?;
                recreated.step_exact(1)?;
            }
        }
    }
    Ok(())
}

#[test]
fn tree_recipe_rejects_other_physics_and_unsupported_inputs() {
    let old = include_str!("../../../examples/multifidelity_articulation.sdf");
    for xml in [
        old.into(),
        BRANCH.replace("<self_collide>false", "<self_collide>true"),
        BRANCH.replace("<gravity>0 0 0", "<gravity>0 0 -9.8"),
        BRANCH.replace("0.001</max_step_size>", "0.010</max_step_size>"),
        BRANCH.replace(
            "</axis>",
            "<limit><lower>-1</lower><upper>1</upper></limit></axis>",
        ),
        BRANCH.replace("</collision>", "<surface/></collision>"),
    ] {
        assert!(distributed_floating_tree_spec(&xml, "excluded.sdf", "default").is_err());
    }
}

#[test]
fn authored_stops_bind_finite_axes_without_changing_public_or_solver_order()
-> Result<(), Box<dyn Error>> {
    use gz_sim_scene::distributed_stopped_tree_spec;
    const XML: &str = include_str!("../../../examples/distributed_stopped_tree.sdf");
    let spec = distributed_stopped_tree_spec(XML, "stops.sdf", "default")?;
    let original = distributed_floating_tree_spec(BRANCH, "branch.sdf", "default")?;
    assert_eq!(spec.tree, original);
    assert!(
        spec.limits
            .windows(2)
            .all(|w| w[0].joint_id < w[1].joint_id)
    );
    assert_eq!(spec.limits.len(), 3);
    assert!(distributed_stopped_tree_spec(BRANCH, "unlimited.sdf", "default").is_err());
    assert!(distributed_floating_tree_spec(XML, "stops.sdf", "default").is_err());
    for invalid in [
        XML.replace("<lower>-0.035</lower>", ""),
        XML.replace("<upper>0.055</upper>", ""),
        XML.replace(
            "<lower>-0.035</lower>",
            "<lower>-0.035</lower><lower>-0.035</lower>",
        ),
        XML.replace("<lower>-0.035</lower>", "<lower>0.01</lower>"),
        XML.replace("<upper>0.055</upper>", "<upper>-0.02</upper>"),
        XML.replace(
            "<upper>0.055</upper>",
            "<upper>0.055</upper><velocity>0.2</velocity>",
        ),
        XML.replace(
            "</model>",
            "<limit><lower>-0.2</lower><upper>0.3</upper></limit></model>",
        ),
    ] {
        assert!(distributed_stopped_tree_spec(&invalid, "invalid.sdf", "default").is_err());
    }
    let locked = XML.replace(
        "<lower>-0.035</lower><upper>0.055</upper>",
        "<lower>0</lower><upper>0</upper>",
    );
    let locked = distributed_stopped_tree_spec(&locked, "locked.sdf", "default")?;
    assert!(locked.limits.iter().any(|l| l.lower == 0. && l.upper == 0.));
    Ok(())
}

#[test]
fn externally_contacting_trees_require_the_authored_independent_body() -> Result<(), Box<dyn Error>>
{
    use gz_sim_scene::{distributed_grounded_tree_spec, distributed_payload_tree_spec};
    let leg = include_str!("../../../examples/distributed_grounded_leg.sdf");
    let gripper = include_str!("../../../examples/distributed_gripper_payload.sdf");
    let g = distributed_grounded_tree_spec(leg, "leg.sdf", "default")?;
    assert_eq!(g.tree.links.len(), 5);
    assert_eq!(g.limits.len(), 4);
    assert!(g.ground.gravity_z < 0. && g.ground.height == 0.);
    for bad in [
        leg.replace("0 0 -9.81</gravity>", "1 0 -9.81</gravity>"),
        leg.replace("<normal>0 0 1", "<normal>0 1 0"),
        leg.replace(
            "<collision name=\"terrain\">",
            "<collision name=\"terrain\"><pose>0.3 0 0 0 0 0</pose>",
        ),
    ] {
        assert!(distributed_grounded_tree_spec(&bad, "invalid.sdf", "default").is_err());
    }
    let p = distributed_payload_tree_spec(gripper, "gripper.sdf", "default")?;
    assert_eq!(p.tree.links.len(), 3);
    assert_eq!(p.payload.mass, 0.72);
    assert_eq!(p.payload.radius, 0.16);
    assert!(!p.payload_can_sleep);
    let sleeping = gripper.replace("<allow_auto_disable>false</allow_auto_disable>", "");
    assert!(distributed_payload_tree_spec(&sleeping, "sleep.sdf", "default")?.payload_can_sleep);
    for bad in [
        gripper.replace("0 0 0</gravity>", "0 0 -1</gravity>"),
        gripper.replace(
            "<collision name=\"ball\"><geometry><sphere><radius>0.16",
            "<collision name=\"ball\"><pose>0.1 0 0 0 0 0</pose><geometry><sphere><radius>0.16",
        ),
    ] {
        assert!(distributed_payload_tree_spec(&bad, "invalid.sdf", "default").is_err());
    }
    assert!(distributed_grounded_tree_spec(gripper, "gripper.sdf", "default").is_err());
    assert!(distributed_payload_tree_spec(leg, "leg.sdf", "default").is_err());
    Ok(())
}

#[test]
fn exact_zero_stops_are_authored_optional_and_bound_to_payload_images() -> Result<(), Box<dyn Error>>
{
    const XML: &str = include_str!("../../../examples/distributed_gripper_payload.sdf");
    let old_marker = "name=\"gz::sim::systems::SwitchableArticulation\"/>";
    let configured = |xml: &str, value: &str| {
        xml.replace(old_marker, &format!(
        "name=\"gz::sim::systems::SwitchableArticulation\"><exact_revolute_locks>{value}</exact_revolute_locks></plugin>"
    ))
    };
    let zero = XML.replacen(
        "<lower>-0.42</lower><upper>0.42</upper>",
        "<lower>0</lower><upper>0</upper>",
        1,
    );
    let marked = configured(&zero, "true");
    let spec = gz_sim_scene::distributed_payload_tree_spec(&marked, "marked.sdf", "default")?;
    assert!(spec.exact_revolute_locks);
    let plain = gz_sim_scene::distributed_payload_tree_spec(&zero, "plain.sdf", "default")?;
    assert!(!plain.exact_revolute_locks);
    let load = |s: &str| {
        load_world_with_sdf_step(
            s,
            "gripper.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
    };
    for bad in [
        configured(XML, "true"),
        configured(&zero, "t"),
        marked.replace(
            "</exact_revolute_locks>",
            "</exact_revolute_locks><exact_revolute_locks>false</exact_revolute_locks>",
        ),
    ] {
        assert!(load(&bad).is_err());
    }
    let mut a = load(&marked)?;
    let mut b = load(&zero)?;
    let mut a_restore = load(&marked)?;
    let mut b_restore = load(&zero)?;
    let id = spec
        .limits
        .iter()
        .find(|l| l.lower == 0. && l.upper == 0.)
        .unwrap()
        .joint_id;
    let joint = a
        .world()
        .get::<JointHandle>(entity_by_id(a.world(), id).unwrap())
        .unwrap()
        .id();
    for sim in [&mut a, &mut b] {
        for _ in 0..10 {
            queue_joint_effort(sim, id, 1.)?;
            sim.step_exact(1)?;
        }
    }
    let be = |sim: &gz_sim_core::Simulation| {
        sim.world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .checkpoint_payload_revolute_tree()
    };
    let ia = be(&a)?;
    let ib = be(&b)?;
    assert_ne!(&ia[40..72], &ib[40..72]);
    for (sim, wrong, right) in [(&mut a_restore, &ib, &ia), (&mut b_restore, &ia, &ib)] {
        let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
        let native = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
        assert!(native.restore_payload_revolute_tree(wrong).is_err());
        native.restore_payload_revolute_tree(right)?;
    }
    a.step_exact(1)?;
    a_restore.step_exact(1)?;
    assert_eq!(be(&a)?, be(&a_restore)?);
    let backend = a_restore.world().resource::<PhysicsBackend>();
    assert_eq!(backend.joint_state(joint).unwrap().position, 0.);
    assert_eq!(backend.joint_state(joint).unwrap().velocity, 0.);
    Ok(())
}
