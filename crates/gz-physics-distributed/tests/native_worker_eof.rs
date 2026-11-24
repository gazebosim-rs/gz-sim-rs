// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! An optional real-protocol capture used to record native worker coverage on graceful EOF.
use gz_physics::JointVelocityMotor;
use gz_physics_distributed::{
    articulation::ArticulationPool, floating_tree::*, moving::MovingPool,
    multi_sphere::MultiSpherePool,
};
use gz_sim_scene::distributed_hinge::*;
use std::{error::Error, path::Path};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn exercise<S: TreeConstruction>(exe: &Path, spec: S) -> Result<()> {
    let dir = tempfile::tempdir()?;
    let mut p = TreePool::create(exe, dir.path(), 3, spec)?;
    let ids = p
        .topology()
        .joints
        .iter()
        .map(|j| j.joint_id)
        .collect::<Vec<_>>();
    for &id in &ids {
        p.queue_effort(id, -0.02)?;
    }
    p.advance_with_efforts_at(0, 0, &ids.iter().map(|&id| (id, 0.04)).collect::<Vec<_>>())?;
    p.transfer(2)?;
    drop(p);
    let mut p = TreePool::<S>::resume(exe, dir.path())?;
    p.queue_effort(ids[0], -0.0)?;
    p.advance()?;
    p.reset()?;
    p.advance()?;
    Ok(())
}
#[test]
#[ignore = "requires a separately named real child protocol recorder"]
fn collect_native_managed_worker_protocols_for_graceful_eof() -> Result<()> {
    let path = std::env::var_os("GZ_COVERAGE_CHILD_PROXY").ok_or("set GZ_COVERAGE_CHILD_PROXY")?;
    let exe = Path::new(&path);
    let branch = include_str!("../../../examples/distributed_branched_tree.sdf");
    let stopped = include_str!("../../../examples/distributed_stopped_tree.sdf");
    let floor = include_str!("../../../examples/distributed_grounded_leg.sdf");
    let grasp = include_str!("../../../examples/distributed_gripper_payload.sdf");
    exercise(
        exe,
        distributed_floating_tree_spec(branch, "branch.sdf", "default")?,
    )?;
    exercise(
        exe,
        distributed_stopped_tree_spec(stopped, "stops.sdf", "default")?,
    )?;
    exercise(
        exe,
        distributed_grounded_tree_spec(floor, "floor.sdf", "default")?,
    )?;
    exercise(
        exe,
        distributed_motor_tree_spec(stopped, "motor.sdf", "default")?,
    )?;
    let payload = distributed_payload_tree_spec(grasp, "grip.sdf", "default")?;
    exercise(exe, payload.clone())?;
    let grip = GraspTreeSpec {
        payload: payload.clone(),
    };
    let d = tempfile::tempdir()?;
    let mut p = GraspTreePool::create(exe, d.path(), 3, grip.clone())?;
    let joint = payload.tree.joints[0].joint_id;
    p.set_velocity_motor(
        joint,
        Some(JointVelocityMotor {
            target_velocity: 2.,
            max_force: 0.8,
        }),
    )?;
    p.advance_with_payload_force_at(0, 0, &[(joint, 0.01)], [0.1, 0., 0.2])?;
    p.transfer(2)?;
    p.set_velocity_motor(joint, None)?;
    p.reset()?;
    drop(p);
    let d = tempfile::tempdir()?;
    let mut distant = grip;
    distant.payload.payload.position[0] = 1.1;
    distant.payload.payload.velocity = [-3., 0., 0.];
    let mut p = SplitGraspPool::create(
        exe,
        d.path(),
        3,
        distant,
        GraspPlacement::Adaptive { release_dwell: 1 },
    )?;
    let mut separate = false;
    let mut rejoined = false;
    for k in 0..250 {
        p.advance_with_payload_force_at(0, k, &[(joint, 0.01)], [0., 0.005, 0.])?;
        if p.owners()[0] != p.owners()[1] && !separate {
            separate = true;
            p.transfer_payload(2)?;
        }
        if separate && p.owners()[0] == p.owners()[1] {
            rejoined = true;
            break;
        }
    }
    assert!(separate && rejoined);
    p.reset()?;
    drop(p);
    let fleet = include_str!("../../../examples/distributed_grasp_fleet.sdf");
    let spec = GraspFleetSpec {
        assemblies: distributed_grasp_fleet_specs(fleet, "fleet.sdf", "default")?,
    };
    let d = tempfile::tempdir()?;
    let mut p = GraspFleetPool::create(
        exe,
        d.path(),
        3,
        spec.clone(),
        GraspFleetPlacement::RoundRobin,
    )?;
    p.advance()?;
    p.transfer(spec.assemblies[0].payload.tree.model_id, 2)?;
    p.reset()?;
    drop(p);
    let d = tempfile::tempdir()?;
    let mut p = CoupledGraspFleetPool::create(exe, d.path(), 3, spec.clone())?;
    p.advance()?;
    p.transfer(1)?;
    p.reset()?;
    drop(p);
    let d = tempfile::tempdir()?;
    let x = include_str!("../../../examples/distributed_hinge_impact.sdf");
    let s = distributed_moving_hinge_spec(x, "moving.sdf", "default")?;
    let mut p = MovingPool::create(exe, d.path(), 3, s)?;
    p.advance_with_effort(s.hinge.joint_id, 0.02)?;
    p.reset()?;
    drop(p);
    let d = tempfile::tempdir()?;
    let x = include_str!("../../../examples/distributed_hinge_two_spheres.sdf");
    let s = distributed_multi_sphere_spec(x, "three.sdf", "default")?;
    let mut p = MultiSpherePool::create(exe, d.path(), 3, s)?;
    p.advance_with_effort(s.hinge.joint_id, 0.02)?;
    p.reset()?;
    drop(p);
    let d = tempfile::tempdir()?;
    let x = include_str!("../../../examples/distributed_grounded_hinge.sdf");
    let s = distributed_hinge_spec(x, "hinge.sdf", "default")?;
    for ground in [s.ground, None] {
        let d = tempfile::tempdir_in(d.path())?;
        let mut spec = s;
        spec.ground = ground;
        let mut p = ArticulationPool::create(exe, d.path(), 3, spec)?;
        p.advance_with_effort(s.joint_id, 0.02)?;
        p.transfer(2)?;
        p.reset()?;
    }
    Ok(())
}

#[test]
#[ignore = "requires a separately named real child protocol recorder"]
fn collect_composite_humanoid_and_trajectory_protocols_for_graceful_eof() -> Result<()> {
    use gz_physics_distributed::grounded_robot::{
        GroundedRobotPool, RobotGraphEncoding, RobotPostureInterval, RobotPostureProgram,
    };
    use gz_sim_scene::deformable::articulated_impact::distributed_automatic_material_contact_tree_spec;
    use gz_sim_scene::{
        PhysicsEngine, WorldLoadOptions, load_world_configured, load_world_with_sdf_step,
    };
    let path = std::env::var_os("GZ_COVERAGE_CHILD_PROXY").ok_or("expected real protocol proxy")?;
    let exe = Path::new(&path);
    let cloth = include_str!("../../../examples/articulated_transient_cable.sdf");
    let recipe = distributed_automatic_material_contact_tree_spec(cloth, "cable.sdf", "default")?;
    let cs = ContactTreeSpec {
        tree: recipe.tree,
        limits: recipe.limits,
    };
    let s = AutomaticMaterialSpec {
        tree: cs.clone(),
        step_ns: recipe.step.as_nanos() as u64,
        links: recipe.links.clone(),
        restitution: recipe.restitution,
        material_name: recipe.material_name,
    };
    let d = tempfile::tempdir()?;
    let mut p = AutomaticMaterialPool::create(exe, d.path(), 2, s, recipe.material.clone())?;
    p.advance()?;
    assert!(p.last_impact().is_some());
    p.transfer(1)?;
    p.advance()?;
    p.reset()?;
    drop(p);
    let d = tempfile::tempdir()?;
    let mut tree = ContactTreePool::create(exe, d.path(), 2, cs)?;
    tree.advance()?;
    let sites = recipe
        .links
        .iter()
        .map(|&link| ArticulatedPointSite {
            link,
            local_point: [0.002, -0.001, 0.003],
        })
        .collect::<Vec<_>>();
    let cert = tree.sample_contact_points_at(0, 1, &sites)?;
    tree.advance_with_contact_impulses(&cert, &[[1e-6, -2e-6, 3e-6], [-2e-6, 3e-6, -1e-6]])?;
    tree.transfer(1)?;
    tree.advance()?;
    tree.reset()?;
    drop(tree);
    let xml = include_str!("../../../examples/humanoid_stand.sdf");
    let s = distributed_grounded_robot_spec(xml, "human.sdf", "default")?;
    let ordinary = load_world_with_sdf_step(
        xml,
        "human.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let feedback = ordinary
        .world()
        .resource::<gz_sim_scene::joint_posture::JointPostures>()
        .get(s.model_id)
        .unwrap();
    let program = RobotPostureProgram {
        law: feedback.law(),
        intervals: vec![
            RobotPostureInterval {
                targets: feedback.targets(),
                forces: vec![]
            };
            2
        ],
    };
    for encoding in [RobotGraphEncoding::Auto, RobotGraphEncoding::FullPages] {
        let d = tempfile::tempdir()?;
        let mut robot =
            GroundedRobotPool::create_with_encoding(exe, d.path(), 2, s.clone(), encoding)?;
        robot.queue_effort(s.joints[0].joint_id, 0.12)?;
        robot.advance_with_inputs_at(0, 0, &[], &[])?;
        if encoding == RobotGraphEncoding::Auto {
            robot.advance_posture_batch_at(0, 1, &program)?;
        }
        robot.transfer(1)?;
        robot.reset()?;
    }
    for xml in [
        include_str!("../../../examples/multifidelity_curved_shuttle.sdf"),
        include_str!("../../../examples/multifidelity_curved_cable_guide.sdf"),
    ] {
        let mut ordinary = load_world_configured(
            xml,
            "curve.sdf",
            "default",
            &sdformat::NoResources,
            WorldLoadOptions {
                engine: PhysicsEngine::MultiFidelity,
                step: None,
                noise_seed: [0; 32],
                distributed_worker: Some(exe),
            },
        )?;
        ordinary.step_exact(2)?;
        gz_sim_scene::trajectory_workers::select(&mut ordinary, 0, 2, 1)?;
        ordinary.step_exact(2)?;
    }
    Ok(())
}

#[test]
#[ignore = "requires a witnessed prior and an actual native child protocol recorder"]
fn collect_warm_and_replayed_gripper_promotions_for_graceful_eof() -> Result<()> {
    let path = std::env::var_os("GZ_COVERAGE_CHILD_PROXY").ok_or("expected real trace worker")?;
    let exe = Path::new(&path);
    let root =
        std::env::var_os("GZ_MOVING_LEGACY_ARTIFACTS").ok_or("expected witness archive root")?;
    for phase in [
        "multifidelity-adaptive-grasp-fleet",
        "multifidelity-warm-grasp-fleet",
        "multifidelity-verified-grasp-replay",
    ] {
        let d = tempfile::tempdir()?;
        std::fs::copy(
            Path::new(&root)
                .join(phase)
                .join("witness/prior.gzadaptivegraspfleet"),
            d.path().join("barrier.gzadaptivegraspfleet"),
        )?;
        let mut p = AdaptiveGraspFleetPool::resume(exe, d.path())?;
        let groups = p.spec().assemblies.clone();
        let loads = groups
            .iter()
            .map(|g| {
                let sign = -g.payload.payload.position[1].signum();
                let f = if phase == "multifidelity-adaptive-grasp-fleet" {
                    [0., 4. * sign, 0.03 * sign]
                } else {
                    [0., 3. * sign, 0.]
                };
                (g.payload.payload.id, f)
            })
            .collect::<Vec<_>>();
        let efforts = if phase == "multifidelity-adaptive-grasp-fleet" {
            vec![(groups[0].payload.tree.joints[0].joint_id, -0.0001)]
        } else {
            vec![]
        };
        let (epoch, tick) = (p.epoch(), p.tick());
        p.advance_with_inputs_at(epoch, tick, &efforts, &loads)?;
        assert!(p.is_coupled(), "{phase} did not actually join");
        p.advance()?;
    }
    Ok(())
}

#[test]
#[ignore = "requires a real v2-capability worker and physical receipt capture"]
fn collect_packed_v2_humanoid_graph_for_graceful_eof() -> Result<()> {
    use gz_physics_distributed::grounded_robot::{GroundedRobotPool, RobotGraphEncoding};
    let path =
        std::env::var_os("GZ_COVERAGE_CHILD_PROXY").ok_or("expected traced robot v2 worker")?;
    let xml = include_str!("../../../examples/humanoid_stand.sdf");
    let spec = distributed_grounded_robot_spec(xml, "robot.sdf", "default")?;
    let d = tempfile::tempdir()?;
    let mut p = GroundedRobotPool::create(Path::new(&path), d.path(), 2, spec)?;
    assert_eq!(p.encoding(), RobotGraphEncoding::Packed);
    p.advance_with_inputs_at(0, 0, &[], &[])?;
    p.transfer(1)?;
    p.advance_with_inputs_at(0, 1, &[], &[])?;
    p.reset()?;
    Ok(())
}

#[test]
#[ignore = "requires a separately named real child protocol recorder"]
fn collect_rotating_material_workers_for_graceful_eof() -> Result<()> {
    let proxy =
        std::env::var_os("GZ_COVERAGE_CHILD_PROXY").ok_or("expected child-protocol recorder")?;
    let exe = Path::new(&proxy);
    for (xml, warm) in [
        (
            include_str!("../../../examples/distributed_rotating_pad_cable.sdf"),
            26,
        ),
        (
            include_str!("../../../examples/distributed_rotating_pad_cloth.sdf"),
            15,
        ),
    ] {
        let r =
            gz_sim_scene::deformable::articulated_impact::distributed_rotor_material_contact_spec(
                xml,
                "rotor.sdf",
                "default",
            )?;
        let spec = AutomaticRotorMaterialSpec {
            pad: r.pad,
            step_ns: r.step.as_nanos() as u64,
            restitution: r.restitution,
            material_name: r.material_name,
        };
        let dir = tempfile::tempdir()?;
        let mut p = RotorMaterialPool::create(exe, dir.path(), 3, spec, r.material.clone())?;
        for _ in 0..warm {
            p.advance()?;
        }
        assert!(p.last_impact().is_some());
        p.transfer(2)?;
        drop(p);
        let mut p = RotorMaterialPool::resume(exe, dir.path(), r.material)?;
        p.advance()?;
        assert_eq!(p.tick(), warm + 1);
        p.reset()?;
        assert_eq!((p.tick(), p.epoch()), (0, 1));
    }
    Ok(())
}
