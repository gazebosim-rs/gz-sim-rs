// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "distributed-physics")]
#[path = "support/grasp_momentum.rs"]
mod grasp_momentum;
use glam::DVec3;
use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Inertia, JointId, JointVelocityMotor,
    PhysicsBackend, RotationSpec, Shape,
};
use gz_physics_distributed::articulation::ArticulationLinkSpec;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointHandle, PhysicsEngine, distributed_payload_tree_spec, load_world_with_sdf_step,
};
use std::{error::Error, time::Duration};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const XML: &str = include_str!("../../../examples/distributed_gripper_payload.sdf");
const DT: Duration = Duration::from_millis(1);

fn load(xml: &str) -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        xml,
        "release.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
fn spawn_payload(be: &mut MultiFidelityBackend, s: &ArticulationLinkSpec) -> Result<BodyId> {
    let id = be.spawn_link(
        BodySpec {
            position: s.position,
            velocity: s.velocity,
            mass: Some(s.mass),
            shape: Shape::Sphere { radius: s.radius },
        },
        RotationSpec {
            orientation: s.orientation,
            angular_velocity: s.angular_velocity,
        },
        Some(Inertia {
            center: s.inertia_center,
            orientation: s.inertia_orientation,
            tensor: s.inertia_tensor,
        }),
    )?;
    be.keep_body_awake(id)?;
    Ok(id)
}
fn force(t: u64) -> f64 {
    match t {
        650..850 | 1250..1450 => 1.2,
        850..1250 => -1.2,
        _ => 0.,
    }
}
fn command(be: &mut PhysicsBackend, joints: [JointId; 2], t: u64) -> Result<()> {
    if matches!(t, 0 | 600 | 1450) {
        let speed = if t == 600 { -6. } else { 6. };
        for (id, s) in joints.into_iter().zip([speed, -speed]) {
            be.set_joint_velocity_motor(
                id,
                Some(JointVelocityMotor {
                    target_velocity: s,
                    max_force: 4.,
                }),
            )?;
        }
    }
    Ok(())
}
fn exact_state(s: BodyState) -> [u64; 13] {
    let mut bits = [0; 13];
    for (dst, v) in bits.iter_mut().zip(
        s.position
            .into_iter()
            .chain(s.orientation)
            .chain(s.velocity)
            .chain(s.angular_velocity),
    ) {
        *dst = v.to_bits();
    }
    bits
}
fn solver(sim: &Simulation) -> &MultiFidelityBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
}
#[test]
fn empty_fingers_and_free_payload_separate_then_recontact_the_native_tree() -> Result<()> {
    let spec = distributed_payload_tree_spec(XML, "gripper.sdf", "default")?;
    let (before, _) = XML.rsplit_once("<model name=\"payload\">").unwrap();
    let bare = format!("{before}</world></sdf>");
    let mut reference = load(XML)?;
    let mut candidate = load(XML)?;
    let mut joints = spec
        .tree
        .joints
        .iter()
        .map(|j| {
            let native = reference
                .world()
                .get::<JointHandle>(entity_by_id(reference.world(), j.joint_id).unwrap())
                .unwrap()
                .id();
            (j.joint_id, native)
        })
        .collect::<Vec<_>>();
    joints.sort_by_key(|j| j.0);
    let joints = [joints[0].1, joints[1].1];
    let id = |public| {
        reference
            .world()
            .get::<BodyHandle>(entity_by_id(reference.world(), public).unwrap())
            .unwrap()
            .id()
    };
    let links = spec.tree.links.iter().map(|l| id(l.id)).collect::<Vec<_>>();
    let part = id(spec.payload.id);
    let mut alone = MultiFidelityBackend::new([0.; 3])?;
    let free = spawn_payload(&mut alone, &spec.payload)?;
    let mut split = false;
    let mut after = 0;
    let mut before_cut = 0;
    let mut detached_ticks = 0;
    let mut bounds_clear = Vec::new();
    let mut external = DVec3::ZERO;
    let mut external_angular = DVec3::ZERO;
    let mut payload_contacts = DVec3::ZERO;
    let (mut linear_peak, mut angular_peak, mut payload_peak) = (0f64, 0f64, 0f64);
    for t in 0..2600 {
        for sim in [&mut reference, &mut candidate] {
            let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
            command(&mut backend, joints, t)?;
        }
        let f = force(t);
        let impulse = DVec3::Z * (DT.as_secs_f64() * f);
        external += impulse;
        external_angular +=
            DVec3::from_array(solver(&reference).state(part).unwrap().position).cross(impulse);
        if f != 0. {
            reference
                .world_mut()
                .resource_mut::<PhysicsBackend>()
                .add_world_wrench(part, [0., 0., f], [0.; 3])?;
            if split {
                alone.add_world_wrench(free, [0., 0., f], [0.; 3])?;
            } else {
                candidate
                    .world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .add_world_wrench(part, [0., 0., f], [0.; 3])?;
            }
        }
        reference.step_exact(1)?;
        candidate.step_exact(1)?;
        if split {
            alone.step(DT)?;
            detached_ticks += 1;
        }
        let ordinary = solver(&reference);
        let actual = solver(&candidate);
        let part_box = ordinary.grasp_collision_bounds(part, DT)?;
        let split_box = if split {
            alone.grasp_collision_bounds(free, DT)?
        } else {
            actual.grasp_collision_bounds(part, DT)?
        };
        assert_eq!(part_box, split_box);
        let mut clear = true;
        for &link in &links {
            let link_box = ordinary.grasp_collision_bounds(link, DT)?;
            assert_eq!(link_box, actual.grasp_collision_bounds(link, DT)?);
            clear &= (0..3).any(|i| link_box.1[i] < part_box.0[i] || part_box.1[i] < link_box.0[i]);
        }
        if clear {
            bounds_clear.push(t + 1);
        }
        for &link in &links {
            assert_eq!(
                exact_state(ordinary.state(link).unwrap()),
                exact_state(actual.state(link).unwrap()),
                "tree tick {} link {:?}",
                t + 1,
                link
            );
        }
        let state = if split {
            alone.state(free)
        } else {
            actual.state(part)
        }
        .unwrap();
        assert_eq!(
            exact_state(ordinary.state(part).unwrap()),
            exact_state(state),
            "payload tick {}",
            t + 1
        );
        let (part_p, part_l) = grasp_momentum::momenta(&spec.payload, state);
        let (mut linear, mut angular) = (part_p, part_l);
        let mut warm = 0;
        for (&link, receipt) in links.iter().zip(&spec.tree.links) {
            let (p, l) = grasp_momentum::momenta(receipt, actual.state(link).unwrap());
            linear += p;
            angular += l;
            let r = ordinary.payload_tree_reaction(link, part)?;
            payload_contacts -= DVec3::from_array(r.normal) * r.normal_impulse
                + DVec3::from_array(r.tangent_impulse);
            if r.warmstart_normal_impulse > 0. {
                warm += 1;
            }
            if split {
                assert!(
                    r == Default::default(),
                    "split through contact tick {}",
                    t + 1
                );
            } else {
                assert_eq!(
                    actual.payload_tree_reaction(link, part)?,
                    r,
                    "reaction tick {}",
                    t + 1
                );
            }
        }
        linear_peak = linear_peak.max((linear - external).length());
        angular_peak = angular_peak.max((angular - external_angular).length());
        payload_peak = payload_peak.max((part_p - external - payload_contacts).length());
        if t < 600 && warm == 2 {
            before_cut += 1;
            assert!(actual.detached_grasp_tree_image(part).is_err());
        }
        if t >= 1450 && warm == 2 {
            after += 1;
        }
        if t == 899 {
            let compact = actual.detached_grasp_tree_image(part)?;
            assert_eq!(&compact[..8], b"GZFMOT01");
            println!(
                "detached tick={} compact={} joined={}",
                t + 1,
                compact.len(),
                actual.checkpoint_grasp_revolute_tree()?.len()
            );
            candidate = load(&bare)?;
            candidate
                .world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .restore_motor_revolute_tree(&compact)?;
            alone.place_detached_sphere(free, state)?;
            split = true;
        }
        if t == 1399 {
            let mut be = candidate.world_mut().resource_mut::<PhysicsBackend>();
            let tree = be.downcast_mut::<MultiFidelityBackend>().unwrap();
            assert_eq!(spawn_payload(tree, &spec.payload)?, part);
            tree.place_detached_sphere(part, state)?;
            let image = tree.checkpoint_grasp_revolute_tree()?;
            // A new controller restores the combined world before it next meets either finger.
            candidate = load(XML)?;
            candidate
                .world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .restore_grasp_revolute_tree(&image)?;
            split = false;
        }
    }
    println!(
        "initial_bilateral={before_cut} separated={detached_ticks} recaptured_bilateral={after}"
    );
    println!(
        "bounds_clear={} first={:?} last={:?}",
        bounds_clear.len(),
        bounds_clear.first(),
        bounds_clear.last()
    );
    println!(
        "max_total_momentum_residual_ns={linear_peak:e} max_angular_momentum_residual_nms={angular_peak:e} max_payload_impulse_residual_ns={payload_peak:e}"
    );
    assert!(linear_peak < 0.003 && angular_peak < 0.003 && payload_peak < 1e-10);
    assert!(bounds_clear.len() > 100 && bounds_clear.iter().all(|t| *t > 600));
    assert!(before_cut > 300 && after > 75 && detached_ticks == 500);
    Ok(())
}
