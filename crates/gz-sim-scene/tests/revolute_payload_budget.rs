// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "distributed-physics")]
#[path = "support/grasp_momentum.rs"]
mod grasp_momentum;
use glam::{DQuat, DVec3};
use grasp_momentum::momenta;
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, PhysicsEngine, distributed_payload_tree_spec, load_world_with_sdf_step,
    queue_joint_effort,
};
use std::error::Error;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const XML: &str = include_str!("../../../examples/distributed_gripper_payload.sdf");

fn ids(sim: &Simulation, id: u64) -> gz_physics::BodyId {
    sim.world()
        .get::<BodyHandle>(entity_by_id(sim.world(), id).unwrap())
        .unwrap()
        .id()
}

#[test]
fn free_payload_contact_impulses_balance_linear_momentum_from_closure_to_release() -> Result<()> {
    let spec = distributed_payload_tree_spec(XML, "gripper.sdf", "default")?;
    let mut results = Vec::new();
    for micros in [1000, 500, 250] {
        let xml = XML.replace(
            "0.001</max_step_size>",
            &format!("{}</max_step_size>", micros as f64 * 1e-6),
        );
        let mut sim = load_world_with_sdf_step(
            &xml,
            "gripper.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let mut all = spec
            .tree
            .joints
            .iter()
            .map(|j| j.joint_id)
            .collect::<Vec<_>>();
        all.sort_unstable();
        let payload = ids(&sim, spec.payload.id);
        let links = spec
            .tree
            .links
            .iter()
            .map(|l| ids(&sim, l.id))
            .collect::<Vec<_>>();
        let mut impulses = DVec3::ZERO;
        let mut momentum_peak = 0f64;
        let mut angular_peak = 0f64;
        let mut payload_peak = 0f64;
        let mut loaded = 0;
        let mut released = 0;
        let mut load_impulse = 0f64;
        let mut max_part = 0f64;
        for i in 1..=1_400_000 / micros {
            let start_us = (i - 1) * micros;
            let sign = if (start_us / 400_000) % 2 == 0 {
                1.
            } else {
                -1.
            };
            for (&id, e) in all.iter().zip([0.8, -0.91]) {
                queue_joint_effort(&mut sim, id, sign * e)?;
            }
            sim.step_exact(1)?;
            let backend = sim.world().resource::<PhysicsBackend>();
            let native = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
            let state = backend.state(payload).unwrap();
            let (part_p, part_l) = momenta(&spec.payload, state);
            let mut p = part_p;
            let mut ang = part_l;
            let mut touching = 0;
            for (l, &b) in spec.tree.links.iter().zip(&links) {
                let (lp, la) = momenta(l, backend.state(b).unwrap());
                p += lp;
                ang += la;
                let r = native.payload_tree_reaction(b, payload)?;
                let j = DVec3::from_array(r.normal) * r.normal_impulse
                    + DVec3::from_array(r.tangent_impulse);
                impulses -= j;
                if r.active && r.normal_impulse > 1e-10 && r.warmstart_normal_impulse > 0. {
                    touching += 1;
                }
                load_impulse += r.normal_impulse;
            }
            if touching >= 2 {
                loaded += micros;
            }
            if i * micros > 450_000 && i * micros < 800_000 && touching == 0 {
                released += micros;
            }
            max_part = max_part.max(part_p.length());
            payload_peak = payload_peak.max((part_p - impulses).length());
            momentum_peak = momentum_peak.max(p.length());
            angular_peak = angular_peak.max(ang.length());
        }
        println!(
            "dt_us={micros} both_warm_us={loaded} released_us={released} summed_compressive_impulse={load_impulse} peak_payload_momentum={max_part} max_payload_impulse_residual={payload_peak} max_total_linear_momentum={momentum_peak} max_total_angular_momentum={angular_peak}"
        );
        assert!(loaded > 120_000 && released > 200_000 && load_impulse > 0.1 && max_part > 0.004);
        assert!(payload_peak < 1e-10);
        assert!(
            momentum_peak < 0.003 && angular_peak < 0.003,
            "p={momentum_peak}, L={angular_peak}"
        );
        results.push((momentum_peak, angular_peak));
    }
    assert!(results[2].0 < results[0].0 && results[2].1 < results[0].1);
    Ok(())
}

#[derive(Debug, Default)]
struct LoadTrial {
    bilateral_before: u64,
    bilateral_loaded: u64,
    released: u64,
    slip_loaded: f64,
    slip_after: f64,
    load_tangent_z: f64,
    max_part_residual: f64,
    max_p_residual: f64,
    max_l_residual: f64,
}

fn loaded_trial(cap: f64, target: f64, micros: u64) -> Result<LoadTrial> {
    let spec = distributed_payload_tree_spec(XML, "gripper.sdf", "default")?;
    let dt = micros as f64 * 1e-6;
    let xml = XML.replace("0.001</max_step_size>", &format!("{dt}</max_step_size>"));
    let mut sim = load_world_with_sdf_step(
        &xml,
        "gripper.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut joints = spec
        .tree
        .joints
        .iter()
        .map(|j| {
            let id = sim
                .world()
                .get::<gz_sim_scene::JointHandle>(entity_by_id(sim.world(), j.joint_id).unwrap())
                .unwrap()
                .id();
            (j.joint_id, id)
        })
        .collect::<Vec<_>>();
    joints.sort_by_key(|j| j.0);
    let payload = ids(&sim, spec.payload.id);
    let links = spec
        .tree
        .links
        .iter()
        .map(|j| ids(&sim, j.id))
        .collect::<Vec<_>>();
    let mut trial = LoadTrial::default();
    let mut external = DVec3::ZERO;
    let mut ext_l = DVec3::ZERO;
    let mut contacts = DVec3::ZERO;
    let mut reference = DVec3::ZERO;
    for i in 0..1_500_000 / micros {
        let t = i * micros;
        if matches!(t, 0 | 650_000 | 1_100_000) {
            let mut be = sim.world_mut().resource_mut::<PhysicsBackend>();
            let sign = if t < 1_100_000 { target } else { -6. };
            for ((_, id), speed) in joints.iter().zip([sign, -sign]) {
                be.set_joint_velocity_motor(
                    *id,
                    Some(gz_physics::JointVelocityMotor {
                        target_velocity: speed,
                        max_force: if t < 650_000 { 4. } else { cap },
                    }),
                )?;
            }
        }
        // The free payload carries a calibrated +Z load through the squeeze and release.
        if t >= 650_000 {
            let force = DVec3::Z * 1.2;
            let state = sim
                .world()
                .resource::<PhysicsBackend>()
                .state(payload)
                .unwrap();
            external += dt * force;
            ext_l += dt * DVec3::from_array(state.position).cross(force);
            gz_sim_scene::queue_link_world_wrench(
                &mut sim,
                spec.payload.id,
                force.to_array(),
                [0.; 3],
            )?;
        }
        sim.step_exact(1)?;
        let be = sim.world().resource::<PhysicsBackend>();
        let native = be.downcast_ref::<MultiFidelityBackend>().unwrap();
        let part = be.state(payload).unwrap();
        let (part_p, part_l) = momenta(&spec.payload, part);
        let palm = be.state(links[0]).unwrap();
        let in_palm = DQuat::from_array(palm.orientation).conjugate()
            * (DVec3::from_array(part.position) - DVec3::from_array(palm.position));
        if t + micros == 650_000 {
            reference = in_palm;
        }
        let mut count = 0;
        let mut p = part_p;
        let mut l = part_l;
        for (link, &id) in spec.tree.links.iter().zip(&links) {
            let (a, b) = momenta(link, be.state(id).unwrap());
            p += a;
            l += b;
            let r = native.payload_tree_reaction(id, payload)?;
            contacts -= r.normal_impulse * DVec3::from_array(r.normal)
                + DVec3::from_array(r.tangent_impulse);
            if r.normal_impulse > 1e-9 && r.warmstart_normal_impulse > 0. {
                count += 1;
            }
            if (650_000..1_100_000).contains(&t) {
                trial.load_tangent_z -= r.tangent_impulse[2];
            }
        }
        if t < 650_000 && count == 2 {
            trial.bilateral_before += micros;
        }
        if (650_000..1_100_000).contains(&t) {
            if count == 2 {
                trial.bilateral_loaded += micros;
            }
            trial.slip_loaded = trial.slip_loaded.max((in_palm - reference).length());
        }
        if t >= 1_100_000 {
            if count == 0 {
                trial.released += micros;
            }
            trial.slip_after = trial.slip_after.max((in_palm - reference).length());
        }
        trial.max_part_residual = trial
            .max_part_residual
            .max((part_p - external - contacts).length());
        trial.max_p_residual = trial.max_p_residual.max((p - external).length());
        trial.max_l_residual = trial.max_l_residual.max((l - ext_l).length());
    }
    Ok(trial)
}

#[test]
fn torque_limited_fingers_transfer_a_transverse_payload_load_and_release_it() -> Result<()> {
    for micros in [1000, 500, 250] {
        let strong = loaded_trial(4., 6., micros)?;
        let weak = loaded_trial(0.08, 6., micros)?;
        let open = loaded_trial(4., -6., micros)?;
        for (name, trial) in [("strong", &strong), ("weak", &weak), ("open", &open)] {
            println!("dt_us={micros} {name} {trial:?}");
            assert!(
                trial.max_part_residual < 1e-9
                    && trial.max_p_residual < 1e-3
                    && trial.max_l_residual < 1e-3
            );
            assert!(trial.released > 390_000);
        }
        assert_eq!(strong.bilateral_before, weak.bilateral_before);
        assert!(strong.bilateral_before > 410_000 && strong.bilateral_loaded == 450_000);
        assert!(weak.bilateral_loaded < 140_000 && open.bilateral_loaded == 0);
        assert!(strong.slip_loaded < 0.001 && weak.slip_loaded > 0.12 && open.slip_loaded > 0.15);
        assert!(
            strong.load_tangent_z < -0.36
                && weak.load_tangent_z.abs() < 0.03
                && open.load_tangent_z == 0.
        );
        assert!(strong.slip_after > 0.12);
    }
    Ok(())
}
