// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent ordinary Bevy control for the frozen contact-and-stop counterexample.
use glam::DVec3;
use gz_physics::{BodySpec, DynamicBackend, Inertia, PhysicsBackend, RotationSpec, Shape};
use gz_physics_distributed::{
    articulation::ArticulationLinkSpec,
    floating_tree::{
        FloatingTreeJointLimit, FloatingTreeJointSpec, FloatingTreeSpec, PayloadTreeSpec,
    },
};
use gz_physics_multifidelity::{ArticulationTier, MultiFidelityBackend};
use gz_sim_core::Simulation;
use std::{error::Error, time::Duration};
#[path = "support/tree_payload_fixture.rs"]
mod fixture;

#[derive(Default)]
struct Metrics {
    peak: f64,
    last: f64,
    closure: f64,
    max_csv_delta: f64,
    peak_contacts: usize,
    drift_at: [f64; 3],
}

struct ControlWorld {
    sim: Simulation,
    links: Vec<gz_physics::BodyId>,
    payload: gz_physics::BodyId,
    joints: Vec<gz_physics::JointId>,
}
fn build(spec: &PayloadTreeSpec, dt: Duration) -> Result<ControlWorld, Box<dyn Error>> {
    let mut backend = MultiFidelityBackend::new([0.; 3])?;
    let mut links = Vec::new();
    let mut joints = Vec::new();
    let mut payload = None;
    let mut sorted = spec
        .tree
        .links
        .iter()
        .chain([&spec.payload])
        .collect::<Vec<_>>();
    sorted.sort_by_key(|l| l.id);
    for l in sorted {
        let body = BodySpec {
            position: l.position,
            velocity: l.velocity,
            mass: Some(l.mass),
            shape: Shape::Sphere { radius: l.radius },
        };
        let rotation = RotationSpec {
            orientation: l.orientation,
            angular_velocity: l.angular_velocity,
        };
        let inertia = Some(Inertia {
            center: l.inertia_center,
            orientation: l.inertia_orientation,
            tensor: l.inertia_tensor,
        });
        if l.id == spec.payload.id {
            let id = backend.spawn_link(body, rotation, inertia)?;
            if !spec.payload_can_sleep {
                backend.keep_body_awake(id)?;
            }
            payload = Some(id);
        } else {
            let id = backend.spawn_articulation_link(body, rotation, inertia)?;
            backend.set_self_collision_group(id, Some(spec.tree.model_id))?;
            links.push(id);
        }
    }
    for j in &spec.tree.joints {
        let parent = links[spec
            .tree
            .links
            .iter()
            .position(|l| l.id == j.parent_id)
            .unwrap()];
        let child = links[spec
            .tree
            .links
            .iter()
            .position(|l| l.id == j.child_id)
            .unwrap()];
        let id = if let Some(s) = spec.limits.iter().find(|r| r.joint_id == j.joint_id) {
            backend.add_limited_revolute(
                Some(parent),
                child,
                j.anchor,
                j.axis,
                [s.lower, s.upper],
            )?
        } else {
            backend.add_reduced_revolute(Some(parent), child, j.anchor, j.axis)?
        };
        joints.push(id);
    }
    backend.switch_articulation(links[0], ArticulationTier::Dynamic, dt)?;
    let mut sim = Simulation::new(dt)?;
    gz_physics::install(&mut sim, backend)?;
    Ok(ControlWorld {
        sim,
        links,
        payload: payload.unwrap(),
        joints,
    })
}

fn run(
    radius: f64,
    locked: bool,
    drive: bool,
    permit_sleep: bool,
    substeps: u32,
    reference: &[f64],
) -> Result<Metrics, Box<dyn Error>> {
    let mut spec = fixture::cage(8, true, 0.24, [0.26, 0.074, -0.021]);
    spec.payload_can_sleep = permit_sleep;
    if !locked {
        spec.limits.clear();
    }
    for l in &mut spec.tree.links {
        for x in &mut l.position[..2] {
            *x *= radius / 0.238;
        }
    }
    for j in &mut spec.tree.joints {
        for x in &mut j.anchor[..2] {
            *x *= radius / 0.238;
        }
    }
    let dt = Duration::from_millis(1) / substeps;
    let ControlWorld {
        mut sim,
        links,
        payload,
        joints,
    } = build(&spec, dt)?;
    let p0 = DVec3::from_array(spec.payload.velocity) * spec.payload.mass;
    let mut cumulative = DVec3::ZERO;
    let mut metrics = Metrics::default();
    for tick in 0..280 {
        for _ in 0..substeps {
            if drive {
                let j = joints[tick % joints.len()];
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .add_joint_effort(j, if tick % 15 < 7 { 0.036 } else { -0.024 })?;
            }
            sim.step_exact(1)?;
            let backend = sim.world().resource::<PhysicsBackend>();
            let native = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
            let mut active = 0;
            for &id in &links {
                let r = native.payload_tree_reaction(id, payload)?;
                active += usize::from(r.normal_impulse > 0.);
                cumulative += DVec3::from_array(r.normal) * r.normal_impulse
                    + DVec3::from_array(r.tangent_impulse);
            }
            metrics.peak_contacts = metrics.peak_contacts.max(active);
            let part =
                DVec3::from_array(backend.state(payload).unwrap().velocity) * spec.payload.mass;
            let tree: DVec3 = links
                .iter()
                .zip(&spec.tree.links)
                .map(|(&id, l)| DVec3::from_array(backend.state(id).unwrap().velocity) * l.mass)
                .sum();
            metrics.last = (part + tree - p0).length();
            metrics.peak = metrics.peak.max(metrics.last);
            metrics.closure = metrics.closure.max((part - p0 + cumulative).length());
        }
        for (i, sample) in [0, 20, 100].iter().enumerate() {
            if *sample == tick {
                metrics.drift_at[i] = metrics.last;
            }
        }
        if let Some(expected) = reference.get(tick) {
            metrics.max_csv_delta = metrics.max_csv_delta.max((metrics.last - expected).abs());
        }
    }
    Ok(metrics)
}

#[test]
fn ordinary_bevy_contact_and_stop_momentum_controls() -> Result<(), Box<dyn Error>> {
    let file = include_str!(
        "fixtures/first-light-contact-history.csv"
    );
    let frozen: Vec<f64> = file
        .lines()
        .skip(1)
        .map(|s| s.rsplit(',').next().unwrap().parse().unwrap())
        .collect();
    assert_eq!(frozen.len(), 280);
    eprintln!(
        "radius_m,locked,driven,sleep,dt_us,peak_contacts,tick1_drift,tick21_drift,tick101_drift,peak_drift,final_drift,payload_closure,max_difference_from_original_worker"
    );
    for (radius, locked, drive, sleep) in [
        (0.238, true, true, true),
        (0.238, true, true, false),
        (0.238, true, false, false),
        (0.240, true, true, false),
        (0.242, true, true, false),
        (0.238, false, true, false),
        (0.240, false, true, false),
        (0.242, false, true, false),
    ] {
        for n in [1, 2, 4] {
            let same = radius == 0.238 && locked && drive && sleep && n == 1;
            let m = run(
                radius,
                locked,
                drive,
                sleep,
                n,
                if same { &frozen } else { &[] },
            )?;
            eprintln!(
                "{radius},{locked},{drive},{sleep},{},{},{},{},{},{},{},{},{}",
                1000 / n,
                m.peak_contacts,
                m.drift_at[0],
                m.drift_at[1],
                m.drift_at[2],
                m.peak,
                m.last,
                m.closure,
                m.max_csv_delta
            );
            assert!(
                m.closure < 1e-8,
                "payload contact impulses must close on every substep"
            );
        }
    }
    Ok(())
}
