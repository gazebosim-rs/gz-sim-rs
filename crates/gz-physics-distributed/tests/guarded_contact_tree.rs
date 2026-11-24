// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_physics_distributed::floating_tree::{
    ArticulatedPointSite, ContactTreePool, ContactTreeSpec,
};
use gz_physics_multifidelity::{ArticulatedContactSite, MultiFidelityBackend};
use gz_sim_core::entity_by_id;
use gz_sim_scene::{
    BodyHandle, PhysicsEngine,
    deformable::articulated_impact::distributed_material_contact_tree_spec,
    load_world_with_sdf_step,
};
use std::{error::Error, fs, path::Path, process::Command};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const XML: &str = include_str!("../../../examples/articulated_material_cloth_impact.sdf");
#[test]
fn guarded_batch_regenerates_cross_link_mobility_and_applies_one_native_impulse_after_owner_death()
-> Result<()> {
    let a = distributed_material_contact_tree_spec(XML, "cloth.sdf", "default")?;
    let s = ContactTreeSpec {
        tree: a.tree,
        limits: a.limits,
    };
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let d = tempfile::tempdir()?;
    let other = tempfile::tempdir()?;
    let mut p = ContactTreePool::create(exe, d.path(), 3, s.clone())?;
    let mut foreign = ContactTreePool::create(exe, other.path(), 2, s.clone())?;
    let mut control = load_world_with_sdf_step(
        XML,
        "cloth.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let ids = s
        .tree
        .links
        .iter()
        .map(|l| {
            control
                .world()
                .get::<BodyHandle>(entity_by_id(control.world(), l.id).unwrap())
                .unwrap()
                .id()
        })
        .collect::<Vec<_>>();
    for _ in 0..11 {
        p.advance()?;
        foreign.advance()?;
        control.step_exact(1)?;
    }
    let sites = [
        ArticulatedPointSite {
            link: s.tree.links[1].id,
            local_point: [0.027, -0.05, 0.083],
        },
        ArticulatedPointSite {
            link: s.tree.links[2].id,
            local_point: [-0.031, 0.035, 0.089],
        },
    ];
    let impulses = [[0.015, 0.014, -0.032], [-0.021, 0.008, -0.017]];
    let save = || fs::read(d.path().join("barrier.gzcontacttree"));
    let start = save()?;
    let stale = p.sample_contact_points_at(0, 11, &sites)?;
    assert_eq!(start, save()?);
    assert!(
        foreign
            .advance_with_contact_impulses(&stale, &impulses)
            .is_err()
    );
    assert!(
        p.advance_with_contact_impulses(&stale, &[[f64::NAN; 3]; 2])
            .is_err()
    );
    assert_eq!(start, save()?);
    p.transfer(2)?;
    assert!(p.advance_with_contact_impulses(&stale, &impulses).is_err());
    let before = p.sample_contact_points_at(0, 11, &sites)?;
    p.queue_effort(s.tree.joints[0].joint_id, 0.)?;
    assert!(p.advance_with_contact_impulses(&before, &impulses).is_err());
    let cur = p.sample_contact_points_at(0, 11, &sites)?;
    let native_sites = sites
        .iter()
        .zip(&ids[1..])
        .map(|(s, id)| ArticulatedContactSite {
            body: *id,
            local_point: s.local_point,
        })
        .collect::<Vec<_>>();
    let ordinary_report = {
        let mut backend = control.world_mut().resource_mut::<PhysicsBackend>();
        let native = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
        let tx = native.prepare_articulated_contacts(&native_sites)?;
        assert_eq!(tx.sample().world_points, cur.basis().world_points);
        assert_eq!(tx.sample().velocities, cur.basis().velocities);
        for i in 0..2 {
            for j in 0..2 {
                assert_eq!(tx.sample().mobility(i, j), cur.basis().mobility(i, j));
            }
        }
        tx.commit(&impulses)?
    };
    let pid = p.worker_pid(p.owner()).unwrap();
    assert!(
        Command::new("kill")
            .args(["-KILL", &pid.to_string()])
            .status()?
            .success()
    );
    let got = p.advance_with_contact_impulses(&cur, &impulses)?;
    assert_eq!(got.after_velocities, ordinary_report.after_velocities);
    assert_eq!(
        got.kinetic_energy_change.to_bits(),
        ordinary_report.kinetic_energy_change.to_bits()
    );
    assert!(p.advance_with_contact_impulses(&cur, &impulses).is_err());
    let mut frames = 0;
    for t in 12..70 {
        if t > 12 {
            p.advance()?;
        }
        control.step_exact(1)?;
        let b = control.world().resource::<PhysicsBackend>();
        for (i, &id) in ids.iter().enumerate() {
            let x = b.state(id).unwrap();
            let y = &p.observation().links[i];
            assert_eq!(
                (x.position, x.orientation, x.velocity, x.angular_velocity),
                (y.position, y.orientation, y.velocity, y.angular_velocity)
            );
        }
        frames += 1;
        if t == 28 {
            p.transfer(1)?;
            drop(p);
            p = ContactTreePool::resume(exe, d.path())?;
        }
    }
    let before = p.sample_contact_points_at(0, p.observation().tick, &sites)?;
    drop(p);
    let mut p = ContactTreePool::resume(exe, d.path())?;
    assert!(p.advance_with_contact_impulses(&before, &impulses).is_err());
    eprintln!(
        "8 reduced coordinates, 36 matched cross/site coefficients, signed impulses exactly accepted once after process death; {frames} full ordinary body frames, native dK={} J; resumed and transferred",
        got.kinetic_energy_change
    );
    Ok(())
}
