// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DMat3, DQuat, DVec3};
use gz_physics::PhysicsBackend;
use gz_physics_distributed::floating_tree::*;
use gz_sim_core::entity_by_id;
use gz_sim_scene::{
    BodyHandle, PhysicsEngine,
    deformable::{self, articulated_impact::distributed_material_contact_tree_spec},
    load_world_with_sdf_step,
};
use std::{error::Error, fs, path::Path, process::Command};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const CLOTH: &str = include_str!("../../../examples/articulated_material_cloth_impact.sdf");
const CABLE: &str = include_str!("../../../examples/articulated_material_cable_impact.sdf");
const FILE: &str = "barrier.gzmaterialtree";
fn setup(xml: &str) -> Result<(ArticulatedMaterialSpec, gz_physics_xpbd::SoftBody)> {
    let a = distributed_material_contact_tree_spec(xml, "impact.sdf", "default")?;
    assert_eq!(a.step.as_nanos(), 1_000_000);
    Ok((
        ArticulatedMaterialSpec {
            tree: ContactTreeSpec {
                tree: a.tree,
                limits: a.limits,
            },
            material_name: a.material_name,
            barriers: a
                .barriers
                .into_iter()
                .map(|b| MaterialImpactBarrier {
                    tick: (b.time.as_nanos() / 1_000_000) as u64,
                    links: b.links,
                    restitution: b.restitution,
                })
                .collect(),
        },
        a.material,
    ))
}
fn momentum(p: &ArticulatedMaterialPool) -> (DVec3, DVec3, f64) {
    let (mut lin, mut ang, mut ke) = (DVec3::ZERO, DVec3::ZERO, 0.);
    for (l, s) in p.spec().tree.tree.links.iter().zip(&p.observation().links) {
        let rot = DQuat::from_array(s.orientation);
        let c = DVec3::from(s.position) + rot * DVec3::from(l.inertia_center);
        let frame = DMat3::from_quat(rot * DQuat::from_array(l.inertia_orientation));
        let [xx, yy, zz, xy, xz, yz] = l.inertia_tensor;
        let i = frame
            * DMat3::from_cols_array(&[xx, xy, xz, xy, yy, yz, xz, yz, zz])
            * frame.transpose();
        let v = DVec3::from(s.velocity);
        let w = DVec3::from(s.angular_velocity);
        let m = l.mass * v;
        lin += m;
        ang += c.cross(m) + i * w;
        ke += 0.5 * (m.dot(v) + w.dot(i * w));
    }
    for x in p.particles() {
        assert!(x.inverse_mass > 0.);
        let v = DVec3::from(x.velocity);
        let mv = v / x.inverse_mass;
        lin += mv;
        ang += DVec3::from(x.position).cross(mv);
        ke += 0.5 * mv.dot(v);
    }
    (lin, ang, ke)
}
#[test]
fn material_and_native_keep_one_clock_across_true_contact_owner_loss_and_reopen() -> Result<()> {
    let exe = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    for (name, xml) in [("cloth", CLOTH), ("cable", CABLE)] {
        let (s, cold) = setup(xml)?;
        let d = tempfile::tempdir()?;
        let mut p = ArticulatedMaterialPool::create(exe, d.path(), 3, s.clone(), cold.clone())?;
        let mut sim = load_world_with_sdf_step(
            xml,
            "impact.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let ids = s
            .tree
            .tree
            .links
            .iter()
            .map(|l| {
                sim.world()
                    .get::<BodyHandle>(entity_by_id(sim.world(), l.id).unwrap())
                    .unwrap()
                    .id()
            })
            .collect::<Vec<_>>();
        let initial = momentum(&p);
        let (mut pi, mut li, mut ki, mut maxdot, mut peak_j, mut normal) = (0., 0., 0., 0., 0., 0.);
        let mut lost = 0;
        for t in 0..240 {
            if t == 52 || t == 99 || t == 100 || t == 172 {
                let to = (p.owner() + 1) % 3;
                assert!(
                    Command::new("kill")
                        .args([
                            "-KILL",
                            &p.worker_pid(if t == 52 || t == 100 { to } else { p.owner() })
                                .unwrap()
                                .to_string()
                        ])
                        .status()?
                        .success()
                );
                if t != 99 {
                    p.transfer(to)?;
                }
                lost += 1;
            }
            let witness = std::env::var_os("GZ_DISTRIBUTED_MATERIAL_WITNESS");
            if t == 99
                && let Some(w) = &witness
            {
                let root = Path::new(w).join(name);
                fs::create_dir_all(&root)?;
                fs::copy(d.path().join(FILE), root.join("prior.gzmaterialtree"))?;
            }
            p.advance_with_efforts_at(0, t, &[])?;
            sim.step_exact(1)?;
            let soft = deformable::bodies(&sim).next().unwrap().1;
            assert_eq!(
                p.particles(),
                soft.particles(),
                "particle phase {name} t={t}"
            );
            let b = sim.world().resource::<PhysicsBackend>();
            for (id, row) in ids.iter().zip(&p.observation().links) {
                let v = b.state(*id).unwrap();
                assert_eq!(
                    (v.position, v.orientation, v.velocity, v.angular_velocity),
                    (
                        row.position,
                        row.orientation,
                        row.velocity,
                        row.angular_velocity
                    ),
                    "body phase {name} t={t}"
                );
            }
            if t == 99 {
                let ours = p.last_impact().ok_or("missing synchronized impact")?;
                let reference = soft.latest_articulated_impact().unwrap();
                assert_eq!(ours.loads.len(), 2);
                assert_eq!(ours.tick, 100);
                for (l, r) in ours.loads.iter().zip(&reference.material.loads) {
                    assert_eq!(l.point, r.site.point);
                    assert_eq!(l.on_link, r.on_rigid);
                    assert_eq!(l.on_particles, r.on_particles);
                    assert_eq!(l.initial_normal_velocity, r.initial_normal_velocity);
                    assert_eq!(l.final_normal_velocity, r.final_normal_velocity);
                    let mut sum = DVec3::from(l.on_link);
                    for (_, j) in &l.on_particles {
                        sum += DVec3::from(*j);
                    }
                    assert!(sum.length() < 1e-14);
                    peak_j = f64::max(peak_j, DVec3::from(l.on_link).length());
                    normal = l.initial_normal_velocity;
                }
                assert_eq!(
                    ours.native.kinetic_energy_change.to_bits(),
                    reference.native.kinetic_energy_change.to_bits()
                );
                let ni = DVec3::from(ours.loads[0].normal);
                let nj = DVec3::from(ours.loads[1].normal);
                let k = ours.basis.mobility(0, 1).unwrap();
                maxdot = ni.dot(DVec3::new(
                    DVec3::from(k[0]).dot(nj),
                    DVec3::from(k[1]).dot(nj),
                    DVec3::from(k[2]).dot(nj),
                ));
                let m = momentum(&p);
                pi = (m.0 - initial.0).length();
                li = (m.1 - initial.1).length();
                ki = (m.2 - initial.2 + ours.dissipated_energy).abs();
                if let Some(w) = &witness {
                    let root = Path::new(w).join(name);
                    fs::copy(d.path().join(FILE), root.join("accepted.gzmaterialtree"))?;
                    fs::write(root.join("impact.json"), serde_json::to_vec_pretty(ours)?)?;
                }
                let old = fs::read(d.path().join(FILE))?;
                assert!(p.advance_with_efforts_at(0, 99, &[]).is_err());
                assert_eq!(old, fs::read(d.path().join(FILE))?);
            }
            if [17, 98, 99, 100, 188].contains(&t) {
                drop(p);
                p = ArticulatedMaterialPool::resume(exe, d.path(), cold.clone())?;
                assert_eq!(p.particles(), soft.particles());
            }
        }
        eprintln!(
            "material={name} exact_bevy_frames=240 graph_material_joint_cut=true worker_kills={lost} reopen=5 event_tick=100 sites=2 pre_contact_vn_m_s={normal} cross_link_inverse_kg={maxdot} peak_site_Ns={peak_j} momentum_Ns={pi} angular_Nms={li} energy_balance_J={ki}"
        );
        assert!(maxdot.abs() > 0.1 && peak_j > 0.01 && pi < 2e-10 && li < 2e-10 && ki < 2e-10);
        p.reset()?;
        assert_eq!((p.epoch(), p.tick()), (1, 0));
        assert!(p.last_impact().is_none());
        assert_eq!(p.particles(), cold.particles());
    }
    Ok(())
}
