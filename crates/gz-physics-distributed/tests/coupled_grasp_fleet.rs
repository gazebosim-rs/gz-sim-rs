// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics::{BodyId, JointId, JointVelocityMotor, PhysicsBackend};
use gz_physics_distributed::{
    articulation::ArticulationLinkSpec,
    floating_tree::{
        CoupledGraspFleetPool, FloatingTreeJointLimit, FloatingTreeJointSpec, FloatingTreeSpec,
        GraspFleetSpec, GraspTreeSpec, PayloadTreeSpec,
    },
};
use gz_physics_multifidelity::{ArticulationTier, MovingReaction, MultiFidelityBackend};
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::{BodyHandle, JointDefinition, JointHandle, PhysicsEngine};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, error::Error, fs, path::Path, process::Command};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const XML: &str = include_str!("../../../examples/distributed_grasp_fleet.sdf");
const FILE: &str = "barrier.gzgraspcluster";
#[path = "support/tree_payload_fixture.rs"]
mod fixture;

fn exe() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
fn kill(id: u32) -> Result<()> {
    assert!(
        Command::new("kill")
            .args(["-KILL", &id.to_string()])
            .status()?
            .success()
    );
    Ok(())
}
fn impulse(r: &MovingReaction) -> DVec3 {
    DVec3::from_array(r.normal) * r.normal_impulse + DVec3::from_array(r.tangent_impulse)
}
fn warm(r: &MovingReaction) -> bool {
    r.active && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.
}

struct Ordinary {
    sim: Simulation,
    keys: [(BodyId, BodyId); 2],
    bodies: BTreeMap<u64, BodyId>,
    joints: BTreeMap<u64, (JointId, f64)>,
}
impl Ordinary {
    fn new(xml: &str) -> Result<(Self, GraspFleetSpec)> {
        let spec = GraspFleetSpec {
            assemblies: gz_sim_scene::distributed_hinge::distributed_grasp_fleet_specs(
                xml,
                "fleet.sdf",
                "default",
            )?,
        };
        let mut sim = gz_sim_scene::load_world_with_sdf_step(
            xml,
            "fleet.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        for s in &spec.assemblies {
            gz_sim_scene::set_model_fidelity(
                &mut sim,
                s.payload.tree.model_id,
                ArticulationTier::Dynamic,
            )?;
        }
        let w = sim.world();
        let mut bodies = BTreeMap::new();
        let mut joints = BTreeMap::new();
        let mut keys = Vec::new();
        for s in &spec.assemblies {
            let t = &s.payload.tree;
            for l in t.links.iter().chain([&s.payload.payload]) {
                bodies.insert(
                    l.id,
                    w.get::<BodyHandle>(entity_by_id(w, l.id).unwrap())
                        .unwrap()
                        .id(),
                );
            }
            for j in &t.joints {
                let e = entity_by_id(w, j.joint_id).unwrap();
                let sign = if w.get::<JointDefinition>(e).unwrap().0.name == "right_close" {
                    1.
                } else {
                    -1.
                };
                joints.insert(j.joint_id, (w.get::<JointHandle>(e).unwrap().id(), sign));
            }
            let root = t
                .links
                .iter()
                .find(|l| !t.joints.iter().any(|j| j.child_id == l.id))
                .unwrap();
            keys.push((bodies[&root.id], bodies[&s.payload.payload.id]));
        }
        Ok((
            Self {
                sim,
                keys: keys.try_into().unwrap(),
                bodies,
                joints,
            },
            spec,
        ))
    }
    fn motor(&mut self, p: &mut CoupledGraspFleetPool, id: u64, speed: f64) -> Result<()> {
        let (h, sign) = self.joints[&id];
        let m = Some(JointVelocityMotor {
            target_velocity: sign * speed,
            max_force: 4.,
        });
        self.sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_joint_velocity_motor(h, m)?;
        p.set_velocity_motor(id, m)?;
        Ok(())
    }
    fn apply(&mut self, efforts: &[(u64, f64)], loads: &[(u64, [f64; 3])]) -> Result<()> {
        for &(id, e) in efforts {
            gz_sim_scene::queue_joint_effort(&mut self.sim, id, e)?;
        }
        for &(id, f) in loads {
            if f != [0.; 3] {
                self.sim
                    .world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .add_world_wrench(self.bodies[&id], f, [0.; 3])?;
            }
        }
        self.sim.step_exact(1)?;
        Ok(())
    }
    fn verify(&self, p: &CoupledGraspFleetPool) -> Result<()> {
        let backend = self.sim.world().resource::<PhysicsBackend>();
        let s = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
        let native = s.checkpoint_grasp_fleet(&self.keys)?;
        let img = p.image().unwrap();
        if native != img {
            let i = native.iter().zip(img).position(|(a, b)| a != b);
            panic!(
                "full coupled native differs at tick {} native len {} pool len {} first byte {i:?}, ordinary {:x} vs managed {:x}",
                p.tick(),
                native.len(),
                img.len(),
                Sha256::digest(native),
                Sha256::digest(img)
            );
        }
        for g in &p.spec().assemblies {
            let m = g.payload.tree.model_id;
            let tree = p.observation(m).unwrap();
            let part = p.payload_state(m).unwrap();
            assert_eq!(tree.tick, p.tick());
            for row in tree.links.iter().chain([part]) {
                let b = backend.state(self.bodies[&row.id]).unwrap();
                assert_eq!(row.position.map(f64::to_bits), b.position.map(f64::to_bits));
                assert_eq!(
                    row.orientation.map(f64::to_bits),
                    b.orientation.map(f64::to_bits)
                );
                assert_eq!(row.velocity.map(f64::to_bits), b.velocity.map(f64::to_bits));
                assert_eq!(
                    row.angular_velocity.map(f64::to_bits),
                    b.angular_velocity.map(f64::to_bits)
                );
            }
            for j in &tree.joints {
                let h = self.joints[&j.joint_id].0;
                let r = backend.joint_state(h).unwrap();
                assert_eq!(j.q.to_bits(), r.position.to_bits());
                assert_eq!(j.qdot.to_bits(), r.velocity.to_bits());
                assert_eq!(j.pending_effort, s.pending_joint_effort(h));
                assert_eq!(p.velocity_motor(j.joint_id), s.motor_revolute_velocity(h)?);
            }
            for (l, r) in tree.links.iter().zip(p.payload_reactions(m).unwrap()) {
                assert_eq!(
                    *r,
                    s.payload_tree_reaction(self.bodies[&l.id], self.bodies[&part.id])?
                );
            }
        }
        for row in p.cross_reactions() {
            let a = self.bodies[&row.first_body];
            let b = self.bodies[&row.second_body];
            assert_eq!(row.reaction, s.sphere_pair_reaction(a, b)?);
            let reverse = s.sphere_pair_reaction(b, a)?;
            assert_eq!(reverse.active, row.reaction.active);
            assert_eq!(
                reverse.warmstart_normal_impulse.to_bits(),
                row.reaction.warmstart_normal_impulse.to_bits()
            );
            assert!((impulse(&reverse) + impulse(&row.reaction)).length() < 1e-13);
        }
        Ok(())
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
enum Case {
    FreeParts,
    ObliqueParts,
    LoadedGrippers,
    Staggered,
}
fn exercise(case: Case) -> Result<()> {
    let held = case == Case::LoadedGrippers;
    let oblique = case == Case::ObliqueParts;
    let free = case == Case::FreeParts || oblique;
    let xml = match case {
        Case::FreeParts | Case::ObliqueParts => XML
            .replace("0.317 0.954 0.58", "2.0 0.954 0.58")
            .replace("0.317 -0.946 0.58", "2.0 -0.946 0.58"),
        Case::Staggered => XML.to_owned(),
        Case::LoadedGrippers => XML
            .replace("-0.95 0.58", "-0.52 0.58")
            .replace(" 0.95 0.58", " 0.52 0.58")
            .replace("-0.946 0.58", "-0.516 0.58")
            .replace(" 0.954 0.58", " 0.524 0.58"),
    };
    let (mut o, spec) = Ordinary::new(&xml)?;
    let path = tempfile::tempdir()?;
    let mut p = CoupledGraspFleetPool::create(exe(), path.path(), 3, spec.clone())?;
    let count = if free {
        1450
    } else if held {
        3300
    } else {
        2450
    };
    let mut first = None;
    let mut firstwarm = None;
    let mut active_ticks = 0;
    let mut cross_and_grip = 0;
    let mut released = 0;
    let mut moves = 0;
    let mut maximum = 0f64;
    let mut cross_sum = 0f64;
    let mut peak_shear = 0f64;
    let mut expected = [DVec3::ZERO; 2];
    let mut max_residual = 0f64;
    let mut graph = Sha256::new();
    let mut pair_types = BTreeMap::<(u64, u64), usize>::new();
    for t in 0..count {
        let mut forces = Vec::new();
        let mut efforts = Vec::new();
        for (i, g) in spec.assemblies.iter().enumerate() {
            let j = g.payload.tree.joints[0].joint_id;
            let sign = -g.payload.payload.position[1].signum();
            if free && i == 0 {
                // Retain the actual source control used in the independently frozen approaching cut.
                p.queue_effort(j, 0.0002)?;
                gz_sim_scene::queue_joint_effort(&mut o.sim, j, 0.0002)?;
            }
            if held && t == 0 {
                for j in &g.payload.tree.joints {
                    o.motor(&mut p, j.joint_id, 6.)?;
                }
            }
            let local = t as i64 - if i == 1 { 350 } else { 0 };
            if !free && !held && local >= 0 {
                let open = if i == 0 { 600 } else { 1100 };
                if local == 0 || local == open || (i == 0 && local == 1450) {
                    for j in &g.payload.tree.joints {
                        o.motor(&mut p, j.joint_id, if local == open { -6. } else { 6. })?;
                    }
                }
            }
            let f = if free {
                [
                    0.,
                    if t < 900 { sign * 4. } else { -sign * 4. },
                    if oblique && t < 900 { sign * 0.03 } else { 0. },
                ]
            } else if held {
                [
                    0.,
                    if (650..1980).contains(&t) {
                        sign * 3.
                    } else if t >= 1980 {
                        -sign * 3.
                    } else {
                        0.
                    },
                    0.,
                ]
            } else {
                let z = if i == 0 {
                    match local {
                        650..850 | 1250..1450 => 1.2,
                        850..1250 => -1.2,
                        _ => 0.,
                    }
                } else if (650..1500).contains(&local) {
                    1.2
                } else {
                    0.
                };
                [0., 0., z]
            };
            if f != [0.; 3] {
                forces.push((g.payload.payload.id, f));
                expected[i] += DVec3::from_array(f) * 0.001;
            }
            if held && t % 107 == 0 {
                efforts.push((j, 0.0002 * sign));
            }
        }
        if t == 0 {
            p.drop_next_acknowledgement(p.owner())?;
        }
        o.apply(&efforts, &forces)?;
        p.advance_with_inputs_at(0, t, &efforts, &forces)?;
        o.verify(&p)?;
        graph.update(p.image().unwrap());
        let mut warm_grip = [false; 2];
        let mut warm_cross = false;
        for (i, g) in spec.assemblies.iter().enumerate() {
            let m = g.payload.tree.model_id;
            let own = p.payload_reactions(m).unwrap();
            warm_grip[i] = own.iter().filter(|r| warm(r)).count() >= 2;
            for r in own {
                expected[i] -= impulse(r);
            }
        }
        for r in p.cross_reactions() {
            for (i, g) in spec.assemblies.iter().enumerate() {
                if r.first_body == g.payload.payload.id {
                    expected[i] += impulse(&r.reaction);
                }
                if r.second_body == g.payload.payload.id {
                    expected[i] -= impulse(&r.reaction);
                }
            }
            let j = impulse(&r.reaction).length();
            maximum = maximum.max(j);
            cross_sum += j;
            peak_shear = peak_shear.max(DVec3::from_array(r.reaction.tangent_impulse).length());
            if j > 0. {
                first.get_or_insert(t + 1);
            }
            if warm(&r.reaction) {
                warm_cross = true;
                firstwarm.get_or_insert(t + 1);
                *pair_types.entry((r.first_body, r.second_body)).or_default() += 1;
            }
        }
        for (i, g) in spec.assemblies.iter().enumerate() {
            let part = p.payload_state(g.payload.tree.model_id).unwrap();
            max_residual = max_residual.max(
                (DVec3::from_array(part.velocity) * g.payload.payload.mass - expected[i]).length(),
            );
        }
        if warm_cross {
            active_ticks += 1;
            cross_and_grip += usize::from(warm_grip == [true, true]);
            if moves == 0 || (moves < 4 && t % 19 == 0) {
                let dest = (p.owner() + 1) % p.width();
                kill(
                    p.worker_pid(if moves % 2 == 0 { p.owner() } else { dest })
                        .unwrap(),
                )?;
                p.transfer(dest)?;
                drop(p);
                p = CoupledGraspFleetPool::resume(exe(), path.path())?;
                o.verify(&p)?;
                assert!(p.advance_with_inputs_at(0, t, &efforts, &forces).is_err());
                if moves == 0
                    && let Some(d) = std::env::var_os("GZ_CLUSTER_WITNESS")
                {
                    let d = Path::new(&d).join(if held {
                        "holding"
                    } else if oblique {
                        "oblique"
                    } else {
                        "impact"
                    });
                    fs::create_dir_all(&d)?;
                    fs::copy(path.path().join(FILE), d.join(FILE))?;
                    fs::write(d.join("scene.sdf"), &xml)?;
                    fs::write(
                        d.join("contact.json"),
                        serde_json::to_vec_pretty(
                            &serde_json::json!({"tick":p.tick(),"next_request_force":forces,"pairs":p.cross_reactions(),"owners":p.owners(),"sha256":format!("{:x}",Sha256::digest(p.image().unwrap()))}),
                        )?,
                    )?;
                }
                moves += 1;
            }
        } else if firstwarm.is_some() {
            released += 1;
        }
    }
    eprintln!(
        "{} frames={} full_native_and_all_receipts={count} first_impulse={first:?} first_warm={firstwarm:?} warm_cross_ticks={active_ticks} both_warm_grips_and_warm_cross={cross_and_grip} free_after_cross={released} real_killed_owner_or_destination_warm_transfers={moves} peak_cross_Ns={maximum} peak_shear_Ns={peak_shear} cumulative_cross_Ns={cross_sum} max_payload_momentum_residual_Ns={max_residual} warm_pairs={pair_types:?} sha256={:x}",
        if oblique {
            "oblique"
        } else if free {
            "approach"
        } else if held {
            "holding"
        } else {
            "stagger350"
        },
        count,
        graph.finalize()
    );
    assert!(max_residual < 1e-8);
    if oblique {
        assert!(peak_shear > 1e-3);
    }
    if case == Case::Staggered {
        assert!(first.is_none());
    } else {
        assert!(moves == 4 && active_ticks > 50 && released > 100 && maximum > 0.005);
        if held {
            assert!(cross_and_grip > 50);
        }
    }
    Ok(())
}
#[test]
fn two_independently_approaching_payloads_collide_and_release() -> Result<()> {
    exercise(Case::FreeParts)
}
#[test]
fn oblique_free_part_collision_reports_the_net_world_friction_impulse() -> Result<()> {
    exercise(Case::ObliqueParts)
}
#[test]
fn two_loaded_grippers_exchange_cross_assembly_contact_and_release() -> Result<()> {
    exercise(Case::LoadedGrippers)
}
#[test]
fn staggered_loading_tracks_one_ordinary_world_after_disjoint_solver_order_diverges() -> Result<()>
{
    exercise(Case::Staggered)
}

#[test]
fn maximum_fixed_cages_transfer_large_warm_physics_through_multiple_pages() -> Result<()> {
    let mut groups = Vec::new();
    for i in 0..2 {
        let offset = i * 2000;
        let y = (i * 4) as f64;
        let mut s = fixture::cage(8, false, 0.24, [0.26, 0.074, -0.021]);
        s.exact_revolute_locks = true;
        s.payload_can_sleep = false;
        s.tree.model_id += offset;
        for l in &mut s.tree.links {
            l.id += offset;
            l.position[1] += y;
        }
        for j in &mut s.tree.joints {
            j.joint_id += offset;
            j.parent_id += offset;
            j.child_id += offset;
            j.anchor[1] += y;
        }
        for l in &mut s.limits {
            l.joint_id += offset;
        }
        s.payload.id += offset;
        s.payload.position[1] += y;
        groups.push(GraspTreeSpec { payload: s });
    }
    let spec = GraspFleetSpec { assemblies: groups };
    let a = tempfile::tempdir()?;
    let b = tempfile::tempdir()?;
    let mut p = CoupledGraspFleetPool::create(exe(), a.path(), 3, spec.clone())?;
    let mut q = CoupledGraspFleetPool::create(exe(), b.path(), 3, spec.clone())?;
    let mut maxbytes = 0;
    let mut warm_total = 0;
    let mut up = 0;
    let mut down = 0;
    for t in 0..64 {
        let inputs = spec
            .assemblies
            .iter()
            .flat_map(|g| g.payload.tree.joints.iter().map(|j| (j.joint_id, 0.001)))
            .collect::<Vec<_>>();
        let loads = spec
            .assemblies
            .iter()
            .map(|g| (g.payload.payload.id, [0.01, -0.003, 0.004]))
            .collect::<Vec<_>>();
        p.advance_with_inputs_at(0, t, &inputs, &loads)?;
        q.advance_with_inputs(&inputs, &loads)?;
        if t == 8 || t == 24 {
            let dst = (p.owner() + 1) % 3;
            kill(p.worker_pid(if t == 8 { dst } else { p.owner() }).unwrap())?;
            p.drop_next_acknowledgement(dst)?;
            p.transfer(dst)?;
            up += p.traffic().uploaded_pages;
            down += p.traffic().downloaded_pages;
            drop(p);
            p = CoupledGraspFleetPool::resume(exe(), a.path())?;
        }
        assert_eq!(p.image(), q.image());
        assert_eq!(p.cross_reactions(), q.cross_reactions());
        let n = spec
            .assemblies
            .iter()
            .flat_map(|g| p.payload_reactions(g.payload.tree.model_id).unwrap())
            .filter(|r| warm(r))
            .count();
        warm_total = warm_total.max(n);
        maxbytes = maxbytes.max(p.image().unwrap().len());
    }
    let archive = fs::metadata(a.path().join(FILE))?.len();
    eprintln!(
        "loaded_maximum_cages full_bytes={maxbytes} archive_bytes={archive} warm={warm_total} upload_pages={up} download_pages={down} exact_steps=64 worker_deaths=2"
    );
    assert!(maxbytes > 65536 && warm_total == 16 && up >= 4 && down >= 8);
    Ok(())
}

#[test]
#[ignore = "requires the frozen original GZCCWK01 worker and warm archives"]
fn original_v1_coupled_cut_and_worker_still_advance_their_native_semantics() -> Result<()> {
    let original = std::env::var_os("GZ_COUPLED_V1_WORKER").ok_or("set GZ_COUPLED_V1_WORKER")?;
    let saved = std::env::var_os("GZ_COUPLED_V1_FIXTURES").ok_or("set GZ_COUPLED_V1_FIXTURES")?;
    for example in ["impact", "holding"] {
        let a = tempfile::tempdir()?;
        let b = tempfile::tempdir()?;
        for d in [&a, &b] {
            fs::copy(
                Path::new(&saved).join(example).join(FILE),
                d.path().join(FILE),
            )?;
        }
        let mut old = CoupledGraspFleetPool::resume(Path::new(&original), a.path())?;
        let mut new = CoupledGraspFleetPool::resume(exe(), b.path())?;
        for k in 0..64 {
            let forces = old
                .spec()
                .assemblies
                .iter()
                .map(|g| {
                    (
                        g.payload.payload.id,
                        [0.003, -g.payload.payload.position[1].signum() * 3., 0.002],
                    )
                })
                .collect::<Vec<_>>();
            let j = old.spec().assemblies[0].payload.tree.joints[0].joint_id;
            if k == 3 {
                let d = (old.owner() + 1) % old.width();
                old.transfer(d)?;
                new.transfer(d)?;
            }
            if k == 30 {
                old.reset()?;
                new.reset()?;
            }
            old.advance_with_inputs(&[(j, 0.0002)], &forces)?;
            new.advance_with_inputs(&[(j, 0.0002)], &forces)?;
            assert!(new.image().unwrap().starts_with(b"GZFCPL01"));
            assert_eq!(old.image(), new.image());
            assert_eq!(old.cross_reactions(), new.cross_reactions());
            assert_eq!(
                fs::read(a.path().join(FILE))?,
                fs::read(b.path().join(FILE))?
            );
        }
        eprintln!(
            "original GZCCCU01/{example} versus actual original GZCCWK01 binary: 64 identical archives, transfer and reset"
        );
    }
    Ok(())
}
