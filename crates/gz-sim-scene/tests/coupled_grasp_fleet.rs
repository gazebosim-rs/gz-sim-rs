// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "distributed-physics")]
use glam::DVec3;
use gz_physics::{BodyId, DynamicBackend, JointId, JointVelocityMotor, PhysicsBackend};
use gz_physics_multifidelity::{GRASP_FLEET_IMAGE_LIMIT, MultiFidelityBackend};
use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, JointHandle, PhysicsEngine,
    distributed_hinge::{GraspTreeSpec, distributed_grasp_fleet_specs},
    load_world_with_sdf_step,
};
use std::error::Error;
const XML: &str = include_str!("../../../examples/distributed_grasp_fleet.sdf");
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn load(xml: &str) -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        xml,
        "coupled-grasp.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
fn native(s: &Simulation) -> &MultiFidelityBackend {
    s.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
fn restore(sim: &mut Simulation, groups: &[(BodyId, BodyId); 2], data: &[u8]) -> Result<()> {
    Ok(sim
        .world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<MultiFidelityBackend>()
        .unwrap()
        .restore_grasp_fleet(groups, data)?)
}
struct Group {
    root: BodyId,
    part: BodyId,
    links: Vec<BodyId>,
    joints: Vec<(JointId, f64)>,
}
fn bind(sim: &Simulation, s: &GraspTreeSpec) -> Group {
    let body = |id| {
        sim.world()
            .get::<BodyHandle>(entity_by_id(sim.world(), id).unwrap())
            .unwrap()
            .id()
    };
    let p = &s.payload;
    let root = p
        .tree
        .links
        .iter()
        .find(|l| p.tree.joints.iter().all(|j| j.child_id != l.id))
        .unwrap();
    let mut joints = p
        .tree
        .joints
        .iter()
        .map(|j| {
            let e = entity_by_id(sim.world(), j.joint_id).unwrap();
            let sign = if sim
                .world()
                .get::<gz_sim_scene::Name>(e)
                .unwrap()
                .0
                .ends_with("::right_close")
            {
                1.
            } else {
                -1.
            };
            (
                j.joint_id,
                sim.world().get::<JointHandle>(e).unwrap().id(),
                sign,
            )
        })
        .collect::<Vec<_>>();
    joints.sort_by_key(|v| v.0);
    Group {
        root: body(root.id),
        part: body(p.payload.id),
        links: p.tree.links.iter().map(|l| body(l.id)).collect(),
        joints: joints.into_iter().map(|(_, id, s)| (id, s)).collect(),
    }
}
fn inputs(sim: &mut Simulation, groups: &[Group], parts: bool, t: u32) -> Result<[DVec3; 2]> {
    let mut be = sim.world_mut().resource_mut::<PhysicsBackend>();
    let mut loads = [DVec3::ZERO; 2];
    for (i, g) in groups.iter().enumerate() {
        let s = if i == 0 { 1. } else { -1. };
        if t == 0 || t == 780 {
            for &(id, sign) in &g.joints {
                be.set_joint_velocity_motor(
                    id,
                    Some(JointVelocityMotor {
                        target_velocity: if t == 0 { 6. * sign } else { -6. * sign },
                        max_force: 4.,
                    }),
                )?;
            }
        }
        loads[i] = if parts {
            if t < 720 {
                DVec3::new(0., 4. * s, 0.02 * s)
            } else {
                DVec3::new(0., -2. * s, 0.)
            }
        } else if (450..1000).contains(&t) {
            DVec3::Z * (s * 0.9)
        } else {
            DVec3::ZERO
        };
        if loads[i] != DVec3::ZERO {
            be.add_world_wrench(g.part, loads[i].to_array(), [0.; 3])?;
        }
    }
    Ok(loads)
}
fn exercise(xml: &str, parts: bool) -> Result<()> {
    let specs = distributed_grasp_fleet_specs(xml, "coupled-grasp.sdf", "default")?;
    let mut reference = load(xml)?;
    let mut actual = load(xml)?;
    let groups = specs
        .iter()
        .map(|s| bind(&reference, s))
        .collect::<Vec<_>>();
    let keys = [
        (groups[0].root, groups[0].part),
        (groups[1].root, groups[1].part),
    ];
    let parts_ids = [keys[0].1, keys[1].1];
    let mut held = 0;
    let mut swaps = 0;
    let mut max_bytes = 0;
    let mut previous_warm = false;
    let mut entries = 0;
    let mut releases = 0;
    let mut expected = [DVec3::ZERO; 2];
    let mut max_dp = 0f64;
    let mut max_tangent = 0f64;
    for t in 0..1200 {
        let ext = inputs(&mut reference, &groups, parts, t)?;
        inputs(&mut actual, &groups, parts, t)?;
        reference.step_exact(1)?;
        actual.step_exact(1)?;
        let r = native(&reference);
        let p = native(&actual);
        let image = p
            .checkpoint_grasp_fleet(&keys)
            .map_err(|e| format!("capture candidate at {}: {e:?}", t + 1))?;
        assert_eq!(
            image,
            r.checkpoint_grasp_fleet(&keys)?,
            "warm graph tick {}",
            t + 1
        );
        assert_eq!(&image[..8], b"GZFCPL02");
        max_bytes = max_bytes.max(image.len());
        let pp = r.sphere_pair_reaction(parts_ids[0], parts_ids[1])?;
        let reverse = r.sphere_pair_reaction(parts_ids[1], parts_ids[0])?;
        assert_eq!(pp.normal_impulse, reverse.normal_impulse);
        assert_eq!(
            pp.warmstart_normal_impulse,
            reverse.warmstart_normal_impulse
        );
        let world = |q: gz_physics_multifidelity::MovingReaction| {
            DVec3::from_array(q.normal) * q.normal_impulse + DVec3::from_array(q.tangent_impulse)
        };
        assert!((world(pp) + world(reverse)).length() < 1e-14);
        let mut warm = if parts {
            pp.warmstart_normal_impulse > 0.
        } else {
            false
        };
        max_tangent = max_tangent.max(DVec3::from_array(pp.tangent_impulse).length());
        for i in 0..2 {
            let part = groups[i].part;
            expected[i] += ext[i] * 1e-3 + if i == 0 { world(pp) } else { world(reverse) };
            for (j, g) in groups.iter().enumerate() {
                for &link in &g.links {
                    let q = r.sphere_pair_reaction(part, link)?;
                    assert_eq!(q, p.sphere_pair_reaction(part, link)?);
                    expected[i] += world(q);
                    if !parts && i != j && q.warmstart_normal_impulse > 0. {
                        warm = true
                    }
                }
            }
            let pbody =
                DVec3::from_array(p.state(part).unwrap().velocity) * specs[i].payload.payload.mass;
            max_dp = max_dp.max((expected[i] - pbody).length());
        }
        if warm && !previous_warm {
            entries += 1;
        }
        if !warm && previous_warm {
            releases += 1;
        }
        held += u32::from(warm);
        previous_warm = warm;
        if (warm && swaps == 0) || (t == 720 && swaps == 1) || (t == 999 && swaps == 2) {
            let mut fresh = load(xml)?;
            // Reject a corrupt payload or changed pairing while the destination remains cold.
            let mut invalid = image.clone();
            let end = invalid.len() - 1;
            invalid[end] ^= 0x80;
            assert!(restore(&mut fresh, &keys, &invalid).is_err());
            assert!(
                restore(
                    &mut fresh,
                    &[(keys[0].0, keys[1].1), (keys[1].0, keys[0].1)],
                    &image
                )
                .is_err()
            );
            assert!(restore(&mut fresh, &[keys[1], keys[0]], &image).is_err());
            restore(&mut fresh, &keys, &image)
                .map_err(|e| format!("restore live cut at {}: {e:?}", t + 1))?;
            assert_eq!(native(&fresh).checkpoint_grasp_fleet(&keys)?, image);
            actual = fresh;
            swaps += 1;
        }
    }
    println!(
        "payload_to_payload={parts} warm_frames={held} entries={entries} releases={releases} restores={swaps} max_native_bytes={max_bytes} max_payload_momentum_error={max_dp:e} max_payload_pair_friction={max_tangent:e}"
    );
    assert!(
        held >= 50
            && entries > 0
            && releases > 0
            && swaps == 3
            && max_bytes < GRASP_FLEET_IMAGE_LIMIT
            && max_dp < 1e-10
    );
    if parts {
        assert!(max_tangent > 1e-8);
    }
    Ok(())
}
#[test]
fn ballistic_payload_collision_roundtrips_loaded_normal_and_shear() -> Result<()> {
    let xml = XML
        .replace("0.317 0.954 0.58", "2.0 0.954 0.58")
        .replace("0.317 -0.946 0.58", "2.0 -0.946 0.58");
    exercise(&xml, true)
}
#[test]
fn both_grippers_warm_contact_with_the_other_assemblies_payloads_survives_recovery() -> Result<()> {
    let swapped = XML
        .replace("payload=\"north_part\"", "payload=\"MARK\"")
        .replace("payload=\"south_part\"", "payload=\"north_part\"")
        .replace("MARK", "south_part");
    exercise(&swapped, false)
}

fn separated_member(
    xml: &str,
    name: &str,
    payload: &str,
    world_group: u64,
) -> Result<(Simulation, Group)> {
    let doc = roxmltree::Document::parse(xml)?;
    let world = doc.descendants().find(|e| e.has_tag_name("world")).unwrap();
    let mut removed = world
        .children()
        .filter(|e| {
            e.has_tag_name("plugin")
                || e.has_tag_name("model")
                    && ![Some(name), Some(payload)].contains(&e.attribute("name"))
        })
        .map(|e| e.range())
        .collect::<Vec<_>>();
    let mut selected = xml.to_string();
    while let Some(range) = removed.pop() {
        selected.replace_range(range, "");
    }
    let recipe = gz_sim_scene::distributed_payload_tree_spec(&selected, "clear.sdf", "default")?;
    let mut sim = load(&selected)?;
    let group = bind(&sim, &GraspTreeSpec { payload: recipe });
    let mut physics = sim.world_mut().resource_mut::<PhysicsBackend>();
    let b = physics.downcast_mut::<MultiFidelityBackend>().unwrap();
    for &id in &group.links {
        b.set_self_collision_group(id, Some(world_group))?;
    }
    Ok((sim, group))
}

#[test]
fn drained_grippers_import_their_actual_coordinates_before_a_new_payload_collision() -> Result<()> {
    // Same positions and ±4 N loads as the captured independent-worker first refusal.
    let xml = XML.replace("<pose>0.317 ", "<pose>2.0 ");
    let fleets = distributed_grasp_fleet_specs(&xml, "clear.sdf", "default")?;
    let mut full = load(&xml)?;
    let g = fleets.iter().map(|s| bind(&full, s)).collect::<Vec<_>>();
    let keys = [(g[0].root, g[0].part), (g[1].root, g[1].part)];
    let mut halves = [
        separated_member(&xml, "south", "south_part", fleets[0].payload.tree.model_id)?,
        separated_member(&xml, "north", "north_part", fleets[1].payload.tree.model_id)?,
    ];
    let dt = std::time::Duration::from_millis(1);
    let mut warm = 0;
    let mut imported: Option<PhysicsBackend> = None;
    for t in 0..700 {
        for (i, group) in g.iter().enumerate() {
            let force = [0., if i == 0 { 4. } else { -4. }, 0.];
            let mut be = full.world_mut().resource_mut::<PhysicsBackend>();
            be.add_world_wrench(group.part, force, [0.; 3])?;
            if t == 500 {
                be.add_joint_effort(group.joints[0].0, 0.0002)?;
            }
            let (half, member) = &mut halves[i];
            if t < 500 {
                if t == 0 {
                    for ((global, sign), (local, _)) in group.joints.iter().zip(&member.joints) {
                        let motor = Some(JointVelocityMotor {
                            target_velocity: sign * 0.3,
                            max_force: 0.002,
                        });
                        be.set_joint_velocity_motor(*global, motor)?;
                        half.world_mut()
                            .resource_mut::<PhysicsBackend>()
                            .set_joint_velocity_motor(*local, motor)?;
                    }
                }
                half.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .add_world_wrench(member.part, force, [0.; 3])?;
                half.step_exact(1)?;
            }
            if let Some(other) = &mut imported {
                other.add_world_wrench(group.part, force, [0.; 3])?;
            }
        }
        full.step_exact(1)?;
        if t == 499 {
            let mut images = Vec::new();
            for (half, member) in &mut halves {
                half.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .add_joint_effort(member.joints[0].0, 0.0002)?;
                images.push(
                    native(half)
                        .checkpoint_clear_grasp_member(member.root, member.part)?
                        .ok_or("source not clear")?,
                );
            }
            let mut target = load(&xml)?;
            let mut backend = target
                .world_mut()
                .remove_resource::<PhysicsBackend>()
                .unwrap();
            let dst = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
            assert!(
                dst.restore_clear_grasp_fleet(&keys, [&images[1], &images[0]])
                    .is_err()
            );
            let mut bad = images[0].clone();
            *bad.last_mut().unwrap() ^= 1;
            assert!(
                dst.restore_clear_grasp_fleet(&keys, [&bad, &images[1]])
                    .is_err()
            );
            dst.restore_clear_grasp_fleet(&keys, [&images[0], &images[1]])?;
            for (i, group) in g.iter().enumerate() {
                for (&a, &b) in group
                    .links
                    .iter()
                    .chain([&group.part])
                    .zip(halves[i].1.links.iter().chain([&halves[i].1.part]))
                {
                    assert_eq!(
                        dst.state(a).unwrap(),
                        native(&halves[i].0).state(b).unwrap()
                    );
                }
            }
            let image = dst
                .checkpoint_grasp_fleet(&keys)
                .map_err(|e| format!("full graph after remap: {e:?}"))?;
            let mut replacement = load(&xml)?;
            restore(&mut replacement, &keys, &image)
                .map_err(|e| format!("reopen new coupled graph: {e:?}"))?;
            backend = replacement
                .world_mut()
                .remove_resource::<PhysicsBackend>()
                .unwrap();
            println!(
                "native clear cuts at tick 500, {} and {} bytes",
                images[0].len(),
                images[1].len()
            );
            imported = Some(backend);
        } else if let Some(backend) = &mut imported {
            let dst = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
            dst.step(dt)?;
            let r = native(&full);
            if let Some(dir) = std::env::var_os("GZ_GRASP_CLEAR_PROBE").filter(|s| !s.is_empty())
                && (528..=538).contains(&t)
            {
                let dir = std::path::PathBuf::from(dir);
                std::fs::write(
                    dir.join(format!("ordinary-{}.gzfc", t + 1)),
                    r.checkpoint_grasp_fleet(&keys)?,
                )?;
                std::fs::write(
                    dir.join(format!("imported-{}.gzfc", t + 1)),
                    dst.checkpoint_grasp_fleet(&keys)?,
                )?;
            }
            for group in &g {
                for &id in group.links.iter().chain([&group.part]) {
                    assert_eq!(
                        dst.state(id).unwrap(),
                        r.state(id).unwrap(),
                        "body {} at {}",
                        id.raw(),
                        t + 1
                    );
                }
            }
            let expected = r.sphere_pair_reaction(g[0].part, g[1].part)?;
            let actual = dst.sphere_pair_reaction(g[0].part, g[1].part)?;
            assert_eq!(
                expected.normal_impulse,
                actual.normal_impulse,
                "payload impact at {}",
                t + 1
            );
            assert_eq!(expected.tangent_impulse, actual.tangent_impulse);
            warm += usize::from(actual.normal_impulse > 0.);
        }
    }
    assert!(warm > 0);
    println!(
        "drained imported ongoing coupled contact: {} warm frames",
        warm
    );
    Ok(())
}

fn pressing(be: &mut PhysicsBackend, g: &Group, inward: f64, t: u32) -> Result<()> {
    if t == 0 {
        for &(id, sign) in &g.joints {
            be.set_joint_velocity_motor(
                id,
                Some(JointVelocityMotor {
                    target_velocity: 6. * sign,
                    max_force: 4.,
                }),
            )?;
        }
    }
    if t >= 650 {
        be.add_world_wrench(
            g.part,
            [0., if t < 1980 { 3. * inward } else { -3. * inward }, 0.],
            [0.; 3],
        )?;
    }
    if t.is_multiple_of(107) {
        be.add_joint_effort(g.joints[0].0, 0.0002 * inward)?;
    }
    Ok(())
}
#[test]
fn promote_two_still_loaded_grasps_before_their_fingers_make_foreign_contact() -> Result<()> {
    loaded_merge(None)
}

#[test]
fn retain_a_loaded_gripper_and_its_recycling_history_when_the_other_worker_is_clear() -> Result<()>
{
    loaded_merge(Some(1))?;
    loaded_merge(Some(0))
}
fn loaded_merge(clear: Option<usize>) -> Result<()> {
    let xml = XML
        .replace("-0.95 0.58", "-0.52 0.58")
        .replace(" 0.95 0.58", " 0.52 0.58")
        .replace("-0.946 0.58", "-0.516 0.58")
        .replace(" 0.954 0.58", " 0.524 0.58");
    let xml = match clear {
        Some(1) => xml.replace("0.317 0.524", "2.0 0.524"),
        Some(0) => xml.replace("0.317 -0.516", "2.0 -0.516"),
        _ => xml,
    };
    let specs = distributed_grasp_fleet_specs(&xml, "pressing.sdf", "default")?;
    let mut full = load(&xml)?;
    let groups = specs.iter().map(|s| bind(&full, s)).collect::<Vec<_>>();
    let keys = [
        (groups[0].root, groups[0].part),
        (groups[1].root, groups[1].part),
    ];
    let mut halves = [
        separated_member(&xml, "south", "south_part", specs[0].payload.tree.model_id)?,
        separated_member(&xml, "north", "north_part", specs[1].payload.tree.model_id)?,
    ];
    let mut merged: Option<PhysicsBackend> = None;
    let dt = std::time::Duration::from_millis(1);
    let mut count = 0;
    for t in 0..2200 {
        for (i, g) in groups.iter().enumerate() {
            let sign = if i == 0 { 1. } else { -1. };
            pressing(
                &mut full.world_mut().resource_mut::<PhysicsBackend>(),
                g,
                sign,
                t,
            )?;
            if let Some(be) = &mut merged {
                pressing(be, g, sign, t)?;
            } else {
                let (s, h) = &mut halves[i];
                pressing(
                    &mut s.world_mut().resource_mut::<PhysicsBackend>(),
                    h,
                    sign,
                    t,
                )?;
                s.step_exact(1)?;
            }
        }
        full.step_exact(1)?;
        if t == 998 {
            let mut cuts = Vec::new();
            for (i, (s, h)) in halves.iter().enumerate() {
                let b = native(s);
                if Some(i) == clear {
                    cuts.push(
                        b.checkpoint_clear_grasp_member(h.root, h.part)?
                            .ok_or("expected clear member")?,
                    );
                    continue;
                }
                assert!(b.checkpoint_clear_grasp_member(h.root, h.part)?.is_none());
                assert_eq!(
                    h.links
                        .iter()
                        .filter(|&&l| b
                            .payload_tree_reaction(l, h.part)
                            .unwrap()
                            .warmstart_normal_impulse
                            > 0.)
                        .count(),
                    2
                );
                cuts.push(
                    b.checkpoint_warm_grasp_member(h.root, h.part)?
                        .ok_or("missing warm native contact")?,
                );
            }
            let mut target = load(&xml)?;
            let mut backend = target
                .world_mut()
                .remove_resource::<PhysicsBackend>()
                .unwrap();
            let dst = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
            assert!(
                dst.prepare_warm_grasp_fleet(&keys, [&cuts[1], &cuts[0]])
                    .is_err()
            );
            let corrupt = if clear == Some(0) { 1 } else { 0 };
            let mut invalid = cuts[corrupt].clone();
            let end = invalid.len();
            invalid[end - 8..].copy_from_slice(&f64::NAN.to_le_bytes());
            use sha2::{Digest, Sha256};
            let checksum = Sha256::digest(&invalid[40..]);
            invalid[8..40].copy_from_slice(&checksum);
            let mut wrong = [cuts[0].as_slice(), cuts[1].as_slice()];
            wrong[corrupt] = &invalid;
            assert!(dst.prepare_warm_grasp_fleet(&keys, wrong).is_err());
            dst.prepare_warm_grasp_fleet(&keys, [&cuts[0], &cuts[1]])?;
            for (i, g) in groups.iter().enumerate() {
                let (s, h) = &halves[i];
                for (&l, &k) in g.links.iter().zip(&h.links) {
                    assert_eq!(dst.state(l), native(s).state(k));
                    assert_eq!(
                        dst.payload_tree_reaction(l, g.part)?,
                        native(s).payload_tree_reaction(k, h.part)?
                    );
                }
            }
            let graph = dst.checkpoint_grasp_fleet(&keys)?;
            if let Some(dir) = std::env::var_os("GZ_GRASP_WARM_PROBE") {
                let dir = std::path::PathBuf::from(dir);
                std::fs::write(dir.join("imported-999.gzfc"), &graph)?;
                std::fs::write(
                    dir.join("ordinary-999.gzfc"),
                    native(&full).checkpoint_grasp_fleet(&keys)?,
                )?;
                for (i, (sim, _)) in halves.iter().enumerate() {
                    std::fs::write(
                        dir.join(format!("member-{i}-999.gzfr")),
                        native(sim).checkpoint_grasp_revolute_tree()?,
                    )?;
                }
            }
            let mut second = load(&xml)?;
            restore(&mut second, &keys, &graph)?;
            println!(
                "t999 source packets {} + {} bytes, clear member {clear:?}, remapped and reopened combined graph {} bytes",
                cuts[0].len(),
                cuts[1].len(),
                graph.len()
            );
            merged = Some(
                second
                    .world_mut()
                    .remove_resource::<PhysicsBackend>()
                    .unwrap(),
            );
        } else if let Some(be) = &mut merged {
            let dst = be.downcast_mut::<MultiFidelityBackend>().unwrap();
            dst.step(dt)?;
            let original = native(&full);
            if let Some(dir) = std::env::var_os("GZ_GRASP_WARM_PROBE")
                && t == 999
            {
                let dir = std::path::PathBuf::from(dir);
                std::fs::write(
                    dir.join("imported-1000.gzfc"),
                    dst.checkpoint_grasp_fleet(&keys)?,
                )?;
                std::fs::write(
                    dir.join("ordinary-1000.gzfc"),
                    original.checkpoint_grasp_fleet(&keys)?,
                )?;
            }
            for g in &groups {
                for &l in g.links.iter().chain([&g.part]) {
                    assert_eq!(
                        dst.state(l),
                        original.state(l),
                        "body {} at tick {}",
                        l.raw(),
                        t + 1
                    );
                }
                for &l in &g.links {
                    assert_eq!(
                        dst.payload_tree_reaction(l, g.part)?,
                        original.payload_tree_reaction(l, g.part)?,
                        "own contact at {}",
                        t + 1
                    );
                }
            }
            let mut foreign = false;
            for &a in &groups[0].links {
                for &b in &groups[1].links {
                    let r = dst.sphere_pair_reaction(a, b)?;
                    assert_eq!(r, original.sphere_pair_reaction(a, b)?);
                    foreign |= r.warmstart_normal_impulse > 0.;
                }
            }
            count += usize::from(foreign);
        }
    }
    if clear.is_none() {
        assert!(count > 700);
    }
    println!("continued through {count} warm contacts between independently loaded grippers");
    Ok(())
}
