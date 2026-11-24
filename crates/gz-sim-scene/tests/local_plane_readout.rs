// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics_xpbd::ParticleBoundaryImpulse;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, PhysicsEngine,
    deformable::{DeformableSample, bodies},
    load_world_with_sdf_step, remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use std::collections::BTreeMap;

const WORLD: &str = include_str!("../../../examples/worlds/local_floor_impulses.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "floor.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn sample<'a>(sim: &'a Simulation, name: &str) -> &'a DeformableSample {
    bodies(sim).find(|(n, _)| *n == name).unwrap().1
}
fn rows(sim: &Simulation, name: &str) -> Vec<ParticleBoundaryImpulse> {
    sample(sim, name)
        .last_plane_impulses()
        .unwrap()
        .unwrap()
        .to_vec()
}
fn retire(sim: &mut Simulation) {
    let id = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::spacer"))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    remove_model(sim, id).unwrap();
}

#[test]
fn native_floor_and_pad_keep_separate_identities_and_node_momentum_on_contact_removal_and_reset() {
    // Leave the pad-supported cable uninstrumented in an otherwise observed coupled group.
    let mixed = WORLD.replace(
        "name=\"on_pad\" plane_impulses=\"true\"",
        "name=\"on_pad\" plane_impulses=\"false\"",
    );
    for xml in [
        mixed.clone(),
        mixed
            .lines()
            .filter(|l| !l.contains("coupling=\"articulated_wrench\""))
            .collect::<Vec<_>>()
            .join("\n"),
    ] {
        let pads = xml.contains("coupling=\"articulated_wrench\"");
        let mut sim = load(&xml);
        let mut sham = load(&xml.replace("plane_impulses=\"true\"", "plane_impulses=\"false\""));
        let mut first_epoch = vec![];
        for epoch in 0..2 {
            assert!(rows(&sim, "sliding").is_empty());
            let mut seen_landing = false;
            assert!(sample(&sim, "on_pad").last_plane_impulses().is_none());
            for tick in 1..=50 {
                if tick == 12 {
                    for s in [&mut sim, &mut sham] {
                        retire(s);
                    }
                    let previous = rows(&sim, "sliding");
                    sim.tick().unwrap();
                    assert_eq!(rows(&sim, "sliding"), previous);
                }
                let before: BTreeMap<_, _> = bodies(&sim)
                    .map(|(n, s)| (n.to_string(), s.particles().to_vec()))
                    .collect();
                sim.step_exact(1).unwrap();
                sham.step_exact(1).unwrap();
                let mut now = vec![];
                for (name, s) in bodies(&sim) {
                    let ref_s = sample(&sham, name);
                    assert_eq!(s.particles(), ref_s.particles());
                    assert_eq!(s.last_boundary_impulses(), ref_s.last_boundary_impulses());
                    assert_eq!(s.last_pad_loads(), ref_s.last_pad_loads());
                    if name == "on_pad" {
                        if pads {
                            assert!(
                                s.last_pad_loads()
                                    .iter()
                                    .any(|v| v.boundary == 2 && v.normal[2] < 0.)
                            );
                        }
                        continue;
                    }
                    let loads = s.last_plane_impulses().unwrap().unwrap();
                    if name == "falling" && !loads.is_empty() {
                        seen_landing = true;
                    }
                    let mut sum = DVec3::ZERO;
                    for (node, (old, new)) in before[name].iter().zip(s.particles()).enumerate() {
                        let v = loads.iter().find(|r| r.particle == node);
                        let measured = v.map_or(DVec3::ZERO, |r| {
                            assert_eq!(r.boundary, 1);
                            DVec3::from(r.normal) + DVec3::from(r.tangent)
                        });
                        let expected = -(DVec3::from(new.velocity)
                            - DVec3::from(old.velocity)
                            - DVec3::new(0., 0., -0.05))
                            / new.inverse_mass;
                        assert!(
                            expected.distance(measured) < 2e-11,
                            "{name} {tick} {node} {expected} {measured}"
                        );
                        sum += measured;
                    }
                    assert!(sum.distance(s.last_boundary_impulses()[1].linear.into()) < 1e-12);
                    now.push(loads.to_vec());
                }
                if epoch == 0 {
                    first_epoch.push(now);
                } else {
                    assert_eq!(first_epoch[tick - 1], now);
                }
            }
            assert!(seen_landing);
            for s in [&mut sim, &mut sham] {
                reset_loaded_world(s, |_| ()).unwrap();
            }
        }
    }
}

fn one_material() -> String {
    let doc = roxmltree::Document::parse(WORLD).unwrap();
    let mut ranges: Vec<_> = doc
        .descendants()
        .filter(|n| {
            (n.has_tag_name("cable") && n.attribute("name") != Some("sliding"))
                || (n.has_tag_name("particle_visual") && n.attribute("material") != Some("sliding"))
        })
        .map(|n| n.range())
        .collect();
    let mut xml = WORLD.to_owned();
    for r in ranges.drain(..).rev() {
        xml.replace_range(r, "");
    }
    xml
}

#[test]
fn each_public_step_integrates_all_four_exchanges_against_independent_native_ticks() {
    let xml = one_material();
    let outer_xml = xml.replace(
        "gz-sim-deformables-system\"",
        "gz-sim-deformables-system\" rigid_exchanges=\"4\"",
    );
    let mut outer = load(&outer_xml);
    let mut fine = load(&xml.replace("0.005</max_step_size>", "0.00125</max_step_size>"));
    for i in 0..80 {
        if i == 10 {
            retire(&mut outer);
            retire(&mut fine);
        }
        let mut total = BTreeMap::<_, (DVec3, DVec3)>::new();
        for _ in 0..4 {
            fine.step_exact(1).unwrap();
            for j in rows(&fine, "sliding") {
                let v = total.entry((j.boundary, j.particle)).or_default();
                v.0 += DVec3::from(j.normal);
                v.1 += DVec3::from(j.tangent);
            }
        }
        outer.step_exact(1).unwrap();
        let obs = sample(&outer, "sliding");
        assert_eq!(obs.rigid_material_exchanges(), 4);
        assert_eq!(obs.particles(), sample(&fine, "sliding").particles());
        let actual = rows(&outer, "sliding");
        assert_eq!(actual.len(), 5);
        for j in actual {
            let (n, t) = total[&(j.boundary, j.particle)];
            assert_eq!(j.normal, n.to_array());
            assert_eq!(j.tangent, t.to_array());
        }
    }
}
#[test]
fn invalid_spelling_or_absent_authored_support_is_reported_at_load() {
    for xml in [
        WORLD.replace("plane_impulses=\"true\"", "plane_impulses=\"yes\""),
        WORLD
            .lines()
            .filter(|l| !l.contains("<plane point="))
            .collect::<Vec<_>>()
            .join("\n"),
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml,
                "floor.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .err()
            .unwrap()
            .to_string()
            .contains("plane_impulses")
        );
    }
}

#[test]
fn failed_physical_solve_does_not_publish_a_partial_public_tick() {
    let mut sim = load(WORLD);
    sim.step_exact(3).unwrap();
    let before = rows(&sim, "sliding");
    let stamp = sample(&sim, "sliding").sim_time();
    gz_sim_scene::deformable::set_material_force(&mut sim, "sliding", [1e300, 0., 0.]).unwrap();
    assert!(sim.step_exact(1).is_err());
    assert!(sim.is_faulted());
    assert_eq!(sample(&sim, "sliding").sim_time(), stamp);
    assert_eq!(rows(&sim, "sliding"), before);
}
