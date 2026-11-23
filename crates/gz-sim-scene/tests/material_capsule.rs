// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics::{BodyState, PhysicsBackend};
use gz_physics_xpbd::{CapsuleRegion, MaterialFeature};
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, deformable::bodies, load_world_with_sdf_step,
    reset_loaded_world,
};
const CABLE: &str = include_str!("../../../examples/material_capsule_cable.sdf");
const CLOTH: &str = include_str!("../../../examples/material_capsule_cloth.sdf");
fn load(xml: &str) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_with_sdf_step(
        xml,
        "material_capsule.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
fn guide(sim: &Simulation) -> BodyState {
    let e = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::roller::guide")
        })
        .unwrap();
    sim.world()
        .resource::<PhysicsBackend>()
        .state(e.get::<BodyHandle>().unwrap().id())
        .unwrap()
}
#[test]
fn ordinary_bevy_scene_exposes_material_support_and_reset() {
    for (name, xml) in [("cable", CABLE), ("cloth", CLOTH)] {
        for n in [1, 2, 4, 8] {
            let mut results = vec![];
            for enabled in [false, true] {
                let xml = xml
                    .replace("substeps=\"4\"", &format!("substeps=\"{n}\""))
                    .replace(
                        "capsule_contact=\"material\"",
                        &format!(
                            "capsule_contact=\"{}\"",
                            if enabled { "material" } else { "particles" }
                        ),
                    );
                let mut sim = load(&xml).unwrap();
                let initial = guide(&sim);
                let mut history = vec![];
                let mut times = vec![];
                let (mut peak, mut terminal, mut worst_load_error) = (0f64, 0f64, 0f64);
                let mut levels = [0.; 2];
                let mut received = false;
                for tick in 1..=200 {
                    sim.step_exact(1).unwrap_or_else(|e| {
                        panic!(
                            "{name} n={n} material={enabled} tick={tick}: {e}, {:?}",
                            sim.fault()
                        )
                    });
                    let sample = bodies(&sim).next().unwrap().1;
                    let bot = guide(&sim);
                    let t = tick as f64 * 0.005;
                    assert!(
                        DVec3::from(bot.position)
                            .distance(DVec3::from(initial.position) + DVec3::Y * (0.06 * t))
                            < 1e-12
                    );
                    assert_eq!(bot.angular_velocity, initial.angular_velocity);
                    let loads = sample.last_capsule_material_loads();
                    if enabled {
                        for (id, r) in sample
                            .last_boundary_impulses()
                            .iter()
                            .take(if name == "cloth" { 2 } else { 1 })
                            .enumerate()
                        {
                            let j = loads
                                .iter()
                                .filter(|v| v.boundary == id)
                                .map(|v| DVec3::from(v.linear))
                                .sum::<DVec3>();
                            let l = loads
                                .iter()
                                .filter(|v| v.boundary == id)
                                .map(|v| DVec3::from(v.angular))
                                .sum::<DVec3>();
                            worst_load_error = worst_load_error
                                .max(j.distance(r.linear.into()))
                                .max(l.distance(r.angular.into()));
                        }
                        for v in loads {
                            assert!(v.boundary < if name == "cloth" { 2 } else { 1 });
                            assert!((0.0..=0.005000001).contains(&v.first_time));
                            let nodal = v.nodal.iter().map(|p| DVec3::from(*p)).sum::<DVec3>();
                            worst_load_error =
                                worst_load_error.max((nodal + DVec3::from(v.linear)).length());
                            received |= if name == "cable" {
                                v.feature == MaterialFeature::Segment([0, 1])
                                    && v.region == CapsuleRegion::Barrel
                            } else {
                                v.feature == MaterialFeature::Triangle([0, 1, 3])
                                    && v.region == CapsuleRegion::PositiveCap
                            };
                            if times.is_empty() {
                                times.push(t - 0.005 + v.first_time);
                            }
                        }
                        if n == 4 {
                            history.push((
                                sample.particles().to_vec(),
                                loads.to_vec(),
                                sample.last_boundary_impulses().to_vec(),
                                bot,
                            ));
                        }
                    } else {
                        assert!(loads.is_empty());
                    }
                    let force = -sample
                        .last_boundary_impulses()
                        .iter()
                        .take(if name == "cloth" { 2 } else { 1 })
                        .map(|v| v.linear[2])
                        .sum::<f64>()
                        / 0.005;
                    peak = peak.max(force);
                    if tick > 160 {
                        terminal += force / 40.;
                    }
                    if tick == 200 {
                        let total = sample
                            .particles()
                            .iter()
                            .map(|p| p.inverse_mass.recip())
                            .sum::<f64>();
                        levels = [
                            sample
                                .particles()
                                .iter()
                                .map(|p| p.position[2] / p.inverse_mass)
                                .sum::<f64>()
                                / total,
                            total * 9.81,
                        ];
                    }
                }
                println!(
                    "{name} n={n} material={enabled}: first={:?} s, peak={peak:e} N, final mean={terminal:e} N, weight={:e} N, COM z={:e} m, attributed load residual={worst_load_error:e}",
                    times.first(),
                    levels[1],
                    levels[0]
                );
                if enabled {
                    assert!(received && worst_load_error < 1e-10);
                    assert!((terminal - levels[1]).abs() < 1e-8);
                    assert!((levels[0] - 0.72).abs() < 1e-9);
                    if n == 4 {
                        reset_loaded_world(&mut sim, |_| ()).unwrap();
                        assert!(
                            bodies(&sim)
                                .next()
                                .unwrap()
                                .1
                                .last_capsule_material_loads()
                                .is_empty()
                        );
                        for (p, l, r, b) in history {
                            sim.step_exact(1).unwrap();
                            let sample = bodies(&sim).next().unwrap().1;
                            assert_eq!(sample.particles(), p);
                            assert_eq!(sample.last_capsule_material_loads(), l);
                            assert_eq!(sample.last_boundary_impulses(), r);
                            assert_eq!(guide(&sim), b);
                        }
                    }
                } else {
                    assert!(peak < 1e-10);
                }
                results.push(levels[0]);
            }
            assert!(results[1] > results[0] + 0.20, "{results:?}");
        }
    }
}
#[test]
fn material_contact_rejects_conflicting_ownership_and_unimplemented_friction() {
    for xml in [
        CABLE.replace("friction=\"0\"", "friction=\"0.2\""),
        CABLE.replace("capsule_contact=\"material\"", "capsule_contact=\"typo\""),
        include_str!("../../../examples/dynamic_capsule_cable.sdf")
            .replace(
                "self_contact=",
                "capsule_contact=\"material\" self_contact=",
            )
            .replace("friction=\"0.15\"", "friction=\"0\"")
            .replace("coupling=\"two_way\"", "coupling=\"one_way\""),
    ] {
        let e = load(&xml).err().unwrap();
        assert!(e.to_string().contains("capsule"), "{e}");
    }
}

#[test]
fn surviving_material_loads_keep_the_original_sdf_slot_after_removal() {
    let mut sim = load(CLOTH).unwrap();
    sim.step_exact(90).unwrap();
    let old = bodies(&sim).next().unwrap().1.last_capsule_material_loads();
    assert!(old.iter().any(|x| x.boundary == 0) && old.iter().any(|x| x.boundary == 1));
    let id = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::roller"))
        .unwrap()
        .get::<gz_sim_core::EntityId>()
        .unwrap()
        .get();
    gz_sim_scene::remove_model(&mut sim, id).unwrap();
    sim.step_exact(1).unwrap();
    let s = bodies(&sim).next().unwrap().1;
    assert_eq!(s.last_boundary_impulses()[0].linear, [0.; 3]);
    assert!(
        !s.last_capsule_material_loads().is_empty()
            && s.last_capsule_material_loads()
                .iter()
                .all(|l| l.boundary == 1)
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(90).unwrap();
    let restored = bodies(&sim).next().unwrap().1.last_capsule_material_loads();
    assert!(restored.iter().any(|l| l.boundary == 0) && restored.iter().any(|l| l.boundary == 1));
}
