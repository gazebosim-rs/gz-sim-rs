// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![allow(deprecated)]
use gz_physics::{BodyState, PhysicsBackend};
use gz_physics_multifidelity::{MultiFidelityBackend, SoleFoot};
use gz_sim_core::{EntityId, Simulation, StepFault};
use gz_sim_scene::{
    BodyHandle, LoadError, Name, PhysicsEngine, damped_sole::last_step, load_world_with_sdf_step,
    queue_link_world_wrench, remove_model, reset_loaded_world,
};
const WELD: &str = include_str!("../../../examples/worlds/local_compliant_weld.sdf");
const COMPOUND: &str = include_str!("../../../examples/worlds/local_compliant_compound.sdf");
fn load(xml: &str) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        xml,
        "foot.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
}
fn entity<'a>(sim: &'a Simulation, name: &str) -> bevy_ecs::world::EntityRef<'a> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
}
fn body(sim: &Simulation, name: &str) -> (u64, gz_physics::BodyId) {
    let e = entity(sim, name);
    (
        e.get::<EntityId>().unwrap().get(),
        e.get::<BodyHandle>().unwrap().id(),
    )
}
fn pose(sim: &Simulation, id: gz_physics::BodyId) -> BodyState {
    sim.world().resource::<PhysicsBackend>().state(id).unwrap()
}
fn native(sim: &Simulation) -> &MultiFidelityBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
}

#[test]
fn either_link_can_be_the_root_and_reset_reproduces_the_loaded_poses() {
    let reversed = WELD.replace(
        "<parent>sole</parent><child>upper",
        "<parent>upper</parent><child>sole",
    );
    let mut root_baseline: Option<[BodyState; 2]> = None;
    for xml in [WELD, reversed.as_str()] {
        let mut s = load(xml).unwrap();
        let (low_id, lo) = body(&s, "default::boot::sole");
        let (high_id, hi) = body(&s, "default::boot::upper");
        let (_, support) = body(&s, "default::deck::plate");
        let before = [pose(&s, lo), pose(&s, hi)];
        let mut runs = vec![];
        for pass in 0..2 {
            s.set_paused(true);
            queue_link_world_wrench(&mut s, low_id, [2., 3., 1.], [0.; 3]).unwrap();
            queue_link_world_wrench(&mut s, high_id, [3., -1., 0.], [0.1, 0., 0.]).unwrap();
            s.tick().unwrap();
            assert_eq!(before, [pose(&s, lo), pose(&s, hi)]);
            s.step_exact(1).unwrap();
            assert_eq!(
                native(&s)
                    .sole_wrench(
                        SoleFoot::Weld {
                            lower: lo,
                            upper: hi
                        },
                        [support].into_iter()
                    )
                    .unwrap(),
                ([0.; 3], [0.; 3])
            );
            s.step_exact(499).unwrap();
            let row = last_step(&s).unwrap();
            assert!(row.step.normal_impulse > 0.);
            runs.push(([pose(&s, lo), pose(&s, hi)], row.step.impulse));
            if pass == 0 {
                reset_loaded_world(&mut s, |_| ()).unwrap();
                assert!(last_step(&s).is_none());
                assert_eq!(before, [pose(&s, lo), pose(&s, hi)]);
            }
        }
        assert_eq!(runs[0], runs[1]);
        if let Some(baseline) = root_baseline {
            for (a, b) in baseline.into_iter().zip(runs[0].0) {
                assert!(glam::DVec3::from(a.position).distance(b.position.into()) < 1e-11);
                assert!(glam::DVec3::from(a.velocity).distance(b.velocity.into()) < 1e-10);
            }
        }
        root_baseline = Some(runs[0].0);
        let deck = entity(&s, "default::deck").get::<EntityId>().unwrap().get();
        remove_model(&mut s, deck).unwrap();
        let vz = native(&s).welded_assembly_state(lo).unwrap().velocity[2];
        s.step_exact(1).unwrap();
        assert!(last_step(&s).is_none());
        assert!(
            (native(&s).welded_assembly_state(lo).unwrap().velocity[2] - vz + 9.81 * 0.0005).abs()
                < 1e-6
        );
    }
}
#[test]
fn upper_crossing_rejects_whole_interval_and_preserves_external_wrenches() {
    let modified = WELD
        .replace("0 0 0.13 0.04 -0.06 0.12", "-0.43 0 0.13 0 0 0")
        .replace("0.035 -0.018 -0.24", "20 0 0")
        .replace("0 0 0.14 0 0 0", "0 0 -0.13 0 0 0");
    let mut s = load(&modified).unwrap();
    let (_, lo) = body(&s, "default::boot::sole");
    let (id, hi) = body(&s, "default::boot::upper");
    let (_, deck) = body(&s, "default::deck::plate");
    let start = [pose(&s, lo), pose(&s, hi), pose(&s, deck)];
    queue_link_world_wrench(&mut s, id, [10., 0., 0.], [0.; 3]).unwrap();
    assert!(s.step_exact(1).is_err());
    assert!(
        s.world()
            .resource::<StepFault>()
            .error()
            .unwrap()
            .to_string()
            .contains("upper foot collider")
    );
    assert_eq!(s.elapsed().as_nanos(), 0);
    assert_eq!(start, [pose(&s, lo), pose(&s, hi), pose(&s, deck)]);
    assert!(last_step(&s).is_none());
    assert_eq!(
        native(&s)
            .sole_wrench(
                SoleFoot::Weld {
                    lower: lo,
                    upper: hi
                },
                [deck].into_iter()
            )
            .unwrap()
            .0,
        [10., 0., 0.]
    );
    // The upper face travels 10 mm through a 5 mm initial clearance. The same
    // sweep stays clear of the finite deck when 400 mm to one side.
    let mut clear = load(&modified.replace("-0.43 0 0.13", "-0.43 0.4 0.13")).unwrap();
    clear.step_exact(8).unwrap();
    assert_eq!(last_step(&clear).unwrap().step.normal_impulse, 0.);
}
#[test]
fn unsupported_geometry_is_rejected_at_admission() {
    let bad = [
        (
            WELD.replace("0 0 0.14 0 0 0", "0 0 -0.13 0 0 0"),
            "upper foot collider",
        ),
        (
            WELD.replace("0 0 0.14 0 0 0", "0.035 0 0.14 0 0 0"),
            "aggregate sole inertia",
        ),
        (
            WELD.replace("type=\"fixed\"", "type=\"revolute\""),
            "coated foot requires",
        ),
        (
            COMPOUND.replace("0 0 0.14 0 0 0", "0 0 0 0 0 0"),
            "exactly one collision centered",
        ),
        (
            COMPOUND.replace(
                "<box><size>0.10 0.12 0.12</size></box>",
                "<sphere><radius>0.03</radius></sphere>",
            ),
            "requires box colliders",
        ),
        (
            WELD.replace(
                "<collision name=\"sole_shape\">",
                "<collision name=\"sole_shape\"><pose>0.02 0 0 0 0 0</pose>",
            ),
            "sole collision must be centered",
        ),
        (
            WELD.replace("<foot>default::boot::sole", "<foot>default::boot::missing"),
            "missing sole foot",
        ),
    ];
    let extra=WELD.replace("<link name=\"upper\">","<link name=\"upper\"><collision name=\"extra\"><geometry><box><size>0.01 0.01 0.01</size></box></geometry></collision>");
    assert!(
        load(&extra)
            .err()
            .unwrap()
            .to_string()
            .contains("one box on each link")
    );
    for (xml, reason) in bad {
        let e = load(&xml).err().expect("invalid coating domain");
        assert!(e.to_string().contains(reason), "{e}");
    }
}
#[test]
fn compound_material_order_and_off_center_inertia_are_physical() {
    let i = COMPOUND.find("<collision name=\"sole_shape\">").unwrap();
    let j = COMPOUND.find("<collision name=\"upper_shape\">").unwrap();
    let end = COMPOUND[j..].find("</collision>").unwrap() + j + "</collision>".len();
    let rearranged = format!(
        "{}{}{}{}",
        &COMPOUND[..i],
        &COMPOUND[j..end],
        &COMPOUND[i..j],
        &COMPOUND[end..]
    );
    let (mut a, mut b) = (load(COMPOUND).unwrap(), load(&rearranged).unwrap());
    a.step_exact(500).unwrap();
    b.step_exact(500).unwrap();
    let (a, b) = (last_step(&a).unwrap(), last_step(&b).unwrap());
    assert_eq!(a.state.center, b.state.center);
    assert_eq!(a.step.impulse, b.step.impulse);
}

#[test]
fn both_representations_deliver_matching_reactions_to_moving_supports() {
    fn configure(xml: &str, seam: bool) -> String {
        let a = xml.find("    <model name=\"deck\">").unwrap();
        let b = a + xml[a..].find("</model>").unwrap() + "</model>".len();
        let old = &xml[a..b];
        let boundary = if seam { "0 0 0" } else { "0 0 0.8" };
        let drive = format!(
            "<plugin name=\"gz::sim::systems::PrescribedMotion\" filename=\"gz-sim-prescribed-motion-system\"><linear_velocity>0.01 0 0</linear_velocity><angular_velocity>{boundary}</angular_velocity></plugin>"
        );
        let deck = old.replace("<static>true</static>", &drive);
        if seam {
            let half = deck.replace("0.75 0.48", "0.375 0.48");
            let left = half.replace("0 0 -0.03", "-0.1875 0 -0.03");
            let right = half
                .replace("name=\"deck\"", "name=\"other\"")
                .replace("0 0 -0.03", "0.1875 0 -0.03");
            xml.replace(old, &(left + &right)).replace(
                "</support>",
                "</support><support>default::other::plate</support>",
            )
        } else {
            xml.replace(old, &deck)
        }
    }
    for seam in [false, true] {
        let (mut a, mut b) = (
            load(&configure(WELD, seam)).unwrap(),
            load(&configure(COMPOUND, seam)).unwrap(),
        );
        let mut impulse = [0.; 2];
        let mut work = [0.; 2];
        for _ in 0..500 {
            a.step_exact(1).unwrap();
            b.step_exact(1).unwrap();
            let (ar, br) = (
                gz_sim_scene::damped_sole::panel_steps(&a),
                gz_sim_scene::damped_sole::panel_steps(&b),
            );
            assert_eq!(ar.len(), if seam { 2 } else { 1 });
            for (i, (x, y)) in ar.iter().zip(br).enumerate() {
                impulse[i] += x.step.normal_impulse;
                work[i] += x.support_work_j.abs();
                assert!(x.step.impulse.distance(y.step.impulse) < 1e-11);
                assert!((x.support_work_j - y.support_work_j).abs() < 1e-12);
            }
        }
        let count = if seam { 2 } else { 1 };
        assert!(impulse[..count].iter().all(|n| *n > 0.1), "{impulse:?}");
        assert!(work[..count].iter().all(|n| *n > 0.), "{work:?}");
    }
}
