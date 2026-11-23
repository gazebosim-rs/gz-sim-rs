// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier::{Dynamic, Prescribed},
    BodyHandle, Name, PhysicsEngine,
    contact_promotion::{self, PredictionError},
    load_world_with_sdf_step, model_fidelity, queue_link_world_velocity, queue_link_world_wrench,
    remove_model, reset_loaded_world, set_model_fidelity,
};
use sdformat::NoResources;
const SDF: &str = include_str!("../../../examples/multifidelity_predicted_contact.sdf");

fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "prediction.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    let qualified = format!("default::{name}");
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == qualified))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn body(sim: &Simulation, id: u64) -> BodyId {
    sim.world()
        .get::<BodyHandle>(entity_by_id(sim.world(), id).unwrap())
        .unwrap()
        .id()
}
fn state(sim: &Simulation, id: u64) -> BodyState {
    sim.world()
        .resource::<PhysicsBackend>()
        .state(body(sim, id))
        .unwrap()
}
fn enable(sim: &mut Simulation) -> (u64, u64) {
    let arm = id(sim, "arm");
    let ram = id(sim, "ram::head");
    set_model_fidelity(sim, arm, Prescribed).unwrap();
    contact_promotion::configure(sim, arm, vec![ram]).unwrap();
    (arm, ram)
}

#[test]
fn advancing_contact_matches_always_dynamic_links_and_consumes_held_wrench_once() {
    let mut guarded = load(SDF);
    let mut reference = load(SDF);
    let (arm, ram) = enable(&mut guarded);
    let tip = id(&guarded, "arm::tip");
    let base = id(&guarded, "arm::base");
    let handles = [body(&guarded, tip), body(&guarded, base)];
    let force = [0., 0., 12.];
    queue_link_world_wrench(&mut guarded, tip, force, [0.; 3]).unwrap();
    let mut first = None;
    let mut max_difference = 0.0_f64;
    for step in 1..=1200 {
        guarded.step_exact(1).unwrap();
        if first.is_none() && model_fidelity(&guarded, arm).unwrap() == Dynamic {
            first = Some(step);
            assert!(state(&reference, ram).position[0] < -0.30);
            queue_link_world_wrench(&mut reference, tip, force, [0.; 3]).unwrap();
        }
        if first == Some(step) || first.is_some_and(|event| step == event + 1) {
            // The two fixture links each weigh 1 kg, and the ram still has positive
            // geometric clearance on these ticks. Force integrated exactly once.
            let momentum = state(&guarded, base).velocity[2] + state(&guarded, tip).velocity[2];
            assert!((momentum - 0.012).abs() < 1e-12, "z momentum {momentum}");
        }
        reference.step_exact(1).unwrap();
        for link in [base, tip] {
            let a = state(&guarded, link);
            let b = state(&reference, link);
            for (x, y) in a
                .position
                .into_iter()
                .chain(a.orientation)
                .chain(a.velocity)
                .chain(a.angular_velocity)
                .zip(
                    b.position
                        .into_iter()
                        .chain(b.orientation)
                        .chain(b.velocity)
                        .chain(b.angular_velocity),
                )
            {
                max_difference = max_difference.max((x - y).abs());
            }
        }
    }
    eprintln!(
        "promotion tick {first:?}, maximum pose/twist difference {max_difference:.3e}, final tip x {}",
        state(&guarded, tip).position[0]
    );
    assert!(matches!(first, Some(535..=551)));
    assert!(max_difference < 1e-10);
    assert!(state(&guarded, tip).position[0] > 0.8);
    assert_eq!(handles, [body(&guarded, tip), body(&guarded, base)]);
}

#[test]
fn swept_crossing_observes_queued_relative_velocity_pause_reset_and_removal() {
    let mut sim = load(&SDF.replace(">0.001<", ">0.04<"));
    let original = sim.scheduled_system_counts();
    let (arm, ram) = enable(&mut sim);
    let body_id = body(&sim, ram);
    // Cross completely in one tick; both endpoints are clear.
    queue_link_world_velocity(&mut sim, ram, [70., 0., 0.], [0.; 3]).unwrap();
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(body(&sim, ram), body_id);
    set_model_fidelity(&mut sim, arm, Prescribed).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    queue_link_world_velocity(&mut sim, ram, [70., 0., 0.], [0.; 3]).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
    let ram_model = id(&sim, "ram");
    remove_model(&mut sim, ram_model).unwrap();
    assert_eq!(sim.scheduled_system_counts(), original);
}

#[test]
fn corner_near_miss_and_disabled_policy_preserve_prescribed_tier() {
    // The two moving/static AABBs overlap along both transverse axes:
    // 0.13+0.14 > 0.255, 0.16+0.14 > 0.285, yet the corner is
    // sqrt(0.125^2+0.125^2) m from the sphere center at closest approach.
    let mut sim = load(&SDF.replace("-1.4 0.65 0 ", "-1.4 0.905 0.285 "));
    let (arm, _) = enable(&mut sim);
    sim.step_exact(900).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    let mut sim = load(SDF);
    let (arm, _) = enable(&mut sim);
    contact_promotion::clear(&mut sim, arm).unwrap();
    sim.step_exact(900).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
}

#[test]
fn rotating_extended_obstacle_uses_angular_extent_and_invalid_replacement_preserves_policy() {
    let xml = SDF
        .replace("-1.4 0.65 0 ", "0.85 0.37 0 ")
        .replace("0.32 0.26 0.32", "1.7 0.06 0.06")
        .replace(">2 0 0<", ">0 0 0<");
    let mut sim = load(&xml);
    let (arm, ram) = enable(&mut sim);
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    let base = id(&sim, "arm::base");
    assert!(contact_promotion::configure(&mut sim, arm, vec![base]).is_err());
    assert!(matches!(
        contact_promotion::configure(&mut sim, arm, vec![ram, ram]),
        Err(PredictionError::InvalidObstacles)
    ));
    queue_link_world_velocity(&mut sim, ram, [0.; 3], [0., 0., -400.]).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
}

#[test]
fn relative_sweep_of_moving_assembly_and_obstacle_requires_closing_velocity() {
    // Both links travel axially at 2 m/s. The obstacle starts just outside the
    // native margin, and moves with them until a queued stop changes the sweep.
    let mut sim = load(
        &SDF.replace("-1.4 0.65 0 ", "0 0.941 0 ")
            .replace(">2 0 0<", ">0 2 0<"),
    );
    let base = id(&sim, "arm::base");
    let tip = id(&sim, "arm::tip");
    for link in [base, tip] {
        queue_link_world_wrench(&mut sim, link, [0., 2000., 0.], [0.; 3]).unwrap();
    }
    sim.step_exact(1).unwrap();
    let (arm, ram) = enable(&mut sim);
    assert!((state(&sim, tip).velocity[1] - 2.).abs() < 1e-12);
    sim.step_exact(50).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    queue_link_world_velocity(&mut sim, ram, [0.; 3], [0.; 3]).unwrap();
    sim.step_exact(3).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
}

#[test]
fn fixed_obstacle_moving_arm_and_source_removal_retire_demand() {
    let start = SDF
        .find("<plugin name=\"gz::sim::systems::PrescribedMotion\"")
        .unwrap();
    let end = start + SDF[start..].find("</plugin>").unwrap() + "</plugin>".len();
    let xml = format!("{}<static>true</static>{}", &SDF[..start], &SDF[end..])
        .replace("-1.4 0.65 0 ", "0 1.06 0 ");
    let mut sim = load(&xml);
    let count = sim.scheduled_system_counts();
    let base = id(&sim, "arm::base");
    let tip = id(&sim, "arm::tip");
    for link in [base, tip] {
        queue_link_world_wrench(&mut sim, link, [0., 2000., 0.], [0.; 3]).unwrap();
    }
    sim.step_exact(1).unwrap();
    let (arm, _) = enable(&mut sim);
    sim.step_exact(40).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    sim.step_exact(40).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
    remove_model(&mut sim, arm).unwrap();
    // Removing the tree additionally retires its demand-driven joint sampler.
    assert!(sim.scheduled_system_counts().0 <= count.0);
}

#[test]
fn rigid_spin_keeps_its_model() {
    let mut sim = load(SDF);
    let arm = id(&sim, "arm");
    let ram = id(&sim, "ram::head");
    contact_promotion::configure(&mut sim, arm, vec![ram]).unwrap();
    for link in [id(&sim, "arm::base"), id(&sim, "arm::tip")] {
        queue_link_world_wrench(&mut sim, link, [0.; 3], [0., 1., 0.]).unwrap();
    }
    sim.step_exact(1).unwrap();
    let tip = id(&sim, "arm::tip");
    let initial = state(&sim, tip);
    set_model_fidelity(&mut sim, arm, Prescribed).unwrap();
    assert_eq!(initial, state(&sim, tip));
    sim.step_exact(5).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
}

fn free_ram_scene() -> String {
    let start = SDF
        .find("<plugin name=\"gz::sim::systems::PrescribedMotion\"")
        .unwrap();
    let end = start + SDF[start..].find("</plugin>").unwrap() + "</plugin>".len();
    format!("{}{}", &SDF[..start], &SDF[end..])
}

#[test]
fn a_force_on_a_stationary_free_payload_promotes_before_advancing_and_reset_clears_it() {
    let xml = free_ram_scene()
        .replace("-1.4 0.65 0 ", "-0.40 0.65 0 ")
        .replace(">0.001<", ">0.04<");
    let mut sim = load(&xml);
    let (arm, ram) = enable(&mut sim);
    let initial = state(&sim, ram);
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);

    sim.set_paused(true);
    queue_link_world_wrench(&mut sim, ram, [300., 0., 0.], [0.; 3]).unwrap();
    sim.tick().unwrap();
    assert_eq!(state(&sim, ram), initial);
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
    assert!(state(&sim, ram).position[0] > initial.position[0]);

    reset_loaded_world(&mut sim, |_| ()).unwrap();
    set_model_fidelity(&mut sim, arm, Prescribed).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(state(&sim, ram), initial);
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
}

#[test]
fn queued_velocity_on_a_free_payload_is_used_for_the_whole_sweep() {
    let xml = free_ram_scene().replace(">0.001<", ">0.04<");
    for (height, expected) in [("0.65", Dynamic), ("1.15", Prescribed)] {
        let mut sim = load(&xml.replace("-1.4 0.65 0 ", &format!("-1.4 {height} 0 ")));
        let (arm, ram) = enable(&mut sim);
        queue_link_world_velocity(&mut sim, ram, [70., 0., 0.], [0.; 3]).unwrap();
        sim.step_exact(1).unwrap();
        // At both endpoints of the free flight the swept box clears the tip.
        assert_eq!(model_fidelity(&sim, arm).unwrap(), expected);
    }
}

#[test]
fn centered_spinning_sphere_near_miss_has_no_angular_padding() {
    let xml = SDF
        .replace("-1.4 0.65 0 ", "0.35 0.65 0 ")
        .replace(">2 0 0<", ">0 0 0<")
        .replace(
            "<box><size>0.32 0.26 0.32</size></box>",
            "<sphere><radius>0.16</radius></sphere>",
        );
    let mut sim = load(&xml);
    let (arm, ram) = enable(&mut sim);
    queue_link_world_velocity(&mut sim, ram, [0.; 3], [0., 0., 400.]).unwrap();
    sim.step_exact(10).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
}

const AUTHORED: &str = include_str!("../../../examples/multifidelity_authored_rotor.sdf");
fn without_policy(s: &str) -> String {
    let start = s
        .find("<plugin name=\"gz::sim::systems::ContactPromotion\"")
        .unwrap();
    let end = start + s[start..].find("</plugin>").unwrap() + 9;
    let mut out = s.to_owned();
    out.replace_range(start..end, "");
    out
}

#[test]
fn authored_spinning_model_promotes_and_loaded_reset_restores_policy_and_twist() {
    let mut sim = load(AUTHORED);
    let arm = id(&sim, "rotor");
    let stop = id(&sim, "stop");
    let links = [id(&sim, "rotor::base"), id(&sim, "rotor::tip")];
    let read = |s: &Simulation| links.map(|l| state(s, l));
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    let first = read(&sim);
    assert_eq!(first[0].angular_velocity, [0., 0., 7.5]);
    for (s, linear) in first
        .iter()
        .zip([[2.4975, -0.02, 0.], [-2.3775, -0.02, 0.]])
    {
        for (x, y) in s.velocity.into_iter().zip(linear) {
            assert!((x - y).abs() < 1e-12);
        }
    }
    let mut control = load(&without_policy(AUTHORED));
    set_model_fidelity(&mut control, arm, Prescribed).unwrap();
    let mut timeline = Vec::new();
    sim.tick().unwrap();
    assert_eq!(read(&sim), first);
    for t in 1..=75 {
        sim.step_exact(1).unwrap();
        let tier = model_fidelity(&sim, arm).unwrap();
        if tier == Dynamic {
            set_model_fidelity(&mut control, arm, Dynamic).unwrap();
        }
        control.step_exact(1).unwrap();
        assert_eq!(read(&sim), read(&control), "manual control at {t}");
        timeline.push((tier, read(&sim)));
    }
    let first_dynamic = timeline.iter().position(|r| r.0 == Dynamic).unwrap();
    assert!(first_dynamic > 10 && first_dynamic < 40);
    remove_model(&mut sim, stop).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(read(&sim), first);
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    for (t, (tier, expected)) in timeline.into_iter().enumerate() {
        sim.step_exact(1).unwrap();
        assert_eq!(
            model_fidelity(&sim, arm).unwrap(),
            tier,
            "reset tier at {t}"
        );
        assert_eq!(read(&sim), expected, "reset pose at {t}");
    }
}

#[test]
fn authored_twist_uses_mass_weighted_rotated_inertial_centers() {
    use glam::{DQuat, DVec3};
    let xml = without_policy(AUTHORED)
        .replacen(
            "<inertial><mass>1</mass>",
            "<inertial><pose>0.04 -0.03 0.02 0 0 0</pose><mass>3</mass>",
            1,
        )
        .replace(
            "<model name=\"rotor\">",
            "<model name=\"rotor\"><pose>3 -2 1 0.5 -0.2 1.3</pose>",
        );
    let sim = load(&xml);
    let ids = [id(&sim, "rotor::base"), id(&sim, "rotor::tip")];
    let states = ids.map(|l| state(&sim, l));
    let native_com = |i: usize| {
        DVec3::from(states[i].position)
            + DQuat::from_array(states[i].orientation)
                * if i == 0 {
                    DVec3::new(0.04, -0.03, 0.02)
                } else {
                    DVec3::ZERO
                }
    };
    let center = (3. * native_com(0) + native_com(1)) / 4.;
    for (i, l) in ids.into_iter().enumerate() {
        let expected =
            DVec3::new(0.06, -0.02, 0.) + DVec3::new(0., 0., 7.5).cross(native_com(i) - center);
        assert!(DVec3::from(states[i].velocity).distance(expected) < 1e-12);
        let v = sim
            .world()
            .get::<gz_sim_scene::Velocity>(entity_by_id(sim.world(), l).unwrap())
            .unwrap();
        let base = DVec3::new(0.06, -0.02, 0.)
            + DVec3::new(0., 0., 7.5).cross(DVec3::from(states[i].position) - center);
        assert!(DVec3::from(v.linear).distance(base) < 1e-12);
    }
}
#[test]
fn authored_contact_policy_rejects_wrong_owners_names_and_unsupported_fields() {
    for xml in [
        AUTHORED.replace("default::stop::head", "default::rotor::tip"),
        AUTHORED.replace("default::stop::head", "stop::missing"),
        AUTHORED.replace(
            "</obstacle>",
            "</obstacle><obstacle>default::stop::head</obstacle>",
        ),
        AUTHORED.replace(
            "<initial_tier>prescribed</initial_tier>",
            "<initial_tier>wide</initial_tier>",
        ),
        AUTHORED.replace(
            "<initial_tier>prescribed</initial_tier>",
            "<steps>10</steps>",
        ),
        AUTHORED.replace("default::rotor</model>", "default::stop</model>"),
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml,
                "bad.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
    assert!(
        load_world_with_sdf_step(
            AUTHORED,
            "bad.sdf",
            "default",
            &NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
}

const SLIDING: &str = include_str!("../../../examples/multifidelity_sliding_payload.sdf");

#[test]
fn sustained_floor_support_delays_promotion_and_both_passing_boxes_stay_prescribed() {
    for (offset, should_hit) in [(0.71, true), (1.15, false), (1.65, false)] {
        let mut sim = load(&SLIDING.replace("-1.1 0.71 0.17", &format!("-1.1 {offset} 0.17")));
        sim.step_exact(2000).unwrap();
        let (arm, ram) = enable(&mut sim);
        let tip = body(&sim, id(&sim, "arm::tip"));
        let payload = body(&sim, ram);
        queue_link_world_velocity(&mut sim, ram, [4., 0., 0.], [0.; 3]).unwrap();
        let mut first_promotion = None;
        let mut first_impact = None;
        for tick in 1..=500 {
            sim.step_exact(1).unwrap();
            if model_fidelity(&sim, arm).unwrap() == Dynamic {
                first_promotion.get_or_insert(tick);
            }
            let backend = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
                .unwrap();
            if backend.rigid_contact_observations().iter().any(|p| {
                (p.a == tip && p.b == payload || p.b == tip && p.a == payload)
                    && p.normal_impulse > 0.
            }) {
                first_impact.get_or_insert(tick);
            }
        }
        eprintln!(
            "floor slide y={offset}: promotion {first_promotion:?}, first payload-tip impulse {first_impact:?}"
        );
        if should_hit {
            let promotion = first_promotion.expect("sliding payload should promote the arm");
            let impact = first_impact.expect("sliding payload should strike the tip");
            assert!(promotion > 75 && promotion <= impact);
        } else {
            assert_eq!(first_impact, None);
            assert_eq!(first_promotion, None);
        }
    }
}

#[test]
fn promotion_witness_survives_pause_and_is_scoped_to_the_policy_and_epoch() {
    use gz_sim_scene::contact_promotion::last_promotion;
    let mut sim = load(&free_ram_scene().replace(">0.001<", ">0.04<"));
    let (arm, ram) = enable(&mut sim);
    let mut prior = None;
    for run in 0..2 {
        sim.set_paused(true);
        queue_link_world_velocity(&mut sim, ram, [70., 0., 0.], [0.; 3]).unwrap();
        sim.tick().unwrap();
        assert_eq!(last_promotion(&sim, arm), None);
        sim.step_exact(1).unwrap();
        let observed = last_promotion(&sim, arm).unwrap();
        assert_eq!((observed.model, observed.obstacle), (arm, ram));
        assert_eq!((observed.epoch, observed.iteration), (sim.epoch(), 1));
        assert_eq!(observed.start_time, std::time::Duration::ZERO);
        assert_eq!(observed.sim_time, sim.step_duration());
        sim.tick().unwrap();
        assert_eq!(last_promotion(&sim, arm), Some(observed));
        if run == 0 {
            prior = Some(observed);
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(last_promotion(&sim, arm), None);
            set_model_fidelity(&mut sim, arm, Prescribed).unwrap();
        } else {
            assert!(prior.unwrap().epoch < observed.epoch);
            // A refused replacement must preserve the previous decision.
            assert!(contact_promotion::configure(&mut sim, arm, vec![ram, arm]).is_err());
            assert_eq!(last_promotion(&sim, arm), Some(observed));
            contact_promotion::configure(&mut sim, arm, vec![ram]).unwrap();
            assert_eq!(last_promotion(&sim, arm), None);
        }
    }
}

#[test]
fn simultaneous_targets_have_a_stable_witness_and_removed_targets_retire_independently() {
    use gz_sim_scene::contact_promotion::last_promotion;
    let source = free_ram_scene().replace(">0.001<", ">0.04<");
    let begin = source.find(r#"<model name="ram">"#).unwrap();
    let end = begin + source[begin..].find("</model>").unwrap() + "</model>".len();
    // The rams traverse different arm links on the same step.
    let second = source[begin..end]
        .replace(r#"name="ram""#, r#"name="other""#)
        .replace("-1.4 0.65", "-1.4 0");
    let source = source.replace("</world>", &format!("{second}</world>"));
    for remove in [false, true] {
        let mut sim = load(&source);
        let (arm, ram) = enable(&mut sim);
        let other = id(&sim, "other::head");
        contact_promotion::configure(&mut sim, arm, vec![other, ram]).unwrap();
        if remove {
            let model = id(&sim, "ram");
            remove_model(&mut sim, model).unwrap();
        } else {
            queue_link_world_velocity(&mut sim, ram, [70., 0., 0.], [0.; 3]).unwrap();
        }
        queue_link_world_velocity(&mut sim, other, [70., 0., 0.], [0.; 3]).unwrap();
        sim.step_exact(1).unwrap();
        let event = last_promotion(&sim, arm).unwrap();
        assert_eq!(event.obstacle, if remove { other } else { ram.min(other) });
        if !remove {
            let winner = if event.obstacle == ram {
                "ram"
            } else {
                "other"
            };
            let model = id(&sim, winner);
            remove_model(&mut sim, model).unwrap();
            assert_eq!(last_promotion(&sim, arm), None);
        }
        contact_promotion::clear(&mut sim, arm).unwrap();
        assert_eq!(last_promotion(&sim, arm), None);
        assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
    }
}

#[test]
fn a_contact_policy_only_promotes_its_threatened_model() {
    let source = free_ram_scene().replace(">0.001<", ">0.04<");
    let begin = source.find(r#"<model name="arm">"#).unwrap();
    let end = begin + source[begin..].find("</model>").unwrap() + "</model>".len();
    let second = source[begin..end].replace(
        r#"<model name="arm">"#,
        r#"<model name="neighbor"><pose>0 80 0 0 0 0</pose>"#,
    );
    let mut sim = load(&source.replace("</world>", &format!("{second}</world>")));
    let (arm, ram) = enable(&mut sim);
    let neighbor = id(&sim, "neighbor");
    set_model_fidelity(&mut sim, neighbor, Prescribed).unwrap();
    contact_promotion::configure(&mut sim, neighbor, vec![ram]).unwrap();
    queue_link_world_velocity(&mut sim, ram, [70., 0., 0.], [0.; 3]).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
    assert_eq!(model_fidelity(&sim, neighbor).unwrap(), Prescribed);
    assert_eq!(contact_promotion::last_promotion(&sim, neighbor), None);
}

#[test]
fn a_paused_velocity_edit_redirects_a_supported_miss_into_a_real_contact() {
    let source = SLIDING.replace("-1.1 0.71 0.17", "-1.1 1.15 0.17");
    let mut guarded = load(&source);
    let mut reference = load(&source);
    for s in [&mut guarded, &mut reference] {
        s.step_exact(2000).unwrap();
    }
    let (arm, ram) = enable(&mut guarded);
    for sim in [&mut guarded, &mut reference] {
        queue_link_world_velocity(sim, ram, [4., 0., 0.], [0.; 3]).unwrap();
        sim.step_exact(110).unwrap();
        sim.set_paused(true);
        let before = state(sim, ram);
        queue_link_world_velocity(sim, ram, [4., -3.9, 0.], [0.; 3]).unwrap();
        sim.tick().unwrap();
        assert_eq!(state(sim, ram).position, before.position);
        assert_eq!(state(sim, ram).velocity, before.velocity);
    }
    assert_eq!(model_fidelity(&guarded, arm).unwrap(), Prescribed);
    let tip = id(&guarded, "arm::tip");
    let first_impulse =
        |sim: &mut Simulation| -> Option<usize> {
            let a = body(sim, tip);
            let b = body(sim, ram);
            for tick in 1..=300 {
                sim.step_exact(1).unwrap();
                let backend = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
                    .unwrap();
                if backend.rigid_contact_observations().iter().any(|p| {
                    ((p.a, p.b) == (a, b) || (p.b, p.a) == (a, b)) && p.normal_impulse > 0.
                }) {
                    return Some(tick);
                }
            }
            None
        };
    let contact = first_impulse(&mut reference).expect("redirect should hit the tip");
    assert_eq!(first_impulse(&mut guarded), Some(contact));
    let event = contact_promotion::last_promotion(&guarded, arm).unwrap();
    let launch_tick = event.iteration - 2000;
    assert!(launch_tick > 110 && launch_tick <= 110 + contact as u64);
    eprintln!(
        "redirect at tick 110: promotion {launch_tick}, first reference and automatic impulse {}",
        110 + contact
    );
}

#[test]
fn removing_support_before_advancing_retires_its_contact_history() {
    // Separate disjoint floor strips so removing the payload support keeps the arm grounded.
    let base = SLIDING
        .replace("-1.1 0.71 0.17", "-1.1 1.15 0.17")
        .replace("0 0 -0.1", "0 1.15 -0.1")
        .replace("10 10 0.2", "10 0.4 0.2");
    let arm_floor = r#"<model name="arm_floor"><static>true</static><pose>0 -0.12 -0.1 0 0 0</pose>
        <link name="body"><collision name="c"><geometry><box><size>10 2 0.2</size></box></geometry></collision></link></model>"#;
    let source = base.replace("</world>", &format!("{arm_floor}</world>"));
    let mut sim = load(&source);
    sim.step_exact(2000).unwrap();
    let (arm, ram) = enable(&mut sim);
    queue_link_world_velocity(&mut sim, ram, [4., 0., 0.], [0.; 3]).unwrap();
    sim.step_exact(50).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    let floor = id(&sim, "floor");
    let height = state(&sim, ram).position[2];
    remove_model(&mut sim, floor).unwrap();
    sim.step_exact(200).unwrap();
    assert!(state(&sim, ram).position[2] < height - 0.1);
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
    assert_eq!(contact_promotion::last_promotion(&sim, arm), None);
}

#[test]
fn mature_zone_dwell_vetoes_a_queued_obstacle_crossing_and_admits_a_near_miss() {
    use gz_sim_scene::fidelity_zone::{FidelityZone, configure_with_eligibility_dwell};
    for (height, expected) in [("0.65", Dynamic), ("1.15", Prescribed)] {
        let xml = SDF
            .replace(">0.001<", ">0.04<")
            .replace(">2 0 0<", ">0 0 0<")
            .replace("-1.4 0.65 0 ", &format!("-1.4 {height} 0 "));
        let mut sim = load(&xml);
        let arm = id(&sim, "arm");
        let ram = id(&sim, "ram::head");
        let dt = sim.step_duration();
        contact_promotion::configure(&mut sim, arm, vec![ram]).unwrap();
        let zone = FidelityZone {
            center: [10., 0., 0.],
            promote_radius: 1.,
            demote_radius: 2.,
        };
        configure_with_eligibility_dwell(&mut sim, arm, vec![zone], std::time::Duration::ZERO, dt)
            .unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
        queue_link_world_velocity(&mut sim, ram, [70., 0., 0.], [0.; 3]).unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(model_fidelity(&sim, arm).unwrap(), expected);
        assert!(
            contact_promotion::last_promotion(&sim, arm).is_none(),
            "demoted and immediately promoted on the same solve"
        );
        // A veto cancels the interval. The first clear step starts a new one.
        if expected == Dynamic {
            sim.step_exact(1).unwrap();
            assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
            sim.step_exact(1).unwrap();
            assert_eq!(model_fidelity(&sim, arm).unwrap(), Prescribed);
        }
    }
}

#[test]
fn prospective_rotor_sweep_matches_the_installed_transport_and_rejects_tumbling() {
    use gz_physics::PhysicsError;
    use gz_physics_multifidelity::MultiFidelityBackend;
    use gz_sim_scene::fidelity_zone::{FidelityZone, configure_with_eligibility_dwell};
    let original = without_policy(AUTHORED);
    for (x, expected) in [("-0.115", true), ("0.115", false)] {
        let xml = original.replace("-0.269 0.175 0 ", &format!("{x} 0.325 0 "));
        let mut sim = load(&xml);
        let arm = id(&sim, "rotor");
        let root = body(&sim, id(&sim, "rotor::base"));
        let stop = body(&sim, id(&sim, "stop::head"));
        let dt = sim.step_duration();
        let query = |sim: &Simulation, prospective| {
            let native = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap();
            if prospective {
                native.demotion_contact_prediction(root, dt).unwrap()(stop).unwrap()
            } else {
                native
                    .predicts_articulation_contact(root, stop, dt)
                    .unwrap()
            }
        };
        // Both positions start 25 mm outside sphere contact. Only the turning
        // tip enters the 20 mm prediction band on this 4 ms step.
        let stationary = load(&xml.replace(">0 0 7.5<", ">0 0 0<"));
        assert!(!query(&stationary, true));
        assert_eq!(query(&sim, true), expected);
        assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
        set_model_fidelity(&mut sim, arm, Prescribed).unwrap();
        assert_eq!(query(&sim, false), expected);
    }
    let mut sim = load(&original.replace(">0 0 7.5<", ">0 5 7.5<"));
    let arm = id(&sim, "rotor");
    let root = body(&sim, id(&sim, "rotor::base"));
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    assert!(matches!(
        native.demotion_contact_prediction(root, sim.step_duration()),
        Err(PhysicsError::TransitionDeferred {
            reason: "torque-free motion requires a principal-axis spin"
        })
    ));
    assert!(matches!(
        set_model_fidelity(&mut sim, arm, Prescribed),
        Err(gz_sim_scene::FidelityError::Physics(
            PhysicsError::TransitionDeferred {
                reason: "torque-free motion requires a principal-axis spin"
            }
        ))
    ));
    let stop = id(&sim, "stop::head");
    let dt = sim.step_duration();
    contact_promotion::configure(&mut sim, arm, vec![stop]).unwrap();
    let zone = FidelityZone {
        center: [10., 0., 0.],
        promote_radius: 1.,
        demote_radius: 2.,
    };
    configure_with_eligibility_dwell(&mut sim, arm, vec![zone], std::time::Duration::ZERO, dt)
        .unwrap();
    sim.step_exact(3).unwrap();
    assert_eq!(model_fidelity(&sim, arm).unwrap(), Dynamic);
    assert!(contact_promotion::last_promotion(&sim, arm).is_none());
}
