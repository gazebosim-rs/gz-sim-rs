// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A powered stop in one mechanism must not change a second robot's independent strike.
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, set_joint_force};
use sdformat::NoResources;

const SDF: &str = include_str!("../../../examples/worlds/local_powered_slider_impact.sdf");

fn two_rigs(reduced: bool) -> Simulation {
    let marker = r#"<plugin name="gz::sim::systems::ReducedScalarJoints" filename="gz-sim-reduced-scalar-joints-system"/>"#;
    let unmarked = SDF.replace(marker, "");
    assert_ne!(unmarked, SDF);
    let remote = if reduced { SDF } else { &unmarked };
    let document = roxmltree::Document::parse(remote).unwrap();
    let models: String = document
        .descendants()
        .filter(|n| n.has_tag_name("model"))
        .map(|n| &remote[n.range()])
        .collect();
    let models = models
        .replace(
            "<model name=\"wrist_rig\">",
            "<model name=\"remote\"><pose>0 0 1 0 0 0</pose>",
        )
        .replace(
            "<model name=\"parcel\"><pose>-0.02405 0.12 0.1",
            "<model name=\"remote_parcel\"><pose>-0.02405 0.12 1.1",
        );
    let xml = unmarked.replace("</world>", &format!("{models}</world>"));
    let mut sim = load_world_with_sdf_step(
        &xml,
        "two-wrists.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    for name in ["default::wrist_rig::slider", "default::remote::slider"] {
        let id = named(&sim, name).get::<EntityId>().unwrap().get();
        set_joint_force(&mut sim, id, 2.0).unwrap();
    }
    sim
}

#[allow(deprecated)]
fn named<'a>(sim: &'a Simulation, name: &str) -> bevy_ecs::world::EntityRef<'a> {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
}

#[derive(PartialEq, Debug)]
struct Witness {
    states: [BodyState; 3],
    normal_impulse_ns: f64,
}

fn observe(sim: &Simulation, ids: [BodyId; 3]) -> Witness {
    let backend = sim.world().resource::<PhysicsBackend>();
    let states = ids.map(|id| backend.state(id).unwrap());
    let mut normal_impulse_ns = 0.;
    for patch in backend
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .rigid_contact_patches()
    {
        let left = ids.contains(&patch.a);
        let right = ids.contains(&patch.b);
        assert_eq!(left, right, "the two mechanisms physically touched");
        if left {
            normal_impulse_ns += patch.points.iter().map(|p| p.normal_impulse).sum::<f64>();
        }
    }
    Witness {
        states,
        normal_impulse_ns,
    }
}

#[test]
fn changing_a_remote_loaded_stop_preserves_the_other_rigs_impacts_and_trajectory() {
    let mut ordinary = two_rigs(false);
    let mut coupled = two_rigs(true);
    let bodies = |sim: &Simulation, names: [&str; 3]| {
        names.map(|n| named(sim, n).get::<BodyHandle>().unwrap().id())
    };
    let local = [
        "default::wrist_rig::wrist",
        "default::wrist_rig::finger",
        "default::parcel::body",
    ];
    let remote = [
        "default::remote::wrist",
        "default::remote::finger",
        "default::remote_parcel::body",
    ];
    let a = bodies(&ordinary, local);
    let b = bodies(&coupled, local);
    let ra = bodies(&ordinary, remote);
    let rb = bodies(&coupled, remote);
    let mut first = [0.; 3];
    for tick in 1..=300 {
        ordinary.step_exact(1).unwrap();
        coupled.step_exact(1).unwrap();
        let witness = observe(&ordinary, a);
        assert_eq!(
            witness,
            observe(&coupled, b),
            "unmarked wrist changed on tick {tick}"
        );
        let impulses = [
            witness.normal_impulse_ns,
            observe(&ordinary, ra).normal_impulse_ns,
            observe(&coupled, rb).normal_impulse_ns,
        ];
        for (value, j) in first.iter_mut().zip(impulses) {
            if *value == 0. && j > 0. {
                *value = j;
            }
        }
    }
    assert!(first.iter().all(|j| *j > 0.05));
    assert!(
        first[2] > first[1] + 0.005,
        "marked stop did not influence its own strike: {first:?}"
    );
}

/// Scale density at fixed dimensions, including every specified component of inertia.
fn change_density(xml: &str, model: &str, link: &str, factor: f64) -> String {
    let doc = roxmltree::Document::parse(xml).unwrap();
    let body = doc
        .descendants()
        .find(|n| n.has_tag_name("model") && n.attribute("name") == Some(model))
        .unwrap()
        .children()
        .find(|n| n.has_tag_name("link") && n.attribute("name") == Some(link))
        .unwrap();
    let inertia = body
        .children()
        .find(|n| n.has_tag_name("inertial"))
        .unwrap();
    let mut values: Vec<_> = inertia
        .descendants()
        .filter(|n| n.is_text())
        .filter_map(|n| {
            n.text()?
                .parse::<f64>()
                .ok()
                .map(|v| (n.range(), v * factor))
        })
        .collect();
    let mut result = xml.to_owned();
    while let Some((range, v)) = values.pop() {
        result.replace_range(range, &v.to_string());
    }
    result
}

#[test]
fn a_light_wrist_releases_and_a_heavy_wrist_braces_the_same_driven_finger() {
    for (wrist_scale, parcel_scale) in [(0.25, 1.), (0.35, 1.), (10., 1.), (10., 10.), (100., 10.)]
    {
        let xml = change_density(
            &change_density(SDF, "wrist_rig", "wrist", wrist_scale),
            "parcel",
            "body",
            parcel_scale,
        );
        let mut sim = load_world_with_sdf_step(
            &xml,
            "wrist-inertia.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        let slider = named(&sim, "default::wrist_rig::slider")
            .get::<EntityId>()
            .unwrap()
            .get();
        set_joint_force(&mut sim, slider, 2.).unwrap();
        let bodies = [
            "default::wrist_rig::wrist",
            "default::wrist_rig::finger",
            "default::parcel::body",
        ]
        .map(|n| named(&sim, n).get::<BodyHandle>().unwrap().id());
        let spin_inertia = 0.0004266666666666667 * wrist_scale + 0.00003208666666666667;
        let finger_mass = 0.08;
        let ball_mass = 0.2 * parcel_scale;
        let radius = 0.1;
        let offset = 0.02;
        // First solve the free impact. The upper stop carries compression only if
        // the free slider would move farther out. Locking it then adds m*r² to I.
        let free_impulse =
            1.6 * 0.75 / (1. / ball_mass + 1. / finger_mass + offset * offset / spin_inertia);
        let free_rate = free_impulse * (1. / finger_mass - radius * offset / spin_inertia);
        let blocked = free_rate > 0.;
        let expected = if blocked {
            1.6 * 0.75
                / (1. / ball_mass
                    + (radius + offset) * (radius + offset)
                        / (spin_inertia + finger_mass * radius * radius))
        } else {
            free_impulse
        };
        let hit = (0..100)
            .find_map(|_| {
                sim.step_exact(1).unwrap();
                let j = observe(&sim, bodies).normal_impulse_ns;
                (j > 0.).then_some(j)
            })
            .unwrap();
        assert!(
            (hit - expected).abs() < 2e-6,
            "scales {wrist_scale}/{parcel_scale}: {hit} vs {expected}"
        );
        let rate = named(&sim, "default::wrist_rig::slider")
            .get::<gz_sim_scene::JointSample>()
            .unwrap()
            .state
            .velocity;
        let predicted_rate = if blocked { 0. } else { free_rate };
        assert!(
            (rate - predicted_rate).abs() < 2e-4,
            "scales {wrist_scale}/{parcel_scale}: {rate} vs {predicted_rate}"
        );
    }
}

#[test]
#[ignore = "run on a pinned CPU to measure the first loaded Bevy step"]
fn measure_the_powered_impact_tick() {
    let unmarked = SDF.replace(r#"<plugin name="gz::sim::systems::ReducedScalarJoints" filename="gz-sim-reduced-scalar-joints-system"/>"#,"");
    let mut runs = [&unmarked[..], SDF].map(|xml| {
        let sim = load_world_with_sdf_step(
            xml,
            "timed-wrist.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        let slider = named(&sim, "default::wrist_rig::slider")
            .get::<EntityId>()
            .unwrap()
            .get();
        let bodies = [
            "default::wrist_rig::wrist",
            "default::wrist_rig::finger",
            "default::parcel::body",
        ]
        .map(|n| named(&sim, n).get::<BodyHandle>().unwrap().id());
        (sim, slider, bodies, Vec::new())
    });
    // Alternate which world goes first. Charge, approach, contact readout and reset
    // are outside the timer; the measured call advances the first loaded tick.
    for rep in 0..201 {
        for index in [rep % 2, 1 - rep % 2] {
            let (sim, slider, bodies, times) = &mut runs[index];
            set_joint_force(sim, *slider, 2.).unwrap();
            sim.step_exact(66).unwrap();
            assert_eq!(observe(sim, *bodies).normal_impulse_ns, 0.);
            let t = std::time::Instant::now();
            sim.step_exact(1).unwrap();
            times.push(t.elapsed().as_nanos());
            assert!(observe(sim, *bodies).normal_impulse_ns > 0.05);
            sim.reset().unwrap();
        }
    }
    for ((_, _, _, mut times), name) in runs.into_iter().zip(["ordinary", "coupled"]) {
        times.sort_unstable();
        println!(
            "{name} n={} p50_ns={} p95_ns={} p99_ns={}",
            times.len(),
            times[100],
            times[190],
            times[198]
        );
    }
}
