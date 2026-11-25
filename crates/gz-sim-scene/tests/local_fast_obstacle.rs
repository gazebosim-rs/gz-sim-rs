// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{BodyHandle, Name, load_local_replay_world};
use sdformat::NoResources;

const COURSE: &str = include_str!("../../../examples/worlds/local_fast_obstacle.sdf");

#[allow(deprecated)]
fn id(sim: &Simulation, name: &str) -> BodyId {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<BodyHandle>()
        .unwrap()
        .id()
}

fn guards_first() -> String {
    let tips = COURSE.find("<model name=\"hit\">").unwrap();
    let guards = COURSE.find("<model name=\"hit_guard\">").unwrap();
    let end = COURSE.find("</world>").unwrap();
    format!(
        "{}{}{}{}",
        &COURSE[..tips],
        &COURSE[guards..end],
        &COURSE[tips..guards],
        &COURSE[end..]
    )
}

#[test]
fn head_on_edge_and_clear_keep_their_analytic_momentum_in_either_insertion_order() {
    for source in [COURSE, &guards_first()] {
        let mut sim =
            load_local_replay_world(source, "fast-obstacle.sdf", "default", [0; 32]).unwrap();
        let tips = ["hit", "graze", "clear"].map(|n| id(&sim, &format!("default::{n}::tip")));
        let rails =
            ["hit", "graze", "clear"].map(|n| id(&sim, &format!("default::{n}_guard::rail")));
        let mut totals = [0.; 3];
        let mut first = [None; 3];
        for step in 1..=200 {
            sim.step_exact(1).unwrap();
            let backend = sim.world().resource::<PhysicsBackend>();
            let native = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
            for patch in native.rigid_contact_patches() {
                let impulse: f64 = patch.points.iter().map(|p| p.normal_impulse).sum();
                if impulse > 0. {
                    let lane = (0..3)
                        .find(|&i| {
                            let (tip, rail) = (tips[i], rails[i]);
                            (patch.a, patch.b) == (tip.min(rail), tip.max(rail))
                        })
                        .expect("contact outside the intended lane");
                    totals[lane] += impulse;
                    first[lane].get_or_insert(step);
                }
            }
            for tip in tips {
                assert!(DVec3::from(backend.state(tip).unwrap().angular_velocity).length() < 1e-7);
            }
        }
        let normal = DVec3::new(-(0.06_f64.powi(2) - 0.055_f64.powi(2)).sqrt(), 0.055, 0.) / 0.06;
        let incoming = DVec3::X * 100.;
        let outgoing = incoming - normal.dot(incoming) * normal;
        let backend = sim.world().resource::<PhysicsBackend>();
        for (i, predicted) in [DVec3::ZERO, outgoing, incoming].into_iter().enumerate() {
            assert!(
                DVec3::from(backend.state(tips[i]).unwrap().velocity).distance(predicted) < 1e-4
            );
        }
        assert!((totals[0] - 200.).abs() < 1e-7);
        assert!((totals[1] + 2. * normal.dot(incoming)).abs() < 1e-4);
        assert_eq!(totals[2], 0.);
        assert_eq!(first, [Some(79), Some(83), None]);
        assert!(
            DVec3::from(backend.state(tips[2]).unwrap().position)
                .distance(DVec3::new(1.15, 1.415, 0.15))
                < 1e-10
        );
    }
}

// Gravity can turn a geometrically clear initial velocity chord into a contact
// during the tick. With this tiny drop native CCD and the ordinary discrete
// solver see the same near contact; both must support the ball.
#[test]
fn gravity_driven_drop_keeps_native_speculative_contacts() {
    const CCD: &str = r#"<plugin filename="gz-sim-continuous-collision-system" name="gz::sim::systems::ContinuousCollision"><linear_slop>0.0001</linear_slop><link>default::ball::tip</link></plugin>"#;
    let mut traces = Vec::new();
    for sweep in ["", CCD] {
        let source = format!(
            r#"<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity>
          <physics name="p" type="ignored"><max_step_size>0.01</max_step_size></physics>{sweep}
          <model name="floor"><static>true</static><pose>0 0 -0.05 0 0 0</pose><link name="rail"><collision name="floor"><geometry><box><size>1 1 0.1</size></box></geometry></collision></link></model>
          <model name="ball"><pose>0 0 0.0601 0 0 0</pose><link name="tip"><inertial><mass>2</mass></inertial><collision name="ball"><geometry><sphere><radius>0.06</radius></sphere></geometry></collision></link>
          <plugin filename="gz-sim-initial-velocity-system" name="gz::sim::systems::InitialVelocity"><linear_velocity>0 0 -0.001</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin></model></world></sdf>"#
        );
        let mut sim = gz_sim_scene::load_world_with_sdf_step(
            &source,
            "drop.sdf",
            "default",
            &NoResources,
            gz_sim_scene::PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        let tip = id(&sim, "default::ball::tip");
        let mut trace = Vec::new();
        for _ in 0..8 {
            sim.step_exact(1).unwrap();
            let engine = sim.world().resource::<PhysicsBackend>();
            let patches = engine
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap()
                .rigid_contact_patches();
            let impulse = patches
                .iter()
                .flat_map(|p| &p.points)
                .map(|p| p.normal_impulse)
                .sum::<f64>();
            trace.push((engine.state(tip).unwrap().position[2], impulse));
        }
        assert!(trace.iter().all(|(z, _)| *z > 0.0598));
        assert!(trace[0].1 > 0. && trace[7].1 > 0.);
        traces.push(trace);
    }
    assert_eq!(traces[0], traces[1]);
}

// Two rail pieces share an edge at the sphere's center height. Impulses from
// either part can change the other's incoming state in this very solve.
#[test]
fn simultaneous_zero_gravity_neighbors_keep_the_native_multi_contact_solution() {
    let original = r#"<model name="graze_guard">
      <static>true</static><pose>0 0 0.15 0 0 0</pose>"#;
    let start = COURSE.find(original).unwrap();
    let end = start + COURSE[start..].find("</model>").unwrap() + "</model>".len();
    let lower = COURSE[start..end]
        .replace("0 0 0.15 0 0 0", "0 0 0.075 0 0 0")
        .replace("0.012 0.70 0.30", "0.012 0.70 0.15");
    let upper = lower
        .replace("graze_guard", "upper_guard")
        .replace("0 0 0.075 0 0 0", "0 0 0.225 0 0 0");
    let source = format!("{}{lower}{upper}{}", &COURSE[..start], &COURSE[end..]);
    // The second run forces native narrow-phase handling while changing the free
    // position by less than a representable coordinate at this height.
    let control = source.replace("<gravity>0 0 0</gravity>", "<gravity>0 0 -1e-20</gravity>");
    let mut traces = Vec::new();
    for xml in [&source, &control] {
        let mut sim = load_local_replay_world(xml, "split-rail.sdf", "default", [0; 32]).unwrap();
        let tip = id(&sim, "default::graze::tip");
        let mut trace = Vec::new();
        for _ in 0..200 {
            sim.step_exact(1).unwrap();
            let backend = sim.world().resource::<PhysicsBackend>();
            let state = backend.state(tip).unwrap();
            let native = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
            let impulse: f64 = native
                .rigid_contact_patches()
                .iter()
                .filter(|p| p.a == tip || p.b == tip)
                .flat_map(|p| &p.points)
                .map(|p| p.normal_impulse)
                .sum();
            trace.push((
                DVec3::from(state.position),
                DVec3::from(state.velocity),
                impulse,
            ));
        }
        assert!(trace.iter().any(|(_, _, j)| *j > 0.));
        traces.push(trace);
    }
    for ((p, v, j), (q, w, k)) in traces[0].iter().zip(&traces[1]) {
        assert!(p.distance(*q) < 1e-12 && v.distance(*w) < 1e-10 && (j - k).abs() < 1e-10);
    }
}
