// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{JointSample, Name, PhysicsEngine, load_world_with_sdf_step, set_joint_force};

const BASE: &str = include_str!("../../../examples/worlds/local_reduced_wrist_slider.sdf");
const PLUGIN: &str = r#"<plugin name="gz::sim::systems::ApplyJointForce" filename="gz-sim-apply-joint-force-system"><joint_name>slider</joint_name></plugin>"#;

fn xml(lower: f64, upper: f64, powered: bool) -> String {
    let mut sdf = BASE.replace("-0.02405 0.12", "3.0 0.12").replace(
        "<xyz>1 0 0</xyz>",
        &format!("<xyz>1 0 0</xyz><limit><lower>{lower}</lower><upper>{upper}</upper></limit>"),
    );
    if powered {
        sdf = sdf.replace(
            "<link name=\"wrist\">",
            &format!("{PLUGIN}<link name=\"wrist\">"),
        );
    }
    sdf
}

fn load(sdf: &str) -> Simulation {
    load_world_with_sdf_step(
        sdf,
        "wrist.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn slider(sim: &Simulation) -> (u64, JointSample) {
    #[allow(deprecated)]
    let joint = sim
        .world()
        .iter_entities()
        .find(|node| {
            node.get::<Name>()
                .is_some_and(|n| n.0 == "default::wrist_rig::slider")
        })
        .unwrap();
    (
        joint.get::<EntityId>().unwrap().get(),
        *joint.get::<JointSample>().unwrap(),
    )
}

#[test]
fn the_loaded_native_stop_holds_and_releases_in_both_directions() {
    for side in [-1., 1.] {
        let bounds = if side > 0. { [-0.01, 0.] } else { [0., 0.01] };
        let mut sim = load(&xml(bounds[0], bounds[1], true));
        let (id, initial) = slider(&sim);
        let run = |sim: &mut Simulation| {
            set_joint_force(sim, id, 2. * side).unwrap();
            sim.step_exact(100).unwrap();
            let loaded = slider(sim).1.state;
            println!("side {side} loaded {loaded:?}");
            assert!(loaded.position.abs() < 1e-8 && loaded.velocity.abs() < 1e-4);
            set_joint_force(sim, id, -2. * side).unwrap();
            sim.step_exact(100).unwrap();
            let away = slider(sim).1.state;
            println!("side {side} released {away:?}");
            assert!(side * away.position < -1e-5 && side * away.velocity < -0.02);
            [loaded, away]
        };
        sim.step_exact(10).unwrap();
        let path = run(&mut sim);
        sim.reset().unwrap();
        assert_eq!(slider(&sim).1.state, initial.state);
        sim.step_exact(10).unwrap();
        assert_eq!(slider(&sim).1.state, initial.state);
        assert_eq!(run(&mut sim), path);
    }
}

#[test]
fn unpowered_slider_is_passive_and_joint_force_has_one_owner() {
    let sdf = xml(-0.001, 0.001, true);
    let mut sim = load(&sdf);
    let (id, initial) = slider(&sim);
    sim.step_exact(200).unwrap();
    assert_eq!(slider(&sim).1.state, initial.state);
    set_joint_force(&mut sim, id, 2.).unwrap();
    sim.step_exact(1500).unwrap();
    let st = slider(&sim).1.state;
    println!("interior reached positive stop {st:?}");
    assert!((st.position - 0.001).abs() < 1e-7 && st.velocity.abs() < 0.001);
    for bad in [
        sdf.replace(PLUGIN, &format!("{PLUGIN}{PLUGIN}")),
        sdf.replace(PLUGIN, &PLUGIN.replace("slider", "hinge")),
        sdf.replace("<upper>0.001</upper>", "<upper>-0.002</upper>"),
    ] {
        assert!(
            load_world_with_sdf_step(
                &bad,
                "bad.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn a_seated_drive_shares_the_first_sphere_impulse_with_its_native_stop() {
    let mass = 0.08;
    let inertia = 0.0004266666666666667 + 0.00003208666666666667 + mass * 0.1 * 0.1;
    for restitution in [0., 0.6, 1.] {
        for ns in [1_000u32, 100] {
            for sign in [-1., 1.] {
                let ends = if sign > 0. { [-0.01, 0.] } else { [0., 0.01] };
                let sdf = xml(ends[0], ends[1], true)
                    .replace("3.0 0.12", &format!("{} 0.12", -sign * 0.02405))
                    .replace(">0.75 0 0<", &format!(">{} 0 0<", sign * 0.75))
                    .replace(">0.00001<", &format!(">{:.9}<", f64::from(ns) * 1e-9))
                    .replace(">0.6<", &format!(">{restitution}<"));
                let mut sim = load(&sdf);
                let (id, _) = slider(&sim);
                set_joint_force(&mut sim, id, sign * 2.).unwrap();
                let expected = (1. + restitution) * 0.75 / (1. / 0.2 + 0.12 * 0.12 / inertia);
                sim.step_exact(66_666 / u64::from(ns)).unwrap();
                sim.step_exact(1).unwrap();
                let p = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .unwrap();
                let measured: f64 = p
                    .rigid_contact_patches()
                    .iter()
                    .flat_map(|p| &p.points)
                    .map(|p| p.normal_impulse)
                    .sum();
                assert!(
                    (measured - expected).abs() < 1e-10,
                    "{sign} {ns} {restitution}: {measured} vs {expected}"
                );
                assert!(slider(&sim).1.state.velocity.abs() < 1e-9);
            }
        }
    }
}

fn first_patch(sdf: &str) -> (Vec<f64>, f64) {
    let mut sim = load(sdf);
    let (id, _) = slider(&sim);
    set_joint_force(&mut sim, id, 2.).unwrap();
    for _ in 0..100 {
        sim.step_exact(1).unwrap();
        let native = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let mut normal = Vec::new();
        let mut tangent = 0.0;
        for patch in native.rigid_contact_patches() {
            let n: f64 = patch.points.iter().map(|p| p.normal_impulse).sum();
            if n > 0. {
                normal.push(n);
                for p in &patch.points {
                    let t = p.tangent_impulse_on_a.unwrap_or([0.; 3]);
                    tangent += t.iter().map(|x| x * x).sum::<f64>().sqrt();
                }
            }
        }
        if !normal.is_empty() {
            return (normal, tangent);
        }
    }
    panic!("No native finger impact");
}

#[test]
fn a_second_finite_coordinate_preserves_the_regular_contact_solve() {
    let contact = xml(-0.01, 0., true)
        .replace("3.0 0.12", "-0.02405 0.12")
        .replace(">0.00001<", ">0.000001<");
    let link = contact
        .split_once("<link name=\"finger\">")
        .unwrap()
        .1
        .split_once("</link>")
        .unwrap()
        .0;
    let alternate = format!(
        "<link name=\"other\">{}</link><joint name=\"other_stop\" type=\"prismatic\"><parent>wrist</parent><child>other</child><axis><xyz>0 0 1</xyz>LIMIT</axis></joint>",
        link.replace("0 0.1 0.1 0 0 0", "0 -0.1 0.1 0 0 0")
    );
    let pair = contact.replacen("</model>", &format!("{alternate}</model>"), 1);
    let bounded = pair.replace(
        "LIMIT",
        "<limit><lower>-0.01</lower><upper>0</upper></limit>",
    );
    let free = pair.replace("LIMIT", "");
    let single = first_patch(&free).0[0];
    let two = first_patch(&bounded).0[0];
    // Infinitesimal friction selects the ordinary coupled normal/friction
    // response, with no material change to this purely normal collision.
    let regular = first_patch(
        &bounded
            .replace("<mu>0</mu>", "<mu>1e-15</mu>")
            .replace("<mu2>0</mu2>", "<mu2>1e-15</mu2>"),
    )
    .0[0];
    assert!((two - regular).abs() < 1e-12, "{two} {regular}");
    assert!(single > two + 0.001, "{single} {two}");
}

#[test]
fn simultaneous_and_frictional_impacts_keep_their_native_contact_rows() {
    let contact = xml(-0.01, 0., true).replace("3.0 0.12", "-0.02405 0.12");
    let rough = contact
        .replace("<mu>0</mu>", "<mu>0.4</mu>")
        .replace("<mu2>0</mu2>", "<mu2>0.4</mu2>")
        .replace(">0.75 0 0<", ">0.75 0 0.1<");
    let (normal, tangent) = first_patch(&rough);
    assert_eq!(normal.len(), 1);
    assert!(tangent > 0.001 && tangent <= 0.4 * normal[0]);
    let (_, parcel) = contact.split_once("<model name=\"parcel\">").unwrap();
    let (sphere, _) = parcel.split_once("</model>").unwrap();
    let both = contact
        .replace("-0.02405 0.12 0.1 ", "-0.02405 0.12 0.12 ")
        .replace(
            "</world>",
            &format!(
                "<model name=\"second\">{}</model></world>",
                sphere.replace("-0.02405 0.12 0.1 ", "-0.02405 0.12 0.08 ")
            ),
        );
    let (normal, tangent) = first_patch(&both);
    assert_eq!(normal.len(), 2);
    assert!(normal.iter().all(|n| *n > 0.01 && *n < 0.15));
    assert_eq!(tangent, 0.);
}
