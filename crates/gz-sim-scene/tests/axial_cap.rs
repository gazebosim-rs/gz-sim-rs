// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Publicly reported contact wrenches for a free cylinder cap impact.
#![allow(deprecated)]
use glam::DVec3;
use gz_physics::{BodyState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions, load_world_with_runtime,
};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn world() -> String {
    r#"<sdf version="1.12"><world name="default"><gravity>0 0 0</gravity>
    <physics name="local" type="ignored"><max_step_size>0.001</max_step_size></physics>
    <model name="floor"><static>true</static><pose>0 0 0 0 0 0</pose><link name="slab"><collision name="floor"><pose>0 0 -0.025 0 0 0</pose><geometry><box><size>1 1 0.05</size></box></geometry></collision></link></model>
    <model name="drop"><pose>0 0 0.105 0 0 0</pose>
    <plugin name="gz::sim::systems::InitialVelocity" filename="gz-sim-initial-velocity-system"><linear_velocity>0 0 -1.6</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>
    <plugin name="gz::sim::systems::ContactMaterial" filename="gz-sim-contact-material-system"><restitution link="body">0.65</restitution></plugin>
    <link name="body"><inertial><mass>0.25</mass><inertia><ixx>0.0010453125</ixx><iyy>0.0010453125</iyy><izz>0.000253125</izz></inertia></inertial>
    <collision name="solid"><geometry><cylinder><radius>0.045</radius><length>0.21</length></cylinder></geometry></collision></link></model></world></sdf>"#.into()
}
fn load(s: &str, engine: PhysicsEngine, enabled: bool) -> Result<Simulation> {
    Ok(load_world_with_runtime(
        s,
        "cap.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            step: None,
            engine,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions {
            rapier_axial_cap_impact: enabled,
            ..Default::default()
        },
    )?)
}
#[derive(Debug, PartialEq)]
struct Hit {
    state: BodyState,
    impulse: DVec3,
    levers: Vec<[f64; 3]>,
}
fn measure(sim: &mut Simulation) -> Result<Hit> {
    let id = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::drop::body")
        })
        .unwrap()
        .get::<BodyHandle>()
        .unwrap()
        .id();
    sim.step_exact(1)?;
    let engine = sim.world().resource::<PhysicsBackend>();
    let patches = if let Some(native) = engine.downcast_ref::<MultiFidelityBackend>() {
        native.rigid_contact_patches()
    } else {
        engine
            .downcast_ref::<gz_physics_rapier::RapierBackend>()
            .unwrap()
            .rigid_contact_patches()
    };
    let mut impulse = DVec3::ZERO;
    let mut levers = Vec::new();
    for patch in patches {
        if patch.a != id && patch.b != id {
            continue;
        }
        let sign = if patch.a == id { 1. } else { -1. };
        for px in patch.points {
            if px.normal_impulse > 0. {
                impulse += sign * DVec3::from(patch.normal_on_a) * px.normal_impulse;
                levers.push(if patch.a == id {
                    px.lever_a
                } else {
                    px.lever_b
                });
            }
        }
    }
    Ok(Hit {
        state: engine.state(id).unwrap(),
        impulse,
        levers,
    })
}
#[test]
fn native_row_carries_the_centered_newton_impulse_in_rotated_worlds() -> Result<()> {
    {
        for (xml, n) in [
            (world(), DVec3::Z),
            (
                world()
                    .replace(
                        "0 0 0 0 0 0</pose><link",
                        "0 0 0 0 1.5707963267948966 0</pose><link",
                    )
                    .replace("0 0 0.105 0 0 0", "0.105 0.12 0 0 1.5707963267948966 0.0")
                    .replace(">0 0 -1.6<", ">-1.6 0 0<"),
                DVec3::X,
            ),
        ] {
            let mut sim = load(&xml, PhysicsEngine::MultiFidelity, true)?;
            let h = measure(&mut sim)?;
            assert_eq!(h.levers.len(), 1, "{h:?}");
            let center = DVec3::from(h.levers[0]);
            assert!(center.cross(n).length() < 1e-12);
            assert!((center.dot(n) + 0.105).abs() < 1e-10);
            assert!(
                (h.impulse - 0.25 * 1.65 * 1.6 * n).length() < 1e-10,
                "{h:?}"
            );
            assert!((DVec3::from(h.state.velocity) - 0.65 * 1.6 * n).length() < 1e-10);
            assert!(DVec3::from(h.state.angular_velocity).length() < 1e-10);
            sim.reset()?;
            assert_eq!(measure(&mut sim)?, h);
        }
    }
    Ok(())
}
#[test]
fn tilted_spinning_eccentric_grazing_and_unenrolled_shapes_keep_native_response() -> Result<()> {
    let base = world();
    for (label, xml) in [
        ("tilted", base.replace("0.105 0 0 0", "0.105 0.005 0 0")),
        (
            "spin",
            base.replace("<angular_velocity>0 0 0", "<angular_velocity>0.2 0 0"),
        ),
        (
            "oblique",
            base.replace("<linear_velocity>0 0", "<linear_velocity>0.05 0"),
        ),
        (
            "eccentric",
            base.replace(
                "<inertial><mass>",
                "<inertial><pose>0.004 0 0 0 0 0</pose><mass>",
            ),
        ),
        ("edge", base.replace("0 0 0.105", "0.480 0 0.105")),
        ("inelastic", base.replace(">0.65<", ">0<")),
        (
            "capsule",
            base.replace(
                "<cylinder><radius>0.045</radius><length>0.21</length></cylinder>",
                "<capsule><radius>0.045</radius><length>0.12</length></capsule>",
            ),
        ),
    ] {
        let old = measure(&mut load(&xml, PhysicsEngine::MultiFidelity, false)?)?;
        let new = measure(&mut load(&xml, PhysicsEngine::MultiFidelity, true)?)?;
        assert_eq!(new, old, "{label}");
    }
    assert!(load(&base, PhysicsEngine::Tpe, true).is_err());
    Ok(())
}

#[test]
fn incoming_fast_third_body_constraint_and_large_origin_preserve_regular_manifolds() -> Result<()> {
    let sphere = r#"<model name="fast"><pose>0.14 0 0.105 0 0 0</pose>
    <plugin name="gz::sim::systems::InitialVelocity" filename="gz-sim-initial-velocity-system"><linear_velocity>-100 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>
    <link name="body"><inertial><mass>0.1</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial><collision name="sphere"><geometry><sphere><radius>0.01</radius></sphere></geometry></collision></link></model>"#;
    let base = world();
    let fast = base.replace("</world>", &format!("{sphere}</world>"));
    let distant = base
        .replace("0 0 0 0 0 0</pose><link", "10000000 0 0 0 0 0</pose><link")
        .replace("0 0 0.105", "10000000 0 0.105");
    let guided=base.lines().filter(|l|!l.contains("systems::InitialVelocity")).collect::<Vec<_>>().join("\n").replace("<gravity>0 0 0","<gravity>0 0 -9.81").replace("0.105 0 0 0","0.115 0 0 0").replace("</link></model></world>",r#"</link><joint name="guide" type="prismatic"><parent>world</parent><child>body</child><axis><xyz>0 0 1</xyz></axis></joint></model></world>"#);
    for (name, xml) in [("third", fast), ("large", distant), ("guided", guided)] {
        let mut old = load(&xml, PhysicsEngine::MultiFidelity, false)?;
        let mut new = load(&xml, PhysicsEngine::MultiFidelity, true)?;
        let mut saw = false;
        for _ in 0..70 {
            let a = measure(&mut old)?;
            let b = measure(&mut new)?;
            assert_eq!(a, b, "{name}");
            if a.impulse.length() > 0. {
                assert!(a.levers.len() > 1, "{name}");
                saw = true;
                break;
            }
        }
        assert!(saw, "{name}");
    }
    Ok(())
}
#[test]
fn resetting_with_a_live_clone_rebuilds_eligibility_and_repeats_the_native_first_hit() -> Result<()>
{
    use gz_sim_scene::{WorldPose, prepare_model_template, spawn_model_template};
    let source = include_str!("../../../examples/worlds/local_curved_delivery.sdf");
    let sdf = sdformat::load_str(source, "clone.sdf", &sdformat::NoResources)?;
    let mut sim = load(source, PhysicsEngine::MultiFidelity, true)?;
    let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[1])?;
    let spawn = |s: &mut Simulation| {
        spawn_model_template(
            s,
            &template,
            WorldPose {
                translation: [0., 0., 0.25],
                rotation: [0., 0., 0., 1.],
            },
        )
    };
    sim.step_exact(1)?;
    let first = spawn(&mut sim)?;
    sim.step_exact(180)?;
    let get = |s: &Simulation, id| gz_sim_scene::model_state(s, id);
    let hit = get(&sim, first)?;
    assert!(hit.linear_velocity[2] > 0.8 && DVec3::from(hit.angular_velocity).length() < 1e-9);
    gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
    sim.step_exact(1)?;
    assert!(get(&sim, first).is_err());
    let again = spawn(&mut sim)?;
    sim.step_exact(180)?;
    let repeat = get(&sim, again)?;
    assert_eq!(repeat, hit);
    Ok(())
}

#[test]
fn energetic_returns_remain_axial_and_weak_reloading_contacts_stay_ordinary() -> Result<()> {
    let drop = |height: f64, restitution: f64| {
        world()
            .lines()
            .filter(|l| !l.contains("systems::InitialVelocity"))
            .collect::<Vec<_>>()
            .join("\n")
            .replace("0 0 0.105", &format!("0 0 {}", 0.105 + height))
            .replace("<gravity>0 0 0", "<gravity>0 0 -9.81")
            .replace(">0.65<", &format!(">{restitution}<"))
    };
    for (height, restitution) in [(0.145, 0.), (0.00001, 0.01), (0.010, 0.05)] {
        let xml = drop(height, restitution);
        let mut regular = load(&xml, PhysicsEngine::MultiFidelity, false)?;
        let mut enabled = load(&xml, PhysicsEngine::MultiFidelity, true)?;
        for _ in 0..500 {
            assert_eq!(
                measure(&mut enabled)?,
                measure(&mut regular)?,
                "height={height}, restitution={restitution}"
            );
        }
    }
    let mut sim = load(&drop(0.145, 0.65), PhysicsEngine::MultiFidelity, true)?;
    let mut returns = Vec::new();
    let mut loaded = false;
    let mut rest = 0.;
    let mut max_spin: f64 = 0.;
    for tick in 1..=3000 {
        let h = measure(&mut sim)?;
        let now = h.impulse.z > 0.;
        max_spin = max_spin.max(DVec3::from(h.state.angular_velocity).length());
        if now && !loaded {
            returns.push((
                tick,
                h.impulse.z,
                h.levers.len(),
                DVec3::from(h.state.angular_velocity).length(),
            ));
        }
        if tick > 2900 {
            rest += h.impulse.z;
        }
        loaded = now;
    }
    println!(
        "native axial returns {returns:?}, max spin {max_spin}, terminal mean force {} N",
        rest / 0.1
    );
    assert!(returns.len() >= 3);
    assert!(max_spin < 1e-9);
    for r in &returns[..3] {
        assert_eq!(r.2, 1);
        assert!(r.3 < 1e-9);
    }
    assert!((rest / 0.1 - 0.25 * 9.81).abs() < 0.002);
    Ok(())
}
