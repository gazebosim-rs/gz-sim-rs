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
