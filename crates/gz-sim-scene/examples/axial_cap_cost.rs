// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Paired ordinary local SDF step costs in flight and on the first simultaneous impact.
#![allow(deprecated)]
use gz_physics::PhysicsBackend;
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions};
use serde_json::{Value, json};
use std::error::Error;
use std::fmt::Write;
use std::time::Instant;

fn fixture(n: usize) -> String {
    let mut s = String::from(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity><physics name="local" type="ignored"><max_step_size>0.001</max_step_size></physics>"#,
    );
    for i in 0..n {
        let x = 1.5 * (i % 32) as f64;
        let y = 1.5 * (i / 32) as f64;
        write!(s,r#"<model name="floor{i}"><static>true</static><pose>{x} {y} 0 0 0 0</pose><link name="slab"><collision name="floor"><pose>0 0 -0.025 0 0 0</pose><geometry><box><size>1.1 0.9 0.05</size></box></geometry></collision></link></model>"#).unwrap();
        write!(s,r#"<model name="p{i}"><pose>{x} {y} 0.25 0 0 0</pose><plugin name="gz::sim::systems::ContactMaterial" filename="gz-sim-contact-material-system"><restitution link="c">0.65</restitution></plugin><link name="c"><inertial><mass>0.25</mass><inertia><ixx>0.0010453125</ixx><iyy>0.0010453125</iyy><izz>0.000253125</izz></inertia></inertial><collision name="cap"><geometry><cylinder><radius>0.045</radius><length>0.21</length></cylinder></geometry></collision></link></model>"#).unwrap();
        for j in 0..2 {
            let bx = x + 75.;
            let by = y + 10. * f64::from(j);
            write!(s,r#"<model name="box{i}_{j}"><static>true</static><pose>{bx} {by} 0 0 0 0</pose><link name="b"><collision name="box"><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link></model>"#).unwrap();
        }
    }
    s.push_str("</world></sdf>");
    s
}
fn run(xml: &str, n: usize, enabled: bool) -> Result<Value, Box<dyn Error>> {
    let mut sim = gz_sim_scene::load_world_with_runtime(
        xml,
        "scale.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            step: None,
            engine: PhysicsEngine::MultiFidelity,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions {
            rapier_axial_cap_impact: enabled,
            ..Default::default()
        },
    )?;
    let cylinders: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0.starts_with("default::p") && n.0.ends_with("::c"))
        })
        .map(|e| e.get::<BodyHandle>().unwrap().id())
        .collect();
    assert_eq!(cylinders.len(), n);
    let observed = cylinders[0];
    let mut flight = Vec::new();
    for tick in 1..=185 {
        let start = Instant::now();
        sim.step_exact(1)?;
        let elapsed = start.elapsed().as_nanos() as u64;
        if (10..=95).contains(&tick) {
            flight.push(elapsed);
        }
        let state = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(observed)
            .unwrap();
        if state.velocity[2] > 0. {
            let engine = sim.world().resource::<PhysicsBackend>();
            let axial = cylinders
                .iter()
                .filter(|id| {
                    let s = engine.state(**id).unwrap();
                    s.velocity[2] > 0.9 && s.angular_velocity.iter().all(|w| w.abs() < 1e-9)
                })
                .count();
            let expected = if enabled { n } else { 0 };
            if axial != expected {
                return Err(
                    format!("expected {expected} centered rebounds, observed {axial}").into(),
                );
            }
            return Ok(
                json!({"bodies":n,"fixed_boxes":3*n,"centered_rebounds":axial,"enabled":enabled,"flight_ns":flight,"impact_ns":elapsed,"impact_tick":tick,"outgoing_z":state.velocity[2],"omega":state.angular_velocity}),
            );
        }
    }
    Err("missing full-cap impact".into())
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let n: usize = args.next().unwrap_or_else(|| "128".into()).parse()?;
    let repeats: usize = args.next().unwrap_or_else(|| "7".into()).parse()?;
    if !(1..=1024).contains(&n) || repeats == 0 {
        return Err("supply 1..1024 cylinders and a positive paired repeat count".into());
    }
    let xml = fixture(n);
    let mut rows = Vec::new();
    for i in 0..repeats {
        for enabled in [i % 2 != 0, i % 2 == 0] {
            rows.push(run(&xml, n, enabled)?);
        }
    }
    println!("{}", serde_json::to_string(&rows)?);
    Ok(())
}
