// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Show three selected decks from 100,000 observed bodies and a late finite payload.
use bevy_ecs::prelude::*;
use gz_physics::{BodyId, BodySpec, BodyState, DynamicBackend, PhysicsBackend, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{Phase, Simulation};
#[path = "support/fleet_visual.rs"]
mod visual;
use serde_json::json;
use std::{error::Error, path::PathBuf, time::Duration};
use visual::{box_visual, catalog};

const DT: Duration = Duration::from_millis(10);
const COUNT: usize = 100_000;
const DECK: [f64; 3] = [0.9, 0.52, 0.16];
const LOAD: [f64; 3] = [0.28, 0.28, 0.24];
#[derive(Component)]
struct Observed {
    id: BodyId,
    state: BodyState,
}
fn observe(backend: Res<PhysicsBackend>, mut q: Query<&mut Observed>) {
    backend.read_states(&mut |states| {
        for mut obs in &mut q {
            obs.state = states.state(obs.id).expect("live fleet body");
        }
    });
}
fn spec(at: [f64; 3], speed: f64, size: [f64; 3]) -> BodySpec {
    BodySpec {
        position: at,
        velocity: [speed, 0., 0.],
        shape: Shape::Box {
            half_extents: size.map(|v| v * 0.5),
        },
        mass: Some(1.),
    }
}
struct Visible {
    entity: Entity,
    model: u64,
    name: &'static str,
    size: [f64; 3],
    color: [f64; 3],
}
fn add(sim: &mut Simulation, shape: BodySpec, prescribed: bool) -> Result<Entity, Box<dyn Error>> {
    let mut resource = sim.world_mut().resource_mut::<PhysicsBackend>();
    let solver = resource
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("wrong backend")?;
    let id = if prescribed {
        solver.spawn_kinematic_link(shape, Default::default(), None)?
    } else {
        solver.spawn_link(shape, Default::default(), None)?
    };
    let state = solver.state(id).ok_or("missing new body")?;
    Ok(sim.world_mut().spawn(Observed { id, state }).id())
}
fn sample(sim: &Simulation, visible: &[Visible], sequence: u64) -> (String, String) {
    const S: f64 = std::f64::consts::FRAC_1_SQRT_2;
    let mut visuals = Vec::new();
    let mut models = Vec::new();
    // All physical bodies travel along X. The unmodeled rails mark that axis.
    for (i, z) in [0., 1., 2.].into_iter().enumerate() {
        visuals.push(box_visual(
            3 + i as u64,
            &format!("default::tracks::lane_{i}"),
            [0., 0., z - 0.13],
            [0., 0., 0., 1.],
            [0.09, 5.2, 0.026],
            [0.27, 0.33, 0.39],
        ));
    }
    models.push(catalog(2, "default::tracks", &[3, 4, 5]));
    for v in visible {
        let state = sim.world().get::<Observed>(v.entity).unwrap().state;
        let [x, y, z] = state.position;
        let [qx, qy, qz, qw] = state.orientation;
        let rotation = [S * (qx + qy), S * (qy - qx), S * (qz - qw), S * (qw + qz)];
        let name = format!("default::{}", v.name);
        visuals.push(box_visual(
            v.model + 1,
            &format!("{name}::body"),
            [y, -x, z],
            rotation,
            v.size,
            v.color,
        ));
        models.push(catalog(v.model, &name, &[v.model + 1]));
    }
    let t = sim.elapsed();
    let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":sequence,"seconds":t.as_secs(),"nanoseconds":t.subsec_nanos(),"paused":false,"visuals":visuals});
    let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,"templates":[],"models":models});
    (scene.to_string(), catalog.to_string())
}

fn main() -> Result<(), Box<dyn Error>> {
    let dir = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("usage: fleet_lifecycle REPLAY_DIRECTORY")?,
    );
    let mut sim = Simulation::new(DT)?;
    let mut solver = MultiFidelityBackend::new([0., 0., -9.81])?;
    let mut visible = Vec::new();
    for i in 0..COUNT {
        let position = match i {
            0 => [-1.55, 0., 2.],
            1 => [-1.55, 0., 0.],
            _ => [10. + 2. * (i % 320) as f64, 10. + 2. * (i / 320) as f64, 0.],
        };
        let id =
            solver.spawn_kinematic_link(spec(position, 0.8, DECK), Default::default(), None)?;
        let entity = sim
            .world_mut()
            .spawn(Observed {
                id,
                state: solver.state(id).unwrap(),
            })
            .id();
        if i < 2 {
            visible.push(Visible {
                entity,
                model: 10 + 10 * i as u64,
                name: if i == 0 { "edited" } else { "unchanged" },
                size: DECK,
                color: if i == 0 {
                    [0.04, 0.78, 0.72]
                } else {
                    [0.19, 0.55, 0.95]
                },
            });
        }
    }
    gz_physics::install(&mut sim, solver)?;
    sim.add_systems(Phase::PhysicsCommit, observe);
    let mut frames = vec![sample(&sim, &visible, 1)];
    let mut max_error = 0.0_f64;
    for tick in 1..=200 {
        let t = (tick - 1) as f64 * 0.01;
        match tick {
            71 => {
                let id = sim.world().get::<Observed>(visible[0].entity).unwrap().id;
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .set_world_velocity(id, [-0.8, 0., 0.], [0.; 3])?;
            }
            91 => {
                let entity = add(&mut sim, spec([1.25, 0., 1.], -0.65, DECK), true)?;
                visible.push(Visible {
                    entity,
                    model: 30,
                    name: "inserted",
                    size: DECK,
                    color: [0.87, 0.32, 0.06],
                });
            }
            121 => {
                let x = 1.25 - 0.65 * (t - 0.90);
                let entity = add(&mut sim, spec([x, 0., 1.40], 0., LOAD), false)?;
                visible.push(Visible {
                    entity,
                    model: 40,
                    name: "payload",
                    size: LOAD,
                    color: [0.90, 0.72, 0.18],
                });
            }
            _ => {}
        }
        if tick == 91 || tick == 121 {
            frames.push(sample(&sim, &visible, frames.len() as u64 + 1));
        }
        sim.step_exact(1)?;
        let t = sim.elapsed().as_secs_f64();
        for (i, v) in visible.iter().take(3).enumerate() {
            let subject = sim.world().get::<Observed>(v.entity).unwrap();
            let expected = match i {
                0 => -1.55 + 0.8 * t.min(0.7) - 0.8 * (t - 0.7).max(0.),
                1 => -1.55 + 0.8 * t,
                _ => 1.25 - 0.65 * (t - 0.9),
            };
            max_error = max_error.max((subject.state.position[0] - expected).abs());
            assert_eq!(
                subject.state,
                sim.world()
                    .resource::<PhysicsBackend>()
                    .state(subject.id)
                    .unwrap()
            );
        }
        if tick % 4 == 0 || tick == 70 || tick == 90 {
            frames.push(sample(&sim, &visible, frames.len() as u64 + 1));
        }
    }
    assert!(max_error < 1e-10);
    let physics = sim.world().resource::<PhysicsBackend>();
    let contacts = physics
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .active_contact_pairs();
    assert_eq!(contacts, 1);
    let load = sim
        .world()
        .get::<Observed>(visible[3].entity)
        .unwrap()
        .state;
    assert!((load.velocity[0] + 0.65).abs() < 0.02 && (load.position[2] - 1.2).abs() < 0.02);
    println!(
        "{COUNT} initially observed decks; speed reversal t=0.70 s, deck insertion t=0.90 s, native payload insertion t=1.20 s; {} samples; max deck error {max_error:.3e} m; active pairs {contacts}, payload vx {:.6} m/s, z {:.6} m",
        frames.len(),
        load.velocity[0],
        load.position[2]
    );
    gz_sim_cli::export_archive(&dir, Default::default(), frames)?;
    Ok(())
}
