// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Strike one outer jaw of the complete serial gripper from a matched incoming state.
#![allow(deprecated)]
use std::{error::Error, path::PathBuf, time::Duration};

use bevy_ecs::prelude::{Entity, Res, ResMut, Resource};
use clap::Parser;
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::{ContactSegment, MultiFidelityBackend, RigidContactPatch};
use gz_sim_core::{EntityId, Phase, Simulation, StepFault, UpdateInfo};
use gz_sim_scene::{
    BodyHandle, JointSample, Name, PhysicsEngine, UniversalJointSample, load_world_with_sdf_step,
};
use serde::Serialize;

#[path = "serial_impact_inertia/compare.rs"]
mod compare;
#[cfg(test)]
#[path = "serial_impact_inertia/tests.rs"]
mod tests;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WORLD: &str = include_str!("../../../examples/worlds/local_robot_serial_transfer.sdf");
const LINKS: [(&str, f64, [f64; 3]); 5] = [
    ("carriage", 1.2, [0.08, 0.08, 0.04]),
    ("nut", 0.7, [0.17, 0.10, 0.045]),
    ("palm", 0.4, [0.175, 0.095, 0.035]),
    ("left", 0.08, [0.018, 0.067, 0.10]),
    ("right", 0.08, [0.018, 0.067, 0.10]),
];
const JOINTS: [&str; 5] = ["travel", "spindle", "close_left", "close_right", "wrist"];

#[derive(Parser, Clone, Serialize)]
struct Args {
    #[command(flatten)]
    trial: Trial,
    /// Compare the selected public trajectory with independently initialized 250 ns and 125 ns native steps.
    #[arg(long)]
    compare: bool,
}

#[derive(clap::Args, Clone, Serialize)]
struct Trial {
    /// Concentrate each fixed mass inside a centered cuboid with dimensions divided by this number.
    #[arg(long, default_value_t = 1., value_parser = divisor)]
    divisor: f64,
    /// Native contact solves in one ordinary 1 ms command/publication interval. One uses the unmodified step.
    #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u32).range(1..=8000))]
    substeps: u32,
    /// Move the incoming parcel sideways to miss both jaws.
    #[arg(long)]
    miss: bool,
    #[arg(long, default_value_t = 24, value_parser = clap::value_parser!(u32).range(1..=100))]
    ticks: u32,
    /// Export ordinary committed primitive frames to a native playback directory.
    #[arg(long)]
    replay: Option<PathBuf>,
}

fn divisor(input: &str) -> std::result::Result<f64, String> {
    let d = input.parse::<f64>().map_err(|e| e.to_string())?;
    if (1.0..=1024.0).contains(&d) {
        Ok(d)
    } else {
        Err("core divisor must be between 1 and 1024".into())
    }
}
fn tensor(m: f64, dims: [f64; 3], divisor: f64) -> [f64; 3] {
    let [x, y, z] = dims.map(|side| (side / divisor).powi(2) * m / 12.);
    [y + z, z + x, x + y]
}
fn fixture(trial: &Trial) -> String {
    let mut xml = WORLD
        .replace(
            "<model name=\"parcel\">",
            r#"<model name="parcel">
        <plugin name="gz::sim::systems::InitialVelocity" filename="gz-sim-initial-velocity-system">
          <linear_velocity>0.75 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity>
        </plugin>"#,
        )
        .replace(
            "<pose>0 0 0.535 0 0 0</pose>",
            &format!(
                "<pose>-0.099 {} 0.569 0 0 0</pose>",
                if trial.miss { 0.16 } else { 0.01 }
            ),
        );
    for (name, mass, dims) in LINKS {
        let [ix, iy, iz] = tensor(mass, dims, trial.divisor);
        let link = xml.find(&format!("<link name=\"{name}\">")).unwrap();
        let begin = link + xml[link..].find("<inertia>").unwrap();
        let end = begin + xml[begin..].find("</inertia>").unwrap();
        xml.replace_range(
            begin..end,
            &format!("<inertia><ixx>{ix}</ixx><iyy>{iy}</iyy><izz>{iz}</izz>"),
        );
    }
    xml
}

#[derive(Default, Clone, Resource, Serialize)]
struct Impact {
    left_ns: f64,
    other_ns: f64,
    normal_on_parcel_ns: [f64; 3],
    first_end_ns: Option<u128>,
}
impl Impact {
    fn record(
        &mut self,
        patches: &[RigidContactPatch],
        parcel: BodyId,
        left: BodyId,
        end: Duration,
    ) {
        for p in patches {
            let (other, sign) = if p.a == parcel {
                (p.b, 1.)
            } else if p.b == parcel {
                (p.a, -1.)
            } else {
                continue;
            };
            for point in &p.points {
                let j = point.normal_impulse;
                if other == left {
                    self.left_ns += j;
                    if j > 0. {
                        self.first_end_ns.get_or_insert(end.as_nanos());
                    }
                } else {
                    self.other_ns += j;
                }
                for (i, sum) in self.normal_on_parcel_ns.iter_mut().enumerate() {
                    *sum += sign * (p.normal_on_a[i] * j);
                }
            }
        }
    }
}

#[derive(Serialize, PartialEq, Debug)]
struct State {
    xyz: [f64; 3],
    q: [f64; 4],
    v: [f64; 3],
    w: [f64; 3],
}
impl From<BodyState> for State {
    fn from(s: BodyState) -> Self {
        Self {
            xyz: s.position,
            q: s.orientation,
            v: s.velocity,
            w: s.angular_velocity,
        }
    }
}
#[derive(Serialize)]
struct Sample {
    ms: u32,
    bodies: Vec<State>,
    impact: Impact,
    joint_position: [f64; 6],
    joint_velocity: [f64; 6],
}
#[derive(Serialize)]
struct Run {
    trial: Trial,
    tensors_kg_m2: [[f64; 3]; 5],
    native_left_tensor_kg_m2: [[f64; 3]; 3],
    native_robot_mass_kg: f64,
    initial_patch_points: usize,
    incoming: Vec<State>,
    samples: Vec<Sample>,
}

fn entity(sim: &Simulation, name: &str) -> Entity {
    let full = format!("default::{name}");
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == full))
        .unwrap()
        .id()
}
fn states(sim: &Simulation, ids: &[BodyId]) -> Vec<State> {
    let backend = sim.world().resource::<PhysicsBackend>();
    ids.iter()
        .map(|id| backend.state(*id).unwrap().into())
        .collect()
}

fn run(trial: Trial) -> Result<Run> {
    let xml = fixture(&trial);
    let sdf = sdformat::load_str(&xml, "serial-impact.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "serial-impact.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let body = |name: &str| {
        sim.world()
            .get::<BodyHandle>(entity(&sim, name))
            .unwrap()
            .id()
    };
    let mut bodies = LINKS
        .map(|(name, _, _)| body(&format!("gripper::{name}")))
        .to_vec();
    let left = bodies[3];
    let parcel = body("parcel::box");
    bodies.push(parcel);
    let joint = JOINTS.map(|name| entity(&sim, &format!("gripper::{name}")));
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    let initial_patch_points = native
        .rigid_contact_patches()
        .iter()
        .map(|p| p.points.len())
        .sum();
    let subtree = |i: usize| {
        let id = sim.world().get::<EntityId>(joint[i]).unwrap().get();
        gz_sim_scene::locked_subtree_inertia(&sim, id)
    };
    let native_left_tensor_kg_m2 = subtree(2)?.inertia_kg_m2;
    let native_robot_mass_kg = subtree(0)?.mass_kg;
    let incoming = states(&sim, &bodies);
    let n = trial.substeps;
    if n > 1 {
        sim.initialize_schedules()?;
        assert_eq!(
            sim.remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)?,
            1
        );
        sim.world_mut().insert_resource(Impact::default());
        sim.add_systems(
            Phase::PhysicsStep,
            move |clock: Res<UpdateInfo>,
                  mut backend: ResMut<PhysicsBackend>,
                  mut impact: ResMut<Impact>,
                  fault: Res<StepFault>| {
                let native = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
                let segment = ContactSegment {
                    duration: clock.dt,
                    native_steps: n,
                    iterations: native.ordinary_contact_iterations(),
                };
                *impact = Impact::default();
                if let Err(e) = native.observe_contact_schedule(clock.dt, &[segment], &[], |cut| {
                    impact.record(
                        &cut.contact_patches(),
                        parcel,
                        left,
                        clock.start_time + cut.elapsed,
                    );
                }) {
                    fault.report(e);
                }
            },
        );
    }
    let mut samples = Vec::new();
    let mut frames = Vec::new();
    for ms in 0..=trial.ticks {
        if ms > 0 {
            sim.step_exact(1)?;
            let impact = if n == 1 {
                let mut impact = Impact::default();
                let native = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .unwrap();
                impact.record(
                    &native.rigid_contact_patches(),
                    parcel,
                    left,
                    Duration::from_millis(u64::from(ms)),
                );
                impact
            } else {
                sim.world().resource::<Impact>().clone()
            };
            let mut joint_position = [0.; 6];
            let mut joint_velocity = [0.; 6];
            for i in 0..4 {
                let state = sim.world().get::<JointSample>(joint[i]).unwrap().state;
                joint_position[i] = state.position;
                joint_velocity[i] = state.velocity;
            }
            let wrist = sim
                .world()
                .get::<UniversalJointSample>(joint[4])
                .unwrap()
                .state;
            joint_position[4..].copy_from_slice(&wrist.position);
            joint_velocity[4..].copy_from_slice(&wrist.velocity);
            samples.push(Sample {
                ms,
                bodies: states(&sim, &bodies),
                impact,
                joint_position,
                joint_velocity,
            });
        }
        if trial.replay.is_some() {
            frames.push(gz_sim_cli::primitive_frame(
                &sim,
                &sdf,
                "default",
                u64::from(ms) + 1,
            )?);
        }
    }
    if let Some(path) = &trial.replay {
        gz_sim_cli::export_archive(path, Default::default(), frames)?;
    }
    Ok(Run {
        tensors_kg_m2: LINKS.map(|(_, m, d)| tensor(m, d, trial.divisor)),
        trial,
        native_left_tensor_kg_m2,
        native_robot_mass_kg,
        initial_patch_points,
        incoming,
        samples,
    })
}
fn main() -> Result<()> {
    let args = Args::parse();
    let report = run(args.trial)?;
    let output = std::io::stdout().lock();
    if args.compare {
        serde_json::to_writer_pretty(output, &compare::against_fine(&report)?)?;
    } else {
        serde_json::to_writer(output, &report)?;
    }
    Ok(())
}
