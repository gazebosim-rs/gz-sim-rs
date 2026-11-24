// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Strike one outer jaw of the complete serial gripper from a matched incoming state.
#![allow(deprecated)]
use std::{error::Error, path::PathBuf, time::Duration};

use bevy_ecs::prelude::{Entity, Res, ResMut, Resource};
use clap::{Parser, ValueEnum};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::{
    ContactSegment, ContactSolverIterations, MultiFidelityBackend, RigidContactPatch,
};
use gz_sim_core::{EntityId, Phase, Simulation, StepFault, UpdateInfo};
use gz_sim_scene::{
    BodyHandle, JointSample, Name, PhysicsEngine, UniversalJointSample, load_world_with_sdf_step,
};
use serde::Serialize;

#[path = "serial_impact_inertia/compare.rs"]
mod compare;
#[path = "serial_impact_inertia/pulses.rs"]
mod pulses;
#[cfg(test)]
#[path = "serial_impact_inertia/spindle_tests.rs"]
mod spindle_tests;
#[path = "serial_impact_inertia/strike.rs"]
mod strike;
#[cfg(test)]
#[path = "serial_impact_inertia/tests.rs"]
mod tests;
#[path = "serial_impact_inertia/timing.rs"]
mod timing;

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
    /// Inspect native jaw-load transitions in the first controller tick and optionally replay those instants.
    #[arg(long, conflicts_with_all = ["compare", "timing_runs", "impact_pulses"])]
    audit_first_strike: bool,
    /// Measure the impulse transmitted through the finger and wrist against an unloaded, clock-matched arm.
    #[arg(long, requires = "audit_first_strike")]
    load_path: bool,
    /// Write the exact authored study world for the ordinary simulator CLI.
    #[arg(long)]
    write_world: Option<PathBuf>,
    /// Time public steps across newly loaded copies and check all six bodies against the observed reference.
    #[arg(long, conflicts_with_all = ["compare", "replay"], value_parser = clap::value_parser!(u32).range(1..=1000))]
    timing_runs: Option<u32>,
}

#[derive(Clone, Copy, PartialEq, ValueEnum, Serialize)]
enum PassiveSpindle {
    Capped,
    Uncapped,
}

#[derive(clap::Args, Clone, Serialize)]
struct Trial {
    /// Concentrate each fixed mass inside a centered cuboid with dimensions divided by this number.
    #[arg(long, default_value_t = 1., value_parser = divisor)]
    divisor: f64,
    /// Native contact solves per 1 ms controller interval, up to one per nanosecond. One uses the ordinary step.
    #[arg(long, default_value_t = 1, value_parser = clap::value_parser!(u32).range(1..=1_000_000))]
    substeps: u32,
    /// Use the authored local face detector. The substep budget is spent only on selected impacts.
    #[arg(long)]
    automatic: bool,
    /// Solver iterations per native call on a selected impact. The ordinary and comparison grids use their own default.
    #[arg(long, requires = "automatic", value_parser = clap::value_parser!(u32).range(1..=32))]
    impact_iterations: Option<u32>,
    /// Set the carriage guide's native stabilization frequency (Hz) on all selected and reference grids.
    #[arg(long)]
    #[serde(skip_serializing_if = "Option::is_none")]
    guide_hz: Option<f64>,
    /// Fixed-time projected Gauss–Seidel passes per selected solver iteration.
    #[arg(long, requires = "automatic", value_parser = clap::value_parser!(u32).range(1..=32))]
    impact_pgs: Option<u32>,
    /// Maximum native timestep during the predicted early arrival of a separated parallel face.
    #[arg(long, requires_all = ["automatic", "early_face_max_steps"], value_parser = clap::value_parser!(u32).range(1..))]
    #[serde(skip_serializing_if = "Option::is_none")]
    early_face_step_ns: Option<u32>,
    /// Native-call cap for the fine prefix; a later arrival keeps the ordinary impact budget.
    #[arg(long, requires_all = ["automatic", "early_face_step_ns"], value_parser = clap::value_parser!(u32).range(1..))]
    #[serde(skip_serializing_if = "Option::is_none")]
    early_face_max_steps: Option<u32>,
    /// Maximum timestep after an admitted separated face has delivered its reaction.
    #[arg(long,requires_all=["early_face_guard_ns","early_face_step_ns"],value_parser=clap::value_parser!(u32).range(1..))]
    #[serde(skip_serializing_if = "Option::is_none")]
    early_face_tail_step_ns: Option<u32>,
    /// Keep the fine clock for at least this long beyond the forecast arrival.
    #[arg(long,requires="early_face_tail_step_ns",value_parser=clap::value_parser!(u32).range(1..))]
    #[serde(skip_serializing_if = "Option::is_none")]
    early_face_guard_ns: Option<u32>,
    /// Use the qualified four-call solver only for a single pair already touching with parallel faces.
    #[arg(long, requires = "automatic")]
    touching_budget: bool,
    /// Also resolve the spindle's rated effort against its physical speed stop on advancing ticks.
    #[arg(long, requires = "automatic")]
    stop_guard: bool,
    /// First-tick budget for a uniform reference that must start its tail at the same post-impact state.
    #[arg(long,conflicts_with="automatic",value_parser=clap::value_parser!(u32).range(1..=1_000_000))]
    first_native_steps: Option<u32>,
    /// Reverse the passive 3 N·m effort to challenge the negative speed stop.
    #[arg(long, requires = "passive_spindle")]
    reverse_torque: bool,
    /// Strike the outer right jaw from positive X.
    #[arg(long)]
    right: bool,
    /// Additional initial separation from the chosen face, in micrometres.
    #[arg(long, default_value_t = 0)]
    gap_um: u32,
    /// Report each uninterrupted jaw contact and its impulse and moment on the parcel at uniform native steps.
    #[arg(long, conflicts_with_all = ["automatic", "compare", "timing_runs"])]
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    impact_pulses: bool,
    /// Remove spindle velocity control, apply a held 3 N·m input, and retain or remove its physical speed stop.
    #[arg(long, value_enum)]
    passive_spindle: Option<PassiveSpindle>,
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
    let side = if trial.right { 1.0 } else { -1.0 };
    let x = side * (0.099 + f64::from(trial.gap_um) * 1e-6);
    xml = xml.replace("<pose>-0.099 ", &format!("<pose>{x} "));
    if trial.right {
        xml = xml.replace("<linear_velocity>0.75 ", "<linear_velocity>-0.75 ");
    }
    if let Some(control) = trial.passive_spindle {
        let begin = xml
            .find(r#"<plugin filename="gz-sim-joint-position-controller-system""#)
            .unwrap();
        let end = begin + xml[begin..].find("</plugin>").unwrap() + "</plugin>".len();
        assert!(xml[begin..end].contains("<joint_name>spindle</joint_name>"));
        xml.replace_range(begin..end, "");
        if control == PassiveSpindle::Uncapped {
            xml = xml.replace("<velocity>0.8</velocity>", "");
        }
    }
    if trial.automatic {
        let pgs = trial
            .impact_pgs
            .map(|n| format!("<pgs_iterations>{n}</pgs_iterations>"))
            .unwrap_or_default();
        let touching = if trial.touching_budget {
            "<touching_solver><native_steps>4</native_steps><solver_iterations>8</solver_iterations><pgs_iterations>16</pgs_iterations></touching_solver>"
        } else {
            ""
        };
        let early = trial.early_face_step_ns.zip(trial.early_face_max_steps)
            .map(|(ns, calls)| format!("<early_face_step_ns>{ns}</early_face_step_ns><early_face_max_steps>{calls}</early_face_max_steps>"))
            .unwrap_or_default();
        let early = if let Some((step, guard)) =
            trial.early_face_tail_step_ns.zip(trial.early_face_guard_ns)
        {
            format!(
                "{early}<early_face_tail_step_ns>{step}</early_face_tail_step_ns><early_face_guard_ns>{guard}</early_face_guard_ns>"
            )
        } else {
            early
        };
        let speed_stop = if trial.stop_guard {
            "<speed_stop_joint>default::gripper::spindle</speed_stop_joint>"
        } else {
            ""
        };
        let calls = trial.substeps;
        let iterations = trial.impact_iterations.unwrap_or(4);
        let plugin = format!(
            r#"
          <plugin name="gz::sim::systems::PrimaryContactRefinement" filename="gz-sim-primary-contact-refinement-system">
            {speed_stop}<native_steps>{calls}</native_steps><solver_iterations>{iterations}</solver_iterations>{pgs}
            {touching}{early}
            <pair><foot>default::parcel::box</foot><support>default::gripper::left</support><incoming_face>-x</incoming_face></pair>
            <pair><foot>default::parcel::box</foot><support>default::gripper::right</support><incoming_face>+x</incoming_face></pair>
          </plugin></world>"#
        );
        xml = xml.replace("</world>", &plugin);
    }
    if let Some(hz) = trial.guide_hz {
        xml = xml.replace("</world>", &format!(r#"<plugin name="gz::sim::systems::MountedPrismaticGuide" filename="gz-sim-mounted-prismatic-guide-system"><natural_frequency_hz joint="default::gripper::travel">{hz}</natural_frequency_hz></plugin></world>"#));
    }
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
    right_ns: f64,
    other_ns: f64,
    normal_on_parcel_ns: [f64; 3],
    first_end_ns: Option<u128>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pulses: Option<Vec<pulses::JawPulse>>,
}
#[derive(Clone, Copy)]
struct Contacts {
    parcel: BodyId,
    left: BodyId,
    right: BodyId,
}
impl Contacts {
    fn counterpart(self, a: BodyId, b: BodyId) -> Option<(BodyId, f64)> {
        if a == self.parcel {
            Some((b, 1.))
        } else if b == self.parcel {
            Some((a, -1.))
        } else {
            None
        }
    }
}
impl Impact {
    fn add(
        &mut self,
        ids: Contacts,
        other: BodyId,
        impulse: f64,
        on_parcel: [f64; 3],
        end: Option<Duration>,
    ) {
        if other == ids.left {
            self.left_ns += impulse;
        } else if other == ids.right {
            self.right_ns += impulse;
        } else {
            self.other_ns += impulse;
        }
        if (other == ids.left || other == ids.right)
            && let Some(end) = end.filter(|_| impulse > 0.)
        {
            let ns = end.as_nanos();
            self.first_end_ns = Some(self.first_end_ns.map_or(ns, |old| old.min(ns)));
        }
        for (sum, v) in self.normal_on_parcel_ns.iter_mut().zip(on_parcel) {
            *sum += v;
        }
    }
    fn record(
        &mut self,
        patches: &[RigidContactPatch],
        ids: Contacts,
        end: Duration,
        dt: Duration,
    ) {
        if let Some(pulses) = &mut self.pulses {
            pulses::record(pulses, patches, ids, end, dt);
        }
        for p in patches {
            if let Some((other, sign)) = ids.counterpart(p.a, p.b) {
                for point in &p.points {
                    let j = point.normal_impulse;
                    self.add(
                        ids,
                        other,
                        j,
                        p.normal_on_a.map(|v| v * j * sign),
                        Some(end),
                    );
                }
            }
        }
    }
    fn integrated(r: &gz_sim_scene::primary_contact::Refinement, ids: Contacts) -> Self {
        let mut impact = Self::default();
        for p in &r.interval.pairs {
            if let Some((other, sign)) = ids.counterpart(p.a, p.b) {
                impact.add(
                    ids,
                    other,
                    p.normal_impulse,
                    p.normal_impulse_on_a.map(|v| v * sign),
                    p.first_positive_end.map(|t| r.start_time + t),
                );
            }
        }
        impact
    }
}

#[derive(Clone, Serialize, PartialEq, Debug)]
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
    native_steps: u32,
    speed_stop_floor: u32,
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
    #[serde(skip)]
    first_solver: Option<ContactSolverIterations>,
    #[serde(skip)]
    first_early_schedule: Option<Vec<ContactSegment>>,
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

fn load_robot(trial: &Trial, xml: &str) -> Result<(Simulation, Vec<BodyId>, [Entity; 5])> {
    let mut sim = load_world_with_sdf_step(
        xml,
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
    bodies.push(body("parcel::box"));
    let joint = JOINTS.map(|name| entity(&sim, &format!("gripper::{name}")));
    if trial.passive_spindle.is_some() {
        let id = sim.world().get::<EntityId>(joint[1]).unwrap().get();
        gz_sim_scene::JointForceCommand::set_batch(
            &mut sim,
            [(id, if trial.reverse_torque { -3. } else { 3. })],
        )?;
    }
    Ok((sim, bodies, joint))
}

fn run(trial: Trial) -> Result<Run> {
    run_with_first_solver(trial, None, None)
}

fn run_with_first_solver(
    trial: Trial,
    first_solver: Option<ContactSolverIterations>,
    matched_schedule: Option<Vec<ContactSegment>>,
) -> Result<Run> {
    let xml = fixture(&trial);
    let sdf = sdformat::load_str(&xml, "serial-impact.sdf", &sdformat::NoResources)?;
    let (mut sim, bodies, joint) = load_robot(&trial, &xml)?;
    let ids = Contacts {
        left: bodies[3],
        right: bodies[4],
        parcel: bodies[5],
    };
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
    let first = trial.first_native_steps.unwrap_or(n);
    let uniform = (n > 1 || first > 1) && !trial.automatic;
    let report_pulses = trial.impact_pulses;
    if uniform {
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
                    native_steps: if clock.start_time.is_zero() { first } else { n },
                    iterations: if clock.start_time.is_zero() {
                        first_solver.unwrap_or_else(|| native.ordinary_contact_iterations())
                    } else {
                        native.ordinary_contact_iterations()
                    },
                };
                *impact = Impact {
                    pulses: report_pulses.then(Vec::new),
                    ..Default::default()
                };
                let segments = if clock.start_time.is_zero() {
                    matched_schedule
                        .as_deref()
                        .unwrap_or_else(|| std::slice::from_ref(&segment))
                } else {
                    std::slice::from_ref(&segment)
                };
                if let Err(e) = native.observe_contact_schedule(clock.dt, segments, &[], |cut| {
                    impact.record(
                        &cut.contact_patches(),
                        ids,
                        clock.start_time + cut.elapsed,
                        cut.duration,
                    );
                }) {
                    fault.report(e);
                }
            },
        );
    }
    let mut first_selected_solver = None;
    let mut first_early_schedule = None;
    let mut samples = Vec::new();
    let mut frames = Vec::new();
    for ms in 0..=trial.ticks {
        if ms > 0 {
            sim.step_exact(1)?;
            let selected = gz_sim_scene::primary_contact::last_refinement(&sim)
                .filter(|p| p.sim_time == sim.elapsed());
            if ms == 1 {
                first_selected_solver = selected.map(|r| r.solver);
                first_early_schedule =
                    selected.and_then(|r| r.realized_face_schedule().map(<[_]>::to_vec));
            }
            let native_steps = selected.map_or(
                if uniform && ms == 1 {
                    first
                } else if uniform {
                    n
                } else {
                    1
                },
                |r| r.interval.native_steps,
            );
            let impact = if let Some(r) = selected {
                Impact::integrated(r, ids)
            } else if !uniform {
                let mut impact = Impact {
                    pulses: report_pulses.then(Vec::new),
                    ..Default::default()
                };
                let native = sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .unwrap();
                impact.record(
                    &native.rigid_contact_patches(),
                    ids,
                    Duration::from_millis(u64::from(ms)),
                    Duration::from_millis(1),
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
                native_steps,
                speed_stop_floor: selected.map_or(1, |r| r.speed_stop_floor),
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
        first_solver: first_selected_solver,
        first_early_schedule,
        initial_patch_points,
        incoming,
        samples,
    })
}
fn main() -> Result<()> {
    let args = Args::parse();
    if let Some(path) = args.write_world {
        std::fs::write(path, fixture(&args.trial))?;
    }
    if args.audit_first_strike {
        serde_json::to_writer(
            std::io::stdout().lock(),
            &strike::audit(args.trial, args.load_path)?,
        )?;
        return Ok(());
    }
    let report = run(args.trial)?;
    let output = std::io::stdout().lock();
    if let Some(count) = args.timing_runs {
        serde_json::to_writer(output, &timing::measure(&report, count)?)?;
    } else if args.compare {
        serde_json::to_writer_pretty(output, &compare::against_fine(&report)?)?;
    } else {
        serde_json::to_writer(output, &report)?;
    }
    Ok(())
}
