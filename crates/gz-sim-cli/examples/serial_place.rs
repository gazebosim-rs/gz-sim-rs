// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Place a free parcel onto a sliding shelf using solved support reactions.
#![allow(deprecated)]
use std::{error::Error, path::PathBuf};

use clap::{Parser, ValueEnum};
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, JointSample, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
    set_joint_controller_command, set_joint_force,
};
use serde::Serialize;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WEIGHT_N: f64 = 0.2 * 9.81;
const STEPS: usize = 12_000;
const CAPTURE_STRIDE: usize = 40;
const DESCENT_START: usize = 4600;
const FINAL_SAMPLES: usize = 500;
const GRIPPER: &str = include_str!("../../../examples/worlds/local_robot_serial_grasp.sdf");
const SHELF: &str = r#"
    <model name="receiver"><link name="shelf"><pose>0.30 0 0.574 0 0 0</pose>
      <inertial><mass>1</mass><inertia><ixx>0.003</ixx><iyy>0.003</iyy><izz>0.005</izz></inertia></inertial>
      <collision name="shelf"><geometry><box><size>0.14 0.14 0.012</size></box></geometry></collision>
      <visual name="shelf"><geometry><box><size>0.14 0.14 0.012</size></box></geometry><material><diffuse>0.27 0.73 0.48 1</diffuse></material></visual>
    </link><plugin filename="gz-sim-prescribed-motion-system" name="gz::sim::systems::PrescribedMotion">
      <trajectory end="hold">
        <waypoint time_ns="0" displacement="0 0 0" velocity="0 0 0"/>
        <waypoint time_ns="3600000000" displacement="0 0 0" velocity="0 0 0"/>
        <waypoint time_ns="4350000000" displacement="-0.30 0 0" velocity="0 0 0"/>
      </trajectory></plugin>
    </model>
"#;
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum Trial {
    Supported,
    Early,
    MissingShelf,
}
#[derive(Parser)]
struct Args {
    #[arg(value_enum)]
    trial: Trial,
    /// New native recording directory.
    output: PathBuf,
}

fn scene(trial: Trial) -> String {
    let shelf = if trial == Trial::MissingShelf {
        SHELF.replace("-0.30 0 0", "0 0 0")
    } else {
        SHELF.to_owned()
    };
    GRIPPER.replace("</world>", &format!("{shelf}</world>"))
}

struct Part {
    entity: bevy_ecs::entity::Entity,
    id: u64,
}
fn part(sim: &Simulation, name: &str) -> Result<Part> {
    let name = format!("default::{name}");
    let row = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .ok_or_else(|| format!("missing {name}"))?;
    Ok(Part {
        entity: row.id(),
        id: row.get::<EntityId>().ok_or("missing entity ID")?.get(),
    })
}
fn body(sim: &Simulation, part: &Part) -> Result<BodyId> {
    Ok(sim
        .world()
        .get::<BodyHandle>(part.entity)
        .ok_or("missing body")?
        .id())
}

#[derive(Default)]
struct SupportWindow {
    samples: [f64; 30],
    count: usize,
    next: usize,
}

impl SupportWindow {
    fn observe(&mut self, top_face: bool, normal_force: f64, vertical_speed: f64) -> Option<f64> {
        if !(top_face && normal_force >= 0.2 * WEIGHT_N && vertical_speed.abs() <= 0.01) {
            self.count = 0;
            self.next = 0;
            return None;
        }
        self.samples[self.next] = normal_force;
        self.next = (self.next + 1) % self.samples.len();
        self.count = (self.count + 1).min(self.samples.len());
        let average = self.samples.iter().sum::<f64>() / self.samples.len() as f64;
        (self.count == self.samples.len() && (0.94 * WEIGHT_N..=1.25 * WEIGHT_N).contains(&average))
            .then_some(average)
    }
}

#[derive(Debug, Default, PartialEq, Serialize)]
struct Outcome {
    open_tick: Option<usize>,
    lift_mm: f64,
    release_z_m: f64,
    shelf_force_at_open_n: f64,
    shelf_mean_before_open_n: Option<f64>,
    peak_downward_speed_after_open_m_s: f64,
    peak_shelf_impulse_after_open_n_s: f64,
    flight_after_open_ms: usize,
    downward_travel_after_open_mm: f64,
    peak_wrong_tray_n: f64,
    qualified_wrong_tray_ticks: usize,
    final_shelf_load_n: f64,
    final_pad_load_n: f64,
    final_tray_load_n: f64,
    parcel_end_z_m: f64,
    hand_end_z_m: f64,
}

// The named shelf and tray have horizontal tops. A tilted side manifold does
// not qualify. Interval magnitudes retain impulses from earlier native slices.
fn contacts(
    sim: &Simulation,
    load: BodyId,
    peers: [BodyId; 4],
    descending: bool,
) -> Result<([f64; 4], [bool; 2])> {
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or("multifidelity solver required")?;
    // A prescribed shelf splits each public step into native solves. Contact
    // magnitudes must include every slice, including an early impact that
    // disappeared by the last slice. The gate separately checks the face.
    let mut normal = [0.; 4];
    for row in native.translation_interval().1 {
        let peer = if row.a == load {
            row.b
        } else if row.b == load {
            row.a
        } else {
            continue;
        };
        if let Some(slot) = peers.iter().position(|&id| id == peer) {
            normal[slot] += row.normal_impulse;
        }
    }
    let mut top = [false; 2];
    if descending && normal[..2].iter().any(|&j| j > 0.) {
        for p in native.rigid_contact_patches() {
            let (peer, nz) = if p.a == load {
                (p.b, p.normal_on_a[2])
            } else if p.b == load {
                (p.a, -p.normal_on_a[2])
            } else {
                continue;
            };
            if let Some(slot) = peers[..2].iter().position(|&body| body == peer) {
                top[slot] |= nz > 0.999 && p.points.iter().any(|point| point.normal_impulse > 0.);
            }
        }
    }
    Ok((normal, top))
}

fn run(
    sim: &mut Simulation,
    trial: Trial,
    mut capture: impl FnMut(&Simulation, usize) -> Result<()>,
) -> Result<Outcome> {
    let parcel = part(sim, "parcel::box")?;
    let palm = part(sim, "gripper::palm")?;
    let spindle = part(sim, "gripper::spindle")?;
    let wrist = part(sim, "gripper::wrist")?;
    let jaw_joints = [
        part(sim, "gripper::close_left")?.id,
        part(sim, "gripper::close_right")?.id,
    ];
    let load = body(sim, &parcel)?;
    let id = |name| part(sim, name).and_then(|p| body(sim, &p));
    let peers = [
        id("receiver::shelf")?,
        id("stand::post")?,
        id("gripper::left")?,
        id("gripper::right")?,
    ];
    let mut out = Outcome::default();
    let mut window = SupportWindow::default();
    let mut tray_window = SupportWindow::default();
    let mut contact_hold = None;
    let start_z = sim
        .world()
        .get::<WorldPose>(parcel.entity)
        .ok_or("missing parcel pose")?
        .translation[2];
    let dt = sim.step_duration().as_secs_f64();
    capture(sim, 0)?;
    for tick in 1..=STEPS {
        if tick == 3660 {
            set_joint_controller_command(sim, wrist.id, 0.)?;
        }
        if tick == DESCENT_START && trial != Trial::Early {
            set_joint_controller_command(sim, spindle.id, -0.018)?;
        }
        sim.step_exact(1)?;
        let pose = sim
            .world()
            .get::<WorldPose>(parcel.entity)
            .ok_or("missing parcel pose")?;
        let vz = sim
            .world()
            .get::<Velocity>(parcel.entity)
            .ok_or("missing parcel velocity")?
            .linear[2];
        let (normal, top) = contacts(sim, load, peers, tick >= DESCENT_START)?;
        if tick == 3600 {
            out.lift_mm = (pose.translation[2] - start_z) * 1000.;
        }
        let z = pose.translation[2];
        let shelf_n = normal[0] / dt;
        let force_n = (normal[0] + normal[1]) / dt;
        if tick >= DESCENT_START {
            out.peak_wrong_tray_n = out.peak_wrong_tray_n.max(normal[1] / dt);
        }
        if tick >= DESCENT_START
            && trial != Trial::Early
            && out.open_tick.is_none()
            && (top[0] || top[1] || contact_hold.is_some())
        {
            let q = sim
                .world()
                .get::<JointSample>(spindle.entity)
                .ok_or("missing spindle state")?
                .state
                .position;
            let command = contact_hold.get_or_insert(q);
            // Load-control the final millimetre with the existing position
            // actuator. Too little upward reaction calls for a lower spindle.
            *command += (0.003 * (force_n - WEIGHT_N)).clamp(-0.008, 0.008) * dt;
            set_joint_controller_command(sim, spindle.id, *command)?;
        }
        let supported = window.observe(top[0], shelf_n, vz);
        out.qualified_wrong_tray_ticks +=
            usize::from(tray_window.observe(top[1], normal[1] / dt, vz).is_some());
        if out.open_tick.is_none()
            && ((trial == Trial::Early && tick == 4450) || supported.is_some())
        {
            out.open_tick = Some(tick);
            out.release_z_m = z;
            out.shelf_force_at_open_n = shelf_n;
            out.shelf_mean_before_open_n = supported;
            let q = sim
                .world()
                .get::<JointSample>(spindle.entity)
                .ok_or("missing spindle state")?
                .state
                .position;
            set_joint_controller_command(sim, spindle.id, q)?;
            for jaw in jaw_joints {
                set_joint_force(sim, jaw, -20.)?;
            }
        }
        if let Some(at) = out.open_tick {
            if tick == at + 350 {
                set_joint_controller_command(sim, spindle.id, 1.6)?;
            }
            if tick > at {
                out.peak_downward_speed_after_open_m_s =
                    out.peak_downward_speed_after_open_m_s.max(-vz);
                out.peak_shelf_impulse_after_open_n_s =
                    out.peak_shelf_impulse_after_open_n_s.max(normal[0]);
                out.downward_travel_after_open_mm = out
                    .downward_travel_after_open_mm
                    .max((out.release_z_m - z) * 1000.);
                out.flight_after_open_ms +=
                    usize::from(normal[0] == 0. && normal[2] + normal[3] == 0.);
            }
        }
        if tick > STEPS - FINAL_SAMPLES {
            out.final_shelf_load_n += shelf_n / FINAL_SAMPLES as f64;
            out.final_tray_load_n += normal[1] / (FINAL_SAMPLES as f64 * dt);
            out.final_pad_load_n += (normal[2] + normal[3]) / (FINAL_SAMPLES as f64 * dt);
        }
        out.parcel_end_z_m = z;
        out.hand_end_z_m = sim
            .world()
            .get::<WorldPose>(palm.entity)
            .ok_or("missing palm pose")?
            .translation[2];
        if tick % CAPTURE_STRIDE == 0 {
            capture(sim, tick)?;
        }
    }
    Ok(out)
}
fn main() -> Result<()> {
    let args = Args::parse();
    let xml = scene(args.trial);
    let sdf = sdformat::load_str(&xml, "serial_place.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "serial_place.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut frames = Vec::new();
    let outcome = run(&mut sim, args.trial, |sim, _| {
        frames.push(gz_sim_cli::primitive_frame(
            sim,
            &sdf,
            "default",
            frames.len() as u64 + 1,
        )?);
        Ok(())
    })?;
    gz_sim_cli::export_archive(&args.output, Default::default(), frames)?;
    println!("{}", serde_json::to_string_pretty(&outcome)?);
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn support_needs_continuous_upright_contact_and_a_settled_load() {
        let mut window = SupportWindow::default();
        for (face, force, speed) in [
            (false, WEIGHT_N, 0.),
            (true, 0., 0.),
            (true, WEIGHT_N, 0.02),
            (true, WEIGHT_N, f64::NAN),
        ] {
            for _ in 0..29 {
                assert_eq!(window.observe(true, WEIGHT_N, 0.), None);
            }
            assert_eq!(window.observe(face, force, speed), None);
        }
        for _ in 0..30 {
            assert_eq!(window.observe(true, 3. * WEIGHT_N, 0.), None);
        }
        for _ in 0..30 {
            window.observe(true, WEIGHT_N, 0.);
        }
        assert!((window.observe(true, WEIGHT_N, 0.).unwrap() - WEIGHT_N).abs() < 1e-14);
    }

    fn robot() -> Simulation {
        load_world_with_sdf_step(
            &scene(Trial::Supported),
            "serial_place.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap()
    }

    #[test]
    fn an_incomplete_workcell_and_failed_recording_propagate_before_export() {
        for name in [
            "parcel::box",
            "gripper::palm",
            "gripper::spindle",
            "gripper::wrist",
            "gripper::close_left",
            "gripper::close_right",
            "receiver::shelf",
            "stand::post",
            "gripper::left",
            "gripper::right",
        ] {
            let mut sim = robot();
            let entity = part(&sim, name).unwrap().entity;
            sim.world_mut().entity_mut(entity).remove::<Name>();
            assert!(
                run(&mut sim, Trial::Supported, |_, _| Ok(()))
                    .unwrap_err()
                    .to_string()
                    .contains(name)
            );
            assert_eq!(sim.committed_update_info().unwrap().sim_time.as_nanos(), 0);
        }
        let mut sim = robot();
        let parcel = part(&sim, "parcel::box").unwrap();
        sim.world_mut()
            .entity_mut(parcel.entity)
            .remove::<BodyHandle>();
        assert_eq!(
            run(&mut sim, Trial::Supported, |_, _| Ok(()))
                .unwrap_err()
                .to_string(),
            "missing body"
        );
        let mut sim = robot();
        let parcel = part(&sim, "parcel::box").unwrap();
        sim.world_mut()
            .entity_mut(parcel.entity)
            .remove::<WorldPose>();
        assert_eq!(
            run(&mut sim, Trial::Supported, |_, _| Ok(()))
                .unwrap_err()
                .to_string(),
            "missing parcel pose"
        );
        sim.world_mut()
            .entity_mut(parcel.entity)
            .remove::<EntityId>();
        assert_eq!(
            run(&mut sim, Trial::Supported, |_, _| Ok(()))
                .unwrap_err()
                .to_string(),
            "missing entity ID"
        );
        for stop in [0, CAPTURE_STRIDE] {
            let mut sim = robot();
            let failure = run(&mut sim, Trial::Supported, |_, tick| {
                if tick == stop {
                    Err("output unavailable".into())
                } else {
                    Ok(())
                }
            })
            .unwrap_err();
            assert_eq!(failure.to_string(), "output unavailable");
        }
    }

    #[test]
    fn correct_shelf_transfers_the_load_early_opening_drops_it_and_the_wrong_surface_keeps_the_jaws_closed()
     {
        for trial in [Trial::Supported, Trial::Early, Trial::MissingShelf] {
            let mut sim = load_world_with_sdf_step(
                &scene(trial),
                "serial_place.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity,
            )
            .unwrap();
            let outcome = run(&mut sim, trial, |_, _| Ok(())).unwrap();
            eprintln!("{trial:?} {outcome:?}");
            assert!((outcome.lift_mm - 126.).abs() < 1.);
            match trial {
                Trial::Supported => {
                    assert!(
                        outcome
                            .open_tick
                            .is_some_and(|t| (5500..11000).contains(&t))
                    );
                    assert!(
                        outcome
                            .shelf_mean_before_open_n
                            .is_some_and(|f| (0.94 * WEIGHT_N..1.25 * WEIGHT_N).contains(&f))
                    );
                    assert_eq!(outcome.flight_after_open_ms, 0);
                    assert!(outcome.downward_travel_after_open_mm < 0.2);
                    assert!(outcome.peak_shelf_impulse_after_open_n_s < 0.02);
                    assert!(outcome.peak_downward_speed_after_open_m_s < 0.04);
                }
                Trial::Early => {
                    assert_eq!(outcome.open_tick, Some(4450));
                    assert_eq!(outcome.shelf_force_at_open_n, 0.);
                    assert_eq!(outcome.shelf_mean_before_open_n, None);
                    assert!(
                        outcome.flight_after_open_ms > 75
                            && outcome.downward_travel_after_open_mm > 40.
                    );
                    assert!(
                        outcome.peak_shelf_impulse_after_open_n_s > 0.1
                            && outcome.peak_downward_speed_after_open_m_s > 0.8
                    );
                }
                Trial::MissingShelf => {
                    assert_eq!(outcome.open_tick, None);
                    assert!(outcome.qualified_wrong_tray_ticks > 1000);
                    assert!(outcome.final_pad_load_n > 10. && outcome.final_shelf_load_n == 0.);
                    assert!((outcome.final_tray_load_n - WEIGHT_N).abs() < 0.05);
                }
            }
            if trial != Trial::MissingShelf {
                assert!((outcome.final_shelf_load_n - WEIGHT_N).abs() < 0.002);
                assert_eq!(
                    (outcome.final_pad_load_n, outcome.final_tray_load_n),
                    (0., 0.)
                );
                assert!(
                    (outcome.parcel_end_z_m - 0.615).abs() < 0.001 && outcome.hand_end_z_m > 0.82
                );
            }
            gz_sim_scene::reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(run(&mut sim, trial, |_, _| Ok(())).unwrap(), outcome);
        }
    }
}
