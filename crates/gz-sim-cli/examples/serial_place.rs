// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Place a free parcel onto a sliding shelf using solved support reactions.
#![allow(deprecated)]
use std::{error::Error, path::PathBuf};

use clap::{Parser, ValueEnum};
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::receiving_support::{ReceivingSupport, SupportCriteria, SupportReading};
use gz_sim_scene::{
    BodyHandle, JointForceCommand, JointSample, Name, PhysicsEngine, Velocity, WorldPose,
    load_world_with_sdf_step, set_joint_controller_command,
};
use serde::Serialize;

#[path = "serial_place/withdrawal.rs"]
mod withdrawal;

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
    Withdrawal,
    OpenLoop,
}
#[derive(Parser)]
struct Args {
    #[arg(value_enum)]
    trial: Trial,
    /// New native recording directory.
    output: PathBuf,
    /// Reaction time after the contact sensor detects shelf withdrawal.
    #[arg(long,default_value_t=0,value_parser=clap::value_parser!(u16).range(0..=500))]
    delay_ms: u16,
}

fn scene(trial: Trial) -> String {
    let shelf = if trial == Trial::MissingShelf {
        SHELF.replace("-0.30 0 0", "0 0 0")
    } else if matches!(trial, Trial::Withdrawal | Trial::OpenLoop) {
        SHELF.replace(
            "</trajectory>",
            &format!("{}</trajectory>", withdrawal::PATH),
        )
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

#[derive(Debug, Default, PartialEq, Serialize)]
struct Outcome {
    open_tick: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    recovery: Option<withdrawal::Receipt>,
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

#[cfg(test)]
fn run(
    sim: &mut Simulation,
    trial: Trial,
    capture: impl FnMut(&Simulation, usize) -> Result<()>,
) -> Result<Outcome> {
    run_with_delay(sim, trial, 0, capture)
}
fn run_with_delay(
    sim: &mut Simulation,
    trial: Trial,
    delay_ms: usize,
    mut capture: impl FnMut(&Simulation, usize) -> Result<()>,
) -> Result<Outcome> {
    let mut recovery = matches!(trial, Trial::Withdrawal | Trial::OpenLoop)
        .then(|| withdrawal::Recovery::new(trial, delay_ms));
    let steps = if recovery.is_some() { 20_000 } else { STEPS };
    let parcel = part(sim, "parcel::box")?;
    let palm = part(sim, "gripper::palm")?;
    let spindle = part(sim, "gripper::spindle")?;
    let wrist = part(sim, "gripper::wrist")?;
    let jaw_joints = [
        part(sim, "gripper::close_left")?.id,
        part(sim, "gripper::close_right")?.id,
    ];
    let load = body(sim, &parcel)?;
    let shelf = part(sim, "receiver::shelf")?;
    let id = |name| part(sim, name).and_then(|p| body(sim, &p));
    let peers = [
        body(sim, &shelf)?,
        id("stand::post")?,
        id("gripper::left")?,
        id("gripper::right")?,
    ];
    let mut out = Outcome::default();
    let criteria = SupportCriteria::for_weight(WEIGHT_N, std::time::Duration::from_millis(30));
    let mut receiver_sensor = ReceivingSupport::new(sim.world(), parcel.id, shelf.id, criteria)?;
    let mut tray_sensor = ReceivingSupport::new(
        sim.world(),
        parcel.id,
        part(sim, "stand::post")?.id,
        criteria,
    )?;
    let mut contact_hold = None;
    let start_z = sim
        .world()
        .get::<WorldPose>(parcel.entity)
        .ok_or("missing parcel pose")?
        .translation[2];
    let dt = sim.step_duration().as_secs_f64();
    capture(sim, 0)?;
    for tick in 1..=steps {
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
        let solver = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("multifidelity solver required")?;
        let mut normal = [0.; 4];
        for (j, peer) in normal.iter_mut().zip(peers) {
            *j = solver
                .normal_reaction_on(load, peer)?
                .ok_or("no fresh signed contact measurement")?
                .magnitude;
        }
        let [shelf_read, tray_read] = if tick >= DESCENT_START {
            [
                receiver_sensor.sample(sim.world())?,
                tray_sensor.sample(sim.world())?,
            ]
        } else {
            [SupportReading::default(); 2]
        };
        let top = [shelf_read.face_loaded, tray_read.face_loaded];
        if tick == 3600 {
            out.lift_mm = (pose.translation[2] - start_z) * 1000.;
        }
        let z = pose.translation[2];
        let parcel_orientation = pose.rotation;
        let shelf_n = normal[0] / dt;
        let force_n = shelf_read.normal_force_n + tray_read.normal_force_n;
        if tick >= DESCENT_START {
            out.peak_wrong_tray_n = out.peak_wrong_tray_n.max(normal[1] / dt);
        }
        let may_open = out.open_tick.is_none()
            || recovery
                .as_ref()
                .is_some_and(withdrawal::Recovery::retrying);
        if tick >= DESCENT_START
            && trial != Trial::Early
            && may_open
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
        let supported = shelf_read.mean_force_n.filter(|_| shelf_read.ready);
        out.qualified_wrong_tray_ticks += usize::from(tray_read.ready);
        if may_open && ((trial == Trial::Early && tick == 4450) || supported.is_some()) {
            let retry = out.open_tick.is_some();
            if !retry {
                out.open_tick = Some(tick);
                out.release_z_m = z;
                out.shelf_force_at_open_n = shelf_n;
                out.shelf_mean_before_open_n = supported;
            }
            let q = sim
                .world()
                .get::<JointSample>(spindle.entity)
                .ok_or("missing spindle state")?
                .state
                .position;
            set_joint_controller_command(sim, spindle.id, q)?;
            if recovery.is_none() || retry {
                JointForceCommand::set_batch(sim, jaw_joints.map(|jaw| (jaw, -20.)))?;
            }
            if retry {
                recovery.as_mut().unwrap().receipt.final_open_tick = Some(tick);
            }
            contact_hold = None;
        }
        if let Some(at) = out.open_tick {
            if let Some(controller) = &mut recovery {
                let shelf_pose = sim
                    .world()
                    .get::<WorldPose>(shelf.entity)
                    .ok_or("missing receiver pose")?
                    .translation;
                let shelf_velocity = sim
                    .world()
                    .get::<Velocity>(shelf.entity)
                    .ok_or("missing receiver velocity")?
                    .linear;
                let local_up =
                    glam::DQuat::from_array(parcel_orientation).inverse() * glam::DVec3::Z;
                let bottom_z = z - local_up.abs().dot(glam::DVec3::new(0.030, 0.0275, 0.035));
                let sample = withdrawal::GraspSample {
                    drop_mm: (out.release_z_m - z) * 1000.,
                    vertical_speed: vz,
                    bottom_z,
                    normal,
                    spindle_rad: sim
                        .world()
                        .get::<JointSample>(spindle.entity)
                        .ok_or("missing spindle state")?
                        .state
                        .position,
                };
                controller.observe(tick, at, &sample, (shelf_pose, shelf_velocity));
                controller.commands(sim, tick, at, jaw_joints, spindle.id, &sample)?;
            } else if tick == at + 350 {
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
                out.flight_after_open_ms += usize::from(normal.iter().all(|&j| j == 0.));
            }
        }
        if tick > steps - FINAL_SAMPLES {
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
    out.recovery = recovery.map(|r| r.receipt);
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
    let outcome = run_with_delay(
        &mut sim,
        args.trial,
        usize::from(args.delay_ms),
        |sim, _| {
            frames.push(gz_sim_cli::primitive_frame(
                sim,
                &sdf,
                "default",
                frames.len() as u64 + 1,
            )?);
            Ok(())
        },
    )?;
    gz_sim_cli::export_archive(&args.output, Default::default(), frames)?;
    println!("{}", serde_json::to_string_pretty(&outcome)?);
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    fn withdrawal_run(trial: Trial, delay: usize, repeat: bool) -> Outcome {
        let mut sim = load_world_with_sdf_step(
            &scene(trial),
            "withdrawal.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        let outcome = run_with_delay(&mut sim, trial, delay, |_, _| Ok(())).unwrap();
        if repeat {
            gz_sim_scene::reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert_eq!(
                run_with_delay(&mut sim, trial, delay, |_, _| Ok(())).unwrap(),
                outcome
            );
        }
        outcome
    }

    #[test]
    fn withdrawal_response_controls_whether_the_parcel_stays_gripped_and_reaches_the_receiving_shelf()
     {
        std::thread::scope(|scope| {
            for (trial, delay) in [
                (Trial::Withdrawal, 0),
                (Trial::Withdrawal, 50),
                (Trial::OpenLoop, 0),
            ] {
                scope.spawn(move || {
                    let outcome = withdrawal_run(trial, delay, true);
                    let r = outcome.recovery.unwrap();
                    assert_eq!(outcome.open_tick, Some(9216));
                    assert_eq!(r.loss_tick, Some(9226));
                    assert_eq!(r.shelf_return_tick, Some(10900));
                    if trial == Trial::OpenLoop {
                        assert!(r.close_tick.is_none() && r.final_open_tick.is_none());
                        assert!(
                            r.max_drop_before_return_mm > 85.
                                && r.unsupported_before_return_ms > 200
                        );
                        assert!(
                            outcome.final_shelf_load_n == 0.
                                && (outcome.final_tray_load_n - WEIGHT_N).abs() < 0.002
                        );
                    } else {
                        assert_eq!(r.close_tick, Some(9226 + delay));
                        assert_eq!(r.retry_tick, r.shelf_return_tick);
                        assert!(r.held_clear_at_return && r.suspended_on_pads_ms > 900);
                        assert!(r.final_open_tick.is_some_and(|t| t > 10900));
                        assert!(
                            outcome.final_pad_load_n == 0.
                                && (outcome.final_shelf_load_n - WEIGHT_N).abs() < 0.002
                        );
                        if delay == 0 {
                            assert!(
                                r.max_drop_before_return_mm < 0.1
                                    && r.unsupported_before_return_ms == 0
                            );
                        } else {
                            assert!(
                                (5.2..5.5).contains(&r.max_drop_before_return_mm)
                                    && r.unsupported_before_return_ms == 25
                            );
                        }
                    }
                });
            }
        });
    }

    #[test]
    fn adjacent_response_delays_bound_continuity_sag_and_clearance_for_this_fixture() {
        std::thread::scope(|scope| {
            for delays in [[27, 28], [35, 36], [237, 238]] {
                scope.spawn(move || {
                    let [a, b] = delays.map(|d| withdrawal_run(Trial::Withdrawal, d, false));
                    let (ra, rb) = (a.recovery.unwrap(), b.recovery.unwrap());
                    match delays[0] {
                        27 => {
                            assert_eq!(ra.unsupported_before_return_ms, 0);
                            assert_eq!(rb.unsupported_before_return_ms, 1);
                        }
                        35 => {
                            assert!(
                                ra.max_drop_before_return_mm < 1.
                                    && rb.max_drop_before_return_mm > 1.
                            );
                        }
                        _ => {
                            assert!(ra.held_clear_at_return && ra.final_open_tick.is_some());
                            assert!(ra.suspended_on_pads_ms > 900 && rb.suspended_on_pads_ms > 900);
                            assert!(!rb.held_clear_at_return && rb.final_open_tick.is_none());
                            assert!(b.final_pad_load_n > 10.);
                        }
                    }
                });
            }
        });
    }

    #[test]
    #[ignore = "sampled release timing qualification"]
    fn withdrawal_timing_sweep() {
        let delays = [
            0, 20, 26, 27, 28, 34, 35, 36, 50, 70, 72, 73, 100, 150, 220, 224, 227, 228, 232, 235,
            236, 237, 238, 256, 257, 258, 259, 260, 300,
        ];
        std::thread::scope(|scope| {
            for batch in delays.chunks(6) {
                scope.spawn(move || {
                    for &delay in batch {
                        let r = withdrawal_run(Trial::Withdrawal, delay, false);
                        println!("delay_ms={delay} {}", serde_json::to_string(&r).unwrap());
                    }
                });
            }
        });
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
                Trial::Withdrawal | Trial::OpenLoop => unreachable!(),
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
