// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Carry a free parcel on an actuated rail to a fixed receiving shelf.
#![allow(deprecated)]
use std::{error::Error, path::PathBuf, time::Duration};

use bevy_ecs::entity::Entity;
use clap::{Parser, ValueEnum};
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::receiving_support::{ReceivingSupport, SupportCriteria};
use gz_sim_scene::{
    BodyHandle, JointForceCommand, JointSample, Name, PhysicsEngine, WorldPose,
    load_world_with_sdf_step, set_joint_controller_command,
};
use serde::Serialize;

#[path = "serial_transfer/carry.rs"]
mod carry;
#[path = "serial_transfer/cycle.rs"]
mod cycle;
#[path = "serial_transfer/obstacle.rs"]
mod obstacle;
#[path = "serial_transfer/payload.rs"]
mod payload;
#[path = "serial_transfer/pull.rs"]
mod pull;
#[path = "support/rigid_tactile.rs"]
mod rigid_tactile;
use obstacle::{Passage, PassageOutcome};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const XML: &str = include_str!("../../../examples/worlds/local_robot_serial_transfer.sdf");
#[cfg(test)]
const WEIGHT: f64 = 0.2 * 9.81;
const MOVE_START: usize = 3700;
const MOVE_END: usize = 7700;
const LOWER: usize = 8100;
const STEPS: usize = 18000;
const SUPPORT_DWELL: Duration = Duration::from_millis(30);

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum Trial {
    Transfer,
    OpenJaws,
    MissedShelf,
    ClearObstacle,
    GuardedCarry,
    ObserveCarry,
    /// Tighten on slip and use the original direct approach for comparison.
    DirectCarry,
    /// Keep the native recycled contact geometry as a comparison control.
    CachedObstacle,
    LowClearance,
    ContactStop,
}
impl Trial {
    fn carry(self) -> bool {
        matches!(
            self,
            Self::GuardedCarry | Self::ObserveCarry | Self::DirectCarry
        )
    }
    fn obstacle(self) -> bool {
        self.carry()
            || matches!(
                self,
                Self::ClearObstacle | Self::CachedObstacle | Self::LowClearance | Self::ContactStop
            )
    }
    fn xml(self) -> &'static str {
        if self.obstacle() {
            include_str!("../../../examples/worlds/local_robot_serial_obstacle.sdf")
        } else {
            XML
        }
    }
}
#[derive(Parser)]
struct Args {
    #[arg(value_enum)]
    trial: Trial,
    output: PathBuf,
    /// Load a self-contained serial-transfer workcell with the same named parts.
    #[arg(long)]
    world: Option<PathBuf>,
    /// Parcel mass in kg. Scales the authored inertia tensor at fixed geometry.
    #[arg(long)]
    parcel_kg: Option<f64>,
    /// ODE sliding friction on the parcel and both finger pads.
    #[arg(long)]
    grip_friction: Option<f64>,
    /// Time for the 460 mm quintic command in the monitored carry trials.
    #[arg(long, default_value_t = 4000, value_parser=clap::value_parser!(u64).range(1..=4000))]
    travel_ms: u64,
    /// Interval for outer rail, grip, support and recovery commands in milliseconds.
    #[arg(long,default_value_t=1,value_parser=clap::value_parser!(u64).range(1..=2))]
    control_ms: u64,
    /// Optional downward force at the parcel center of mass, given as first_ms,duration_ms,force_N.
    #[arg(long)]
    pull: Option<pull::Pull>,
}

#[derive(Clone, Copy)]
struct Part {
    entity: Entity,
    id: u64,
}
fn part(sim: &Simulation, name: &str) -> Result<Part> {
    let full = format!("default::{name}");
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == full))
        .ok_or_else(|| format!("missing {full}"))?;
    Ok(Part {
        entity: e.id(),
        id: e.get::<EntityId>().ok_or("unidentified part")?.get(),
    })
}
impl Part {
    fn pose(self, sim: &Simulation) -> Result<WorldPose> {
        Ok(*sim
            .world()
            .get::<WorldPose>(self.entity)
            .ok_or("missing world pose")?)
    }
    fn body(self, sim: &Simulation) -> Result<BodyId> {
        Ok(sim
            .world()
            .get::<BodyHandle>(self.entity)
            .ok_or("missing rigid body")?
            .id())
    }
}
#[derive(Debug, Default, PartialEq, Serialize)]
struct Outcome {
    #[serde(skip_serializing_if = "Option::is_none")]
    obstacle: Option<PassageOutcome>,
    #[serde(skip_serializing_if = "Option::is_none")]
    payload: Option<payload::Loads>,
    #[serde(skip_serializing_if = "Option::is_none")]
    carry: Option<carry::Measurements>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pull: Option<pull::Measurements>,
    lift_mm: f64,
    lateral_payload_at_arrival_mm: f64,
    lateral_carriage_at_arrival_mm: f64,
    parcel_carried_on_both_pads_ticks: usize,
    traverse_ticks: usize,
    max_parcel_motion_in_palm_frame_during_traverse_mm: f64,
    shelf_unloaded_ticks_after_release: usize,
    max_drop_after_release_mm: f64,
    pickup_ready_ticks: usize,
    release_tick: Option<usize>,
    dwell_ms_at_release: Option<u128>,
    shelf_mean_at_release_n: Option<f64>,
    shelf_relative_speed_at_release_m_s: Option<f64>,
    final_shelf_load_n: f64,
    final_tray_load_n: f64,
    final_pad_load_n: f64,
    final_parcel_position_m: [f64; 3],
    final_palm_position_m: [f64; 3],
}
/// A quintic move reaches each endpoint with zero commanded speed and acceleration.
fn command_rail(sim: &mut Simulation, rail: Part, tick: usize, goal: f64, dt: f64) -> Result<()> {
    drive_rail(
        sim,
        rail,
        0.,
        goal,
        tick.saturating_sub(MOVE_START),
        MOVE_END - MOVE_START,
        dt,
    )
}
fn drive_rail(
    sim: &mut Simulation,
    rail: Part,
    from: f64,
    to: f64,
    elapsed: usize,
    duration: usize,
    dt: f64,
) -> Result<()> {
    let t = (elapsed as f64 / duration as f64).min(1.);
    let travel_time = duration as f64 * dt;
    let goal = to - from;
    let target = from + goal * t * t * t * (10. + t * (-15. + 6. * t));
    let speed = goal * 30. * t * t * (1. - t) * (1. - t) / travel_time;
    let state = sim
        .world()
        .get::<JointSample>(rail.entity)
        .ok_or("missing rail feedback")?
        .state;
    let effort =
        (100. * (target - state.position) + 28. * (speed - state.velocity)).clamp(-80., 80.);
    JointForceCommand::set_batch(sim, [(rail.id, effort)])?;
    Ok(())
}

#[cfg(test)]
fn run(
    sim: &mut Simulation,
    trial: Trial,
    capture: impl FnMut(&Simulation) -> Result<()>,
) -> Result<Outcome> {
    drive(sim, trial, None, WEIGHT, false, capture)
}
fn drive(
    sim: &mut Simulation,
    trial: Trial,
    mut passage: Option<Passage>,
    weight: f64,
    measure_payload: bool,
    mut capture: impl FnMut(&Simulation) -> Result<()>,
) -> Result<Outcome> {
    let parcel = part(sim, "parcel::box")?;
    let palm = part(sim, "gripper::palm")?;
    let car = part(sim, "gripper::carriage")?;
    let rail = part(sim, "gripper::travel")?;
    let wrist = part(sim, "gripper::wrist")?;
    let spindle = part(sim, "gripper::spindle")?;
    let jaws = [
        part(sim, "gripper::close_left")?.id,
        part(sim, "gripper::close_right")?.id,
    ];
    let pad_bodies = [
        part(sim, "gripper::left")?.body(sim)?,
        part(sim, "gripper::right")?.body(sim)?,
    ];
    let body = parcel.body(sim)?;
    let shelf = part(sim, "receiver::shelf")?;
    let tray = part(sim, "stand::post")?;
    let c = SupportCriteria::for_weight(weight, SUPPORT_DWELL);
    let receiver = ReceivingSupport::new(sim.world(), parcel.id, shelf.id, c)?;
    let pickup = ReceivingSupport::new(sim.world(), parcel.id, tray.id, c)?;
    let mut carry = if trial.carry() {
        let bar = passage.as_ref().ok_or("missing monitored obstacle")?.body();
        Some(carry::Supervisor::new(
            sim,
            trial,
            weight,
            carry::Links {
                body,
                pads: pad_bodies,
                stations: [tray.body(sim)?, shelf.body(sim)?, bar],
                jaws,
                spindle,
                rail,
            },
        )?)
    } else {
        None
    };
    let mut out = Outcome {
        pull: sim
            .world()
            .get_resource::<pull::Pull>()
            .copied()
            .map(pull::Measurements::new),
        payload: measure_payload.then(|| payload::Loads::new(weight)),
        ..Default::default()
    };
    let initial = parcel.pose(sim)?.translation;
    let start_car = car.pose(sim)?.translation[1];
    let mut start_offset = glam::DVec3::ZERO;
    let mut release_height = 0.;
    let mut hold = None;
    let goal = if trial == Trial::MissedShelf {
        0.18
    } else {
        0.46
    };
    let cadence = sim
        .world()
        .get_resource::<cycle::Cadence>()
        .copied()
        .unwrap_or(cycle::Cadence::ONE_SOLVE);
    let rail_time_unit = if carry.is_some() {
        0.001
    } else {
        sim.step_duration().as_secs_f64()
    };
    let control_ms = cadence.control_ms;
    let mut cycle = cycle::Cycle {
        cadence,
        criteria: c,
        pickup,
        receiver,
        parcel,
        car,
        rail,
        start_car,
        body,
        pads: pad_bodies,
        goal,
        trial,
    };
    let control_dt = rail_time_unit * control_ms as f64;
    if trial == Trial::OpenJaws {
        JointForceCommand::set_batch(sim, jaws.map(|id| (id, -20.)))?;
    }
    capture(sim)?;
    for tick in (control_ms..=STEPS).step_by(control_ms) {
        if tick == 2 && matches!(trial, Trial::LowClearance | Trial::ContactStop) {
            set_joint_controller_command(sim, spindle.id, 0.85)?;
        }
        if tick == 3600 {
            set_joint_controller_command(sim, wrist.id, 0.)?;
        }
        if tick > 1 {
            let goal = if passage.as_ref().is_some_and(Passage::aborted) {
                0.
            } else {
                goal
            };
            if let Some(active) = &carry {
                active.command_rail(sim, rail, tick, goal, rail_time_unit)?;
            } else {
                command_rail(sim, rail, tick, goal, rail_time_unit)?;
            }
        }
        let returning = carry.as_ref().is_some_and(carry::Supervisor::recovering);
        if !returning && tick == LOWER && !passage.as_ref().is_some_and(Passage::aborted) {
            set_joint_controller_command(sim, spindle.id, -0.018)?;
        }
        let reading = cycle.advance(
            sim,
            tick,
            passage.as_mut(),
            carry.as_ref(),
            out.pull.as_mut(),
        )?;
        let source = reading.source;
        let target = reading.target;
        let pad = reading.pad;
        let p = reading.parcel_position;
        let carriage = reading.carriage_y;
        let carried = reading.both_pads_loaded && !reading.face_loaded_any.contains(&true);
        if let Some(route) = &mut passage {
            route.record_grip(tick, carriage - start_car, carried);
        }
        if tick == MOVE_START {
            out.lift_mm = 1000. * (p[2] - initial[2]);
        }
        if (MOVE_START..=MOVE_END).contains(&tick) {
            out.traverse_ticks += 1;
            if let Some(loads) = &mut out.payload {
                loads.observe(tick, &reading, (MOVE_END - MOVE_START) / control_ms + 1);
            }
            out.parcel_carried_on_both_pads_ticks += usize::from(carried);
            let hand = palm.pose(sim)?;
            let local = glam::DQuat::from_array(hand.rotation).inverse()
                * (glam::DVec3::from(p) - glam::DVec3::from(hand.translation));
            if tick == MOVE_START {
                start_offset = local;
            }
            out.max_parcel_motion_in_palm_frame_during_traverse_mm = out
                .max_parcel_motion_in_palm_frame_during_traverse_mm
                .max(1000. * local.distance(start_offset));
        }
        if tick == MOVE_END {
            out.lateral_payload_at_arrival_mm = 1000. * (p[1] - initial[1]);
            out.lateral_carriage_at_arrival_mm = 1000. * (carriage - start_car);
        }
        if let (Some(active), Some(route)) = (&mut carry, &passage) {
            active.update_with_reading(
                sim,
                tick,
                carriage - start_car,
                &reading,
                route.aborted(),
            )?;
            active.verify_return(sim, tick, carriage - start_car, route)?;
            active.check_shelf_edge(sim, tick, carriage - start_car, route, &reading)?;
            if let Some(pull) = &mut out.pull {
                pull.after_alarm(
                    sim,
                    parcel,
                    rail,
                    active.report.first_alarm_ms == Some(tick),
                    active.report.repeated_alarm_ms == Some(tick),
                )?;
            }
        }
        if !carry.as_ref().is_some_and(carry::Supervisor::recovering)
            && tick >= LOWER
            && out.release_tick.is_none()
            && !passage.as_ref().is_some_and(Passage::aborted)
        {
            let q = sim
                .world()
                .get::<JointSample>(spindle.entity)
                .ok_or("missing spindle reading")?
                .state
                .position;
            if trial != Trial::OpenJaws
                && (source.face_loaded || target.face_loaded || hold.is_some())
            {
                let demand = hold.get_or_insert(q);
                *demand += (0.003 * (target.normal_force_n + source.normal_force_n - weight))
                    .clamp(-0.008, 0.008)
                    * control_dt;
                set_joint_controller_command(sim, spindle.id, *demand)?;
            }
            out.pickup_ready_ticks += usize::from(source.ready);
            if target.ready {
                out.release_tick = Some(tick);
                release_height = p[2];
                out.dwell_ms_at_release = Some(target.sustained.as_millis());
                out.shelf_mean_at_release_n = target.mean_force_n;
                out.shelf_relative_speed_at_release_m_s = Some(target.relative_normal_speed_m_s);
                set_joint_controller_command(sim, spindle.id, q)?;
                JointForceCommand::set_batch(sim, jaws.map(|id| (id, -20.)))?;
            }
        }
        if out.release_tick.is_some() {
            out.shelf_unloaded_ticks_after_release += usize::from(!target.face_loaded);
            out.max_drop_after_release_mm = out
                .max_drop_after_release_mm
                .max(1000. * (release_height - p[2]));
        }
        if out.release_tick.is_some_and(|at| tick == at + 250) {
            set_joint_controller_command(sim, spindle.id, 1.6)?;
        }
        if tick > STEPS - 500 {
            out.final_shelf_load_n += target.normal_force_n / (500. / control_ms as f64);
            out.final_tray_load_n += source.normal_force_n / (500. / control_ms as f64);
            out.final_pad_load_n += (pad[0] + pad[1]) / (500. / control_ms as f64);
        }
        let event = passage
            .as_ref()
            .is_some_and(|g| g.outcome.abort_tick == Some(tick) || out.release_tick == Some(tick));
        if tick % 40 == 0
            || event
            || out.pull.as_ref().is_some_and(|p| p.boundary(tick))
            || carry.as_ref().is_some_and(|s| s.event(tick))
        {
            capture(sim)?;
        }
    }
    out.carry = carry.map(|active| active.report);
    out.obstacle = passage.map(|p| p.outcome);
    out.final_parcel_position_m = parcel.pose(sim)?.translation;
    out.final_palm_position_m = palm.pose(sim)?.translation;
    Ok(out)
}
fn record(args: Args, xml: &str) -> Result<Outcome> {
    let xml = payload::configure(xml, args.parcel_kg, args.grip_friction)?;
    let sdf = sdformat::load_str(&xml, "serial_transfer.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        &xml,
        "serial_transfer.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    carry::configure_clock(&mut sim, args.trial, args.travel_ms, args.control_ms)?;
    if let Some(pull) = args.pull {
        pull.install(&mut sim, args.trial)?;
    }
    let mut frames = Vec::new();
    let passage = args
        .trial
        .obstacle()
        .then(|| Passage::bind(&sim, &sdf))
        .transpose()?;
    let weight = payload::weight(&sdf)?;
    let outcome = drive(
        &mut sim,
        args.trial,
        passage,
        weight,
        args.world.is_some() || args.parcel_kg.is_some() || args.grip_friction.is_some(),
        |s| {
            frames.push(gz_sim_cli::primitive_frame(
                s,
                &sdf,
                "default",
                frames.len() as u64 + 1,
            )?);
            Ok(())
        },
    )?;
    gz_sim_cli::export_archive(&args.output, Default::default(), frames)?;
    Ok(outcome)
}
fn main() -> Result<()> {
    let args = Args::parse();
    let xml = args
        .world
        .as_ref()
        .map(std::fs::read_to_string)
        .transpose()?;
    let xml = xml.as_deref().unwrap_or(args.trial.xml());
    println!("{}", serde_json::to_string_pretty(&record(args, xml)?)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    fn load(xml: &str) -> Simulation {
        load_world_with_sdf_step(
            xml,
            "serial_transfer.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap()
    }
    type Poses = BTreeMap<u64, ([u64; 3], [u64; 4])>;
    fn measured(sim: &mut Simulation, trial: Trial) -> (Outcome, Vec<Poses>) {
        let mut frames = Vec::new();
        let shelf = part(sim, "receiver::shelf").unwrap();
        let before = shelf.pose(sim).unwrap();
        let outcome = run(sim, trial, |sim| {
            let parked = shelf.pose(sim)?;
            assert_eq!(
                (parked.translation, parked.rotation),
                (before.translation, before.rotation)
            );
            frames.push(
                sim.world()
                    .iter_entities()
                    .filter_map(|e| {
                        let pose = e.get::<WorldPose>()?;
                        Some((
                            e.get::<EntityId>()?.get(),
                            (
                                pose.translation.map(f64::to_bits),
                                pose.rotation.map(f64::to_bits),
                            ),
                        ))
                    })
                    .collect::<Poses>(),
            );
            Ok(())
        })
        .unwrap();
        assert_eq!(frames.len(), 451);
        assert!(frames[0].len() >= 10);
        (outcome, frames)
    }
    #[test]
    fn commanded_transfer_and_both_controls_repeat_every_recorded_pose_after_reset() {
        std::thread::scope(|scope| {
            for trial in [Trial::Transfer, Trial::OpenJaws, Trial::MissedShelf] {
                scope.spawn(move || {
                    let mut sim = load(XML);
                    let first = measured(&mut sim, trial);
                    gz_sim_scene::reset_loaded_world(&mut sim, |_| ()).unwrap();
                    assert_eq!(sim.epoch(), 1);
                    assert_eq!(measured(&mut sim, trial), first);
                    let (out, _) = first;
                    let expected = if trial == Trial::MissedShelf {
                        180.
                    } else {
                        460.
                    };
                    assert!((out.lateral_carriage_at_arrival_mm - expected).abs() < 3.);
                    if trial == Trial::OpenJaws {
                        assert!(
                            out.lift_mm.abs() < 0.1
                                && out.lateral_payload_at_arrival_mm.abs() < 0.1
                        );
                        assert_eq!(out.parcel_carried_on_both_pads_ticks, 0);
                        assert!((out.final_tray_load_n - WEIGHT).abs() < 0.001);
                        assert_eq!(out.final_pad_load_n, 0.);
                    } else {
                        assert!((out.lift_mm - 126.07).abs() < 0.2);
                        assert_eq!(
                            (out.parcel_carried_on_both_pads_ticks, out.traverse_ticks),
                            (4001, 4001)
                        );
                        assert!((out.lateral_payload_at_arrival_mm - expected).abs() < 3.);
                        assert!(
                            out.max_parcel_motion_in_palm_frame_during_traverse_mm < 3.0,
                            "{out:?}"
                        );
                    }
                    if trial == Trial::Transfer {
                        assert!(out.release_tick.unwrap() > LOWER);
                        assert_eq!(out.dwell_ms_at_release, Some(30));
                        assert!(out.shelf_mean_at_release_n.unwrap() >= 0.94 * WEIGHT);
                        assert!(out.shelf_relative_speed_at_release_m_s.unwrap().abs() < 0.01);
                        assert_eq!(out.shelf_unloaded_ticks_after_release, 0);
                        assert!(out.max_drop_after_release_mm < 0.5, "{out:?}");
                        assert!((out.final_shelf_load_n - WEIGHT).abs() < 0.001);
                        assert_eq!((out.final_pad_load_n, out.final_tray_load_n), (0., 0.));
                        assert!((out.final_parcel_position_m[1] - 0.46).abs() < 0.001);
                        assert!(
                            out.final_palm_position_m[2] - out.final_parcel_position_m[2] > 0.2
                        );
                    } else {
                        assert!(
                            out.release_tick.is_none() && out.shelf_mean_at_release_n.is_none()
                        );
                        assert_eq!(out.final_shelf_load_n, 0.);
                        assert!(out.pickup_ready_ticks > 1000);
                        if trial == Trial::MissedShelf {
                            assert!(
                                out.final_pad_load_n > 5. && out.final_tray_load_n > 0.94 * WEIGHT
                            );
                            assert!((out.final_parcel_position_m[1] - 0.18).abs() < 0.001);
                        }
                    }
                });
            }
        })
    }
    #[test]
    fn absent_workcell_parts_and_recording_failures_propagate() {
        let mut missing = load(&XML.replace("name=\"shelf\"", "name=\"absent\""));
        assert!(
            run(&mut missing, Trial::Transfer, |_| Ok(()))
                .unwrap_err()
                .to_string()
                .contains("receiver::shelf")
        );
        for at in [0, 40] {
            let mut sim = load(XML);
            let error = run(&mut sim, Trial::Transfer, |s| {
                if s.elapsed() == Duration::from_millis(at) {
                    Err("capture failed".into())
                } else {
                    Ok(())
                }
            })
            .unwrap_err();
            assert_eq!(error.to_string(), "capture failed");
        }
    }
    #[test]
    fn incomplete_loaded_models_report_the_specific_unusable_binding() {
        for name in [
            "parcel::box",
            "gripper::palm",
            "gripper::carriage",
            "gripper::travel",
            "gripper::wrist",
            "gripper::spindle",
            "gripper::close_left",
            "gripper::close_right",
            "gripper::left",
            "gripper::right",
            "stand::post",
        ] {
            let mut sim = load(XML);
            let e = part(&sim, name).unwrap().entity;
            sim.world_mut().entity_mut(e).remove::<Name>();
            let err = run(&mut sim, Trial::Transfer, |_| Ok(())).unwrap_err();
            assert!(err.to_string().contains(name), "{err}");
            assert_eq!(sim.elapsed(), Duration::ZERO);
        }
        for name in [
            "parcel::box",
            "gripper::left",
            "gripper::right",
            "receiver::shelf",
            "stand::post",
        ] {
            let mut sim = load(XML);
            let e = part(&sim, name).unwrap().entity;
            sim.world_mut().entity_mut(e).remove::<BodyHandle>();
            assert!(run(&mut sim, Trial::Transfer, |_| Ok(())).is_err());
            assert_eq!(sim.elapsed(), Duration::ZERO);
        }
        for name in ["parcel::box", "gripper::carriage"] {
            let mut sim = load(XML);
            let e = part(&sim, name).unwrap().entity;
            sim.world_mut().entity_mut(e).remove::<WorldPose>();
            assert_eq!(
                run(&mut sim, Trial::Transfer, |_| Ok(()))
                    .unwrap_err()
                    .to_string(),
                "missing world pose"
            );
        }
        let mut sim = load(XML);
        let e = part(&sim, "parcel::box").unwrap().entity;
        sim.world_mut().entity_mut(e).remove::<EntityId>();
        assert_eq!(
            part(&sim, "parcel::box").err().unwrap().to_string(),
            "unidentified part"
        );
    }
    #[test]
    fn the_rail_needs_measured_state_and_a_valid_drive_and_faults_stop_the_route() {
        let mut sim = load(XML);
        let rail = part(&sim, "gripper::travel").unwrap();
        sim.step_exact(1).unwrap();
        let stale = Part {
            entity: rail.entity,
            id: u64::MAX,
        };
        assert!(command_rail(&mut sim, stale, MOVE_START, 0.46, 0.001).is_err());
        sim.world_mut()
            .entity_mut(rail.entity)
            .remove::<JointSample>();
        assert_eq!(
            command_rail(&mut sim, rail, MOVE_START, 0.46, 0.001)
                .unwrap_err()
                .to_string(),
            "missing rail feedback"
        );
        let mut sim = load(XML);
        sim.world()
            .resource::<gz_sim_core::StepFault>()
            .report(std::io::Error::other("physics stopped"));
        assert!(run(&mut sim, Trial::Transfer, |_| Ok(())).is_err());
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
    #[test]
    fn losing_a_required_observation_or_jaw_aborts_before_an_unmeasured_move() {
        for name in ["parcel::box", "gripper::carriage"] {
            let mut sim = load(XML);
            let entity = part(&sim, name).unwrap().entity;
            sim.add_systems(
                gz_sim_core::Phase::Finalize,
                move |world: &mut bevy_ecs::world::World| {
                    world.entity_mut(entity).remove::<WorldPose>();
                },
            );
            assert_eq!(
                run(&mut sim, Trial::Transfer, |_| Ok(()))
                    .unwrap_err()
                    .to_string(),
                "missing world pose"
            );
            assert_eq!(sim.elapsed(), Duration::from_millis(1));
        }
        let mut sim = load(XML);
        let jaw = part(&sim, "gripper::close_left").unwrap();
        sim.world_mut()
            .entity_mut(jaw.entity)
            .remove::<gz_sim_scene::JointHandle>();
        assert!(run(&mut sim, Trial::OpenJaws, |_| Ok(())).is_err());
        assert_eq!(sim.elapsed(), Duration::ZERO);
    }
    #[test]
    fn capture_rejects_invalid_sdf_and_refuses_to_overwrite_a_recording() {
        let d = tempfile::tempdir().unwrap();
        let path = d.path().join("saved");
        let args = || Args {
            trial: Trial::Transfer,
            output: path.clone(),
            world: None,
            parcel_kg: None,
            grip_friction: None,
            travel_ms: 4000,
            control_ms: 1,
            pull: None,
        };
        assert!(record(args(), "<sdf").is_err());
        assert!(
            record(
                args(),
                &XML.replace("type=\"prismatic\"", "type=\"gearbox\"")
            )
            .is_err()
        );
        assert!(record(args(), &XML.replace("name=\"shelf\"", "name=\"absent\"")).is_err());
        std::fs::write(&path, b"existing").unwrap();
        assert!(record(args(), XML).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), b"existing");
    }
}
