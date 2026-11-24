// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a parked screw-and-wrist gripper with the continuously solved robot.
#![allow(deprecated)]
use std::{
    error::Error,
    num::NonZeroUsize,
    path::{Path, PathBuf},
    time::Instant,
};

use bevy_ecs::entity::Entity;
use bevy_tasks::{ComputeTaskPool, TaskPoolBuilder};
use clap::Parser;
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::{ArticulationTier as Tier, MultiFidelityBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, JointForceCommand, JointSample, Name, PhysicsEngine, UniversalJointSample,
    Velocity, WorldPose, load_world_with_sdf_step, model_fidelity, queue_universal_joint_effort,
    set_model_fidelity,
};
use serde::Serialize;
use serde_json::json;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const XML: &str = include_str!("../../../examples/worlds/local_robot_parked_spindle.sdf");
const CAPPED: &str = include_str!("../../../examples/worlds/local_robot_capped_spindle.sdf");
const STEPS: usize = 10_000;

#[derive(Clone, Copy)]
struct Part {
    entity: Entity,
    id: u64,
}
fn part(sim: &Simulation, name: &str) -> Part {
    let full = format!("default::{name}");
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == full))
        .expect("authored entity");
    Part {
        entity: e.id(),
        id: e.get::<EntityId>().unwrap().get(),
    }
}
struct Robot {
    model: u64,
    inputs: [u64; 3],
    scalar: [Entity; 3],
    wrist: Part,
    links: [Entity; 5],
    parcel: BodyId,
    contacts: [BodyId; 4],
}
impl Robot {
    fn new(sim: &Simulation) -> Self {
        let names = [
            "gripper::nut",
            "gripper::palm",
            "gripper::left",
            "gripper::right",
            "parcel::box",
        ];
        let links = names.map(|n| part(sim, n).entity);
        let body = |e| sim.world().get::<BodyHandle>(e).unwrap().id();
        Self {
            model: part(sim, "gripper").id,
            inputs: [
                "gripper::close_left",
                "gripper::close_right",
                "gripper::spindle",
            ]
            .map(|s| part(sim, s).id),
            scalar: [
                "gripper::spindle",
                "gripper::close_left",
                "gripper::close_right",
            ]
            .map(|name| part(sim, name).entity),
            wrist: part(sim, "gripper::wrist"),
            links,
            parcel: body(links[4]),
            contacts: [
                links[2],
                links[3],
                part(sim, "stand::post").entity,
                part(sim, "receiver::plate").entity,
            ]
            .map(body),
        }
    }
    fn command(&self, sim: &mut Simulation, targets: [f64; 3]) -> Result<()> {
        JointForceCommand::set_batch(
            sim,
            std::array::from_fn::<_, 3, _>(|i| (self.inputs[i], targets[i])),
        )?;
        Ok(())
    }
    fn observe(&self, sim: &Simulation) -> Result<State> {
        let w = sim.world();
        let native = w
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let mut impulses = [0.; 4];
        for (v, other) in impulses.iter_mut().zip(self.contacts) {
            *v = native
                .normal_reaction_on(self.parcel, other)?
                .unwrap()
                .magnitude;
        }
        let wrist = w
            .get::<UniversalJointSample>(self.wrist.entity)
            .unwrap()
            .state;
        let [spindle, left, right] = self.scalar.map(|e| w.get::<JointSample>(e).unwrap().state);
        Ok(State {
            pose: self.links.map(|e| *w.get::<WorldPose>(e).unwrap()),
            twist: self.links.map(|e| *w.get::<Velocity>(e).unwrap()),
            impulses,
            q: [
                spindle.position,
                wrist.position[0],
                wrist.position[1],
                left.position,
                right.position,
            ],
            dq: [
                spindle.velocity,
                wrist.velocity[0],
                wrist.velocity[1],
                left.velocity,
                right.velocity,
            ],
        })
    }
}
#[derive(Debug, Clone, Copy)]
struct State {
    pose: [WorldPose; 5],
    twist: [Velocity; 5],
    impulses: [f64; 4],
    q: [f64; 5],
    dq: [f64; 5],
}
#[derive(Debug, Default, PartialEq, Serialize)]
struct Trial {
    parked_ticks: usize,
    park_at: Vec<usize>,
    wake_at: Vec<usize>,
    first_jaw: [Option<(usize, f64)>; 2],
    held_impulses_ns: [f64; 4],
    loaded_refusal: Option<String>,
    retained_command: Option<[f64; 3]>,
    lift_m: f64,
    release_support_ns: f64,
    final_support_n: f64,
    peak_abs_speed_rad_s: [f64; 2],
    carried_peak_abs_speed_rad_s: [f64; 2],
    peak_wrist_rad: f64,
    carried_ticks: usize,
}
fn load(xml: &str) -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        xml,
        "parked_spindle.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
fn run(
    sim: &mut Simulation,
    mut capture: impl FnMut(&Simulation) -> Result<()>,
    switched: bool,
) -> Result<(Trial, Vec<State>, Vec<u64>)> {
    let r = Robot::new(sim);
    let mut out = Trial::default();
    let mut states = Vec::with_capacity(STEPS + 1);
    let mut costs = Vec::with_capacity(STEPS);
    states.push(r.observe(sim)?);
    capture(sim)?;
    let mut targets = [0.; 3];
    let mut prior = Tier::Dynamic;
    for tick in 1..=STEPS {
        if switched && (tick == 1001 || tick == 9001) {
            set_model_fidelity(sim, r.model, Tier::Prescribed)?;
            out.park_at.push(tick - 1);
            prior = Tier::Prescribed;
        }
        match tick {
            1251 => targets = [12., 12., 0.],
            1701 => targets[2] = 2.,
            4501 => targets[2] = 0.7,
            7001 => targets[2] = 0.,
            8351 => targets = [0.; 3],
            _ => {}
        }
        if [1251, 1701, 4501, 7001, 8351].contains(&tick) {
            r.command(sim, targets)?;
        }
        if (3801..=3830).contains(&tick) || tick == 9501 {
            queue_universal_joint_effort(sim, r.wrist.id, [0., -0.45])?;
        }
        if tick == 4001 {
            out.held_impulses_ns = states.last().unwrap().impulses;
            if switched {
                let blocked = set_model_fidelity(sim, r.model, Tier::Prescribed);
                if !matches!(blocked, Err(gz_sim_scene::FidelityError::PendingCommand)) {
                    return Err("loaded robot lost its public command guard".into());
                }
                let retained = r.inputs.map(|id| {
                    let entity = gz_sim_core::entity_by_id(sim.world(), id).unwrap();
                    sim.world()
                        .get::<JointForceCommand>(entity)
                        .unwrap()
                        .torque()
                });
                out.retained_command = Some(retained);
                // Remove only the command guard at this same boundary. The loaded
                // contact manifolds must independently protect the free parcel.
                r.command(sim, [0.; 3])?;
                out.loaded_refusal = Some(
                    set_model_fidelity(sim, r.model, Tier::Prescribed)
                        .err()
                        .ok_or("loaded robot unexpectedly parked")?
                        .to_string(),
                );
                let previous = states.last().unwrap();
                let after = r.observe(sim)?;
                if after.q != previous.q
                    || after.dq != previous.dq
                    || after.impulses != previous.impulses
                {
                    return Err("refused transition changed coordinates or solver impulses".into());
                }
                r.command(sim, retained)?;
            }
        }
        let start = Instant::now();
        sim.step_exact(1)?;
        costs.push(start.elapsed().as_nanos() as u64);
        let state = r.observe(sim)?;
        let carried =
            state.impulses[0] > 0. && state.impulses[1] > 0. && state.impulses[2..] == [0.; 2];
        for (i, rate) in [state.dq[0], state.dq[2]].into_iter().enumerate() {
            out.peak_abs_speed_rad_s[i] = out.peak_abs_speed_rad_s[i].max(rate.abs());
            if carried {
                out.carried_peak_abs_speed_rad_s[i] =
                    out.carried_peak_abs_speed_rad_s[i].max(rate.abs());
            }
        }
        let tier = model_fidelity(sim, r.model)?;
        if prior == Tier::Prescribed && tier == Tier::Dynamic {
            out.wake_at.push(tick);
        }
        out.parked_ticks += usize::from(tier == Tier::Prescribed);
        prior = tier;
        for (f, j) in out.first_jaw.iter_mut().zip(state.impulses) {
            if f.is_none() && j > 0. {
                *f = Some((tick, j));
            }
        }
        out.lift_m = out
            .lift_m
            .max(state.pose[4].translation[2] - states[0].pose[4].translation[2]);
        if (3801..=4250).contains(&tick) {
            out.peak_wrist_rad = out.peak_wrist_rad.max(state.q[2].abs());
        }
        if (3201..=4250).contains(&tick) && carried {
            out.carried_ticks += 1;
        }
        if tick == 8350 {
            out.release_support_ns = state.impulses[3];
        }
        if tick > STEPS - 500 {
            out.final_support_n += state.impulses[3] * 2.;
        }
        states.push(state);
        if tick % 40 == 0 {
            capture(sim)?;
        }
    }
    Ok((out, states, costs))
}
fn p95(values: &[u64]) -> u64 {
    let mut v = values.to_vec();
    v.sort_unstable();
    v[(v.len() * 95).div_ceil(100) - 1]
}
fn record(output: &Path, xml: &str) -> Result<serde_json::Value> {
    let sdf = sdformat::load_str(xml, "parked_spindle.sdf", &sdformat::NoResources)?;
    let one = |switched| -> Result<_> {
        let started = Instant::now();
        let mut sim = load(xml)?;
        let mut frames = vec![];
        let trial = run(
            &mut sim,
            |s| {
                frames.push(gz_sim_cli::primitive_frame(
                    s,
                    &sdf,
                    "default",
                    frames.len() as u64 + 1,
                )?);
                Ok(())
            },
            switched,
        )?;
        gz_sim_cli::export_archive(
            &output.join(if switched { "switched" } else { "dynamic" }),
            Default::default(),
            frames,
        )?;
        Ok((
            trial,
            sim.elapsed().as_secs_f64(),
            started.elapsed().as_secs_f64(),
        ))
    };
    let ((reference, a, ca), sim_s, dynamic_wall_s) = one(false)?;
    let ((switched, b, cb), switched_s, switched_wall_s) = one(true)?;
    assert_eq!(sim_s, switched_s);
    let mut inertia = Vec::new();
    for (model_name, link_name) in [
        ("gripper", "nut"),
        ("gripper", "palm"),
        ("gripper", "left"),
        ("gripper", "right"),
        ("parcel", "box"),
    ] {
        let link = sdf.worlds[0]
            .models
            .iter()
            .find(|m| m.name == model_name)
            .unwrap()
            .links
            .iter()
            .find(|l| l.name == link_name)
            .unwrap();
        let [ix, iy, iz, _, _, _] = link.inertial.inertia;
        inertia.push((link.inertial.mass, glam::DVec3::new(ix, iy, iz)));
    }
    let momentum_energy = |s: &State| {
        use glam::{DQuat, DVec3};
        let (mut linear, mut angular, mut energy) = (DVec3::ZERO, DVec3::ZERO, 0.);
        for ((pose, twist), &(m, diagonal)) in s.pose.iter().zip(s.twist).zip(&inertia) {
            let (v, w, x) = (
                DVec3::from(twist.linear),
                DVec3::from(twist.angular),
                DVec3::from(pose.translation),
            );
            let rot = DQuat::from_array(pose.rotation);
            let spin = rot * (diagonal * (rot.inverse() * w));
            linear += m * v;
            angular += x.cross(m * v) + spin;
            energy += 0.5 * (m * v.length_squared() + spin.dot(w)) + m * 9.81 * x.z;
        }
        (linear, angular, energy)
    };
    let (mut dp, mut dl, mut de) = (0_f64, 0_f64, 0_f64);
    let (mut rot, mut spin) = (0_f64, 0_f64);
    let (mut q, mut dq) = ([0_f64; 5], [0_f64; 5]);
    let mut dx = 0_f64;
    let mut dv = 0_f64;
    let mut dj = [0_f64; 4];
    for (a, b) in a.iter().zip(&b) {
        let (pa, la, ea) = momentum_energy(a);
        let (pb, lb, eb) = momentum_energy(b);
        dp = dp.max(pa.distance(pb));
        dl = dl.max(la.distance(lb));
        de = de.max((ea - eb).abs());
        for i in 0..5 {
            q[i] = q[i].max((a.q[i] - b.q[i]).abs());
            dq[i] = dq[i].max((a.dq[i] - b.dq[i]).abs());
        }
        for (a, b) in a.pose.iter().zip(b.pose) {
            rot = rot.max(
                glam::DQuat::from_array(a.rotation)
                    .angle_between(glam::DQuat::from_array(b.rotation)),
            );
            dx = dx.max(glam::DVec3::from(a.translation).distance(b.translation.into()));
        }
        for (a, b) in a.twist.iter().zip(b.twist) {
            spin = spin.max(glam::DVec3::from(a.angular).distance(b.angular.into()));
            dv = dv.max(glam::DVec3::from(a.linear).distance(b.linear.into()));
        }
        for (i, d) in dj.iter_mut().enumerate() {
            *d = d.max((a.impulses[i] - b.impulses[i]).abs());
        }
    }
    let parked_cost = |v: &[u64]| p95(&[&v[1000..1250], &v[9000..9500]].concat());
    Ok(json!({
        "dynamic":reference,"switched":switched,
        "simulated_seconds":sim_s,"public_steps":STEPS,
        "full_step_wall_ns":[ca.iter().sum::<u64>(),cb.iter().sum::<u64>()],
        "load_run_record_wall_seconds":[dynamic_wall_s,switched_wall_s],
        "max_all_link_position_error_m":dx,"max_all_link_velocity_error_m_s":dv,
        "max_all_link_rotation_error_rad":rot,"max_all_link_angular_velocity_error_rad_s":spin,
        "max_coordinate_error":q,"max_coordinate_rate_error":dq,
        "max_linear_momentum_error_kg_m_s":dp,"max_angular_momentum_error_kg_m2_s":dl,
        "max_kinetic_plus_gravitational_energy_error_j":de,"max_impulse_error_ns":dj,
        "full_step_p95_ns":[p95(&ca),p95(&cb)],"park_window_p95_ns":[parked_cost(&ca),parked_cost(&cb)]
    }))
}
#[derive(Parser)]
#[command(about)]
struct Args {
    /// Fresh output directory for the reference, switched recording and report.
    output: PathBuf,
    /// Use the screw and universal joint with authored speed limits.
    #[arg(long)]
    capped: bool,
    /// Number of Bevy compute threads. Omit to use Bevy's usual pool.
    #[arg(long)]
    threads: Option<NonZeroUsize>,
}

fn main() -> Result<()> {
    let args = Args::parse();
    if let Some(count) = args.threads {
        ComputeTaskPool::get_or_init(|| TaskPoolBuilder::new().num_threads(count.get()).build());
    }
    let path = &args.output;
    std::fs::create_dir(path)?;
    let report =
        serde_json::to_string_pretty(&record(path, if args.capped { CAPPED } else { XML })?)?;
    std::fs::write(path.join("comparison.json"), &report)?;
    println!("{report}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parking_keeps_loaded_contacts_and_reset_reproduces_the_entire_cycle() -> Result<()> {
        // The speed-limited wrist can travel at most 0.0066 rad during its 30 ms pulse.
        for (xml, minimum_wrist_excursion) in [(XML, 0.009), (CAPPED, 0.006)] {
            verify_reset_cycle(xml, minimum_wrist_excursion)?;
        }
        Ok(())
    }

    fn verify_reset_cycle(xml: &str, minimum_wrist_excursion: f64) -> Result<()> {
        let mut sim = load(xml)?;
        let mut before = Vec::new();
        for pass in 0..2 {
            let (trial, states, _) = run(&mut sim, |_| Ok(()), true)?;
            assert_eq!(trial.park_at, [1000, 9000]);
            assert_eq!(trial.wake_at, [1251, 9501]);
            assert_eq!(trial.parked_ticks, 750);
            assert_eq!(trial.carried_ticks, 1050);
            assert_eq!(trial.retained_command, Some([12., 12., 2.]));
            assert!(trial.loaded_refusal.unwrap().contains("external contact"));
            assert!(trial.lift_m > 0.12 && trial.peak_wrist_rad > minimum_wrist_excursion);
            assert!(trial.held_impulses_ns[0..2].iter().all(|j| *j > 0.009));
            assert_eq!(trial.held_impulses_ns[2..], [0.; 2]);
            assert!(
                trial.release_support_ns > 0.0019 && (trial.final_support_n - 1.962).abs() < 0.001
            );
            if pass == 0 {
                before = states;
                gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
            } else {
                for (a, b) in before.iter().zip(states) {
                    assert_eq!(a.impulses, b.impulses);
                    assert_eq!(a.q, b.q);
                    assert_eq!(a.dq, b.dq);
                    for (a, b) in a.pose.iter().zip(b.pose) {
                        assert_eq!(a.translation, b.translation);
                        assert_eq!(a.rotation, b.rotation);
                    }
                    for (a, b) in a.twist.iter().zip(b.twist) {
                        assert_eq!(a.linear, b.linear);
                        assert_eq!(a.angular, b.angular);
                    }
                }
            }
        }
        Ok(())
    }

    #[test]
    fn both_speed_stops_limit_the_loaded_robot_after_each_wake() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let report = record(dir.path(), CAPPED)?;
        for trial in ["dynamic", "switched"] {
            let r = &report[trial];
            assert_eq!(r["carried_ticks"], 1050);
            assert!(r["lift_m"].as_f64().unwrap() > 0.125);
            assert!((r["final_support_n"].as_f64().unwrap() - 1.962).abs() < 1e-4);
            for (index, limit) in [0.60, 0.22].into_iter().enumerate() {
                for key in ["peak_abs_speed_rad_s", "carried_peak_abs_speed_rad_s"] {
                    assert!(
                        (r[key][index].as_f64().unwrap() - limit).abs() < 1e-5,
                        "{key}: {r}"
                    );
                }
            }
        }
        assert_eq!(report["switched"]["parked_ticks"], 750);
        assert_eq!(report["switched"]["wake_at"], json!([1251, 9501]));
        assert!(
            report["switched"]["loaded_refusal"]
                .as_str()
                .unwrap()
                .contains("external contact")
        );
        assert!(report["max_all_link_position_error_m"].as_f64().unwrap() < 1e-5);
        assert!(
            report["max_impulse_error_ns"]
                .as_array()
                .unwrap()
                .iter()
                .all(|v| v.as_f64().unwrap() < 2e-6)
        );
        Ok(())
    }

    #[test]
    fn a_recording_error_stops_on_the_last_measured_public_boundary() -> Result<()> {
        let archive = tempfile::tempdir()?;
        assert!(record(archive.path(), "<sdf").is_err());
        assert!(
            record(
                archive.path(),
                r#"<sdf version="1.12"><world name="other"/></sdf>"#
            )
            .is_err()
        );
        for tick in [0, 40] {
            let mut sim = load(XML)?;
            let error = run(
                &mut sim,
                |s| {
                    if s.elapsed().as_millis() == tick {
                        return Err("archive write failed".into());
                    }
                    Ok(())
                },
                true,
            )
            .unwrap_err();
            assert_eq!(error.to_string(), "archive write failed");
            assert_eq!(sim.elapsed().as_millis(), tick);
        }
        let mut sim = load(XML)?;
        sim.world()
            .resource::<gz_sim_core::StepFault>()
            .report(std::io::Error::other("broken solver"));
        assert!(run(&mut sim, |_| Ok(()), true).is_err());
        assert!(sim.elapsed().is_zero());
        Ok(())
    }

    #[test]
    fn continuously_solved_control_and_native_archive_bound_the_switching_error() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let report = record(dir.path(), XML)?;
        for branch in ["dynamic", "switched"] {
            let r = &report[branch];
            assert_eq!(r["first_jaw"][0][0], 1274);
            assert_eq!(r["first_jaw"][1][0], 1274);
            assert_eq!(r["carried_ticks"], 1050);
            assert!(r["lift_m"].as_f64().unwrap() > 0.12);
        }
        for i in 0..2 {
            let first = |branch: &str| report[branch]["first_jaw"][i][1].as_f64().unwrap();
            assert!((first("switched") - first("dynamic")).abs() < 0.00002);
        }
        assert!(report["max_all_link_position_error_m"].as_f64().unwrap() < 0.001);
        assert!(report["max_all_link_velocity_error_m_s"].as_f64().unwrap() < 0.08);
        Ok(())
    }
}
