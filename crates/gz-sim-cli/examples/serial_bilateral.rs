// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure both jaw reactions on the same free parcel throughout release and recapture.
#![allow(deprecated)]
use bevy_ecs::prelude::{Res, ResMut, Resource};
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::{ContactSegment, MultiFidelityBackend, RigidContactPatch};
use gz_sim_core::{EntityId, Phase, Simulation, StepFault, UpdateInfo};
use gz_sim_scene::{
    BodyHandle, JointForceCommand, Name, PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions,
};
use serde::Serialize;
use std::{
    borrow::Cow,
    error::Error,
    io::Write,
    path::{Path, PathBuf},
};

#[path = "serial_bilateral/summary.rs"]
mod summary;
#[cfg(test)]
#[path = "serial_bilateral/tests.rs"]
mod tests;

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const TICKS: u32 = 280;
const WORLD: &str = include_str!("../../../examples/worlds/local_robot_serial_bilateral.sdf");
const GRAVITY: DVec3 = DVec3::new(0., 0., -9.81);
const MASS: f64 = 0.2;
const INERTIA: DVec3 = DVec3::new(0.00016, 0.00014, 0.00013);

#[derive(Parser)]
struct Args {
    /// Fixed native contact steps per 1 ms command tick. One uses the ordinary local schedule.
    #[arg(long, default_value_t=1, value_parser=clap::value_parser!(u32).range(1..=2048))]
    substeps: u32,
    /// Also impose a rightward force and pitching couple during the loaded windows.
    #[arg(long)]
    biased: bool,
    /// Keep both jaws open after the release, letting the parcel land on the stand.
    #[arg(long)]
    stay_open: bool,
    /// Substitute flat rectangular pads at the same initial 60 mm opening.
    #[arg(long)]
    flat_pads: bool,
    /// Report held forces, release and recapture times, and impulse balance.
    #[arg(long)]
    summary: bool,
    /// Repeat from a loaded-world reset and compare every physical sample.
    #[arg(long)]
    repeat: bool,
    #[arg(long)]
    replay: Option<PathBuf>,
    #[arg(long)]
    write_world: Option<PathBuf>,
}
#[derive(Clone, Copy)]
struct Parts {
    parcel: BodyId,
    parcel_id: u64,
    tray: BodyId,
    jaws: [BodyId; 2],
    drives: [u64; 2],
}
fn parts(sim: &Simulation) -> Parts {
    let part = |name: &str| {
        let scoped = format!("default::{name}");
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == scoped))
            .unwrap()
    };
    let parcel = part("parcel::box");
    Parts {
        parcel: parcel.get::<BodyHandle>().unwrap().id(),
        parcel_id: parcel.get::<EntityId>().unwrap().get(),
        tray: part("stand::post").get::<BodyHandle>().unwrap().id(),
        jaws: ["gripper::left", "gripper::right"]
            .map(|n| part(n).get::<BodyHandle>().unwrap().id()),
        drives: ["gripper::close_left", "gripper::close_right"]
            .map(|n| part(n).get::<EntityId>().unwrap().get()),
    }
}
#[derive(Clone, Default, Debug, PartialEq, Serialize)]
struct Reaction {
    normal_ns: f64,
    impulse_ns: [f64; 3],
    angular_nms: [f64; 3],
}
#[derive(Clone, Default, Debug, PartialEq, Resource, Serialize)]
struct Contacts {
    left: Reaction,
    right: Reaction,
    tray: Reaction,
    other: Reaction,
}
impl Contacts {
    fn add(&mut self, patches: Vec<RigidContactPatch>, p: Parts) {
        for patch in patches {
            let (peer, sign) = if patch.a == p.parcel {
                (patch.b, 1.)
            } else if patch.b == p.parcel {
                (patch.a, -1.)
            } else {
                continue;
            };
            let reaction = if peer == p.jaws[0] {
                &mut self.left
            } else if peer == p.jaws[1] {
                &mut self.right
            } else if peer == p.tray {
                &mut self.tray
            } else {
                &mut self.other
            };
            for pt in patch.points {
                let tangent = pt
                    .tangent_impulse_on_a
                    .expect("study selects per-point Coulomb friction");
                let j = sign
                    * (DVec3::from(patch.normal_on_a) * pt.normal_impulse + DVec3::from(tangent));
                let lever = if sign > 0. { pt.lever_a } else { pt.lever_b };
                reaction.normal_ns += pt.normal_impulse;
                reaction.impulse_ns = (DVec3::from(reaction.impulse_ns) + j).into();
                reaction.angular_nms =
                    (DVec3::from(reaction.angular_nms) + DVec3::from(lever).cross(j)).into();
            }
        }
    }
}
fn measured_schedule(sim: &mut Simulation, n: u32) -> Result<()> {
    if n == 1 {
        return Ok(());
    }
    sim.initialize_schedules()?;
    assert_eq!(
        sim.remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)?,
        1
    );
    let p = parts(sim);
    sim.world_mut().insert_resource(Contacts::default());
    sim.add_systems(
        Phase::PhysicsStep,
        move |time: Res<UpdateInfo>,
              mut backend: ResMut<PhysicsBackend>,
              mut contacts: ResMut<Contacts>,
              fault: Res<StepFault>| {
            let native = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
            let segment = ContactSegment {
                duration: time.dt,
                native_steps: n,
                iterations: native.ordinary_contact_iterations(),
            };
            *contacts = Contacts::default();
            if let Err(e) = native.observe_contact_schedule(time.dt, &[segment], &[], |cut| {
                contacts.add(cut.contact_patches(), p)
            }) {
                fault.report(e);
            }
        },
    );
    Ok(())
}
#[derive(Debug, PartialEq, Serialize)]
struct Sample {
    tick: u32,
    xyz_m: [f64; 3],
    q: [f64; 4],
    vel_m_s: [f64; 3],
    omega_rad_s: [f64; 3],
    jaw_m: [[f64; 3]; 2],
    drive_n: f64,
    force_n: [f64; 3],
    torque_nm: [f64; 3],
    contacts: Contacts,
    linear_residual_ns: [f64; 3],
    angular_residual_nms: [f64; 3],
}
fn angular(s: BodyState) -> DVec3 {
    let q = DQuat::from_array(s.orientation).normalize();
    q * (INERTIA * (q.inverse() * DVec3::from(s.angular_velocity)))
}
fn commands(ms: u32, biased: bool, stay_open: bool) -> (f64, DVec3, DVec3) {
    let drive = if (140..164).contains(&ms) || (stay_open && ms >= 164) {
        -8.
    } else {
        8.
    };
    let loaded = (60..140).contains(&ms) || (220..TICKS).contains(&ms);
    let mut force = DVec3::ZERO;
    let mut torque = DVec3::ZERO;
    if loaded {
        force.z = -2.;
        if biased {
            force.x = 2.4;
            torque.y = 0.025;
        }
    }
    (drive, force, torque)
}
fn run(
    sim: &mut Simulation,
    xml: &str,
    biased: bool,
    stay_open: bool,
    replay: Option<&Path>,
) -> Result<Vec<Sample>> {
    let sdf = sdformat::load_str(xml, "bilateral.sdf", &sdformat::NoResources)?;
    let p = parts(sim);
    let mut out = Vec::with_capacity(TICKS as usize);
    let mut frames = Vec::new();
    if replay.is_some() {
        frames.push(gz_sim_cli::primitive_frame(sim, &sdf, "default", 1)?);
    }
    for ms in 0..TICKS {
        let (drive, force, torque) = commands(ms, biased, stay_open);
        JointForceCommand::set_batch(sim, p.drives.map(|j| (j, drive)))?;
        gz_sim_scene::queue_link_world_wrench(sim, p.parcel_id, force.into(), torque.into())?;
        let before = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(p.parcel)
            .unwrap();
        sim.step_exact(1)?;
        let engine = sim.world().resource::<PhysicsBackend>();
        let after = engine.state(p.parcel).unwrap();
        let contacts = sim
            .world()
            .get_resource::<Contacts>()
            .cloned()
            .unwrap_or_else(|| {
                let mut c = Contacts::default();
                c.add(
                    engine
                        .downcast_ref::<MultiFidelityBackend>()
                        .unwrap()
                        .rigid_contact_patches(),
                    p,
                );
                c
            });
        let mut dj = MASS * (DVec3::from(after.velocity) - DVec3::from(before.velocity))
            - (MASS * GRAVITY + force) * 0.001;
        let mut dh = angular(after) - angular(before) - torque * 0.001;
        for r in [
            &contacts.left,
            &contacts.right,
            &contacts.tray,
            &contacts.other,
        ] {
            dj -= DVec3::from(r.impulse_ns);
            dh -= DVec3::from(r.angular_nms);
        }
        out.push(Sample {
            tick: ms + 1,
            xyz_m: after.position,
            q: after.orientation,
            vel_m_s: after.velocity,
            omega_rad_s: after.angular_velocity,
            jaw_m: p.jaws.map(|id| engine.state(id).unwrap().position),
            drive_n: drive,
            force_n: force.into(),
            torque_nm: torque.into(),
            contacts,
            linear_residual_ns: dj.into(),
            angular_residual_nms: dh.into(),
        });
        if replay.is_some() && (ms + 1) % 2 == 0 {
            frames.push(gz_sim_cli::primitive_frame(
                sim,
                &sdf,
                "default",
                u64::from(ms + 1) + 1,
            )?);
        }
    }
    if let Some(path) = replay {
        gz_sim_cli::export_archive(path, Default::default(), frames)?;
    }
    Ok(out)
}
fn simulate(args: Args) -> Result<Vec<Sample>> {
    let xml = if args.flat_pads {
        Cow::Owned(WORLD.replace(
            "<sphere><radius>0.009</radius></sphere>",
            "<box><size>0.018 0.067 0.10</size></box>",
        ))
    } else {
        Cow::Borrowed(WORLD)
    };
    if let Some(path) = args.write_world {
        std::fs::write(path, xml.as_bytes())?;
    }
    let options = WorldLoadOptions {
        engine: PhysicsEngine::MultiFidelity,
        step: None,
        noise_seed: [0; 32],
        distributed_worker: None,
    };
    let runtime = WorldRuntimeOptions {
        rapier_per_point_friction: true,
        ..Default::default()
    };
    let mut sim = gz_sim_scene::load_world_with_runtime(
        &xml,
        "bilateral.sdf",
        "default",
        &sdformat::NoResources,
        options,
        runtime,
    )?;
    measured_schedule(&mut sim, args.substeps)?;
    let samples = run(
        &mut sim,
        &xml,
        args.biased,
        args.stay_open,
        args.replay.as_deref(),
    )?;
    if args.repeat {
        sim.reset()?;
        assert_eq!(
            samples,
            run(&mut sim, &xml, args.biased, args.stay_open, None)?
        );
    }
    Ok(samples)
}
fn main() -> Result<()> {
    let args = Args::parse();
    let summary = args.summary;
    let samples = simulate(args)?;
    let mut stdout = std::io::stdout().lock();
    if summary {
        serde_json::to_writer(&mut stdout, &summary::report(&samples))?;
    } else {
        serde_json::to_writer(&mut stdout, &samples)?;
    }
    stdout.flush()?;
    Ok(())
}
