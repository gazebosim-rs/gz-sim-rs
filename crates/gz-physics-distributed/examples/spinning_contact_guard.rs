// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the ordinary Bevy fidelity guard to the continuous sphere-orbit intersection.
#[allow(dead_code)]
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, BodyHandle, Name, PhysicsEngine, contact_promotion, load_world_with_sdf_step,
    model_fidelity, queue_link_world_wrench, set_model_fidelity,
};
use primitive_replay_view::View;
use serde_json::{Value, json};
use std::{error::Error, fs, io::Write, path::PathBuf};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const WORLD: &str = include_str!("../../../examples/multifidelity_spinning_guard.sdf");
const R: f64 = 0.325;
const CONTACT: f64 = 0.09;
#[derive(Parser)]
struct Args {
    /// Number of alternating-direction impacts and near misses.
    #[arg(long, default_value_t=1, value_parser=clap::value_parser!(u32).range(1..))]
    cases: u32,
    #[arg(long, default_value_t = 0)]
    first: u32,
    /// Write the actual Bevy poses and native gz-web-rs archive for the first case.
    #[arg(long)]
    view: Option<PathBuf>,
}
fn id(sim: &Simulation, name: &str) -> Result<u64> {
    sim.world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<Name>()?.0 == name)
                .then(|| e.get::<EntityId>().map(EntityId::get))
                .flatten()
        })
        .ok_or_else(|| format!("missing {name}").into())
}
fn body(sim: &Simulation, id: u64) -> Result<BodyState> {
    let world = sim.world();
    let e = entity_by_id(world, id).ok_or("missing link")?;
    world
        .resource::<PhysicsBackend>()
        .state(world.get::<BodyHandle>(e).ok_or("missing handle")?.id())
        .ok_or_else(|| "missing state".into())
}
fn load(xml: &str) -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        xml,
        "spinning-guard.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
struct Case {
    index: u32,
    dt: f64,
    speed: f64,
    angle: f64,
    height: f64,
    stop: DVec3,
}
impl Case {
    fn new(index: u32) -> Self {
        let i = u64::from(index);
        let sample = |k: u64| {
            ((i.wrapping_mul(6364136223846793005)
                .rotate_left(k as u32)
                .wrapping_add(k * 997))
                % 1009) as f64
                / 1009.
        };
        let dt = [0.004, 0.001, 0.008, 0.016][index as usize / 4 % 4];
        let sign = if index.is_multiple_of(2) { 1. } else { -1. };
        let speed = sign * (1.3 + 9. * sample(19));
        let angle = sign * (0.7 + 0.9 * sample(31));
        let height = if index % 4 < 2 {
            0.05 * sample(43)
        } else {
            0.16 + 0.08 * sample(43)
        };
        Self {
            index,
            dt,
            speed,
            angle,
            height,
            stop: DVec3::new(-R * angle.sin(), R * angle.cos(), height),
        }
    }
    fn xml(&self) -> String {
        WORLD.replace(">0.004<", &format!(">{}<", self.dt)).replace(
            "-0.273478 0.175598 0 0 0 0",
            &format!("{} {} {} 0 0 0", self.stop.x, self.stop.y, self.stop.z),
        )
    }
    fn spin(&self, sim: &mut Simulation, links: [u64; 2]) -> Result<()> {
        let a = self.speed / self.dt;
        for (i, link) in links.into_iter().enumerate() {
            let r = if i == 0 { -R } else { R };
            queue_link_world_wrench(sim, link, [-r * a, 0., 0.], [0., 0., 0.00121 * a])?;
        }
        sim.step_exact(1)?;
        Ok(())
    }
    fn run(&self, out: Option<&PathBuf>) -> Result<Value> {
        let xml = self.xml();
        let mut sim = load(&xml)?;
        let mut control = load(&xml)?;
        let model = id(&sim, "default::rotor")?;
        let stop = id(&sim, "default::stop::head")?;
        let links = [
            id(&sim, "default::rotor::base")?,
            id(&sim, "default::rotor::tip")?,
        ];
        let mut archive = Vec::new();
        let mut channels = Vec::new();
        self.frame(
            &sim,
            links,
            0,
            true,
            out.is_some(),
            &mut archive,
            &mut channels,
        )?;
        for s in [&mut sim, &mut control] {
            self.spin(s, links)?;
            set_model_fidelity(s, model, ArticulationTier::Prescribed)?;
        }
        contact_promotion::configure(&mut sim, model, vec![stop])?;
        self.frame(
            &sim,
            links,
            1,
            false,
            out.is_some(),
            &mut archive,
            &mut channels,
        )?;
        let base = body(&sim, links[0])?;
        let tip = body(&sim, links[1])?;
        let omega = DVec3::from(tip.angular_velocity);
        let com = (DVec3::from(base.position) + DVec3::from(tip.position)) / 2.;
        let vcom = (DVec3::from(base.velocity) + DVec3::from(tip.velocity)) / 2.;
        if omega.truncate().length() > 1e-8
            || (omega.z - self.speed).abs() > 0.015 * self.speed.abs()
            || !vcom.is_finite()
        {
            return Err(format!(
                "initial spin or translation differs from fixture: {omega:?} {vcom:?}"
            )
            .into());
        }
        let initial = DVec3::from(tip.position) - com;
        let azimuth = |p: DVec3| p.y.atan2(p.x);
        let theta = ((azimuth(self.stop - com) - azimuth(initial)) * omega.z.signum())
            .rem_euclid(std::f64::consts::TAU);
        let until = ((theta + 0.6) / omega.z.abs() / self.dt).ceil() as u64;
        // The drive interval also imparts a small measured COM velocity. Use
        // the translated circular orbit from the admitted cut, then bisect
        // its first crossing. The axial separation certifies every clear case.
        let orbit =
            |time: f64| com + vcom * time + DQuat::from_rotation_z(omega.z * time) * initial;
        let horizon = until as f64 * self.dt;
        let mut impact = None;
        for i in 1..=4096 {
            let mut hi = horizon * f64::from(i) / 4096.;
            if orbit(hi).distance(self.stop) > CONTACT {
                continue;
            }
            let mut lo = horizon * f64::from(i - 1) / 4096.;
            for _ in 0..48 {
                let mid = (lo + hi) * 0.5;
                if orbit(mid).distance(self.stop) <= CONTACT {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            impact = Some(hi);
            break;
        }
        if impact.is_some() != (self.height < CONTACT) {
            return Err("fixture did not cover its intended impact or clear path".into());
        }
        let mut promote = None;
        let mut max_orbit: f64 = 0.;
        let mut max_impulse: f64 = 0.;
        let mut max_momentum_change: f64 = 0.;
        let stop_body = sim
            .world()
            .get::<BodyHandle>(entity_by_id(sim.world(), stop).unwrap())
            .unwrap()
            .id();
        let mut min_gap = f64::INFINITY;
        for step in 1..=until {
            sim.step_exact(1)?;
            let dynamic = model_fidelity(&sim, model)? == ArticulationTier::Dynamic;
            if dynamic && promote.is_none() {
                promote = Some(step);
                set_model_fidelity(&mut control, model, ArticulationTier::Dynamic)?;
            }
            control.step_exact(1)?;
            for link in links {
                if body(&sim, link)? != body(&control, link)? {
                    return Err(
                        format!("guard handoff differs from manual promotion at {step}").into(),
                    );
                }
            }
            let a = body(&sim, links[0])?;
            let b = body(&sim, links[1])?;
            let t = step as f64 * self.dt;
            let predicted = com + vcom * t + DQuat::from_rotation_z(omega.z * t) * initial;
            if !dynamic {
                max_orbit = max_orbit.max(predicted.distance(b.position.into()));
            }
            let gap = self.stop.distance(b.position.into()) - CONTACT;
            min_gap = min_gap.min(gap);
            max_momentum_change = max_momentum_change
                .max((DVec3::from(a.velocity) + DVec3::from(b.velocity) - 2. * vcom).length());
            let contacts = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
                .ok_or("missing native solver")?
                .rigid_contact_observations();
            for c in contacts
                .iter()
                .filter(|c| c.a == stop_body || c.b == stop_body)
            {
                max_impulse = max_impulse.max(c.normal_impulse);
            }
            self.frame(
                &sim,
                links,
                step + 1,
                dynamic,
                out.is_some(),
                &mut archive,
                &mut channels,
            )?;
        }
        if max_orbit > 1e-7
            || impact.is_some_and(|t| promote.is_none_or(|n| (n - 1) as f64 * self.dt > t + 1e-8))
            || impact.is_some() && max_impulse < 0.01
        {
            return Err(format!("missed or inaccurate curved impact case {}: {impact:?}, tick {promote:?}, impulse {max_impulse}, orbit {max_orbit}",self.index).into());
        }
        if impact.is_none() && max_impulse > 1e-8 {
            return Err("near miss delivered external impulse".into());
        }
        let report = json!({"case":self.index,"dt_s":self.dt,"omega_rad_s":omega.z,"orbit_radius_m":initial.truncate().length(),"stop_angle_rad":self.angle,"height_m":self.height,"analytic_contact_after_drive_s":impact,"promotion_interval_after_drive":promote,"evaluated_intervals":until,"max_prescribed_orbit_error_m":max_orbit,"min_sampled_surface_gap_m":min_gap,"max_change_in_linear_momentum_ns":max_momentum_change,"peak_stop_normal_impulse_ns":max_impulse});
        if let Some(out) = out {
            fs::create_dir(out)?;
            gz_sim_cli::export_archive(&out.join("replay"), Default::default(), archive)?;
            fs::write(
                out.join("index.json"),
                serde_json::to_vec(&json!({"case":report,"frames":channels}))?,
            )?;
        }
        Ok(report)
    }
    #[allow(clippy::too_many_arguments)]
    fn frame(
        &self,
        sim: &Simulation,
        links: [u64; 2],
        tick: u64,
        dynamic: bool,
        record: bool,
        archive: &mut Vec<(String, String)>,
        channels: &mut Vec<Value>,
    ) -> Result<()> {
        if !record {
            return Ok(());
        }
        let mut v = View::default();
        let mut link_poses = Vec::new();
        for (i, link) in links.into_iter().enumerate() {
            let b = body(sim, link)?;
            v.add(
                10 + i as u64 * 2,
                ["base", "tip"][i],
                b.position,
                b.orientation,
                json!({"kind":"sphere","radius":0.055}),
                if i == 0 {
                    [0.14, 0.55, 0.92, 1.]
                } else {
                    [0.95, 0.57, 0.12, 1.]
                },
            );
            link_poses.push(json!({"position":b.position,"rotation":b.orientation}));
        }
        let a = DVec3::from(body(sim, links[0])?.position);
        let b = DVec3::from(body(sim, links[1])?.position);
        let axis = (b - a).normalize();
        v.add(
            14,
            "hinge_span",
            ((a + b) / 2.).to_array(),
            DQuat::from_rotation_arc(DVec3::Y, axis).to_array(),
            json!({"kind":"box","size":[0.015,2.*R,0.015]}),
            [0.35, 0.48, 0.58, 1.],
        );
        v.add(
            16,
            "fixed_stop",
            self.stop.to_array(),
            [0., 0., 0., 1.],
            json!({"kind":"sphere","radius":0.035}),
            [0.91, 0.23, 0.21, 1.],
        );
        // The indicator is a recorded solver-tier channel separate from collision geometry.
        v.cuboid(
            18,
            "solver_tier",
            [-0.48, -0.35, if dynamic { 0.17 } else { 0. }],
            [0.05; 3],
            [0.82, 0.3, 0.55, 1.],
        );
        let ns = sim.elapsed().as_nanos().try_into()?;
        archive.push(v.finish_at(tick + 1, ns));
        channels.push(json!({"tick":tick,"time_ns":ns,"dynamic":dynamic,"links":link_poses}));
        Ok(())
    }
}
fn main() -> Result<()> {
    let a = Args::parse();
    let last = a.first.checked_add(a.cases).ok_or("case range overflow")?;
    let mut log = std::io::stdout().lock();
    for i in a.first..last {
        let r = Case::new(i).run(a.view.as_ref().filter(|_| i == a.first))?;
        serde_json::to_writer(&mut log, &r)?;
        writeln!(log)?;
    }
    Ok(())
}
