// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare complete Bevy ticks, motions and per-pad loads in a single multi-foot
//! world, one world per foot, and an ordinary native world.
#[path = "support/shared_batch/approach.rs"]
mod approach;
#[path = "support/shared_batch/staggered.rs"]
mod staggered;
#[path = "support/shared_batch/tier_cost.rs"]
mod tier_cost;
use glam::DVec3;
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::ArticulationTier::{Dynamic, Prescribed};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, primary_contact::last_refinement,
    queue_link_world_wrench, set_model_fidelity,
};
use serde_json::{Value, json};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
const SOURCE: &str = include_str!("../../../examples/multifidelity_parked_shared_tier.sdf");
const DT: f64 = 0.0005;
fn velocity(i: usize) -> DVec3 {
    let down = DVec3::new(1.7562801153073353, 2.200326579036272, -9.39734766315276).normalize();
    let x = (DVec3::X - down * down.x).normalize();
    let y = (-down).cross(x);
    let phi = [0.36, 2.38, -1.83, -0.85][i % 4];
    let (sn, cs) = f64::sin_cos(phi);
    let t = [0.01821381976, 0.04349999975, 0.08700000025, 0.12312957][i % 4];
    (x * cs + y * sn) * (0.31 * 9.81 * t)
}
fn rename(s: &str, i: usize) -> String {
    ["rocker", "lift", "toe"]
        .into_iter()
        .fold(s.to_string(), |s, name| {
            s.replace(&format!("name=\"{name}\""), &format!("name=\"{name}_{i}\""))
                .replace(
                    &format!("default::{name}::"),
                    &format!("default::{name}_{i}::"),
                )
        })
}
fn source(stations: &[usize], registered: bool, spacing: f64, stationary: bool) -> String {
    let template = SOURCE.find("    <plugin").unwrap();
    let models = SOURCE.find("    <model").unwrap();
    let closing = SOURCE.find("  </world>").unwrap();
    let hydra = &SOURCE[template..models];
    let begin = hydra.find("<pair>").unwrap();
    let end = hydra.rfind("</pair>").unwrap() + 7;
    let mut out = SOURCE[..template].to_string();
    if registered {
        out.push_str(&hydra[..begin]);
        for &i in stations {
            out.push_str(&rename(&hydra[begin..end], i));
        }
        out.push_str(&hydra[end..]);
    }
    for &i in stations {
        let prefix = rename(&SOURCE[models..closing], i);
        for (j, m) in prefix.split("<model name=").skip(1).enumerate() {
            let a = m.find("<pose>").unwrap() + 6;
            let z = a + m[a..].find("</pose>").unwrap();
            let vals = m[a..z]
                .split_whitespace()
                .map(str::parse::<f64>)
                .collect::<std::result::Result<Vec<_>, _>>()
                .unwrap();
            let shift = (i % 4) as f64 * spacing;
            let row = (i / 4) as f64 * spacing;
            out.push_str("<model name=");
            out.push_str(&m[..a]);
            out.push_str(&format!(
                "{:.17} {:.17} {:.17} {:.17} {:.17} {:.17}",
                vals[0] + shift,
                vals[1] + row,
                vals[2],
                vals[3],
                vals[4],
                vals[5]
            ));
            let mut suffix = m[z..].to_string();
            if j == 0 && !stationary {
                let v = velocity(i);
                suffix = suffix.replacen(
                    "<linear_velocity>0 0 0</linear_velocity>",
                    &format!(
                        "<linear_velocity>{:.17} {:.17} {:.17}</linear_velocity>",
                        v.x, v.y, v.z
                    ),
                    1,
                );
            }
            out.push_str(&suffix);
        }
    }
    out.push_str(&SOURCE[closing..]);
    out
}
fn bind(s: &Simulation, name: &str) -> (u64, Option<BodyId>) {
    #[allow(deprecated)]
    let e = s
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .expect("authored entity");
    (
        e.get::<EntityId>().unwrap().get(),
        e.get::<BodyHandle>().map(BodyHandle::id),
    )
}
fn load(src: &str) -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        src,
        "batch.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
struct Rig {
    sim: Simulation,
    ids: Vec<(u64, BodyId, u64)>,
}
impl Rig {
    fn new(stations: &[usize], registered: bool, spacing: f64, stationary: bool) -> Result<Self> {
        Self::from_source(stations, &source(stations, registered, spacing, stationary))
    }
    fn from_source(stations: &[usize], src: &str) -> Result<Self> {
        let sim = load(src)?;
        let ids = stations
            .iter()
            .map(|i| {
                (
                    bind(&sim, &format!("default::rocker_{i}")).0,
                    bind(&sim, &format!("default::rocker_{i}::foot")).1.unwrap(),
                    bind(&sim, &format!("default::rocker_{i}::upper")).0,
                )
            })
            .collect();
        Ok(Self { sim, ids })
    }
    fn states(&self) -> Vec<BodyState> {
        let b = self.sim.world().resource::<PhysicsBackend>();
        self.ids
            .iter()
            .map(|(_, h, _)| b.state(*h).unwrap())
            .collect()
    }
    fn loads(&self, t: u64) -> Vec<Option<[f64; 2]>> {
        let mut out = vec![None; self.ids.len()];
        if let Some(r) = last_refinement(&self.sim).filter(|r| r.iteration == t) {
            for phase in &r.interval.shared_faces {
                let i = self
                    .ids
                    .iter()
                    .position(|(_, f, _)| *f == phase.foot)
                    .unwrap();
                let v = out[i].get_or_insert([0.; 2]);
                for (a, site) in v.iter_mut().zip(phase.loads) {
                    *a += site.selected_normal_impulse;
                }
            }
        }
        out
    }
}
fn error(a: BodyState, b: BodyState) -> [f64; 4] {
    [
        DVec3::from(a.position).distance(DVec3::from(b.position)),
        DVec3::from(a.velocity).distance(DVec3::from(b.velocity)),
        DVec3::from(a.angular_velocity).distance(DVec3::from(b.angular_velocity)),
        a.orientation
            .into_iter()
            .zip(b.orientation)
            .map(|(a, b)| (a - b).abs())
            .fold(0., f64::max),
    ]
}
fn field(out: &Path) -> Result<()> {
    let ids = [0, 1, 2, 3];
    let src = source(&ids, true, 0.72, false);
    fs::write(out.join("batch.sdf"), &src)?;
    let mut batch = Rig::new(&ids, true, 0.72, false)?;
    let mut single = ids
        .iter()
        .map(|i| Rig::new(&[*i], true, 0.72, false))
        .collect::<Result<Vec<_>>>()?;
    let p0 = batch.states();
    let mut eq = [0f64; 5];
    let mut truth = [0f64; 3];
    let mut calls = 0;
    let mut ticks = vec![];
    let mut phases = vec![];
    for tick in 1..=300 {
        batch.sim.step_exact(1)?;
        for station in &mut single {
            station.sim.step_exact(1)?;
        }
        let loads = batch.loads(tick);
        let states = batch.states();
        let report = last_refinement(&batch.sim)
            .filter(|r| r.iteration == tick)
            .ok_or("missing batched interval")?;
        calls += report.interval.native_steps;
        for (i, station) in single.iter().enumerate() {
            let e = error(states[i], station.states()[0]);
            for k in 0..4 {
                eq[k] = eq[k].max(e[k]);
            }
            let a = loads[i].ok_or_else(|| format!("missing batch {i} at {tick}"))?;
            let b = station.loads(tick)[0].ok_or("missing isolated reference")?;
            eq[4] = eq[4].max((a[0] - b[0]).abs()).max((a[1] - b[1]).abs());
            let v = velocity(i);
            let h = (tick as f64 * DT).min(v.length() / (0.31 * 9.81));
            let accel = -v.normalize() * 0.31 * 9.81;
            truth[0] = truth[0].max(
                (DVec3::from(p0[i].position) + v * h + 0.5 * accel * h * h)
                    .distance(DVec3::from(states[i].position)),
            );
            truth[1] = truth[1].max((v + accel * h).distance(DVec3::from(states[i].velocity)));
            truth[2] = truth[2].max((a[0] + a[1] - 0.00981).abs());
        }
        for span in &report.interval.shared_faces {
            if span.comoving_rest && span.physical_start_seconds > 0. {
                phases.push(json!({"station":batch.ids.iter().position(|(_,f,_)|*f==span.foot).unwrap(),"event_s":(tick-1) as f64*DT+span.physical_start_seconds}));
            }
        }
        ticks.push(
            json!({"tick":tick,"native_steps":report.interval.native_steps,"support_ns":loads}),
        );
    }
    if eq.into_iter().chain(truth).any(|e| e > 5e-10) {
        return Err(format!("batch differences {eq:?} physical {truth:?}").into());
    }
    fs::write(
        out.join("field.json"),
        serde_json::to_vec(
            &json!({"native_steps":calls,"matched_errors_position_velocity_spin_quaternion_each_impulse":eq,"physical_errors_position_velocity_weight":truth,"arrests":phases,"ticks":ticks}),
        )?,
    )?;
    Ok(())
}
fn controls() -> Result<Value> {
    let mut saved = vec![];
    for (case, spacing, load) in [
        ("overlapping feet", 0., false),
        ("queued upper load", 0.72, true),
    ] {
        let mut s = Rig::new(&[0, 1], true, spacing, true)?;
        let mut control = Rig::new(&[0, 1], false, spacing, true)?;
        if load {
            for r in [&mut s, &mut control] {
                queue_link_world_wrench(&mut r.sim, r.ids[1].2, [0., 0., 2.], [0.; 3])?;
            }
        }
        s.sim.step_exact(1)?;
        control.sim.step_exact(1)?;
        assert!(s.loads(1).iter().all(Option::is_none));
        let es = s
            .states()
            .into_iter()
            .zip(control.states())
            .map(|(a, b)| error(a, b))
            .collect::<Vec<_>>();
        saved.push(json!({"control":case,"native_differences":es}));
    }
    let mut s = Rig::new(&[0, 1, 2, 3], true, 0.72, true)?;
    s.sim.step_exact(1)?;
    let roots = s.ids.clone();
    for (id, _, _) in &roots {
        set_model_fidelity(&mut s.sim, *id, Prescribed)?;
    }
    s.sim.step_exact(3)?;
    for (id, _, _) in &roots {
        set_model_fidelity(&mut s.sim, *id, Dynamic)?;
    }
    s.sim.step_exact(1)?;
    assert!(s.loads(5).iter().all(Option::is_some));
    saved.push(
        json!({"control":"four measured demotions and resumptions","resumed_pair_ns":s.loads(5)}),
    );
    Ok(json!(saved))
}
fn thread_clock_ns() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        let t = rustix::time::clock_gettime(rustix::time::ClockId::ThreadCPUTime);
        Some(t.tv_sec as u64 * 1_000_000_000 + t.tv_nsec as u64)
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}
fn bench(out: &Path) -> Result<()> {
    let mut cost = vec![];
    for rep in 0..9 {
        for k in 0..4 {
            let count = [1, 4, 16, 48][(k + rep) % 4];
            let ids = (0..count).collect::<Vec<_>>();
            for n in 0..3 {
                let mode = ["batch", "isolated", "ordinary"][(n + rep) % 3];
                let mut rigs = if mode == "isolated" {
                    ids.iter()
                        .map(|i| Rig::new(&[*i], true, 0.72, false))
                        .collect::<Result<Vec<_>>>()?
                } else {
                    vec![Rig::new(&ids, mode == "batch", 0.72, false)?]
                };
                let mut times = vec![];
                let mut cpu = vec![];
                let mut owners = 0;
                for tick in 1..=100 {
                    let start = thread_clock_ns();
                    let t = Instant::now();
                    for r in &mut rigs {
                        r.sim.step_exact(1)?;
                    }
                    times.push(t.elapsed().as_nanos() as u64);
                    cpu.push(thread_clock_ns().zip(start).map(|(t, s)| t - s));
                    for r in &rigs {
                        owners += r.loads(tick).into_iter().flatten().count();
                    }
                }
                if mode != "ordinary" && owners != 100 * count {
                    return Err(format!("{mode} retained {owners}/{}", 100 * count).into());
                }
                cost.push(json!({"mode":mode,"stations":count,"rep":rep,"wall_ns":times,"thread_cpu_ns":cpu,"registered_foot_ticks":owners}));
            }
        }
    }
    fs::write(out.join("cost.json"), serde_json::to_vec(&cost)?)?;
    Ok(())
}
fn main() -> Result<()> {
    let mut args = std::env::args_os().skip(1);
    let out = PathBuf::from(args.next().ok_or("supply output directory")?);
    fs::create_dir_all(&out)?;
    let mode = args.next();
    if mode.as_deref() == Some(std::ffi::OsStr::new("--approach-cost")) {
        return approach::cost(&out);
    }
    if mode.as_deref() == Some(std::ffi::OsStr::new("--approach")) {
        return approach::run(&out);
    }
    if mode.as_deref() == Some(std::ffi::OsStr::new("--staggered")) {
        return staggered::run(&out);
    }
    if mode.as_deref() == Some(std::ffi::OsStr::new("--tier-cost")) {
        return tier_cost::run(&out);
    }
    field(&out)?;
    fs::write(
        out.join("controls.json"),
        serde_json::to_vec_pretty(&controls()?)?,
    )?;
    if mode.as_deref() != Some(std::ffi::OsStr::new("--field-only")) {
        bench(&out)?;
    }
    Ok(())
}
