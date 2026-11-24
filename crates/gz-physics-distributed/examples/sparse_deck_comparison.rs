// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare sparse velocity edits against materializing the complete translating fleet.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/fleet_observation.rs"]
mod fleet;
#[allow(dead_code)]
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
use clap::Parser;
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{BodyHandle, Name, queue_link_world_velocity};
use primitive_replay_view::View;
use serde_json::json;
use std::{
    error::Error,
    fs,
    path::PathBuf,
    time::{Duration, Instant},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long,default_value_t=1000,value_parser=clap::value_parser!(u32).range(256..=100000))]
    decks: u32,
    #[arg(long,default_value_t=1220,value_parser=clap::value_parser!(u32).range(302..))]
    ticks: u32,
    #[arg(long)]
    view: Option<PathBuf>,
    #[arg(long)]
    global_first: bool,
    /// Override grid width to reproduce a region of a larger scene.
    #[arg(long)]
    columns: Option<usize>,
    /// Use the fully native solver from construction as the comparison arm.
    #[arg(long)]
    native_from_start: bool,
    /// Measure differences in inactive zero-impulse pairs in addition to strict physical checks.
    #[arg(long)]
    audit_speculation: bool,
}
#[derive(Clone)]
struct Link {
    index: u32,
    id: u64,
    body: BodyId,
    payload: bool,
}
fn links(s: &Simulation) -> Result<Vec<Link>> {
    let mut rows = Vec::new();
    for e in s.world().iter_entities() {
        let Some(b) = e.get::<BodyHandle>() else {
            continue;
        };
        let name = &e.get::<Name>().ok_or("anonymous link")?.0;
        let (tag, index) = name
            .split("::")
            .nth(1)
            .ok_or("invalid scoped name")?
            .split_once('_')
            .ok_or("unexpected model")?;
        rows.push(Link {
            index: index.parse()?,
            id: e.get::<EntityId>().ok_or("missing id")?.get(),
            body: b.id(),
            payload: tag == "payload",
        });
    }
    rows.sort_by_key(|l| l.id);
    Ok(rows)
}
fn native(sim: &Simulation) -> &MultiFidelityBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
#[derive(Default)]
struct Peaks {
    pos: f64,
    vel: f64,
    spin: f64,
    angle: f64,
    normal: f64,
}
fn diff(a: [f64; 3], b: [f64; 3]) -> f64 {
    a.into_iter()
        .zip(b)
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f64>()
        .sqrt()
}
impl Peaks {
    fn add(&mut self, a: BodyState, b: BodyState) {
        self.pos = self.pos.max(diff(a.position, b.position));
        self.vel = self.vel.max(diff(a.velocity, b.velocity));
        self.spin = self.spin.max(diff(a.angular_velocity, b.angular_velocity));
        let q = |sign: f64| {
            a.orientation
                .into_iter()
                .zip(b.orientation)
                .map(|(x, y)| (x - sign * y).powi(2))
                .sum::<f64>()
                .sqrt()
        };
        self.angle = self.angle.max(4. * (q(1.).min(q(-1.)) / 2.).min(1.).asin());
    }
    fn value(&self) -> serde_json::Value {
        json!({"position_m":self.pos,"velocity_mps":self.vel,"spin_radps":self.spin,"orientation_rad":self.angle,"normal_impulse_ns":self.normal})
    }
}
fn main() -> Result<()> {
    let opts = Args::parse();
    let n = opts.decks;
    let new = || match opts.columns {
        Some(cols) => fleet::scene_with_columns(n as usize, true, cols),
        None => fleet::scene(n as usize, true),
    };
    let mut worlds = [new()?, new()?];
    if opts.native_from_start {
        worlds[0]
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .downcast_mut::<MultiFidelityBackend>()
            .unwrap()
            .disable_fleet_isolation();
    }
    let mut speculation_misses = 0;
    let mut speculation_first = None;
    let rows = links(&worlds[0])?;
    let other = links(&worlds[1])?;
    if rows
        .iter()
        .map(|x| (x.id, x.body))
        .ne(other.iter().map(|x| (x.id, x.body)))
    {
        return Err("unequal world identity".into());
    }
    let command: Vec<_> = rows
        .iter()
        .filter(|r| !r.payload && r.index % 1000 == 0)
        .collect();
    let observed: Vec<_> = rows
        .iter()
        .filter(|r| r.payload || r.index % 1000 == 0 || r.index < 4)
        .collect();
    let mut peak = Peaks::default();
    let mut peak_deck = Peaks::default();
    let mut frames = Vec::new();
    let mut index = Vec::new();
    let mut contact_intervals = 0;
    let mut timing = [Vec::new(), Vec::new()];
    let mut sign = 1.;
    for tick in 0..=opts.ticks {
        let edit = tick > 1 && (tick - 1) % 300 == 0;
        if edit {
            sign = -sign;
        }
        for lane in if (tick % 2 == 0) ^ opts.global_first {
            [0, 1]
        } else {
            [1, 0]
        } {
            let sim = &mut worlds[lane];
            if edit {
                for r in &command {
                    queue_link_world_velocity(sim, r.id, [0.25 * sign, 0., 0.], [0.; 3])?;
                }
            }
            let t = Instant::now();
            if tick == 301 && lane == 0 {
                sim.world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .downcast_mut::<MultiFidelityBackend>()
                    .unwrap()
                    .disable_fleet_isolation();
            }
            if tick > 0 {
                sim.step_exact(1)?;
            }
            if tick >= 301 {
                timing[lane].push(t.elapsed());
            }
        }
        let backends = worlds
            .each_ref()
            .map(|w| w.world().resource::<PhysicsBackend>());
        for l in &observed {
            let [a, b] = backends.map(|p| p.state(l.body).unwrap());
            if l.payload {
                peak.add(a, b);
            } else {
                peak_deck.add(a, b);
            }
        }
        let [mut x, mut y] = worlds
            .each_ref()
            .map(|s| native(s).rigid_contact_observations());
        x.sort_unstable_by_key(|c| (c.a, c.b));
        y.sort_unstable_by_key(|c| (c.a, c.b));
        if opts.audit_speculation {
            let dormant = |rows: &[gz_physics_multifidelity::RigidContactObservation]| {
                rows.iter()
                    .filter(|r| !r.active && r.normal_impulse == 0.)
                    .map(|r| (r.a, r.b))
                    .collect::<std::collections::BTreeSet<_>>()
            };
            let mismatch = dormant(&x).symmetric_difference(&dormant(&y)).count();
            if mismatch > 0 {
                speculation_misses += mismatch;
                speculation_first.get_or_insert(tick);
            }
            x.retain(|r| r.active || r.normal_impulse != 0.);
            y.retain(|r| r.active || r.normal_impulse != 0.);
        }
        let id = |i: BodyId| {
            rows.iter()
                .find(|l| l.body == i)
                .map(|r| (r.payload, r.index))
        };
        for (a, b) in x.iter().zip(&y) {
            if (a.a, a.b, a.active) != (b.a, b.b, b.active) {
                return Err(format!("contact identity at {tick}: {a:?} {:?}/{:?} vs {b:?} {:?}/{:?}, prior physical peaks {}",id(a.a),id(a.b),id(b.a),id(b.b),peak.value()).into());
            }
            peak.normal = peak.normal.max((a.normal_impulse - b.normal_impulse).abs());
        }
        if x.len() != y.len() {
            return Err(format!(
                "contact count at tick {tick}: {} vs {} with prior peaks {}",
                x.len(),
                y.len(),
                peak.value()
            )
            .into());
        }
        contact_intervals += x.iter().filter(|c| c.normal_impulse > 0.).count();
        if peak.pos > 1e-6
            || peak.vel > 1e-6
            || peak.angle > 1e-6
            || peak.spin > 1e-5
            || peak.normal > 1e-6
            || peak_deck.pos > 1e-7
        {
            return Err(format!(
                "diverged at tick {tick}: {} {}",
                peak.value(),
                peak_deck.value()
            )
            .into());
        }
        if opts.view.is_some() && (tick % 20 == 0 || edit || tick == opts.ticks) {
            let mut v = View::default();
            let mut seen = Vec::new();
            for lane in 0..2 {
                let offset = if lane == 0 { -0.57 } else { 0.57 };
                let tag = if lane == 0 {
                    "whole_world"
                } else {
                    "sparse_edit"
                };
                let origin = 0.25 * 0.01 * tick as f64;
                let mut physical = Vec::new();
                for (j, r) in observed.iter().filter(|r| r.index < 2).enumerate() {
                    let state = backends[lane].state(r.body).unwrap();
                    let name = format!(
                        "{tag}_{}_{}",
                        if r.payload { "load" } else { "deck" },
                        r.index
                    );
                    let [x, y, z] = state.position;
                    let center = [x - origin, y + offset, z];
                    v.add(100+lane as u64*50+j as u64*2,&name,center,state.orientation,
                        json!({"kind":"box","size":if r.payload{[0.20,0.20,0.20]}else{[1.0,1.0,0.2]}}),
                        if r.payload{[0.98,0.67,0.15,1.]}else if lane==0{[0.70,0.40,0.38,1.]}else{[0.13,0.58,0.76,1.]});
                    physical.push(
                        json!({"name":name,"position":state.position,"rotation":state.orientation}),
                    );
                }
                let active = native(&worlds[lane])
                    .fleet_isolation_stats()
                    .map_or(n as usize, |s| s.native_decks);
                let gauge = active as f64 / n as f64;
                v.cuboid(
                    130 + lane as u64 * 50,
                    &format!("{tag}_scale"),
                    [0., offset, 0.57],
                    [0.64, 0.015, 0.014],
                    [0.45, 0.49, 0.55, 1.],
                );
                v.add(
                    132 + lane as u64 * 50,
                    &format!("{tag}_native_fraction"),
                    [-0.32 + 0.64 * gauge, offset, 0.57],
                    [0., 0., 0., 1.],
                    json!({"kind":"sphere","radius":0.026}),
                    [0.9, 0.32, 0.6, 1.],
                );
                seen.push(json!({"name":tag,"native_decks":active,"origin_x":origin,"display_y":offset,"physical":physical}));
            }
            index.push(json!({"tick":tick,"lanes":seen}));
            frames.push(v.finish_at(frames.len() as u64 + 1, tick as u64 * 10_000_000));
        }
    }
    let times=timing.map(|mut samples|{samples.sort_unstable();let n=samples.len();let sum:Duration=samples.iter().sum();json!({"steps":n,"p50_us":samples[n/2].as_micros(),"p95_us":samples[(n*95).div_ceil(100)-1].as_micros(),"p99_us":samples[(n*99).div_ceil(100)-1].as_micros(),"rtf":n as f64*0.01/sum.as_secs_f64()})});
    let stats = native(&worlds[1])
        .fleet_isolation_stats()
        .ok_or("lost sparse isolation")?;
    let result = json!({"decks":n,"ticks":opts.ticks,"edited":command.len(),"global_native_from_start":opts.native_from_start,"inactive_pair_tick_differences":speculation_misses,"first_inactive_difference":speculation_first,"native_decks":stats.native_decks,"analytic_decks":stats.analytic_decks,"observed_payloads":observed.iter().filter(|l|l.payload).count(),"active_pair_ticks":contact_intervals,"peak_payload_differences":peak.value(),"peak_selected_deck_differences":peak_deck.value(),"whole":times[0],"sparse":times[1]});
    println!("{result}");
    if let Some(path) = opts.view {
        fs::create_dir(&path)?;
        gz_sim_cli::export_archive(&path.join("replay"), Default::default(), frames)?;
        fs::write(
            path.join("index.json"),
            serde_json::to_vec(&json!({"results":result,"frames":index}))?,
        )?;
    }
    Ok(())
}
