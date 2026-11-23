// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare every committed public pose at sampled cuts across Bevy worker widths,
//! including direction changes, carried payloads and full-world reset.
#[allow(dead_code)]
#[path = "../../gz-sim-scene/examples/support/fleet_observation.rs"]
mod fleet;
use bevy_ecs::entity::Entity;
use clap::Parser;
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    ImuSample, LocalPose, Name, Velocity, WorldPose, queue_link_world_velocity, reset_loaded_world,
};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    io::Write,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long,default_value_t=10000,value_parser=clap::value_parser!(u32).range(1..=100000))]
    decks: u32,
    #[arg(long,default_value_t=4000,value_parser=clap::value_parser!(u32).range(1..))]
    steps: u32,
    #[arg(long,default_value_t=2,value_parser=clap::value_parser!(u32).range(1..))]
    epochs: u32,
    #[arg(long,default_value_t=200,value_parser=clap::value_parser!(u32).range(1..))]
    sample_every: u32,
    /// Stop after the next checked sample when Unix time reaches this value.
    #[arg(long)]
    until: Option<u64>,
}
struct Node {
    entity: Entity,
    id: u64,
    deck: Option<u32>,
    commanded: bool,
}
fn nodes(sim: &Simulation) -> Result<Vec<Node>> {
    let mut nodes = Vec::new();
    for e in sim.world().iter_entities() {
        let (Some(id), Some(name), Some(_)) =
            (e.get::<EntityId>(), e.get::<Name>(), e.get::<WorldPose>())
        else {
            continue;
        };
        let deck = name
            .0
            .split("::")
            .nth(1)
            .and_then(|n| n.strip_prefix("deck_"))
            .map(str::parse::<u32>)
            .transpose()?;
        let commanded = deck.is_some_and(|i| i % 1000 == 0) && e.contains::<Velocity>();
        nodes.push(Node {
            entity: e.id(),
            id: id.get(),
            deck,
            commanded,
        });
    }
    nodes.sort_unstable_by_key(|n| n.id);
    if nodes.windows(2).any(|v| v[0].id == v[1].id) {
        return Err("duplicate physical scene identity".into());
    }
    Ok(nodes)
}
fn hash_vec<const N: usize>(h: &mut Sha256, v: [f64; N]) -> Result<()> {
    for x in v {
        if !x.is_finite() {
            return Err("nonfinite published state".into());
        }
        h.update(x.to_bits().to_le_bytes());
    }
    Ok(())
}
fn observe(
    sim: &Simulation,
    nodes: &[Node],
    count: u32,
    tick: u32,
    edited_distance: i64,
    sign: i32,
) -> Result<(String, f64, usize)> {
    let cols = count.isqrt() + u32::from(count.isqrt().pow(2) < count);
    let mut h = Sha256::new();
    let mut max_error: f64 = 0.;
    let mut imu = 0;
    for n in nodes {
        let e = sim.world().get_entity(n.entity)?;
        if e.get::<EntityId>().map(EntityId::get) != Some(n.id) {
            return Err("reused physical entity".into());
        }
        let pose = e.get::<WorldPose>().ok_or("lost pose")?;
        let local = e.get::<LocalPose>().ok_or("lost local pose")?;
        h.update(n.id.to_le_bytes());
        hash_vec(&mut h, pose.translation)?;
        hash_vec(&mut h, pose.rotation)?;
        hash_vec(&mut h, local.0.translation)?;
        hash_vec(&mut h, local.0.rotation)?;
        if let Some(v) = e.get::<Velocity>() {
            h.update([1]);
            hash_vec(&mut h, v.linear)?;
            hash_vec(&mut h, v.angular)?;
        } else {
            h.update([0]);
        }
        if let Some(i) = n.deck {
            let steps = if i % 1000 == 0 {
                edited_distance as f64
            } else {
                f64::from(tick)
            };
            let expected = [
                f64::from(i % cols) * 1.1 + steps * 0.0025,
                f64::from(i / cols) * 1.1,
                0.,
            ];
            for (k, x) in pose.translation.iter().enumerate() {
                max_error = max_error.max((x - expected[k]).abs());
            }
            if let Some(v) = e.get::<Velocity>() {
                let vx = if i % 1000 == 0 {
                    f64::from(sign) * 0.25
                } else {
                    0.25
                };
                if v.linear != [vx, 0., 0.] || v.angular != [0.; 3] {
                    return Err("prescribed twist differs from command".into());
                }
            }
        }
        if let Some(sample) = e.get::<ImuSample>() {
            let r = sample.reading.as_ref().map_err(|_| "invalid IMU")?;
            if sample.time != sim.elapsed() || r.specific_force[2] != 9.81 {
                return Err("IMU timestamp or gravity differs".into());
            }
            imu += 1;
            hash_vec(&mut h, r.specific_force)?;
            hash_vec(&mut h, r.angular_velocity)?;
        }
    }
    if max_error > 1e-9 + f64::from(tick) * 2e-13 || imu != usize::from(tick > 0) {
        return Err(format!(
            "deck observation exceeds its error budget: tick {tick}, error {max_error}, IMUs {imu}"
        )
        .into());
    }
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or("missing native backend")?;
    let contacts = native.rigid_contact_observations();
    let loaded = contacts.iter().filter(|c| c.normal_impulse > 0.).count();
    let mut sorted = contacts;
    sorted.sort_unstable_by_key(|c| (c.a, c.b));
    for c in sorted {
        h.update(c.a.raw().to_le_bytes());
        h.update(c.b.raw().to_le_bytes());
        h.update([u8::from(c.active)]);
        hash_vec(&mut h, [c.normal_impulse])?;
    }
    Ok((format!("{:x}", h.finalize()), max_error, loaded))
}
fn main() -> Result<()> {
    let a = Args::parse();
    let mut sim = fleet::scene(a.decks as usize, true)?;
    let mut output = std::io::stdout().lock();
    for epoch in 0..a.epochs {
        let reset_start = Instant::now();
        if epoch != 0 {
            reset_loaded_world(&mut sim, |_| ())?;
        }
        let reset = reset_start.elapsed();
        let records = nodes(&sim)?;
        let mut signed_steps = 0_i64;
        let mut sign = 1_i32;
        let mut timings = Vec::new();
        for tick in 0..=a.steps {
            let change = tick > 1 && (tick - 1) % 300 == 0;
            if tick > 0 {
                if change {
                    sign = -sign;
                    for n in records.iter().filter(|n| n.commanded) {
                        queue_link_world_velocity(
                            &mut sim,
                            n.id,
                            [0.25 * f64::from(sign), 0., 0.],
                            [0.; 3],
                        )?;
                    }
                }
                signed_steps += i64::from(sign);
                let t = Instant::now();
                sim.step_exact(1)?;
                timings.push(t.elapsed());
            }
            if tick == 0
                || tick == a.steps
                || tick % a.sample_every == 0
                || change
                || tick > 2 && (tick - 2) % 300 == 0
            {
                let t = Instant::now();
                let (digest, error, contacts) =
                    observe(&sim, &records, a.decks, tick, signed_steps, sign)?;
                let audit = t.elapsed();
                let n = timings.len();
                let total: Duration = timings.iter().sum();
                timings.sort_unstable();
                let p95 = if n == 0 {
                    None
                } else {
                    Some(timings[(n * 95).div_ceil(100) - 1].as_micros())
                };
                let p99 = if n == 0 {
                    None
                } else {
                    Some(timings[(n * 99).div_ceil(100) - 1].as_micros())
                };
                serde_json::to_writer(
                    &mut output,
                    &json!({"epoch":epoch,"tick":tick,"decks":a.decks,"published_rows":records.len(),"active_loads":contacts,"max_deck_error_m":error,"sha256":digest,"sample_us":audit.as_micros(),"reset_us":reset.as_micros(),"timed_steps":n,"step_total_us":total.as_micros(),"step_p95_us":p95,"step_p99_us":p99}),
                )?;
                writeln!(output)?;
                output.flush()?;
                timings.clear();
                if a.until.is_some_and(|cut| {
                    SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .is_ok_and(|v| v.as_secs() >= cut)
                }) {
                    return Ok(());
                }
            }
        }
    }
    Ok(())
}
