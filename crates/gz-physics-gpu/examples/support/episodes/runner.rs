// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{pair, plane, support, task::*};
use bincode::Options;
use gz_physics_gpu::{
    Environment as Env, EnvironmentState as State, MixedBatch, MixedCheckpoint,
    simulation::{
        BatchTiming, batch_boundary, checkpoint_mixed, mixed_batch_simulation,
        queue_environment_impulses, reset_environments, restore_mixed_simulation,
    },
};
use gz_sim_core::Simulation;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::File,
    io::{Read, Write},
    path::Path,
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Default, Debug)]
struct Agreement {
    plane: plane::check::Errors,
    pair: pair::check::Errors,
    supported: pair::check::Errors,
    weight: f64,
}
impl Agreement {
    fn add(&mut self, gpu: &Env, cpu: &Env) {
        assert_eq!(gpu.id, cpu.id);
        match (&gpu.state, &cpu.state) {
            (State::SpherePlane(p), State::SpherePlane(q)) => self.plane.add(p, q),
            (State::SpherePair(p), State::SpherePair(q)) => self.pair.dense(p, q),
            (State::SupportedSpherePair(p), State::SupportedSpherePair(q)) => {
                self.supported.dense(&p.pair, &q.pair);
                for (a, b) in p
                    .support_normal_impulses
                    .iter()
                    .zip(q.support_normal_impulses)
                {
                    self.weight = self.weight.max((a - b).abs());
                }
            }
            _ => panic!("profile changed"),
        }
    }
    fn check(&self) {
        let e = &self.plane;
        assert!(e.p < 2e-9 && e.v < 5e-9 && e.w < 2e-7 && e.q < 2e-8 && e.jn < 5e-9 && e.jt < 5e-9);
        for e in [&self.pair, &self.supported] {
            assert!(
                e.motion.p < 2e-8
                    && e.motion.v < 2e-8
                    && e.motion.w < 2e-7
                    && e.motion.q < 2e-8
                    && e.impulse < 2e-8
                    && e.normal < 2e-8
                    && e.toi < 2e-10
            );
        }
        assert!(self.weight < 2e-10);
    }
}
#[derive(Default, Debug)]
struct Transfers {
    upload: u64,
    readback: u64,
    sparse_epochs: u64,
    full_epochs: u64,
    unchanged_epochs: u64,
    latency: Vec<u128>,
    end_to_end: Vec<u128>,
    last_stage: u128,
    dispatch: u128,
    timestamped: u64,
    pack: u128,
    wait_map: u128,
    decode: u128,
}
pub struct Runner {
    pub sim: Simulation,
    pub agent: Agent,
    cpu: Vec<Env>,
    agreement: Agreement,
    transfers: Transfers,
}
impl Runner {
    pub fn new(
        adapter: &str,
        seed: u64,
        mode: Mode,
        policy: Policy,
        reverse: bool,
    ) -> Result<Self> {
        let (agent, cpu) = Agent::new(seed, mode, policy);
        let state = (0..COUNT)
            .map(|k| {
                cpu[if reverse {
                    COUNT - 1 - k
                } else {
                    (733 * k + 59) % COUNT
                }]
            })
            .collect();
        Ok(Self {
            sim: mixed_batch_simulation(adapter, state)?,
            agent,
            cpu,
            agreement: Default::default(),
            transfers: Default::default(),
        })
    }
    pub fn states(&self) -> &[Env] {
        self.sim.world().resource::<MixedBatch>().environments()
    }
    pub fn stage(&mut self, chunk: usize) -> Result<Edits> {
        let began = Instant::now();
        let at = batch_boundary(&self.sim);
        let edits = self
            .agent
            .plan(at.tick, self.sim.world().resource::<MixedBatch>(), chunk)?;
        for resets in edits.resets.chunks(chunk) {
            reset_environments(&mut self.sim, at, resets)?;
        }
        for commands in edits.actions.chunks(chunk) {
            queue_environment_impulses(&mut self.sim, at, commands)?;
        }
        self.transfers.last_stage = began.elapsed().as_nanos();
        for &env in &edits.resets {
            self.cpu[index(env.id)] = env;
        }
        for act in &edits.actions {
            let state = &mut self.cpu[index(act.id)].state;
            let mass = sphere(state).inverse_mass;
            let dst = match state {
                State::SpherePlane(s) => &mut s.velocity,
                State::SpherePair(s) => &mut s.a.velocity,
                State::SupportedSpherePair(s) => &mut s.pair.a.velocity,
            };
            for (v, j) in dst.iter_mut().zip(act.impulse) {
                *v += j * mass;
            }
        }
        Ok(edits)
    }
    pub fn step(&mut self, validate: bool) -> Result<()> {
        let t = Instant::now();
        self.sim.step_exact(1)?;
        let wall = t.elapsed().as_nanos();
        self.transfers.latency.push(wall);
        let batch = self.sim.world().resource::<MixedBatch>();
        let stats = self.sim.world().resource::<BatchTiming>().last.unwrap();
        self.transfers.upload += stats.uploaded_bytes;
        self.transfers.readback += stats.readback_bytes;
        if stats.uploaded_bytes == 0 {
            self.transfers.unchanged_epochs += 1;
        } else if stats.uploaded_bytes == batch.device_info().state_bytes {
            self.transfers.full_epochs += 1;
        } else {
            self.transfers.sparse_epochs += 1;
        }
        self.transfers.pack += stats.pack.as_nanos();
        self.transfers.wait_map += stats.wait_map.as_nanos();
        self.transfers.decode += stats.decode.as_nanos();
        if let Some(t) = stats.gpu_dispatch {
            self.transfers.dispatch += t.as_nanos();
            self.transfers.timestamped += 1;
        }
        let reward = Instant::now();
        self.agent.observe(batch.environments());
        self.transfers
            .end_to_end
            .push(self.transfers.last_stage + wall + reward.elapsed().as_nanos());
        for (cpu, gpu) in self.cpu.iter_mut().zip(batch.environments()) {
            match &mut cpu.state {
                State::SpherePlane(s) => plane::reference::analytical(s),
                State::SpherePair(s) => pair::reference::analytical(s),
                State::SupportedSpherePair(s) => support::cpu_step(s),
            }
            if validate {
                self.agreement.add(gpu, cpu);
            }
        }
        Ok(())
    }
    pub fn same(&self, other: &Self) {
        assert_eq!(batch_boundary(&self.sim), batch_boundary(&other.sim));
        assert_eq!(self.agent, other.agent);
        for (a, b) in self.states().iter().zip(other.states()) {
            assert_eq!(a.id, b.id);
            assert_eq!(
                bytemuck::cast_slice::<f64, u8>(a.state.values()),
                bytemuck::cast_slice::<f64, u8>(b.state.values())
            );
        }
    }
    pub fn report(&mut self, name: &str) {
        self.agreement.check();
        let x = &mut self.transfers;
        x.latency.sort_unstable();
        x.end_to_end.sort_unstable();
        let n = x.end_to_end.len();
        let p99_total = x.end_to_end[(99 * n).div_ceil(100) - 1] as f64 / 1000.;
        let rate = COUNT as f64 * n as f64 / (x.end_to_end.iter().sum::<u128>() as f64 * 1e-9);
        let device: Option<f64> =
            (x.timestamped > 0).then(|| x.dispatch as f64 / x.timestamped as f64 / 1000.);
        println!(
            "{name}: decision + sparse admission + device/readback + reward: {rate:.0} environment steps/s, p99 {p99_total:.1} us. Host mean pack, wait/map, decode {:?} us. Device mean dispatch {device:?} us.",
            [x.pack, x.wait_map, x.decode].map(|ns| ns as f64 / n as f64 / 1000.)
        );
        let p99 = x.latency[(x.latency.len() * 99).div_ceil(100) - 1] as f64 / 1e3;
        println!(
            "{name}: analytic physical errors {:?}; decisions {} impulses {} selected resets {} episode spans [{},{}] ticks; mean velocity error m/s {:?}; full/sparse/unchanged epochs {}/{}/{}; uploaded {} B, read back {} B, Bevy physics epoch p99 {p99:.2} us",
            self.agreement,
            self.agent.decisions,
            self.agent.impulse_count,
            self.agent.resets,
            self.agent.min_horizon,
            self.agent.max_horizon,
            self.agent.error(),
            x.full_epochs,
            x.sparse_epochs,
            x.unchanged_epochs,
            x.upload,
            x.readback
        );
    }
    /// Save the complete experiment just after policy admission and before
    /// the device advance. Include the reference only for subsequent validation.
    pub fn save(&self, path: &Path) -> Result<usize> {
        let snap = Bundle {
            version: 1,
            physics: checkpoint_mixed(&self.sim)?.to_bytes()?,
            policy: self.agent.clone(),
            cpu: self.cpu.clone(),
        };
        let data = codec().serialize(&snap)?;
        let mut f = File::create_new(path)?;
        f.write_all(&data)?;
        f.write_all(&Sha256::digest(&data))?;
        f.sync_all()?;
        Ok(data.len() + 32)
    }
    pub fn load(adapter: &str, path: &Path) -> Result<Self> {
        let mut f = File::open(path)?;
        let size = f.metadata()?.len();
        if !(32..=256 * 1024 * 1024).contains(&size) {
            return Err("invalid experiment checkpoint length".into());
        }
        let mut bytes = Vec::with_capacity(size as usize);
        f.read_to_end(&mut bytes)?;
        let (data, hash) = bytes.split_at(bytes.len() - 32);
        if &Sha256::digest(data)[..] != hash {
            return Err("experiment checkpoint checksum mismatch".into());
        }
        let saved: Bundle = codec().deserialize(data)?;
        if saved.version != 1 || saved.policy.episodes.len() != COUNT || saved.cpu.len() != COUNT {
            return Err("invalid experiment checkpoint topology or version".into());
        }
        let snap = MixedCheckpoint::from_bytes(&saved.physics)?;
        Ok(Self {
            sim: restore_mixed_simulation(adapter, &snap)?,
            agent: saved.policy,
            cpu: saved.cpu,
            agreement: Default::default(),
            transfers: Default::default(),
        })
    }
}
#[derive(Serialize, Deserialize)]
struct Bundle {
    version: u16,
    physics: Vec<u8>,
    policy: Agent,
    cpu: Vec<Env>,
}
fn codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(256 * 1024 * 1024)
        .reject_trailing_bytes()
}
