// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded little-endian native-f64 pair observations, full reset seeds and optional Bevy clock.
use super::{SpherePairBlockBatch, SpherePairBlockObservation as Obs};
use crate::{
    GpuError, GpuInfo, MAX_BLOCK_TICKS, ObservationKernel, STEP, SpherePairLane as Lane,
    SphereRotation,
};
use bytemuck::Zeroable;
use gz_sim_core::{Simulation, UpdateInfo};
use sha2::{Digest, Sha256};
use std::{mem::size_of, time::Duration};
const MAGIC: &[u8; 8] = b"GZPRBL01";
const HEADER: usize = 80;
const CHECKSUM: usize = 32;
const LANE_BYTES: usize = size_of::<Lane>() + size_of::<Obs>();
fn bad(s: impl ToString) -> GpuError {
    GpuError::Checkpoint(s.to_string())
}
fn fingerprint(info: &GpuInfo) -> [u8; 32] {
    let mut h = Sha256::new();
    h.update(info.vendor_id.to_le_bytes());
    h.update(info.device_id.to_le_bytes());
    for s in [
        &info.driver[..],
        &info.driver_info[..],
        include_str!("../sphere_pair.rs"),
        include_str!("../sphere_pair_block.rs"),
        include_str!("../pair_rotation.rs"),
        include_str!("../arithmetic.rs"),
        include_str!("../spatial.rs"),
        include_str!("../kernel.rs"),
    ] {
        h.update((s.len() as u64).to_le_bytes());
        h.update(s.as_bytes());
    }
    h.finalize().into()
}
#[derive(Debug, Clone, Copy)]
struct Clock {
    cadence: u32,
    epoch: u64,
    iteration: u64,
    update_paused: bool,
    paused: bool,
    advanced: bool,
}
impl Clock {
    fn info(&self, tick: u64) -> Result<UpdateInfo, GpuError> {
        if !(1..=MAX_BLOCK_TICKS).contains(&self.cadence)
            || self.iteration.checked_mul(u64::from(self.cadence)) != Some(tick)
            || (if self.advanced {
                self.update_paused || self.iteration == 0
            } else {
                !self.update_paused && !(self.iteration == 0 && self.epoch != 0)
            })
        {
            return Err(bad("inconsistent public observation clock"));
        }
        let sim_time = Duration::from_millis(tick);
        let dt = if self.advanced {
            STEP * self.cadence
        } else {
            Duration::ZERO
        };
        Ok(UpdateInfo {
            start_time: sim_time
                .checked_sub(dt)
                .ok_or_else(|| bad("negative observation start"))?,
            sim_time,
            dt,
            iteration: self.iteration,
            epoch: self.epoch,
            paused: self.update_paused,
        })
    }
}
/// An immutable committed GPU observation including the original reset seeds.
/// Restoring on the same Vulkan device/driver/kernel profile creates fresh buffers.
/// Captures taken through Bevy also preserve its observation cadence, epoch and pause state.
#[derive(Debug, Clone)]
pub struct SpherePairBlockCheckpoint {
    initial: Vec<Lane>,
    observation: Vec<Obs>,
    tick: u64,
    span: u32,
    rotation: SphereRotation,
    target: [u8; 32],
    clock: Option<Clock>,
}
impl SpherePairBlockCheckpoint {
    pub const MAX_BYTES: usize = 128 * 1024 * 1024;
    pub fn tick(&self) -> u64 {
        self.tick
    }
    pub fn lanes(&self) -> usize {
        self.initial.len()
    }
    pub fn last_block_ticks(&self) -> u32 {
        self.span
    }
    /// Internal 1 ms steps per Bevy publication, when this cut includes its public clock.
    pub fn observation_cadence(&self) -> Option<u32> {
        self.clock.map(|c| c.cadence)
    }
    pub fn rotation(&self) -> SphereRotation {
        self.rotation
    }
    pub fn observations(&self) -> &[Obs] {
        &self.observation
    }
    fn size(n: usize) -> Result<usize, GpuError> {
        n.checked_mul(LANE_BYTES)
            .and_then(|b| b.checked_add(HEADER + CHECKSUM))
            .filter(|b| n > 0 && *b <= Self::MAX_BYTES)
            .ok_or_else(|| bad("invalid resident pair checkpoint population or length"))
    }
    /// Capture the last successfully observed block, including after a failed advance.
    pub fn capture(batch: &SpherePairBlockBatch) -> Result<Self, GpuError> {
        Self::size(batch.lanes().len())?;
        let control = batch
            .block_control
            .as_ref()
            .ok_or_else(|| bad("missing pair block control"))?;
        let ObservationKernel::Pair(rotation) = control.kernel else {
            return Err(bad("wrong observation kernel"));
        };
        Ok(Self {
            initial: batch.initial.iter().map(|r| r.state).collect(),
            observation: batch.lanes().to_vec(),
            tick: batch.tick(),
            span: control.last_ticks,
            rotation,
            target: fingerprint(batch.device_info()),
            clock: None,
        })
    }
    pub fn capture_simulation(sim: &Simulation) -> Result<Self, GpuError> {
        let batch = sim
            .world()
            .get_resource::<SpherePairBlockBatch>()
            .ok_or_else(|| bad("world has no resident sphere pairs"))?;
        let mut cp = Self::capture(batch)?;
        let info = sim.committed_update_info().map_err(bad)?;
        let ms = u32::try_from(sim.step_duration().as_millis()).map_err(bad)?;
        let c = Clock {
            cadence: ms,
            epoch: info.epoch,
            iteration: info.iteration,
            update_paused: info.paused,
            paused: sim.is_paused(),
            advanced: !info.dt.is_zero(),
        };
        let derived = c.info(cp.tick)?;
        if ms == 0
            || sim.step_duration() != STEP * ms
            || derived.start_time != info.start_time
            || derived.dt != info.dt
            || derived.sim_time != info.sim_time
            || (cp.tick != 0 && cp.span != ms)
        {
            return Err(bad("uncommitted device/public clock pair"));
        }
        cp.clock = Some(c);
        Ok(cp)
    }
    fn header(&self, n: usize, clock: Option<Clock>) -> Vec<u8> {
        let mut out = Vec::with_capacity(HEADER);
        out.extend_from_slice(MAGIC);
        out.push(match self.rotation {
            SphereRotation::Midpoint => 0,
            SphereRotation::FourthOrder => 1,
        });
        out.push(clock.map_or(0, |c| {
            1 | (u8::from(c.paused) << 1)
                | (u8::from(c.update_paused) << 2)
                | (u8::from(c.advanced) << 3)
        }));
        out.extend_from_slice(&[0; 2]);
        out.extend_from_slice(&self.span.to_le_bytes());
        out.extend_from_slice(&self.tick.to_le_bytes());
        out.extend_from_slice(&(n as u32).to_le_bytes());
        out.extend_from_slice(&clock.map_or(0, |c| c.cadence).to_le_bytes());
        out.extend_from_slice(&clock.map_or(0, |c| c.epoch).to_le_bytes());
        out.extend_from_slice(&clock.map_or(0, |c| c.iteration).to_le_bytes());
        out.extend_from_slice(&self.target);
        debug_assert_eq!(out.len(), HEADER);
        out
    }
    fn range_digest(
        &self,
        r: std::ops::Range<usize>,
        clock: Option<Clock>,
    ) -> Result<[u8; 32], GpuError> {
        if !cfg!(target_endian = "little") {
            return Err(GpuError::ByteOrder);
        }
        let mut hash = Sha256::new();
        hash.update(self.header(r.len(), clock));
        hash.update(bytemuck::cast_slice::<_, u8>(&self.initial[r.clone()]));
        hash.update(bytemuck::cast_slice::<_, u8>(&self.observation[r]));
        Ok(hash.finalize().into())
    }
    pub(crate) fn digest(&self) -> Result<[u8; 32], GpuError> {
        self.range_digest(0..self.lanes(), self.clock)
    }
    pub(crate) fn subset_digest(&self, r: std::ops::Range<usize>) -> Result<[u8; 32], GpuError> {
        self.range_digest(r, None)
    }
    pub fn to_bytes(&self) -> Result<Vec<u8>, GpuError> {
        if !cfg!(target_endian = "little") {
            return Err(GpuError::ByteOrder);
        }
        let total = Self::size(self.lanes())?;
        let mut out = Vec::with_capacity(total);
        out.extend_from_slice(&self.header(self.lanes(), self.clock));
        out.extend_from_slice(bytemuck::cast_slice(&self.initial));
        out.extend_from_slice(bytemuck::cast_slice(&self.observation));
        let digest = Sha256::digest(&out);
        out.extend_from_slice(&digest);
        Ok(out)
    }
    /// Verify the entire envelope, clock and physical rows before requesting a GPU.
    pub fn from_bytes(blob: &[u8]) -> Result<Self, GpuError> {
        if !cfg!(target_endian = "little") {
            return Err(GpuError::ByteOrder);
        }
        if !(HEADER + CHECKSUM..=Self::MAX_BYTES).contains(&blob.len()) || &blob[..8] != MAGIC {
            return Err(bad("invalid resident pair checkpoint header"));
        }
        let end = blob.len() - CHECKSUM;
        if Sha256::digest(&blob[..end])[..] != blob[end..] {
            return Err(bad("invalid resident pair checkpoint checksum"));
        }
        let u32at = |i| u32::from_le_bytes(blob[i..i + 4].try_into().unwrap());
        let u64at = |i| u64::from_le_bytes(blob[i..i + 8].try_into().unwrap());
        let rotation = match blob[8] {
            0 => SphereRotation::Midpoint,
            1 => SphereRotation::FourthOrder,
            _ => return Err(bad("unknown angular integration profile")),
        };
        let flags = blob[9];
        let span = u32at(12);
        let tick = u64at(16);
        let n = u32at(24) as usize;
        if Self::size(n)? != blob.len()
            || blob[10..12] != [0; 2]
            || flags > 15
            || (tick == 0) != (span == 0)
            || span > MAX_BLOCK_TICKS
            || u64::from(span) > tick
        {
            return Err(bad("invalid resident pair block metadata"));
        }
        let clock = if flags & 1 != 0 {
            let c = Clock {
                cadence: u32at(28),
                epoch: u64at(32),
                iteration: u64at(40),
                update_paused: flags & 4 != 0,
                paused: flags & 2 != 0,
                advanced: flags & 8 != 0,
            };
            c.info(tick)?;
            if tick != 0 && span != c.cadence {
                return Err(bad("last resident block differs from Bevy cadence"));
            }
            Some(c)
        } else {
            if flags != 0 || blob[28..48] != [0; 20] {
                return Err(bad("unexpected detached clock fields"));
            }
            None
        };
        let target = blob[48..80].try_into().unwrap();
        let split = HEADER + n * size_of::<Lane>();
        let seeds = blob[HEADER..split].chunks_exact(size_of::<Lane>());
        let states = blob[split..end].chunks_exact(size_of::<Obs>());
        let mut initial = Vec::with_capacity(n);
        let mut observation = Vec::with_capacity(n);
        for (i, (seed, state)) in seeds.zip(states).enumerate() {
            let s: Lane = bytemuck::pod_read_unaligned(seed);
            let o: Obs = bytemuck::pod_read_unaligned(state);
            if !s.initial_valid() || !valid(s, o, span) {
                return Err(bad(format!("invalid resident pair observation {i}")));
            }
            initial.push(s);
            observation.push(o);
        }
        Ok(Self {
            initial,
            observation,
            span,
            tick,
            clock,
            target,
            rotation,
        })
    }
    pub(crate) fn check_next_block(&self, ticks: u32) -> Result<u64, GpuError> {
        if !(1..=MAX_BLOCK_TICKS).contains(&ticks) {
            return Err(GpuError::BlockLength(ticks));
        }
        if self.clock.is_some_and(|c| c.cadence != ticks) {
            return Err(bad("shard interval differs from Bevy cadence"));
        }
        self.tick
            .checked_add(u64::from(ticks))
            .ok_or(GpuError::Clock)
    }
    pub(crate) fn subset(&self, r: std::ops::Range<usize>) -> Self {
        Self {
            initial: self.initial[r.clone()].to_vec(),
            observation: self.observation[r].to_vec(),
            clock: None,
            tick: self.tick,
            span: self.span,
            rotation: self.rotation,
            target: self.target,
        }
    }
    pub(crate) fn decode_subset_successor(
        &self,
        range: std::ops::Range<usize>,
        span: u32,
        data: &[u8],
    ) -> Result<Self, GpuError> {
        if !cfg!(target_endian = "little") {
            return Err(GpuError::ByteOrder);
        }
        let tick = self.check_next_block(span)?;
        let seeds = &self.initial[range];
        if data.len() != std::mem::size_of::<Obs>() * seeds.len() {
            return Err(bad("wrong measurement count"));
        }
        let rows = data
            .chunks_exact(std::mem::size_of::<Obs>())
            .zip(seeds)
            .map(|(bytes, &seed)| {
                let row = bytemuck::pod_read_unaligned(bytes);
                valid(seed, row, span)
                    .then_some(row)
                    .ok_or_else(|| bad("invalid resident worker observation"))
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            initial: seeds.to_vec(),
            observation: rows,
            tick,
            span,
            clock: None,
            rotation: self.rotation,
            target: self.target,
        })
    }
    pub(crate) fn matches_subset(&self, r: std::ops::Range<usize>, other: &Self) -> bool {
        self.target == other.target
            && self.rotation == other.rotation
            && other.clock.is_none()
            && bytemuck::cast_slice::<_, u8>(&self.initial[r])
                == bytemuck::cast_slice::<_, u8>(&other.initial)
    }
    pub(crate) fn advance_with_rows(&self, ticks: u32, rows: Vec<Obs>) -> Result<Self, GpuError> {
        let tick = self.check_next_block(ticks)?;
        if rows.len() != self.lanes() {
            return Err(bad("incomplete barrier"));
        }
        let mut clock = self.clock;
        if let Some(c) = &mut clock {
            c.iteration = c.iteration.checked_add(1).ok_or(GpuError::Clock)?;
            c.update_paused = false;
            c.advanced = true;
            c.info(tick)?;
        }
        Ok(Self {
            initial: self.initial.clone(),
            observation: rows,
            tick,
            span: ticks,
            rotation: self.rotation,
            target: self.target,
            clock,
        })
    }
    pub fn restore(&self, adapter: &str) -> Result<SpherePairBlockBatch, GpuError> {
        let mut batch =
            SpherePairBlockBatch::with_rotation(adapter, self.initial.clone(), self.rotation)?;
        if fingerprint(batch.device_info()) != self.target {
            return Err(bad("checkpoint GPU device, driver or kernel differs"));
        }
        batch.committed.copy_from_slice(&self.observation);
        batch.tick = self.tick;
        batch
            .block_control
            .as_mut()
            .expect("block profile")
            .last_ticks = self.span;
        Ok(batch)
    }
    pub fn restore_simulation(&self, adapter: &str) -> Result<Simulation, GpuError> {
        let c = self
            .clock
            .ok_or_else(|| bad("batch checkpoint has no public observation clock"))?;
        let sim = Simulation::from_committed_clock(STEP * c.cadence, c.info(self.tick)?, c.paused)
            .map_err(bad)?;
        Ok(crate::simulation::attach_observations(
            sim,
            self.restore(adapter)?,
            c.cadence,
        ))
    }
}
fn valid(s: Lane, o: Obs, span: u32) -> bool {
    if span == 0 {
        return bytemuck::bytes_of(&o)
            == bytemuck::bytes_of(&Obs {
                state: s,
                ..Obs::zeroed()
            });
    }
    let x = o.state;
    let same = |a: f64, b: f64| a.to_bits() == b.to_bits();
    let bodies = [(s.a, x.a), (s.b, x.b)];
    if !bytemuck::cast_slice::<_, f64>(std::slice::from_ref(&o))
        .iter()
        .all(|v| v.is_finite())
        || !bodies.iter().all(|(seed, body)| {
            same(seed.inverse_mass, body.inverse_mass)
                && same(seed.radius, body.radius)
                && (body.orientation.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 2e-12
        })
        || x.gravity.map(f64::to_bits) != s.gravity.map(f64::to_bits)
        || !same(x.friction, s.friction)
        || !same(x.restitution, s.restitution)
        || o.invalid_at != 0.
        || o.count.fract() != 0.
        || !(0.0..=f64::from(span)).contains(&o.count)
        || x.normal_impulse < 0.
        || o.maximum_normal_impulse < x.normal_impulse
        || o.normal_impulse_sum < o.maximum_normal_impulse
        || !(x.impact_time == -1. || (0.0..=STEP.as_secs_f64()).contains(&x.impact_time))
    {
        return false;
    }
    if x.impact_time == -1.
        && (x.normal_impulse != 0.
            || x.tangential_impulse != [0.; 3]
            || x.contact_normal != [0.; 3])
    {
        return false;
    }
    // A positive impulse in the final internal tick must also be the last
    // event in the block reduction. Rounding the sub-tick offset is device-specific.
    if x.normal_impulse > 0. {
        let expected = f64::from(span - 1) * STEP.as_secs_f64() + x.impact_time;
        if o.count == 0.
            || (o.last_time - expected).abs()
                > 8. * f64::EPSILON * expected.abs().max(STEP.as_secs_f64())
        {
            return false;
        }
    }
    if o.count == 1. && o.normal_impulse_sum != o.maximum_normal_impulse {
        return false;
    }
    if o.count == 0. {
        o.first_time == 0.
            && o.last_time == 0.
            && o.normal_impulse_sum == 0.
            && o.maximum_normal_impulse == 0.
            && o.impulse_on_b == [0.; 3]
    } else {
        o.maximum_normal_impulse > 0.
            && o.first_time >= 0.
            && o.first_time <= o.last_time
            && o.last_time <= f64::from(span) * STEP.as_secs_f64() + f64::EPSILON * 8.
            && (o.count != 1. || o.first_time == o.last_time)
    }
}
