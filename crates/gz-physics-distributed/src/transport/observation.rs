// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Self-contained little-endian observation for a negotiated Zenoh reply.
//! No deltas depend on a previous message. Replay and duplicate requests thus
//! use exactly the same decoder as the first observation from a fresh worker.
use super::{Reply, Snapshot, TransportError, wire};
use crate::{
    physics::{
        Body, ComponentInfo, ContactImpulse, GroundPlane, GroundSupport, NativeSolverProfile,
        MAX_PARTITION_BODIES, Primitive, SimulationStep, Tier,
    },
    protocol::{Observation, Outcome},
};
use std::collections::BTreeMap;
const MAGIC: &[u8; 5] = b"GZOB\x01";
const EXTENDED_MAGIC: &[u8; 5] = b"GZOB\x02";
const POINT32_MAGIC: &[u8; 5] = b"GZOB\x03";
const POINT64_MAGIC: &[u8; 5] = b"GZOB\x04";

struct Encoding {
    parameter: &'static str,
    selector: &'static str,
    helper: &'static str,
    magic: &'static [u8; 5],
}
macro_rules! encoding {
    ($parameter:literal,$magic:expr) => {
        Encoding {
            parameter: $parameter,
            selector: concat!($parameter, "=1"),
            helper: concat!($parameter, "=1;gz_world_helper=1"),
            magic: $magic,
        }
    };
}
fn encoding(p: NativeSolverProfile) -> Encoding {
    match p {
        NativeSolverProfile::Standard32 => encoding!("gz_observation_v1", MAGIC),
        NativeSolverProfile::Extended64 => encoding!("gz_observation_v2", EXTENDED_MAGIC),
        NativeSolverProfile::Standard32PerPoint => encoding!("gz_observation_v3", POINT32_MAGIC),
        NativeSolverProfile::Extended64PerPoint => encoding!("gz_observation_v4", POINT64_MAGIC),
    }
}
pub(super) fn parameter(profile: NativeSolverProfile) -> &'static str {
    encoding(profile).parameter
}
pub(super) fn selector(profile: NativeSolverProfile, helper: bool) -> &'static str {
    let format = encoding(profile);
    if helper {
        format.helper
    } else {
        format.selector
    }
}
fn magic(profile: NativeSolverProfile) -> &'static [u8; 5] {
    encoding(profile).magic
}
fn max_pairs(profile: NativeSolverProfile) -> usize {
    MAX_PARTITION_BODIES * (profile.max_bodies() - 1) / 2
}

struct Reader<'a>(&'a [u8]);
impl Reader<'_> {
    fn bytes<const N: usize>(&mut self) -> Result<[u8; N], TransportError> {
        let (v, rest) = self
            .0
            .split_first_chunk()
            .ok_or_else(|| wire("incomplete packed observation"))?;
        self.0 = rest;
        Ok(*v)
    }
    fn byte(&mut self) -> Result<u8, TransportError> {
        Ok(self.bytes::<1>()?[0])
    }
    fn flag(&mut self) -> Result<bool, TransportError> {
        match self.byte()? {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(wire("bad observation flag")),
        }
    }
    fn count(&mut self, limit: usize) -> Result<usize, TransportError> {
        let n = usize::from(u16::from_le_bytes(self.bytes()?));
        if n > limit {
            return Err(wire("packed section exceeds its physical limit"));
        }
        Ok(n)
    }
    fn id(&mut self) -> Result<u64, TransportError> {
        Ok(u64::from_le_bytes(self.bytes()?))
    }
    fn number(&mut self) -> Result<f64, TransportError> {
        let n = f64::from_le_bytes(self.bytes()?);
        if n.is_finite() {
            Ok(n)
        } else {
            Err(wire("nonfinite physics observation"))
        }
    }
}
fn count(out: &mut Vec<u8>, n: usize, max: usize) -> Result<(), TransportError> {
    if n > max {
        return Err(wire("worker observation exceeds its physical limit"));
    }
    out.extend_from_slice(&(n as u16).to_le_bytes());
    Ok(())
}
fn write_body(out: &mut Vec<u8>, b: &Body, generation: u64) {
    out.extend_from_slice(&b.id.to_le_bytes());
    out.extend_from_slice(&generation.to_le_bytes());
    let tag = match b.tier {
        Tier::Prescribed => 0,
        Tier::Dynamic => 1,
        Tier::Fixed => 2,
    };
    let geometry = match b.primitive {
        Primitive::Box => 0,
        Primitive::Sphere => 1,
        Primitive::Capsule => 2,
        Primitive::Cylinder => 3,
    };
    out.push(tag | (geometry << 2));
    let [sx, sy, sz] = b.size;
    let [x, y, z] = b.position;
    let [qx, qy, qz, qw] = b.orientation;
    let [vx, vy, vz] = b.linear;
    let [wx, wy, wz] = b.angular;
    let field = [
        sx, sy, sz, b.mass, b.friction, x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz,
    ];
    // Positive zero is the only omitted bit pattern, including for orientations.
    let mut mask = 0u32;
    for (i, v) in field.iter().enumerate() {
        if v.to_bits() != 0 {
            mask |= 1 << i;
        }
    }
    out.extend_from_slice(&mask.to_le_bytes());
    for v in field {
        if v.to_bits() != 0 {
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
}
fn read_body(r: &mut Reader<'_>) -> Result<(Body, u64), TransportError> {
    let id = r.id()?;
    let generation = r.id()?;
    let tag = r.byte()?;
    let primitive = match tag >> 2 {
        0 => Primitive::Box,
        1 => Primitive::Sphere,
        2 => Primitive::Capsule,
        3 => Primitive::Cylinder,
        _ => return Err(wire("unknown packed primitive")),
    };
    let tier = match tag & 3 {
        0 => Tier::Prescribed,
        1 => Tier::Dynamic,
        2 => Tier::Fixed,
        _ => return Err(wire("unknown packed body type")),
    };
    let mask = u32::from_le_bytes(r.bytes()?);
    if mask >> 18 != 0 {
        return Err(wire("unknown body field bits"));
    }
    let mut field = [0.; 18];
    for (i, v) in field.iter_mut().enumerate() {
        if mask & (1 << i) != 0 {
            *v = r.number()?;
        }
    }
    let [
        sx,
        sy,
        sz,
        mass,
        friction,
        x,
        y,
        z,
        qx,
        qy,
        qz,
        qw,
        vx,
        vy,
        vz,
        wx,
        wy,
        wz,
    ] = field;
    Ok((
        Body {
            id,
            primitive,
            tier,
            size: [sx, sy, sz],
            mass,
            friction,
            position: [x, y, z],
            orientation: [qx, qy, qz, qw],
            linear: [vx, vy, vz],
            angular: [wx, wy, wz],
        },
        generation,
    ))
}
/// Return None for a receipt carrying anything other than a complete observation.
pub(super) fn encode_with_profile(
    reply: &Reply,
    profile: NativeSolverProfile,
) -> Result<Option<Vec<u8>>, TransportError> {
    let (kind, obs) = match &reply.outcome {
        Outcome::State(o) => (0, o),
        Outcome::Activated(o) => (1, o),
        Outcome::WarmActivated(o) => (2, o),
        _ => return Ok(None),
    };
    let s = &obs.snapshot;
    if s.contacts != s.impulses.len() || obs.generations.len() != s.bodies.len() {
        return Err(wire("inconsistent worker observation"));
    }
    let mut out = Vec::with_capacity(64 + 180 * s.bodies.len());
    out.extend_from_slice(magic(profile));
    out.push(kind);
    out.extend_from_slice(&reply.run_id.to_le_bytes());
    out.push(reply.worker_id);
    out.extend_from_slice(&reply.sequence.to_le_bytes());
    out.extend_from_slice(&s.tick.to_le_bytes());
    out.push(s.step.duration().as_millis() as u8);
    match s.ground {
        None => out.push(0),
        Some(g) if g.horizontal() => {
            out.push(1);
            out.extend_from_slice(&g.height().to_le_bytes());
            out.extend_from_slice(&g.friction().to_le_bytes());
        }
        Some(g) => {
            out.push(2);
            for v in g
                .point()
                .into_iter()
                .chain(g.normal())
                .chain([g.friction()])
            {
                out.extend_from_slice(&v.to_le_bytes());
            }
        }
    }
    out.push(u8::from(s.contact_history));
    count(&mut out, s.bodies.len(), MAX_PARTITION_BODIES)?;
    for b in &s.bodies {
        write_body(
            &mut out,
            b,
            *obs.generations
                .get(&b.id)
                .ok_or_else(|| wire("missing observed generation"))?,
        );
    }
    count(&mut out, obs.components.len(), s.bodies.len())?;
    for c in &obs.components {
        out.push(u8::from(c.contact_history));
        count(&mut out, c.ids.len(), profile.max_bodies())?;
        for id in &c.ids {
            out.extend_from_slice(&id.to_le_bytes());
        }
    }
    count(&mut out, s.impulses.len(), max_pairs(profile))?;
    for p in &s.impulses {
        out.extend_from_slice(&p.a.to_le_bytes());
        out.extend_from_slice(&p.b.to_le_bytes());
        out.extend_from_slice(&p.normal_impulse.to_le_bytes());
    }
    count(&mut out, s.supports.len(), s.bodies.len())?;
    for p in &s.supports {
        out.extend_from_slice(&p.body.to_le_bytes());
        out.extend_from_slice(&p.normal_impulse.to_le_bytes());
    }
    Ok(Some(out))
}
#[cfg(test)]
pub(super) fn encode(reply: &Reply) -> Result<Option<Vec<u8>>, TransportError> {
    encode_with_profile(reply, NativeSolverProfile::Standard32)
}
#[cfg(test)]
pub(super) fn decode(bytes: &[u8]) -> Result<Reply, TransportError> {
    decode_with_profile(bytes, NativeSolverProfile::Standard32)
}
pub(super) fn is_packed(bytes: &[u8]) -> bool {
    [MAGIC, EXTENDED_MAGIC, POINT32_MAGIC, POINT64_MAGIC]
        .iter()
        .any(|&prefix| bytes.starts_with(prefix))
}
pub(super) fn decode_with_profile(
    bytes: &[u8],
    profile: NativeSolverProfile,
) -> Result<Reply, TransportError> {
    let mut r = Reader(
        bytes
            .strip_prefix(magic(profile))
            .ok_or_else(|| wire("unknown observation format"))?,
    );
    let kind = r.byte()?;
    let run_id = r.id()?;
    let worker_id = r.byte()?;
    let sequence = r.id()?;
    let tick = r.id()?;
    let step = SimulationStep::from_duration(std::time::Duration::from_millis(r.byte()?.into()))
        .ok_or_else(|| wire("unknown packed time step"))?;
    let ground = match r.byte()? {
        0 => None,
        1 => Some(GroundPlane::with_friction(r.number()?, r.number()?).map_err(wire)?),
        2 => {
            let point = [r.number()?, r.number()?, r.number()?];
            let normal = [r.number()?, r.number()?, r.number()?];
            let plane = GroundPlane::oriented(point, normal, r.number()?).map_err(wire)?;
            if plane.horizontal() {
                return Err(wire("noncanonical oriented ground"));
            }
            Some(plane)
        }
        _ => return Err(wire("unknown ground type")),
    };
    let contact_history = r.flag()?;
    let n = r.count(MAX_PARTITION_BODIES)?;
    let mut bodies = Vec::with_capacity(n);
    let mut generations = BTreeMap::new();
    for _ in 0..n {
        let (b, generation) = read_body(&mut r)?;
        if generations.insert(b.id, generation).is_some() {
            return Err(wire("duplicate packed body"));
        }
        bodies.push(b);
    }
    let ncomp = r.count(n)?;
    let mut components = Vec::with_capacity(ncomp);
    let mut remaining = n;
    for _ in 0..ncomp {
        let history = r.flag()?;
        let members = r.count(remaining.min(profile.max_bodies()))?;
        if members == 0 {
            return Err(wire("empty packed component"));
        }
        remaining -= members;
        let ids = (0..members)
            .map(|_| r.id())
            .collect::<Result<Vec<_>, _>>()?;
        components.push(ComponentInfo {
            ids,
            contact_history: history,
        });
    }
    if remaining != 0 {
        return Err(wire("missing packed component members"));
    }
    let contacts = r.count(max_pairs(profile).min(n * n.saturating_sub(1) / 2))?;
    let mut impulses = Vec::with_capacity(contacts);
    for _ in 0..contacts {
        impulses.push(ContactImpulse {
            a: r.id()?,
            b: r.id()?,
            normal_impulse: r.number()?,
        });
    }
    let nsupports = r.count(n)?;
    let mut supports = Vec::with_capacity(nsupports);
    for _ in 0..nsupports {
        supports.push(GroundSupport {
            body: r.id()?,
            normal_impulse: r.number()?,
        });
    }
    if !r.0.is_empty() {
        return Err(wire("trailing packed observation data"));
    }
    let o = Observation {
        snapshot: Snapshot {
            tick,
            step,
            ground,
            contact_history,
            bodies,
            contacts,
            impulses,
            supports,
        },
        components,
        generations,
    };
    let outcome = match kind {
        0 => Outcome::State(o),
        1 => Outcome::Activated(o),
        2 => Outcome::WarmActivated(o),
        _ => return Err(wire("unknown packed observation receipt")),
    };
    Ok(Reply {
        run_id,
        worker_id,
        sequence,
        outcome,
    })
}
