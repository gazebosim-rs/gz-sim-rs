// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Checked, generation-fenced transport for native graphs larger than a worker frame.
use crate::{articulation::ArticulationError, floating_tree::MotorSetting};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
type Result<T> = std::result::Result<T, ArticulationError>;
const fn invalid(message: &'static str) -> ArticulationError {
    ArticulationError::Invalid(message)
}
pub(crate) trait PagedWorld: Sized {
    type Spec;
    type Version: Copy;
    const LIMIT: usize;
    fn restore_bytes(
        spec: &Self::Spec,
        version: Self::Version,
        tick: u64,
        bytes: &[u8],
    ) -> Result<Self>;
    fn capture_bytes(&self, tick: u64) -> Result<Vec<u8>>;
    fn queue(&mut self, id: u64, effort: f64) -> Result<()>;
    fn motor(&mut self, id: u64, setting: Option<MotorSetting>) -> Result<()>;
    fn advance(&mut self, efforts: &[(u64, f64)], forces: &[(u64, [f64; 3])]) -> Result<()>;
    fn private(&mut self, _source: [u8; 32], _generation: u64, _bytes: &[u8]) -> Result<Vec<u8>> {
        Err(invalid("native world has no speculative subinterval"))
    }
    fn finish_private(&mut self, _bytes: &[u8]) -> Result<()> {
        Err(invalid("native world has no speculative finish"))
    }
    fn posture_batch(&mut self, _count: u8, _program: &[u8]) -> Result<Vec<u8>> {
        Err(invalid("native world has no local posture path"))
    }
}
pub(crate) const PAGE: usize = 64 * 1024;
// Bounded below the 192 KiB managed frame to leave room for wire structure and integrity metadata.
const PACKED_LIMIT: usize = 128 * 1024;
#[derive(Serialize, Deserialize)]
pub(crate) struct PackedGraph {
    raw_len: u32,
    digest: [u8; 32],
    #[serde(with = "serde_bytes")]
    data: Vec<u8>,
}
impl PackedGraph {
    pub(crate) fn pack(bytes: &[u8], digest: [u8; 32]) -> Option<Self> {
        if bytes.len() <= PAGE {
            return None;
        }
        let data = lz4_flex::block::compress(bytes);
        (data.len() <= PACKED_LIMIT && data.len() < bytes.len()).then_some(Self {
            raw_len: bytes.len() as u32,
            digest,
            data,
        })
    }
    pub(crate) fn payload(&self) -> (usize, [u8; 32]) {
        (self.data.len(), self.digest)
    }
    pub(crate) fn open(self, limit: usize) -> Result<Vec<u8>> {
        let len = self.raw_len as usize;
        if !(PAGE + 1..=limit).contains(&len)
            || self.data.is_empty()
            || self.data.len() > PACKED_LIMIT
            || self.data.len() >= len
        {
            return Err(invalid("invalid packed native graph dimensions"));
        }
        let mut out = vec![0; len];
        let decoded = lz4_flex::block::decompress_into(&self.data, &mut out)
            .map_err(|_| invalid("malformed packed native graph block"))?;
        if decoded != len || Sha256::digest(&out)[..] != self.digest[..] {
            return Err(invalid("packed native graph length or digest differs"));
        }
        Ok(out)
    }
}

#[derive(Serialize, Deserialize)]
pub(crate) struct Req {
    pub generation: u64,
    pub op: Op,
}
#[derive(Serialize, Deserialize)]
pub(crate) enum Op {
    Stage(Vec<u8>),
    Begin {
        len: u32,
        digest: [u8; 32],
    },
    Append {
        offset: u32,
        bytes: Vec<u8>,
    },
    Finish,
    Activate([u8; 32]),
    Fence([u8; 32]),
    Queue(u64, f64),
    Motor(u64, Option<MotorSetting>),
    Step {
        efforts: Vec<(u64, f64)>,
        forces: Vec<(u64, [f64; 3])>,
    },
    Fetch {
        digest: [u8; 32],
        offset: u32,
    },
    StagePacked(PackedGraph),
    PostureBatch {
        count: u8,
        program: Vec<u8>,
    },
    ReadPosture([u8; 32]),
    Private(Vec<u8>),
    FinishPrivate(Vec<u8>),
}
#[derive(Serialize, Deserialize)]
pub(crate) enum Answer {
    Ack,
    Staged([u8; 32]),
    Inline(Vec<u8>),
    Paged { len: u32, digest: [u8; 32] },
    Page(Vec<u8>),
    Packed(PackedGraph),
    Posture(Vec<u8>),
    Private(Vec<u8>),
}
struct Resident<W> {
    world: W,
    generation: u64,
    tick: u64,
    digest: [u8; 32],
}
struct Upload {
    len: usize,
    digest: [u8; 32],
    generation: u64,
    tick: u64,
    data: Vec<u8>,
}
pub(crate) struct State<W> {
    controlled: bool,
    feedback: Option<([u8; 32], Vec<u8>)>,
    packed: bool,
    staged: Option<Resident<W>>,
    active: Option<Resident<W>>,
    high: Option<u64>,
    upload: Option<Upload>,
    outgoing: Option<Vec<u8>>,
}
impl<W> Default for State<W> {
    fn default() -> Self {
        Self {
            controlled: false,
            feedback: None,
            packed: false,
            staged: None,
            active: None,
            high: None,
            upload: None,
            outgoing: None,
        }
    }
}
impl<W: PagedWorld> State<W> {
    pub(crate) fn with_capabilities(packed: bool, controlled: bool) -> Self {
        Self {
            packed,
            controlled,
            ..Default::default()
        }
    }
    fn permit_stage(&self, generation: u64) -> Result<()> {
        if self.staged.is_some() || self.high.is_some_and(|h| h >= generation) {
            return Err(invalid("stale native graph stage generation"));
        }
        Ok(())
    }
    fn stage(
        &mut self,
        spec: &W::Spec,
        version: W::Version,
        tick: u64,
        generation: u64,
        bytes: Vec<u8>,
    ) -> Result<Answer> {
        self.permit_stage(generation)?;
        let world = W::restore_bytes(spec, version, tick, &bytes)?;
        let digest = Sha256::digest(&bytes).into();
        self.staged = Some(Resident {
            world,
            generation,
            tick,
            digest,
        });
        self.high = Some(generation);
        Ok(Answer::Staged(digest))
    }
    pub(crate) fn handle(
        &mut self,
        spec: &W::Spec,
        version: W::Version,
        tick: u64,
        req: Req,
        steps: &mut u64,
    ) -> Result<Answer> {
        let generation = req.generation;
        match req.op {
            Op::StagePacked(block) => {
                self.permit_stage(generation)?;
                if !self.packed || self.upload.is_some() {
                    return Err(invalid(
                        "packed upload unavailable for native graph session",
                    ));
                }
                self.stage(spec, version, tick, generation, block.open(W::LIMIT)?)
            }
            Op::Stage(bytes) => {
                if self.upload.is_some() || bytes.len() > PAGE {
                    return Err(invalid("invalid inline native graph upload"));
                }
                self.stage(spec, version, tick, generation, bytes)
            }
            Op::Begin { len, digest } => {
                self.permit_stage(generation)?;
                let len = len as usize;
                if self.upload.is_some() || !(PAGE + 1..=W::LIMIT).contains(&len) {
                    return Err(invalid("incorrect native graph upload length"));
                }
                self.upload = Some(Upload {
                    len,
                    digest,
                    generation,
                    tick,
                    data: Vec::with_capacity(len),
                });
                Ok(Answer::Ack)
            }
            Op::Append { offset, bytes } => {
                let u = self
                    .upload
                    .as_mut()
                    .filter(|u| u.generation == generation && u.tick == tick)
                    .ok_or(invalid("stale chunked native graph stage"))?;
                if offset as usize != u.data.len()
                    || bytes.len() != (u.len - u.data.len()).min(PAGE)
                    || bytes.is_empty()
                {
                    return Err(invalid("misordered or truncated native graph chunk"));
                }
                u.data.extend(bytes);
                Ok(Answer::Ack)
            }
            Op::Finish => {
                let u = self
                    .upload
                    .take()
                    .ok_or(invalid("no pending native graph upload"))?;
                if u.generation != generation
                    || u.tick != tick
                    || u.data.len() != u.len
                    || u.digest[..] != Sha256::digest(&u.data)[..]
                {
                    return Err(invalid("native graph upload checksum or length differs"));
                }
                self.stage(spec, version, tick, generation, u.data)
            }
            Op::Activate(digest) => {
                if self.active.is_some()
                    || self.staged.as_ref().is_none_or(|r| {
                        r.generation != generation || r.tick != tick || r.digest != digest
                    })
                {
                    return Err(invalid("stale native graph activation"));
                }
                self.feedback = None;
                self.active = self.staged.take();
                self.outgoing = None;
                Ok(Answer::Ack)
            }
            op => {
                let r = self
                    .active
                    .as_mut()
                    .filter(|r| r.generation == generation && r.tick == tick)
                    .ok_or(invalid("stale native graph capability or clock"))?;
                match op {
                    Op::Private(bytes) if bytes.len() <= PAGE => {
                        let data = r.world.private(r.digest, generation, &bytes)?;
                        if data.len() > PAGE {
                            return Err(invalid("subinterval reply exceeds frame"));
                        }
                        return Ok(Answer::Private(data));
                    }
                    Op::FinishPrivate(bytes) if bytes.len() <= PAGE => {
                        let next = r
                            .tick
                            .checked_add(1)
                            .ok_or(invalid("native subinterval clock exhausted"))?;
                        let total = steps
                            .checked_add(1)
                            .ok_or(invalid("native subinterval counter exhausted"))?;
                        r.world.finish_private(&bytes)?;
                        self.feedback = None;
                        r.tick = next;
                        *steps = total;
                    }
                    Op::ReadPosture(digest) if self.controlled => {
                        let (_, rows) = self
                            .feedback
                            .as_ref()
                            .filter(|(hash, _)| *hash == digest)
                            .ok_or(invalid("stale local posture proof request"))?;
                        return Ok(Answer::Posture(rows.clone()));
                    }
                    Op::PostureBatch { count, program } if self.controlled => {
                        if !(1..=16).contains(&count) || program.len() > PAGE {
                            return Err(invalid("invalid native posture batch size"));
                        }
                        let next = r
                            .tick
                            .checked_add(u64::from(count))
                            .ok_or(invalid("native batch clock exhausted"))?;
                        let total = steps
                            .checked_add(u64::from(count))
                            .ok_or(invalid("native batch step count exhausted"))?;
                        let data = r.world.posture_batch(count, &program)?;
                        if data.len() > PAGE {
                            return Err(invalid("native posture feedback exceeds limit"));
                        }
                        r.tick = next;
                        *steps = total;
                        self.feedback = Some((Sha256::digest(&program).into(), data));
                    }
                    Op::Fence(d) if d == r.digest => {
                        self.feedback = None;
                        self.active = None;
                        self.outgoing = None;
                        return Ok(Answer::Ack);
                    }
                    Op::Fetch { digest, offset } if digest == r.digest => {
                        let bytes = self
                            .outgoing
                            .as_ref()
                            .ok_or(invalid("no native graph pages available"))?;
                        let start = offset as usize;
                        if !start.is_multiple_of(PAGE) || start >= bytes.len() {
                            return Err(invalid("invalid native graph receipt offset"));
                        }
                        return Ok(Answer::Page(
                            bytes[start..bytes.len().min(start + PAGE)].to_vec(),
                        ));
                    }
                    Op::Queue(id, e) => {
                        self.feedback = None;
                        r.world.queue(id, e)?
                    }
                    Op::Motor(id, m) => {
                        self.feedback = None;
                        r.world.motor(id, m)?
                    }
                    Op::Step { efforts, forces } => {
                        self.feedback = None;
                        r.world.advance(&efforts, &forces)?;
                        r.tick = r
                            .tick
                            .checked_add(1)
                            .ok_or(invalid("native graph clock exhausted"))?;
                        *steps = steps
                            .checked_add(1)
                            .ok_or(invalid("native graph worker step count exhausted"))?;
                    }
                    _ => return Err(invalid("invalid native graph capability operation")),
                }
                let bytes = r.world.capture_bytes(r.tick)?;
                r.digest = Sha256::digest(&bytes).into();
                if self.packed
                    && let Some(block) = PackedGraph::pack(&bytes, r.digest)
                {
                    self.outgoing = None;
                    return Ok(Answer::Packed(block));
                }
                if bytes.len() <= PAGE {
                    self.outgoing = None;
                    Ok(Answer::Inline(bytes))
                } else {
                    let len = bytes.len() as u32;
                    self.outgoing = Some(bytes);
                    Ok(Answer::Paged {
                        len,
                        digest: r.digest,
                    })
                }
            }
        }
    }
}

/// Send a whole solver proposal, verifying every acknowledged byte range and the final digest.
pub(crate) fn stage(
    bytes: Vec<u8>,
    limit: usize,
    call: impl FnMut(Op) -> Result<Answer>,
    mut accepted_page: impl FnMut(),
) -> Result<[u8; 32]> {
    stage_encoded(bytes, limit, false, call, |chunk| {
        if matches!(chunk, TransferChunk::Page(_)) {
            accepted_page()
        }
    })
}
#[derive(Copy, Clone, Debug)]
pub(crate) enum TransferChunk {
    Page(usize),
    Packed(usize),
    Inline(usize),
}
pub(crate) fn stage_encoded(
    bytes: Vec<u8>,
    limit: usize,
    packed: bool,
    mut call: impl FnMut(Op) -> Result<Answer>,
    mut accepted: impl FnMut(TransferChunk),
) -> Result<[u8; 32]> {
    if bytes.len() > limit || limit > u32::MAX as usize {
        return Err(invalid("native graph exceeds upload limit"));
    }
    let digest = Sha256::digest(&bytes).into();
    let result = if packed && let Some(block) = PackedGraph::pack(&bytes, digest) {
        let len = block.data.len();
        let ans = call(Op::StagePacked(block))?;
        accepted(TransferChunk::Packed(len));
        ans
    } else if bytes.len() <= PAGE {
        let len = bytes.len();
        let ans = call(Op::Stage(bytes))?;
        accepted(TransferChunk::Inline(len));
        ans
    } else {
        acknowledge(call(Op::Begin {
            len: bytes.len() as u32,
            digest,
        })?)?;
        for (index, chunk) in bytes.chunks(PAGE).enumerate() {
            acknowledge(call(Op::Append {
                offset: (index * PAGE) as u32,
                bytes: chunk.to_vec(),
            })?)?;
            accepted(TransferChunk::Page(chunk.len()));
        }
        call(Op::Finish)?
    };
    if !matches!(result,Answer::Staged(d) if d==digest) {
        return Err(invalid("native graph upload returned different digest"));
    }
    Ok(digest)
}
pub(crate) fn acknowledge(answer: Answer) -> Result<()> {
    if matches!(answer, Answer::Ack) {
        Ok(())
    } else {
        Err(invalid("native graph acknowledgement missing"))
    }
}
/// Fetch a bounded receipt from a private worker proposal. No page is visible until verification.
pub(crate) fn receive(
    answer: Answer,
    limit: usize,
    call: impl FnMut(Op) -> Result<Answer>,
    mut accepted_page: impl FnMut(),
) -> Result<Vec<u8>> {
    receive_encoded(answer, limit, false, call, |chunk| {
        if matches!(chunk, TransferChunk::Page(_)) {
            accepted_page()
        }
    })
}
pub(crate) fn receive_encoded(
    answer: Answer,
    limit: usize,
    packed: bool,
    mut call: impl FnMut(Op) -> Result<Answer>,
    mut accepted: impl FnMut(TransferChunk),
) -> Result<Vec<u8>> {
    match answer {
        Answer::Packed(block) if packed => {
            let len = block.data.len();
            let bytes = block.open(limit)?;
            accepted(TransferChunk::Packed(len));
            Ok(bytes)
        }
        Answer::Inline(b) if b.len() <= PAGE && b.len() <= limit => {
            accepted(TransferChunk::Inline(b.len()));
            Ok(b)
        }
        Answer::Paged { len, digest } if (PAGE + 1..=limit).contains(&(len as usize)) => {
            let len = len as usize;
            let mut b = Vec::with_capacity(len);
            while b.len() < len {
                let Answer::Page(page) = call(Op::Fetch {
                    digest,
                    offset: b.len() as u32,
                })?
                else {
                    return Err(invalid("expected native graph receipt page"));
                };
                if page.len() != (len - b.len()).min(PAGE) {
                    return Err(invalid("truncated native graph receipt page"));
                }
                accepted(TransferChunk::Page(page.len()));
                b.extend(page);
            }
            if Sha256::digest(&b)[..] != digest[..] {
                return Err(invalid("native graph receipt checksum differs"));
            }
            Ok(b)
        }
        _ => Err(invalid("invalid native graph receipt descriptor")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn packed_native_envelope_enforces_allocation_length_digest_and_protocol() {
        let src: Vec<_> = (0..3 * PAGE).map(|i| ((i >> 8) & 255) as u8).collect();
        let digest = Sha256::digest(&src).into();
        let block = || PackedGraph::pack(&src, digest).unwrap();
        assert_eq!(block().open(src.len()).unwrap(), src);
        assert!(block().open(src.len() - 1).is_err());
        for case in 0..6 {
            let mut b = block();
            match case {
                0 => b.raw_len = u32::MAX,
                1 => b.raw_len -= 1,
                2 => b.digest[7] ^= 1,
                3 => b.data.truncate(2),
                4 => b.data.resize(PACKED_LIMIT + 1, 0),
                _ => b.data.clear(),
            }
            assert!(b.open(src.len()).is_err());
        }
        let a = Answer::Packed(block());
        assert!(receive(a, src.len(), |_| unreachable!(), || unreachable!()).is_err());
        // A maximal-width permitted graph of random bytes still fits the original paged path.
        let mut x = 43u32;
        let random = (0..4 * PAGE)
            .map(|_| {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                x as u8
            })
            .collect::<Vec<_>>();
        assert!(PackedGraph::pack(&random, Sha256::digest(&random).into()).is_none());
    }
}
