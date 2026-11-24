// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::DynamicBackend;
enum Graph {
    Rigid(MultiFidelityBackend),
    Material(material::World),
}
impl Graph {
    fn new(
        bytes: &[u8],
        material: bool,
        threads: u8,
        executor: material::Executor,
    ) -> Result<Self> {
        Ok(if material {
            Self::Material(material::World::new(bytes, usize::from(threads), executor)?)
        } else {
            Self::Rigid(MultiFidelityBackend::from_trajectory_construction(bytes)?)
        })
    }
    fn threads(&self) -> u8 {
        match self {
            Self::Rigid(_) => 1,
            Self::Material(w) => w.threads() as u8,
        }
    }
    fn snapshot(&self) -> Result<Vec<u8>> {
        match self {
            Self::Rigid(b) => Ok(b.checkpoint_trajectory()?),
            Self::Material(w) => w.snapshot(),
        }
    }
    fn restore(&mut self, image: &[u8]) -> Result<()> {
        match self {
            Self::Rigid(b) => Ok(b.restore_trajectory(image)?),
            Self::Material(w) => w.restore(image),
        }
    }
    fn step(&mut self, dt: Duration) -> Result<()> {
        match self {
            Self::Rigid(b) => Ok(b.step(dt)?),
            Self::Material(w) => w.step(dt),
        }
    }
}
/// An incompressible graph is retried at bounded intervals. Large size changes and a
/// new native source reopen the opportunity immediately.
#[derive(Default)]
struct PackingProbe {
    previous: usize,
    skip: u8,
    interval: u8,
}
impl PackingProbe {
    fn ready(&mut self, size: usize) -> bool {
        if size.abs_diff(self.previous) > self.previous / 8 {
            self.skip = 0;
            self.interval = 0;
        }
        self.previous = size;
        if size <= PAGE {
            return false;
        }
        if self.skip > 0 {
            self.skip -= 1;
            return false;
        }
        true
    }
    fn result(&mut self, fit: bool) {
        self.interval = if fit {
            0
        } else {
            (self.interval * 2 + 1).min(15)
        };
        self.skip = self.interval;
    }
}
#[cfg(test)]
mod packing_tests {
    use super::*;
    #[test]
    fn a_non_fitting_graph_is_reprobed_and_size_changes_reopen_immediately() {
        let mut probe = PackingProbe::default();
        assert!(!probe.ready(PAGE));
        let mut attempts = Vec::new();
        for tick in 0..33 {
            if probe.ready(4 * PAGE) {
                attempts.push(tick);
                probe.result(false);
            }
        }
        assert_eq!(attempts, [0, 2, 6, 14, 30]);
        assert!(probe.ready(3 * PAGE));
        probe.result(true);
        assert!(probe.ready(3 * PAGE));
    }
}
struct Transfer {
    kind: Blob,
    size: usize,
    digest: Hash,
    data: Vec<u8>,
}
/// Arm the Linux parent-death contract when supervising a managed trajectory worker.
/// A launcher which forks another solver establishes a new direct-parent contract for it.
pub fn arm_launch_owner() -> io::Result<()> {
    crate::moving::worker::lifetime::arm()
}
/// Serve one leased, same-build native graph. Requests and receipts use the same bounded,
/// sequenced, digested transport as the floating articulation workers.
pub fn serve(slot: u8, lease: &Path) -> io::Result<()> {
    serve_graph(slot, lease, false)
}
pub fn serve_material(slot: u8, lease: &Path) -> io::Result<()> {
    serve_graph(slot, lease, true)
}
fn serve_graph(slot: u8, lease: &Path, material: bool) -> io::Result<()> {
    arm_launch_owner()?;
    let magic = if material { MATERIAL_MAGIC } else { MAGIC };
    crate::moving::worker::serve_profile(slot, lease, magic, |recipe_hash: Hash| {
        let mut threads = 1u8;
        let mut packing = PackingProbe::default();
        let executor = material::Executor::default();
        let mut recipe: Option<Vec<u8>> = None;
        let mut body: Option<Graph> = None;
        let mut latest: Option<(Hash, Vec<u8>)> = None;
        let mut transfer: Option<Transfer> = None;
        Ok(move |_, cmd, steps: &mut u64| -> Result<Reply> {
            let profile = matches!(
                cmd,
                Operation::ProfiledStep { .. } | Operation::PackedStep { profile: true, .. }
            );
            let packed = matches!(cmd, Operation::PackedStep { .. });
            match cmd {
                Operation::SetMaterialThreadLimit(n) => {
                    if !material || recipe.is_some() || transfer.is_some() || !(1..=4).contains(&n)
                    {
                        return Err(invalid("invalid material thread budget"));
                    }
                    threads = n;
                    Ok(Reply::Ok)
                }
                Operation::Start(kind, n, digest) => {
                    if n == 0
                        || n as usize > MAX
                        || kind == Blob::Recipe && (recipe.is_some() || recipe_hash != digest)
                        || kind == Blob::State && recipe.is_none()
                    {
                        return Err(invalid("invalid curve image offer"));
                    }
                    transfer = Some(Transfer {
                        kind,
                        size: n as usize,
                        digest,
                        data: Vec::with_capacity(n as usize),
                    });
                    Ok(Reply::Ok)
                }
                Operation::Write(offset, page) => {
                    let t = transfer
                        .as_mut()
                        .ok_or(invalid("missing curve image offer"))?;
                    if offset as usize != t.data.len()
                        || page.is_empty()
                        || page.len() != PAGE.min(t.size - t.data.len())
                    {
                        return Err(invalid("incorrect curve transfer offset"));
                    }
                    t.data.extend_from_slice(&page);
                    Ok(Reply::Ok)
                }
                Operation::Finish => {
                    let t = transfer.take().ok_or(invalid("missing curve transfer"))?;
                    if t.data.len() != t.size || hash(&t.data) != t.digest {
                        return Err(invalid("curve transfer integrity error"));
                    }
                    packing = PackingProbe::default();
                    if t.kind == Blob::Recipe {
                        let b = Graph::new(&t.data, material, threads, executor.clone())?;
                        let src = b.snapshot()?;
                        latest = Some((hash(&src), src));
                        body = Some(b);
                        recipe = Some(t.data);
                    } else {
                        let mut b = Graph::new(
                            recipe
                                .as_ref()
                                .ok_or(invalid("missing curve construction"))?,
                            material,
                            threads,
                            executor.clone(),
                        )?;
                        b.restore(&t.data)?;
                        latest = Some((t.digest, t.data));
                        body = Some(b);
                    }
                    Ok(Reply::Ok)
                }
                Operation::Step { source, dt_ns }
                | Operation::ProfiledStep { source, dt_ns }
                | Operation::PackedStep { source, dt_ns, .. } => {
                    if latest.as_ref().is_none_or(|(digest, _)| *digest != source) {
                        return Ok(Reply::NeedState);
                    }
                    // Only a successfully captured graph may be offered as the source of a later step.
                    latest = None;
                    let b = body
                        .as_mut()
                        .ok_or(invalid("missing native curved graph"))?;
                    let start = profile.then(Instant::now);
                    b.step(Duration::from_nanos(dt_ns))?;
                    let solved = profile.then(Instant::now);
                    let out = b.snapshot()?;
                    let digest = hash(&out);
                    let bytes = out.len() as u32;
                    // Reuse the single-frame envelope already used by native articulation workers.
                    let attempted = packed && packing.ready(out.len());
                    if attempted
                        && let Some(block) = crate::paged_native::PackedGraph::pack(&out, digest)
                    {
                        packing.result(true);
                        let stages = start.zip(solved).map(|(a, bx)| {
                            (
                                (bx - a).as_nanos() as u64,
                                bx.elapsed().as_nanos() as u64,
                                b.threads(),
                            )
                        });
                        latest = Some((digest, out));
                        *steps += 1;
                        return Ok(Reply::Packed { block, stages });
                    }
                    if attempted {
                        packing.result(false);
                    }
                    let first = out[..out.len().min(PAGE)].to_vec();
                    let reply = if let Some((start, solved)) = start.zip(solved) {
                        let solve_ns = (solved - start).as_nanos() as u64;
                        let snapshot_ns = solved.elapsed().as_nanos() as u64;
                        if packed {
                            Reply::PackedFallbackProfile {
                                bytes,
                                digest,
                                first,
                                stages: (solve_ns, snapshot_ns, b.threads()),
                                attempted,
                            }
                        } else if threads > 1 {
                            Reply::BatchProfile {
                                bytes,
                                digest,
                                first,
                                solve_ns,
                                snapshot_ns,
                                threads: b.threads(),
                            }
                        } else {
                            Reply::ProfiledSuccess {
                                bytes,
                                digest,
                                first,
                                solve_ns,
                                snapshot_ns,
                            }
                        }
                    } else {
                        Reply::Success {
                            bytes,
                            digest,
                            first,
                        }
                    };
                    latest = Some((digest, out));
                    *steps += 1;
                    Ok(reply)
                }
                Operation::Read(offset) => {
                    let (_, image) = latest
                        .as_ref()
                        .ok_or(invalid("missing committed curve image"))?;
                    let off = offset as usize;
                    if off >= image.len() || !off.is_multiple_of(PAGE) {
                        return Err(invalid("invalid curve read offset"));
                    }
                    Ok(Reply::Page(
                        image[off..image.len().min(off + PAGE)].to_vec(),
                    ))
                }
            }
        })
    })
}
