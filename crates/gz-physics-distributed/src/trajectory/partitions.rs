// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent one-way materials on local process leases. Every participating process receives
//! the same warm native source. A single validated native graph is published at the barrier.
use super::*;
use bevy_tasks::{TaskPool, TaskPoolBuilder};
use gz_physics_xpbd::{OneWayState, SoftBody};

type ShardResult = (usize, TrajectoryWorkerStep, Vec<u8>, Vec<OneWayState>);
struct Shard {
    rows: Vec<usize>,
    pool: TrajectoryWorkers,
}

/// Multiprocess ownership for an explicitly requested one-way material scene.
pub struct MaterialPartitions {
    shards: Vec<Shard>,
    join: TaskPool,
    identities: Vec<Hash>,
    surfaces: usize,
    last: Vec<TrajectoryWorkerStep>,
    aggregate: Option<TrajectoryWorkerStep>,
}
impl MaterialPartitions {
    pub fn new(
        worker: &Path,
        width: usize,
        native: &MultiFidelityBackend,
        recipe: material::MaterialRecipe,
        processes: u8,
        threads: u8,
    ) -> Result<Self> {
        Self::with_transport(
            worker,
            width,
            native,
            recipe,
            processes,
            threads,
            WorkerTransport::Pipe,
        )
    }
    pub fn with_transport(
        worker: &Path,
        width: usize,
        native: &MultiFidelityBackend,
        recipe: material::MaterialRecipe,
        processes: u8,
        threads: u8,
        transport: WorkerTransport,
    ) -> Result<Self> {
        if !(2..=4).contains(&processes) {
            return Err(invalid("partitioned material needs two to four processes"));
        }
        let identities = recipe.identities()?;
        let surfaces = recipe.surface_count();
        let groups = recipe.partition(usize::from(processes))?;
        let join = TaskPoolBuilder::new()
            .num_threads(groups.len() - 1)
            .thread_name("gz material exchange".into())
            .build();
        let shards = groups
            .into_iter()
            .map(|(rows, recipe)| {
                Ok(Shard {
                    rows,
                    pool: TrajectoryWorkers::with_material_transport(
                        worker, width, native, recipe, threads, transport,
                    )?,
                })
            })
            .collect::<Result<Vec<_>>>()?;
        Ok(Self {
            shards,
            join,
            identities,
            surfaces,
            last: Vec::new(),
            aggregate: None,
        })
    }
    pub fn owner(&self) -> usize {
        self.shards[0].pool.owner()
    }
    pub fn workers(&self) -> usize {
        self.shards[0].pool.workers()
    }
    pub fn owners(&self) -> Vec<usize> {
        self.shards.iter().map(|s| s.pool.owner()).collect()
    }
    pub fn pid(&self, group: usize, slot: usize) -> Option<u32> {
        self.shards.get(group)?.pool.pid(slot)
    }
    pub fn set_warm_budget(&mut self, floor: Option<Duration>) -> Result<()> {
        for s in &mut self.shards {
            s.pool.set_warm_budget(floor)?;
        }
        Ok(())
    }
    pub fn set_packed_replies(&mut self, enabled: bool) {
        for shard in &mut self.shards {
            shard.pool.set_packed_replies(enabled);
        }
    }
    pub fn transports(&self) -> Vec<WorkerTransport> {
        self.shards.iter().map(|s| s.pool.transport()).collect()
    }
    pub fn set_transport(&mut self, group: usize, transport: WorkerTransport) -> Result<()> {
        self.shards
            .get_mut(group)
            .ok_or(invalid("unknown material partition"))?
            .pool
            .set_transport(transport);
        Ok(())
    }
    pub fn last(&self) -> Option<TrajectoryWorkerStep> {
        self.aggregate
    }
    pub fn last_shards(&self) -> &[TrajectoryWorkerStep] {
        &self.last
    }
    pub fn set_profiling(&mut self, enabled: bool) {
        for s in &mut self.shards {
            s.pool.set_profiling(enabled);
        }
    }
    pub fn select(&mut self, slot: usize) -> Result<()> {
        if slot >= self.workers() {
            return Err(invalid("material owner outside the bounded pool"));
        }
        for s in &mut self.shards {
            s.pool.select(slot)?;
        }
        Ok(())
    }
    pub fn select_group(&mut self, group: usize, slot: usize) -> Result<()> {
        self.shards
            .get_mut(group)
            .ok_or(invalid("unknown material partition"))?
            .pool
            .select(slot)
    }
    pub fn reset(&mut self) {
        for s in &mut self.shards {
            s.pool.reset();
        }
        self.last.clear();
        self.aggregate = None;
    }
    pub fn advance(
        &mut self,
        native: &mut MultiFidelityBackend,
        bodies: &mut [&mut SoftBody],
        tick: u64,
        dt: Duration,
    ) -> Result<TrajectoryWorkerStep> {
        let start = Instant::now();
        let ns = u64::try_from(dt.as_nanos())
            .map_err(|_| invalid("material interval overflows nanoseconds"))?;
        if ns == 0 || bodies.len() != self.identities.len() {
            return Err(invalid(
                "invalid partitioned material interval or population",
            ));
        }
        for (b, id) in bodies.iter().zip(&self.identities) {
            if b.one_way_identity()? != *id {
                return Err(invalid(
                    "partitioned material laws differ from their public rows",
                ));
            }
        }
        let native_source = native.checkpoint_trajectory()?;
        let mut sources = Vec::with_capacity(self.shards.len());
        for s in &self.shards {
            let states = s
                .rows
                .iter()
                .map(|&i| bodies[i].one_way_state())
                .collect::<std::result::Result<Vec<_>, _>>()?;
            sources.push(material::pack_scene_state(native_source.clone(), states)?);
        }
        let prepared = Instant::now();
        let results = self.join.scope_with_executor(false, None, |scope| {
            for (group, (s, source)) in self.shards.iter_mut().zip(sources).enumerate() {
                let work = async move {
                    let now = Instant::now();
                    let (r, restarted) = s.pool.request(tick, ns, &source, hash(&source))?;
                    let slot = s.pool.owner();
                    let wall = now.elapsed();
                    let stats = TrajectoryWorkerStep {
                        slot,
                        pid: s.pool.pid(slot).expect("successful leased exchange"),
                        restarted,
                        warm_deadline: r.budget,
                        warm_timeout: r.timed_out,
                        transferred_source: r.sent_source,
                        source_bytes: if r.sent_source { source.len() } else { 0 },
                        successor_bytes: r.bytes.len(),
                        successor_payload_bytes: r.payload_bytes,
                        wall,
                        profile: r
                            .stages
                            .map(|(solve, snapshot, threads)| TrajectoryProfile {
                                capture: Duration::ZERO,
                                exchange: wall,
                                install: Duration::ZERO,
                                child_solve: Duration::from_nanos(solve),
                                child_snapshot: Duration::from_nanos(snapshot),
                                material_threads: usize::from(threads),
                                packing_attempts: usize::from(r.packing_attempted),
                            }),
                    };
                    let (native, soft) = material::unpack_scene_state(&r.bytes)?;
                    Ok::<_, ArticulationError>((group, stats, native, soft))
                };
                if group == 0 {
                    scope.spawn_on_scope(work);
                } else {
                    scope.spawn(work);
                }
            }
        });
        let returned = Instant::now();
        let results = results.into_iter().collect::<Result<Vec<_>>>()?;
        let stats = publish(
            native,
            bodies,
            self.surfaces,
            self.shards.iter().map(|s| s.rows.as_slice()),
            results,
        )?;
        let end = Instant::now();
        let profile = if stats.iter().all(|s| s.profile.is_some()) {
            let mut p = TrajectoryProfile {
                capture: prepared - start,
                exchange: returned - prepared,
                install: end - returned,
                ..Default::default()
            };
            for row in &stats {
                let s = row.profile.unwrap();
                p.child_solve = p.child_solve.max(s.child_solve);
                p.child_snapshot = p.child_snapshot.max(s.child_snapshot);
                p.material_threads += s.material_threads;
                p.packing_attempts += s.packing_attempts;
            }
            Some(p)
        } else {
            None
        };
        let report = TrajectoryWorkerStep {
            slot: stats[0].slot,
            pid: stats[0].pid,
            restarted: stats.iter().any(|s| s.restarted),
            warm_deadline: stats.iter().filter_map(|s| s.warm_deadline).max(),
            warm_timeout: stats.iter().any(|s| s.warm_timeout),
            transferred_source: stats.iter().any(|s| s.transferred_source),
            source_bytes: stats.iter().map(|s| s.source_bytes).sum(),
            successor_bytes: stats.iter().map(|s| s.successor_bytes).sum(),
            successor_payload_bytes: stats.iter().map(|s| s.successor_payload_bytes).sum(),
            wall: end - start,
            profile,
        };
        self.last = stats;
        self.aggregate = Some(report);
        Ok(report)
    }
}
fn publish<'a>(
    native: &mut MultiFidelityBackend,
    bodies: &mut [&mut SoftBody],
    surfaces: usize,
    placements: impl ExactSizeIterator<Item = &'a [usize]>,
    mut results: Vec<ShardResult>,
) -> Result<Vec<TrajectoryWorkerStep>> {
    // Receipt arrival order is unrelated to the order of a material in the authored world.
    // Finish all physical/identity checks while every successor is still private.
    if results.len() != placements.len() {
        return Err(invalid("incomplete material barrier"));
    }
    results.sort_unstable_by_key(|v| v.0);
    let mut merged: Vec<Option<OneWayState>> = (0..bodies.len()).map(|_| None).collect();
    let mut common = None;
    let mut stats = Vec::with_capacity(results.len());
    for (index, (rows, (group, report, image, states))) in placements.zip(results).enumerate() {
        if index != group {
            return Err(invalid("duplicate or unknown material group"));
        }
        if let Some(expected) = &common {
            if expected != &image {
                return Err(invalid(
                    "material workers disagree on their shared native successor",
                ));
            }
        } else {
            common = Some(image);
        }
        if states.len() != rows.len() {
            return Err(invalid("material worker returned the wrong population"));
        }
        for (&i, state) in rows.iter().zip(states) {
            if state.reactions.len() != surfaces {
                return Err(invalid(
                    "material reaction count differs from its construction",
                ));
            }
            bodies[i].validate_one_way_state(&state)?;
            merged[i] = Some(state);
        }
        stats.push(report);
    }
    if merged.iter().any(Option::is_none) {
        return Err(invalid("incomplete material barrier"));
    }
    native.install_trajectory(
        common
            .as_deref()
            .ok_or(invalid("missing native material result"))?,
    )?;
    for (b, state) in bodies.iter_mut().zip(merged) {
        b.install_one_way_state(state.unwrap())
            .expect("validated before the atomic native commit");
    }
    Ok(stats)
}

#[cfg(test)]
mod tests;
