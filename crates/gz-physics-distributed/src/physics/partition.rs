// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent native solvers on one worker, formed before the first possible contact.
use super::*;
mod edits;
mod image;
mod join;
mod ground_patch;
pub use ground_patch::GroundContactPatch;
mod lanes;
mod schedule;
#[cfg(feature = "lane-trace")]
mod trace;
pub(crate) use schedule::SolveSchedule;
mod split;
pub(crate) use split::separated_groups;
mod work;
pub(crate) use image::PartitionImage;
pub use work::ComponentWork;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentInfo {
    /// Stable IDs in a single native solver, in ascending order.
    pub ids: Vec<u64>,
    /// Sticky only for this solver: at least one native contact cache has existed.
    pub contact_history: bool,
}

impl ComponentInfo {
    fn from_snapshot(s: &Snapshot) -> Self {
        Self {
            ids: s.bodies.iter().map(|b| b.id).collect(),
            contact_history: s.contact_history,
        }
    }
}

/// Is this exact set of dynamic bodies connected by current native active contact
/// edges, with no active contact to an unselected body? IDs must be canonical.
pub fn active_contact_group(snapshot: &Snapshot, ids: &[u64]) -> bool {
    active_contact_group_with_profile(snapshot, ids, NativeSolverProfile::Standard32)
}
fn active_contact_group_with_profile(
    snapshot: &Snapshot,
    ids: &[u64],
    profile: NativeSolverProfile,
) -> bool {
    if !(2..=profile.max_bodies()).contains(&ids.len())
        || ids.windows(2).any(|p| p[0] >= p[1])
        || ids.iter().any(|id| {
            !snapshot
                .bodies
                .iter()
                .any(|b| b.id == *id && b.tier == Tier::Dynamic)
        })
    {
        return false;
    }
    let mut neighbors = vec![Vec::new(); ids.len()];
    for edge in &snapshot.impulses {
        match (ids.binary_search(&edge.a), ids.binary_search(&edge.b)) {
            (Ok(a), Ok(b))
                if a < b && edge.normal_impulse.is_finite() && edge.normal_impulse >= 0. =>
            {
                neighbors[a].push(b);
                neighbors[b].push(a);
            }
            (Err(_), Err(_)) => {}
            _ => return false,
        }
    }
    let mut seen = vec![false; ids.len()];
    let mut stack = vec![0];
    seen[0] = true;
    while let Some(i) = stack.pop() {
        for &neighbor in &neighbors[i] {
            if !seen[neighbor] {
                seen[neighbor] = true;
                stack.push(neighbor);
            }
        }
    }
    seen.into_iter().all(|v| v)
}

/// Geometry of a finite contact manifold at the last native substep of a committed world.
/// The public body IDs are ordered. The normal points in the direction of the reaction on a.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub struct FiniteContactPatch {
    pub a: u64,
    pub b: u64,
    pub normal_on_a: [f64; 3],
    pub points: Vec<gz_physics_multifidelity::RigidPatchPoint>,
}

impl FiniteContactPatch {
    fn from_native(patch: gz_physics_multifidelity::RigidContactPatch, a: u64, b: u64) -> Self {
        let mut points = patch.points;
        let mut normal = patch.normal_on_a;
        if a > b {
            normal = normal.map(|component| -component);
            for point in &mut points {
                std::mem::swap(&mut point.lever_a, &mut point.lever_b);
                if let Some(impulse) = &mut point.tangent_impulse_on_a {
                    *impulse = impulse.map(|component| -component);
                }
            }
        }
        Self {
            a: a.min(b),
            b: a.max(b),
            normal_on_a: normal,
            points,
        }
    }
}

pub struct Partition {
    pub(crate) profile: NativeSolverProfile,
    solvers: Vec<Island>,
    committed: Snapshot,
    components: Vec<ComponentInfo>,
    pending: Vec<PendingImpulse>,
    fault: Option<String>,
    probe: Option<work::Probe>,
    // The worker can borrow one spare execution lane for this request. Neither
    // execution policy nor the lazily created thread changes a checkpoint.
    pub(crate) helper_allowed: bool,
    helper: Option<bevy_tasks::TaskPool>,
}

/// Native staging is complete before changing any resident ECS world. A warm
/// append keeps the resident Bevy clock, entity identities, systems and bindings.
struct WarmAppend {
    resident: usize,
    backend: MultiFidelityBackend,
    new_bodies: Vec<(Body, BodyId)>,
}

#[derive(Default)]
struct MergePlan {
    replacements: Vec<Island>,
    appends: Vec<WarmAppend>,
    retired: Vec<usize>,
}

impl Island {
    fn native_backend(&self) -> &MultiFidelityBackend {
        self.sim.world().resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .expect("installed island backend")
    }

    fn public_body_ids(&self) -> HashMap<BodyId, u64> {
        let world = self.sim.world();
        self.index.iter().map(|(&id, &entity)| {
            let native = world.get::<Binding>(entity).expect("installed native binding").native;
            (native, id)
        }).collect()
    }
}

impl Partition {
    pub fn restore(seed: &Snapshot) -> Result<Self, WorldError> {
        Self::restore_with_layout(seed, NativeSeedLayout::Standard32)
    }
    pub(crate) fn restore_with_layout(
        seed: &Snapshot,
        layout: NativeSeedLayout,
    ) -> Result<Self, WorldError> {
        let profile = layout.profile();
        if layout.is_canonical() && seed.bodies.len() > profile.max_bodies() {
            return Err(WorldError::Checkpoint);
        }
        // Admit all boxes together once, including separation, duplicate IDs, and
        // construction values. Spatial partitioning must not weaken native admission.
        let complete = Island::restore_with_profile(seed, profile)?;
        let committed = complete.snapshot();
        let singles: Vec<_> = committed
            .bodies
            .iter()
            .map(|b| ComponentInfo {
                ids: vec![b.id],
                contact_history: false,
            })
            .collect();
        let groups = if layout.is_canonical() && !singles.is_empty() {
            vec![(0..singles.len()).collect()]
        } else {
            swept::forced_groups_with_profile(&committed, &singles, &[], &[], profile)?
        };
        let solvers = if groups.len() == 1 {
            vec![complete]
        } else {
            groups
                .into_iter()
                .map(|g| {
                    Island::restore_with_profile(&Snapshot {
                        tick: seed.tick,
                        step: seed.step,
                        ground: seed.ground,
                        bodies: g.into_iter().map(|i| committed.bodies[i].clone()).collect(),
                        ..Snapshot::default()
                    }, profile)
                })
                .collect::<Result<Vec<_>, _>>()?
        };
        let components = solvers
            .iter()
            .map(|s| ComponentInfo::from_snapshot(&s.committed))
            .collect();
        Ok(Self {
            profile,
            components,
            solvers,
            committed,
            pending: Vec::new(),
            fault: None,
            probe: None,
            helper_allowed: false,
            helper: None,
        })
    }

    fn healthy(&self) -> Result<(), WorldError> {
        if let Some(reason) = &self.fault {
            Err(WorldError::Fault(reason.clone()))
        } else {
            Ok(())
        }
    }

    pub(crate) fn finite_contact_patches(&self) -> Vec<FiniteContactPatch> {
        let mut result = Vec::new();
        for island in &self.solvers {
            let patches = island.native_backend().rigid_contact_patches();
            if patches.is_empty() {
                continue;
            }
            let ids = island.public_body_ids();
            for patch in patches {
                let a = ids[&patch.a];
                let b = ids[&patch.b];
                result.push(FiniteContactPatch::from_native(patch, a, b));
            }
        }
        result.sort_by_key(|patch| (patch.a, patch.b));
        result
    }

    pub fn snapshot(&self) -> Snapshot {
        self.committed.clone()
    }
    pub fn topology(&self) -> Vec<ComponentInfo> {
        self.components.clone()
    }

    fn publish(&mut self, tick: u64) {
        self.solvers.sort_by_key(|s| s.committed.bodies[0].id);
        let mut frame = Snapshot {
            tick,
            step: self.committed.step,
            ground: self.committed.ground,
            ..Snapshot::default()
        };
        self.components.clear();
        for s in &self.solvers {
            let c = &s.committed;
            self.components.push(ComponentInfo::from_snapshot(c));
            frame.bodies.extend_from_slice(&c.bodies);
            frame.supports.extend_from_slice(&c.supports);
            frame.impulses.extend_from_slice(&c.impulses);
            frame.contacts += c.contacts;
            frame.contact_history |= c.contact_history;
        }
        frame.bodies.sort_unstable_by_key(|b| b.id);
        frame.supports.sort_unstable_by_key(|p| p.body);
        frame.impulses.sort_unstable_by_key(|i| (i.a, i.b));
        self.committed = frame;
    }

    fn owner(&self, id: u64) -> Result<usize, WorldError> {
        self.solvers
            .iter()
            .position(|s| s.index.contains_key(&id))
            .ok_or(WorldError::Missing(id))
    }

    pub fn queue_impulse(&mut self, id: u64, impulse: [f64; 3]) -> Result<(), WorldError> {
        self.healthy()?;
        validate_impulse(&self.committed.bodies, &self.pending, id, impulse)?;
        self.pending.push(PendingImpulse { id, impulse });
        Ok(())
    }

    pub fn check_step(&self) -> Result<(), WorldError> {
        self.healthy()?;
        self.committed
            .tick
            .checked_add(1)
            .ok_or(WorldError::TimeOverflow)?;
        self.plan_merges()?;
        Ok(())
    }

    fn cold_union(&self, indices: &[usize]) -> Result<Island, WorldError> {
        let mut bodies = Vec::new();
        for &i in indices {
            let frame = &self.solvers[i].committed;
            if frame.contact_history {
                return Err(WorldError::ContactHistory);
            }
            bodies.extend_from_slice(&frame.bodies);
        }
        Island::restore_with_profile(&Snapshot {
            tick: self.committed.tick,
            step: self.committed.step,
            ground: self.committed.ground,
            bodies,
            ..Snapshot::default()
        }, self.profile)
    }

    fn plan_merges(&self) -> Result<MergePlan, WorldError> {
        self.plan_forced_merges(&[], SolveSchedule::Uniform(1))
    }
    fn plan_forced_merges(
        &self,
        forced: &[ForceMotion],
        schedule: SolveSchedule<'_>,
    ) -> Result<MergePlan, WorldError> {
        let mut plan = MergePlan::default();
        for group in swept::forced_groups_with_profile(
            &self.committed,
            &self.components,
            &self.pending,
            forced,
            self.profile,
        )? {
            schedule.check_group(&self.components, &group)?;
            if group.len() < 2 {
                continue;
            }
            if group
                .iter()
                .filter(|&&i| self.components[i].contact_history)
                .count()
                >= 2
            {
                plan.replacements.push(self.joined_solver(&group)?);
                plan.retired.extend(group);
            } else if let Some(&resident) =
                group.iter().find(|&&i| self.components[i].contact_history)
            {
                // Keep the resident cache and stage every incoming body in
                // stable global-ID order, even when cold components interleave.
                let target = &self.solvers[resident];
                let mut boxes = Vec::new();
                for index in group.into_iter().filter(|&i| i != resident) {
                    boxes.extend(&self.solvers[index].committed.bodies);
                    plan.retired.push(index);
                }
                boxes.sort_unstable_by_key(|b: &&Body| b.id);
                gz_sim_core::next_entity_id(target.sim.world())?
                    .checked_add(boxes.len() as u64 - 1)
                    .ok_or(IdentityError::Exhausted)?;
                let live = target
                    .sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .unwrap();
                let archive = live.checkpoint_solver_with_profile(self.profile)?;
                let mut backend =
                    MultiFidelityBackend::restore_solver_with_profile(&archive, self.profile)?;
                let mut new_bodies = Vec::with_capacity(boxes.len());
                for b in boxes {
                    let native = spawn_box(&mut backend, b, self.committed.step)?;
                    new_bodies.push((b.clone(), native));
                }
                plan.appends.push(WarmAppend {
                    resident,
                    backend,
                    new_bodies,
                });
            } else {
                plan.replacements.push(self.cold_union(&group)?);
                plan.retired.extend(group);
            }
        }
        Ok(plan)
    }

    fn commit_merges(&mut self, mut plan: MergePlan) {
        for append in plan.appends {
            let resident = &mut self.solvers[append.resident];
            for (b, native) in append.new_bodies {
                let entity = spawn_binding(&mut resident.sim, b.id, native)
                    .expect("exclusive ECS identity allocation was preflighted");
                resident.index.insert(b.id, entity);
                resident.committed.bodies.push(b);
            }
            resident.committed.bodies.sort_unstable_by_key(|b| b.id);
            resident
                .sim
                .world_mut()
                .insert_resource(PhysicsBackend::from(append.backend));
        }
        plan.retired.sort_unstable();
        for i in plan.retired.into_iter().rev() {
            self.solvers.remove(i);
        }
        self.solvers.extend(plan.replacements);
    }

    /// All admission failures occur before mutating any solver. A native failure
    /// after integration starts leaves the last complete public barrier visible
    /// and permanently faults this partition until recovery from the journal.
    pub fn step(&mut self) -> Result<(), WorldError> {
        self.step_with_wrenches(&[])
    }
    pub(crate) fn step_with_wrenches(&mut self, wrenches: &[BodyWrench]) -> Result<(), WorldError> {
        self.step_in_field([0.; 3], wrenches)
    }

    pub(crate) fn step_in_field(
        &mut self,
        gravity: [f64; 3],
        wrenches: &[BodyWrench],
    ) -> Result<(), WorldError> {
        self.step_refined_in_field(gravity, wrenches, 1)
    }

    /// Advance the whole worker by one public barrier with a journaled solve count.
    pub fn step_refined_in_field(
        &mut self,
        gravity: [f64; 3],
        wrenches: &[BodyWrench],
        substeps: u8,
    ) -> Result<(), WorldError> {
        self.step_scheduled(gravity, wrenches, SolveSchedule::Uniform(substeps))
    }

    pub(crate) fn step_scheduled(
        &mut self,
        gravity: [f64; 3],
        wrenches: &[BodyWrench],
        schedule: SolveSchedule<'_>,
    ) -> Result<(), WorldError> {
        schedule.validate(&self.committed)?;
        self.healthy()?;
        let tick = self
            .committed
            .tick
            .checked_add(1)
            .ok_or(WorldError::TimeOverflow)?;
        let forced = field_motion(
            self.committed.step,
            &self.committed.bodies,
            gravity,
            wrenches,
        )?;
        let plan = self.plan_forced_merges(&forced, schedule)?;
        self.commit_merges(plan);
        for wrench in wrenches {
            let index = self.owner(wrench.id)?;
            if let Err(e) = self.solvers[index].add_step_wrench(wrench) {
                let reason = e.to_string();
                self.fault = Some(reason.clone());
                return Err(WorldError::Fault(reason));
            }
        }
        // Apply pending inputs only once all physical admission has succeeded.
        // Each solver's list retains submission order for commands on its bodies.
        let pending = &self.pending;
        let advance = |_index: usize, island: &mut Island| {
            island.pending.extend(
                pending
                    .iter()
                    .filter(|p| island.index.contains_key(&p.id))
                    .cloned(),
            );
            if schedule.fine(island.committed.bodies[0].id) {
                island.stage_refinement(wrenches);
            }
            island.advance_native_field(gravity)
        };
        let probe = self.probe.as_ref().filter(|p| p.measuring(tick));
        let split = if self.helper_allowed && probe.is_none_or(|p| p.scheduled) {
            lanes::plan(&self.solvers, schedule)
        } else {
            None
        };
        #[cfg(feature = "lane-trace")]
        let trace = trace::Capture::from_env(self.solvers.len());
        #[cfg(feature = "lane-trace")]
        let advance = |index: usize, island: &mut Island| match &trace {
            Some(trace) => trace.measure(index, island, schedule, |world| advance(index, world)),
            None => advance(index, island),
        };
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        let barrier = crate::barrier_trace::current();
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        let advance = |index: usize, island: &mut Island| match &barrier {
            Some(recorder) => {
                recorder.native(island.committed.bodies[0].id, || advance(index, island))
            }
            None => advance(index, island),
        };
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        crate::barrier_trace::mark("batch_start", None);
        let advance = |index: usize, island: &mut Island| {
            if let Some(probe) = probe {
                let start = std::time::Instant::now();
                advance(index, island)?;
                probe.record(index, island, start);
                Ok(())
            } else {
                advance(index, island)
            }
        };
        let result = if let Some(split) = split {
            let pool = self.helper.get_or_insert_with(|| {
                bevy_tasks::TaskPoolBuilder::new()
                    .num_threads(1)
                    .thread_name("local physics".to_owned())
                    .build()
            });
            split.run(pool, &mut self.solvers, schedule, advance)
        } else {
            self.solvers.iter_mut().enumerate().try_for_each(|(i, world)| advance(i, world))
        };
        #[cfg(all(target_os = "linux", feature = "barrier-trace"))]
        crate::barrier_trace::mark("batch_end", None);
        if let Err(error) = result {
            let reason = error.to_string();
            self.fault = Some(reason.clone());
            return Err(WorldError::Fault(reason));
        }
        if matches!(schedule, SolveSchedule::Islands(_))
            && let Err(error) = self.split_separated_solvers()
        {
            let reason = error.to_string();
            self.fault = Some(reason.clone());
            return Err(WorldError::Fault(reason));
        }
        self.pending.clear();
        self.publish(tick);
        #[cfg(feature = "lane-trace")]
        if let Some(trace) = trace {
            trace.publish(tick);
        }
        Ok(())
    }

    #[cfg(test)]
    pub(crate) fn inject_commit_fault(&mut self) {
        self.solvers.first_mut().unwrap().inject_commit_fault();
    }

    pub fn export(&self, id: u64) -> Result<Body, WorldError> {
        self.healthy()?;
        if self.pending.iter().any(|p| p.id == id) {
            return Err(WorldError::Pending);
        }
        self.solvers[self.owner(id)?].export(id)
    }

    pub fn remove(&mut self, id: u64) -> Result<(), WorldError> {
        self.export(id)?;
        let i = self.owner(id)?;
        self.solvers[i].remove(id)?;
        if self.solvers[i].committed.bodies.is_empty() {
            self.solvers.remove(i);
        }
        self.publish(self.committed.tick);
        Ok(())
    }

    fn cold_incoming(&self, body: &Body) -> Result<Island, WorldError> {
        self.healthy()?;
        if self.owner(body.id).is_ok() {
            return Err(WorldError::Duplicate(body.id));
        }
        let incoming = Island::restore_with_profile(&Snapshot {
            tick: self.committed.tick,
            step: self.committed.step,
            ground: self.committed.ground,
            bodies: vec![body.clone()],
            ..Snapshot::default()
        }, self.profile)?;
        // Check the incoming box against each live body at its committed pose. Resident
        // boxes may already touch each other, so admitting the entire set cold would fail.
        let inserted = body.bounds()?;
        for existing in &self.committed.bodies {
            if inserted.predicts_overlap(&existing.bounds()?) {
                return Err(PhysicsError::TransitionDeferred {
                    reason: "box intersects an existing body's contact prediction bounds",
                }
                .into());
            }
        }
        self.check_projected(&incoming)?;
        Ok(incoming)
    }

    pub fn validate_import(&self, body: &Body) -> Result<(), WorldError> {
        self.cold_incoming(body)?;
        Ok(())
    }
    pub fn import(&mut self, body: Body) -> Result<(), WorldError> {
        let incoming = self.cold_incoming(&body)?;
        self.insert_validated(incoming);
        Ok(())
    }

    fn selection(&self, ids: &[u64]) -> Result<Vec<usize>, WorldError> {
        self.healthy()?;
        if !(2..=self.profile.max_bodies()).contains(&ids.len())
            || ids.windows(2).any(|p| p[0] >= p[1])
        {
            return Err(WorldError::Checkpoint);
        }
        let first = self.owner(ids[0])?;
        let legacy = self.committed.bodies.len() == 2 && ids.len() == 2;
        if legacy {
            let second = self.owner(ids[1])?;
            if self
                .committed
                .bodies
                .iter()
                .any(|b| b.tier != Tier::Dynamic)
            {
                return Err(WorldError::Checkpoint);
            }
            if first != second && !self.committed.contact_history {
                return Ok(vec![first, second]);
            }
        } else if !active_contact_group_with_profile(&self.committed, ids, self.profile) {
            return Err(WorldError::Checkpoint);
        }
        if self.components[first].ids == ids {
            Ok(vec![first])
        } else {
            Err(WorldError::Checkpoint)
        }
    }

    fn complete_solver(&self, ids: &[u64]) -> Result<usize, WorldError> {
        self.healthy()?;
        let i = self.owner(*ids.first().ok_or(WorldError::Checkpoint)?)?;
        if self.components[i].ids != ids {
            return Err(WorldError::Checkpoint);
        }
        Ok(i)
    }

    pub(crate) fn checkpoint_component(&self, ids: &[u64]) -> Result<WarmCheckpoint, WorldError> {
        let index = self.complete_solver(ids)?;
        let inputs = self
            .pending
            .iter()
            .filter(|p| ids.binary_search(&p.id).is_ok())
            .cloned()
            .collect::<Vec<_>>();
        self.solvers[index].solver_image(&inputs, self.profile)
    }

    pub(crate) fn release_component(&mut self, ids: &[u64]) -> Result<(), WorldError> {
        let index = self.complete_solver(ids)?;
        self.solvers.remove(index);
        self.pending.retain(|p| ids.binary_search(&p.id).is_err());
        self.publish(self.committed.tick);
        Ok(())
    }

    pub fn checkpoint_pair(&self, ids: [u64; 2]) -> Result<WarmCheckpoint, WorldError> {
        self.checkpoint_group(&ids)
    }

    /// Capture exactly one complete native solver and its ordered commands.
    pub fn checkpoint_group(&self, ids: &[u64]) -> Result<WarmCheckpoint, WorldError> {
        let selected = self.selection(ids)?;
        let commands: Vec<_> = self
            .pending
            .iter()
            .filter(|p| ids.contains(&p.id))
            .cloned()
            .collect();
        if selected.len() == 1 {
            self.solvers[selected[0]].checkpoint_with_commands(&commands, self.profile)
        } else {
            self.cold_union(&selected)?
                .checkpoint_with_commands(&commands, self.profile)
        }
    }

    pub fn release_pair(&mut self, ids: [u64; 2]) -> Result<(), WorldError> {
        self.release_group(&ids)
    }

    pub fn release_group(&mut self, ids: &[u64]) -> Result<(), WorldError> {
        let mut selected = self.selection(ids)?;
        selected.sort_unstable();
        for i in selected.into_iter().rev() {
            self.solvers.remove(i);
        }
        self.pending.retain(|p| !ids.contains(&p.id));
        self.publish(self.committed.tick);
        Ok(())
    }

    fn check_projected(&self, incoming: &Island) -> Result<(), WorldError> {
        let mut projected = self.snapshot();
        projected
            .bodies
            .extend_from_slice(&incoming.committed.bodies);
        let mut topology = self.topology();
        topology.push(ComponentInfo::from_snapshot(&incoming.committed));
        let mut queued = self.pending.clone();
        queued.extend_from_slice(&incoming.pending);
        swept::forced_groups_with_profile(&projected, &topology, &queued, &[], self.profile)?;
        Ok(())
    }

    pub fn check_insert(&self, incoming: &Island) -> Result<(), WorldError> {
        self.healthy()?;
        incoming.healthy()?;
        let frame = &incoming.committed;
        let ids: Vec<_> = frame.bodies.iter().map(|b| b.id).collect();
        if frame.ground != self.committed.ground
            || frame.step != self.committed.step
            || frame.tick != self.committed.tick
            || !(2..=self.profile.max_bodies()).contains(&ids.len())
            || frame.bodies.iter().any(|b| b.tier != Tier::Dynamic)
            || (!(self.solvers.is_empty() && ids.len() == 2)
                && !active_contact_group_with_profile(frame, &ids, self.profile))
        {
            return Err(WorldError::Checkpoint);
        }
        self.check_insert_component(incoming)
    }

    pub(crate) fn check_insert_component(&self, incoming: &Island) -> Result<(), WorldError> {
        self.healthy()?;
        incoming.healthy()?;
        let frame = &incoming.committed;
        if incoming.profile != self.profile
            || frame.ground != self.committed.ground
            || frame.step != self.committed.step
            || frame.tick != self.committed.tick
            || !(1..=self.profile.max_bodies()).contains(&frame.bodies.len())
        {
            return Err(WorldError::Checkpoint);
        }
        if self.pending.len() + incoming.pending.len() > MAX_PENDING_IMPULSES {
            return Err(WorldError::Pending);
        }
        for body in &frame.bodies {
            if self.owner(body.id).is_ok() {
                return Err(WorldError::Duplicate(body.id));
            }
        }
        self.check_projected(incoming)
    }

    /// Interleave two independent FIFO streams in their original coordinator
    /// acceptance order. A set bit selects the next incoming impulse. Both
    /// streams must be consumed exactly once before a receipt can be issued.
    pub(crate) fn ordered_arrival(
        &self,
        incoming: &Island,
        mask: u32,
    ) -> Result<Vec<PendingImpulse>, WorldError> {
        let count = self.pending.len() + incoming.pending.len();
        if count > MAX_PENDING_IMPULSES
            || mask.count_ones() as usize != incoming.pending.len()
            || (count < u32::BITS as usize && (mask >> count) != 0)
        {
            return Err(WorldError::Pending);
        }
        let mut resident = self.pending.iter();
        let mut arrivals = incoming.pending.iter();
        (0..count)
            .map(|i| {
                let stream = if (mask & (1 << i)) != 0 {
                    &mut arrivals
                } else {
                    &mut resident
                };
                stream.next().cloned().ok_or(WorldError::Pending)
            })
            .collect()
    }

    /// Commit a prepared incoming solver after checking its epoch, ID set,
    /// pending capacity and separation while both workers are frozen.
    pub(crate) fn insert_validated(&mut self, incoming: Island) {
        self.insert_ordered(incoming, None);
    }

    pub(crate) fn insert_ordered(
        &mut self,
        mut incoming: Island,
        order: Option<Vec<PendingImpulse>>,
    ) {
        if let Some(merged) = order {
            incoming.pending.clear();
            self.pending = merged;
        } else {
            self.pending.append(&mut incoming.pending);
        }
        self.solvers.push(incoming);
        self.publish(self.committed.tick);
    }
}

#[cfg(test)]
#[path = "../../tests/private/physical_partition.rs"]
mod tests;

#[cfg(test)]
mod friction_identity;
