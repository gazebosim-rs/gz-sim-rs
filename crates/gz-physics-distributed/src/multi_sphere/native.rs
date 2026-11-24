// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodyId, BodyState, DynamicBackend, JointId, PhysicsBackend, PhysicsError};
use gz_physics_multifidelity::MultiFidelityBackend as Backend;
use gz_sim_core::Simulation;
use std::time::Duration;

fn public(world: &Backend, id: BodyId, name: u64) -> Result<ArticulationLinkState> {
    let s = world.state(id).ok_or(PhysicsError::MissingBody)?;
    Ok(ArticulationLinkState {
        id: name,
        position: s.position,
        orientation: s.orientation,
        velocity: s.velocity,
        angular_velocity: s.angular_velocity,
    })
}
fn native(s: ArticulationLinkState) -> BodyState {
    BodyState {
        position: s.position,
        orientation: s.orientation,
        velocity: s.velocity,
        angular_velocity: s.angular_velocity,
    }
}
fn simulation(b: Backend) -> Result<Simulation> {
    let mut sim = Simulation::new(Duration::from_millis(1)).map_err(NativeError::from)?;
    gz_physics::install(&mut sim, b).map_err(NativeError::from)?;
    Ok(sim)
}
struct Hinge {
    sim: Simulation,
    links: [BodyId; 2],
    joint: JointId,
    spheres: [BodyId; 2],
    // Populated only from the last successfully captured state of this resident.
    saved: Option<Vec<u8>>,
    verified_step: bool,
}
impl Hinge {
    fn load(spec: &MultiSphereSpec, image: Option<&[u8]>) -> Result<Self> {
        let mut c = spec.hinge.build()?;
        for id in c.bodies {
            c.backend.configure_moving_body(id)?;
        }
        let spheres = [
            spec.single(0).spawn_sphere(&mut c.backend)?,
            spec.single(1).spawn_sphere(&mut c.backend)?,
        ];
        c.backend.configure_pair_revolute()?;
        if let Some(image) = image {
            c.backend.restore_pair_revolute(image)?;
        } else {
            for id in spheres {
                // An initially touching or predicted companion belongs to the hinge's first cut.
                match c.backend.set_pair_sphere_active(id, false, None) {
                    Ok(()) | Err(PhysicsError::Unsupported { .. }) => (),
                    Err(error) => return Err(error.into()),
                }
            }
        }
        Ok(Self {
            sim: simulation(c.backend)?,
            links: c.bodies,
            joint: c.joint,
            spheres,
            saved: None,
            verified_step: false,
        })
    }
    fn backend(&self) -> &Backend {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref()
            .expect("multi-sphere engine")
    }
    fn change(&mut self, id: usize, on: bool, s: ArticulationLinkState) -> Result<bool> {
        let mut backend = self.sim.world_mut().resource_mut::<PhysicsBackend>();
        let b = backend.downcast_mut::<Backend>().unwrap();
        if b.pair_sphere_active(self.spheres[id])? == on {
            return Ok(true);
        }
        match b.set_pair_sphere_active(self.spheres[id], on, on.then(|| native(s))) {
            Ok(()) => {
                self.saved = None;
                Ok(true)
            }
            Err(PhysicsError::Unsupported { .. }) if !on => Ok(false),
            Err(e) => Err(e.into()),
        }
    }
    fn edit(
        &mut self,
        operation: impl FnOnce(&mut Backend) -> std::result::Result<(), PhysicsError>,
    ) -> std::result::Result<(), PhysicsError> {
        let mut resource = self.sim.world_mut().resource_mut::<PhysicsBackend>();
        operation(resource.downcast_mut::<Backend>().unwrap())?;
        self.saved = None;
        Ok(())
    }
    fn merge_pair(&mut self, spec: &MultiSphereSpec, pair: &PairPart) -> Result<Reply> {
        let mut joined = Self::load(spec, Some(self.image()?))?;
        joined.edit(|b| b.merge_sphere_island(&pair.image))?;
        let joined = joined.state(spec)?;
        if joined.sample.spheres != pair.spheres
            || worker::encode(&joined.sample.sphere_reaction)? != worker::encode(&pair.reaction)?
        {
            return Err(invalid("sphere pair receipt differs after merge"));
        }
        Ok(Reply::Hinge(Box::new(joined)))
    }
    fn split_pair(&mut self, spec: &MultiSphereSpec) -> Result<Reply> {
        let image = self.image()?;
        let mut hinge = Self::load(spec, Some(image))?;
        match hinge.edit(|b| b.extract_sphere_island(false)) {
            Err(PhysicsError::Unsupported { .. }) => return Ok(Reply::Empty),
            other => other?,
        }
        let mut donor = Self::load(spec, Some(image))?;
        donor.edit(|b| b.extract_sphere_island(true))?;
        let pair = PairIsland::from_native(donor);
        Ok(Reply::SplitPair(Box::new((
            hinge.state(spec)?,
            pair.state(spec)?,
        ))))
    }
    fn mask(&self) -> Result<u8> {
        let b = self.backend();
        Ok(u8::from(b.pair_sphere_active(self.spheres[0])?)
            | (u8::from(b.pair_sphere_active(self.spheres[1])?) << 1))
    }
    fn sample(&self, spec: &MultiSphereSpec) -> Result<GraphSample> {
        let b = self.backend();
        let [a, c] = self.spheres;
        let mask = u8::from(b.pair_sphere_active(a)?) | (u8::from(b.pair_sphere_active(c)?) << 1);
        let j = b
            .joint_state(self.joint)
            .ok_or(PhysicsError::MissingJoint)?;
        let mut reactions = [[MovingReaction::default(); 2]; 2];
        for (i, row) in reactions.iter_mut().enumerate() {
            if includes(mask, i) {
                for (slot, id) in row.iter_mut().zip(self.links) {
                    *slot = b.pair_reaction(id, self.spheres[i])?;
                }
            }
        }
        Ok(GraphSample {
            mask,
            links: [
                public(b, self.links[0], spec.hinge.links[0].id)?,
                public(b, self.links[1], spec.hinge.links[1].id)?,
            ],
            q: j.position,
            qdot: j.velocity,
            spheres: [
                public(b, a, spec.spheres[0].id)?,
                public(b, c, spec.spheres[1].id)?,
            ],
            reactions,
            sphere_reaction: if mask == 3 {
                b.pair_reaction(a, c)?
            } else {
                Default::default()
            },
        })
    }
    fn image(&mut self) -> Result<&[u8]> {
        if self.saved.is_none() {
            self.saved = Some(timed!(
                checkpoint_ns,
                self.backend().checkpoint_pair_revolute()
            )?);
        }
        Ok(self.saved.as_deref().unwrap())
    }
    fn state(&mut self, spec: &MultiSphereSpec) -> Result<HingePart> {
        Ok(HingePart {
            sample: self.sample(spec)?,
            image: self.image()?.to_vec(),
        })
    }
    fn step(&mut self) -> Result<()> {
        self.saved = None;
        timed!(native_step_ns, self.sim.step_exact(1)).map_err(NativeError::from)?;
        Ok(())
    }
    fn with_members(
        spec: &MultiSphereSpec,
        image: &[u8],
        mask: u8,
        spheres: [ArticulationLinkState; 2],
    ) -> Result<Self> {
        let mut h = Self::load(spec, Some(image))?;
        for (i, s) in spheres.into_iter().enumerate() {
            h.change(i, includes(mask, i), s)?;
        }
        Ok(h)
    }
    // The native narrow phase and manifold caches live in the full counterfactual.
    // Both actual candidate trajectories must agree, and each excluded companion
    // must pass its drain check after that exact discrete step.
    fn agrees_with_split(
        &mut self,
        reduced: &Self,
        spec: &MultiSphereSpec,
        tick: u64,
        spheres: [ArticulationLinkState; 2],
    ) -> Result<bool> {
        let full = self.sample(spec)?;
        let split = reduced.sample(spec)?;
        if full.links != split.links
            || full.q.to_bits() != split.q.to_bits()
            || full.qdot.to_bits() != split.qdot.to_bits()
            || worker::encode(&(full.reactions, full.sphere_reaction))?
                != worker::encode(&(split.reactions, split.sphere_reaction))?
        {
            return Ok(false);
        }
        for (i, s) in spheres.into_iter().enumerate() {
            if includes(split.mask, i) {
                if full.spheres[i] != split.spheres[i] {
                    return Ok(false);
                }
            } else {
                let mut loose = Sphere::load(spec, i, tick, s)?;
                loose.step()?;
                if loose.state(spec)? != full.spheres[i]
                    || !self.change(i, false, full.spheres[i])?
                {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }
    fn plan(
        &mut self,
        spec: &MultiSphereSpec,
        tick: u64,
        mask: u8,
        spheres: [ArticulationLinkState; 2],
    ) -> Result<Reply> {
        let before = self.mask()?;
        let image = self.image()?.to_vec();
        if before == mask {
            // Stage the actual resident's next step. Fence/rollback always restores the
            // committed cut. The subsequent Step receipt consumes the staged result once.
            let mut combined = Self::with_members(spec, &image, 3, spheres)?;
            combined.step()?;
            self.step()?;
            if combined.agrees_with_split(self, spec, tick, spheres)? {
                self.verified_step = true;
                return Ok(Reply::Empty);
            }
            *self = Self::load(spec, Some(&image))?;
            return Ok(Reply::Hinge(Box::new(
                Self::with_members(spec, &image, 3, spheres)?.state(spec)?,
            )));
        }
        let mut proposed = Self::with_members(spec, &image, mask, spheres)?;
        let effective = proposed.mask()?;
        if effective == 3 {
            return Ok(if before == 3 {
                Reply::Empty
            } else {
                Reply::Hinge(Box::new(proposed.state(spec)?))
            });
        }
        let candidate = (effective != before)
            .then(|| proposed.state(spec))
            .transpose()?;
        let mut combined = Self::with_members(spec, &image, 3, spheres)?;
        combined.step()?;
        proposed.step()?;
        if combined.agrees_with_split(&proposed, spec, tick, spheres)? {
            if let Some(part) = candidate {
                return Ok(Reply::Hinge(Box::new(part)));
            }
            proposed.verified_step = true;
            *self = proposed;
            return Ok(Reply::Empty);
        }
        if before == 3 {
            return Ok(Reply::Empty);
        }
        Ok(Reply::Hinge(Box::new(
            Self::with_members(spec, &image, 3, spheres)?.state(spec)?,
        )))
    }
}
struct Sphere {
    sim: Simulation,
    index: usize,
    id: BodyId,
}
impl Sphere {
    fn load(
        spec: &MultiSphereSpec,
        index: usize,
        tick: u64,
        state: ArticulationLinkState,
    ) -> Result<Self> {
        if index >= 2 || state.id != spec.spheres[index].id {
            return Err(invalid("unexpected independent sphere identity"));
        }
        let mut b = Backend::new([0.; 3])?;
        let s = spec.single(index);
        let id = s.spawn_sphere(&mut b)?;
        if tick > 0 {
            b.place_detached_sphere(id, native(state))?;
        } else if s.initial_sphere() != state {
            return Err(invalid("wrong cold sphere pose"));
        }
        Ok(Self {
            sim: simulation(b)?,
            id,
            index,
        })
    }
    fn state(&self, spec: &MultiSphereSpec) -> Result<ArticulationLinkState> {
        let b = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref()
            .unwrap();
        public(b, self.id, spec.spheres[self.index].id)
    }
    fn step(&mut self) -> Result<()> {
        timed!(native_step_ns, self.sim.step_exact(1)).map_err(NativeError::from)?;
        Ok(())
    }
}
struct PairIsland {
    sim: Simulation,
    ids: [BodyId; 2],
    staged: bool,
}
impl PairIsland {
    fn from_native(world: Hinge) -> Self {
        Self {
            sim: world.sim,
            ids: world.spheres,
            staged: false,
        }
    }
    fn load(
        spec: &MultiSphereSpec,
        tick: u64,
        image: Option<&[u8]>,
        spheres: [ArticulationLinkState; 2],
    ) -> Result<Self> {
        let mut world = Hinge::load(spec, None)?;
        if let Some(image) = image {
            world.edit(|b| b.restore_sphere_island(image))?;
        } else {
            for (i, s) in spheres.into_iter().enumerate() {
                if s.id != spec.spheres[i].id || (tick == 0 && s != spec.single(i).initial_sphere())
                {
                    return Err(invalid("invalid initial pair state"));
                }
                if !world.backend().pair_sphere_active(world.spheres[i])? {
                    world.change(i, true, s)?;
                } else if tick != 0 {
                    let id = world.spheres[i];
                    world.edit(|b| b.place_detached_sphere(id, native(s)))?;
                }
            }
            world.edit(|b| b.extract_sphere_island(true))?;
        }
        let pair = Self::from_native(world);
        if pair.spheres(spec)? != spheres {
            return Err(invalid("restored pair bodies differ from their receipt"));
        }
        Ok(pair)
    }
    fn backend(&self) -> &Backend {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref()
            .unwrap()
    }
    fn spheres(&self, spec: &MultiSphereSpec) -> Result<[ArticulationLinkState; 2]> {
        Ok([
            public(self.backend(), self.ids[0], spec.spheres[0].id)?,
            public(self.backend(), self.ids[1], spec.spheres[1].id)?,
        ])
    }
    fn state(&self, spec: &MultiSphereSpec) -> Result<PairPart> {
        let b = self.backend();
        Ok(PairPart {
            image: timed!(checkpoint_ns, b.checkpoint_sphere_island())?,
            spheres: self.spheres(spec)?,
            reaction: b.pair_reaction(self.ids[0], self.ids[1])?,
            drained: b.sphere_island_drained()?,
        })
    }
    fn step(&mut self) -> Result<()> {
        timed!(native_step_ns, self.sim.step_exact(1)).map_err(NativeError::from)?;
        Ok(())
    }
}
enum World {
    Hinge(Hinge),
    Sphere(Sphere),
    Pair(PairIsland),
}
impl World {
    fn load(spec: &MultiSphereSpec, t: u64, seed: Seed) -> Result<Self> {
        Ok(match seed {
            Seed::Hinge(image) => Self::Hinge(Hinge::load(spec, image.as_deref())?),
            Seed::Pair { image, spheres } => {
                Self::Pair(PairIsland::load(spec, t, image.as_deref(), spheres)?)
            }
            Seed::Sphere(i, s) => Self::Sphere(Sphere::load(spec, i, t, s)?),
        })
    }
    fn state(&mut self, spec: &MultiSphereSpec) -> Result<Reply> {
        match self {
            Self::Hinge(h) => Ok(Reply::Hinge(Box::new(h.state(spec)?))),
            Self::Sphere(s) => Ok(Reply::Sphere(s.state(spec)?)),
            Self::Pair(p) => Ok(Reply::Pair(Box::new(p.state(spec)?))),
        }
    }
}
pub(super) fn verify_restart(spec: &MultiSphereSpec, cut: &Cut) -> Result<()> {
    cut.install_worlds(|_, seed| World::load(spec, cut.tick, seed)?.state(spec))?;
    Ok(())
}

/// Entry point for a stock physics_worker managed by a multi-sphere coordinator.
pub fn serve(slot: u8, path: &Path) -> io::Result<()> {
    serve_at(slot, path, None)
}

/// Serve one native multi-sphere slot behind an SSH loopback port forward.
pub fn serve_forwarded(
    slot: u8,
    path: &Path,
    remote: std::num::NonZeroU16,
    local: std::num::NonZeroU16,
) -> io::Result<()> {
    validate_forwarded_slot(slot, path)?;
    serve_at(slot, path, Some(worker::ForwardedPorts { remote, local }))
}

/// Check the public SSH slot and lease path before attempting bootstrap.
pub fn validate_forwarded_slot(slot: u8, path: &Path) -> io::Result<()> {
    if slot >= 8 || !path.is_absolute() || path.file_name().is_none() {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "expected a multi-sphere slot in 0..8 and an absolute lease file path",
        ));
    }
    Ok(())
}

fn serve_at(slot: u8, path: &Path, forward: Option<worker::ForwardedPorts>) -> io::Result<()> {
    worker::serve_profile_at(slot, path, MAGIC, forward, |spec: MultiSphereSpec| {
        spec.validate()?;
        let mut world = None::<World>;
        let mut tick = 0u64;
        Ok(move |t: u64, cmd: Command, steps: &mut u64| {
            #[cfg(feature = "sphere-pair-trace")]
            timing::worker_command(&cmd);
            let response = (|| {
                match cmd {
                    Command::Fence => {
                        world = None;
                        return Ok(Reply::Empty);
                    }
                    Command::Install(seed) if world.is_none() => {
                        world = Some(World::load(&spec, t, seed)?);
                        tick = t;
                    }
                    Command::Install(_) => {
                        return Err(invalid("multi-sphere slot requires fencing"));
                    }
                    _ if t != tick => return Err(invalid("multi-sphere worker clock mismatch")),
                    cmd => match (cmd, world.as_mut()) {
                        (Command::Step, Some(w)) => {
                            match w {
                                World::Hinge(h) => {
                                    if !std::mem::take(&mut h.verified_step) {
                                        h.step()?
                                    }
                                }
                                World::Sphere(s) => s.step()?,
                                World::Pair(p) => {
                                    if !std::mem::take(&mut p.staged) {
                                        p.step()?;
                                    }
                                }
                            }
                            tick = tick.checked_add(1).ok_or(invalid("tick exhausted"))?;
                            *steps += 1;
                        }
                        (Command::Torque(e), Some(World::Hinge(h))) if !h.verified_step => {
                            h.saved = None;
                            h.sim
                                .world_mut()
                                .resource_mut::<PhysicsBackend>()
                                .add_joint_effort(h.joint, e)?;
                        }
                        (Command::PreviewPair, Some(World::Pair(p))) if !p.staged => {
                            p.step()?;
                            p.staged = true;
                            return Ok(Reply::PairEndpoint(p.spheres(&spec)?.map(|s| s.position)));
                        }
                        (Command::MergePair(part), Some(World::Hinge(h))) if !h.verified_step => {
                            return h.merge_pair(&spec, &part);
                        }
                        (Command::SplitPair, Some(World::Hinge(h))) if !h.verified_step => {
                            return h.split_pair(&spec);
                        }
                        (Command::Plan { mask, spheres }, Some(World::Hinge(h)))
                            if mask <= 3 && !h.verified_step =>
                        {
                            return h.plan(&spec, t, mask, spheres);
                        }
                        _ => return Err(invalid("operation incompatible with multi-sphere slot")),
                    },
                }
                world
                    .as_mut()
                    .ok_or(invalid("missing worker world"))?
                    .state(&spec)
            })();
            if response.is_err() {
                world = None;
            }
            response
        })
    })
}

#[cfg(test)]
#[path = "legacy_pair_tests.rs"]
mod legacy_pair_tests;
