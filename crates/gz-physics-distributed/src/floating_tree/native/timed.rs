// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::floating_tree::automatic::{
    self, AutomaticConstruction, Subreply, Subrequest, TimedCut,
};
use crate::paged_native::PagedWorld;
use bevy_ecs::{
    prelude::{ResMut, Resource},
    schedule::Schedule,
};

#[derive(Resource)]
struct NativeSubstep {
    dt: Duration,
    failed: Option<gz_physics::PhysicsError>,
}
fn integrate(mut input: ResMut<NativeSubstep>, mut be: ResMut<PhysicsBackend>) {
    input.failed = be.step(input.dt).err();
}
#[derive(Clone, Copy)]
enum Phase {
    Accepted,
    Head { nanos: u64, stamp: [u8; 32] },
    Applied { nanos: u64, stamp: [u8; 32] },
}
pub(in crate::floating_tree) struct TimedWorld<S: AutomaticConstruction> {
    tree: TreeWorld,
    step: Schedule,
    spec: S,
    tick: u64,
    last: u64,
    consumed: bool,
    phase: Phase,
}

impl<S: AutomaticConstruction> TimedWorld<S> {
    pub(in crate::floating_tree) fn new(s: &S) -> Result<Self> {
        s.validate_native()?;
        let mut tree = if let Some(t) = s.tree() {
            TreeWorld::new(t)?
        } else {
            TreeWorld::new_rotor(s.rotor().ok_or(invalid("missing world revolute"))?)?
        };
        tree.sim.world_mut().insert_resource(NativeSubstep {
            dt: Duration::ZERO,
            failed: None,
        });
        let mut step = Schedule::default();
        step.add_systems(integrate);
        Ok(Self {
            tree,
            step,
            spec: s.clone(),
            tick: 0,
            last: 0,
            consumed: false,
            phase: Phase::Accepted,
        })
    }
    fn substep(&mut self, nanos: u64) -> Result<()> {
        if nanos > 0 {
            self.tree.sim.world_mut().resource_mut::<NativeSubstep>().dt =
                Duration::from_nanos(nanos);
            self.step.run(self.tree.sim.world_mut());
            if let Some(e) = self
                .tree
                .sim
                .world_mut()
                .resource_mut::<NativeSubstep>()
                .failed
                .take()
            {
                return Err(NativeError::from(e).into());
            }
            self.last = nanos;
        }
        Ok(())
    }
    fn image(&self) -> Result<Option<Vec<u8>>> {
        if self.last == 0 {
            return Ok(None);
        }
        let be = self
            .tree
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .expect("owned native tree");
        let dt = Duration::from_nanos(self.last);
        Ok(Some(
            if self.spec.rotor().is_some() {
                be.checkpoint_timed_world_revolute(dt)
            } else {
                be.checkpoint_timed_revolute_tree(dt)
            }
            .map_err(NativeError::from)?,
        ))
    }
    pub(in crate::floating_tree) fn cut(&self) -> Result<TimedCut> {
        if !matches!(self.phase, Phase::Accepted) {
            return Err(invalid("speculative material head cannot be published"));
        }
        Ok(TimedCut {
            state: self.tree.observation_at(self.tick),
            last_ns: self.last,
            consumed: self.consumed,
            image: self.image()?,
        })
    }
    pub(in crate::floating_tree) fn swept_spheres(&self) -> Result<Subreply> {
        use gz_physics_multifidelity::ArticulatedSweep;
        let ids = self
            .spec
            .receivers()
            .iter()
            .map(|id| self.spec.link_index(*id).map(|i| self.tree.links[i].1))
            .collect::<Result<Vec<_>>>()?;
        let be = self
            .tree
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .expect("owned native tree");
        let geometry = self.tree.contact_spheres(self.spec.receivers())?;
        match be
            .articulated_sweep(&ids, Duration::from_nanos(self.spec.interval_ns()))
            .map_err(NativeError::from)?
        {
            ArticulatedSweep::Translation { velocity } => {
                Ok(Subreply::Motion { geometry, velocity })
            }
            ArticulatedSweep::Revolution {
                pivot,
                axis,
                angular_speed,
            } if self.spec.rotor().is_some() => Ok(Subreply::Orbit {
                geometry,
                pivot,
                axis,
                angular_speed,
            }),
            _ => Err(invalid(
                "rotation requires the world-revolute material profile",
            )),
        }
    }
    fn req(&mut self, parent: [u8; 32], generation: u64, op: Subrequest) -> Result<Subreply> {
        match (self.phase, op) {
            (Phase::Accepted, Subrequest::Motion) if !self.consumed => self.swept_spheres(),
            (Phase::Accepted, Subrequest::Head { nanos })
                if !self.consumed && nanos <= self.spec.interval_ns() =>
            {
                self.substep(nanos)?;
                let state = self.tree.observation_at(self.tick);
                let geometry = self.tree.contact_spheres(self.spec.receivers())?;
                let mut h = Sha256::new();
                h.update(b"GZATWK01 private geometry");
                h.update(parent);
                h.update(generation.to_le_bytes());
                h.update(self.tick.to_le_bytes());
                h.update(nanos.to_le_bytes());
                bincode::serialize_into(&mut h, &(&state, &geometry, self.image()?))
                    .map_err(std::io::Error::other)?;
                let stamp = h.finalize().into();
                self.phase = Phase::Head { nanos, stamp };
                Ok(Subreply::Head {
                    geometry,
                    state,
                    stamp,
                })
            }
            (
                Phase::Head { stamp, .. },
                Subrequest::Sample {
                    stamp: proof,
                    sites,
                },
            ) if proof == stamp => {
                let basis = self.tree.point_basis(&sites)?;
                let seal = basis.seal(stamp, generation, self.tick, &sites);
                Ok(Subreply::Sample { basis, seal })
            }
            (
                Phase::Head { stamp, nanos },
                Subrequest::Apply {
                    stamp: proof,
                    sites,
                    seal,
                    impulses,
                },
            ) if proof == stamp => {
                if impulses.len() != sites.len() || impulses.iter().all(|j| *j == [0.; 3]) {
                    return Err(invalid("missing signed automatic material impulse"));
                }
                let receipt = self
                    .tree
                    .commit_point_impulses(&sites, &impulses, seal, stamp, generation, self.tick)?;
                self.phase = Phase::Applied { nanos, stamp };
                Ok(Subreply::Applied(receipt))
            }
            _ => Err(invalid(
                "stale native material subinterval proof or invalid phase",
            )),
        }
    }
}
impl<S: AutomaticConstruction> PagedWorld for TimedWorld<S> {
    type Spec = S;
    type Version = ();
    const LIMIT: usize = automatic::NATIVE_LIMIT;
    fn restore_bytes(s: &Self::Spec, (): (), tick: u64, bytes: &[u8]) -> Result<Self> {
        let c: TimedCut = automatic::decode(bytes, Self::LIMIT)?;
        c.check(s)?;
        if c.state.tick != tick {
            return Err(invalid("timed native restore clock differs"));
        }
        let mut w = Self::new(s)?;
        if let Some(image) = &c.image {
            let mut b = w.tree.sim.world_mut().resource_mut::<PhysicsBackend>();
            let native = b
                .downcast_mut::<MultiFidelityBackend>()
                .expect("owned native tree");
            if (if s.rotor().is_some() {
                native.restore_timed_world_revolute(image)
            } else {
                native.restore_timed_revolute_tree(image)
            })
            .map_err(NativeError::from)?
                != Duration::from_nanos(c.last_ns)
            {
                return Err(invalid("timed native duration disagrees with public cut"));
            }
        }
        w.tick = tick;
        w.consumed = c.consumed;
        w.last = c.last_ns;
        if w.capture_bytes(tick)? != bytes {
            return Err(invalid(
                "time-bound tree restore changed accepted native motion",
            ));
        }
        Ok(w)
    }
    fn capture_bytes(&self, tick: u64) -> Result<Vec<u8>> {
        if tick != self.tick {
            return Err(invalid("private timed tree clock differs"));
        }
        automatic::encode(&self.cut()?, Self::LIMIT)
    }
    fn queue(&mut self, _id: u64, _effort: f64) -> Result<()> {
        Err(invalid("automatic tree must be passive"))
    }
    fn motor(&mut self, _id: u64, _m: Option<MotorSetting>) -> Result<()> {
        Err(invalid("automatic tree must be passive"))
    }
    fn advance(&mut self, _e: &[(u64, f64)], _f: &[(u64, [f64; 3])]) -> Result<()> {
        Err(invalid(
            "automatic tree advances through checked native subintervals",
        ))
    }
    fn private(&mut self, source: [u8; 32], g: u64, bytes: &[u8]) -> Result<Vec<u8>> {
        let req = automatic::decode(bytes, Self::LIMIT)?;
        automatic::encode(&self.req(source, g, req)?, Self::LIMIT)
    }
    fn finish_private(&mut self, bytes: &[u8]) -> Result<()> {
        let proof: Option<[u8; 32]> = automatic::decode(bytes, Self::LIMIT)?;
        let head = match (self.phase, proof) {
            (Phase::Accepted, None) => 0,
            (Phase::Applied { nanos, stamp }, Some(proof)) if stamp == proof => {
                self.consumed = true;
                nanos
            }
            _ => {
                return Err(invalid(
                    "automatic material tail requires its own applied head",
                ));
            }
        };
        self.substep(self.spec.interval_ns() - head)?;
        self.tick = self
            .tick
            .checked_add(1)
            .ok_or(invalid("timed native clock exhausted"))?;
        self.phase = Phase::Accepted;
        Ok(())
    }
}

impl TreeWorld {
    fn new_rotor(p: &super::super::automatic::RotorPadSpec) -> Result<Self> {
        let mut backend = MultiFidelityBackend::new([0.; 3]).map_err(NativeError::from)?;
        let body = p.link.spawn(&mut backend).map_err(NativeError::from)?;
        backend
            .set_self_collision_group(body, Some(p.model_id))
            .map_err(NativeError::from)?;
        let joint = backend
            .add_limited_revolute(None, body, p.anchor, p.axis, [p.lower, p.upper])
            .map_err(NativeError::from)?;
        let mut sim = Simulation::new(DT).map_err(NativeError::from)?;
        gz_physics::install(&mut sim, backend).map_err(NativeError::from)?;
        Ok(Self {
            sim,
            links: vec![(p.link.id, body)],
            joints: vec![(p.joint_id, joint)],
            recipe: Sha256::digest(automatic::encode(p, 4096)?).into(),
            motorized: false,
            ground: None,
            payload: None,
        })
    }
}
