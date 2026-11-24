// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodyId, BodyState, DynamicBackend, JointId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;

#[derive(Clone, Serialize, Deserialize)]
pub(super) enum Motion {
    Cold(Option<f64>),
    Warm(Vec<u8>),
}

impl Motion {
    pub(super) fn pending_effort(&self) -> Result<Option<f64>> {
        match self {
            Self::Cold(Some(x)) if !x.is_finite() => Err(invalid("nonfinite moving hinge effort")),
            Self::Cold(effort) => Ok(*effort),
            Self::Warm(bytes) => Ok(MultiFidelityBackend::moving_checkpoint_effort(bytes)?),
        }
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub(super) enum Seed {
    Hinge(Motion),
    Sphere(ArticulationLinkState),
    Joined(Motion),
    Combine(Motion, ArticulationLinkState),
}

#[derive(Clone, Serialize, Deserialize)]
pub(super) struct HingeSample {
    pub links: [ArticulationLinkState; 2],
    pub q: f64,
    pub qdot: f64,
}

#[derive(Clone, Serialize, Deserialize)]
pub(super) enum Part {
    Hinge(Motion, HingeSample),
    Sphere(ArticulationLinkState),
    Joined(
        Motion,
        Box<HingeSample>,
        ArticulationLinkState,
        [MovingReaction; 2],
    ),
}

pub(super) struct Resident {
    sim: Simulation,
    tick: u64,
    hinge: Option<([BodyId; 2], JointId)>,
    sphere: Option<BodyId>,
    held: Option<f64>,
}

impl MovingSpec {
    pub fn validate(&self) -> Result<()> {
        self.hinge.validate()?;
        let s = &self.sphere;
        if self.hinge.ground.is_some()
            || s.id
                <= self
                    .hinge
                    .joint_id
                    .max(self.hinge.model_id)
                    .max(self.hinge.links[0].id)
                    .max(self.hinge.links[1].id)
            || s.inertia_center != [0.; 3]
            || s.inertia_orientation != [0., 0., 0., 1.]
            || s.inertia_tensor[0] != s.inertia_tensor[1]
            || s.inertia_tensor[1] != s.inertia_tensor[2]
            || s.inertia_tensor[3..] != [0.; 3]
        {
            return Err(invalid(
                "expected a zero-gravity hinge followed by one centered isotropic sphere",
            ));
        }
        let mut world = MultiFidelityBackend::new([0.; 3])?;
        self.spawn_sphere(&mut world)?;
        Ok(())
    }

    pub(crate) fn spawn_sphere(&self, world: &mut MultiFidelityBackend) -> Result<BodyId> {
        let id = self.sphere.spawn_dynamic(world)?;
        world.configure_moving_body(id)?;
        Ok(id)
    }
    pub(crate) fn initial_sphere(&self) -> ArticulationLinkState {
        let s = self.sphere;
        ArticulationLinkState {
            id: s.id,
            position: s.position,
            orientation: s.orientation,
            velocity: s.velocity,
            angular_velocity: s.angular_velocity,
        }
    }
}

impl Resident {
    pub fn load(spec: &MovingSpec, tick: u64, seed: &Seed) -> Result<Self> {
        let mut hinge = None;
        let mut sphere = None;
        let mut held = None;
        let mut world = if matches!(seed, Seed::Sphere(_)) {
            MultiFidelityBackend::new([0.; 3])?
        } else {
            let mut c = spec.hinge.build()?;
            for id in c.bodies {
                c.backend.configure_moving_body(id)?;
            }
            hinge = Some((c.bodies, c.joint));
            c.backend
        };
        if let Seed::Hinge(m) | Seed::Combine(m, _) = seed {
            match m {
                Motion::Cold(e) if tick == 0 => held = *e,
                Motion::Warm(bytes) if tick > 0 => world.restore_floating_revolute(bytes)?,
                _ => return Err(invalid("moving hinge image and tick differ")),
            }
        }
        if !matches!(seed, Seed::Hinge(_)) {
            sphere = Some(spec.spawn_sphere(&mut world)?);
        }
        if let Seed::Sphere(s) | Seed::Combine(_, s) = seed {
            if s.id != spec.sphere.id {
                return Err(invalid("sphere identity differs"));
            }
            if tick > 0 {
                world.place_detached_sphere(
                    sphere.unwrap(),
                    BodyState {
                        position: s.position,
                        orientation: s.orientation,
                        velocity: s.velocity,
                        angular_velocity: s.angular_velocity,
                    },
                )?;
            } else if *s != spec.initial_sphere() {
                return Err(invalid("incorrect cold sphere pose"));
            }
        }
        if let Seed::Joined(m) = seed {
            match m {
                Motion::Cold(e) if tick == 0 => held = *e,
                Motion::Warm(bytes) if tick > 0 => world.restore_moving_revolute(bytes)?,
                _ => return Err(invalid("joined image and tick differ")),
            }
        }
        if let Some(e) = held {
            world.add_joint_effort(hinge.unwrap().1, e)?;
        }
        let mut sim =
            Simulation::new(std::time::Duration::from_millis(1)).map_err(NativeError::from)?;
        gz_physics::install(&mut sim, world).map_err(NativeError::from)?;
        Ok(Self {
            sim,
            tick,
            hinge,
            sphere,
            held,
        })
    }

    fn world(&self) -> &MultiFidelityBackend {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref()
            .expect("moving native backend")
    }
    fn body(&self, id: BodyId, public: u64) -> ArticulationLinkState {
        let s = self.world().state(id).expect("moving native body");
        ArticulationLinkState {
            id: public,
            position: s.position,
            orientation: s.orientation,
            velocity: s.velocity,
            angular_velocity: s.angular_velocity,
        }
    }
    pub fn capture(&self, spec: &MovingSpec) -> Result<Part> {
        let sphere = self.sphere.map(|id| self.body(id, spec.sphere.id));
        let Some((links, joint)) = self.hinge else {
            return Ok(Part::Sphere(sphere.unwrap()));
        };
        let joint = self
            .world()
            .joint_state(joint)
            .expect("moving native joint");
        let state = HingeSample {
            links: std::array::from_fn(|i| self.body(links[i], spec.hinge.links[i].id)),
            q: joint.position,
            qdot: joint.velocity,
        };
        let motion = if self.tick == 0 {
            Motion::Cold(self.held)
        } else if sphere.is_some() {
            Motion::Warm(self.world().checkpoint_moving_revolute()?)
        } else {
            Motion::Warm(self.world().checkpoint_floating_revolute()?)
        };
        if let Some(s) = sphere {
            let obstacle = self.sphere.unwrap();
            let contacts = [
                self.world().moving_reaction(links[0], obstacle)?,
                self.world().moving_reaction(links[1], obstacle)?,
            ];
            Ok(Part::Joined(motion, Box::new(state), s, contacts))
        } else {
            Ok(Part::Hinge(motion, state))
        }
    }
    pub fn advance(&mut self) -> Result<()> {
        self.sim.step_exact(1).map_err(NativeError::from)?;
        self.tick = self
            .tick
            .checked_add(1)
            .ok_or(invalid("moving tick exhausted"))?;
        self.held = None;
        Ok(())
    }
    pub fn effort(&mut self, e: f64) -> Result<()> {
        let joint = self.hinge.ok_or(invalid("no hinge in sphere slot"))?.1;
        self.sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_joint_effort(joint, e)?;
        if self.tick == 0 {
            self.held = Some(self.held.unwrap_or(0.) + e)
        }
        Ok(())
    }
    pub fn detached(&self, spec: &MovingSpec) -> Result<Option<(Motion, ArticulationLinkState)>> {
        let sphere = self.sphere.ok_or(invalid("missing moving sphere"))?;
        let hinge = match self.world().detached_hinge_image(sphere) {
            Ok(bytes) => Motion::Warm(bytes),
            Err(gz_physics::PhysicsError::Unsupported { .. }) => return Ok(None),
            Err(e) => return Err(e.into()),
        };
        Ok(Some((hinge, self.body(sphere, spec.sphere.id))))
    }
}
