// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodyId, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{Simulation, UpdateInfo};
use std::{io, path::Path, time::Duration};

#[derive(Serialize, Deserialize)]
pub(super) enum Seed {
    Install(Piece),
    Combine(Box<Snapshot>, ArticulationLinkState),
}
#[derive(Serialize, Deserialize)]
pub(super) struct Request {
    pub generation: u64,
    pub op: Op,
}
#[derive(Serialize, Deserialize)]
pub(super) enum Op {
    Stage(Seed),
    Activate([u8; 32]),
    Fence([u8; 32]),
    Detach,
    Queue(u64, f64),
    Motor(u64, Option<MotorSetting>),
    Step {
        efforts: Vec<(u64, f64)>,
        force: [f64; 3],
        bounds: bool,
    },
    ClearMember,
    WarmMember,
    PriorWarm {
        prior_tick: u64,
        prior_digest: [u8; 32],
        trial_digest: [u8; 32],
    },
}
#[derive(Default, Serialize, Deserialize)]
pub(super) struct Bounds {
    pub tree: Vec<Box3>,
    pub part: Option<Box3>,
}
impl Bounds {
    fn swept(self, end: Self, extra: [f64; 3]) -> Self {
        let part = self.part.zip(end.part).map(|(a, b)| {
            let mut b = a.union(b);
            for (i, m) in extra.iter().enumerate() {
                b.lo[i] -= m.abs();
                b.hi[i] += m.abs();
            }
            b
        });
        let tree = if self.tree.len() == end.tree.len() {
            self.tree
                .into_iter()
                .zip(end.tree)
                .map(|(a, b)| a.union(b))
                .collect()
        } else {
            Vec::new()
        };
        Self { tree, part }
    }
    pub fn clear(&self, part: &Self, count: usize) -> bool {
        self.tree.len() == count
            && part
                .part
                .is_some_and(|p| self.tree.iter().all(|b| b.disjoint(p)))
    }
}
#[derive(Serialize, Deserialize)]
pub(super) enum Reply {
    Ack,
    State(Piece),
    Advanced(u64, Piece, Bounds),
    Detached(Option<Box<Snapshot>>),
    ClearMember(Option<Vec<u8>>),
    PriorWarm {
        prior_digest: [u8; 32],
        trial_digest: [u8; 32],
        packet: Option<Vec<u8>>,
    },
}
struct PartWorld {
    sim: Simulation,
    id: BodyId,
    public: u64,
}
impl PartWorld {
    fn restore(spec: &GraspTreeSpec, tick: u64, state: &ArticulationLinkState) -> Result<Self> {
        if tick == 0 || !valid_link(state, spec.payload.payload.id) {
            return Err(invalid("invalid detached payload clock or body"));
        }
        let mut native = MultiFidelityBackend::new([0.; 3])?;
        let id = spec.payload.payload.spawn_dynamic(&mut native)?;
        native.keep_body_awake(id)?;
        native.place_detached_sphere(id, super::super::native::as_body(state))?;
        let dt = Duration::from_millis(1);
        let mut sim = Simulation::from_committed_clock(
            dt,
            UpdateInfo {
                start_time: Duration::from_millis(tick - 1),
                sim_time: Duration::from_millis(tick),
                dt,
                iteration: tick,
                epoch: 0,
                paused: false,
            },
            true,
        )
        .map_err(NativeError::from)?;
        gz_physics::install(&mut sim, native)?;
        Ok(Self {
            sim,
            id,
            public: state.id,
        })
    }
    fn capture(&self) -> Piece {
        let s = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .state(self.id)
            .expect("owned detached part");
        Piece::Part(ArticulationLinkState {
            id: self.public,
            position: s.position,
            orientation: s.orientation,
            velocity: s.velocity,
            angular_velocity: s.angular_velocity,
        })
    }
}
enum World {
    Coupled(TreeWorld),
    Tree(TreeWorld),
    Part(PartWorld),
}
impl World {
    fn stage(s: &GraspTreeSpec, tick: u64, seed: Seed) -> Result<(Self, Piece)> {
        let (world, expected) = match seed {
            Seed::Install(piece) => {
                if !matches!(&piece, Piece::Coupled(_)) && !s.partitionable() {
                    return Err(invalid("grasp recipe does not admit separate owners"));
                }
                let world = match &piece {
                    Piece::Coupled(c) if c.state.tick == tick => {
                        Self::Coupled(TreeWorld::restore(s, c)?)
                    }
                    Piece::Tree(c) if c.state.tick == tick && tick > 0 => {
                        Self::Tree(TreeWorld::restore(&s.bare_motor_spec(), c)?)
                    }
                    Piece::Part(p) => Self::Part(PartWorld::restore(s, tick, p)?),
                    _ => return Err(invalid("selective grasp stage clock differs")),
                };
                (world, Some(piece))
            }
            Seed::Combine(tree, part) => {
                if !s.partitionable() {
                    return Err(invalid("grasp recipe does not admit recombination"));
                }
                if tree.state.tick != tick {
                    return Err(invalid("incorrect grasp rejoin tick"));
                }
                let world = TreeWorld::combine_grasp(s, &tree, &part)?;
                let coupled = world.capture()?;
                if coupled.state != tree.state
                    || coupled.image.payload().is_none_or(|p| {
                        p.body != part
                            || p.reactions.iter().any(|r| *r != MovingReaction::default())
                    })
                    || coupled.image.motors() != tree.image.motors()
                {
                    return Err(invalid(
                        "rejoin changed physical state or inherited stale contact",
                    ));
                }
                return Ok((Self::Coupled(world), Piece::Coupled(Box::new(coupled))));
            }
        };
        let current = world.capture()?;
        if expected.as_ref() != Some(&current) {
            return Err(invalid(
                "restored grasp piece differs from its accepted state",
            ));
        }
        Ok((world, current))
    }
    fn capture(&self) -> Result<Piece> {
        Ok(match self {
            Self::Coupled(w) => Piece::Coupled(Box::new(w.capture()?)),
            Self::Tree(w) => Piece::Tree(Box::new(w.capture()?)),
            Self::Part(p) => p.capture(),
        })
    }
    fn bounds(&self) -> Bounds {
        match self {
            Self::Coupled(w) | Self::Tree(w) => w
                .collision_bounds()
                .map(|(tree, part)| Bounds { tree, part })
                .unwrap_or_default(),
            Self::Part(p) => {
                let b = p
                    .sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .expect("owned detached part");
                Bounds {
                    tree: vec![],
                    part: b
                        .grasp_collision_bounds(p.id, Duration::from_millis(1))
                        .ok()
                        .map(Box3::from),
                }
            }
        }
    }
    fn tree(&mut self) -> Result<&mut TreeWorld> {
        match self {
            Self::Coupled(w) | Self::Tree(w) => Ok(w),
            Self::Part(_) => Err(invalid("detached part has no joint controller")),
        }
    }
    fn advance(
        &mut self,
        s: &GraspTreeSpec,
        efforts: &[(u64, f64)],
        force: [f64; 3],
        bounds: bool,
    ) -> Result<Bounds> {
        if !force.iter().all(|v| v.is_finite()) {
            return Err(invalid("invalid payload force"));
        }
        match self {
            Self::Coupled(w) => {
                w.apply_efforts(efforts)?;
                if force != [0.; 3] {
                    w.load_payload(force)?;
                }
            }
            Self::Tree(w) => {
                if force != [0.; 3] {
                    return Err(invalid(
                        "free tree worker received an external payload force",
                    ));
                }
                w.apply_efforts(efforts)?;
            }
            Self::Part(p) => {
                if !efforts.is_empty() {
                    return Err(invalid("free part received tree torque"));
                }
                if force != [0.; 3] {
                    p.sim
                        .world_mut()
                        .resource_mut::<PhysicsBackend>()
                        .add_world_wrench(p.id, force, [0.; 3])?;
                }
            }
        }
        let start = bounds.then(|| self.bounds());
        match self {
            Self::Coupled(w) | Self::Tree(w) => w.advance()?,
            Self::Part(p) => p.sim.step_exact(1).map_err(NativeError::from)?,
        }
        Ok(start.map_or_else(Bounds::default, |box0| {
            box0.swept(
                self.bounds(),
                force.map(|x| x * 1e-6 / s.payload.payload.mass),
            )
        }))
    }
}
struct Slot {
    generation: u64,
    tick: u64,
    digest: [u8; 32],
    world: World,
    piece: Piece,
    prior: Option<Box<Snapshot>>,
}
#[derive(Default)]
pub(super) struct RoleWorker {
    active: Option<Slot>,
    staged: Option<Slot>,
    high: Option<u64>,
}
impl RoleWorker {
    pub fn handle(
        &mut self,
        spec: &GraspTreeSpec,
        tick: u64,
        req: Request,
        steps: &mut u64,
        retain_prior: bool,
    ) -> Result<Reply> {
        let generation = req.generation;
        match req.op {
            Op::Stage(seed) => {
                if self.staged.is_some() || self.high.is_some_and(|i| i >= generation) {
                    return Err(invalid("obsolete or duplicate grasp role preparation"));
                }
                let (world, piece) = World::stage(spec, tick, seed)?;
                let digest = piece.digest();
                self.staged = Some(Slot {
                    generation,
                    tick,
                    digest,
                    world,
                    piece: piece.clone(),
                    prior: None,
                });
                self.high = Some(generation);
                Ok(Reply::State(piece))
            }
            Op::Activate(digest) => {
                if self.active.is_some()
                    || self.staged.as_ref().is_none_or(|s| {
                        s.generation != generation || s.tick != tick || s.digest != digest
                    })
                {
                    return Err(invalid("incorrect grasp role activation"));
                }
                self.active = self.staged.take();
                Ok(Reply::Ack)
            }
            op => {
                let r = self
                    .active
                    .as_mut()
                    .filter(|s| s.generation == generation && s.tick == tick)
                    .ok_or(invalid("stale active grasp role or clock"))?;
                match op {
                    Op::Fence(d) if d == r.digest => {
                        self.active = None;
                        Ok(Reply::Ack)
                    }
                    Op::Fence(_) => Err(invalid("incorrect fenced grasp role digest")),
                    Op::PriorWarm {
                        prior_tick,
                        prior_digest,
                        trial_digest,
                    } => {
                        let prior = r
                            .prior
                            .as_deref()
                            .ok_or(invalid("missing retained grasp source"))?;
                        let current = match &r.piece {
                            Piece::Coupled(c) => c,
                            _ => return Err(invalid("missing current grasp trial")),
                        };
                        if !retain_prior
                            || prior_tick.checked_add(1) != Some(r.tick)
                            || prior.state.tick != prior_tick
                            || prior.digest != prior_digest
                            || current.digest != trial_digest
                        {
                            return Err(invalid(
                                "incorrect retained interval, prior digest or trial digest",
                            ));
                        }
                        let packet = TreeWorld::restore(spec, prior)?.member_packet(spec, true)?;
                        Ok(Reply::PriorWarm {
                            prior_digest,
                            trial_digest,
                            packet,
                        })
                    }
                    Op::ClearMember | Op::WarmMember => match &r.world {
                        World::Coupled(w) => Ok(Reply::ClearMember(
                            w.member_packet(spec, matches!(op, Op::WarmMember))?,
                        )),
                        _ => Err(invalid(
                            "clear-fleet export requires an independent coupled member",
                        )),
                    },
                    Op::Detach => match (&r.world, &r.piece) {
                        (World::Coupled(w), Piece::Coupled(snap)) => {
                            Ok(Reply::Detached(w.detach_grasp(spec, snap)?.map(Box::new)))
                        }
                        _ => Err(invalid("detachment requires a coupled role")),
                    },
                    Op::Queue(id, effort) => {
                        r.prior = None;
                        r.world.tree()?.effort(id, effort)?;
                        r.piece = r.world.capture()?;
                        r.digest = r.piece.digest();
                        Ok(Reply::State(r.piece.clone()))
                    }
                    Op::Motor(id, motor) => {
                        r.prior = None;
                        if motor.is_some_and(|m| !m.valid()) {
                            return Err(invalid("invalid grasp motor target"));
                        }
                        r.world.tree()?.set_motor(id, motor)?;
                        r.piece = r.world.capture()?;
                        r.digest = r.piece.digest();
                        Ok(Reply::State(r.piece.clone()))
                    }
                    Op::Step {
                        efforts,
                        force,
                        bounds,
                    } => {
                        let boxes = r.world.advance(spec, &efforts, force, bounds)?;
                        r.tick = r
                            .tick
                            .checked_add(1)
                            .ok_or(invalid("grasp worker clock exhausted"))?;
                        *steps = steps
                            .checked_add(1)
                            .ok_or(invalid("grasp worker step counter exhausted"))?;
                        let next = r.world.capture()?;
                        let old = std::mem::replace(&mut r.piece, next);
                        r.prior = match old {
                            Piece::Coupled(s) if retain_prior => Some(s),
                            _ => None,
                        };
                        r.digest = r.piece.digest();
                        Ok(Reply::Advanced(r.tick, r.piece.clone(), boxes))
                    }
                    _ => Err(invalid("invalid grasp role command")),
                }
            }
        }
    }
}
/// Serve one private, leased worker capable of independent tree, part, or coupled roles.
pub fn serve(slot: u8, lease: &Path) -> io::Result<()> {
    if slot >= 8 || !lease.is_absolute() {
        return Err(io::Error::other("invalid selective grasp worker slot"));
    }
    crate::moving::worker::serve_profile(slot, lease, WIRE, |spec: GraspTreeSpec| {
        spec.validate()?;
        let mut role = RoleWorker::default();
        Ok(move |tick, req: Request, steps: &mut u64| role.handle(&spec, tick, req, steps, false))
    })
}

#[cfg(test)]
mod retained_tests {
    use super::*;
    #[test]
    fn prior_export_binds_trial_digest_tick_recipe_and_active_lease()
    -> std::result::Result<(), Box<dyn std::error::Error>> {
        let xml = include_str!("../../../../../examples/distributed_gripper_payload.sdf");
        let ordinary = gz_sim_scene::distributed_hinge::distributed_payload_tree_spec(
            xml, "grip.sdf", "default",
        )?;
        let spec = GraspTreeSpec {
            payload: serde_json::from_value(serde_json::to_value(ordinary)?)?,
        };
        let mut world = TreeWorld::new(&spec)?;
        for j in &spec.payload.tree.joints {
            let palm = spec.payload.tree.links[0].position[1];
            let child = spec
                .payload
                .tree
                .links
                .iter()
                .find(|l| l.id == j.child_id)
                .unwrap();
            world.set_motor(
                j.joint_id,
                Some(MotorSetting::from(gz_physics::JointVelocityMotor {
                    target_velocity: -6. * (child.position[1] - palm).signum(),
                    max_force: 4.,
                })),
            )?;
        }
        for _ in 0..500 {
            world.advance()?;
        }
        let cut = world.capture()?;
        let before = cut.digest;
        let expected = world.member_packet(&spec, true)?.unwrap();
        let piece = Piece::Coupled(Box::new(cut));
        let digest = piece.digest();
        let mut role = RoleWorker::default();
        let mut steps = 0;
        let mut request = |tick, generation, op| {
            role.handle(&spec, tick, Request { generation, op }, &mut steps, true)
        };
        request(500, 7, Op::Stage(Seed::Install(piece)))?;
        request(500, 7, Op::Activate(digest))?;
        let Reply::Advanced(_, Piece::Coupled(s), _) = request(
            500,
            7,
            Op::Step {
                efforts: vec![],
                force: [0.; 3],
                bounds: true,
            },
        )?
        else {
            panic!("missing served trial")
        };
        let after = s.digest;
        let proof = |prior_tick, prior_digest, trial_digest| Op::PriorWarm {
            prior_tick,
            prior_digest,
            trial_digest,
        };
        for (tick, lease, pt, a, b) in [
            (500, 7, 500, before, after),
            (501, 6, 500, before, after),
            (501, 7, 499, before, after),
            (501, 7, 500, [1; 32], after),
            (501, 7, 500, before, [2; 32]),
        ] {
            assert!(request(tick, lease, proof(pt, a, b)).is_err());
        }
        for _ in 0..2 {
            let Reply::PriorWarm {
                prior_digest,
                trial_digest,
                packet: Some(packet),
            } = request(501, 7, proof(500, before, after))?
            else {
                panic!("missing exact retained packet")
            };
            assert_eq!((prior_digest, trial_digest), (before, after));
            assert_eq!(packet, expected);
        }
        request(
            501,
            7,
            Op::Queue(spec.payload.tree.joints[0].joint_id, 0.001),
        )?;
        assert!(request(501, 7, proof(500, before, after)).is_err());
        Ok(())
    }
}
