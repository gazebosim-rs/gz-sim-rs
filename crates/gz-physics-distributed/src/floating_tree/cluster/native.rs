// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodyId, DynamicBackend, JointId, PhysicsBackend};
use gz_physics_multifidelity::{ArticulationTier, MultiFidelityBackend};
use gz_sim_core::{Simulation, UpdateInfo};
use std::{collections::BTreeMap, time::Duration};
const DT: Duration = Duration::from_millis(1);
#[derive(Default)]
struct Group {
    root: Option<BodyId>,
    links: Vec<(u64, BodyId)>,
    joints: Vec<(u64, JointId)>,
    part: Option<(u64, BodyId)>,
}
#[derive(Clone, Copy)]
enum Bootstrap<'a> {
    Cold,
    Checkpoint(&'a ClusterSnapshot),
    Members {
        prior: &'a crate::floating_tree::placement::fleet::FleetCut,
        members: [&'a [u8]; 2],
        warm: bool,
    },
}
pub(in crate::floating_tree) struct ClusterWorld {
    sim: Simulation,
    version: Version,
    handles: [Group; 2],
    pairs: Vec<([u64; 4], BodyId, BodyId)>,
}
impl ClusterWorld {
    pub fn new(s: &GraspFleetSpec, version: Version) -> Result<Self> {
        Self::from_cut(s, Bootstrap::Cold, version)
    }
    pub fn restore(s: &GraspFleetSpec, snap: &ClusterSnapshot, version: Version) -> Result<Self> {
        snap.check(s, version)?;
        let w = Self::from_cut(s, Bootstrap::Checkpoint(snap), version)?;
        if w.capture()? != *snap {
            return Err(invalid(
                "warm coupled grasp graph changed after restoration",
            ));
        }
        Ok(w)
    }
    pub fn restore_members(
        s: &GraspFleetSpec,
        prior: &crate::floating_tree::placement::fleet::FleetCut,
        bytes: [&[u8]; 2],
        warm: bool,
    ) -> Result<Self> {
        let mut w = if prior.tick > 0 {
            Self::from_cut(
                s,
                Bootstrap::Members {
                    prior,
                    members: bytes,
                    warm,
                },
                Version::ObservedShear,
            )?
        } else {
            Self::new(s, Version::ObservedShear)?
        };
        if prior.tick == 0 {
            for group in &prior.groups {
                let motors = group
                    .image
                    .motors()
                    .ok_or(invalid("missing initial grasp motor settings"))?;
                for (j, setting) in group.state.joints.iter().zip(motors) {
                    if let Some(e) = j.pending_effort {
                        w.queue(j.joint_id, e)?;
                    }
                    if setting.is_some() {
                        w.motor(j.joint_id, *setting)?;
                    }
                }
            }
        }
        let snap = w.capture()?;
        for (got, old) in snap.members.iter().zip(&prior.groups) {
            if got.tree != old.state
                || old.image.payload() != Some(&got.part)
                || old.image.motors() != Some(&got.motors)
            {
                return Err(invalid(
                    "merged grasp graph does not reproduce all accepted independent body, joint, contact and motor rows",
                ));
            }
        }
        if snap.pairs.iter().any(|p| {
            p.reaction.active
                || p.reaction.normal_impulse != 0.
                || p.reaction.warmstart_normal_impulse != 0.
                || p.reaction.tangent_impulse != [0.; 3]
        }) {
            return Err(invalid(
                "unexpected interassembly contact already present at the independent cut",
            ));
        }
        Ok(w)
    }
    fn from_cut(s: &GraspFleetSpec, seed: Bootstrap, version: Version) -> Result<Self> {
        admissible(s)?;
        let mut engine = MultiFidelityBackend::new([0.; 3])?;
        let mut handles: [Group; 2] = Default::default();
        let mut order = Vec::with_capacity(4);
        for (i, a) in s.assemblies.iter().enumerate() {
            order.push((a.payload.tree.links[0].id, i, false));
            order.push((a.payload.payload.id, i, true));
        }
        order.sort_unstable();
        let mut bodies = BTreeMap::new();
        for (_, i, external) in order {
            let setup = &s.assemblies[i].payload;
            let group = &mut handles[i];
            if external {
                let l = &setup.payload;
                let id = l.spawn_dynamic(&mut engine)?;
                if !setup.payload_can_sleep {
                    engine.keep_body_awake(id)?;
                }
                group.part = Some((l.id, id));
                bodies.insert(l.id, id);
                continue;
            }
            let tree = &setup.tree;
            let root = tree.root()?;
            for l in &tree.links {
                let id = l.spawn(&mut engine)?;
                engine.set_self_collision_group(id, Some(tree.model_id))?;
                group.links.push((l.id, id));
                bodies.insert(l.id, id);
            }
            group.root = Some(group.links[root].1);
            for j in &tree.joints {
                let p = bodies[&j.parent_id];
                let c = bodies[&j.child_id];
                let id = match setup
                    .limits
                    .binary_search_by_key(&j.joint_id, |v| v.joint_id)
                {
                    Ok(index) => engine.add_limited_revolute(
                        Some(p),
                        c,
                        j.anchor,
                        j.axis,
                        [setup.limits[index].lower, setup.limits[index].upper],
                    )?,
                    Err(_) => engine.add_reduced_revolute(Some(p), c, j.anchor, j.axis)?,
                };
                group.joints.push((j.joint_id, id));
            }
            if setup.exact_revolute_locks {
                engine.enable_exact_revolute_locks(group.root.unwrap())?;
            }
            group.joints.sort_unstable_by_key(|j| j.0);
        }
        for g in &handles {
            engine.switch_articulation(g.root.unwrap(), ArticulationTier::Dynamic, DT)?;
        }
        let key = handles
            .each_ref()
            .map(|g| (g.root.unwrap(), g.part.unwrap().1));
        if version == Version::ObservedShear {
            engine.configure_grasp_payload_pair(key.map(|(_, part)| part))?;
        }
        let tick = match seed {
            Bootstrap::Cold => 0,
            Bootstrap::Checkpoint(snap) => snap.tick,
            Bootstrap::Members { prior, .. } => prior.tick,
        };
        if let Bootstrap::Members { members, warm, .. } = seed {
            if warm && members.iter().any(|p| p.starts_with(b"GZFWAM01")) {
                engine.prepare_warm_grasp_fleet(&key, members)?;
            } else {
                engine.restore_clear_grasp_fleet(&key, members)?;
            }
        }
        if let Bootstrap::Checkpoint(snap) = seed {
            if let Some(bytes) = &snap.image {
                engine.restore_grasp_fleet(&key, bytes)?;
            } else {
                for (g, m) in handles.iter().zip(&snap.members) {
                    for (((_, h), row), motor) in g.joints.iter().zip(&m.tree.joints).zip(&m.motors)
                    {
                        if let Some(e) = row.pending_effort {
                            engine.add_joint_effort(*h, e)?;
                        }
                        if let Some(v) = motor {
                            engine.set_joint_velocity_motor(*h, Some(v.native()))?;
                        }
                    }
                }
            }
        }
        let mut sim = if tick == 0 {
            Simulation::new(DT)
        } else {
            Simulation::from_committed_clock(
                DT,
                UpdateInfo {
                    start_time: Duration::from_millis(tick - 1),
                    sim_time: Duration::from_millis(tick),
                    dt: DT,
                    iteration: tick,
                    epoch: 0,
                    paused: false,
                },
                true,
            )
        }
        .map_err(NativeError::from)?;
        gz_physics::install(&mut sim, engine)?;
        let pairs = public_pairs(s)
            .into_iter()
            .map(|k| (k, bodies[&k[1]], bodies[&k[3]]))
            .collect();
        Ok(Self {
            sim,
            version,
            handles,
            pairs,
        })
    }
    fn joint(&self, id: u64) -> Result<JointId> {
        self.handles
            .iter()
            .find_map(|g| {
                g.joints
                    .binary_search_by_key(&id, |j| j.0)
                    .ok()
                    .map(|i| g.joints[i].1)
            })
            .ok_or(invalid("unknown coupled fleet joint"))
    }
    pub fn queue(&mut self, id: u64, e: f64) -> Result<()> {
        let joint = self.joint(id)?;
        self.sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .add_joint_effort(joint, e)?;
        Ok(())
    }
    pub fn motor(&mut self, id: u64, setting: Option<MotorSetting>) -> Result<()> {
        if setting.is_some_and(|m| !m.valid()) {
            return Err(invalid("invalid coupled fleet motor"));
        }
        let joint = self.joint(id)?;
        self.sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_joint_velocity_motor(joint, setting.map(MotorSetting::native))?;
        Ok(())
    }
    pub fn advance(&mut self, efforts: &[(u64, f64)], forces: &[(u64, [f64; 3])]) -> Result<()> {
        let mut seen = BTreeSet::new();
        for &(id, e) in efforts {
            if !seen.insert(id) || !e.is_finite() {
                return Err(invalid("invalid or duplicate coupled fleet torque"));
            }
            self.queue(id, e)?;
        }
        for &(id, force) in forces {
            let part = self
                .handles
                .iter()
                .find_map(|g| g.part.filter(|v| v.0 == id))
                .ok_or(invalid("unknown coupled fleet payload"))?
                .1;
            if !seen.insert(id) || !force.iter().all(|v| v.is_finite()) {
                return Err(invalid("invalid or duplicate coupled fleet force"));
            }
            if force != [0.; 3] {
                self.sim
                    .world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .add_world_wrench(part, force, [0.; 3])?;
            }
        }
        self.sim.step_exact(1).map_err(NativeError::from)?;
        Ok(())
    }
    pub fn capture_without_image(&self) -> Result<ClusterSnapshot> {
        self.sample(false)
    }
    pub fn capture(&self) -> Result<ClusterSnapshot> {
        self.sample(true)
    }
    fn sample(&self, include_image: bool) -> Result<ClusterSnapshot> {
        let tick = self
            .sim
            .committed_update_info()
            .map_err(NativeError::from)?
            .iteration;
        let backend = self.sim.world().resource::<PhysicsBackend>();
        let solver = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
        let body = |(id, h): (u64, BodyId)| {
            let s = backend.state(h).expect("owned coupled body");
            ArticulationLinkState {
                id,
                position: s.position,
                orientation: s.orientation,
                velocity: s.velocity,
                angular_velocity: s.angular_velocity,
            }
        };
        let mut members = Vec::with_capacity(2);
        for g in &self.handles {
            let links = g.links.iter().copied().map(body).collect();
            let mut joints = Vec::new();
            let mut motors = Vec::new();
            for &(joint_id, h) in &g.joints {
                let row = backend.joint_state(h).unwrap();
                joints.push(FloatingTreeJointState {
                    joint_id,
                    q: row.position,
                    qdot: row.velocity,
                    pending_effort: solver.pending_joint_effort(h),
                });
                motors.push(solver.motor_revolute_velocity(h)?.map(MotorSetting::from));
            }
            let part = g.part.unwrap();
            let reactions = g
                .links
                .iter()
                .map(|&(_, id)| solver.payload_tree_reaction(id, part.1))
                .collect::<std::result::Result<_, _>>()?;
            members.push(Member {
                tree: FloatingTreeObservation {
                    tick,
                    links,
                    joints,
                },
                motors,
                part: PayloadReceipt {
                    body: body(part),
                    reactions,
                },
            });
        }
        let pairs = self
            .pairs
            .iter()
            .map(|&(k, a, b)| {
                Ok(GraspFleetPairReaction {
                    first_model: k[0],
                    first_body: k[1],
                    second_model: k[2],
                    second_body: k[3],
                    reaction: match self.version {
                        Version::Legacy => solver.sphere_pair_reaction_legacy(a, b)?,
                        Version::ObservedShear => solver.sphere_pair_reaction(a, b)?,
                    },
                })
            })
            .collect::<Result<_>>()?;
        let key = self
            .handles
            .each_ref()
            .map(|g| (g.root.unwrap(), g.part.unwrap().1));
        let image = if tick == 0 || !include_image {
            None
        } else {
            Some(solver.checkpoint_grasp_fleet(&key)?)
        };
        Ok(ClusterSnapshot {
            tick,
            members,
            pairs,
            image,
        })
    }
}
