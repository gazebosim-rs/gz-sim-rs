// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::articulation::ArticulationLinkState;
use crate::floating_tree::{FloatingTreeJointState, MotorSetting};
use crate::paged_native::PagedWorld;
use gz_physics::{
    BodyId, BodySpec, DynamicBackend, Inertia, JointId, PhysicsBackend, RotationSpec, Shape,
};
use gz_physics_multifidelity::{ArticulationTier, MultiFidelityBackend};
use gz_sim_core::{Simulation, UpdateInfo};
use std::{io, time::Duration};
const DT: Duration = Duration::from_millis(1);

pub(super) struct RobotWorld {
    spec: GroundedRobotSpec,
    sim: Simulation,
    links: Vec<(u64, BodyId)>,
    joints: Vec<(u64, JointId)>,
    root: BodyId,
    ground: BodyId,
}
impl RobotWorld {
    pub(super) fn new(
        spec: &GroundedRobotSpec,
        tick: u64,
        snapshot: Option<&Snapshot>,
    ) -> std::result::Result<Self, NativeError> {
        let root = spec.root()?;
        let mut be = MultiFidelityBackend::new([0., 0., spec.ground.gravity_z])?;
        let mut ground = if spec.ground.link_id < spec.links[0].id {
            Some(be.spawn_static_plane(spec.ground.plane())?)
        } else {
            None
        };
        let mut links = Vec::with_capacity(spec.links.len());
        for l in &spec.links {
            let body = be.spawn_articulation_link(
                BodySpec {
                    shape: Shape::Box {
                        half_extents: l.size.map(|x| x * 0.5),
                    },
                    mass: Some(l.mass),
                    position: l.position,
                    velocity: l.velocity,
                },
                RotationSpec {
                    orientation: l.orientation,
                    angular_velocity: l.angular_velocity,
                },
                Some(Inertia {
                    center: l.inertia_center,
                    orientation: l.inertia_orientation,
                    tensor: l.inertia_tensor,
                }),
            )?;
            be.set_self_collision_group(body, Some(spec.model_id))?;
            links.push((l.id, body));
        }
        let root = links[root].1;
        let mut joints = Vec::with_capacity(spec.joints.len());
        for j in &spec.joints {
            let body = |key| {
                links
                    .binary_search_by_key(&key, |&(p, _)| p)
                    .map(|i| links[i].1)
                    .map_err(|_| NativeError::Invalid("missing robot link"))
            };
            let (p, c) = (Some(body(j.parent_id)?), body(j.child_id)?);
            let id = match spec
                .limits
                .binary_search_by_key(&j.joint_id, |l| l.joint_id)
            {
                Ok(i) => {
                    let l = spec.limits[i];
                    be.add_limited_revolute(p, c, j.anchor, j.axis, [l.lower, l.upper])?
                }
                Err(_) => be.add_reduced_revolute(p, c, j.anchor, j.axis)?,
            };
            joints.push((j.joint_id, id));
        }
        if ground.is_none() {
            ground = Some(be.spawn_static_plane(spec.ground.plane())?);
        }
        let ground = ground.expect("before or after all robot links");
        joints.sort_by_key(|(id, _)| *id);
        be.switch_articulation(root, ArticulationTier::Dynamic, DT)?;
        if let Some(snap) = snapshot {
            if let Some(image) = &snap.image {
                be.restore_grounded_robot_tree(root, ground, image)?;
            } else {
                for (j, (_, id)) in snap.state.joints.iter().zip(&joints) {
                    if let Some(e) = j.pending_effort {
                        be.add_joint_effort(*id, e)?;
                    }
                }
            }
        }
        let mut sim = if tick == 0 {
            Simulation::new(DT)?
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
            )?
        };
        gz_physics::install(&mut sim, be)?;
        Ok(Self {
            spec: spec.clone(),
            sim,
            links,
            joints,
            root,
            ground,
        })
    }
    pub(super) fn install_native_image(&mut self, image: &[u8]) -> std::result::Result<(), NativeError> {
        self.sim.world_mut().resource_mut::<PhysicsBackend>()
            .downcast_mut::<MultiFidelityBackend>()
            .expect("owned robot backend")
            .restore_grounded_robot_tree(self.root, self.ground, image)?;
        Ok(())
    }

    pub(super) fn capture(&self) -> std::result::Result<Snapshot, NativeError> {
        let tick = self.sim.committed_update_info()?.iteration;
        let be = self.sim.world().resource::<PhysicsBackend>();
        let native = be
            .downcast_ref::<MultiFidelityBackend>()
            .expect("owned robot backend");
        let mut links = Vec::with_capacity(self.links.len());
        let mut reactions = Vec::with_capacity(self.links.len());
        for &(public, id) in &self.links {
            let b = be
                .state(id)
                .ok_or(NativeError::Invalid("missing robot body"))?;
            links.push(ArticulationLinkState {
                id: public,
                position: b.position,
                orientation: b.orientation,
                velocity: b.velocity,
                angular_velocity: b.angular_velocity,
            });
            reactions.push(native.ground_reaction(id, self.ground)?);
        }
        let mut joints = Vec::with_capacity(self.joints.len());
        for &(public, id) in &self.joints {
            let j = be
                .joint_state(id)
                .ok_or(NativeError::Invalid("missing robot joint"))?;
            joints.push(FloatingTreeJointState {
                joint_id: public,
                q: j.position,
                qdot: j.velocity,
                pending_effort: native.pending_joint_effort(id),
            });
        }
        let mut patches = Vec::new();
        for p in native.fixed_ground_patches(self.ground)? {
            let link_id = self
                .links
                .iter()
                .find(|(_, body)| *body == p.body)
                .ok_or(NativeError::Invalid("foreign robot support"))?
                .0;
            patches.push(RobotGroundPatch {
                link_id,
                normal_on_body: p.normal_on_body,
                points: p.points,
            });
        }
        patches.sort_by_key(|p| p.link_id);
        let image = if tick == 0 {
            None
        } else {
            Some(native.checkpoint_grounded_robot_tree(self.root, self.ground)?)
        };
        Ok(Snapshot {
            state: FloatingTreeObservation {
                tick,
                links,
                joints,
            },
            patches,
            reactions,
            image,
        })
    }
    fn joint(&self, id: u64) -> Result<JointId> {
        let i = self
            .joints
            .binary_search_by_key(&id, |(p, _)| *p)
            .map_err(|_| invalid("unknown robot joint"))?;
        Ok(self.joints[i].1)
    }
}

impl PagedWorld for RobotWorld {
    type Spec = GroundedRobotSpec;
    type Version = ();
    const LIMIT: usize = SNAP_LIMIT;
    fn restore_bytes(s: &Self::Spec, (): (), tick: u64, bytes: &[u8]) -> Result<Self> {
        let snap: Snapshot = codec(SNAP_LIMIT)
            .deserialize(bytes)
            .map_err(io::Error::other)?;
        snap.check(s)?;
        if snap.state.tick != tick {
            return Err(invalid("robot restore clock differs"));
        }
        let world = Self::new(s, tick, Some(&snap))?;
        if world.capture_bytes(tick)? != bytes {
            return Err(invalid(
                "robot native restore disagrees with the committed graph and observations",
            ));
        }
        Ok(world)
    }
    fn capture_bytes(&self, tick: u64) -> Result<Vec<u8>> {
        let snap = self.capture()?;
        if snap.state.tick != tick {
            return Err(invalid("robot native clock differs from worker"));
        }
        Ok(codec(SNAP_LIMIT)
            .serialize(&snap)
            .map_err(io::Error::other)?)
    }
    fn queue(&mut self, id: u64, effort: f64) -> Result<()> {
        if !effort.is_finite() {
            return Err(invalid("nonfinite robot joint effort"));
        }
        let j = self.joint(id)?;
        let mut be = self.sim.world_mut().resource_mut::<PhysicsBackend>();
        if !(be
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .pending_joint_effort(j)
            .unwrap_or(0.)
            + effort)
            .is_finite()
        {
            return Err(invalid("robot effort overflow"));
        }
        be.add_joint_effort(j, effort).map_err(NativeError::from)?;
        Ok(())
    }
    fn posture_batch(&mut self, count: u8, bytes: &[u8]) -> Result<Vec<u8>> {
        use gz_physics::posture::{MassCenterSum, PostureMode};
        let p: RobotPostureProgram = codec(posture::PROGRAM_LIMIT)
            .deserialize(bytes)
            .map_err(io::Error::other)?;
        if p.intervals.len() != usize::from(count) {
            return Err(invalid("robot posture interval count differs"));
        }
        p.check(&self.spec)?;
        let segments = p
            .law
            .link_order
            .iter()
            .map(|public| {
                let i = self
                    .links
                    .binary_search_by_key(public, |(id, _)| *id)
                    .map_err(|_| invalid("missing robot posture segment"))?;
                Ok((
                    self.links[i].1,
                    self.spec.links[i].mass,
                    self.spec.links[i].inertia_center,
                ))
            })
            .collect::<Result<Vec<_>>>()?;
        let mass = segments.iter().map(|(_, m, _)| *m).sum();
        let axes = p
            .law
            .axes
            .iter()
            .map(|a| self.joint(a.joint_id))
            .collect::<Result<Vec<_>>>()?;
        let start = self
            .sim
            .committed_update_info()
            .map_err(NativeError::from)?
            .iteration;
        #[cfg(feature = "barrier-trace")]
        let interlock =
            std::env::var_os("GZ_ROBOT_SUBSTEP_TRACE_MARKER").map(std::path::PathBuf::from);
        let mut intervals = Vec::with_capacity(p.intervals.len());
        let mut efforts = Vec::with_capacity(axes.len());
        for (i, interval) in p.intervals.iter().enumerate() {
            let be = self.sim.world().resource::<PhysicsBackend>();
            let center_of_mass = if interval.targets.mode == PostureMode::Balance {
                let mut sum = MassCenterSum::default();
                for &(id, mass, center) in &segments {
                    sum.add(
                        mass,
                        center,
                        be.state(id)
                            .ok_or(invalid("missing native robot feedback body"))?,
                    );
                }
                Some(sum.finish(mass))
            } else {
                None
            };
            let correction = center_of_mass.map_or([0.; 2], |(p0, v)| {
                p.law
                    .balance
                    .correction(p0, v, interval.targets.balance_offset)
            });
            let mut torques = Vec::new();
            efforts.clear();
            if interval.targets.mode != PostureMode::Passive {
                torques.reserve(axes.len());
                for (a, &joint) in p.law.axes.iter().zip(&axes) {
                    let q = be
                        .joint_state(joint)
                        .ok_or(invalid("missing native robot feedback joint"))?;
                    let target = interval
                        .targets
                        .axes
                        .iter()
                        .find(|t| t.joint_id == a.joint_id);
                    let [target_rad, effort_nm] = a.command(q, correction, target);
                    be.validate_joint_effort(joint, effort_nm)
                        .map_err(NativeError::from)?;
                    efforts.push((a.joint_id, effort_nm));
                    torques.push(RobotPostureTorque {
                        joint_id: a.joint_id,
                        target_rad,
                        effort_nm,
                    });
                }
            }
            self.advance(&efforts, &interval.forces)?;
            // Developer trace builds can stop at a deterministic native boundary for process-loss qualification.
            #[cfg(feature = "barrier-trace")]
            if i == 6
                && let Some(path) = &interlock
            {
                match std::fs::File::create_new(path) {
                    Ok(mut file) => {
                        use std::io::Write;
                        writeln!(file, "{} {}", std::process::id(), start + i as u64 + 1)?;
                        file.sync_all()?;
                        std::thread::sleep(Duration::from_secs(10));
                    }
                    Err(e) if e.kind() == io::ErrorKind::AlreadyExists => {}
                    Err(e) => return Err(e.into()),
                }
            }
            intervals.push(RobotPostureFeedback {
                tick: start + i as u64,
                center_of_mass,
                torques,
            });
        }
        let receipt = RobotPostureReceipt {
            input_digest: Sha256::digest(bytes).into(),
            start,
            end: self
                .sim
                .committed_update_info()
                .map_err(NativeError::from)?
                .iteration,
            intervals,
        };
        receipt.check(&p, receipt.input_digest, start)?;
        Ok(codec(posture::PROGRAM_LIMIT)
            .serialize(&receipt)
            .map_err(io::Error::other)?)
    }
    fn motor(&mut self, _id: u64, _setting: Option<MotorSetting>) -> Result<()> {
        Err(invalid("robot takes interval joint torques"))
    }
    fn advance(&mut self, efforts: &[(u64, f64)], forces: &[(u64, [f64; 3])]) -> Result<()> {
        if efforts.len() > self.joints.len() || forces.len() > self.links.len() {
            return Err(invalid("robot input exceeds cardinality"));
        }
        let mut seen = BTreeSet::new();
        let be = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        for &(id, e) in efforts {
            let id_native = self.joint(id)?;
            if !seen.insert(id)
                || !(be.pending_joint_effort(id_native).unwrap_or(0.) + e).is_finite()
            {
                return Err(invalid("duplicate or invalid robot joint effort"));
            }
        }
        let mut resolved = Vec::with_capacity(forces.len());
        for &(id, force) in forces {
            let i = self
                .links
                .binary_search_by_key(&id, |(id, _)| *id)
                .map_err(|_| invalid("unknown robot force receiver"))?;
            if !seen.insert(id) || !force.iter().all(|x| x.is_finite()) {
                return Err(invalid("duplicate or nonfinite robot force"));
            }
            resolved.push((self.links[i].1, force));
        }
        for &(id, e) in efforts {
            self.queue(id, e)?;
        }
        {
            let mut be = self.sim.world_mut().resource_mut::<PhysicsBackend>();
            for (id, f) in resolved {
                if f != [0.; 3] {
                    be.add_world_wrench(id, f, [0.; 3])
                        .map_err(NativeError::from)?;
                }
            }
        }
        self.sim.step_exact(1).map_err(NativeError::from)?;
        Ok(())
    }
}
