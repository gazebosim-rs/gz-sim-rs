// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Result, Simulation};
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::EntityId;
use gz_sim_scene::{
    BallJointSample, BodyHandle, ImuSample, JointHandle, JointSample, Name, UniversalJointSample,
    WorldPose,
};
use serde::Serialize;
use sha2::{Digest, Sha256};

#[derive(Default)]
struct Signature(Sha256);

impl Signature {
    fn item(&mut self, value: impl Serialize) {
        // Domains are separate; length-framing preserves tuple and record boundaries.
        let bytes = serde_json::to_vec(&value)
            .expect("the signature consists only of integer tuples, arrays and names");
        self.0.update((bytes.len() as u64).to_le_bytes());
        self.0.update(bytes);
    }
    fn finish(self) -> String {
        format!("{:x}", self.0.finalize())
    }
}

#[derive(Serialize)]
struct Wrist {
    id: u64,
    orientation_xyzw: [f64; 4],
    angular_velocity_rad_s: [f64; 3],
    pending_torque_nm: Option<[f64; 3]>,
}

/// Coordinates and accumulated one-step efforts in the joint's authored axes.
#[derive(Serialize)]
struct Encoder<T> {
    id: u64,
    position: T,
    velocity: T,
    pending_effort: Option<T>,
}

#[derive(Serialize)]
struct LinkLoad {
    id: u64,
    world_force_n: [f64; 3],
    world_torque_nm: [f64; 3],
}

#[derive(Serialize)]
pub(super) struct State {
    command: usize,
    epoch: u64,
    step: u64,
    time_ns: u128,
    entities: usize,
    scene: String,
    physics: String,
    contacts: String,
    inputs: String,
    imu: String,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    wrists: Vec<Wrist>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    model_tiers: Vec<(u64, &'static str)>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    scalar_joints: Vec<Encoder<f64>>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    universal_joints: Vec<Encoder<[f64; 2]>>,
    /// Aggregate physical checks for qualification and fault isolation.
    pub(crate) held_targets: usize,
    pub(crate) pending_wrenches: usize,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    link_loads: Vec<LinkLoad>,
    imu_samples: usize,
    /// Native body IDs in ascending pair order, with summed normal impulse in N·s.
    pair_reactions_ns: Vec<(u64, u64, f64)>,
    contact_points: usize,
    delivered_normal_ns: f64,
    stored_normal_ns: f64,
}

pub(super) fn state(sim: &Simulation, command: usize) -> Result<State> {
    let world = sim.world();
    let backend = world.resource::<PhysicsBackend>();
    let solver = backend
        .downcast_ref::<MultiFidelityBackend>()
        .expect("local CPU profile owns a native CPU solver");
    let mut entities: Vec<_> = world
        .iter_entities()
        .filter_map(|e| Some((e.get::<EntityId>()?.get(), e)))
        .collect();
    entities.sort_unstable_by_key(|(id, _)| *id);
    let mut scene = Signature::default();
    let mut physics = Signature::default();
    let mut inputs = Signature::default();
    let mut imu = Signature::default();
    let mut model_tiers = Vec::new();
    let mut wrists = Vec::new();
    let mut scalar_joints = Vec::new();
    let mut universal_joints = Vec::new();
    let mut held_targets = 0;
    let mut link_loads = Vec::new();
    let mut imu_samples = 0;
    for &(id, e) in &entities {
        let bits = f64::to_bits;
        let pose = e
            .get::<WorldPose>()
            .map(|p| (p.translation.map(bits), p.rotation.map(bits)));
        scene.item((id, e.get::<Name>().map(|n| &n.0), pose));
        if e.get::<gz_sim_scene::EntityKind>() == Some(&gz_sim_scene::EntityKind::Model) {
            use gz_sim_scene::{ArticulationTier, FidelityError, model_fidelity};
            match model_fidelity(sim, id) {
                Ok(tier) => {
                    let dynamic = tier == ArticulationTier::Dynamic;
                    physics.item((id, "dynamic", dynamic));
                    model_tiers.push((id, if dynamic { "dynamic" } else { "prescribed" }));
                }
                Err(FidelityError::NotSwitchable(_)) => {}
                Err(error) => return Err(error.into()),
            }
        }
        if let Some(handle) = e.get::<BodyHandle>() {
            let body = handle.id();
            let s = backend.state(body).ok_or("native body state is missing")?;
            physics.item((
                id,
                body.raw(),
                s.position.map(bits),
                s.orientation.map(bits),
                s.velocity.map(bits),
                s.angular_velocity.map(bits),
            ));
        }
        if let Some(j) = e.get::<JointSample>() {
            physics.item((id, j.state.position.to_bits(), j.state.velocity.to_bits()));
            let pending = e
                .get::<JointHandle>()
                .and_then(|h| solver.pending_joint_effort(h.id()));
            if let Some(value) = pending {
                inputs.item((id, "joint_effort", bits(value)));
            }
            scalar_joints.push(Encoder {
                id,
                position: j.state.position,
                velocity: j.state.velocity,
                pending_effort: pending,
            });
        }
        if let (Some(j), Some(handle)) = (e.get::<BallJointSample>(), e.get::<JointHandle>()) {
            let pending = solver.pending_ball_effort(handle.id());
            if let Some(torque) = pending {
                inputs.item((id, "ball_torque_nm", torque.map(bits)));
            }
            physics.item((
                id,
                j.state.orientation.map(bits),
                j.state.angular_velocity.map(bits),
            ));
            wrists.push(Wrist {
                id,
                orientation_xyzw: j.state.orientation,
                angular_velocity_rad_s: j.state.angular_velocity,
                pending_torque_nm: pending,
            });
        }
        if let Some(j) = e.get::<UniversalJointSample>() {
            physics.item((id, j.state.position.map(bits), j.state.velocity.map(bits)));
            let pending = e
                .get::<JointHandle>()
                .and_then(|h| solver.pending_universal_effort(h.id()));
            if let Some(value) = pending {
                inputs.item((id, "universal_torque_nm", value.map(bits)));
            }
            universal_joints.push(Encoder {
                id,
                position: j.state.position,
                velocity: j.state.velocity,
                pending_effort: pending,
            });
        }
        if let Some(target) = gz_sim_scene::held_joint_controller_target(sim, id) {
            held_targets += 1;
            inputs.item((id, bits(target)));
        }
        if e.contains::<BodyHandle>()
            && let Some((f, t)) = gz_sim_scene::pending_link_world_wrench(sim, id)
        {
            link_loads.push(LinkLoad {
                id,
                world_force_n: f,
                world_torque_nm: t,
            });
            inputs.item((id, f.map(bits), t.map(bits)));
        }
        if let Some(s) = e.get::<ImuSample>() {
            imu_samples += 1;
            let v = s.reading.as_ref().map_err(|error| error.to_string())?;
            imu.item((
                id,
                s.time.as_nanos(),
                v.orientation.map(bits),
                v.specific_force.map(bits),
                v.angular_velocity.map(bits),
                v.linear_acceleration_covariance.map(bits),
                v.angular_velocity_covariance.map(bits),
            ));
        }
    }
    let mut contacts = Signature::default();
    let mut delivered_normal_ns = 0.;
    let mut stored_normal_ns = 0.;
    let mut contact_points = 0;
    let mut reactions = std::collections::BTreeMap::new();
    for patch in solver.rigid_contact_patches() {
        let reaction = reactions
            .entry((patch.a.raw(), patch.b.raw()))
            .or_insert(0.);
        contacts.item((
            patch.a.raw(),
            patch.b.raw(),
            patch.normal_on_a.map(f64::to_bits),
            patch.points.len(),
        ));
        for point in patch.points {
            *reaction += point.normal_impulse;
            delivered_normal_ns += point.normal_impulse;
            contact_points += 1;
            contacts.item((
                point.lever_a.map(f64::to_bits),
                point.lever_b.map(f64::to_bits),
                point.normal_impulse.to_bits(),
                point.tangent_impulse_on_a.map(|t| t.map(f64::to_bits)),
            ));
        }
    }
    // Include dormant point history even when the public solver delivered no impulse.
    for point in solver.stored_contact_points() {
        let bits = f64::to_bits;
        stored_normal_ns += point.normal_ns;
        contacts.item((
            point.colliders,
            point.subshapes,
            point.features,
            point.local_anchors.map(|v| v.map(bits)),
            point.solver_arms.map(|v| v.map(bits)),
            point.normal.map(bits),
            bits(point.distance_m),
            bits(point.normal_ns),
            point.tangent_basis_ns.map(bits),
            point.tangent_world_ns.map(bits),
            bits(point.twist_nms),
        ));
    }
    let clock = sim.committed_update_info()?;
    Ok(State {
        command,
        epoch: clock.epoch,
        step: clock.iteration,
        time_ns: clock.sim_time.as_nanos(),
        entities: entities.len(),
        scene: scene.finish(),
        physics: physics.finish(),
        contacts: contacts.finish(),
        inputs: inputs.finish(),
        imu: imu.finish(),
        model_tiers,
        wrists,
        scalar_joints,
        universal_joints,
        held_targets,
        pending_wrenches: link_loads.len(),
        link_loads,
        imu_samples,
        pair_reactions_ns: reactions
            .into_iter()
            .map(|((a, b), ns)| (a, b, ns))
            .collect(),
        contact_points,
        delivered_normal_ns,
        stored_normal_ns,
    })
}
