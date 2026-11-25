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
struct ObservedLink {
    id: u64,
    native_body_id: u64,
    translation_m: [f64; 3],
    rotation_xyzw: [f64; 4],
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

/// Impulses delivered to the lower-numbered native body, in world axes.
#[derive(Serialize)]
struct PairImpulse {
    a: u64,
    b: u64,
    #[serde(skip)]
    normal_sum_ns: f64,
    normal_on_a_ns: [f64; 3],
    tangent_on_a_ns: Option<[f64; 3]>,
}

impl PairImpulse {
    fn add_point(&mut self, normal: [f64; 3], magnitude: f64, tangent: Option<[f64; 3]>) {
        self.normal_sum_ns += magnitude;
        for (sum, axis) in self.normal_on_a_ns.iter_mut().zip(normal) {
            *sum += magnitude * axis;
        }
        // An unloaded Coulomb point contributes zero even when the grouped
        // rigid solver does not publish individual tangential impulses.
        if magnitude != 0. || tangent.is_some() {
            self.tangent_on_a_ns = self
                .tangent_on_a_ns
                .zip(tangent)
                .map(|(sum, t)| std::array::from_fn(|i| sum[i] + t[i]));
        }
    }
}

#[derive(Serialize)]
struct RefinedContact {
    foot_link: u64,
    support_link: u64,
    normal_impulse_ns: f64,
    normal_on_foot_ns: [f64; 3],
    tangent_on_foot_ns: Option<[f64; 3]>,
    first_positive_end_ns: Option<u128>,
    last_native_normal_ns: f64,
    active_at_end: bool,
}

/// A peak is the mean over one native solve. Its time is the end of that
/// solve, measured from the start of the current public tick.
#[derive(Serialize)]
struct PadImpact {
    foot: u64,
    support: u64,
    peak_normal_force_n: f64,
    peak_end_ns: u128,
    angular_impulse_on_foot_nms: Option<[f64; 3]>,
}

#[derive(Serialize)]
struct RefinedStep {
    native_steps: u32,
    speed_stop_floor: u32,
    solver_iterations: usize,
    pgs_iterations: Option<usize>,
    relaxation_iterations: Option<usize>,
    requested_pairs: Vec<RefinedContact>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pad_impacts: Vec<PadImpact>,
}

impl From<&gz_sim_scene::primary_contact::Refinement> for RefinedStep {
    fn from(r: &gz_sim_scene::primary_contact::Refinement) -> Self {
        Self {
            native_steps: r.interval.native_steps,
            speed_stop_floor: r.speed_stop_floor,
            pad_impacts: r
                .pad_impacts
                .iter()
                .map(|p| PadImpact {
                    foot: p.foot.raw(),
                    support: p.support.raw(),
                    peak_normal_force_n: p.peak_normal_force_n,
                    peak_end_ns: p.peak_end.as_nanos(),
                    angular_impulse_on_foot_nms: p.angular_impulse_on_foot,
                })
                .collect(),
            solver_iterations: r.solver.temporal,
            pgs_iterations: r.solver.pgs,
            relaxation_iterations: r.solver.relaxation,
            requested_pairs: r
                .requested_pairs
                .iter()
                .map(|p| RefinedContact {
                    foot_link: p.foot_link,
                    support_link: p.support_link,
                    normal_impulse_ns: p.normal_impulse,
                    normal_on_foot_ns: p.normal_impulse_on_foot,
                    tangent_on_foot_ns: p.tangent_impulse_on_foot,
                    first_positive_end_ns: p.first_positive_end.map(|t| t.as_nanos()),
                    last_native_normal_ns: p.last_native_normal_impulse,
                    active_at_end: p.active_at_end,
                })
                .collect(),
        }
    }
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
    observed_links: Vec<ObservedLink>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    universal_joints: Vec<Encoder<[f64; 2]>>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    held_joint_forces: Vec<(u64, f64)>,
    /// Aggregate physical checks for qualification and fault isolation.
    pub(crate) held_targets: usize,
    pub(crate) pending_wrenches: usize,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    link_loads: Vec<LinkLoad>,
    imu_samples: usize,
    /// Native body IDs in ascending pair order, with summed normal impulse in N·s.
    pair_reactions_ns: Vec<(u64, u64, f64)>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pair_impulses: Vec<PairImpulse>,
    #[serde(skip_serializing_if = "Option::is_none")]
    refinement: Option<RefinedStep>,
    #[serde(skip_serializing_if = "Option::is_none")]
    contact_refinement_enabled: Option<bool>,
    contact_points: usize,
    delivered_normal_ns: f64,
    stored_normal_ns: f64,
}

pub(super) fn state(
    sim: &Simulation,
    command: usize,
    observed: &std::collections::BTreeSet<u64>,
) -> Result<State> {
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
    let mut observed_links = Vec::new();
    let mut universal_joints = Vec::new();
    let mut held_joint_forces = Vec::new();
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
            if observed.contains(&id) {
                let p = e
                    .get::<WorldPose>()
                    .ok_or("observed link has no committed world pose")?;
                observed_links.push(ObservedLink {
                    id,
                    native_body_id: body.raw(),
                    translation_m: p.translation,
                    rotation_xyzw: p.rotation,
                });
            }
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
        if let Some(force) = e.get::<gz_sim_scene::JointForceCommand>() {
            held_joint_forces.push((id, force.torque()));
            inputs.item((id, "held_joint_force", bits(force.torque())));
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
        let (a, b) = (patch.a.raw(), patch.b.raw());
        let reaction = reactions.entry((a, b)).or_insert(PairImpulse {
            a,
            b,
            normal_sum_ns: 0.,
            normal_on_a_ns: [0.; 3],
            tangent_on_a_ns: Some([0.; 3]),
        });
        contacts.item((
            patch.a.raw(),
            patch.b.raw(),
            patch.normal_on_a.map(f64::to_bits),
            patch.points.len(),
        ));
        for point in patch.points {
            reaction.add_point(
                patch.normal_on_a,
                point.normal_impulse,
                point.tangent_impulse_on_a,
            );
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
    // Manifold points and warm starts above describe the final native solve. An
    // interval can have delivered its entire impulse and separated before then.
    let refinement = gz_sim_scene::primary_contact::last_refinement(sim)
        .filter(|r| r.iteration == clock.iteration && r.sim_time == clock.sim_time);
    if let Some(r) = refinement {
        reactions.clear();
        delivered_normal_ns = 0.;
        for pair in &r.interval.pairs {
            let (a, b) = (pair.a.raw(), pair.b.raw());
            reactions.insert(
                (a, b),
                PairImpulse {
                    a,
                    b,
                    normal_sum_ns: pair.normal_impulse,
                    normal_on_a_ns: pair.normal_impulse_on_a,
                    tangent_on_a_ns: pair.tangent_impulse_on_a,
                },
            );
            delivered_normal_ns += pair.normal_impulse;
        }
    }
    let contact_refinement_enabled = gz_sim_scene::primary_contact::is_enabled(sim);
    if let Some(enabled) = contact_refinement_enabled {
        inputs.item(("primary_contact_refinement", enabled));
    }
    Ok(State {
        contact_refinement_enabled,
        refinement: refinement.map(RefinedStep::from),
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
        observed_links,
        universal_joints,
        held_targets,
        held_joint_forces,
        pending_wrenches: link_loads.len(),
        link_loads,
        imu_samples,
        pair_reactions_ns: reactions
            .values()
            .map(|p| (p.a, p.b, p.normal_sum_ns))
            .collect(),
        pair_impulses: reactions.into_values().collect(),
        contact_points,
        delivered_normal_ns,
        stored_normal_ns,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn opposing_faces_add_pressure_and_cancel_force_without_inventing_friction() {
        for unknown_first in [false, true] {
            let mut load = PairImpulse {
                a: 1,
                b: 2,
                normal_sum_ns: 0.,
                normal_on_a_ns: [0.; 3],
                tangent_on_a_ns: Some([0.; 3]),
            };
            let face = [1., 0., 0.];
            let applied = [0., -0.2, 0.];
            load.add_point(face, 0., None);
            load.add_point(face, 1., Some(applied));
            assert_eq!(load.tangent_on_a_ns, Some(applied));
            for unknown in [unknown_first, !unknown_first] {
                load.add_point([-1., 0., 0.], 0.5, (!unknown).then_some(applied));
            }
            assert_eq!(load.normal_sum_ns, 2.);
            assert_eq!(load.normal_on_a_ns, [0.; 3]);
            assert_eq!(load.tangent_on_a_ns, None);
        }
    }
}
