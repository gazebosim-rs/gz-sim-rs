// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::DQuat;
use gz_sim_scene::deformable::{DeformableSample, bodies, material_energy};
use serde::Serialize;
use std::time::Duration;

#[derive(Debug, Default, Serialize)]
pub(super) struct EnergyAudit {
    pub initial_material_j: f64,
    pub final_material_change_j: f64,
    pub max_gravity_gain_j: f64,
    pub max_kinetic_j: f64,
    pub max_elastic_j: f64,
    pub positive_pad_work_j: f64,
    pub negative_pad_work_j: f64,
    pub self_friction_loss_j: f64,
    pub max_material_gain_above_pad_work_j: f64,
    pub max_angular_impulse_residual_nms: f64,
    #[serde(skip)]
    history: Option<History>,
}

#[derive(Debug)]
struct History {
    material: String,
    jaw_ids: [u64; 2],
    jaws: [BodyState; 2],
    bench: DVec3,
    time: Duration,
    potential: f64,
    angular_momentum: DVec3,
    gravity_torque: DVec3,
}

// Gravity and the support plane are the authored, stationary laboratory frame.
fn momenta(sample: &DeformableSample) -> (DVec3, DVec3) {
    let mut angular = DVec3::ZERO;
    let mut gravity_torque = DVec3::ZERO;
    for particle in sample.particles().iter().filter(|p| p.inverse_mass > 0.) {
        let arm = DVec3::from(particle.position) / particle.inverse_mass;
        angular += arm.cross(particle.velocity.into());
        gravity_torque += arm.cross(-9.81 * DVec3::Z);
    }
    (angular, gravity_torque)
}

// The reported moment is about the beginning-of-tick jaw COM. Move it to the
// midpoint before pairing it with the shortest world rotation across the tick.
fn pad_work(before: BodyState, after: BodyState, linear: DVec3, moment: DVec3, dt: f64) -> f64 {
    let dx = DVec3::from(after.position) - DVec3::from(before.position);
    let q =
        DQuat::from_array(after.orientation) * DQuat::from_array(before.orientation).conjugate();
    let rotation = if q.w < 0. { -q } else { q }.to_scaled_axis();
    -(linear.dot(dx) + (moment - 0.5 * dx.cross(linear)).dot(rotation)) / dt
}

impl EnergyAudit {
    pub(super) fn new(
        sim: &Simulation,
        material: &str,
        jaw_ids: [u64; 2],
        bench_z: f64,
    ) -> Result<Self> {
        let energy =
            material_energy(sim, material).ok_or("missing sheet for energy measurement")??;
        let sample = bodies(sim)
            .find(|(n, _)| *n == material)
            .ok_or("missing published sheet")?
            .1;
        let (angular_momentum, gravity_torque) = momenta(sample);
        Ok(Self {
            initial_material_j: energy.total(),
            history: Some(History {
                material: material.into(),
                jaw_ids,
                jaws: [state(sim, jaw_ids[0])?, state(sim, jaw_ids[1])?],
                bench: DVec3::Z * bench_z,
                time: sample.sim_time(),
                potential: energy.gravitational,
                angular_momentum,
                gravity_torque,
            }),
            ..Default::default()
        })
    }

    pub(super) fn observe(&mut self, sim: &Simulation, sample: &DeformableSample) -> Result<()> {
        let history = self
            .history
            .as_mut()
            .ok_or("energy measurement not initialized")?;
        let dt = sim.step_duration();
        if history.time.checked_add(dt) != Some(sample.sim_time()) {
            return Err("energy measurement requires consecutive physics steps".into());
        }
        let energy =
            material_energy(sim, &history.material).ok_or("missing committed sheet energy")??;
        let reactions = sample.last_boundary_impulses();
        let [left, right, bench] = reactions else {
            return Err("expected two jaws and a bench for the work balance".into());
        };
        let mut external = DVec3::from(bench.angular) + history.bench.cross(bench.linear.into());
        let mut jaws = history.jaws;
        for (i, load) in [left, right].into_iter().enumerate() {
            let linear = DVec3::from(load.linear);
            let moment = DVec3::from(load.angular);
            jaws[i] = state(sim, history.jaw_ids[i])?;
            let work = pad_work(history.jaws[i], jaws[i], linear, moment, dt.as_secs_f64());
            self.positive_pad_work_j += work.max(0.);
            self.negative_pad_work_j += work.min(0.);
            external += moment + DVec3::from(history.jaws[i].position).cross(linear);
        }
        let (angular, gravity_torque) = momenta(sample);
        let gravity = 0.5 * dt.as_secs_f64() * (gravity_torque + history.gravity_torque);
        let residual = angular - history.angular_momentum + external - gravity;
        self.max_angular_impulse_residual_nms =
            self.max_angular_impulse_residual_nms.max(residual.length());
        self.final_material_change_j = energy.total() - self.initial_material_j;
        self.max_kinetic_j = self.max_kinetic_j.max(energy.kinetic);
        self.max_elastic_j = self.max_elastic_j.max(energy.distance + energy.bending);
        self.max_gravity_gain_j = self
            .max_gravity_gain_j
            .max(energy.gravitational - history.potential);
        self.self_friction_loss_j += sample.last_self_contact_impulse().friction_work;
        let net_work = self.positive_pad_work_j + self.negative_pad_work_j;
        self.max_material_gain_above_pad_work_j = self
            .max_material_gain_above_pad_work_j
            .max(self.final_material_change_j - net_work);
        history.jaws = jaws;
        history.time = sample.sim_time();
        history.angular_momentum = angular;
        history.gravity_torque = gravity_torque;
        Ok(())
    }
}

#[cfg(test)]
#[path = "sheet_energy_tests.rs"]
mod tests;
