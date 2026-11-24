// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_gpu::{CarryGripperBatch, CarryGripperLane, FreeBoxState};
use rapier3d_f64::prelude::*;
use serde::Serialize;
use serde_json::{Value, json};
pub const MOMENTS: DVec3 = DVec3::new(0.00001450666666666667, 0.00002602666666666667, 0.0000192);
const OFFSET: DVec3 = DVec3::new(0.005, 0., -0.002);
pub const MASS: f64 = 0.08;
pub const WEIGHT: f64 = 9.81 * MASS;

#[derive(Clone, Copy)]
pub struct Settings {
    pub steps: usize,
    pub stabilization: usize,
    pub reversed_shelf_frame: bool,
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            steps: 32,
            stabilization: 1,
            reversed_shelf_frame: false,
        }
    }
}
pub struct Reference {
    steps: usize,
    world: PhysicsWorld,
    body: RigidBodyHandle,
    shape: ColliderHandle,
    shelf: ColliderHandle,
}
impl Reference {
    fn new(body: FreeBoxState, l: &CarryGripperLane, settings: Settings) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = -9.81 * Vector::Z;
        let p = &mut world.integration_parameters;
        p.dt = 0.001 / settings.steps as f64;
        p.num_internal_stabilization_iterations = settings.stabilization;
        p.normalized_allowed_linear_error = 0.;
        p.normalized_prediction_distance = 0.;
        p.normalized_contact_recycle_distance = 0.;
        p.num_solver_iterations = 32;
        p.num_internal_pgs_iterations = 8;
        p.friction_model = FrictionModel::Coulomb;
        // Match the hard unilateral normal rows on the device.
        p.static_contact_softness.natural_frequency = 1e6;
        p.static_contact_softness.damping_ratio = 1.;
        p.normalized_max_corrective_velocity = 0.;
        let material = |c: ColliderBuilder, mu| {
            c.friction(mu)
                .friction_combine_rule(CoefficientCombineRule::Min)
                .restitution(0.)
        };
        // Rotating the exact rectangular collider half a turn preserves the occupied
        // volume and the material, but changes its native feature ordering.
        let mut slab = material(
            ColliderBuilder::cuboid(l.shelf_half[0], 0.105, l.shelf_half[1])
                .translation(Vector::new(l.shelf_center[0], 0., l.shelf_center[1])),
            l.shelf_friction,
        )
        .build();
        if settings.reversed_shelf_frame {
            slab.set_position(Pose::from_parts(
                slab.translation(),
                Rotation::from_xyzw(0., 0., 1., 0.),
            ));
        }
        let shelf = world.insert_collider(slab, None);
        world.insert_collider(
            material(
                ColliderBuilder::cuboid(1., 1., 0.02)
                    .translation(Vector::Z * (l.hand.ground - 0.02)),
                l.hand.friction[2],
            ),
            None,
        );
        let q = DQuat::from_array(body.orientation);
        let pos = DVec3::from(body.mass_center) - q * OFFSET;
        let omega = q * ((q.inverse() * DVec3::from(body.momentum)) / MOMENTS);
        let id = world.insert_body(
            RigidBodyBuilder::dynamic()
                .translation(Vector::from_array(pos.to_array()))
                .rotation(Vector::from_array(q.to_scaled_axis().to_array()))
                .can_sleep(false),
        );
        let shape = world.insert_collider(
            material(
                ColliderBuilder::cuboid(0.024, 0.012, 0.020).mass_properties(MassProperties::new(
                    Vector::from_array(OFFSET.to_array()),
                    MASS,
                    Vector::from_array(MOMENTS.to_array()),
                )),
                1.,
            ),
            Some(id),
        );
        world.bodies[id].set_linvel(Vector::from_array(body.center_velocity), true);
        world.bodies[id].set_angvel(Vector::from_array(omega.to_array()), true);
        Self {
            steps: settings.steps,
            world,
            body: id,
            shape,
            shelf,
        }
    }
    fn advance(&mut self) -> f64 {
        let mut j = 0.;
        for _ in 0..self.steps {
            self.world.step();
            if let Some(pair) = self.world.narrow_phase.contact_pair(self.shelf, self.shape) {
                let sign = if pair.collider2 == self.shape {
                    1.
                } else {
                    -1.
                };
                for m in pair.solver_manifolds() {
                    // Narrow phase retains historical witnesses when a face replaces
                    // an edge. Read only the points admitted to this contact solve.
                    let normal: f64 = m
                        .data
                        .solver_contacts
                        .iter()
                        .map(|row| m.points[row.contact_indices()[0] as usize].data.impulse)
                        .sum();
                    j += sign * m.data.normal.z * normal;
                }
            }
        }
        j
    }
    fn state(&self, j: f64) -> Sample {
        let b = &self.world.bodies[self.body];
        let origin = b.translation();
        Sample {
            position: origin.to_array(),
            orientation: b.rotation().to_array(),
            center_velocity: b.linvel().to_array(),
            omega: b.angvel().to_array(),
            impulse: j,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Sample {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub center_velocity: [f64; 3],
    pub omega: [f64; 3],
    pub impulse: f64,
}
impl Sample {
    pub fn gpu(b: &CarryGripperBatch) -> Self {
        let l = b.free_box_link().unwrap();
        Self {
            position: l.position,
            orientation: l.orientation,
            center_velocity: b.free_box().unwrap().center_velocity,
            omega: l.angular,
            impulse: b.lanes()[0].shelf_up,
        }
    }
}
pub fn compare(a: &[(usize, Sample)], b: &[(usize, Sample)]) -> [f64; 4] {
    let mut delta = [0_f64; 4];
    for ((_, g), (_, c)) in a.iter().zip(b) {
        delta[2] = DVec3::from(g.position).distance(c.position.into());
        delta[3] = DQuat::from_array(g.orientation).angle_between(DQuat::from_array(c.orientation));
        delta[0] = delta[0].max(delta[2]);
        delta[1] = delta[1].max(delta[3]);
    }
    delta
}
pub fn cpu_tick(
    reference: &mut Option<Reference>,
    state: FreeBoxState,
    lane: &CarryGripperLane,
    settings: Settings,
) -> Sample {
    let impulse = match reference {
        Some(cpu) => cpu.advance(),
        None => {
            *reference = Some(Reference::new(state, lane, settings));
            0.
        }
    };
    reference.as_ref().unwrap().state(impulse)
}
pub fn summarize(samples: &[(usize, Sample)]) -> Value {
    let first = samples
        .iter()
        .find(|(_, s)| s.impulse > 1e-10)
        .map(|(t, _)| t);
    let last = samples.last().unwrap().1;
    let window = &samples[samples.len() - 200..];
    let max = |f: fn(Sample) -> f64| window.iter().map(|(_, s)| f(*s)).fold(0., f64::max);
    let lo = window.first().unwrap().1;
    let mut episodes = Vec::new();
    let mut last_loaded = false;
    for (tick, s) in samples {
        let loaded = s.impulse > 1e-10;
        if loaded != last_loaded {
            episodes.push((*tick, loaded));
        }
        last_loaded = loaded;
    }
    let first_impulse = samples
        .iter()
        .find(|(_, s)| s.impulse > 1e-10)
        .map(|(_, s)| s.impulse);
    let settled = samples
        .windows(40)
        .find(|window| {
            window.iter().all(|(_, s)| {
                (1000. * s.impulse - WEIGHT).abs() < 0.005
                    && DVec3::from(s.center_velocity).length() < 0.001
                    && DVec3::from(s.omega).length() < 0.05
            })
        })
        .map(|w| w[0].0);
    json!({"first_contact_ms":first,"first_up_ns":first_impulse,"begin_40ms_supported_rest_ms":settled,"load_transitions":episodes, "last":last,"last_200_mean_up_n":window.iter().map(|(_,s)|s.impulse).sum::<f64>()/0.2,
        "last_200_max_load_error_n":max(|s|(1000.*s.impulse-WEIGHT).abs()),
        "last_200_max_com_speed_m_s":max(|s|DVec3::from(s.center_velocity).length()),"last_200_max_spin_rad_s":max(|s|DVec3::from(s.omega).length()),
        "last_200_link_drift_m":DVec3::from(last.position).distance(lo.position.into()),"last_200_angle_drift_rad":DQuat::from_array(last.orientation).angle_between(DQuat::from_array(lo.orientation))})
}

pub fn rate(samples: &mut [f64]) -> Value {
    samples.sort_by(f64::total_cmp);
    let p95 = (samples.len() * 95).div_ceil(100) - 1;
    json!({"steps_s":samples.len() as f64 / samples.iter().sum::<f64>(), "p95_ms":1000. * samples[p95]})
}
