// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent production CPU solvers for the batch's vertical-cell fixture.
use gz_physics_gpu::Lane;
use gz_physics_rapier::{BodyId, BodySpec, DynamicBackend, RapierBackend, Shape};
use gz_physics_xpbd::{
    Boundary, DistanceConstraint, Particle, SoftBody, SolverConfig, SphereMobility, step_coupled,
};
use std::{error::Error, time::Duration};

pub const DT: Duration = Duration::from_millis(1);
pub const GRAVITY: f64 = -9.81;
const BOB_RADIUS: f64 = 0.03;
const RIGID_RADIUS: f64 = 0.08;

pub fn fixture(n: usize) -> Vec<Lane> {
    // Each row of the 2048-cell batch permutes the range for that parameter.
    // Coprime strides avoid lockstep changes of mass, gap and stiffness.
    (0..n)
        .map(|i| {
            let choose = |stride, offset, lo: f64, hi: f64| {
                lo + (hi - lo) * (((i * stride + offset) % 2048) as f64 / 2047.0)
            };
            let anchor_z = choose(277, 41, 0.14, 0.22);
            let anchor_v = choose(499, 1205, 0.035, 0.14);
            let rest_length = choose(691, 193, 0.22, 0.37);
            let bob_z = anchor_z + rest_length;
            let combined_radius = RIGID_RADIUS + BOB_RADIUS;
            Lane {
                anchor_z,
                anchor_v,
                bob_z,
                bob_v: anchor_v,
                combined_radius,
                rest_length,
                rigid_z: bob_z
                    + combined_radius
                    + choose(839, 73, if i % 2 == 0 { 0.016 } else { 0.09 }, 0.18),
                rigid_v: choose(653, 115, -0.18, 0.0),
                compliance: if i % 2 == 0 {
                    choose(955, 145, 1.6e-6, 6.4e-6)
                } else {
                    choose(955, 145, 0.0006, 0.0018)
                },
                bob_inverse_mass: choose(593, 75, 0.7, 2.0),
                rigid_inverse_mass: choose(783, 367, 0.45, 1.6),
                last_receiver_impulse_z: 0.0,
            }
        })
        .collect()
}

pub struct Reference {
    rigid: RapierBackend,
    cells: Vec<(BodyId, SoftBody)>,
    pub lanes: Vec<Lane>,
}

impl Reference {
    pub fn new(initial: &[Lane]) -> Result<Self, Box<dyn Error>> {
        let mut rigid = RapierBackend::new([0.0, 0.0, GRAVITY])?;
        let mut cells = Vec::with_capacity(initial.len());
        for (i, lane) in initial.iter().enumerate() {
            // Physical separation eliminates all native rigid contacts between cells.
            let x = (i % 64) as f64;
            let y = (i / 64) as f64;
            let id = rigid.spawn(BodySpec {
                shape: Shape::Sphere {
                    radius: RIGID_RADIUS,
                },
                position: [x, y, lane.rigid_z],
                velocity: [0.0, 0.0, lane.rigid_v],
                mass: Some(lane.rigid_inverse_mass.recip()),
            })?;
            let particles = vec![
                Particle {
                    position: [x, y, lane.anchor_z],
                    velocity: [0., 0., lane.anchor_v],
                    inverse_mass: 0.,
                },
                Particle {
                    position: [x, y, lane.bob_z],
                    velocity: [0., 0., lane.bob_v],
                    inverse_mass: lane.bob_inverse_mass,
                },
            ];
            let constraints = vec![DistanceConstraint {
                a: 0,
                b: 1,
                rest_length: lane.rest_length,
                compliance: lane.compliance,
            }];
            let body = SoftBody::new(
                particles,
                constraints,
                SolverConfig {
                    gravity: [0., 0., GRAVITY],
                    iterations: 1,
                    substeps: 1,
                    particle_radius: BOB_RADIUS,
                    self_contact: false,
                    friction: 0.,
                },
            )?;
            cells.push((id, body));
        }
        Ok(Self {
            rigid,
            cells,
            lanes: initial.to_vec(),
        })
    }

    pub fn step(&mut self) -> Result<(), Box<dyn Error>> {
        self.rigid.step(DT)?;
        let h = DT.as_secs_f64();
        for ((id, body), lane) in self.cells.iter_mut().zip(&mut self.lanes) {
            let mut end = self.rigid.state(*id).ok_or("missing reference sphere")?;
            let mut start = end.position;
            start[2] = lane.rigid_z;
            // The finite receiver follows the actual rigid start→end chord during
            // the soft substep. Apply the response to Rapier's end velocity.
            let boundary = Boundary::Sphere {
                center: start,
                radius: RIGID_RADIUS,
                velocity: [0., 0., (end.position[2] - start[2]) / h],
                angular_velocity: [0.; 3],
            };
            let mobility = SphereMobility {
                inverse_mass: lane.rigid_inverse_mass,
                inverse_inertia: lane.rigid_inverse_mass / (0.4 * RIGID_RADIUS * RIGID_RADIUS),
            };
            let motion = step_coupled(&mut [body], DT, &[boundary], &[Some(mobility)])?[0]
                .ok_or("missing finite sphere correction")?;
            end.position[2] += motion.translation_correction[2];
            end.velocity[2] += motion.linear_velocity_correction[2];
            self.rigid.set_body_state(*id, end)?;
            let [anchor, bob] = body.particles() else {
                return Err("unexpected reference topology".into());
            };
            lane.anchor_z = anchor.position[2];
            lane.anchor_v = anchor.velocity[2];
            lane.bob_z = bob.position[2];
            lane.bob_v = bob.velocity[2];
            lane.rigid_z = end.position[2];
            lane.rigid_v = end.velocity[2];
            lane.last_receiver_impulse_z = body.last_boundary_impulses()[0].linear[2];
        }
        Ok(())
    }
}
