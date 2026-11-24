// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Read-only measurements from committed physics samples.
use super::{COLUMNS, FINGER_RADIUS, PARTICLE_RADIUS, ROWS};
use glam::DVec3;
use gz_physics_xpbd::{BoundaryImpulse, Particle};
use gz_sim_scene::Velocity;
use parry3d_f64::{math::Vector, query::PointQuery, shape::Triangle};

#[derive(Default)]
pub struct Carry {
    relative: Option<(DVec3, DVec3)>,
    pub held_translation: f64,
    pub pad_travel: f64,
    pub relative_com_drift: f64,
    pub tangent_distance: [f64; 2],
    pub tangent_samples: [usize; 2],
    pub last_pad_impulse: Option<usize>,
    pub first_clear: Option<usize>,
    pub touchdown: Option<usize>,
    pub final_clearance: f64,
    pub max_shell_overlap: f64,
    pub max_mid_overlap: f64,
    pub initial_fold: f64,
    pub hold_fold: f64,
}

fn fold(points: &[Particle]) -> f64 {
    (0..ROWS - 1)
        .map(|r| {
            let i = r * COLUMNS + 4;
            let q = |j: usize| DVec3::from(points[j].position);
            let n1 = (q(i) - q(i - 1)).cross(q(i + COLUMNS) - q(i - 1));
            let n2 = (q(i + COLUMNS + 1) - q(i)).cross(q(i + COLUMNS) - q(i));
            n1.angle_between(n2).to_degrees()
        })
        .sum::<f64>()
        / (ROWS - 1) as f64
}

impl Carry {
    pub fn from_seed(points: &[Particle]) -> Self {
        Self {
            initial_fold: fold(points),
            ..Self::default()
        }
    }
    pub fn observe(
        &mut self,
        tick: usize,
        centers: [DVec3; 2],
        twists: [Velocity; 2],
        p: &[Particle],
        reactions: &[BoundaryImpulse],
        dt: f64,
    ) {
        let mid = (centers[0] + centers[1]) * 0.5;
        let com = p.iter().map(|p| DVec3::from(p.position)).sum::<DVec3>() / p.len() as f64;
        if tick == 249 {
            self.relative = Some((com, mid));
        }
        if let Some((initial, pads)) = self.relative
            && (250..1000).contains(&tick)
        {
            self.held_translation = com.x - initial.x;
            self.pad_travel = mid.x - pads.x;
            self.relative_com_drift = self
                .relative_com_drift
                .max((self.held_translation - self.pad_travel).abs());
        }
        if (750..1000).contains(&tick) {
            self.hold_fold += fold(p) / 250.;
        }
        for (i, (center, twist)) in centers.into_iter().zip(twists).enumerate() {
            let loaded = DVec3::from(reactions[i].linear).length() > 1e-10;
            if loaded {
                self.last_pad_impulse = Some(tick + 1);
            }
            if !loaded || !(250..700).contains(&tick) {
                continue;
            }
            let nearest = p
                .iter()
                .min_by(|a, b| {
                    let dist = |p: &Particle| center.distance_squared(p.position.into());
                    dist(a).total_cmp(&dist(b))
                })
                .unwrap();
            let radial = DVec3::from(nearest.position) - center;
            if radial.length() - FINGER_RADIUS - PARTICLE_RADIUS > 0.0001 {
                continue;
            }
            let n = radial.normalize();
            let pad_velocity =
                DVec3::from(twist.linear) + DVec3::from(twist.angular).cross(n * FINGER_RADIUS);
            let rel = DVec3::from(nearest.velocity) - pad_velocity;
            self.tangent_distance[i] += (rel - n * rel.dot(n)).length() * dt;
            self.tangent_samples[i] += 1;
        }
        // The solver samples particle contacts against the pads. Independently
        // check the complete mesh skin against each finite sphere at this tick.
        let mut min_gap = f64::INFINITY;
        for row in 1..ROWS {
            for col in 1..COLUMNS {
                let i = row * COLUMNS + col;
                for tri in [
                    [i - COLUMNS - 1, i - COLUMNS, i],
                    [i - COLUMNS - 1, i, i - 1],
                ] {
                    let [a, b, c] = tri.map(|j| Vector::from_array(p[j].position));
                    let tr = Triangle::new(a, b, c);
                    for center in centers {
                        min_gap = min_gap.min(
                            tr.distance_to_local_point(
                                Vector::from_array(center.to_array()),
                                false,
                            ) - FINGER_RADIUS,
                        );
                    }
                }
            }
        }
        self.max_mid_overlap = self.max_mid_overlap.max(-min_gap);
        self.max_shell_overlap = self.max_shell_overlap.max(PARTICLE_RADIUS - min_gap);
        self.final_clearance = min_gap - PARTICLE_RADIUS;
        if tick >= 1000 {
            if self.first_clear.is_none()
                && self.final_clearance > 1e-5
                && reactions[..2]
                    .iter()
                    .all(|r| DVec3::from(r.linear).length() <= 1e-10)
            {
                self.first_clear = Some(tick + 1);
            }
            if self.touchdown.is_none() && reactions[2].linear[2] < -1e-9 {
                self.touchdown = Some(tick + 1);
            }
        }
    }
    pub fn report(&self) {
        println!(
            "  commanded carry: pad travel {:.6} m, COM travel {:.6} m, max lateral relative COM drift {:.6} m",
            self.pad_travel, self.held_translation, self.relative_com_drift
        );
        println!(
            "  loaded pad tangent travel {:?} m, witness samples {:?}/450; fold {:.2}° at seed, mean {:.2}° at hold",
            self.tangent_distance, self.tangent_samples, self.initial_fold, self.hold_fold
        );
        println!(
            "  last measured pad impulse {:?} ms, clear after opening {:?} ms, first post-open table impulse {:?} ms, final triangle-skin clearance {:.6} m",
            self.last_pad_impulse, self.first_clear, self.touchdown, self.final_clearance
        );
        println!(
            "  max geometric overlap: material mid-surface {:.9e} m, 3 mm mesh skin {:.9e} m",
            self.max_mid_overlap, self.max_shell_overlap
        );
    }
}
