// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Material quadrature for compliant prescribed cuboids.
use super::*;
use crate::collision::{Interval, Projection};

/// A material point on one cloth triangle. Its stiffness is this dimensionless
/// fraction of the pad's reference stiffness. Weights on a face sum to one.
#[derive(Clone, Copy, Debug)]
pub struct PadSample {
    pub particles: [usize; 3],
    pub weights: [f64; 3],
    pub stiffness_weight: f64,
}
impl PadSample {
    fn interpolate(self, at: impl Fn(usize) -> DVec3) -> DVec3 {
        let [a, b, c] = self.particles.map(at);
        a + self.weights[1] * (b - a) + self.weights[2] * (c - a)
    }
}
impl SoftBody {
    /// Replace particle pad contacts with material samples for every compliant
    /// prescribed cuboid. Other boundaries retain their collision model.
    /// For equal indentation, the total pad stiffness is the sum of the
    /// stiffness weights in contact divided by the boundary's compliance.
    pub fn with_pad_samples(mut self, mut samples: Vec<PadSample>) -> Result<Self, XpbdError> {
        let MaterialTopology::Cloth(columns) = self.topology else {
            return Err(XpbdError::InvalidConstraint);
        };
        if samples.is_empty() {
            return Err(XpbdError::InvalidConstraint);
        }
        for s in &mut samples {
            let mut ids = s.particles;
            ids.sort_unstable();
            let sum = s.weights.iter().sum::<f64>();
            let br = ids[2];
            let real_face = br < self.particles.len()
                && br > columns
                && !br.is_multiple_of(columns)
                && topology::cell_faces(br, columns)
                    .into_iter()
                    .any(|mut face| {
                        face.sort_unstable();
                        face == ids
                    });
            if !real_face
                || !s.stiffness_weight.is_finite()
                || s.stiffness_weight <= 0.
                || s.weights.iter().any(|w| !w.is_finite() || *w < 0.)
                || (sum - 1.).abs() > 32. * f64::EPSILON
            {
                return Err(XpbdError::InvalidConstraint);
            }
            s.weights = s.weights.map(|w| w / sum);
        }
        self.contacts.pad_samples = samples;
        Ok(self)
    }
}
impl Contacts {
    pub(super) fn project_pad_samples(
        &mut self,
        x: &mut [DVec3],
        particles: &mut [Particle],
        boundaries: &[PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
        time: Interval,
        active: &mut Vec<(usize, Projection, f64)>,
    ) -> Result<(), XpbdError> {
        for i in 0..self.pad_samples.len() {
            let s = self.pad_samples[i];
            let inverse = s
                .particles
                .into_iter()
                .zip(s.weights)
                .map(|(n, w)| particles[n].inverse_mass * w * w)
                .sum::<f64>();
            let start = s.interpolate(|n| particles[n].position.into());
            let mut predicted = s.interpolate(|n| x[n]);
            // The same scalar XPBD and Coulomb rows act at the interpolated
            // witness. Jᵀ distributes the computed impulse to the nodes.
            let mut witness = Particle {
                position: start.to_array(),
                velocity: [0.; 3],
                inverse_mass: inverse,
            };
            let impulse = self.project_compliant(
                (particles.len() + i, &mut witness, Some(s)),
                &mut predicted,
                boundaries,
                reactions,
                time,
                active,
            )?;
            for (n, w) in s.particles.into_iter().zip(s.weights) {
                let dx = time.h * particles[n].inverse_mass * w * impulse;
                correct(&mut x[n], &mut particles[n], dx, time.h);
            }
        }
        Ok(())
    }

    pub(super) fn verify_pad_samples(
        &self,
        p: &[Particle],
        x: &[DVec3],
        boundaries: &[PreparedBoundary],
        time: Interval,
    ) -> Result<(), XpbdError> {
        if self.pad_samples.is_empty() {
            return Ok(());
        }
        // A later row can move an earlier witness. Check the accepted sweep
        // of every sample and vertex, including vertices outside the patch.
        for b in boundaries.iter().filter(|b| b.normal_compliance > 0.) {
            for (node, &end) in p.iter().zip(x) {
                b.projection(node.position.into(), end, time, false)?;
            }
            for &s in &self.pad_samples {
                b.projection(
                    s.interpolate(|n| p[n].position.into()),
                    s.interpolate(|n| x[n]),
                    time,
                    false,
                )?;
            }
        }
        Ok(())
    }
}
