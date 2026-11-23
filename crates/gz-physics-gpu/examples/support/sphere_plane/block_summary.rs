// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::{SpherePlaneBlockObservation as Observed, SpherePlaneLane as Lane};

#[derive(Clone, Copy, Debug)]
pub struct Summary {
    pub normal: f64,
    pub tangent: [f64; 3],
    pub clearance: f64,
    pub peak: f64,
    pub contacts: u32,
    pub first: u32,
    pub last: u32,
}
fn clearance(s: &Lane) -> f64 {
    s.plane_normal
        .iter()
        .zip(s.position)
        .map(|(n, p)| n * p)
        .sum::<f64>()
        - s.plane_offset
        - s.radius
}
impl Summary {
    pub fn initial(s: &Lane) -> Self {
        Self {
            normal: 0.,
            tangent: [0.; 3],
            clearance: clearance(s),
            peak: 0.,
            contacts: 0,
            first: 0,
            last: 0,
        }
    }
    pub fn add(&mut self, s: &Lane, offset: u32) {
        self.normal += s.normal_impulse;
        for (acc, value) in self.tangent.iter_mut().zip(s.tangential_impulse) {
            *acc += value;
        }
        self.clearance = self.clearance.min(clearance(s));
        self.peak = self.peak.max(s.normal_impulse);
        if s.normal_impulse > 0. {
            self.contacts += 1;
            if self.first == 0 {
                self.first = offset;
            }
            self.last = offset;
        }
    }
    pub fn check_device(&self, v: &Observed) {
        assert_eq!(
            self.normal.to_bits(),
            v.normal_impulse_sum.to_bits(),
            "ordered normal impulse"
        );
        assert_eq!(
            self.tangent.map(f64::to_bits),
            v.tangential_impulse_sum.map(f64::to_bits),
            "ordered tangential impulse"
        );
        assert_eq!(self.peak.to_bits(), v.maximum_normal_impulse.to_bits());
        assert_eq!(self.contacts, v.contact_ticks());
        assert_eq!(
            (self.contacts > 0).then_some((self.first, self.last)),
            v.contact_range()
        );
        // The dot product in a COM/plane gap may contract on Vulkan.
        assert!((self.clearance - v.minimum_clearance).abs() < 5e-12);
    }
    pub fn distance(&self, v: &Observed) -> f64 {
        (self.normal - v.normal_impulse_sum).abs().max(
            self.tangent
                .iter()
                .zip(v.tangential_impulse_sum)
                .map(|(a, b)| (a - b).powi(2))
                .sum::<f64>()
                .sqrt(),
        )
    }
}
