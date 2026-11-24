// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Rate-dependent normal response of two elastic layers over their contact area.
use super::ElasticAreaFoundation;

/// Linear Hunt–Crossley contact. At positive overlap the elastic pressure is
/// multiplied by max(0, 1 + α v), where v is the local compression rate.
/// The layer pair never pulls on the opposing surface.
#[derive(Debug, Clone, Copy)]
pub struct DampedAreaFoundation {
    elastic: ElasticAreaFoundation,
    damping_s_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DampedAreaSample {
    /// Total compressive pressure; also sets the local Coulomb friction limit.
    pub pressure_pa: f64,
    /// Pressure the layers would exert at this compression with zero velocity.
    pub elastic_pressure_pa: f64,
    /// Recoverable elastic strain energy per square metre.
    pub energy_j_m2: f64,
    /// Rate of irreversible energy loss per square metre, including during
    /// rapid unloading when the tensile pressure limit is active.
    pub dissipation_w_m2: f64,
}

impl ElasticAreaFoundation {
    /// Add damping to the combined layer pair. The coefficient is in seconds
    /// per metre of indentation, independent of mesh size or simulation step.
    pub fn with_damping(self, damping_s_m: f64) -> Option<DampedAreaFoundation> {
        (damping_s_m.is_finite() && damping_s_m >= 0.).then_some(DampedAreaFoundation {
            elastic: self,
            damping_s_m,
        })
    }
}

impl DampedAreaFoundation {
    /// Infer α (s/m) from positive incoming and outgoing normal speeds of a
    /// centered, flat impact on a stationary support. Use relative speeds at
    /// first touch and release. This inversion assumes negligible external
    /// impulse during contact, no rotation and no other energy loss.
    pub fn coefficient_from_flat_impact(incoming_m_s: f64, outgoing_m_s: f64) -> Option<f64> {
        if !incoming_m_s.is_finite()
            || !outgoing_m_s.is_finite()
            || outgoing_m_s <= 0.
            || outgoing_m_s > incoming_m_s
        {
            return None;
        }
        if outgoing_m_s == incoming_m_s {
            return Some(0.);
        }
        // Separating variables in m x'' = -k x (1 + α x') between x=0
        // crossings gives ln(1+α vin) - ln(1-α vout) = s = α(vin+vout).
        // Thus z = α vout = 1 - s/expm1(s). Bracket the nonzero root on
        // [0,1], including very inelastic drops where z rounds to one.
        let e = outgoing_m_s / incoming_m_s;
        let relative_speed_loss = ((incoming_m_s - outgoing_m_s) / incoming_m_s) / (1. + e);
        let (mut lo, mut hi) = (0., 1.);
        for _ in 0..128 {
            let z = (lo + hi) * 0.5;
            if z == lo || z == hi {
                break;
            }
            let s = z * (1. + e) / e;
            let above_root = if s < 0.1 {
                // coth(s/2)-2/s evaluated about zero for nearly elastic drops.
                let s2 = s * s;
                let split = s * (1. / 6. + s2 * (-1. / 360. + s2 * (1. / 15120. - s2 / 604800.)));
                split >= relative_speed_loss
            } else if s > 700. {
                z >= 1.
            } else {
                z >= 1. - s / s.exp_m1()
            };
            if above_root {
                hi = z;
            } else {
                lo = z;
            }
        }
        let alpha = (0.5 * (lo + hi)) / outgoing_m_s;
        alpha.is_finite().then_some(alpha)
    }

    /// Positive speed closes the gap. At zero or negative overlap both
    /// pressure and dissipation vanish, including for an approaching contact.
    pub fn sample(
        self,
        compression_m: f64,
        compression_speed_m_s: f64,
    ) -> Option<DampedAreaSample> {
        if !compression_speed_m_s.is_finite() {
            return None;
        }
        let e = self.elastic.sample(compression_m)?;
        let mut s = DampedAreaSample {
            pressure_pa: e.pressure_pa,
            elastic_pressure_pa: e.pressure_pa,
            energy_j_m2: e.energy_j_m2,
            dissipation_w_m2: 0.,
        };
        if e.pressure_pa == 0. || self.damping_s_m == 0. {
            return Some(s);
        }
        // Compute the rate contribution separately from total pressure: subtracting
        // two almost identical pressures loses the small dissipative work.
        let ratio = self.damping_s_m * compression_speed_m_s;
        s.pressure_pa = e.pressure_pa * self.damping_s_m.mul_add(compression_speed_m_s, 1.).max(0.);
        s.dissipation_w_m2 = (e.pressure_pa * ratio.max(-1.)) * compression_speed_m_s;
        (s.pressure_pa.is_finite() && s.dissipation_w_m2.is_finite()).then_some(s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovers_flat_impact_coefficient_across_impact_speeds() {
        for alpha in [0.02, 0.3, 0.8, 3.] {
            // For s=ln(2), the exact incoming and outgoing dimensionless
            // speeds are 2 ln(2) - 1 and 1 - ln(2), respectively.
            let incoming = (2. * std::f64::consts::LN_2 - 1.) / alpha;
            let outgoing = (1. - std::f64::consts::LN_2) / alpha;
            let got =
                DampedAreaFoundation::coefficient_from_flat_impact(incoming, outgoing).unwrap();
            assert!((got / alpha - 1.).abs() < 1e-13);
        }
        let infer = DampedAreaFoundation::coefficient_from_flat_impact;
        assert_eq!(infer(1., 1.), Some(0.));
        let almost = 1_f64.next_down();
        assert!((infer(1., almost).unwrap() / (1. - almost) - 1.5).abs() < 1e-14);
        assert_eq!(infer(1., 0.001), Some(1000.));
        for (incoming, outgoing) in [
            (0., 0.),
            (1., 0.),
            (1., 1.1),
            (1., -0.1),
            (f64::INFINITY, 1.),
            (1., f64::NAN),
        ] {
            assert!(infer(incoming, outgoing).is_none());
        }
    }

    #[test]
    fn contact_power_splits_into_stored_energy_and_heat_even_when_pressure_clips() {
        let foundation = ElasticAreaFoundation::new(800., 800.)
            .unwrap()
            .with_damping(0.5)
            .unwrap();
        for (speed, pressure, heat) in [
            (2., 80., 80.),
            (0., 40., 0.),
            (-1., 20., 20.),
            (-2., 0., 80.),
            (-3., 0., 120.),
        ] {
            let sample = foundation.sample(0.1, speed).unwrap();
            assert_eq!(sample.pressure_pa, pressure);
            assert_eq!(sample.energy_j_m2, 2.);
            assert_eq!(sample.dissipation_w_m2, heat);
            assert_eq!(speed * pressure, speed * sample.elastic_pressure_pa + heat);
        }
    }

    #[test]
    fn no_contact_or_zero_damping_recovers_elastic_law_and_zero_loss() {
        let elastic = ElasticAreaFoundation::new(2e8, 2e8).unwrap();
        let damped = elastic.with_damping(0.8).unwrap();
        let undamped = elastic.with_damping(0.).unwrap();
        for d in [-1., 0., 1e-10, 0.003] {
            let e = elastic.sample(d).unwrap();
            for v in [-3.5, 0., 2.] {
                let s = undamped.sample(d, v).unwrap();
                assert_eq!(
                    (s.pressure_pa, s.energy_j_m2, s.dissipation_w_m2),
                    (e.pressure_pa, e.energy_j_m2, 0.)
                );
                if d <= 0. {
                    assert_eq!(damped.sample(d, v).unwrap(), s);
                }
            }
        }
        assert!(damped.sample(0.1, 1e-18).unwrap().dissipation_w_m2 > 0.);
        for bad in [f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
            assert!(elastic.with_damping(bad).is_none());
            assert!(damped.sample(0., bad).is_none());
            assert!(damped.sample(bad, 0.).is_none());
        }
        assert!(elastic.with_damping(-0.1).is_none());
        assert!(damped.sample(0.5, f64::MAX).is_none());
    }
}
