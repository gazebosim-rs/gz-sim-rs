// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local, compression-only normal response of two elastic layers along a line.

use super::elastic_layers::SeriesSprings;

/// Two linear springs in series per metre of contact edge. Neighboring positions
/// on the line deform independently. Integrate the sample over the current
/// contact length for total force and stored energy.
#[derive(Debug, Clone, Copy)]
pub struct ElasticLineFoundation {
    springs: SeriesSprings,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElasticLineSample {
    /// Compressive normal force per metre of edge, in N/m.
    pub line_load_n_m: f64,
    /// Recoverable energy per metre of edge, in J/m.
    pub energy_j_m: f64,
    /// Positive deformation of the foot and deck layers, in metres.
    pub foot_compression_m: f64,
    pub deck_compression_m: f64,
}

/// Exact normal response integrated over a straight edge with uniform layer stiffnesses.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct ElasticLineIntegral {
    pub normal_force_n: f64,
    pub energy_j: f64,
    /// Length carrying positive pressure, in metres.
    pub loaded_length_m: f64,
    /// Position from the first endpoint, as a fraction of the full segment length.
    /// Undefined when the segment exerts zero normal force.
    pub center_of_pressure_fraction: Option<f64>,
}

impl ElasticLineFoundation {
    /// Stiffness is force per metre of edge per metre of compression (N/m²).
    pub fn new(foot_stiffness_n_m2: f64, deck_stiffness_n_m2: f64) -> Option<Self> {
        Some(Self {
            springs: SeriesSprings::new(foot_stiffness_n_m2, deck_stiffness_n_m2)?,
        })
    }

    /// A positive signed overlap compresses the layers. Zero or a gap carries no load.
    pub fn sample(self, compression_m: f64) -> Option<ElasticLineSample> {
        let sample = self.springs.response(compression_m)?;
        Some(ElasticLineSample {
            line_load_n_m: sample.load,
            energy_j_m: sample.energy,
            foot_compression_m: sample.foot_compression,
            deck_compression_m: sample.deck_compression,
        })
    }

    /// Integrate the positive part of the signed overlap. Overlap varies
    /// linearly between the two endpoints of a straight edge above a plane.
    pub fn integrate_segment(
        self,
        length_m: f64,
        overlap_m: [f64; 2],
    ) -> Option<ElasticLineIntegral> {
        let [a, b] = overlap_m;
        if !length_m.is_finite() || length_m < 0. || !a.is_finite() || !b.is_finite() {
            return None;
        }
        let high = a.max(b);
        if high <= 0. || length_m == 0. {
            return Some(ElasticLineIntegral::default());
        }
        let low = a.min(b);
        let active_fraction = if low < 0. {
            // Measure the compressed portion from the loaded end. Subtracting
            // a near-one root from one loses very short corner contacts.
            let scale = high.max(-low);
            (high / scale) / (high / scale - low / scale)
        } else {
            1.
        };
        let loaded_length_m = length_m * active_fraction;
        let ratio = low.max(0.) / high;
        let peak = self.sample(high)?;
        // Pressure is a trapezoid and stored energy varies quadratically.
        let normal_force_n = peak.line_load_n_m * (0.5 * (1. + ratio) * loaded_length_m);
        let energy_j = peak.energy_j_m * (((1. + ratio + ratio * ratio) / 3.) * loaded_length_m);
        let offset = active_fraction * ((1. + 2. * ratio) / (3. * (1. + ratio)));
        let center = if a >= b { offset } else { 1. - offset };
        (normal_force_n.is_finite() && energy_j.is_finite()).then_some(ElasticLineIntegral {
            normal_force_n,
            energy_j,
            loaded_length_m,
            center_of_pressure_fraction: (normal_force_n > 0.).then_some(center),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tilted_edge_gives_exact_load_center_and_energy_through_release() {
        let f = ElasticLineFoundation::new(16., 16.).unwrap();
        // One quarter of the 4 m segment is compressed, with peak line load 8 N/m.
        let triangle = f.integrate_segment(4., [1., -3.]).unwrap();
        assert_eq!(triangle.loaded_length_m, 1.);
        assert_eq!(triangle.normal_force_n, 4.);
        assert_eq!(triangle.energy_j, 4. / 3.);
        assert_eq!(triangle.center_of_pressure_fraction, Some(1. / 12.));
        let reversed = f.integrate_segment(4., [-3., 1.]).unwrap();
        assert_eq!(reversed.normal_force_n, triangle.normal_force_n);
        assert_eq!(reversed.energy_j, triangle.energy_j);
        assert_eq!(reversed.center_of_pressure_fraction, Some(1. - 1. / 12.));
        let both = f.integrate_segment(3., [0.5, 1.]).unwrap();
        assert_eq!(both.normal_force_n, 18.);
        assert_eq!(both.energy_j, 7.);
        assert!((both.center_of_pressure_fraction.unwrap() - 5. / 9.).abs() < 1e-15);
        let flat = f.integrate_segment(3., [0.5; 2]).unwrap();
        assert_eq!(flat.normal_force_n, 12.);
        assert_eq!(flat.energy_j, 3.);
        assert_eq!(flat.center_of_pressure_fraction, Some(0.5));
        for endpoints in [[-1., -1.], [0., -1.], [-1., 0.], [0., 0.]] {
            assert_eq!(
                f.integrate_segment(3., endpoints),
                Some(ElasticLineIntegral::default())
            );
        }
        assert_eq!(
            f.integrate_segment(0., [1., 2.]),
            Some(ElasticLineIntegral::default())
        );
    }

    #[test]
    fn corner_slivers_survive_clipping_and_invalid_integrals_are_rejected() {
        let f = ElasticLineFoundation::new(2., 2.).unwrap();
        let sliver = f.integrate_segment(1., [-1., 1e-20]).unwrap();
        assert_eq!(sliver.loaded_length_m, 1e-20);
        assert!((sliver.normal_force_n / 5e-41 - 1.).abs() < 1e-14);
        for invalid in [f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
            assert!(f.integrate_segment(1., [0., invalid]).is_none());
            assert!(f.integrate_segment(1., [invalid, 0.]).is_none());
            assert!(f.integrate_segment(invalid, [1., 2.]).is_none());
        }
        assert!(f.integrate_segment(-1., [1., 1.]).is_none());
        assert!(f.integrate_segment(f64::MAX, [2., 2.]).is_none());
    }

    #[test]
    fn series_layers_share_load_and_store_the_work_of_compression() {
        let f = ElasticLineFoundation::new(600., 300.).unwrap();
        let d = 0.03;
        let s = f.sample(d).unwrap();
        assert!((s.foot_compression_m - 0.01).abs() < 1e-16);
        assert!((s.deck_compression_m - 0.02).abs() < 1e-16);
        assert!((s.line_load_n_m - 6.).abs() < 1e-14);
        let energy =
            0.5 * (600. * s.foot_compression_m.powi(2) + 300. * s.deck_compression_m.powi(2));
        assert!((s.energy_j_m - energy).abs() < 1e-15);
        let h = 1e-6;
        let marginal =
            (f.sample(d + h).unwrap().energy_j_m - f.sample(d - h).unwrap().energy_j_m) / (2. * h);
        assert!((marginal - s.line_load_n_m).abs() < 1e-10);
        let reversed = ElasticLineFoundation::new(300., 600.)
            .unwrap()
            .sample(d)
            .unwrap();
        assert_eq!(reversed.line_load_n_m, s.line_load_n_m);
        assert_eq!(reversed.foot_compression_m, s.deck_compression_m);
    }

    #[test]
    fn opens_without_tension_and_remains_finite_for_a_stiff_deck() {
        let f = ElasticLineFoundation::new(80., f64::MAX).unwrap();
        let open = f.sample(-1.).unwrap();
        assert_eq!(open, f.sample(0.).unwrap());
        assert_eq!(
            open.line_load_n_m
                + open.energy_j_m
                + open.foot_compression_m
                + open.deck_compression_m,
            0.
        );
        let pressed = f.sample(0.25).unwrap();
        assert_eq!(pressed.line_load_n_m, 20.);
        assert_eq!(pressed.foot_compression_m, 0.25);
        assert!(pressed.deck_compression_m > 0.);
        let stiff = ElasticLineFoundation::new(f64::MAX, f64::MAX).unwrap();
        assert!(stiff.sample(1.).is_some());
        assert!(stiff.sample(3.).is_none());
        assert!(f.sample(f64::MAX).is_none());
        for invalid in [0., -1., f64::INFINITY, f64::NEG_INFINITY, f64::NAN] {
            assert!(ElasticLineFoundation::new(invalid, 1.).is_none());
            assert!(ElasticLineFoundation::new(1., invalid).is_none());
        }
        for invalid in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(f.sample(invalid).is_none());
        }
    }
}
