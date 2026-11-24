// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compression-only normal response of two elastic layers over a finite surface.
use super::elastic_layers::SeriesSprings;

#[derive(Debug, Clone, Copy)]
pub struct ElasticAreaFoundation {
    springs: SeriesSprings,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElasticAreaSample {
    pub pressure_pa: f64,
    pub energy_j_m2: f64,
    pub foot_compression_m: f64,
    pub deck_compression_m: f64,
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct ElasticAreaIntegral {
    pub normal_force_n: f64,
    pub energy_j: f64,
    /// Integral of pressure times each barycentric basis function. Applying
    /// these parallel forces at the three vertices preserves force and torque.
    pub nodal_forces_n: [f64; 3],
}

impl ElasticAreaFoundation {
    /// Stiffness of each layer is pressure divided by compression (N/m³).
    pub fn new(foot_stiffness_n_m3: f64, deck_stiffness_n_m3: f64) -> Option<Self> {
        Some(Self {
            springs: SeriesSprings::new(foot_stiffness_n_m3, deck_stiffness_n_m3)?,
        })
    }

    pub fn sample(self, compression_m: f64) -> Option<ElasticAreaSample> {
        let s = self.springs.response(compression_m)?;
        Some(ElasticAreaSample {
            pressure_pa: s.load,
            energy_j_m2: s.energy,
            foot_compression_m: s.foot_compression,
            deck_compression_m: s.deck_compression,
        })
    }

    /// Exact integral over a triangle in undeformed material area. Clip to the
    /// support and the compressed side of the zero-overlap line first. The
    /// overlap is affine on the triangle and nonnegative at all three vertices.
    pub fn integrate_triangle(
        self,
        area_m2: f64,
        compression_m: [f64; 3],
    ) -> Option<ElasticAreaIntegral> {
        if !area_m2.is_finite()
            || area_m2 < 0.
            || compression_m.iter().any(|d| !d.is_finite() || *d < 0.)
        {
            return None;
        }
        let peak = compression_m.into_iter().fold(0., f64::max);
        if peak == 0. || area_m2 == 0. {
            return Some(ElasticAreaIntegral::default());
        }
        let peak_pressure = self.sample(peak)?.pressure_pa;
        let ratios = compression_m.map(|d| d / peak);
        let sum = ratios.iter().sum::<f64>();
        // ∫ Ni² dA = A/6; ∫ Ni Nj dA = A/12 for distinct vertices.
        let nodal_forces_n = ratios.map(|s| area_m2 * (peak_pressure * ((sum + s) / 12.)));
        let normal_force_n = nodal_forces_n.iter().sum::<f64>();
        let energy_j = compression_m
            .into_iter()
            .zip(nodal_forces_n)
            .map(|(d, f)| 0.5 * d * f)
            .sum::<f64>();
        (normal_force_n.is_finite() && energy_j.is_finite()).then_some(ElasticAreaIntegral {
            normal_force_n,
            energy_j,
            nodal_forces_n,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ramp_pressures_preserve_energy_and_both_first_moments_under_subdivision() {
        let mat = ElasticAreaFoundation::new(12., 12.).unwrap();
        let flat = mat.integrate_triangle(2., [1.; 3]).unwrap();
        assert_eq!(flat.normal_force_n, 12.);
        assert_eq!(flat.nodal_forces_n, [4.; 3]);
        assert_eq!(flat.energy_j, 6.);
        // Only vertex B is indented. Residual compression is linear along AB and BC.
        let ramp = mat.integrate_triangle(1., [0., 2., 0.]).unwrap();
        assert_eq!(ramp.nodal_forces_n, [1., 2., 1.]);
        assert_eq!(ramp.energy_j, 2.);
        let left = mat.integrate_triangle(0.5, [0., 1., 0.]).unwrap();
        let right = mat.integrate_triangle(0.5, [1., 2., 0.]).unwrap();
        let midpoint = left.nodal_forces_n[1] + right.nodal_forces_n[0];
        assert_eq!(
            [
                left.nodal_forces_n[0] + 0.5 * midpoint,
                right.nodal_forces_n[1] + 0.5 * midpoint,
                left.nodal_forces_n[2] + right.nodal_forces_n[2]
            ],
            ramp.nodal_forces_n
        );
        assert_eq!(left.energy_j + right.energy_j, ramp.energy_j);
    }

    #[test]
    fn area_response_releases_and_rejects_unclipped_or_unrepresentable_integrals() {
        let mat = ElasticAreaFoundation::new(20., 80.).unwrap();
        let loaded = mat.sample(0.25).unwrap();
        assert_eq!(loaded.pressure_pa, 4.);
        assert_eq!(loaded.energy_j_m2, 0.5);
        assert_eq!(loaded.foot_compression_m, 0.2);
        assert_eq!(loaded.deck_compression_m, 0.05);
        assert_eq!(mat.sample(-1.).unwrap().pressure_pa, 0.);
        assert!(mat.sample(f64::NAN).is_none());
        assert_eq!(
            mat.integrate_triangle(0., [2.; 3]),
            Some(ElasticAreaIntegral::default())
        );
        for bad in [f64::NAN, f64::INFINITY, -1.] {
            assert!(mat.integrate_triangle(bad, [1.; 3]).is_none());
            assert!(mat.integrate_triangle(1., [0., 0., bad]).is_none());
            assert!(ElasticAreaFoundation::new(bad, 1.).is_none());
        }
        assert!(mat.integrate_triangle(f64::MAX, [1.; 3]).is_none());
    }
}
