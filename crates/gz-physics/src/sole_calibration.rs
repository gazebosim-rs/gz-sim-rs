// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Infer material properties from controlled measurements in SI units.
use crate::{
    DampedAreaFoundation,
    flat_landing::FlatLanding,
    sole_contact::{SoleError, SoleMaterial},
};

/// Independent equilibrium indentations or known stiffness resolve how the two
/// layers share strain. Impact measurements alone only identify their series stiffness.
#[derive(Clone, Copy, Debug)]
pub struct LayerPair {
    pub foot_n_m3: f64,
    pub deck_n_m3: f64,
}
fn positive(x: f64) -> bool {
    x.is_finite() && x > 0.
}
impl LayerPair {
    pub fn from_static(force_n: f64, area_m2: f64, foot_m: f64, deck_m: f64) -> Option<Self> {
        if ![force_n, area_m2, foot_m, deck_m].into_iter().all(positive) {
            return None;
        }
        let pressure = force_n / area_m2;
        let foot_n_m3 = pressure / foot_m;
        let deck_n_m3 = pressure / deck_m;
        (positive(foot_n_m3) && positive(deck_n_m3)).then_some(Self {
            foot_n_m3,
            deck_n_m3,
        })
    }
    pub fn with_known_deck(effective_n_m3: f64, deck_n_m3: f64) -> Option<Self> {
        if !positive(effective_n_m3) || !positive(deck_n_m3) || effective_n_m3 >= deck_n_m3 {
            return None;
        }
        let foot_n_m3 = effective_n_m3 / ((deck_n_m3 - effective_n_m3) / deck_n_m3);
        positive(foot_n_m3).then_some(Self {
            foot_n_m3,
            deck_n_m3,
        })
    }
    pub fn with_foot_share(effective_n_m3: f64, foot_strain_fraction: f64) -> Option<Self> {
        if !positive(effective_n_m3)
            || !(0. ..1.).contains(&foot_strain_fraction)
            || foot_strain_fraction == 0.
        {
            return None;
        }
        let foot_n_m3 = effective_n_m3 / foot_strain_fraction;
        let deck_n_m3 = effective_n_m3 / (1. - foot_strain_fraction);
        (positive(foot_n_m3) && positive(deck_n_m3)).then_some(Self {
            foot_n_m3,
            deck_n_m3,
        })
    }
    pub fn material(
        self,
        damping: f64,
        friction: f64,
        slip_speed: f64,
    ) -> Result<SoleMaterial, SoleError> {
        SoleMaterial::new(
            self.foot_n_m3,
            self.deck_n_m3,
            damping,
            friction,
            slip_speed,
        )
    }
}

#[derive(Debug, Clone, Copy)]
pub enum ImpactMeasurement {
    MaximumCompressionM(f64),
    ContactDurationS(f64),
    PeakForceN(f64),
}
#[derive(Debug, Clone, Copy)]
pub struct DropFit {
    pub damping_s_m: f64,
    /// The effective stiffness per undeformed loaded area (N/m³).
    pub effective_n_m3: f64,
}
impl DropFit {
    pub fn measure(
        mass_kg: f64,
        area_m2: f64,
        incoming_m_s: f64,
        outgoing_m_s: f64,
        m: ImpactMeasurement,
    ) -> Option<Self> {
        if !positive(area_m2) {
            return None;
        }
        let a = DampedAreaFoundation::coefficient_from_flat_impact(incoming_m_s, outgoing_m_s)?;
        let k = match m {
            ImpactMeasurement::MaximumCompressionM(d) => {
                FlatLanding::stiffness_from_depth(mass_kg, a, incoming_m_s, d)
            }
            ImpactMeasurement::ContactDurationS(t) => {
                FlatLanding::stiffness_from_duration(mass_kg, a, incoming_m_s, t)
            }
            ImpactMeasurement::PeakForceN(f) => {
                FlatLanding::stiffness_from_peak_force(mass_kg, a, incoming_m_s, f)
            }
        }?;
        let effective_n_m3 = k / area_m2;
        positive(effective_n_m3).then_some(Self {
            damping_s_m: a,
            effective_n_m3,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};
    #[test]
    fn elapsed_time_and_both_depth_crossings_recover_state() {
        for alpha in [0., 0.5, 30.] {
            let d = FlatLanding::new(1., 20000., alpha, 0.7).unwrap();
            for phase in [0., 0.1, 0.9, FRAC_PI_2, 2., PI] {
                let s = d.at_phase(phase).unwrap();
                let t = d.at_time(s.elapsed_s).unwrap();
                let x = d
                    .at_compression(s.compression_m, phase > FRAC_PI_2)
                    .unwrap();
                assert!((t.closing_m_s - s.closing_m_s).abs() < 1e-11);
                assert!((x.elapsed_s - s.elapsed_s).abs() < 1e-12);
            }
            assert!(d.at_time(-1.).is_none());
            assert!(d.at_time(2. * d.at_phase(PI).unwrap().elapsed_s).is_none());
        }
    }
    #[test]
    fn three_independent_instruments_identify_the_same_material() {
        let a = 1.3;
        let m = 3.2;
        let area = 0.022;
        let k = 1.5e6;
        let d = FlatLanding::new(m, k * area, a, 0.8).unwrap();
        let out = d.at_phase(PI).unwrap();
        let peak = d.peak_force();
        assert!(peak.elapsed_s < d.at_phase(FRAC_PI_2).unwrap().elapsed_s);
        for off in [-1e-6, 1e-6] {
            assert!(d.at_time(peak.elapsed_s + off).unwrap().normal_force_n < peak.normal_force_n)
        }
        for instrument in [
            ImpactMeasurement::MaximumCompressionM(d.maximum_compression_m()),
            ImpactMeasurement::ContactDurationS(out.elapsed_s),
            ImpactMeasurement::PeakForceN(peak.normal_force_n),
        ] {
            let fit = DropFit::measure(m, area, 0.8, -out.closing_m_s, instrument).unwrap();
            assert!((fit.damping_s_m - a).abs() < 1e-12);
            assert!((fit.effective_n_m3 - k).abs() < 1e-6);
        }
    }
    #[test]
    fn static_layer_measurements_and_known_deck_agree() {
        let static_fit = LayerPair::from_static(24., 0.12, 0.0005, 0.0015).unwrap();
        for pair in [
            LayerPair::with_known_deck(100000., static_fit.deck_n_m3).unwrap(),
            LayerPair::with_foot_share(100000., 0.25).unwrap(),
        ] {
            assert!((pair.foot_n_m3 - static_fit.foot_n_m3).abs() < 1e-8);
            assert!((pair.deck_n_m3 - static_fit.deck_n_m3).abs() < 1e-8);
            pair.material(0.4, 0.3, 0.005).unwrap();
        }
        assert!(LayerPair::with_known_deck(2000., 2000.).is_none());
        assert!(LayerPair::with_foot_share(2000., 1.).is_none());
    }
}

/// Inclusive measurement limits. Callers specify instrument error explicitly.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeasurementRange {
    pub low: f64,
    pub high: f64,
}

impl MeasurementRange {
    pub fn positive(low: f64, high: f64) -> Option<Self> {
        (positive(low) && positive(high) && high >= low).then_some(Self { low, high })
    }
    pub fn intersect(self, rhs: Self) -> Option<Self> {
        if ![self.low, self.high, rhs.low, rhs.high]
            .into_iter()
            .all(f64::is_finite)
        {
            return None;
        }
        let low = self.low.max(rhs.low);
        let high = self.high.min(rhs.high);
        (low <= high).then_some(Self { low, high })
    }
}

/// Enclose the damping coefficients consistent with one flat impact. For a
/// fixed outgoing speed the fitted damping increases with incoming speed;
/// for fixed incoming speed it decreases with outgoing speed. Overlapping
/// speed ranges admit a perfectly elastic sample.
pub fn damping_range(
    incoming: MeasurementRange,
    outgoing: MeasurementRange,
) -> Option<MeasurementRange> {
    let incoming = MeasurementRange::positive(incoming.low, incoming.high)?;
    let outgoing = MeasurementRange::positive(outgoing.low, outgoing.high)?;
    if outgoing.low > incoming.high {
        return None;
    }
    let fit = DampedAreaFoundation::coefficient_from_flat_impact;
    let low = if outgoing.high >= incoming.low {
        0.
    } else {
        fit(incoming.low, outgoing.high)?
    };
    let high = fit(incoming.high, outgoing.low)?;
    Some(MeasurementRange { low, high })
}

/// Intersect admissible damping intervals from repeated impacts. Disjoint
/// intervals identify an inconsistent calibration instead of averaging it away.
pub fn common_damping(drops: &[(MeasurementRange, MeasurementRange)]) -> Option<MeasurementRange> {
    let (first, others) = drops.split_first()?;
    others.iter().try_fold(
        damping_range(first.0, first.1)?,
        |interval, &(incoming, outgoing)| interval.intersect(damping_range(incoming, outgoing)?),
    )
}

#[cfg(test)]
mod measurement_tests {
    use super::*;
    #[test]
    fn speed_bounds_enclose_every_admissible_corner_and_overlap() {
        let r = |a, b| MeasurementRange::positive(a, b).unwrap();
        let envelope = damping_range(r(0.8, 1.2), r(0.5, 0.7)).unwrap();
        for i in 0..=20 {
            for j in 0..=20 {
                let a = DampedAreaFoundation::coefficient_from_flat_impact(
                    0.8 + 0.02 * f64::from(i),
                    0.5 + 0.01 * f64::from(j),
                )
                .unwrap();
                assert!(a >= envelope.low - 1e-14 && a <= envelope.high + 1e-14);
            }
        }
        assert_eq!(damping_range(r(0.8, 1.), r(0.9, 1.1)).unwrap().low, 0.);
        assert!(damping_range(r(0.6, 0.7), r(0.9, 1.)).is_none());
    }
    #[test]
    fn replicated_drops_recover_a_joint_interval_and_flag_incompatible_measurements() {
        let intervals = [0.4, 0.8, 1.6].map(|v| {
            let rebound = -FlatLanding::new(1., 12000., 0.6, v)
                .unwrap()
                .at_phase(std::f64::consts::PI)
                .unwrap()
                .closing_m_s;
            (
                MeasurementRange::positive(v - 0.001, v + 0.001).unwrap(),
                MeasurementRange::positive(rebound - 0.001, rebound + 0.001).unwrap(),
            )
        });
        let band = common_damping(&intervals).unwrap();
        assert!(band.low < 0.6 && band.high > 0.6 && band.high - band.low < 0.01);
        let exact = |v| MeasurementRange::positive(v, v).unwrap();
        assert!(common_damping(&[(exact(1.), exact(0.5)), (exact(1.), exact(1.))]).is_none());
    }
}
