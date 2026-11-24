// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent solution of a centered, flat Hunt–Crossley drop. The entire sole
//! (or a constant, symmetric subset of it) bears load without rotation, and external relative acceleration
//! is zero. All speeds and displacements are relative to the deck.
use std::f64::consts::{FRAC_PI_2, PI};

/// A drop begins as the unloaded surfaces touch, moving toward each other.
#[derive(Clone, Copy, Debug)]
pub struct FlatLanding {
    mass: f64,
    stiffness: f64,
    incoming: f64,
    damping: f64,
    omega: f64,
    amplitude_speed: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct FlatLandingSample {
    pub elapsed_s: f64,
    pub compression_m: f64,
    /// Positive while compressing, negative while unloading.
    pub closing_m_s: f64,
    pub normal_force_n: f64,
    pub impulse_ns: f64,
    pub strain_j: f64,
    pub heat_j: f64,
}

// sqrt(2 (q - log(1+q))) / |q|, continued through zero.
fn energy_factor(q: f64) -> f64 {
    if q.abs() < 1e-3 {
        (1. + q * (-2. / 3. + q * (0.5 + q * (-0.4 + q / 3.)))).sqrt()
    } else {
        (2. * (q - q.ln_1p())).sqrt() / q.abs()
    }
}

// Transform the harmonic velocity h back to physical velocity u. This form
// keeps the time integrand h/u continuous at maximum indentation.
fn time_factor(p: f64) -> f64 {
    if p.abs() < 1e-4 {
        return 1. + p * (-1. / 3. + p * (1. / 12. - p * (2. / 135.)));
    }
    let (mut lo, mut hi) = if p > 0. {
        (0., p * (0.5 * p + (0.5 * p).hypot(1.)))
    } else {
        (-1., 0.)
    };
    for _ in 0..64 {
        let q = 0.5 * (lo + hi);
        if q == lo || q == hi {
            break;
        }
        if q * energy_factor(q) < p {
            lo = q;
        } else {
            hi = q;
        }
    }
    p / (0.5 * (lo + hi))
}

impl FlatLanding {
    /// Combined stiffness equals the contact area times the series stiffness
    /// per unit area. Units are kg, N/m, s/m and m/s respectively.
    pub fn new(mass: f64, stiffness: f64, damping: f64, incoming: f64) -> Option<Self> {
        if ![mass, stiffness, incoming]
            .into_iter()
            .all(|x| x.is_finite() && x > 0.)
            || !damping.is_finite()
            || damping < 0.
        {
            return None;
        }
        let omega = (stiffness / mass).sqrt();
        let q = damping * incoming;
        let amplitude_speed = incoming * energy_factor(q);
        let drop = Self {
            mass,
            stiffness,
            incoming,
            damping,
            omega,
            amplitude_speed,
        };
        (omega > 0.
            && amplitude_speed > 0.
            && [
                q,
                omega,
                amplitude_speed,
                amplitude_speed / omega,
                (damping * amplitude_speed).powi(2),
                mass * incoming * incoming,
                stiffness * (amplitude_speed / omega) * (1. + q),
                PI * (1. + damping * amplitude_speed) / omega,
                mass * incoming * 2.,
            ]
            .into_iter()
            .all(f64::is_finite))
        .then_some(drop)
    }

    pub fn maximum_compression_m(self) -> f64 {
        self.amplitude_speed / self.omega
    }

    fn integrate(self, a: f64, b: f64) -> f64 {
        // Eight-point Gauss–Legendre on the regularized phase coordinate.
        const NODES: [(f64, f64); 4] = [
            (0.1834346424956498, 0.362683783378362),
            (0.525532409916329, 0.3137066458778873),
            (0.7966664774136267, 0.2223810344533745),
            (0.9602898564975363, 0.1012285362903763),
        ];
        let half = (b - a) * 0.5;
        let mid = (a + b) * 0.5;
        let scale = self.damping * self.amplitude_speed;
        half * NODES
            .iter()
            .map(|&(x, w)| {
                w * (time_factor(scale * (mid - half * x).cos())
                    + time_factor(scale * (mid + half * x).cos()))
            })
            .sum::<f64>()
    }

    fn quadrature(self, a: f64, b: f64, whole: f64, depth: u8) -> f64 {
        let mid = (a + b) * 0.5;
        let left = self.integrate(a, mid);
        let right = self.integrate(mid, b);
        if depth == 0 || (left + right - whole).abs() <= 2e-13 * (left + right) {
            left + right
        } else {
            self.quadrature(a, mid, left, depth - 1) + self.quadrature(mid, b, right, depth - 1)
        }
    }

    fn elapsed(self, theta: f64) -> f64 {
        if self.damping == 0. || theta == 0. {
            return theta / self.omega;
        }
        self.quadrature(0., theta, self.integrate(0., theta), 12) / self.omega
    }

    /// Smooth phase runs from zero at touch, through π/2 at maximum strain,
    /// to π at release. Phase is nonlinear in elapsed time when damped.
    pub fn at_phase(self, theta: f64) -> Option<FlatLandingSample> {
        if !(0. ..=PI).contains(&theta) {
            return None;
        }
        let h = self.amplitude_speed * theta.cos();
        let speed = if theta == 0. {
            self.incoming
        } else if theta == FRAC_PI_2 {
            0.
        } else {
            h / time_factor(self.damping * h)
        };
        let x = if theta == PI {
            0.
        } else {
            self.maximum_compression_m() * theta.sin()
        };
        let strain = 0.5 * self.stiffness * x * x;
        let lost_kinetic = 0.5 * self.mass * (self.incoming - speed) * (self.incoming + speed);
        Some(FlatLandingSample {
            elapsed_s: self.elapsed(theta),
            compression_m: x,
            closing_m_s: speed,
            normal_force_n: self.stiffness * x * (1. + self.damping * speed).max(0.),
            impulse_ns: self.mass * (self.incoming - speed),
            strain_j: strain,
            heat_j: if self.damping == 0. {
                0.
            } else {
                (lost_kinetic - strain).max(0.)
            },
        })
    }

    /// First/second crossing of an indentation during loading/unloading.
    pub fn at_compression(self, depth_m: f64, unloading: bool) -> Option<FlatLandingSample> {
        if !depth_m.is_finite() || !(0. ..=self.maximum_compression_m()).contains(&depth_m) {
            return None;
        }
        let angle = (depth_m / self.maximum_compression_m()).asin();
        self.at_phase(if unloading { PI - angle } else { angle })
    }

    /// Query an arbitrary time during contact, including both endpoint events.
    pub fn at_time(self, time_s: f64) -> Option<FlatLandingSample> {
        if !time_s.is_finite() || time_s < 0. {
            return None;
        }
        let release = self.elapsed(PI);
        if time_s > release {
            return None;
        }
        if time_s == 0. {
            return self.at_phase(0.);
        }
        if time_s == release {
            return self.at_phase(PI);
        }
        let mut lo = 0.;
        let mut hi = PI;
        let mut theta = PI * time_s / release;
        for _ in 0..50 {
            let delta = self.elapsed(theta) - time_s;
            if delta.abs() < 2e-14 * release {
                break;
            }
            if delta < 0. {
                lo = theta
            } else {
                hi = theta
            }
            let slope = time_factor(self.damping * self.amplitude_speed * theta.cos()) / self.omega;
            let next = theta - delta / slope;
            theta = if next > lo && next < hi {
                next
            } else {
                (lo + hi) * 0.5
            };
        }
        self.at_phase(theta)
    }

    /// Peak force precedes peak compression in a dissipative drop.
    pub fn peak_force(self) -> FlatLandingSample {
        if self.damping == 0. {
            return self.at_phase(FRAC_PI_2).unwrap();
        }
        let (mut lo, mut hi) = (0., FRAC_PI_2);
        for _ in 0..55 {
            let t = 0.5 * (lo + hi);
            let h = self.amplitude_speed * t.cos();
            let speed = h / time_factor(self.damping * h);
            // F'=k(1+αu)(u-αω²x²), with ωx=amplitude_speed sin θ.
            let w = self.amplitude_speed * t.sin();
            if speed > self.damping * w * w {
                lo = t
            } else {
                hi = t
            }
        }
        self.at_phase((lo + hi) * 0.5).unwrap()
    }

    /// Fit stiffness from either contact duration or peak force of a centered drop.
    /// Both inversions exploit the square-root time/force scaling with stiffness.
    pub fn stiffness_from_duration(
        mass: f64,
        damping: f64,
        incoming: f64,
        duration_s: f64,
    ) -> Option<f64> {
        if !duration_s.is_finite() || duration_s <= 0. {
            return None;
        }
        let unit = Self::new(mass, 1., damping, incoming)?;
        let k = (unit.elapsed(PI) / duration_s).powi(2);
        (k.is_finite() && k > 0.).then_some(k)
    }
    pub fn stiffness_from_peak_force(
        mass: f64,
        damping: f64,
        incoming: f64,
        force_n: f64,
    ) -> Option<f64> {
        if !force_n.is_finite() || force_n <= 0. {
            return None;
        }
        let unit = Self::new(mass, 1., damping, incoming)?;
        let k = (force_n / unit.peak_force().normal_force_n).powi(2);
        (k.is_finite() && k > 0.).then_some(k)
    }

    /// Infer combined stiffness from the maximum compression of this drop.
    pub fn stiffness_from_depth(mass: f64, damping: f64, incoming: f64, depth: f64) -> Option<f64> {
        if !depth.is_finite() || depth <= 0. {
            return None;
        }
        let scale = Self::new(mass, 1., damping, incoming)?.amplitude_speed / depth;
        let k = mass * scale * scale;
        (k.is_finite() && k > 0.).then_some(k)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn undamped_reference_is_a_half_sine() {
        let d = FlatLanding::new(2., 200., 0., 3.).unwrap();
        for i in 0..=32 {
            let t = PI * f64::from(i) / 32.;
            let s = d.at_phase(t).unwrap();
            assert!((s.elapsed_s - t / 10.).abs() < 1e-15);
            assert!((s.closing_m_s - 3. * t.cos()).abs() < 1e-14);
            assert!((s.normal_force_n - 60. * t.sin()).abs() < 1e-12);
            assert_eq!(s.heat_j, 0.);
        }
    }
    #[test]
    fn release_round_trips_independently_fitted_restitution_and_stiffness() {
        for alpha in [1e-6, 0.02, 0.3, 2., 30., 1000.] {
            let d = FlatLanding::new(3., 54000., alpha, 0.85).unwrap();
            let out = d.at_phase(PI).unwrap();
            let fitted =
                crate::DampedAreaFoundation::coefficient_from_flat_impact(0.85, -out.closing_m_s)
                    .unwrap();
            assert!((fitted - alpha).abs() < 3e-9 * alpha.max(1.));
            let peak = d.at_phase(FRAC_PI_2).unwrap();
            assert!(peak.elapsed_s < out.elapsed_s);
            assert!(
                (FlatLanding::stiffness_from_depth(3., alpha, 0.85, peak.compression_m).unwrap()
                    - 54000.)
                    .abs()
                    < 1e-9
            );
        }
    }
    #[test]
    fn phase_integrals_obey_both_equations_of_motion() {
        let d = FlatLanding::new(2.1, 3200., 1.7, 1.6).unwrap();
        for i in 1..30 {
            let t = PI * f64::from(i) / 30.;
            let s = d.at_phase(t).unwrap();
            let l = d.at_phase(t - 1e-5).unwrap();
            let r = d.at_phase(t + 1e-5).unwrap();
            let dt = r.elapsed_s - l.elapsed_s;
            assert!(((r.compression_m - l.compression_m) / dt - s.closing_m_s).abs() < 1e-8);
            assert!(((r.impulse_ns - l.impulse_ns) / dt - s.normal_force_n).abs() < 2e-7);
            assert!(l.heat_j <= r.heat_j + 1e-14);
        }
    }
}
