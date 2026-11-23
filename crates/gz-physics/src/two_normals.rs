// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Unilateral two-point frictionless impact or sustained normal reactions.
use crate::{PhysicsError, point_impact::mul_div};

/// An SPD Delassus matrix and the two free normal velocities (impact) or
/// accelerations (sustained contact). Positive free values point apart.
#[derive(Clone, Copy, Debug)]
pub struct TwoNormalContact {
    pub inverse_mass: [[f64; 2]; 2],
    pub free_normal: [f64; 2],
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NormalSiteState {
    Loaded,
    Separating,
    /// Load and residual are both at the numerical active-set boundary.
    Neutral,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TwoNormalResponse {
    /// Compressive normal impulses (N·s) or forces (N), according to input.
    pub load: [f64; 2],
    /// Resulting normal velocities or accelerations.
    pub normal: [f64; 2],
    pub state: [NormalSiteState; 2],
}
impl TwoNormalContact {
    /// Solve j ≥ 0, v + Wj ≥ 0, jᵢ(v + Wj)ᵢ = 0. Strict positive
    /// definiteness makes the site loads and the generalized change unique.
    pub fn solve(self) -> Result<TwoNormalResponse, PhysicsError> {
        const EPS: f64 = 128. * f64::EPSILON;
        let m = self.inverse_mass;
        let u = self.free_normal;
        if m.iter().flatten().chain(&u).any(|x| !x.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        let ms = m.iter().flatten().map(|x| x.abs()).fold(0., f64::max);
        if ms == 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let w = m.map(|r| r.map(|x| x / ms));
        if (w[0][1] - w[1][0]).abs() > EPS || w[0][0] <= 0. || w[1][1] <= 0. {
            return Err(PhysicsError::InvalidValue);
        }
        let c = 0.5 * w[0][1] + 0.5 * w[1][0];
        let d = w[0][0].mul_add(w[1][1], -c * c);
        if d < -EPS {
            return Err(PhysicsError::InvalidValue);
        }
        if d <= EPS {
            return Err(PhysicsError::Unsupported {
                engine: "multifidelity",
                capability: "unique resolved two-site normal loads",
            });
        }
        let vs = u[0].abs().max(u[1].abs());
        if vs == 0. {
            return Ok(TwoNormalResponse {
                load: [0.; 2],
                normal: [0.; 2],
                state: [NormalSiteState::Neutral; 2],
            });
        }
        let f = u.map(|x| x / vs);
        let mut reaction = [0.; 2];
        let mut residual = f;
        if f[0] < 0. || f[1] < 0. {
            let mut single = None;
            for i in 0..2 {
                if f[i] >= 0. {
                    continue;
                }
                let k = 1 - i;
                let j = -f[i] / w[i][i];
                let other = c.mul_add(j, f[k]);
                if other >= 0. {
                    single = Some((i, j, other));
                    break;
                }
            }
            if let Some((i, j, r)) = single {
                reaction[i] = j;
                residual[i] = 0.;
                residual[1 - i] = r;
            } else {
                reaction = [
                    c.mul_add(f[1], -w[1][1] * f[0]) / d,
                    c.mul_add(f[0], -w[0][0] * f[1]) / d,
                ];
                if reaction.iter().any(|v| *v < 0. || !v.is_finite()) {
                    return Err(PhysicsError::InvalidValue);
                }
                residual = [0.; 2];
            }
        }
        let state = std::array::from_fn(|i| {
            let kick = w[i][i] * reaction[i];
            let scale = f[i].abs() + kick.abs() + (c * reaction[1 - i]).abs();
            if kick > EPS * scale {
                NormalSiteState::Loaded
            } else if residual[i] > EPS * scale {
                NormalSiteState::Separating
            } else {
                NormalSiteState::Neutral
            }
        });
        let load = reaction.map(|j| mul_div(j, vs, ms));
        let normal = residual.map(|r| r * vs);
        if load.iter().chain(&normal).any(|x| !x.is_finite())
            || (0..2).any(|i| {
                (reaction[i] > 0. && load[i] == 0.) || (residual[i] > 0. && normal[i] == 0.)
            })
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(TwoNormalResponse {
            load,
            normal,
            state,
        })
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn supported_single_double_and_free_modes_are_invariant_to_scales_and_order() {
        for c in [-0.65, 0., 0.65] {
            let w = [[1.4, c], [c, 0.9]];
            for (j, v) in [
                ([0., 0.], [0.4, 1.]),
                ([1.2, 0.], [0., 0.72]),
                ([0., 0.7], [0.2, 0.]),
                ([0.7, 1.1], [0., 0.]),
                ([0., 0.], [0., 0.]),
            ] {
                let free =
                    std::array::from_fn::<_, 2, _>(|i| v[i] - w[i][0] * j[0] - w[i][1] * j[1]);
                for p in [[0, 1], [1, 0]] {
                    for m in [1e-80, 1., 1e80] {
                        for speed in [1e-70, 1., 1e70] {
                            let input = TwoNormalContact {
                                inverse_mass: p.map(|i| p.map(|k| w[i][k] / m)),
                                free_normal: p.map(|i| free[i] * speed),
                            };
                            let h = input.solve().unwrap();
                            for (i, k) in p.into_iter().enumerate() {
                                assert!((h.load[i] / m / speed - j[k]).abs() < 3e-15);
                                assert!((h.normal[i] / speed - v[k]).abs() < 3e-15);
                                assert_eq!(
                                    h.state[i],
                                    if j[k] > 0. {
                                        NormalSiteState::Loaded
                                    } else if v[k] > 0. {
                                        NormalSiteState::Separating
                                    } else {
                                        NormalSiteState::Neutral
                                    }
                                );
                            }
                        }
                    }
                }
            }
        }
    }
    #[test]
    fn distinguishes_new_impact_against_a_bearing_site_and_declines_nonidentifiable_normals() {
        for c in [-0.5, 0.5] {
            let h = TwoNormalContact {
                inverse_mass: [[1., c], [c, 2.]],
                free_normal: [0., -1.],
            }
            .solve()
            .unwrap();
            assert!(h.load[1] > 0. && h.normal[1] == 0.);
            if c > 0. {
                assert_eq!(h.load, [0., 0.5]);
                assert_eq!(h.normal[0], 0.25);
            } else {
                assert!(h.load[0] > 0. && h.normal[0] == 0.);
            }
        }
        let rank_one = TwoNormalContact {
            inverse_mass: [[1.; 2]; 2],
            free_normal: [-1.; 2],
        };
        assert!(matches!(
            rank_one.solve(),
            Err(PhysicsError::Unsupported { .. })
        ));
        for matrix in [
            [[1., 0.5], [0.2, 1.]],
            [[1., 2.], [2., 1.]],
            [[f64::NAN, 0.], [0., 1.]],
        ] {
            assert!(matches!(
                TwoNormalContact {
                    inverse_mass: matrix,
                    ..rank_one
                }
                .solve(),
                Err(PhysicsError::InvalidValue)
            ));
        }
    }
}
