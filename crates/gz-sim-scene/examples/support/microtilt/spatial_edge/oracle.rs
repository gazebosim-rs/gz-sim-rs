// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A forward two-point, plastic Coulomb impact on a common normal.
//! The contraction proof is sufficient and deliberately declines other regimes.
use glam::{DMat2, DMat3, DVec2, DVec3};

#[derive(Clone, Copy)]
pub(super) struct Edge {
    pub mass: f64,
    pub inertia: DMat3,
    pub arm: [DVec3; 2],
    pub velocity: DVec3,
    pub spin: DVec3,
    pub friction: [f64; 2],
}
pub(super) struct Impact {
    pub normal: [f64; 2],
    pub impulse: [DVec3; 2],
    pub velocity: DVec3,
    pub spin: DVec3,
    pub point_velocity: [DVec3; 2],
    pub dissipated: f64,
    pub contraction: f64,
    pub slip_floor: [f64; 2],
    pub singleton_opening_upper: [f64; 2],
    pub impulse_error_bound: f64,
    pub iterations: usize,
}
#[derive(Debug, PartialEq, Eq)]
pub(super) enum Decline {
    Invalid,
    NotClosing,
    SingleSitePossible,
    Unbounded,
    SlipUncertain,
    NoContraction,
}

fn block_norm(m: DMat2) -> f64 {
    // Largest singular value, including nonsymmetric off-diagonal blocks.
    let a = m.x_axis.length_squared();
    let b = m.y_axis.length_squared();
    let c = m.x_axis.dot(m.y_axis);
    ((a + b + (a - b).hypot(2. * c)) * 0.5).sqrt()
}
impl Edge {
    pub fn point(&self, i: usize) -> DVec3 {
        self.velocity + self.spin.cross(self.arm[i])
    }
    pub fn mobility(&self, i: usize, j: usize, impulse: DVec3) -> DVec3 {
        impulse / self.mass
            + (self.inertia.inverse() * self.arm[j].cross(impulse)).cross(self.arm[i])
    }
    pub fn solve(&self) -> Result<Impact, Decline> {
        if !(self.mass.is_finite() && self.mass > 0.)
            || !self.inertia.is_finite()
            || self.friction.iter().any(|m| !m.is_finite() || *m < 0.)
            || !(self.velocity.is_finite()
                && self.spin.is_finite()
                && self.arm.iter().all(|r| r.is_finite()))
        {
            return Err(Decline::Invalid);
        }
        let tx = self.inertia.x_axis;
        let ty = self.inertia.y_axis;
        if (self.inertia - self.inertia.transpose())
            .to_cols_array()
            .into_iter()
            .any(|x| x.abs() > 1e-14)
            || tx.x <= 0.
            || tx.x * ty.y <= tx.y * tx.y
            || self.inertia.determinant() <= 0.
        {
            return Err(Decline::Invalid);
        }
        let before = [self.point(0), self.point(1)];
        if before.iter().any(|u| u.z >= 0.) {
            return Err(Decline::NotClosing);
        }
        let dz = |i, j| self.mobility(i, j, DVec3::Z);
        let dx = |i, j| self.mobility(i, j, DVec3::X);
        let dy = |i, j| self.mobility(i, j, DVec3::Y);
        let nt = |i, j| DVec2::new(dx(i, j).z, dy(i, j).z);
        let nn = DMat2::from_cols(
            DVec2::new(dz(0, 0).z, dz(1, 0).z),
            DVec2::new(dz(0, 1).z, dz(1, 1).z),
        );
        if nn.determinant() <= 64. * f64::EPSILON * nn.x_axis.length() * nn.y_axis.length() {
            return Err(Decline::Invalid);
        }
        let mut singleton = [0.; 2];
        // Exclude every one-loaded-site solution, allowing any direction in its cone.
        for i in 0..2 {
            let j = 1 - i;
            let low = dz(i, i).z - self.friction[i] * nt(i, i).length();
            if low <= 0. {
                return Err(Decline::SingleSitePossible);
            }
            let other = dz(j, i).z + self.friction[i] * nt(j, i).length();
            singleton[j] = before[j].z + other.max(0.) * (-before[i].z / low);
            if singleton[j] >= 0. {
                return Err(Decline::SingleSitePossible);
            }
        }
        // With both normals closed, n = n0 - A t and u_t+ = q + S t.
        let inv = nn.inverse();
        let n0 = inv * DVec2::new(-before[0].z, -before[1].z);
        let a: [[DVec2; 2]; 2] = std::array::from_fn(|i| {
            std::array::from_fn(|j| inv.x_axis[i] * nt(0, j) + inv.y_axis[i] * nt(1, j))
        });
        let q: [DVec2; 2] = std::array::from_fn(|i| {
            before[i].truncate() + dz(i, 0).truncate() * n0.x + dz(i, 1).truncate() * n0.y
        });
        let schur: [[DMat2; 2]; 2] = std::array::from_fn(|i| {
            std::array::from_fn(|j| {
                let v = |axis: usize| {
                    [dx(i, j), dy(i, j)][axis].truncate()
                        - dz(i, 0).truncate() * a[0][j][axis]
                        - dz(i, 1).truncate() * a[1][j][axis]
                };
                DMat2::from_cols(v(0), v(1))
            })
        });
        let ar = a.map(|row| row[0].length() + row[1].length());
        let sr = schur.map(|row| block_norm(row[0]) + block_norm(row[1]));
        let beta = (self.friction[0] * ar[0]).max(self.friction[1] * ar[1]);
        if beta >= 1. {
            return Err(Decline::Unbounded);
        }
        let bound =
            (self.friction[0] * n0.x.abs()).max(self.friction[1] * n0.y.abs()) / (1. - beta);
        let floor: [f64; 2] = std::array::from_fn(|i| q[i].length() - sr[i] * bound);
        if (0..2).any(|i| floor[i] <= 0. || n0[i] - ar[i] * bound <= 0.) {
            return Err(Decline::SlipUncertain);
        }
        let lipschitz = (0..2)
            .map(|i| self.friction[i] * (ar[i] + (n0[i] + ar[i] * bound) * sr[i] / floor[i]))
            .fold(0., f64::max);
        if !lipschitz.is_finite() || lipschitz >= 1. {
            return Err(Decline::NoContraction);
        }
        let normal = |t: [DVec2; 2]| {
            [
                n0.x - a[0][0].dot(t[0]) - a[0][1].dot(t[1]),
                n0.y - a[1][0].dot(t[0]) - a[1][1].dot(t[1]),
            ]
        };
        let mut t = [DVec2::ZERO; 2];
        for iterations in 1..=4096 {
            let n = normal(t);
            let next = std::array::from_fn(|i| {
                let v = q[i] + schur[i][0] * t[0] + schur[i][1] * t[1];
                -self.friction[i] * n[i] * v.normalize()
            });
            let step = t[0].distance(next[0]).max(t[1].distance(next[1]));
            // A posteriori Banach bound on the next tangent iterate, then lift
            // by the linear normal recovery. Roundoff is checked independently.
            let error =
                step * lipschitz / (1. - lipschitz) * (1. + ar[0].max(ar[1]).powi(2)).sqrt();
            t = next;
            if error > 2e-14 * (1. + bound) {
                continue;
            }
            let n = normal(t);
            let impulse = [t[0].extend(n[0]), t[1].extend(n[1])];
            let dv = (impulse[0] + impulse[1]) / self.mass;
            let dw = self.inertia.inverse()
                * (self.arm[0].cross(impulse[0]) + self.arm[1].cross(impulse[1]));
            let velocity = self.velocity + dv;
            let spin = self.spin + dw;
            let point_velocity = std::array::from_fn(|i| velocity + spin.cross(self.arm[i]));
            let dissipated = -(0..2)
                .map(|i| 0.5 * impulse[i].dot(before[i] + point_velocity[i]))
                .sum::<f64>();
            return Ok(Impact {
                normal: n,
                impulse,
                velocity,
                spin,
                point_velocity,
                dissipated,
                contraction: lipschitz,
                slip_floor: floor,
                singleton_opening_upper: singleton,
                impulse_error_bound: error,
                iterations,
            });
        }
        Err(Decline::NoContraction)
    }
}
