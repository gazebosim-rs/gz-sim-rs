// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A six-equality master problem for Coulomb traction columns. The caller
//! independently validates primal wrenches and prices the entire circular cone.
use super::*;
const ROWS: usize = 6;
const PIVOT: f64 = 2e-13;
const OPT: f64 = 3e-12;
struct Tableau {
    rows: [Vec<f64>; ROWS],
    basic: [usize; ROWS],
    signs: [f64; ROWS],
    real: usize,
}
pub(super) enum LinearSolution {
    Optimal {
        weights: Vec<(usize, f64)>,
        dual: Wrench,
    },
    Infeasible {
        dual: Wrench,
    },
}
impl Tableau {
    fn new(columns: &[Column], rhs: Wrench) -> Self {
        let real = columns.len();
        let signs = rhs.map(|v| if v < 0. { -1. } else { 1. });
        let rows = std::array::from_fn(|i| {
            let mut row = vec![0.; real + ROWS + 1];
            row[0] = rhs[i].abs();
            for (j, c) in columns.iter().enumerate() {
                row[1 + j] = signs[i] * c.w[i];
            }
            row[1 + real + i] = 1.;
            row
        });
        Self {
            rows,
            basic: std::array::from_fn(|i| real + i),
            signs,
            real,
        }
    }
    fn pivot(&mut self, leave: usize, enter: usize) -> bool {
        let row = &mut self.rows[leave];
        let col = 1 + enter;
        let pivot = row[col];
        if !pivot.is_finite() || pivot.abs() < PIVOT {
            return false;
        }
        for v in row.iter_mut() {
            *v /= pivot;
        }
        row[col] = 1.;
        let base = row.clone();
        for (i, r) in self.rows.iter_mut().enumerate() {
            if i == leave {
                continue;
            }
            let mult = r[col];
            if mult != 0. {
                for (x, v) in r.iter_mut().zip(&base) {
                    *x -= mult * v;
                }
            }
            r[col] = 0.;
        }
        self.basic[leave] = enter;
        self.rows
            .iter()
            .all(|r| r[0] >= -2e-9 && r.iter().all(|x| x.is_finite()))
    }
    fn optimize(&mut self, cost: &[f64], active: usize) -> bool {
        for _ in 0..4000 {
            // Bland's deterministic entering/leaving order handles degenerate
            // pressure bases on exactly rectangular and reflected fixtures.
            let enter = (0..active).find(|&j| {
                !self.basic.contains(&j)
                    && cost[j]
                        - self
                            .rows
                            .iter()
                            .zip(self.basic)
                            .map(|(r, b)| cost[b] * r[1 + j])
                            .sum::<f64>()
                        > OPT
            });
            let Some(j) = enter else {
                return true;
            };
            let leave = (0..ROWS)
                .filter(|&i| self.rows[i][1 + j] > PIVOT)
                .min_by(|&a, &b| {
                    let ratio = |i: usize| self.rows[i][0].max(0.) / self.rows[i][1 + j];
                    let x = ratio(a);
                    let y = ratio(b);
                    if (x - y).abs() <= 128. * f64::EPSILON * (1. + x.abs().min(y.abs())) {
                        self.basic[a].cmp(&self.basic[b])
                    } else {
                        x.total_cmp(&y)
                    }
                });
            if !leave.is_some_and(|i| self.pivot(i, j)) {
                return false;
            }
        }
        false
    }
    fn dual(&self, cost: &[f64]) -> Wrench {
        std::array::from_fn(|i| {
            self.signs[i]
                * self
                    .rows
                    .iter()
                    .zip(self.basic)
                    .map(|(r, b)| cost[b] * r[1 + self.real + i])
                    .sum::<f64>()
        })
    }
}
pub(super) fn solve(columns: &[Column], rhs: Wrench, sign: f64) -> Option<LinearSolution> {
    let n = columns.len();
    let mut t = Tableau::new(columns, rhs);
    let mut cost = vec![0.; n + ROWS];
    cost[n..].fill(-1.);
    if !t.optimize(&cost, n + ROWS) {
        return None;
    }
    let artificial = t
        .rows
        .iter()
        .zip(t.basic)
        .filter(|(_, j)| *j >= n)
        .map(|(r, _)| r[0].max(0.))
        .sum::<f64>();
    if artificial > 5e-11 {
        return Some(LinearSolution::Infeasible {
            dual: t.dual(&cost),
        });
    }
    for i in 0..ROWS {
        if t.basic[i] < n {
            continue;
        }
        let j = (0..n)
            .filter(|j| !t.basic.contains(j))
            .max_by(|&a, &b| t.rows[i][1 + a].abs().total_cmp(&t.rows[i][1 + b].abs()));
        if let Some(j) = j
            && t.rows[i][1 + j].abs() > 1e-10
            && !t.pivot(i, j)
        {
            return None;
        }
    }
    cost[n..].fill(0.);
    for (v, c) in cost.iter_mut().zip(columns) {
        *v = if c.site == 1 { sign } else { 0. };
    }
    if !t.optimize(&cost, n) {
        return None;
    }
    let weights = t
        .rows
        .iter()
        .zip(t.basic)
        .filter(|(r, j)| *j < n && r[0] > 0.)
        .map(|(r, j)| (j, r[0]))
        .collect();
    Some(LinearSolution::Optimal {
        weights,
        dual: t.dual(&cost),
    })
}
