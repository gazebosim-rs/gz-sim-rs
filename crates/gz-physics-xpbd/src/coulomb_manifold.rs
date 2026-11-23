// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Dense coupled Signorini–Coulomb velocity impulse at a frozen contact configuration.
use glam::DVec2;

pub(crate) fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Fixed points are precisely the Signorini conditions and maximum-dissipation
/// Coulomb disk in each contact plane. In particular sliding aligns the *final*
/// tangential velocity with the opposing impulse. Equality at zero slip admits
/// the whole closed disk. The scale per row converts velocity into impulse.
fn evaluate(
    k: &[Vec<f64>],
    free: &[f64],
    x: &[f64],
    mu: f64,
    derivatives: bool,
    beta: f64,
) -> (Vec<f64>, Vec<Vec<f64>>) {
    let d = x.len();
    let mut f = vec![0.; d];
    let mut jac = if derivatives {
        vec![vec![0.; d]; d]
    } else {
        vec![]
    };
    for i in (0..d).step_by(3) {
        let n = i;
        let u = i + 1;
        let v = i + 2;
        let sn = beta / k[n][n];
        let st = 2. * beta / (k[u][u] + k[v][v]);
        let velocity = |i: usize| free[i] + dot(&k[i], x);
        let trial_n = x[n] - sn * velocity(n);
        f[n] = x[n] - trial_n.max(0.);
        let z = DVec2::new(x[u] - st * velocity(u), x[v] - st * velocity(v));
        let length = z.x.hypot(z.y);
        let cap = mu * x[n].max(0.);
        let unit = if length > 0. { z / length } else { DVec2::ZERO };
        let sliding = length > cap;
        let projection = if sliding { cap * unit } else { z };
        f[u] = x[u] - projection.x;
        f[v] = x[v] - projection.y;
        if derivatives {
            for j in 0..d {
                let en = f64::from(j == n);
                let eu = f64::from(j == u);
                let ev = f64::from(j == v);
                jac[n][j] = if trial_n > 0. { sn * k[n][j] } else { en };
                let dz = DVec2::new(eu - st * k[u][j], ev - st * k[v][j]);
                let dp = if sliding {
                    cap / length * (dz - unit * unit.dot(dz))
                        + if x[n] > 0. {
                            mu * en * unit
                        } else {
                            DVec2::ZERO
                        }
                } else {
                    dz
                };
                jac[u][j] = eu - dp.x;
                jac[v][j] = ev - dp.y;
            }
        }
    }
    (f, jac)
}

/// Row-pivoted elimination on the dimensionless projection Jacobian. Singular
/// directions (e.g. coincident material witnesses) use the damped step below.
fn linear(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for i in 0..n {
        let pivot = (i..n).max_by(|&j, &l| a[j][i].abs().total_cmp(&a[l][i].abs()))?;
        a.swap(i, pivot);
        b.swap(i, pivot);
        let diagonal = a[i][i];
        if !diagonal.is_finite() || diagonal.abs() < 1e-14 {
            return None;
        }
        for j in i + 1..n {
            let (previous, next) = a.split_at_mut(j);
            let row = &mut next[0];
            let m = row[i] / diagonal;
            for (entry, pivot) in row[i + 1..].iter_mut().zip(&previous[i][i + 1..]) {
                *entry -= m * pivot;
            }
            b[j] -= m * b[i];
        }
    }
    for i in (0..n).rev() {
        b[i] = (b[i] - dot(&a[i][i + 1..], &b[i + 1..])) / a[i][i];
    }
    b.iter().all(|n| n.is_finite()).then_some(b)
}

fn single(k: &[Vec<f64>], free: &[f64], seed: &[f64], mu: f64, beta: f64) -> Option<Vec<f64>> {
    let d = seed.len();
    if (0..d).any(|i| !k[i][i].is_finite() || k[i][i] <= 0.) {
        return None;
    }
    let mut x = seed.to_vec();
    for _ in 0..48 {
        let (f, jac) = evaluate(k, free, &x, mu, true, beta);
        let mag = f.iter().map(|x| x.abs()).fold(0., f64::max);
        let scale = x.iter().map(|x| x.abs()).fold(0., f64::max);
        if mag <= beta.min(1.) * (1e-16 + 4e-14 * scale) {
            // Remove at most the final roundoff in the cone inequality and
            // recheck complementarity at the exact feasible published values.
            for j in x.chunks_exact_mut(3) {
                j[0] = j[0].max(0.);
                let length = j[1].hypot(j[2]);
                if length > mu * j[0] {
                    let shrink = mu * j[0] / length;
                    j[1] *= shrink;
                    j[2] *= shrink;
                }
            }
            return evaluate(k, free, &x, mu, false, 1.)
                .0
                .iter()
                .all(|r| r.is_finite() && r.abs() <= 3e-16 + 2e-13 * scale)
                .then_some(x);
        }
        let old = dot(&f, &f);
        let negative: Vec<_> = f.iter().map(|f| -f).collect();
        let mut improved = false;
        for damping in [0., 1e-10, 1e-6, 1e-2] {
            let delta = if damping == 0. {
                linear(jac.clone(), negative.clone())
            } else {
                let mut a = vec![vec![0.; d]; d];
                let mut b = vec![0.; d];
                for i in 0..d {
                    b[i] = jac.iter().zip(&negative).map(|(r, v)| r[i] * v).sum();
                    for j in 0..=i {
                        a[i][j] = jac.iter().map(|r| r[i] * r[j]).sum();
                        a[j][i] = a[i][j]
                    }
                    a[i][i] += damping;
                }
                linear(a, b)
            };
            let Some(delta) = delta else { continue };
            let mut fraction = 1.;
            for _ in 0..20 {
                let candidate: Vec<_> = x
                    .iter()
                    .zip(&delta)
                    .map(|(v, h)| v + fraction * h)
                    .collect();
                let next = evaluate(k, free, &candidate, mu, false, beta).0;
                if dot(&next, &next) < old * (1. - 1e-4 * fraction) {
                    x = candidate;
                    improved = true;
                    break;
                }
                fraction *= 0.5;
            }
            if improved {
                break;
            }
        }
        if !improved {
            return None;
        }
    }
    None
}

pub(crate) fn solve(k: &[Vec<f64>], free: &[f64], seed: &[f64], mu: f64) -> Option<Vec<f64>> {
    single(k, free, seed, mu, 1.)
}
/// A positive proximal step changes the numerical map but preserves the normal
/// complementarity and Coulomb disk fixed points. Accept every candidate on the
/// original unit-step residual after projecting rounding error back onto the cone.
pub(crate) fn recover(
    k: &[Vec<f64>],
    free: &[f64],
    seed: &[f64],
    mu: f64,
    retry: bool,
) -> Option<Vec<f64>> {
    if let Some(j) = solve(k, free, seed, mu) {
        return Some(j);
    }
    if !retry {
        return None;
    }
    let zero = vec![0.; seed.len()];
    for beta in [8., 0.2, 32., 128., 0.04, 0.008, 1.] {
        for start in [seed, &zero] {
            if let Some(j) = single(k, free, start, mu, beta) {
                return Some(j);
            }
        }
    }
    // Strongly coupled rows can make the Newton active-set singular. A
    // continuation from independent contacts supplies guesses only. Return a
    // result solely from the original fully coupled system.
    for halving_steps in [4, 12] {
        let mut guess = zero.clone();
        for index in 0..=36 {
            let coupling = 1. - 0.5_f64.powf(f64::from(index) / f64::from(halving_steps));
            let matrix = k
                .iter()
                .enumerate()
                .map(|(i, row)| {
                    row.iter()
                        .enumerate()
                        .map(|(j, v)| if i / 3 == j / 3 { *v } else { coupling * v })
                        .collect()
                })
                .collect::<Vec<_>>();
            guess = match single(&matrix, free, &guess, mu, 1.) {
                Some(value) => value,
                None => break,
            };
            if index % 3 == 0 {
                for beta in [1., 8.] {
                    if let Some(j) = single(k, free, &guess, mu, beta) {
                        return Some(j);
                    }
                }
            }
        }
    }
    None
}
#[cfg(test)]
mod tests {
    use super::solve;
    #[test]
    fn simultaneous_sticking_sliding_and_separating_rows_match_manufactured_coulomb_solution() {
        let d = 12;
        // Almost dependent contact directions force a long tail in block iteration.
        let axis: Vec<_> = (0..d).map(|i| ((i * 13 + 3) as f64).sin()).collect();
        let k: Vec<Vec<_>> = (0..d)
            .map(|i| {
                (0..d)
                    .map(|j| 300. * axis[i] * axis[j] + if i == j { 0.5 } else { 0. })
                    .collect()
            })
            .collect();
        let j = [0.8, 0.1, -0.15, 1., 0.3, -0.4, 0.4, -0.12, 0.16, 0., 0., 0.];
        let speed = [0., 0., 0., 0., -0.09, 0.12, 0., 0.3, -0.4, 0.18, 0.35, -0.3];
        let free: Vec<_> = k
            .iter()
            .zip(speed)
            .map(|(r, v)| v - super::dot(r, &j))
            .collect();
        let seed: Vec<_> = j
            .iter()
            .enumerate()
            .map(|(i, v)| v + 0.0002 * ((i as f64).cos()))
            .collect();
        let result = solve(&k, &free, &seed, 0.5).unwrap();
        for (a, b) in result.iter().zip(j) {
            assert!((a - b).abs() < 3e-12, "{result:?}");
        }
    }
}
