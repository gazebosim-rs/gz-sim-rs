// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
pub(super) type Key = (usize, [usize; 3]);

/// Complete the normal velocity solve simultaneously. The positional stage
/// has already contributed a known impulse, so only its difference from the
/// nonnegative final impulse is committed here.
pub(super) fn resolve(
    contacts: &BTreeMap<Key, Contact>,
    x: &[DVec3],
    p: &mut [Particle],
    pads: &mut [PreparedBoundary],
    reactions: &mut [BoundaryImpulse],
    h: f64,
    skin: f64,
) -> Result<BTreeMap<Key, f64>, XpbdError> {
    let rows = contacts
        .iter()
        .filter(|((k, _), c)| {
            if c.normal_impulse > 0. {
                return true;
            }
            let b = &pads[*k];
            let r = b.receiver.as_ref().unwrap();
            let center = b.origin + b.velocity * r.end_time + r.end_offset;
            let [a, bb, cc] = c.ids.map(|i| x[i]);
            let gap = skin + b.sphere_radius().unwrap();
            vertex_face(&[center, a, bb, cc]).delta.length() <= gap * (1. + 1e-7)
        })
        .collect::<Vec<_>>();
    let n = rows.len();
    if n == 0 {
        return Ok(BTreeMap::new());
    }
    let mut k = vec![vec![0.; n]; n];
    let mut free = Vec::with_capacity(n);
    for (i, &(&(pad, _), c)) in rows.iter().enumerate() {
        let b = &pads[pad];
        let r = b.receiver.as_ref().unwrap();
        let v = blend(c.weights, c.ids.map(|i| p[i].velocity.into()));
        let vn = (v - b.velocity - r.linear).dot(c.normal);
        // Depenetration at rest introduces no rebound.
        let target = (vn - c.overlap / h).max(0.);
        free.push(vn - target);
        for (j, &(&(other, _), d)) in rows[..=i].iter().enumerate() {
            let mut mass = if pad == other { r.inverse_mass } else { 0. };
            for (id, a) in c.ids.into_iter().zip(c.weights) {
                if let Some(m) = d.ids.iter().position(|&v| v == id) {
                    mass += p[id].inverse_mass * a * d.weights[m];
                }
            }
            let v = mass * c.normal.dot(d.normal);
            k[i][j] = v;
            k[j][i] = v;
        }
    }
    let projected: Vec<_> = rows.iter().map(|(_, c)| c.normal_impulse).collect();
    for (v, row) in free.iter_mut().zip(&k) {
        *v -= row.iter().zip(&projected).map(|(a, b)| a * b).sum::<f64>();
    }
    let total = if n <= 8 {
        super::super::impact_group::inelastic(&k, &free, n)
    } else {
        iterative(&k, &free, &projected)
    }
    .ok_or(XpbdError::UnresolvedSurface)?;
    let mut result = BTreeMap::new();
    for ((&key, c), total) in rows.into_iter().zip(total) {
        let b = &mut pads[key.0];
        let r = b.receiver.as_mut().unwrap();
        let center = b.origin + b.velocity * r.end_time + r.end_offset;
        let j = (total - c.normal_impulse) * c.normal;
        for (id, w) in c.ids.into_iter().zip(c.weights) {
            p[id].velocity = (DVec3::from(p[id].velocity) + w * p[id].inverse_mass * j).to_array();
        }
        let q = blend(c.weights, c.ids.map(|i| x[i]));
        r.apply_velocity_impulse(q - center, -j);
        reactions[key.0].react(q - b.origin, j)?;
        result.insert(key, total);
    }
    Ok(result)
}

/// Projected coordinate descent for larger manifolds. Solve the most
/// strongly opposed pair as a 2x2 block when its interior solution is
/// admissible. Every accepted result satisfies normal complementarity.
pub(crate) fn iterative(k: &[Vec<f64>], v: &[f64], seed: &[f64]) -> Option<Vec<f64>> {
    let n = v.len();
    let mut j = seed.to_vec();
    let mut r: Vec<_> = k
        .iter()
        .zip(v)
        .map(|(row, v)| v + row.iter().zip(&j).map(|(a, b)| a * b).sum::<f64>())
        .collect();
    let scale = v.iter().chain(r.iter()).map(|v| v.abs()).fold(1., f64::max);
    for _ in 0..128 * n {
        let error = |i: usize| {
            if j[i] > 0. {
                r[i].abs()
            } else {
                (-r[i]).max(0.)
            }
        };
        let i = (0..n).max_by(|&i, &l| error(i).total_cmp(&error(l)))?;
        if error(i) <= 1e-9 * scale {
            return Some(j);
        }
        let aa = k[i][i];
        let mut a = (j[i] - r[i] / aa).max(0.) - j[i];
        let mut pair = None;
        let mut strength = 0.;
        for l in 0..n {
            let cross = k[l][i];
            let bb = k[l][l];
            if cross >= 0. || l == i {
                continue;
            }
            let correlation = cross * cross / (aa * bb);
            if correlation <= strength || correlation >= 1. - 1e-12 {
                continue;
            }
            let det = aa * bb - cross * cross;
            let ai = (-bb * r[i] + cross * r[l]) / det;
            let al = (-aa * r[l] + cross * r[i]) / det;
            if j[i] + ai >= 0. && j[l] + al >= 0. {
                strength = correlation;
                a = ai;
                pair = Some((l, al));
            }
        }
        j[i] += a;
        if let Some((l, b)) = pair {
            j[l] += b;
        }
        for (m, row) in k.iter().enumerate() {
            r[m] += row[i] * a + pair.map_or(0., |(l, b)| row[l] * b);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::iterative;

    #[test]
    fn opposed_and_separating_rows_on_a_larger_manifold() {
        let n = 10;
        // SPD Gram matrix of two sets of opposed rows on five disjoint
        // membrane patches. The sphere terms retain nonzero inertia.
        let mut k = vec![vec![0.; n]; n];
        for i in 0..n {
            k[i][i] = 2.;
            k[i][i ^ 1] = -1.9;
        }
        let expected = [0., 1., 2., 0., 0.3, 1.2, 0., 0., 1.9, 1.4];
        let v: Vec<_> = k
            .iter()
            .enumerate()
            .map(|(i, row)| {
                let speed = if expected[i] == 0. { 0.2 } else { 0. };
                speed - row.iter().zip(expected).map(|(a, b)| a * b).sum::<f64>()
            })
            .collect();
        let result = iterative(&k, &v, &[0.2; 10]).unwrap();
        assert!(
            result
                .iter()
                .zip(expected)
                .all(|(j, e)| (j - e).abs() < 1e-7)
        );
    }
}
