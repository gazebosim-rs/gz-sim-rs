// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/articulated_mobility.rs"]
mod fixture;
#[path = "support/gripper_cartesian.rs"]
mod independent;
use fixture::*;
use glam::DVec3;
use gz_sim_scene::deformable;
use rapier3d_f64::na::{DMatrix, DVector};
const WORLD_F: &str = include_str!("../../../examples/articulated_friction_impact.sdf");
const N: DVec3 = DVec3::new(0.28, 0., 0.96);
const U: DVec3 = DVec3::new(0.96, 0., -0.28);
const V: DVec3 = DVec3::Y;
const AXIS: [DVec3; 3] = [N, U, V];
const VELOCITY: DVec3 = DVec3::new(0.400, 0.12, -0.950);
fn sample(g: &Gripper) -> &deformable::DeformableSample {
    deformable::bodies(&g.sim).next().unwrap().1
}

/// Independent least-squares fitting of six Cartesian twists to the observed
/// moving triangle, followed by velocity at the offset material skin.
fn soft_jacobian(g: &Gripper) -> DMatrix<f64> {
    let p = sample(g)
        .particles()
        .iter()
        .map(|p| DVec3::from(p.position))
        .collect::<Vec<_>>();
    let world = [DVec3::X, DVec3::Y, DVec3::Z];
    let mut out = DMatrix::zeros(6, 12);
    for (s, ids) in [[0, 2, 3], [0, 1, 3]].into_iter().enumerate() {
        let b = 0.5 + if s == 0 { 0.29 / 0.98 } else { -0.29 / 0.98 };
        let weights = if s == 0 {
            [1. - b, b - 0.5, 0.5]
        } else {
            [0.5, 0.5 - b, b]
        };
        let center = ids.iter().map(|&i| p[i]).sum::<DVec3>() / 3.;
        let mut fit = DMatrix::<f64>::zeros(9, 6);
        for (z, &node) in ids.iter().enumerate() {
            for (j, d) in world.into_iter().enumerate() {
                fit[(3 * z + j, j)] = 1.;
                let spin = d.cross(p[node] - center);
                for l in 0..3 {
                    fit[(3 * z + l, 3 + j)] = spin[l];
                }
            }
        }
        let inverse = (fit.transpose() * &fit)
            .cholesky()
            .unwrap()
            .solve(&fit.transpose());
        for (t, &node) in ids.iter().enumerate() {
            for (j, axis) in world.into_iter().enumerate() {
                let omega = DVec3::new(
                    inverse[(3, 3 * t + j)],
                    inverse[(4, 3 * t + j)],
                    inverse[(5, 3 * t + j)],
                );
                let response = weights[t] * axis + omega.cross(-0.012 * N);
                for q in 0..3 {
                    out[(3 * s + q, 3 * node + j)] = AXIS[q].dot(response);
                }
            }
        }
    }
    out
}

fn matrix(g: &Gripper, h: &DMatrix<f64>) -> DMatrix<f64> {
    let native = independent::cartesian(
        g.states(),
        CENTER,
        false,
        [[0.; 3], (0.1 * N).to_array(), (0.1 * N).to_array()],
    );
    let mut m = DMatrix::zeros(12, 12);
    for (i, p) in sample(g).particles().iter().enumerate() {
        for x in 0..3 {
            m[(3 * i + x, 3 * i + x)] = p.inverse_mass;
        }
    }
    let mut k = h * &m * h.transpose();
    for t in 0..2 {
        for s in 0..2 {
            for (a, n) in AXIS.into_iter().enumerate() {
                for (b, d) in AXIS.into_iter().enumerate() {
                    for i in 0..3 {
                        for j in 0..3 {
                            k[(3 * t + a, 3 * s + b)] +=
                                n[i] * native[(3 + 3 * t + i, 3 + 3 * s + j)] * d[j];
                        }
                    }
                }
            }
        }
    }
    k
}
/// Enumerate the four two-finger stick/slide modes. Normal restitution and
/// sticking are linear at fixed sliding azimuths. Each remaining scalar
/// equation aligns the final tangent with its trial sliding direction.
fn cone_oracle(k: &DMatrix<f64>, initial: &DVector<f64>, mu: f64) -> DVector<f64> {
    for mask in 0..4 {
        let slides = (0..2).filter(|i| mask & (1 << i) != 0).collect::<Vec<_>>();
        let sticks = (0..2).filter(|i| mask & (1 << i) == 0).collect::<Vec<_>>();
        let evaluate = |theta: &DVector<f64>| {
            let mut p = DMatrix::zeros(6, 2 + 2 * sticks.len());
            let mut rows = vec![0, 3];
            for i in 0..2 {
                p[(3 * i, i)] = 1.;
            }
            for (j, &i) in slides.iter().enumerate() {
                p[(3 * i + 1, i)] = -mu * theta[j].cos();
                p[(3 * i + 2, i)] = -mu * theta[j].sin();
            }
            for (j, &i) in sticks.iter().enumerate() {
                for a in 0..2 {
                    p[(3 * i + 1 + a, 2 + 2 * j + a)] = 1.;
                    rows.push(3 * i + 1 + a);
                }
            }
            let km = k * &p;
            let a = DMatrix::from_fn(rows.len(), rows.len(), |r, c| km[(rows[r], c)]);
            let b = DVector::from_iterator(
                rows.len(),
                rows.iter()
                    .map(|&r| -initial[r] * if r % 3 == 0 { 1.55 } else { 1. }),
            );
            let j = p * a.lu().solve(&b).unwrap();
            let v = initial + k * &j;
            let residual = DVector::from_fn(slides.len(), |q, _| {
                let i = slides[q];
                -theta[q].sin() * v[3 * i + 1] + theta[q].cos() * v[3 * i + 2]
            });
            (j, v, residual)
        };
        let mut theta = DVector::from_element(slides.len(), 0.12_f64.atan2(0.65));
        for _ in 0..24 {
            let (j, v, r) = evaluate(&theta);
            if r.norm() < 4e-14 {
                if (0..2).all(|i| j[3 * i] > 0.)
                    && sticks
                        .iter()
                        .all(|&i| j[3 * i + 1].hypot(j[3 * i + 2]) <= mu * j[3 * i] + 1e-12)
                    && slides.iter().enumerate().all(|(q, &i)| {
                        theta[q].cos() * v[3 * i + 1] + theta[q].sin() * v[3 * i + 2] >= -1e-12
                    })
                {
                    return j;
                }
                break;
            }
            let derivative = DMatrix::from_fn(slides.len(), slides.len(), |i, b| {
                let mut trial = theta.clone();
                trial[b] += 1e-5;
                let plus = evaluate(&trial).2;
                trial[b] -= 2e-5;
                let minus = evaluate(&trial).2;
                (plus[i] - minus[i]) / 2e-5
            });
            let Some(change) = derivative.lu().solve(&(-&r)) else {
                break;
            };
            let Some(next) = (0..14)
                .map(|a| &theta + 0.5f64.powi(a) * &change)
                .find(|t| evaluate(t).2.norm() < r.norm())
            else {
                break;
            };
            theta = next;
        }
    }
    panic!("no independently qualified Coulomb mode for {mu}");
}
#[test]
fn real_articulated_fingertips_stick_and_slide_at_the_shared_cloth_skin() {
    for mu in [0.04, 0.09, 0.15, 0.5, 0.55, 0.57, 0.572, 0.7, 0.9, 1.4] {
        let mut g =
            Gripper::new(&WORLD_F.replace("friction=\"0.9\"", &format!("friction=\"{mu}\"")))
                .unwrap();
        let mut clear = Gripper::new(&WORLD_F.replace("100000000", "101000000")).unwrap();
        clear.sim.step_exact(100).unwrap();
        g.sim
            .step_exact(100)
            .unwrap_or_else(|e| panic!("{e}: {:?}", g.sim.fault()));
        let h = soft_jacobian(&clear);
        let k = matrix(&clear, &h);
        let v0 = DVector::from_iterator(6, (0..6).map(|i| AXIS[i % 3].dot(VELOCITY)));
        let mut rhs = -&v0;
        rhs[0] *= 1.55;
        rhs[3] *= 1.55;
        let stick = k.clone().cholesky().unwrap().solve(&rhs);
        let crit = (0..2)
            .map(|i| stick[3 * i + 1].hypot(stick[3 * i + 2]) / stick[3 * i])
            .fold(0f64, f64::max);
        let independent = cone_oracle(&k, &v0, mu);
        let r = sample(&g).latest_articulated_impact().unwrap();
        let mut js = DVector::zeros(6);
        let mut slip = [0.; 2];
        let mut old_moment = DVec3::ZERO;
        let mut measured_nodal = DVector::zeros(12);
        for (i, load) in r.material.loads.iter().enumerate() {
            let j = -DVec3::from(load.on_rigid);
            for a in 0..3 {
                js[3 * i + a] = AXIS[a].dot(j);
            }
            let f = load.friction.unwrap();
            let tangent = DVec3::from(f.on_material);
            let speed = DVec3::from(f.final_slip);
            slip[i] = speed.length();
            assert!(tangent.length() <= mu * js[3 * i] + 2e-12 && speed.dot(tangent) < 1e-12);
            assert!(speed.cross(tangent).length() < 2e-12);
            assert!(speed.length() < 2e-11 || (tangent.length() - mu * js[3 * i]).abs() < 2e-11);
            assert!((load.final_normal_velocity - 0.44).abs() < 2e-11);
            old_moment += (0.012 * N).cross(j);
            for &(v, p) in &load.on_particles {
                for a in 0..3 {
                    measured_nodal[3 * v + a] += p[a];
                }
            }
        }
        let post = &v0 + &k * &js;
        let nodal = h.transpose() * &js;
        let err = (&independent - &js).amax();
        assert!(err < 2e-11 && (&nodal - &measured_nodal).amax() < 2e-12);
        for i in 0..2 {
            assert!((post[3 * i] - 0.44).abs() < 2e-11);
            assert!((post[3 * i + 1].hypot(post[3 * i + 2]) - slip[i]).abs() < 2e-11);
        }
        let mut dp = DVec3::ZERO;
        let mut dl = DVec3::ZERO;
        let mut de = 0.;
        for (p, initial) in sample(&g)
            .particles()
            .iter()
            .zip(sample(&clear).particles())
        {
            let v = DVec3::from(p.velocity);
            let initial = DVec3::from(initial.velocity);
            let delta = (v - initial) / p.inverse_mass;
            dp += delta;
            dl += DVec3::from(p.position).cross(delta);
            de += 0.5 * (v.length_squared() - initial.length_squared()) / p.inverse_mass;
        }
        let a = budgets(clear.states());
        let b = budgets(g.states());
        let dp = (dp + b.0 - a.0).length();
        let dl = (dl + b.1 - a.1).length();
        let de = de + b.2 - a.2 + r.material.dissipated_energy;
        let normal_shift = (js[0] - 0.18790037649016356)
            .abs()
            .max((js[3] - 0.15505433860989226).abs());
        println!(
            "mu {mu:.3}, both-stick threshold {crit:.6}, final slips {slip:?} m/s, impulses {:?} N s, maximum Cartesian cone-solution error {err:e} N s, normal shift {normal_shift:e} N s, P {dp:e} N s, L {dl:e} N m s, E {de:e} J, barycentric-only spurious moment {:e} N m s",
            js.as_slice(),
            old_moment.length()
        );
        assert!(
            dp < 2e-11
                && dl < 2e-11
                && de.abs() < 2e-11
                && normal_shift > 1e-5
                && old_moment.length() > 1e-5
        );
    }
}
